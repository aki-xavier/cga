//! mlx-c `fast.metal_kernel` 的安全封装。
//!
//! 背景：cga 的渲染器是 MLX 数组式 wavefront 管线，但官方 `mlx-rs` 的
//! `fast` 模块只绑定了 rope/attention 等算子，没有绑定自定义 Metal kernel。
//! 底层 mlx-c 的 `mlx_fast_metal_kernel_*` C API 是齐的（本 crate 直接依赖
//! 与 mlx-rs 相同的 `mlx-sys`，由 Cargo 归一为同一构建），这里补齐一层
//! 最小安全封装。用途：BVH 求交内核（每线程一光线 + 短栈遍历）——逐光线的
//! 控制流用数组式 MLX 算子表达不了。
//!
//! 安全边界：全部 unsafe 收敛在本文件；MSL 源码由调用方给文本，JIT 编译
//! 失败（apply 返回非零）→ Err（错误文本来自 mlx-c 的错误处理器）。
//! 输入/输出数组的形状与 dtype 匹配由调用方与 kernel 源码共同保证。

#![allow(unsafe_code)]

use std::ffi::CString;
use std::sync::Mutex;

use mlx_rs::Array;

/// mlx-c 最近一次错误（错误处理器回调捕获；进程级单例，调试用途）。
static LAST_ERROR: Mutex<String> = Mutex::new(String::new());

fn install_error_handler() {
    use std::sync::Once;
    static ONCE: Once = Once::new();
    ONCE.call_once(|| unsafe {
        unsafe extern "C" fn handler(msg: *const std::os::raw::c_char, _data: *mut std::os::raw::c_void) {
            if !msg.is_null() {
                let s = std::ffi::CStr::from_ptr(msg).to_string_lossy().into_owned();
                if let Ok(mut g) = LAST_ERROR.lock() {
                    *g = s;
                }
            }
        }
        mlx_sys::mlx_set_error_handler(Some(handler), std::ptr::null_mut(), None);
    });
}

fn take_last_error() -> String {
    LAST_ERROR
        .lock()
        .map(|mut g| std::mem::take(&mut *g))
        .unwrap_or_default()
}

/// 读取并清空最近一次 mlx-c 错误文本（调试 kernel 编译失败用）。
pub fn take_last_error_string() -> String {
    install_error_handler();
    take_last_error()
}

/// 一个编译好的自定义 Metal kernel。mlx-c 侧按 `name` 缓存 JIT 产物；
/// 本句柄 Drop 时释放 C 侧引用（编译缓存仍由 mlx-c 持有，重复编译很便宜）。
pub struct MetalKernel {
    raw: mlx_sys::mlx_fast_metal_kernel,
}

fn vector_string(words: &[CString]) -> mlx_sys::mlx_vector_string {
    unsafe {
        let v = mlx_sys::mlx_vector_string_new();
        for w in words {
            mlx_sys::mlx_vector_string_append_value(v, w.as_ptr());
        }
        v
    }
}

impl MetalKernel {
    /// 注册一个 kernel：`source` 是 kernel 函数体，`header` 是函数体外的
    /// MSL 声明区（共享的 inline 函数放这里——函数体里不允许定义函数）。
    /// 注意：MSL 语法错误只在 apply 时爆（JIT 惰性），所以调用方应在
    /// 注册后立刻 apply 一次小规模输入做自检。
    pub fn compile(
        name: &str,
        inputs: &[&str],
        outputs: &[&str],
        source: &str,
        header: &str,
    ) -> Result<Self, String> {
        install_error_handler();
        let mk_cstr = |s: &str| {
            CString::new(s).map_err(|_| format!("metal kernel: interior NUL in {s:?}"))
        };
        let name_c = mk_cstr(name)?;
        let src_c = mk_cstr(source)?;
        let header_c = mk_cstr(header)?;
        let ins: Vec<CString> = inputs.iter().map(|s| mk_cstr(s)).collect::<Result<_, _>>()?;
        let outs: Vec<CString> = outputs.iter().map(|s| mk_cstr(s)).collect::<Result<_, _>>()?;
        let vin = vector_string(&ins);
        let vout = vector_string(&outs);
        let raw = unsafe {
            mlx_sys::mlx_fast_metal_kernel_new(
                name_c.as_ptr(),
                vin,
                vout,
                src_c.as_ptr(),
                header_c.as_ptr(),
                true, // ensure_row_contiguous：kernel 内按行主序直接索引
                false,
            )
        };
        unsafe {
            mlx_sys::mlx_vector_string_free(vin);
            mlx_sys::mlx_vector_string_free(vout);
        }
        if raw.ctx.is_null() {
            let e = take_last_error();
            return Err(format!("metal kernel {name:?}: create failed: {e}"));
        }
        Ok(Self { raw })
    }

    /// 以 `grid = (n_threads, 1, 1)` 发射；`out_specs` = (dtype, shape)。
    /// 返回的输出数组按声明顺序排列（MLX 惰性求值，与调用方管线一致）。
    pub fn apply(
        &self,
        inputs: &[&Array],
        out_specs: &[(mlx_sys::mlx_dtype, &[i32])],
        n_threads: i32,
    ) -> Result<Vec<Array>, String> {
        // 注意（rustc 1.99.0 release）：`if n_threads < 256 { n_threads.max(1) }
        // else { 256 }` 在本 crate 的 release 构建中被误编译成 tg=n_threads
        //（debug 正确；min() 形式正确）。已用真实二进制反汇编确认差异，未最小化。
        let tg: i32 = 256.min(n_threads.max(1));
        self.apply_tg(inputs, out_specs, n_threads, tg)
    }

    /// 同 [`Self::apply`]，但显式给 threadgroup 尺寸。
    pub fn apply_tg(
        &self,
        inputs: &[&Array],
        out_specs: &[(mlx_sys::mlx_dtype, &[i32])],
        n_threads: i32,
        tg: i32,
    ) -> Result<Vec<Array>, String> {
        install_error_handler();
        unsafe {
            let vin = mlx_sys::mlx_vector_array_new();
            for a in inputs {
                mlx_sys::mlx_vector_array_append_value(vin, a.as_ptr());
            }
            let cfg = mlx_sys::mlx_fast_metal_kernel_config_new();
            for (dtype, shape) in out_specs {
                if mlx_sys::mlx_fast_metal_kernel_config_add_output_arg(
                    cfg,
                    shape.as_ptr(),
                    shape.len(),
                    *dtype,
                ) != 0
                {
                    mlx_sys::mlx_vector_array_free(vin);
                    mlx_sys::mlx_fast_metal_kernel_config_free(cfg);
                    return Err(format!("metal kernel: bad output spec {shape:?}"));
                }
            }
            mlx_sys::mlx_fast_metal_kernel_config_set_grid(cfg, n_threads, 1, 1);
            mlx_sys::mlx_fast_metal_kernel_config_set_thread_group(cfg, tg, 1, 1);
            // CGA_METAL_VERBOSE=1：打印 mlx-c 生成的完整 MSL（签名+body），调试编译错误用
            if std::env::var("CGA_METAL_VERBOSE").is_ok() {
                mlx_sys::mlx_fast_metal_kernel_config_set_verbose(cfg, true);
            }
            let stream = mlx_sys::mlx_default_gpu_stream_new();
            let mut out = mlx_sys::mlx_vector_array_new();
            let rc = mlx_sys::mlx_fast_metal_kernel_apply(&mut out, self.raw, vin, cfg, stream);
            let mut arrays = Vec::with_capacity(out_specs.len());
            let mut err = None;
            if rc == 0 {
                for i in 0..out_specs.len() {
                    let mut h = mlx_sys::mlx_array_new();
                    // vector_array_get 经 mlx_array_set_ 拷贝（retain）句柄，
                    // from_ptr 接管所有权；之后释放 vector 自身安全。
                    if mlx_sys::mlx_vector_array_get(&mut h, out, i) != 0 {
                        err = Some(format!("metal kernel: missing output {i}"));
                        break;
                    }
                    arrays.push(Array::from_ptr(h));
                }
            } else {
                let e = take_last_error();
                err = Some(format!("metal kernel apply failed: {e}"));
            }
            mlx_sys::mlx_stream_free(stream);
            mlx_sys::mlx_vector_array_free(out);
            mlx_sys::mlx_vector_array_free(vin);
            mlx_sys::mlx_fast_metal_kernel_config_free(cfg);
            match err {
                Some(e) => Err(e),
                None => Ok(arrays),
            }
        }
    }
}

impl Drop for MetalKernel {
    fn drop(&mut self) {
        unsafe {
            mlx_sys::mlx_fast_metal_kernel_free(self.raw);
        }
    }
}

// mlx_fast_metal_kernel 是 C 侧共享指针包装；编译产物在 mlx-c 注册表内，
// 句柄本身只读共享 → 可安全跨线程引用。
unsafe impl Send for MetalKernel {}
unsafe impl Sync for MetalKernel {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smoke_custom_kernel_roundtrip() {
        let k = MetalKernel::compile(
            "cga_smoke_double",
            &["inp"],
            &["out"],
            "uint elem = thread_position_in_grid.x;\n  out[elem] = inp[elem] * 2.0f;",
            "",
        )
        .expect("compile");
        let inp = Array::from_slice(&[1.0f32, 2.0, 3.5, -4.0], &[4]);
        let outs = k
            .apply(&[&inp], &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[4])], 4)
            .expect("apply");
        outs[0].eval().unwrap();
        assert_eq!(outs[0].as_slice::<f32>(), &[2.0, 4.0, 7.0, -8.0]);
    }

    #[test]
    fn bad_msl_reports_error() {
        // MSL 语法错误：mlx-c 在 new 或 apply 之一处拒绝（JIT 时机是其内部
        // 实现细节），两处都必须把失败变成 Err 而不是崩溃/静默成功。
        let bad = MetalKernel::compile("cga_smoke_bad", &["inp"], &["out"], "this is not msl", "");
        let inp = Array::from_slice(&[1.0f32], &[1]);
        let r = bad.and_then(|k| {
            k.apply(&[&inp], &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[1])], 1)
                .and_then(|outs| outs[0].eval().map_err(|e| format!("eval: {e}")))
        });
        assert!(r.is_err(), "坏 MSL 必须报错");
    }
}

#[cfg(test)]
mod grid_tests {
    use super::*;

    #[test]
    fn large_grid_small_threadgroup() {
        let k = MetalKernel::compile(
            "cga_smoke_grid",
            &["inp"],
            &["out"],
            "uint elem = thread_position_in_grid.x;\n  if (elem < inp_shape[0]) out[elem] = inp[elem];",
            "",
        )
        .expect("compile");
        let v = vec![1.0f32; 6912];
        let inp = Array::from_slice(&v, &[6912]);
        let outs = k
            .apply(&[&inp], &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[6912])], 6912)
            .expect("apply grid=6912 tg=256");
        outs[0].eval().unwrap();
        assert_eq!(outs[0].as_slice::<f32>()[6911], 1.0);
    }
}

#[cfg(test)]
mod big_grid_tests {
    use super::*;


    #[test]
    fn probe_dispatch_dims() {
        let k = MetalKernel::compile(
            "cga_smoke_dims",
            &["inp"],
            &["out"],
            "uint elem = thread_position_in_grid.x;\n  if (elem == 0) { out[0] = (float)threads_per_grid.x; out[1] = (float)threads_per_threadgroup.x; }",
            "",
        )
        .expect("compile");
        let v = vec![1.0f32; 6912];
        let inp = Array::from_slice(&v, &[6912]);
        let outs = k
            .apply_tg(&[&inp], &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[6912])], 6912, 256)
            .expect("apply");
        outs[0].eval().unwrap();
        let s = outs[0].as_slice::<f32>();
        println!("DIMS grid={} tg={}", s[0], s[1]);
    }

    #[test]
    fn apply_twice_same_kernel() {
        let k = MetalKernel::compile(
            "cga_smoke_twice",
            &["inp"],
            &["out"],
            "uint elem = thread_position_in_grid.x;\n  if (elem < inp_shape[0]) out[elem] = inp[elem] * 2.0f;",
            "",
        )
        .expect("compile");
        let v = vec![1.0f32; 6912];
        let inp = Array::from_slice(&v, &[6912]);
        for round in 0..2 {
            let outs = k
                .apply(&[&inp], &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[6912])], 6912)
                .expect("apply");
            if let Err(e) = outs[0].eval() {
                panic!("round {round}: {e:?}");
            }
        }
    }



    #[test]
    fn grid_307k() {
        let k = MetalKernel::compile(
            "cga_smoke_grid_big",
            &["inp"],
            &["out"],
            "uint elem = thread_position_in_grid.x;\n  if (elem < inp_shape[0]) out[elem] = inp[elem];",
            "",
        )
        .expect("compile");
        let v = vec![1.0f32; 307200];
        let inp = Array::from_slice(&v, &[307200]);
        let outs = k
            .apply(&[&inp], &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[307200])], 307200)
            .expect("apply");
        if let Err(e) = outs[0].eval() {
            panic!("eval: {e:?} LAST={}", take_last_error_string());
        }
        assert_eq!(outs[0].as_slice::<f32>()[307199], 1.0);
    }
}
pub use mlx_sys;
