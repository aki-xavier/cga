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

use mlx_rs::Array;

// mlx-c 最近一次错误（错误处理器回调捕获，调试用途）。
//
// **线程局部**，不是进程级单例：原实现是进程级 `Mutex<String>`，两个线程同时
// 让 kernel 编译失败会互相覆盖槽位，`compile` 可能读到另一个 kernel 的报错。
// 改成线程局部后各线程只取自己那份。
//
// 但要说清实测边界，别夸大（2026-10-09 核实）：在 MLX 0.32 上，坏 MSL 的
// 编译失败**根本不走这个槽**——`compile`/`apply` 都返回 Ok，错误推迟到
// `Array::eval()` 由 MLX 自己的 Result 带回来（`bad_msl_reports_error` 里钉的
// 就是那条文本）。也就是说：
//   - 现在的错误契约由 eval 的 Result 承担，天然逐调用、无共享状态；
//   - 本槽只在 mlx-c 走 `mlx_fast_metal_kernel_new` 失败并回调处理器的路径上
//     才有内容，而这条路在当前 MLX 版本上观测不到；
//   - 因此**没有**能演示旧竞态的测试——别假装有。线程局部是防御性收敛
//     （去掉共享可变状态），不是已证实的 bug 修复。
thread_local! {
    static LAST_ERROR: std::cell::RefCell<String> = const { std::cell::RefCell::new(String::new()) };
}

fn install_error_handler() {
    use std::sync::Once;
    static ONCE: Once = Once::new();
    ONCE.call_once(|| unsafe {
        unsafe extern "C" fn handler(
            msg: *const std::os::raw::c_char,
            _data: *mut std::os::raw::c_void,
        ) {
            if !msg.is_null() {
                let s = std::ffi::CStr::from_ptr(msg).to_string_lossy().into_owned();
                LAST_ERROR.with(|g| *g.borrow_mut() = s);
            }
        }
        mlx_sys::mlx_set_error_handler(Some(handler), std::ptr::null_mut(), None);
    });
}

fn take_last_error() -> String {
    LAST_ERROR.with(|g| std::mem::take(&mut *g.borrow_mut()))
}

/// GPU（Metal device）可用性探测。返回 `Err(原因)` 时原因可直接展示给用户。
///
/// 用 mlx 官方的 `mlx_metal_is_available`，不自己猜：Intel mac / Metal 被禁用
/// / 虚拟机无 GPU 都会落到这里。
pub fn gpu_available() -> Result<(), String> {
    let mut ok = false;
    let rc = unsafe { mlx_sys::mlx_metal_is_available(&mut ok) };
    if rc != 0 {
        return Err(format!("mlx_metal_is_available 调用失败 (rc={rc})"));
    }
    if ok {
        Ok(())
    } else {
        Err("mlx 报告无可用 Metal device（mlx_metal_is_available = false）".to_string())
    }
}

/// 无 GPU 时的测试闸门（2026-10-09 定的策略：**跳过并报明确原因**）。
///
/// - 有 GPU → 返回 `true`，测试照常跑。
/// - 无 GPU → 打印 `SKIP <测试名>: <原因>` 后返回 `false`，测试**立即返回**
///   （算通过，不算失败——本 crate 的测试全部需要真 GPU）。
/// - 设 `CGA_REQUIRE_GPU=1`（CI 用）→ 改为 `panic!`，让缺 GPU 在 CI 上变响。
///
/// 为什么两种模式都要：本地无 GPU 是正常环境，静默跳过是对的；但 CI 上
/// "因为没 GPU 所以什么都没测" 必须失败，否则是假绿。默认跳过 + 环境变量
/// 变严格，两边各取所需。
pub fn require_gpu(test: &str) -> bool {
    let strict = std::env::var("CGA_REQUIRE_GPU")
        .map(|v| v == "1" || v.eq_ignore_ascii_case("true"))
        .unwrap_or(false);
    match gate(test, gpu_available(), strict) {
        Gate::Run => true,
        Gate::Skip(msg) => {
            eprintln!("{msg}");
            false
        }
        Gate::Fail(msg) => panic!("{msg}"),
    }
}

/// 闸门判定结果（与机器状态解耦，便于在**任何**机器上验证策略本身）。
#[derive(Debug, PartialEq, Eq)]
pub enum Gate {
    /// 有 GPU，照常跑。
    Run,
    /// 无 GPU 且非严格模式：打印原因后跳过。
    Skip(String),
    /// 无 GPU 且严格模式（`CGA_REQUIRE_GPU=1`）：缺 GPU 视为失败。
    Fail(String),
}

/// 闸门策略的纯逻辑（无 I/O、无环境读取），见 [`require_gpu`]。
pub fn gate(test: &str, avail: Result<(), String>, strict: bool) -> Gate {
    match avail {
        Ok(()) => Gate::Run,
        Err(why) if strict => Gate::Fail(format!(
            "{test}: 需要 GPU —— {why}（CGA_REQUIRE_GPU=1 时缺 GPU 视为失败）"
        )),
        Err(why) => Gate::Skip(format!(
            "SKIP {test}: {why}（设 CGA_REQUIRE_GPU=1 可让缺 GPU 变成失败）"
        )),
    }
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
        let mk_cstr =
            |s: &str| CString::new(s).map_err(|_| format!("metal kernel: interior NUL in {s:?}"));
        let name_c = mk_cstr(name)?;
        let src_c = mk_cstr(source)?;
        let header_c = mk_cstr(header)?;
        let ins: Vec<CString> = inputs
            .iter()
            .map(|s| mk_cstr(s))
            .collect::<Result<_, _>>()?;
        let outs: Vec<CString> = outputs
            .iter()
            .map(|s| mk_cstr(s))
            .collect::<Result<_, _>>()?;
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
    use super::require_gpu;
    use super::*;

    #[test]
    fn smoke_custom_kernel_roundtrip() {
        if !require_gpu("smoke_custom_kernel_roundtrip") {
            return;
        }
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
        if !require_gpu("bad_msl_reports_error") {
            return;
        }
        // MSL 语法错误：mlx-c 在 new 或 apply 之一处拒绝（JIT 时机是其内部
        // 实现细节），两处都必须把失败变成 Err 而不是崩溃/静默成功。
        const BAD_SRC: &str = "this is not msl";
        let bad = MetalKernel::compile("cga_smoke_bad", &["inp"], &["out"], BAD_SRC, "");
        let inp = Array::from_slice(&[1.0f32], &[1]);
        let r = bad.and_then(|k| {
            k.apply(&[&inp], &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[1])], 1)
                .and_then(|outs| outs[0].eval().map_err(|e| format!("eval: {e}")))
        });
        let err = r.expect_err("坏 MSL 必须报错");
        // 钉错误文本（原先只 `is_err()`，任何失败都能过）。实测 MLX 0.32 的
        // 编译失败**不走** mlx-c 错误处理器，而是推迟到 `eval()` 由 MLX 自己
        // 的 Result 带回来（见 LAST_ERROR 处的说明），所以错误契约钉在这里。
        assert!(
            err.contains("Unable to build metal library"),
            "应报 JIT 构建失败: {err}"
        );
        assert!(err.contains(BAD_SRC), "错误里应回显注入的坏源码: {err}");
    }
}

#[cfg(test)]
mod grid_tests {
    use super::require_gpu;
    use super::*;

    #[test]
    fn large_grid_small_threadgroup() {
        if !require_gpu("large_grid_small_threadgroup") {
            return;
        }
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
        if !require_gpu("probe_dispatch_dims") {
            return;
        }
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
            .apply_tg(
                &[&inp],
                &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[6912])],
                6912,
                256,
            )
            .expect("apply");
        outs[0].eval().unwrap();
        let s = outs[0].as_slice::<f32>();
        println!("DIMS grid={} tg={}", s[0], s[1]);
        // kernel 写回的是派发参数本身：grid=6912、threadgroup=256（确定性）。
        assert_eq!(s[0], 6912.0, "grid 维数");
        assert_eq!(s[1], 256.0, "threadgroup 维数");
    }

    #[test]
    fn apply_twice_same_kernel() {
        if !require_gpu("apply_twice_same_kernel") {
            return;
        }
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
        if !require_gpu("grid_307k") {
            return;
        }
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
            .apply(
                &[&inp],
                &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[307200])],
                307200,
            )
            .expect("apply");
        if let Err(e) = outs[0].eval() {
            panic!("eval: {e:?} LAST={}", take_last_error_string());
        }
        assert_eq!(outs[0].as_slice::<f32>()[307199], 1.0);
    }

    /// 闸门策略的闭式测试（不依赖本机有没有 GPU，所以每台机器都能跑）。
    ///
    /// 2026-10-09 定的策略：无 GPU 时 `cargo test` **跳过并报明确原因**；设
    /// `CGA_REQUIRE_GPU=1` 时（CI 用）改为失败，避免"因为没 GPU 所以什么都没测"
    /// 变成假绿。
    #[test]
    fn gpu_gate_policy() {
        const WHY: &str = "mlx 报告无可用 Metal device（mlx_metal_is_available = false）";
        // 有 GPU → 跑，两种模式都一样。
        assert_eq!(gate("t", Ok(()), false), Gate::Run);
        assert_eq!(gate("t", Ok(()), true), Gate::Run);
        // 无 GPU + 默认 → 跳过，且消息里带测试名与**具体原因**（不是"跳过"两个字）。
        match gate("smoke_custom_kernel_roundtrip", Err(WHY.to_string()), false) {
            Gate::Skip(msg) => {
                assert!(msg.contains("smoke_custom_kernel_roundtrip"), "{msg}");
                assert!(msg.contains(WHY), "{msg}");
                assert!(msg.contains("CGA_REQUIRE_GPU=1"), "应说明如何变严格: {msg}");
            }
            other => panic!("应跳过，实际 {other:?}"),
        }
        // 无 GPU + 严格 → 失败，消息同样带原因。
        match gate("t", Err(WHY.to_string()), true) {
            Gate::Fail(msg) => {
                assert!(msg.contains(WHY), "{msg}");
                assert!(msg.contains("CGA_REQUIRE_GPU=1"), "{msg}");
            }
            other => panic!("应失败，实际 {other:?}"),
        }
        // 探测本身的错误（mlx 调用返回非零）也走同一分支，不被当成有 GPU。
        let rc_err = "mlx_metal_is_available 调用失败 (rc=-1)".to_string();
        assert!(matches!(
            gate("t", Err(rc_err.clone()), false),
            Gate::Skip(m) if m.contains("rc=-1")
        ));
    }
}
pub use mlx_sys;
