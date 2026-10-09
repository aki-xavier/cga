//! 灯光/材质的数据结构已抽至 `cga-scene`（docs/module-review.md 问题 1）——
//! 它们同时是场景模型的词汇。本模块保留 GPU 侧的活：批量光照
//! （`light_direction_at`/`light_far`，原 `Light` 的 MLX 方法，现在是自由
//! 函数，因为类型定义搬到了别的 crate）与 `shade_batched`。

use crate::mlxops::*;

// 路径兼容层（`cga_gpu::shading::{Light, Material, ...}` 旧路径不变）。
pub use cga_scene::{Light, LightKind, Material, MaterialKind, MaterialParams};

/// 原 `Light::direction_at`：方向/点光的单位方向与衰减（MLX 批量）。
pub fn light_direction_at(light: &Light, p: &mlx_rs::Array) -> (mlx_rs::Array, mlx_rs::Array) {
    match light.kind {
        LightKind::Directional => {
            let ld = ck(ops::broadcast_to(arr3v(light.direction), p.shape()));
            (ld, fs(light.intensity))
        }
        LightKind::Point => {
            let lv = ck(ops::broadcast_to(arr3v(light.position), p.shape())).subtract(p);
            let lv = ck(lv);
            let dist2 = ck(ck(lv.multiply(&lv)).sum_axes(&[-1], true));
            let ld = ck(lv.divide(ck(dist2.sqrt())));
            let atten = s_rdiv(&s_add(&s_div(&dist2, 8.0), 1.0), light.intensity);
            (ld, atten)
        }
        LightKind::Ambient => {
            panic!("ambient light is not part of the per-light loop")
        }
    }
}

/// 原 `Light::far`：点光到采样点的距离（方向光 = INF）。
pub fn light_far(light: &Light, p: &mlx_rs::Array) -> mlx_rs::Array {
    if light.kind == LightKind::Point {
        let lv = ck(ops::broadcast_to(arr3v(light.position), p.shape())).subtract(p);
        let lv = ck(lv);
        return ck(ck(ck(lv.multiply(&lv)).sum_axes(&[-1], false)).sqrt());
    }
    fs(f64::INFINITY)
}

#[allow(clippy::too_many_arguments)]
pub fn shade_batched(
    emissive: &Array,
    diff: &Array,
    spec: &Array,
    expo: &Array,
    p: &Array,
    n: &Array,
    d: &Array,
    lights: &[Light],
    ambient: Option<Light>,
    vis: &[Array],
) -> Array {
    let v = ck(d.negative());
    let mut out = emissive.clone();
    if let Some(amb) = ambient {
        let ambc = s_mul(&arr3v(amb.color.rgb()), amb.intensity);
        out = ck(out.add(ck(ck(ops::broadcast_to(&ambc, p.shape())).multiply(diff))));
    }
    let ndv = s_max(&ck(ck(n.multiply(&v)).sum_axes(&[-1], true)), 0.0);
    for (i, light) in lights.iter().enumerate() {
        let lc = arr3v(light.color.rgb());
        let (ld, atten) = light_direction_at(light, p);
        let nl = s_max(&ck(ck(n.multiply(&ld)).sum_axes(&[-1], true)), 0.0);
        let mut h = ck(ld.add(&v));
        let hn = ck(ck(ck(h.multiply(&h)).sum_axes(&[-1], true)).sqrt());
        h = ck(h.divide(s_max(&hn, 1e-12)));
        let spec_t = ck(s_max(&ck(ck(n.multiply(&h)).sum_axes(&[-1], true)), 0.0).power(expo));
        let mut contrib = ck(ck(lc.multiply(&atten)).multiply(ck(
            ck(diff.multiply(&nl)).add(ck(ck(spec.multiply(&spec_t)).multiply(&ndv)))
        )));
        if !vis.is_empty() {
            contrib = ck(contrib.multiply(ck(vis[i].expand_dims(1))));
        }
        out = ck(out.add(&contrib));
    }
    out
}

use mlx_rs::{ops, Array};
