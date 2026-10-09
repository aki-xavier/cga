//! BVH 求交内核：CPU 建扁平 BVH（确定性中位切分），GPU 上由单个自定义
//! Metal kernel（每线程一光线 + 64 深短栈遍历）完成"光线包 → 最近命中"
//! 与"光线包 → 阴影可见度"两个阶段，替代逐对象全数组循环。
//!
//! 动机（成本模型）：旧路径成本 ≈ 对象数 × 光线数 × 每对象 kernel 启动开销；
//! 本路径 = 每光线 log(n) 次 AABB 测试 + 只测真实候选图元，且单 kernel 发射。
//!
//! 覆盖图元（type）：0 球 / 1 盒 / 2 圆柱 / 3 圆锥 / 4 椭球——与
//! `geometry_ops.rs` / `geometry_extra.rs` 的数组内核逐行同构（f32，guard
//! 常数一致）。平面/圆片/torus/cyclide/CSG/仿射包装/带贴图的对象走旧路径，
//! 两侧结果按 t 取近合并。位级一致性：本路径只在对象数 ≥ `BVH_MIN_OBJECTS`
//! 时自动启用（画廊金标场景全在其下），`CGA_NO_BVH=1` 全关、
//! `CGA_BVH_FORCE=1` 强制启用（测试用）。
//!
//! f64 认证回退本来只覆盖 CSG 子节点（csg.rs），顶层图元无回退——本路径
//! 移植的恰是顶层求交，语义不变。

use std::sync::OnceLock;

use cga_core::{
    BoxParams, ConeParams, CylinderParams, EllipsoidParams, GeometryParams, SphereParams,
};
use cga_fastmetal::MetalKernel;
use mlx_rs::Array;

use crate::geometry_ops::geom_bounds;
use crate::renderer::RenderMode;
use crate::scene::Scene;

/// 自动启用的对象数下限（画廊金标场景全在此下 → 逐位不变）。
pub(crate) const BVH_MIN_OBJECTS: usize = 32;

/// 每个图元的 f32 参数槽数（摆得下圆锥的 14 浮点）。
const PARAM_SLOTS: usize = 32;

// ---------------------------------------------------------------------------
// MSL：图元求交 + BVH 遍历。与数组内核同构；guard 常数（1e-6/1e-9/1e-12）
// 逐一对应。
// ---------------------------------------------------------------------------

pub(crate) const MSL_HEADER: &str = r#"
// prim_params 行布局（32 浮点）：
//   type 0 球:   [c3, r]
//   type 1 盒:   [c3, axes0..2 各3, half3]
//   type 2 圆柱: [q3, u3, r, h]  (h<0 = 无限长)
//   type 3 圆锥: [a_inv3 行主序9, t_inv3, r, h]
//   type 4 椭球: [a_inv3 行主序9, t_inv3]

inline bool aabb_entry(float3 o, float3 invd, float3 lo, float3 hi, thread float& t_en) {
  float3 t1 = (lo - o) * invd;
  float3 t2 = (hi - o) * invd;
  float3 tmn3 = min(t1, t2);
  float3 tmx3 = max(t1, t2);
  t_en = max(max(tmn3.x, tmn3.y), tmn3.z);
  float t_ex = min(min(tmx3.x, tmx3.y), tmx3.z);
  return t_ex > 1e-6f && t_en < t_ex; // NaN（d 分量为 0 且起点贴面）→ false，与旧路径一致
}

inline bool hit_sphere(const thread float* pp, float3 o, float3 d, thread float& th, thread float3& nh) {
  float3 c = float3(pp[0], pp[1], pp[2]);
  float r = pp[3];
  float3 oc = o - c;
  float b = 2.0f * dot(oc, d);
  float cq = dot(oc, oc) - r * r;
  float disc = b * b - 4.0f * cq;
  bool valid = disc > 1e-12f;
  float sq = sqrt(max(disc, 0.0f));
  float t1 = (-b - sq) / 2.0f;
  float t2 = (-b + sq) / 2.0f;
  float t = (valid && t1 > 1e-6f) ? t1 : t2;
  bool mask = valid && t > 1e-6f;
  float3 n = (o + t * d - c) / r;
  if (mask && t1 <= 1e-6f) n = -n; // 内部命中翻法向
  th = t; nh = n;
  return mask;
}

inline bool hit_box(const thread float* pp, float3 o, float3 d, thread float& th, thread float3& nh) {
  float3 c = float3(pp[0], pp[1], pp[2]);
  float3 ax0 = float3(pp[3], pp[4], pp[5]);
  float3 ax1 = float3(pp[6], pp[7], pp[8]);
  float3 ax2 = float3(pp[9], pp[10], pp[11]);
  float3 half_ = float3(pp[12], pp[13], pp[14]);
  float3 oc = o - c;
  float op_[3] = {dot(oc, ax0), dot(oc, ax1), dot(oc, ax2)};
  float dp[3] = {dot(d, ax0), dot(d, ax1), dot(d, ax2)};
  float tmin[3]; float tmax[3];
  for (int i = 0; i < 3; i++) {
    float inv = 1.0f / dp[i];
    float ta = -inv * (op_[i] + half_[i]);
    float tb = -inv * (op_[i] - half_[i]);
    tmin[i] = min(ta, tb);
    tmax[i] = max(ta, tb);
  }
  // argmax/argmin 取首个（严格不等号），与 MLX 一致
  int i_entry = 0;
  for (int i = 1; i < 3; i++) { if (tmin[i] > tmin[i_entry]) i_entry = i; }
  int i_exit = 0;
  for (int i = 1; i < 3; i++) { if (tmax[i] < tmax[i_exit]) i_exit = i; }
  float t_entry = tmin[i_entry];
  float t_exit = tmax[i_exit];
  bool valid = t_entry < t_exit && t_exit > 1e-6f;
  bool inside = valid && t_entry <= 1e-6f;
  float t = (valid && !inside) ? t_entry : t_exit;
  float3 nl = float3(0.0f);
  if (inside) { nl[i_exit] = sign(dp[i_exit]); } else { nl[i_entry] = -sign(dp[i_entry]); }
  float3 n = nl.x * ax0 + nl.y * ax1 + nl.z * ax2; // vecmat(nl, axes)：out_j = Σ_i nl_i·axes_i[j]
  th = t; nh = valid ? n : float3(0.0f);
  return valid;
}

inline bool hit_cylinder(const thread float* pp, float3 o, float3 d, thread float& th, thread float3& nh) {
  float3 q = float3(pp[0], pp[1], pp[2]);
  float3 u = float3(pp[3], pp[4], pp[5]);
  float r = pp[6];
  float h = pp[7];
  float3 oc = o - q;
  float d_par = dot(d, u);
  float o_par = dot(oc, u);
  float3 d_p = d - d_par * u;
  float3 o_p = oc - o_par * u;
  float a = dot(d_p, d_p);
  float b = 2.0f * dot(o_p, d_p);
  float cq = dot(o_p, o_p) - r * r;
  float disc = b * b - 4.0f * a * cq;
  bool valid = a > 1e-12f && disc > 1e-12f;
  float sq = sqrt(max(disc, 0.0f));
  float t1 = (-b - sq) / (2.0f * a);
  float t2 = (-b + sq) / (2.0f * a);
  float t_s = (valid && t1 > 1e-6f) ? t1 : t2;
  bool mask_s = valid && t_s > 1e-6f;
  float3 hp = o_p + t_s * d_p;
  float3 n_s = hp / r;
  if (mask_s && t1 <= 1e-6f) n_s = -n_s;
  if (h < 0.0f) { // 无限长圆柱
    th = t_s; nh = mask_s ? n_s : float3(0.0f);
    return mask_s;
  }
  bool side_ok = mask_s && fabs(o_par + t_s * d_par) <= h;
  float best_t = side_ok ? t_s : INFINITY;
  float3 best_n = n_s;
  bool cap_en = fabs(d_par) > 1e-9f;
  float cap_t[2] = {(h - o_par) / d_par, (-h - o_par) / d_par};
  for (int i = 0; i < 2; i++) {
    float ct = cap_t[i];
    float3 pc = o + ct * d - q;
    float3 lat = pc - dot(pc, u) * u;
    bool ok = cap_en && ct > 1e-6f && dot(lat, lat) <= r * r;
    if (ok && ct < best_t) { best_t = ct; best_n = -sign(d_par) * u; } // 严格 <：侧面优先于盖、盖0 优先于盖1
  }
  bool fin = isfinite(best_t) && best_t > 1e-6f;
  th = fin ? best_t : t_s;
  nh = fin ? best_n : float3(0.0f);
  return fin;
}

// 圆锥/椭球共用：world→local（o_l = A·o + t；A = a_inv3 行主序）
inline void to_local(const thread float* pp, float3 o, float3 d, thread float3& o_l, thread float3& d_u, thread float& lam) {
  float3 r0 = float3(pp[0], pp[1], pp[2]);
  float3 r1 = float3(pp[3], pp[4], pp[5]);
  float3 r2 = float3(pp[6], pp[7], pp[8]);
  float3 t3 = float3(pp[9], pp[10], pp[11]);
  o_l = float3(dot(r0, o), dot(r1, o), dot(r2, o)) + t3;
  float3 d_l = float3(dot(r0, d), dot(r1, d), dot(r2, d));
  lam = length(d_l);
  lam = lam > 1e-12f ? lam : 1.0f;
  d_u = d_l / lam;
}

// 法向 local→world：Aᵀ·n_l 归一化
inline float3 from_local_n(const thread float* pp, float3 n_l) {
  float3 c0 = float3(pp[0], pp[3], pp[6]);
  float3 c1 = float3(pp[1], pp[4], pp[7]);
  float3 c2 = float3(pp[2], pp[5], pp[8]);
  float3 n = float3(dot(c0, n_l), dot(c1, n_l), dot(c2, n_l));
  float norm = length(n);
  return n / (norm > 1e-12f ? norm : 1.0f);
}

inline float3 cone_side_n(float h, float k2, float3 o_l, float3 d_u, float t) {
  float3 p = o_l + t * d_u;
  float3 g = float3(p.x, p.y, (p.z - h * 0.5f) * (1.0f - k2));
  float norm = length(g);
  return g / (norm > 1e-12f ? norm : 1.0f);
}

inline bool hit_cone(const thread float* pp, float3 o, float3 d, thread float& th, thread float3& nh) {
  float r = pp[12];
  float h = pp[13];
  float3 o_l; float3 d_u; float lam;
  to_local(pp, o, d, o_l, d_u, lam);
  float k = r / h;
  float k2 = 1.0f + k * k;
  float wz = o_l.z - h * 0.5f;
  float dz = d_u.z;
  float wd = dot(o_l, d_u) - dz * (h * 0.5f);
  float ww = dot(o_l, o_l) - o_l.z * h + h * h * 0.25f;
  float a = 1.0f - k2 * dz * dz;
  float b = 2.0f * (wd - k2 * wz * dz);
  float c = ww - k2 * wz * wz;
  bool side_ok = fabs(a) > 1e-12f;
  float a_s = side_ok ? a : 1e-12f;
  float disc = b * b - 4.0f * a_s * c;
  side_ok = side_ok && disc > 1e-12f;
  float sq = sqrt(max(disc, 0.0f));
  float r_lo = (-b - sq) / (2.0f * a_s);
  float r_hi = (-b + sq) / (2.0f * a_s);
  float st0 = min(r_lo, r_hi);
  float st1 = max(r_lo, r_hi);
  float safe_dz = fabs(dz) > 1e-9f ? dz : 1e-9f;
  float t_top = -wz / safe_dz;
  float t_bot = -(wz + h) / safe_dz;
  float at0 = min(t_top, t_bot);
  float at1 = max(t_top, t_bot);
  bool pos_a = a > 1e-12f;
  float c1e = pos_a ? max(st0, at0) : at0;
  float c1x = pos_a ? min(st1, at1) : min(st0, at1);
  bool v1 = side_ok && c1e < c1x;
  float c2e = max(st1, at0);
  bool v2 = side_ok && !pos_a && c2e < at1;
  float enter = v1 ? c1e : c2e;
  float exit_ = v1 ? c1x : at1;
  bool valid = v1 || v2;
  bool hit_enter = valid && enter > 1e-6f;
  bool hit_exit = valid && !hit_enter && exit_ > 1e-6f;
  float t_l = hit_enter ? enter : exit_;
  bool mask = hit_enter || hit_exit;
  float3 n_s0 = cone_side_n(h, k2, o_l, d_u, enter);
  float3 n_s1 = cone_side_n(h, k2, o_l, d_u, exit_);
  bool top_first = t_top < t_bot;
  float3 ez = float3(0.0f, 0.0f, 1.0f);
  float3 n_cap0 = top_first ? ez : -ez;
  float3 n_cap1 = top_first ? -ez : ez;
  float3 n0 = (enter == at0) ? n_cap0 : n_s0;
  float3 n1 = (exit_ == at1) ? n_cap1 : n_s1;
  float3 n_l = hit_enter ? n0 : -n1;
  th = t_l / lam;
  nh = mask ? from_local_n(pp, n_l) : float3(0.0f);
  return mask;
}

inline bool hit_ellipsoid(const thread float* pp, float3 o, float3 d, thread float& th, thread float3& nh) {
  float3 o_l; float3 d_u; float lam;
  to_local(pp, o, d, o_l, d_u, lam);
  float b = 2.0f * dot(o_l, d_u);
  float cq = dot(o_l, o_l) - 1.0f;
  float disc = b * b - 4.0f * cq;
  bool valid = disc > 1e-12f;
  float sq = sqrt(max(disc, 0.0f));
  float t1 = (-b - sq) / 2.0f;
  float t2 = (-b + sq) / 2.0f;
  float t_l = (valid && t1 > 1e-6f) ? t1 : t2;
  bool mask = valid && t_l > 1e-6f;
  float3 n_l = o_l + t_l * d_u;
  if (mask && t1 <= 1e-6f) n_l = -n_l;
  th = t_l / lam;
  nh = mask ? from_local_n(pp, n_l) : float3(0.0f);
  return mask;
}

inline bool prim_hit(int type, const thread float* pp, float3 o, float3 d, thread float& t, thread float3& n) {
  switch (type) {
    case 0: return hit_sphere(pp, o, d, t, n);
    case 1: return hit_box(pp, o, d, t, n);
    case 2: return hit_cylinder(pp, o, d, t, n);
    case 3: return hit_cone(pp, o, d, t, n);
    default: return hit_ellipsoid(pp, o, d, t, n);
  }
}

// node_ab：内部节点 [left, right]，叶子 [prim_idx, -1]
// 模板化指针参数：mlx-c 按数组大小选 constant/device 地址空间，两处都要能吃
template <typename Ptr>
inline float3 node_lo_at(Ptr node_lo, int ni) {
  return float3(node_lo[3 * ni], node_lo[3 * ni + 1], node_lo[3 * ni + 2]);
}
template <typename Ptr>
inline float3 node_hi_at(Ptr node_hi, int ni) {
  return float3(node_hi[3 * ni], node_hi[3 * ni + 1], node_hi[3 * ni + 2]);
}
"#;

const MSL_NEAREST: &str = r#"
  uint elem = thread_position_in_grid.x;
  uint n_rays = rays_o_shape[0];
  if (elem >= n_rays) return;
  float3 o = float3(rays_o[3 * elem], rays_o[3 * elem + 1], rays_o[3 * elem + 2]);
  float3 d = float3(rays_d[3 * elem], rays_d[3 * elem + 1], rays_d[3 * elem + 2]);
  float3 invd = 1.0f / d;
  float best_t = INFINITY;
  int best = -1;
  float3 best_n = float3(0.0f);
  int stack[64];
  int sp = 0;
  stack[sp++] = 0;
  while (sp > 0) {
    int ni = stack[--sp];
    float t_en;
    if (!aabb_entry(o, invd, node_lo_at(node_lo, ni), node_hi_at(node_hi, ni), t_en)) continue;
    if (!(t_en < best_t)) continue; // 随 best_t 收紧剪枝
    int2 ab = int2(node_ab[2 * ni], node_ab[2 * ni + 1]);
    if (ab.y < 0) {
      int pi = ab.x;
      float pp[32];
      for (int u = 0; u < 32; u++) pp[u] = prim_params[32 * pi + u];
      float t; float3 n;
      if (prim_hit(prim_type[pi], pp, o, d, t, n) && t < best_t) {
        best_t = t; best = prim_obj[pi]; best_n = n;
      }
    } else {
      float te0; float te1;
      bool h0 = aabb_entry(o, invd, node_lo_at(node_lo, ab.x), node_hi_at(node_hi, ab.x), te0) && te0 < best_t;
      bool h1 = aabb_entry(o, invd, node_lo_at(node_lo, ab.y), node_hi_at(node_hi, ab.y), te1) && te1 < best_t;
      if (h0 && h1) {
        if (te0 < te1) { stack[sp++] = ab.y; stack[sp++] = ab.x; }
        else { stack[sp++] = ab.x; stack[sp++] = ab.y; }
      } else if (h0) { stack[sp++] = ab.x; }
      else if (h1) { stack[sp++] = ab.y; }
    }
  }
  out_t[elem] = best_t;
  out_obj[elem] = best;
  out_n[3 * elem] = best_n.x;
  out_n[3 * elem + 1] = best_n.y;
  out_n[3 * elem + 2] = best_n.z;
"#;

const MSL_SHADOW: &str = r#"
  uint elem = thread_position_in_grid.x;
  uint n_rays = rays_o_shape[0];
  if (elem >= n_rays) return;
  float3 o = float3(rays_o[3 * elem], rays_o[3 * elem + 1], rays_o[3 * elem + 2]);
  float3 d = float3(rays_d[3 * elem], rays_d[3 * elem + 1], rays_d[3 * elem + 2]);
  float3 invd = 1.0f / d;
  float tmax = t_max[elem];
  float vis = 1.0f;
  int stack[64];
  int sp = 0;
  stack[sp++] = 0;
  while (sp > 0) {
    int ni = stack[--sp];
    float t_en;
    if (!aabb_entry(o, invd, node_lo_at(node_lo, ni), node_hi_at(node_hi, ni), t_en)) continue;
    if (!(t_en < tmax)) continue;
    int2 ab = int2(node_ab[2 * ni], node_ab[2 * ni + 1]);
    if (ab.y < 0) {
      int pi = ab.x;
      float pp[32];
      for (int u = 0; u < 32; u++) pp[u] = prim_params[32 * pi + u];
      float t; float3 n;
      // 命中即累计遮挡因子（半透明连乘），全不透明 → 提前退出
      if (prim_hit(prim_type[pi], pp, o, d, t, n) && t < tmax) {
        vis *= obj_occ[prim_obj[pi]];
        if (vis <= 0.0f) break;
      }
    } else {
      float te0; float te1;
      bool h0 = aabb_entry(o, invd, node_lo_at(node_lo, ab.x), node_hi_at(node_hi, ab.x), te0) && te0 < tmax;
      bool h1 = aabb_entry(o, invd, node_lo_at(node_lo, ab.y), node_hi_at(node_hi, ab.y), te1) && te1 < tmax;
      if (h0 && h1) {
        if (te0 < te1) { stack[sp++] = ab.y; stack[sp++] = ab.x; }
        else { stack[sp++] = ab.x; stack[sp++] = ab.y; }
      } else if (h0) { stack[sp++] = ab.x; }
      else if (h1) { stack[sp++] = ab.y; }
    }
  }
  out_vis[elem] = vis;
"#;

// ---------------------------------------------------------------------------
// Rust 侧：BVH 构建 + kernel 调用
// ---------------------------------------------------------------------------

struct Kernels {
    nearest: MetalKernel,
    shadow: MetalKernel,
}

fn kernels() -> Option<&'static Kernels> {
    static K: OnceLock<Option<Kernels>> = OnceLock::new();
    K.get_or_init(|| {
        let nearest = MetalKernel::compile(
            "cga_bvh_nearest",
            &[
                "rays_o", "rays_d", "node_lo", "node_hi", "node_ab", "prim_type", "prim_params",
                "prim_obj",
            ],
            &["out_t", "out_obj", "out_n"],
            MSL_NEAREST,
            MSL_HEADER,
        )
        .ok()?;
        let shadow = MetalKernel::compile(
            "cga_bvh_shadow",
            &[
                "rays_o", "rays_d", "node_lo", "node_hi", "node_ab", "prim_type", "prim_params",
                "prim_obj", "t_max", "obj_occ",
            ],
            &["out_vis"],
            MSL_SHADOW,
            MSL_HEADER,
        )
        .ok()?;
        Some(Kernels { nearest, shadow })
    })
    .as_ref()
}

/// 一场景一份：扁平 BVH 数组 + kernel 调用句柄。在相机空间构建
/// （`params_list` 已是相机空间参数，光线也在相机空间）。
pub(crate) struct BvhTrace {
    /// 调试：导出扁平数组（测试用）
    pub(crate) node_lo: Array,
    pub(crate) node_hi: Array,
    pub(crate) node_ab: Array,
    pub(crate) prim_type: Array,
    pub(crate) prim_params: Array,
    pub(crate) prim_obj: Array,
    pub(crate) obj_occ: Array,
    /// 逐场景对象：true → 该对象仍走旧数组路径。
    pub(crate) legacy: Vec<bool>,
}

/// 本 kernel 支持的图元 + 无贴图 + 有界（平面/无限长圆柱的 AABB 是 None）。
fn eligible(p: &GeometryParams, has_map: bool) -> Option<i32> {
    if has_map {
        return None;
    }
    let ty = match p {
        GeometryParams::SphereParams(_) => 0,
        GeometryParams::BoxParams(_) => 1,
        GeometryParams::CylinderParams(_) => 2,
        GeometryParams::ConeParams(_) => 3,
        GeometryParams::EllipsoidParams(_) => 4,
        _ => return None,
    };
    geom_bounds(p)?;
    Some(ty)
}

fn pack_params(p: &GeometryParams) -> [f32; PARAM_SLOTS] {
    let mut out = [0.0f32; PARAM_SLOTS];
    fn put3(out: &mut [f32; PARAM_SLOTS], i: usize, v: [f64; 3]) {
        out[i] = v[0] as f32;
        out[i + 1] = v[1] as f32;
        out[i + 2] = v[2] as f32;
    }
    match p {
        GeometryParams::SphereParams(SphereParams { c, r, .. }) => {
            put3(&mut out, 0, *c);
            out[3] = *r as f32;
        }
        GeometryParams::BoxParams(BoxParams { c, axes, half }) => {
            put3(&mut out, 0, *c);
            for (k, ax) in axes.iter().enumerate() {
                put3(&mut out, 3 + 3 * k, *ax);
            }
            put3(&mut out, 12, *half);
        }
        GeometryParams::CylinderParams(CylinderParams { q, u, r, h }) => {
            put3(&mut out, 0, *q);
            put3(&mut out, 3, *u);
            out[6] = *r as f32;
            out[7] = *h as f32;
        }
        GeometryParams::ConeParams(ConeParams {
            a_inv3, t_inv, r, h, ..
        }) => {
            for (j, row) in a_inv3.iter().enumerate() {
                put3(&mut out, 3 * j, *row);
            }
            put3(&mut out, 9, *t_inv);
            out[12] = *r as f32;
            out[13] = *h as f32;
        }
        GeometryParams::EllipsoidParams(EllipsoidParams { a_inv3, t_inv, .. }) => {
            for (j, row) in a_inv3.iter().enumerate() {
                put3(&mut out, 3 * j, *row);
            }
            put3(&mut out, 9, *t_inv);
        }
        _ => unreachable!("pack_params: ineligible type"),
    }
    out
}

struct Node {
    lo: [f32; 3],
    hi: [f32; 3],
    a: i32, // 内部 = 左子；叶 = prim 序号
    b: i32, // 内部 = 右子；叶 = -1
}

impl BvhTrace {
    /// 构建；返回 None = 本帧无可 kernel 化对象（全旧路径）。
    pub(crate) fn build(
        scene: &Scene,
        params_list: &[GeometryParams],
        mode: RenderMode,
    ) -> Option<BvhTrace> {
        let ks = kernels()?;
        let _ = ks;
        let n_obj = scene.objects.len();
        let mut legacy = vec![true; n_obj];
        let mut prims: Vec<(i32, i32, [f32; PARAM_SLOTS], [[f32; 3]; 2])> = Vec::new(); // (scene_idx, type, params, aabb)
        for (i, obj) in scene.objects.iter().enumerate() {
            let Some(ty) = eligible(&params_list[i], obj.material.map.is_some()) else {
                continue;
            };
            let bb = geom_bounds(&params_list[i])?;
            // f64 → f32 向內缩一点可能切掉边缘命中：向外扩 1e-4 相对量保守化
            let mut aabb = [[0.0f32; 3]; 2];
            for k in 0..3 {
                let l = bb[0][k] as f32;
                let h = bb[1][k] as f32;
                let pad = (h - l).abs().max(1.0) * 1e-4;
                aabb[0][k] = l - pad;
                aabb[1][k] = h + pad;
            }
            legacy[i] = false;
            prims.push((i as i32, ty, pack_params(&params_list[i]), aabb));
        }
        if prims.is_empty() {
            return None;
        }
        let p = prims.len();
        let aabbs: Vec<[[f32; 3]; 2]> = prims.iter().map(|x| x.3).collect();
        // 中位切分（内联 build_nodes 的逻辑，直接产出扁平数组）
        let mut order: Vec<i32> = (0..p as i32).collect();
        let mut nodes: Vec<Node> = Vec::with_capacity(2 * p - 1);
        fn split(aabbs: &[[[f32; 3]; 2]], prims: &mut [i32], nodes: &mut Vec<Node>) -> i32 {
            let mut nlo = [f32::INFINITY; 3];
            let mut nhi = [f32::NEG_INFINITY; 3];
            for &pi in prims.iter() {
                let b = &aabbs[pi as usize];
                for k in 0..3 {
                    nlo[k] = nlo[k].min(b[0][k]);
                    nhi[k] = nhi[k].max(b[1][k]);
                }
            }
            let idx = nodes.len() as i32;
            nodes.push(Node { lo: nlo, hi: nhi, a: -1, b: -1 });
            if prims.len() == 1 {
                nodes[idx as usize].a = prims[0];
                nodes[idx as usize].b = -1;
                return idx;
            }
            let mut clo = [f32::INFINITY; 3];
            let mut chi = [f32::NEG_INFINITY; 3];
            for &pi in prims.iter() {
                let b = &aabbs[pi as usize];
                for k in 0..3 {
                    let c = (b[0][k] + b[1][k]) * 0.5;
                    clo[k] = clo[k].min(c);
                    chi[k] = chi[k].max(c);
                }
            }
            let mut axis = 0;
            let mut best = chi[0] - clo[0];
            for k in 1..3 {
                if chi[k] - clo[k] > best {
                    best = chi[k] - clo[k];
                    axis = k;
                }
            }
            prims.sort_by(|&x, &y| {
                let cx = (aabbs[x as usize][0][axis] + aabbs[x as usize][1][axis]) * 0.5;
                let cy = (aabbs[y as usize][0][axis] + aabbs[y as usize][1][axis]) * 0.5;
                cx.partial_cmp(&cy)
                    .unwrap_or(std::cmp::Ordering::Equal)
                    .then(x.cmp(&y))
            });
            let mid = prims.len() / 2;
            let (l, r) = prims.split_at_mut(mid);
            let li = split(aabbs, l, nodes);
            let ri = split(aabbs, r, nodes);
            nodes[idx as usize].a = li;
            nodes[idx as usize].b = ri;
            idx
        }
        let root = split(&aabbs, &mut order, &mut nodes);
        debug_assert_eq!(root, 0);

        let mut node_lo = Vec::with_capacity(3 * nodes.len());
        let mut node_hi = Vec::with_capacity(3 * nodes.len());
        let mut node_ab = Vec::with_capacity(2 * nodes.len());
        for nd in &nodes {
            node_lo.extend_from_slice(&nd.lo);
            node_hi.extend_from_slice(&nd.hi);
            node_ab.push(nd.a);
            node_ab.push(nd.b);
        }
        let mut prim_type = Vec::with_capacity(p);
        let mut prim_params = Vec::with_capacity(PARAM_SLOTS * p);
        let mut prim_obj = Vec::with_capacity(p);
        // 注意：叶子里存的是 prims 打包表里的位置（0..P），与 scene 对象号的
        // 映射也在打包表里 —— order 重排的是叶子引用的位置，打包表本身不重排。
        for (scene_idx, ty, params, _) in &prims {
            prim_obj.push(*scene_idx);
            prim_type.push(*ty);
            prim_params.extend_from_slice(params);
        }
        let occ: Vec<f32> = scene
            .objects
            .iter()
            .map(|o| match mode {
                RenderMode::Normal | RenderMode::Toon => 1.0 - o.material.opacity as f32,
                RenderMode::IgnoreOpacity => 0.0,
            })
            .collect();
        Some(BvhTrace {
            node_lo: Array::from_slice(&node_lo, &[nodes.len() as i32, 3]),
            node_hi: Array::from_slice(&node_hi, &[nodes.len() as i32, 3]),
            node_ab: Array::from_slice(&node_ab, &[nodes.len() as i32, 2]),
            prim_type: Array::from_slice(&prim_type, &[p as i32]),
            prim_params: Array::from_slice(&prim_params, &[p as i32, PARAM_SLOTS as i32]),
            prim_obj: Array::from_slice(&prim_obj, &[p as i32]),
            obj_occ: Array::from_slice(&occ, &[n_obj as i32]),
            legacy,
        })
    }

    /// 光线包 → 最近命中（t, 场景对象号, 法向）。调用方负责把结果与旧路径
    /// 合并；miss：t = +inf、obj = -1（调用方钳成 0，与旧路径 miss 一致）。
    pub(crate) fn trace_nearest(&self, o: &Array, d: &Array) -> Option<(Array, Array, Array)> {
        let ks = kernels()?;
        let n = o.shape()[0] as i32;
        let outs = ks
            .nearest
            .apply(
                &[
                    o,
                    d,
                    &self.node_lo,
                    &self.node_hi,
                    &self.node_ab,
                    &self.prim_type,
                    &self.prim_params,
                    &self.prim_obj,
                ],
                &[
                    (mlx_sys::mlx_dtype__MLX_FLOAT32, &[n]),
                    (mlx_sys::mlx_dtype__MLX_INT32, &[n]),
                    (mlx_sys::mlx_dtype__MLX_FLOAT32, &[n, 3]),
                ],
                n,
            )
            .ok()?;
        Some((outs[0].clone(), outs[1].clone(), outs[2].clone()))
    }

    /// 光线包 → 阴影可见度连乘积（遮挡因子语义 = 旧路径 `eff_occlusion`）。
    pub(crate) fn trace_shadow(&self, o: &Array, d: &Array, tmax: &Array) -> Option<Array> {
        let ks = kernels()?;
        let n = o.shape()[0] as i32;
        let outs = ks
            .shadow
            .apply(
                &[
                    o,
                    d,
                    &self.node_lo,
                    &self.node_hi,
                    &self.node_ab,
                    &self.prim_type,
                    &self.prim_params,
                    &self.prim_obj,
                    tmax,
                    &self.obj_occ,
                ],
                &[(mlx_sys::mlx_dtype__MLX_FLOAT32, &[n])],
                n,
            )
            .ok()?;
        Some(outs[0].clone())
    }
}

// 供 renderer 读 env（一次性）：force / off / auto。
pub(crate) fn bvh_mode() -> BvhMode {
    use std::sync::OnceLock;
    static M: OnceLock<BvhMode> = OnceLock::new();
    *M.get_or_init(|| {
        if std::env::var("CGA_NO_BVH").is_ok() {
            BvhMode::Off
        } else if std::env::var("CGA_BVH_FORCE").is_ok() {
            BvhMode::Force
        } else {
            BvhMode::Auto
        }
    })
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum BvhMode {
    Off,
    Auto,
    Force,
}
