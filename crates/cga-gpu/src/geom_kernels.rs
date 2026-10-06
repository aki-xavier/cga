use cga_core::{
    mat3_new, mat3_transpose, AffineGeometry, AffineParams, CsgOp, CsgParams, Geometry,
    GeometryParams, Mat3, Multivector,
};
use mlx_rs::ops;
use mlx_rs::Array;

use crate::mlxops::*;
use crate::{col, identity3, inf_like, tol, vec3_unit};

pub fn mat3_to_mlx(m: Mat3) -> Array {
    Array::from_slice(
        &[
            m[0][0] as f32,
            m[0][1] as f32,
            m[0][2] as f32,
            m[1][0] as f32,
            m[1][1] as f32,
            m[1][2] as f32,
            m[2][0] as f32,
            m[2][1] as f32,
            m[2][2] as f32,
        ],
        &[3, 3],
    )
}

pub fn vecmat(v: &Array, m: Mat3) -> Array {
    let mm = mat3_to_mlx(m);
    ck(ck(ck(v.expand_dims(-1)).multiply(&mm)).sum_axes(&[-2], false))
}

pub fn affine_to_local(
    a_inv3: Mat3,
    t_inv: [f64; 3],
    o: &Array,
    d: &Array,
) -> (Array, Array, Array) {
    let a3t = mat3_transpose(a_inv3);
    let t3 = arr3v(t_inv);
    let o_l = ck(vecmat(o, a3t).add(&t3));
    let d_l = vecmat(d, a3t);
    let mut lam = ck(ck(ck(d_l.multiply(&d_l)).sum_axes(&[-1], true)).sqrt());
    lam = ck(ops::select(
        s_gt(&lam, 1e-12),
        &lam,
        ck(ops::ones_like(&lam)),
    ));
    let d_u = ck(d_l.divide(&lam));
    (o_l, d_u, lam)
}

pub fn affine_normal(n_l: &Array, a_inv3: Mat3) -> Array {
    let n = vecmat(n_l, a_inv3);
    let norm = ck(ck(ck(n.multiply(&n)).sum_axes(&[-1], true)).sqrt());
    ck(n.divide(ck(ops::select(
        s_gt(&norm, 1e-12),
        &norm,
        ck(ops::ones_like(&norm)),
    ))))
}

pub fn affine_intersect(p: &AffineParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (t_l, n_l, mask) = crate::geom_intersect(&p.inner, &o_l, &d_u);
    let t = ck(t_l.divide(col(&lam, 0)));
    let mut n = affine_normal(&n_l, p.a_inv3);
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    (t, n, mask)
}

pub fn affine_shadow(p: &AffineParams, o: &Array, d: &Array) -> (Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (t_l, mask) = crate::geom_shadow(&p.inner, &o_l, &d_u);
    (ck(t_l.divide(col(&lam, 0))), mask)
}

pub fn affine_uv(p: &AffineParams, pos: &Array, n: &Array) -> Array {
    let p_l = crate::affine_point_to_local(p.a_inv3, p.t_inv, pos);
    crate::geom_uv(&p.inner, &p_l, n)
}

pub fn affine_crossings(p: &AffineParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (o_l, d_u, lam) = affine_to_local(p.a_inv3, p.t_inv, o, d);
    let (ts, mut ns, valid) = crate::geom_crossings(&p.inner, &o_l, &d_u);
    ns = affine_normal(&ns, p.a_inv3);
    (ck(ts.divide(ck(col(&lam, 0).expand_dims(1)))), ns, valid)
}

pub fn affine_contains(p: &AffineParams, pos: &Array) -> Array {
    let p_l = crate::affine_point_to_local(p.a_inv3, p.t_inv, pos);
    crate::geom_contains(&p.inner, &p_l)
}

/// 子节点包围球 vs 光线束的保守筛选：返回需要测试的光线下标。
/// `None` = 走全量（无界几何，或子集收益不足）；`Some(空)` = 无光线可命中。
pub(crate) fn child_ray_subset(child: &GeometryParams, o: &Array, d: &Array) -> Option<Vec<i32>> {
    let b = crate::geom_bounds(child)?;
    let (lo, hi) = (b[0], b[1]);
    let c = [
        0.5 * (lo[0] + hi[0]),
        0.5 * (lo[1] + hi[1]),
        0.5 * (lo[2] + hi[2]),
    ];
    let r =
        0.5 * ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt();
    if !(r.is_finite() && r >= 0.0) {
        return None;
    }
    let n = o.shape()[0];
    let to_c = ck(ck(o.negative()).add(&arr3v(c)));
    let proj = ck(ck(to_c.multiply(d)).sum_axes(&[-1], false));
    let perp2 =
        ck(ck(ck(to_c.multiply(&to_c)).sum_axes(&[-1], false)).subtract(&ck(proj.multiply(&proj))));
    let mask = ck(s_le(&perp2, r * r).logical_and(&s_ge(&proj, -r)));
    mask.eval().unwrap();
    let cnt = ck(mask.sum(None)).item_cast::<i32>();
    if cnt * 4 >= n * 3 {
        return None; // 子集收益不足 1/4：直接全量
    }
    if cnt == 0 {
        return Some(Vec::new());
    }
    let mut idx: Vec<i32> = Vec::with_capacity(cnt as usize);
    let data = mask.as_slice::<bool>();
    for (i, &v) in data.iter().enumerate() {
        if v {
            idx.push(i as i32);
        }
    }
    Some(idx)
}

/// 只对 `idx` 指定的光线求穿越点，散布回全量 (b, c) 形状；`空子集`用 +inf 占位
/// （列数由一次 1 光线探针取得，保持与全量调用一致的形状）。
fn child_crossings_subset(
    cp: &GeometryParams,
    o: &Array,
    d: &Array,
    idx: &[i32],
) -> (Array, Array, Array) {
    let b = o.shape()[0];
    let ids = Array::from_slice(idx, &[idx.len() as i32]);
    let o_s = ck(o.take_axis(&ids, 0));
    let d_s = ck(d.take_axis(&ids, 0));
    let (t_s, n_s, v_s) = crate::geom_crossings(cp, &o_s, &d_s);
    let c = t_s.shape()[1];
    let m = idx.len() as i32;
    // 散布回全量：t/n/valid 的未选中行留 +inf / 0 / false
    let t_full = ck(ops::indexing::scatter_single(
        &ck(ops::full::<f32>(&[b, c], &fs(f64::INFINITY))),
        &ids,
        &ck(t_s.reshape(&[m, 1, c])),
        0,
    ));
    let n_full = ck(ops::indexing::scatter_single(
        &ck(ops::zeros::<f32>(&[b, c, 3])),
        &ids,
        &ck(n_s.reshape(&[m, 1, c, 3])),
        0,
    ));
    let v_full_i = ck(ops::indexing::scatter_single(
        &ck(ops::zeros::<i32>(&[b, c])),
        &ids,
        &ck(ck(v_s.as_type::<i32>()).reshape(&[m, 1, c])),
        0,
    ));
    let v_full = ck(v_full_i.ne(Array::from_int(0)));
    (t_full, n_full, v_full)
}

/// 该子节点在全量子集为空时的占位列（列数用 1 光线探针取，值全 +inf / 0 / false）。
fn child_crossings_empty(cp: &GeometryParams, o: &Array) -> (Array, Array, Array) {
    let b = o.shape()[0];
    let dummy_o = ck(ops::zeros::<f32>(&[1, 3]));
    let dummy_d = ck(ck(ops::zeros::<f32>(&[1, 3])).add(&arr3v([0.0, 0.0, 1.0])));
    let (t1, _n1, _) = crate::geom_crossings(cp, &dummy_o, &dummy_d);
    let c = t1.shape()[1];
    (
        ck(ops::full::<f32>(&[b, c], &fs(f64::INFINITY))),
        ck(ops::zeros::<f32>(&[b, c, 3])),
        ck(ops::zeros_dtype(&[b, c], mlx_rs::Dtype::Bool)),
    )
}

pub(crate) fn csg_crossings(p: &CsgParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let (t, n, v, _) = csg_crossings_offsets(p, o, d);
    (t, n, v)
}

/// 同 `csg_crossings`，另返回每个子节点在列轴上的起始偏移（用于按子节点切片，
/// 避免为了凸判定重复计算穿越点）。
pub(crate) fn csg_crossings_offsets(
    p: &CsgParams,
    o: &Array,
    d: &Array,
) -> (Array, Array, Array, Vec<i32>) {
    let mut ts_l: Vec<Array> = Vec::new();
    let mut ns_l: Vec<Array> = Vec::new();
    let mut vs_l: Vec<Array> = Vec::new();
    for cp in &p.children {
        match child_ray_subset(cp, o, d) {
            Some(v) if v.is_empty() => {
                let (t, n, val) = child_crossings_empty(cp, o);
                ts_l.push(t);
                ns_l.push(n);
                vs_l.push(val);
                continue;
            }
            Some(v) => {
                let (t, n, val) = child_crossings_subset(cp, o, d, &v);
                ts_l.push(t);
                ns_l.push(n);
                vs_l.push(val);
                continue;
            }
            None => {}
        }
        let (t, n, v) = crate::geom_crossings(cp, o, d);
        ts_l.push(t);
        ns_l.push(n);
        vs_l.push(v);
    }
    let mut offs: Vec<i32> = Vec::with_capacity(p.children.len() + 1);
    let mut acc = 0i32;
    for t in &ts_l {
        offs.push(acc);
        acc += t.shape()[1];
    }
    offs.push(acc);
    let refs_t: Vec<&Array> = ts_l.iter().collect();
    let refs_n: Vec<&Array> = ns_l.iter().collect();
    let refs_v: Vec<&Array> = vs_l.iter().collect();
    (
        ck(ops::concatenate(&refs_t, 1)),
        ck(ops::concatenate(&refs_n, 1)),
        ck(ops::concatenate(&refs_v, 1)),
        offs,
    )
}

pub(crate) fn csg_contains(p: &CsgParams, pos: &Array) -> Array {
    if p.op == CsgOp::Difference {
        let first = crate::geom_contains(&p.children[0], pos);
        let mut rest = ck(ops::zeros_like(&first));
        for cp in &p.children[1..] {
            rest = ck(rest.logical_or(crate::geom_contains(cp, pos)));
        }
        return ck(first.logical_and(ck(rest.logical_not())));
    }
    let mut acc = crate::geom_contains(&p.children[0], pos);
    for cp in &p.children[1..] {
        let cc = crate::geom_contains(cp, pos);
        acc = if p.op == CsgOp::Union {
            ck(acc.logical_or(&cc))
        } else {
            ck(acc.logical_and(&cc))
        };
    }
    acc
}

/// 凸子节点（球/盒/柱/锥/椭球及其仿射包装）——这些是绝大多数布尔运算的实体。
fn is_convex_inner(p: &GeometryParams) -> bool {
    match p {
        GeometryParams::SphereParams(_)
        | GeometryParams::BoxParams(_)
        | GeometryParams::CylinderParams(_)
        | GeometryParams::ConeParams(_)
        | GeometryParams::EllipsoidParams(_) => true,
        GeometryParams::AffineParams(a) => is_convex_inner(&a.inner),
        _ => false,
    }
}

/// 凸子节点的成员关系由它自己的两个穿越点直接判定：`t_lo ≤ s ≤ t_hi`。
/// 凸体沿一条直线只交一段，这个判据是定义级的，不需要解析 contains——省掉逐
/// 采样点的认证求值（法兰这类"圆柱挖孔"的基准圆柱正是此项的最大开销）。
/// 仅在恰好 2 个穿越列、两者都有效且区间非退化时启用；相切/退化/非凸 → None
/// （回退到认证的 contains 路径）。
fn convex_membership(child: &GeometryParams, ts: &Array, lo: i32, s: &Array) -> Option<Array> {
    if !is_convex_inner(child) {
        return None;
    }
    // 该子节点恰好 2 列穿越（无效穿越在收集阶段已填 +inf）
    let t0 = ck(col(ts, lo).expand_dims(1));
    let t1 = ck(col(ts, lo + 1).expand_dims(1));
    let t_lo = ck(ops::minimum(&t0, &t1));
    let t_hi = ck(ops::maximum(&t0, &t1));
    let clean = s_gt(&ck(t_hi.subtract(&t_lo)), 1e-6); // 非退化（+inf 也在此被排除）
    let inside = ck(ck(s.ge(&t_lo)).logical_and(&ck(s.le(&t_hi))));
    Some(ck(inside.logical_and(&clean)))
}

/// 单子节点的成员关系（全量 (b, k1) bool）：只在“可能命中该子节点”的光线
/// 所对应的采样点上求值，其余置 false。无界子节点（平面）走全量。
fn child_contains_subset(
    child: &GeometryParams,
    o: &Array,
    d: &Array,
    flat: &Array,
    b: i32,
    k1: i32,
) -> Array {
    let Some(idx) = child_ray_subset(child, o, d) else {
        let mem = crate::geom_contains(child, flat);
        return ck(mem.reshape(&[b, k1]));
    };
    if idx.is_empty() {
        return ck(ops::zeros_dtype(&[b, k1], mlx_rs::Dtype::Bool));
    }
    let m = idx.len() as i32;
    let ids = Array::from_slice(&idx, &[m]);
    let pts = ck(ck(flat.reshape(&[b, k1, 3])).take_axis(&ids, 0));
    let pts = ck(pts.reshape(&[m * k1, 3]));
    let mem = ck(ck(crate::geom_contains(child, &pts).as_type::<i32>()).reshape(&[m, k1]));
    let full_i = ck(ops::indexing::scatter_single(
        &ck(ops::zeros::<i32>(&[b, k1])),
        &ids,
        &ck(mem.reshape(&[m, 1, k1])),
        0,
    ));
    ck(full_i.ne(Array::from_int(0)))
}

/// 与 `csg_contains` 同语义，但逐子节点做光线子集裁剪（区间分类的采样点里，
/// 大部分点离多数子节点很远——这一层裁剪是 CSG 求值的主要开销所在）。
fn csg_contains_culled(
    p: &CsgParams,
    o: &Array,
    d: &Array,
    s: &Array,
    pos: &Array,
    ts: &Array,
    offs: &[i32],
) -> Array {
    let shape = pos.shape().to_vec();
    let (b, k1) = (shape[0], shape[1]);
    let flat = ck(pos.reshape(&[b * k1, 3]));
    let dbg = std::env::var("CGA_CSG_TIME").is_ok();
    let mut mems: Vec<Array> = Vec::with_capacity(p.children.len());
    for (ci, cp) in p.children.iter().enumerate() {
        let t0 = std::time::Instant::now();
        let c_cols = offs[ci + 1] - offs[ci];
        let m = match if c_cols == 2 {
            convex_membership(cp, ts, offs[ci], s)
        } else {
            None
        } {
            Some(m) => m,
            None => child_contains_subset(cp, o, d, &flat, b, k1),
        };
        if dbg {
            m.eval().unwrap();
            eprintln!(
                "      child {ci}: {:.1} ms ({} 点)",
                t0.elapsed().as_secs_f64() * 1e3,
                b * k1
            );
        }
        mems.push(m);
    }
    if p.op == CsgOp::Difference {
        let first = mems[0].clone();
        let mut rest = ck(ops::zeros_dtype(&[b, k1], mlx_rs::Dtype::Bool));
        for m in &mems[1..] {
            rest = ck(rest.logical_or(m));
        }
        return ck(first.logical_and(&ck(rest.logical_not())));
    }
    let mut acc = mems[0].clone();
    for m in &mems[1..] {
        acc = if p.op == CsgOp::Union {
            ck(acc.logical_or(m))
        } else {
            ck(acc.logical_and(m))
        };
    }
    acc
}

fn csg_nearest_surface(p: &CsgParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    let dbg = std::env::var("CGA_CSG_TIME").is_ok();
    let t_all = std::time::Instant::now();
    let (ts, ns, _v, offs) = csg_crossings_offsets(p, o, d);
    if dbg {
        ts.eval().unwrap();
        eprintln!(
            "    csg: 穿越点收集 {:.1} ms ({} 列) 光线={}",
            t_all.elapsed().as_secs_f64() * 1e3,
            ts.shape()[1],
            ts.shape()[0]
        );
    }
    let b = ts.shape()[0];
    let k = ts.shape()[1] as usize;
    debug_assert!(k > 0, "csg needs >= 1 crossing column");
    let k1 = (k + 1) as i32;

    let ahead = ck(ck(ts.is_finite()).logical_and(s_gt(&ts, tol::T_MIN)));
    let tf = ck(ops::select(&ahead, &ts, &inf_like(&ts)));
    let order = ck(ops::argsort_axis(&tf, 1));
    let tf = ck(tf.take_along_axis(&order, 1));
    let ns_s = ck(ns.take_along_axis(ck(order.expand_dims(2)), 1));

    let mut va: Vec<i32> = vec![0];
    va.extend(0..k as i32);
    let mut vb: Vec<i32> = (0..k as i32).collect();
    vb.push(k as i32 - 1);
    let ja = ck(ops::broadcast_to(
        Array::from_slice(&va, &[1, k1]),
        &[b, k1],
    ));
    let jb = ck(ops::broadcast_to(
        Array::from_slice(&vb, &[1, k1]),
        &[b, k1],
    ));
    let a = ck(tf.take_along_axis(&ja, 1));
    let mut bc = ck(tf.take_along_axis(&jb, 1));
    let mut last = vec![0.0f32; k1 as usize];
    last[(k1 - 1) as usize] = 1.0;
    let last = ck(ops::broadcast_to(
        s_eq(&Array::from_slice(&last, &[1, k1]), 1.0),
        &[b, k1],
    ));
    bc = ck(ops::select(&last, &inf_like(&bc), &bc));

    let a_fin = ck(a.is_finite());
    let b_fin = ck(bc.is_finite());
    let both = ck(a_fin.logical_and(&b_fin));
    let tail = ck(a_fin.logical_and(ck(b_fin.logical_not())));
    let mid = ck(ops::select(
        &both,
        &ck(ck(a.add(&bc)).multiply(fs(0.5))),
        &ck(ops::zeros_like(&a)),
    ));
    let tail_s = ck(ck(a.add(fs(1.0))).multiply(fs(2.0)));
    let own_val = ck(ops::select(&tail, &tail_s, &mid));
    let gap = ck(bc.subtract(&a));
    let wide = ck(both.logical_and(s_gt(&ck(gap.subtract(&tol::tol_arr(&a))), 0.0)));
    let own = ck(wide.logical_or(&tail));

    let mut s_cols: Vec<Array> = Vec::with_capacity(k + 1);
    s_cols.push(ck(ops::zeros::<f32>(&[b, 1])));
    for j in 1..=k {
        let prev = s_cols[j - 1].clone();
        let own_j = ck(col(&own, j as i32).reshape(&[b, 1]));
        let val_j = ck(col(&own_val, j as i32).reshape(&[b, 1]));
        s_cols.push(ck(ops::select(&own_j, &val_j, &prev)));
    }
    let s = ck(ops::concatenate(&s_cols, 1));

    let t_pts = std::time::Instant::now();
    let pos = ck(ck(o.expand_dims(1)).add(ck(ck(s.expand_dims(2)).multiply(ck(d.expand_dims(1))))));
    if dbg {
        order.eval().unwrap();
        eprintln!(
            "    csg: 排序+采样点 {:.1} ms (采样点 {})",
            t_pts.elapsed().as_secs_f64() * 1e3,
            pos.shape()[1]
        );
    }
    let t_contains = std::time::Instant::now();
    let mem = csg_contains_culled(p, o, d, &s, &pos, &ts, &offs);
    if dbg {
        mem.eval().unwrap();
        eprintln!(
            "    csg: 区间 contains {:.1} ms ({} 子节点)",
            t_contains.elapsed().as_secs_f64() * 1e3,
            p.children.len()
        );
    }

    let ga = ck(ops::broadcast_to(
        Array::from_slice(&(0..k as i32).collect::<Vec<_>>(), &[1, k as i32]),
        &[b, k as i32],
    ));
    let gb = ck(ops::broadcast_to(
        Array::from_slice(&(1..=k as i32).collect::<Vec<_>>(), &[1, k as i32]),
        &[b, k as i32],
    ));
    let m0 = ck(mem.take_along_axis(&ga, 1));
    let m1 = ck(mem.take_along_axis(&gb, 1));
    let near = ck(ck(tf.is_finite()).logical_and(s_gt(&ck(tf.subtract(&tol::tol_arr(&tf))), 0.0)));
    let flip = ck(ck(m0.ne(&m1)).logical_and(&near));
    let cand = ck(ops::select(&flip, &tf, inf_like(&tf)));
    let t = ck(cand.min_axes(&[1], false));
    let mask = ck(t.is_finite());
    let idx = ck(ops::indexing::argmin_axis(&cand, 1, false));
    let mut n = ck(ck(ns_s.take_along_axis(
        ck(ops::broadcast_to(
            ck(ck(idx.expand_dims(1)).expand_dims(2)),
            &[ns_s.shape()[0], 1, 3],
        )),
        1,
    ))
    .take_axis(Array::from_slice(&[0], &[]), 1));
    n = ck(ops::select(
        ck(mask.expand_dims(1)),
        &n,
        ck(ops::zeros_like(&n)),
    ));
    if dbg {
        t.eval().unwrap();
        eprintln!(
            "    csg: 总计 {:.1} ms",
            t_all.elapsed().as_secs_f64() * 1e3
        );
    }
    (t, n, mask)
}

pub fn csg_intersect(p: &CsgParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    csg_nearest_surface(p, o, d)
}

pub fn csg_shadow(p: &CsgParams, o: &Array, d: &Array) -> (Array, Array) {
    let (t, _, mask) = csg_nearest_surface(p, o, d);
    (t, mask)
}

pub fn csg_uv(p: &CsgParams, pos: &Array, n: &Array) -> Array {
    let mut uv = ck(ops::zeros::<f32>(&[pos.shape()[0], 2]));
    let mut found = ck(ops::zeros_dtype(&[pos.shape()[0]], mlx_rs::Dtype::Bool));
    // Scale-relative probe: UV_PROBE times the tree's extent (floor 1). A
    // fixed 1e-4 probe collapses below the f32 ulp at large coordinates and
    // stops straddling the child boundary. Unbounded trees keep the base
    // value (their hit positions stay near the bounded siblings' scale).
    let extent = csg_bounds(p)
        .map(|[lo, hi]| (hi[0] - lo[0]).max(hi[1] - lo[1]).max(hi[2] - lo[2]))
        .unwrap_or(0.0);
    let delta = tol::UV_PROBE * extent.max(1.0);
    for cp in &p.children {
        let bp = ck(pos.add(s_mul(n, delta)));
        let bm = ck(pos.subtract(s_mul(n, delta)));
        let boundary = ck(crate::geom_contains(cp, &bp).ne(crate::geom_contains(cp, &bm)));
        let pick = ck(boundary.logical_and(ck(found.logical_not())));
        let uv_c = crate::geom_uv(cp, pos, n);
        uv = ck(ops::select(ck(pick.expand_dims(1)), &uv_c, &uv));
        found = ck(found.logical_or(&pick));
    }
    uv
}

pub fn geom_to_camera(g: &Geometry, m: &Multivector) -> GeometryParams {
    match g {
        Geometry::SphereGeometry(g) => {
            let s = m.apply(&g.blade);
            let (c, r) = s.to_sphere();
            GeometryParams::SphereParams(cga_core::SphereParams {
                c,
                r,
                axes: [
                    vec3_unit(m.apply(&Multivector::e1()).euclidean_vector()),
                    vec3_unit(m.apply(&Multivector::e2()).euclidean_vector()),
                    vec3_unit(m.apply(&Multivector::e3()).euclidean_vector()),
                ],
            })
        }
        Geometry::PlaneGeometry(g) => {
            let pi = m.apply(&g.blade);
            GeometryParams::PlaneParams(cga_core::PlaneParams {
                n: vec3_unit(pi.euclidean_vector()),
                d: pi.einf_coeff(),
            })
        }
        Geometry::CylinderGeometry(g) => GeometryParams::CylinderParams(cga_core::CylinderParams {
            q: m.apply(&Multivector::point(0.0, 0.0, 0.0)).coords(),
            u: vec3_unit(m.apply(&Multivector::e3()).euclidean_vector()),
            r: g.radius,
            h: g.half,
        }),
        Geometry::BoxGeometry(g) => {
            let hf = g.half;
            GeometryParams::BoxParams(cga_core::BoxParams {
                c: m.apply(&Multivector::point(0.0, 0.0, 0.0)).coords(),
                axes: [
                    vec3_unit(m.apply(&Multivector::e1()).euclidean_vector()),
                    vec3_unit(m.apply(&Multivector::e2()).euclidean_vector()),
                    vec3_unit(m.apply(&Multivector::e3()).euclidean_vector()),
                ],
                half: hf,
            })
        }
        Geometry::CircleGeometry(g) => GeometryParams::CircleParams(cga_core::CircleParams {
            c: m.apply(&Multivector::point(0.0, 0.0, 0.0)).coords(),
            n: vec3_unit(m.apply(&Multivector::e3()).euclidean_vector()),
            r: g.radius,
        }),
        Geometry::ConeGeometry(g) => {
            let (ai, ti, af) = m.affine_from_motor(identity3());
            GeometryParams::ConeParams(cga_core::ConeParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                r: g.radius,
                h: g.height,
            })
        }
        Geometry::TorusGeometry(g) => {
            let (ai, ti, af) = m.affine_from_motor(identity3());
            GeometryParams::TorusParams(cga_core::TorusParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                major: g.major,
                minor: g.minor,
                arc: g.arc,
            })
        }
        Geometry::EllipsoidGeometry(g) => {
            let rr = g.radii;
            let diag = mat3_new([rr[0], 0.0, 0.0], [0.0, rr[1], 0.0], [0.0, 0.0, rr[2]]);
            let (ai, ti, af) = m.affine_from_motor(diag);
            GeometryParams::EllipsoidParams(cga_core::EllipsoidParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
            })
        }
        Geometry::CyclideGeometry(g) => {
            let (ai, ti, af) = m.affine_from_motor(identity3());
            let sh = g.shift;
            GeometryParams::CyclideParams(cga_core::CyclideParams {
                a_inv3: ai,
                t_inv: ti,
                a_fwd: af,
                a: g.a,
                b: g.b,
                d: g.d,
                c: (g.a * g.a - g.b * g.b).sqrt(),
                shift: sh,
            })
        }
        Geometry::CsgGeometry(g) => {
            let mut ch: Vec<GeometryParams> = Vec::new();
            for c in &g.children {
                ch.push(geom_to_camera(c, m));
            }
            GeometryParams::CsgParams(CsgParams {
                op: g.op,
                children: ch,
            })
        }
        Geometry::AffineGeometry(g) => GeometryParams::AffineParams(affine_to_camera(g, m)),
    }
}

pub fn affine_to_camera(g: &AffineGeometry, m: &Multivector) -> AffineParams {
    let ip = geom_to_camera(&g.inner[0], &Multivector::identity());
    let full = m.gp(&g.motor);
    let (ai, ti, af) = full.affine_from_motor(g.linear);
    AffineParams {
        inner: Box::new(ip),
        a_inv3: ai,
        t_inv: ti,
        a_fwd: af,
    }
}

pub fn csg_bounds(p: &CsgParams) -> Option<[[f64; 3]; 2]> {
    let mut bnds: Vec<Option<[[f64; 3]; 2]>> = Vec::new();
    for cp in &p.children {
        bnds.push(crate::geom_bounds(cp));
    }
    if p.op == CsgOp::Difference {
        return bnds[0];
    }
    let mut bounded: Vec<[[f64; 3]; 2]> = Vec::new();
    for bb in bnds.iter().flatten() {
        bounded.push(*bb);
    }
    if bounded.is_empty() {
        return None;
    }
    if p.op == CsgOp::Union {
        let mut bmin = [0.0; 3];
        let mut bmax = [0.0; 3];
        let mut first = true;
        for bb in &bounded {
            if first {
                bmin = bb[0];
                bmax = bb[1];
                first = false;
            } else {
                for i in 0..3 {
                    if bb[0][i] < bmin[i] {
                        bmin[i] = bb[0][i];
                    }
                    if bb[1][i] > bmax[i] {
                        bmax[i] = bb[1][i];
                    }
                }
            }
        }
        return Some([bmin, bmax]);
    }

    let mut bmin = [0.0; 3];
    let mut bmax = [0.0; 3];
    let mut first = true;
    for bb in &bounded {
        if first {
            bmin = bb[0];
            bmax = bb[1];
            first = false;
        } else {
            for i in 0..3 {
                if bb[0][i] > bmin[i] {
                    bmin[i] = bb[0][i];
                }
                if bb[1][i] < bmax[i] {
                    bmax[i] = bb[1][i];
                }
            }
        }
    }
    Some([bmin, bmax])
}

#[allow(dead_code)]
pub(crate) fn to_f32_3(v: &[[f64; 3]]) -> Vec<f32> {
    let mut out = vec![0.0f32; v.len() * 3];
    for i in 0..v.len() {
        out[i * 3] = v[i][0] as f32;
        out[i * 3 + 1] = v[i][1] as f32;
        out[i * 3 + 2] = v[i][2] as f32;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{
        decompose_rigid, AffineGeometry, BoxGeometry, ConeGeometry, CsgGeometry, Multivector,
    };

    fn tf_ray(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn tf_contains(g: &Geometry, m: &Multivector, pts: &[[f64; 3]]) -> Vec<bool> {
        let p = geom_to_camera(g, m);
        let mut flat: Vec<f32> = Vec::with_capacity(pts.len() * 3);
        for q in pts {
            flat.push(q[0] as f32);
            flat.push(q[1] as f32);
            flat.push(q[2] as f32);
        }
        let pos = Array::from_slice(&flat, &[pts.len() as i32, 3]);
        let got = crate::geom_contains(&p, &pos);
        got.eval().unwrap();
        got.as_slice::<bool>().to_vec()
    }

    fn tf_hit(g: &Geometry, m: &Multivector, o: [f64; 3], d: [f64; 3]) -> (f32, [f32; 3], bool) {
        let p = geom_to_camera(g, m);
        let (t, n, mask) =
            crate::geom_intersect(&p, &tf_ray(o[0], o[1], o[2]), &tf_ray(d[0], d[1], d[2]));
        n.eval().unwrap();
        t.eval().unwrap();
        mask.eval().unwrap();
        let nd = n.as_slice::<f32>();
        (
            t.item_cast::<f32>(),
            [nd[0], nd[1], nd[2]],
            mask.as_slice::<bool>()[0],
        )
    }

    #[test]
    fn test_moved_cone_contains() {
        let g = Geometry::ConeGeometry(ConeGeometry::new(1.0, 2.0));

        assert_eq!(
            tf_contains(
                &g,
                &Multivector::translator([1.0, 0.0, 0.0]),
                &[[1.2, 0.0, 0.0], [1.6, 0.0, 0.0]]
            ),
            [true, false]
        );

        assert_eq!(
            tf_contains(
                &g,
                &Multivector::identity(),
                &[[0.0, 0.0, 0.0], [0.6, 0.0, 0.0]]
            ),
            [true, false]
        );
    }
    #[test]
    fn test_csg_moved_cone_contains() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Union,
            vec![
                Geometry::ConeGeometry(ConeGeometry::new(1.0, 2.0)),
                Geometry::ConeGeometry(ConeGeometry::new(1.0, 2.0)),
            ],
        ));
        assert_eq!(
            tf_contains(
                &g,
                &Multivector::translator([1.0, 0.0, 0.0]),
                &[[1.2, 0.0, 0.0], [1.6, 0.0, 0.0]]
            ),
            [true, false]
        );
    }
    #[test]
    fn test_rotated_box_normal() {
        let g = Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0));
        let m = Multivector::rotor([1.0, 0.0, 0.0], std::f64::consts::PI / 6.0);
        let (t, n, hit) = tf_hit(&g, &m, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(hit);
        assert!((f64::from(t) - 4.4226).abs() < 1e-3);
        assert!(f64::from(n[0]).abs() < 1e-2);
        assert!((f64::from(n[1]) + 0.5).abs() < 1e-2);
        assert!((f64::from(n[2]) - 3.0f64.sqrt() / 2.0).abs() < 1e-2);
    }

    #[test]
    fn test_box_inside_exit_normal() {
        let g = Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0));
        let (t, n, hit) = tf_hit(
            &g,
            &Multivector::identity(),
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        );
        assert!(hit);
        assert!((f64::from(t) - 0.5).abs() < 1e-3);
        assert!((f64::from(n[0]) - 1.0).abs() < 1e-2);
        assert!(f64::from(n[1]).abs() < 1e-2);
        assert!(f64::from(n[2]).abs() < 1e-2);
    }

    #[test]
    fn test_box_entry_normal_unchanged() {
        let g = Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0));
        let (t, n, hit) = tf_hit(
            &g,
            &Multivector::identity(),
            [0.0, 0.0, 5.0],
            [0.0, 0.0, -1.0],
        );
        assert!(hit);
        assert!((f64::from(t) - 4.5).abs() < 1e-3);
        assert!(f64::from(n[0]).abs() < 1e-2);
        assert!(f64::from(n[1]).abs() < 1e-2);
        assert!((f64::from(n[2]) - 1.0).abs() < 1e-2);
    }

    fn em_ray(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn em_hit(g: &Geometry, o: [f64; 3], d: [f64; 3]) -> (f32, [f32; 3], bool) {
        let p = geom_to_camera(g, &Multivector::identity());
        let (t, n, mask) =
            crate::geom_intersect(&p, &em_ray(o[0], o[1], o[2]), &em_ray(d[0], d[1], d[2]));
        n.eval().unwrap();
        t.eval().unwrap();
        mask.eval().unwrap();
        let nd = n.as_slice::<f32>();
        (
            t.item_cast::<f32>(),
            [nd[0], nd[1], nd[2]],
            mask.as_slice::<bool>()[0],
        )
    }

    fn em_contains(g: &Geometry, pts: &[[f64; 3]]) -> Vec<bool> {
        let p = geom_to_camera(g, &Multivector::identity());
        let mut flat: Vec<f32> = Vec::with_capacity(pts.len() * 3);
        for q in pts {
            flat.push(q[0] as f32);
            flat.push(q[1] as f32);
            flat.push(q[2] as f32);
        }
        let pos = Array::from_slice(&flat, &[pts.len() as i32, 3]);
        let got = crate::geom_contains(&p, &pos);
        got.eval().unwrap();
        got.as_slice::<bool>().to_vec()
    }

    #[test]
    fn test_affine_scaled_sphere() {
        let g = Geometry::AffineGeometry(AffineGeometry::new(
            Geometry::SphereGeometry(cga_core::SphereGeometry::new(1.0)),
            mat3_new([2.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        ));
        let (t, n, m) = em_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 4.0).abs() < 1e-5);
        assert!(f64::from(n[0]).abs() < 1e-4);
        assert!(f64::from(n[1]).abs() < 1e-4);
        assert!((f64::from(n[2]) - 1.0).abs() < 1e-4);
        let (_, _, m2) = em_hit(&g, [3.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m2);
    }

    #[test]
    fn test_affine_decompose_rigid_roundtrip() {
        let m =
            Multivector::translator([1.0, 2.0, 3.0]).gp(&Multivector::rotor([0.0, 1.0, 0.0], 0.7));
        let (motor2, lin) = decompose_rigid(m.to_matrix());
        let mut err = 0.0f64;
        for (i, row) in lin.iter().enumerate() {
            for (j, &v) in row.iter().enumerate() {
                let want = if i == j { 1.0 } else { 0.0 };
                let e = (v - want).abs();
                if e > err {
                    err = e;
                }
            }
        }
        assert!(err < 1e-4);
        let tm = motor2.to_matrix();
        assert!((tm[3] - 1.0).abs() < 1e-4);
        assert!((tm[7] - 2.0).abs() < 1e-4);
        assert!((tm[11] - 3.0).abs() < 1e-4);
    }

    #[test]
    fn test_cone_side_ray() {
        let g = Geometry::ConeGeometry(ConeGeometry::new(1.0, 2.0));
        let (t, _, m) = em_hit(&g, [5.0, 0.0, -0.5], [-1.0, 0.0, 0.0]);
        assert!(m);
        assert!((f64::from(t) - 4.25).abs() < 1e-4);
    }

    #[test]
    fn test_cone_contains() {
        let g = Geometry::ConeGeometry(ConeGeometry::new(1.0, 2.0));
        assert_eq!(
            em_contains(&g, &[[0.0, 0.0, 0.0], [0.6, 0.0, 0.0]]),
            [true, false]
        );
    }

    #[test]
    fn test_torus_rays() {
        let g = Geometry::TorusGeometry(cga_core::TorusGeometry::new(1.0, 0.3));
        let (_, _, m) = em_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m);
        let (t2, n2, m2) = em_hit(&g, [1.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m2);
        assert!((f64::from(t2) - 4.7).abs() < 1e-3);
        assert!(f64::from(n2[0]).abs() < 1e-2);
        assert!(f64::from(n2[1]).abs() < 1e-2);
        assert!((f64::from(n2[2]) - 1.0).abs() < 1e-2);
    }

    #[test]
    fn test_torus_contains() {
        let g = Geometry::TorusGeometry(cga_core::TorusGeometry::new(1.0, 0.3));
        assert_eq!(
            em_contains(&g, &[[1.0, 0.0, 0.1], [0.0, 0.0, 0.0]]),
            [true, false]
        );
    }

    #[test]
    fn test_tube_rays() {
        let g = Geometry::TorusGeometry(cga_core::TorusGeometry::tube(
            1.0,
            0.3,
            std::f64::consts::PI,
        ));
        for (x, y) in [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)] {
            let (t, _, m) = em_hit(&g, [x, y, 5.0], [0.0, 0.0, -1.0]);
            assert!(m);
            assert!((f64::from(t) - 4.7).abs() < 1e-3);
        }
        let (_, _, m) = em_hit(&g, [0.0, -1.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m);
    }

    #[test]
    fn test_tube_contains() {
        let g = Geometry::TorusGeometry(cga_core::TorusGeometry::tube(
            1.0,
            0.3,
            std::f64::consts::PI,
        ));
        assert_eq!(
            em_contains(&g, &[[0.0, 1.0, 0.1], [0.0, -1.0, 0.1], [1.0, 0.0, 0.1]]),
            [true, false, true]
        );
    }

    #[test]
    fn test_ellipsoid_is_scaled_sphere() {
        let g = Geometry::EllipsoidGeometry(cga_core::EllipsoidGeometry::new(2.0, 1.0, 1.0));
        let (t, _, m) = em_hit(&g, [0.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 4.0).abs() < 1e-4);
    }
}
