//! GPU 可见性：两遍法（GPU scatter 不支持 8 字节 dtype，无法用 int64 打包键）。
//!
//! 每个面取屏幕包围盒窗口（尺寸按 2 的幂分桶，同桶面合成静态形状批次）：
//! - 第一遍：窗口内逐像素重心坐标 + 透视校正 z，scatter_min 进全局 f32 深度缓冲；
//! - 第二遍：重算同一批窗口（逐位确定性），取 z 与最终深度逐位相等者，
//!   scatter_min 写入全局 i32 面下标（共面 tie 取最小面号，确定）。
//! 最后按胜者面重算重心坐标，透视校正插值出 uv，像素光线重建位置，法线朝相机翻转。
use super::prep::FacePrep;
use crate::mlxops::*;
use mlx_rs::ops;
use mlx_rs::Array;

const EMPTY_ID: i32 = i32::MAX;
/// 单个 GPU 批次的窗口像素总量预算（B × bucket_w × bucket_h）。
const BATCH_PX: i32 = 16 << 20;
const BUCKETS: [i32; 9] = [8, 16, 32, 64, 128, 256, 512, 1024, 2048];

fn bucket_of(x: i32) -> i32 {
    for b in BUCKETS {
        if x <= b {
            return b;
        }
    }
    BUCKETS[BUCKETS.len() - 1]
}

/// 每像素可见性结果（super-res 全分辨率缓冲）。
pub(crate) struct GpuVis {
    pub depth: Array, // (h, w) f32，miss = +inf
    pub pos: Array,   // (h, w, 3) f32，miss = 0
    pub nrm: Array,   // (h, w, 3) f32，已朝相机翻转（与光线路径 cos_i 翻转同语义）
    pub uv: Array,    // (h, w, 2) f32
    pub mat: Array,   // (h, w) i32，mesh_objs 下标，miss = -1
    pub hit: Array,   // (h, w) bool
}

fn col6(a: &Array, i: i32) -> Array {
    // 标量下标（shape []）→ 轴被移除，(B,k) → (B,)
    ck(a.take_axis(Array::from_slice(&[i], &[]), -1))
}

/// 重心坐标（屏幕空间）：返回 (w0, w1, w2, den_ok)。
#[allow(clippy::too_many_arguments)]
fn bary(
    px: &Array,
    py: &Array,
    x0: &Array,
    y0: &Array,
    x1: &Array,
    y1: &Array,
    x2: &Array,
    y2: &Array,
) -> (Array, Array, Array, Array) {
    let den = ck(ck(ck(y1.subtract(y2)).multiply(&ck(x0.subtract(x2))))
        .add(&ck(ck(x2.subtract(x1)).multiply(&ck(y0.subtract(y2))))));
    let den_ok = s_gt(&ck(den.abs()), 1e-12);
    let den_s = ck(ops::select(&den_ok, &den, &ck(ops::ones_like(&den))));
    let w0 = ck(ck(ck(ck(y1.subtract(y2)).multiply(&ck(px.subtract(x2))))
        .add(&ck(ck(x2.subtract(x1)).multiply(&ck(py.subtract(y2))))))
    .divide(&den_s));
    let w1 = ck(ck(ck(ck(y2.subtract(y0)).multiply(&ck(px.subtract(x2))))
        .add(&ck(ck(x0.subtract(x2)).multiply(&ck(py.subtract(y2))))))
    .divide(&den_s));
    let w2 = ck(fs(1.0).subtract(&ck(w0.add(&w1))));
    (w0, w1, w2, den_ok)
}

/// 一批窗口的覆盖结果：扁平 (M,) 的像素下标 / z（未覆盖 = +inf）/ 面下标。
fn cover_z(
    prep: &FacePrep,
    faces: &[usize],
    bw: i32,
    bh: i32,
    w: i32,
    h: i32,
) -> (Array, Array, Array) {
    let b = faces.len() as i32;
    let wp = bw * bh;
    let mut ox = vec![0i32; b as usize];
    let mut oy = vec![0i32; b as usize];
    let mut base = vec![0i32; b as usize];
    let mut scr = vec![0f32; b as usize * 6];
    let mut zinv = vec![0f32; b as usize * 3];
    let mut fid = vec![0i32; b as usize];
    for (j, &fi) in faces.iter().enumerate() {
        let [x0, y0, _, _] = prep.bbox[fi];
        ox[j] = x0;
        oy[j] = y0;
        base[j] = y0 * w + x0;
        scr[j * 6..j * 6 + 6].copy_from_slice(&prep.scr[fi]);
        for c in 0..3 {
            zinv[j * 3 + c] = 1.0 / prep.cam_v[fi][c * 3 + 2];
        }
        fid[j] = fi as i32;
    }
    let ox_a = ck(Array::from_slice(&ox, &[b, 1]).as_type::<f32>());
    let oy_a = ck(Array::from_slice(&oy, &[b, 1]).as_type::<f32>());
    let base_a = Array::from_slice(&base, &[b, 1]);
    let fid_a = ck(Array::from_slice(&fid, &[b, 1]).as_type::<i32>());
    let scr_a = Array::from_slice(&scr, &[b, 6]);
    let zinv_a = Array::from_slice(&zinv, &[b, 3]);

    let i = ck(ops::arange::<i32, i32>(0, wp, 1).and_then(|x| x.expand_dims(0))); // (1, wp)
    let bw_a = Array::from_int(bw);
    let lx = ck(i.remainder(&bw_a).and_then(|x| x.as_type::<f32>()));
    let ly = ck(i.floor_divide(&bw_a).and_then(|x| x.as_type::<f32>()));
    let px = ck(ck(ox_a.add(&lx)).add(fs(0.5))); // (B, wp)
    let py = ck(ck(oy_a.add(&ly)).add(fs(0.5)));
    // 分桶窗口可能越出画面右/下缘——越界像素必须排除（scatter 不做边界检查）
    let in_bounds =
        ck(s_lt(&ck(ox_a.add(&lx)), f64::from(w))
            .logical_and(&s_lt(&ck(oy_a.add(&ly)), f64::from(h))));

    let (w0, w1, w2, den_ok) = bary(
        &px,
        &py,
        &ck(col6(&scr_a, 0).expand_dims(1)),
        &ck(col6(&scr_a, 1).expand_dims(1)),
        &ck(col6(&scr_a, 2).expand_dims(1)),
        &ck(col6(&scr_a, 3).expand_dims(1)),
        &ck(col6(&scr_a, 4).expand_dims(1)),
        &ck(col6(&scr_a, 5).expand_dims(1)),
    );
    let mut inside = ck(ops::broadcast_to(&den_ok, &[b, wp]));
    for wv in [&w0, &w1, &w2] {
        inside = ck(inside.logical_and(&s_ge(wv, -1e-6)));
    }

    let zi0 = ck(col6(&zinv_a, 0).expand_dims(1));
    let zi1 = ck(col6(&zinv_a, 1).expand_dims(1));
    let zi2 = ck(col6(&zinv_a, 2).expand_dims(1));
    let iz = ck(ck(ck(w0.multiply(&zi0)).add(&ck(w1.multiply(&zi1)))).add(&ck(w2.multiply(&zi2))));
    let z_ok = s_gt(&iz, 1e-9);
    inside = ck(inside.logical_and(&z_ok));
    let z = ck(fs(1.0).divide(&ck(ops::select(&z_ok, &iz, &ck(ops::ones_like(&iz))))));

    inside = ck(inside.logical_and(&in_bounds));
    let inf = ck(ops::full::<f32>(&[b, wp], &fs(f64::INFINITY)));
    let z_m = ck(ck(ops::select(&inside, &z, &inf)).reshape(&[b * wp]));
    let roww = ck(ck(i.floor_divide(&bw_a)).multiply(&Array::from_int(w)));
    let colw = ck(i.remainder(&bw_a));
    let idx = ck(
        ck(ck(ops::broadcast_to(&base_a, &[b, wp])).add(&ck(roww.add(&colw)))).reshape(&[b * wp]),
    );
    let fid_m = ck(ck(ops::broadcast_to(&fid_a, &[b, wp])).reshape(&[b * wp]));
    (idx, z_m, fid_m)
}

/// 覆盖 + z-buffer + 胜者解析。
#[allow(clippy::too_many_arguments)]
pub(crate) fn gpu_visibility(
    prep: &FacePrep,
    w: i32,
    h: i32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
) -> GpuVis {
    let n = w * h;
    let f_total = prep.cam_v.len();
    let mut depth = ck(ops::full::<f32>(&[n], &fs(f64::INFINITY)));
    let mut ids = ck(ops::full::<i32>(&[n], &Array::from_int(EMPTY_ID)));

    // 分桶 → 分批
    let mut groups: std::collections::BTreeMap<(i32, i32), Vec<usize>> =
        std::collections::BTreeMap::new();
    for (fi, bb) in prep.bbox.iter().enumerate() {
        let bw = bucket_of((bb[2] - bb[0] + 1).min(w));
        let bh = bucket_of((bb[3] - bb[1] + 1).min(h));
        groups.entry((bw, bh)).or_default().push(fi);
    }
    // 第一遍：深度。覆盖结果（idx/z/fid）尽量缓存下来给第二遍复用——重算覆盖是
    // 每批次 ~60 个 MLX op 的主要开销，缓存后第二遍只剩 ~6 个 op。
    // 缓存预算 32M 元素（idx/z/fid 各 4B ≈ 384MB），超了则第二遍重算。
    let cache_cap: usize = 32 << 20;
    let mut est: usize = 0;
    for ((bw, bh), faces) in &groups {
        est += faces.len() * (*bw as usize) * (*bh as usize);
    }
    let cache_ok = est <= cache_cap;
    let mut cached: Vec<(Array, Array, Array)> = Vec::new();
    for ((bw, bh), faces) in &groups {
        let per = (BATCH_PX / (bw * bh)).clamp(1, 32768) as usize;
        for chunk in faces.chunks(per) {
            let (idx, z_m, fid_m) = cover_z(prep, chunk, *bw, *bh, w, h);
            if cache_ok {
                idx.eval().unwrap();
                z_m.eval().unwrap();
                fid_m.eval().unwrap();
                cached.push((idx.clone(), z_m.clone(), fid_m.clone()));
            }
            let upd = ck(z_m.reshape(&[chunk.len() as i32 * bw * bh, 1]));
            depth = ck(ops::indexing::scatter_min_single(&depth, &idx, &upd, 0));
        }
    }
    depth.eval().unwrap();
    // 第二遍：胜者面下标（z 与最终深度逐位相等者；覆盖计算逐位确定）
    let mut ci = 0usize;
    for ((bw, bh), faces) in &groups {
        let per = (BATCH_PX / (bw * bh)).clamp(1, 32768) as usize;
        for chunk in faces.chunks(per) {
            let m = (chunk.len() * (*bw as usize) * (*bh as usize)) as i32;
            let (idx, z_m, fid_m) = if cache_ok {
                let (i, z, f) = &cached[ci];
                ci += 1;
                (i.clone(), z.clone(), f.clone())
            } else {
                cover_z(prep, chunk, *bw, *bh, w, h)
            };
            let d = ck(depth.take_axis(&idx, 0));
            let tie = ck(ck(z_m.eq(&d)).logical_and(&ck(z_m.lt(&fs(1e30)))));
            let empty = ck(ops::full::<i32>(&[m], &Array::from_int(EMPTY_ID)));
            let upd = ck(ck(ops::select(&tie, &fid_m, &empty)).reshape(&[m, 1]));
            ids = ck(ops::indexing::scatter_min_single(&ids, &idx, &upd, 0));
        }
    }
    ids.eval().unwrap();

    // 解析胜者
    let hit = ck(ids.lt(&Array::from_int(EMPTY_ID)));
    let fid_safe = ck(ops::clip(&ids, (0, (f_total as i32 - 1).max(0))));

    let cam_v = Array::from_slice(&prep.cam_v.concat(), &[f_total as i32, 9]);
    let nrm_a = Array::from_slice(&prep.nrm.concat(), &[f_total as i32, 3]);
    let uv_a = Array::from_slice(&prep.uv.concat(), &[f_total as i32, 6]);
    let obj_a = Array::from_slice(&prep.obj, &[f_total as i32]);
    let scr_a = Array::from_slice(&prep.scr.concat(), &[f_total as i32, 6]);

    let gv = ck(cam_v.take_axis(&fid_safe, 0)); // (N, 9)
    let gn = ck(nrm_a.take_axis(&fid_safe, 0)); // (N, 3)
    let gu = ck(uv_a.take_axis(&fid_safe, 0)); // (N, 6)
    let go = ck(obj_a.take_axis(&fid_safe, 0)); // (N,)
    let gs = ck(scr_a.take_axis(&fid_safe, 0)); // (N, 6)

    let idx_n = ck(ops::arange::<i32, i32>(0, n, 1));
    let w_a = Array::from_int(w);
    let px = ck(ck(idx_n.remainder(&w_a).and_then(|x| x.as_type::<f32>())).add(fs(0.5)));
    let py = ck(ck(idx_n.floor_divide(&w_a).and_then(|x| x.as_type::<f32>())).add(fs(0.5)));

    let (w0, w1, w2, _) = bary(
        &px,
        &py,
        &col6(&gs, 0),
        &col6(&gs, 1),
        &col6(&gs, 2),
        &col6(&gs, 3),
        &col6(&gs, 4),
        &col6(&gs, 5),
    );

    let zi = |c: usize| ck(fs(1.0).divide(&col6(&gv, (c * 3 + 2) as i32)));
    let zi0 = zi(0);
    let zi1 = zi(1);
    let zi2 = zi(2);
    let iz = ck(ck(ck(w0.multiply(&zi0)).add(&ck(w1.multiply(&zi1)))).add(&ck(w2.multiply(&zi2))));

    let depth = ck(depth.reshape(&[h, w]));
    let z = ck(depth.reshape(&[n]));

    // 位置：像素光线重建 p = z * ((px-cx)/fx, (py-cy)/fy, 1)（与插值同一点，精确）
    let ux = ck(ck(px.subtract(&fs(cx))).divide(&fs(fx)));
    let uy = ck(ck(py.subtract(&fs(cy))).divide(&fs(fy)));
    let zeros_n = ck(ops::zeros::<f32>(&[n]));
    let pos_x = ck(ops::select(&hit, &ck(z.multiply(&ux)), &zeros_n));
    let pos_y = ck(ops::select(&hit, &ck(z.multiply(&uy)), &zeros_n));
    let pos_z = ck(ops::select(&hit, &z, &zeros_n));
    let pos = ck(ck(ops::stack(&[&pos_x, &pos_y, &pos_z], -1)).reshape(&[h, w, 3]));

    // 法线：朝相机翻转（与光线路径 cos_i 翻转同语义），miss 置零
    let d_len = ck(ck(
        ck(ck(pos_z.multiply(&pos_z)).add(&ck(pos_x.multiply(&pos_x))))
            .add(&ck(pos_y.multiply(&pos_y))),
    )
    .sqrt());
    let d_len_s = ck(ops::select(
        &s_gt(&d_len, 1e-12),
        &d_len,
        &ck(ops::ones_like(&d_len)),
    ));
    let dot = ck(ck(ck(col6(&gn, 0).multiply(&ck(pos_x.divide(&d_len_s))))
        .add(&ck(col6(&gn, 1).multiply(&ck(pos_y.divide(&d_len_s))))))
    .add(&ck(col6(&gn, 2).multiply(&ck(pos_z.divide(&d_len_s))))));
    let flip = s_gt(&dot, 0.0);
    let mut nrm_cols: Vec<Array> = Vec::new();
    for c in 0..3 {
        let nc = col6(&gn, c);
        let flipped = ck(ops::select(&flip, &ck(nc.negative()), &nc));
        nrm_cols.push(ck(ops::select(&hit, &flipped, &zeros_n)));
    }
    let nrm =
        ck(ck(ops::stack(&[&nrm_cols[0], &nrm_cols[1], &nrm_cols[2]], -1)).reshape(&[h, w, 3]));

    // 透视校正 uv
    let u_sum = ck(ck(ck(ck(w0.multiply(&col6(&gu, 0))).multiply(&zi0))
        .add(&ck(ck(w1.multiply(&col6(&gu, 2))).multiply(&zi1))))
    .add(&ck(ck(w2.multiply(&col6(&gu, 4))).multiply(&zi2))));
    let u = ck(u_sum.divide(&iz));
    let v_sum = ck(ck(ck(ck(w0.multiply(&col6(&gu, 1))).multiply(&zi0))
        .add(&ck(ck(w1.multiply(&col6(&gu, 3))).multiply(&zi1))))
    .add(&ck(ck(w2.multiply(&col6(&gu, 5))).multiply(&zi2))));
    let v = ck(v_sum.divide(&iz));
    let u = ck(ops::select(&hit, &u, &zeros_n));
    let v = ck(ops::select(&hit, &v, &zeros_n));
    let uv = ck(ck(ops::stack(&[&u, &v], -1)).reshape(&[h, w, 2]));

    let m1 = ck(ops::full::<i32>(&[n], &Array::from_int(-1)));
    let mat = ck(ck(ops::select(&hit, &go, &m1)).reshape(&[h, w]));

    GpuVis {
        depth,
        pos,
        nrm,
        uv,
        mat,
        hit: ck(hit.reshape(&[h, w])),
    }
}
