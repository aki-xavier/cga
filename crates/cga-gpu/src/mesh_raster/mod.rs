use crate::geometry_ops::geom_shadow;
use crate::mlxops::*;
use crate::scene::{Mesh, PerspectiveCamera};
use crate::shading::{shade_batched, Light};
use crate::texture::WrapMode;
use cga_core::GeometryParams;
use mlx_rs::{ops, Array};

pub mod rast_result;
pub use self::rast_result::*;
pub mod prep;
pub mod rast_face;
pub(crate) use prep::*;
pub mod gpu;
pub(crate) use gpu::*;

/// GPU 光栅化：prep（CPU 逐面）→ 可见性（GPU 覆盖 + scatter z-buffer）→ 着色
/// （GPU，含 D1 阴影）。阴影与着色只在命中像素上做；网格遮挡物跳过自阴影
/// （自身遮挡不自阴影——网格无 BVH，自遮挡是 O(命中像素×面数) 的开销）。
/// w/h/fx/fy/cx/cy 由调用方按 super-res 传入（AA 由此实现）。
#[allow(clippy::too_many_arguments)]
pub fn rasterize_meshes(
    objs: &[Mesh],
    camera: &PerspectiveCamera,
    w: i32,
    h: i32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    lit: &[Light],
    ambient: Option<Light>,
    occluders: &[GeometryParams],
    occluder_opacity: &[f32],
    occluder_is_mesh: &[bool],
) -> RastResult {
    let prep = prepare_faces(objs, camera, w, h, fx, fy, cx, cy);
    if prep.cam_v.is_empty() {
        return RastResult::empty(w, h);
    }
    let n = w * h;
    let vis = gpu_visibility(&prep, w, h, fx, fy, cx, cy);
    let depth = vis.depth;
    let hit_f = ck(vis.hit.reshape(&[n]));

    // 只对命中像素做着色（miss 像素在合成端被 closer&hit 滤掉）
    let hit_i = ck(hit_f.as_type::<i32>());
    let k = ck(hit_i.sum(None)).item_cast::<i32>().max(0) as usize;
    if k == 0 {
        return RastResult {
            depth,
            color: ck(ops::zeros::<f32>(&[h, w, 3])),
            opacity: ck(ops::zeros::<f32>(&[h, w])),
            hit: vis.hit,
        };
    }
    let order = ck(ops::argsort_axis(&ck(hit_i.negative()), 0)); // 命中在前
    let kidx = Array::from_slice(&(0..k as i32).collect::<Vec<_>>(), &[k as i32]);
    let idx_h = ck(order.take_axis(&kidx, 0));
    let pos = ck(ck(vis.pos.reshape(&[n, 3])).take_axis(&idx_h, 0));
    let nrm = ck(ck(vis.nrm.reshape(&[n, 3])).take_axis(&idx_h, 0));
    let uvf = ck(ck(vis.uv.reshape(&[n, 2])).take_axis(&idx_h, 0));
    let mat = ck(ck(vis.mat.reshape(&[n])).take_axis(&idx_h, 0));
    let mat_safe = ck(ops::clip(&mat, (0, (objs.len() as i32 - 1).max(0))));

    // 哪些网格对象有命中像素（用于跳过自阴影）
    mat.eval().unwrap();
    let matv = mat.as_slice::<i32>();
    let mut present: Vec<bool> = vec![false; objs.len()];
    for &m in matv {
        if m >= 0 {
            present[m as usize] = true;
        }
    }

    // D1 阴影：与 renderer::nearest 的阴影循环同逻辑；网格遮挡物跳过自阴影
    let p_s = ck(pos.add(&ck(nrm.multiply(&fs(1e-3)))));
    let mut shadow_vis: Vec<Array> = Vec::new();
    for light in lit {
        let (ld, _) = light.direction_at(&pos);
        let far = light.far(&pos);
        let mut v = ck(ops::ones::<f32>(&[k as i32]));
        for (j, params) in occluders.iter().enumerate() {
            let only_j = (0..objs.len()).all(|i| i == j || !present[i]);
            if occluder_is_mesh.get(j).copied().unwrap_or(false) && only_j {
                continue; // 自身遮挡 = 自阴影：跳过（无 BVH，开销与面数成正比）
            }
            let (st, m) = geom_shadow(params, &p_s, &ld);
            let occ = if far.ndim() == 1 {
                ck(m.logical_and(&ck(st.lt(&far))))
            } else {
                m
            };
            v = ck(v.multiply(&ck(ops::select(
                &occ,
                &fs(1.0 - occluder_opacity[j] as f64),
                &fs(1.0),
            ))));
        }
        shadow_vis.push(v);
    }

    // 材质堆栈 + 批量着色
    let mut em_arr: Vec<Array> = Vec::new();
    let mut diff_arr: Vec<Array> = Vec::new();
    let mut spec_arr: Vec<Array> = Vec::new();
    let mut expo_arr: Vec<Array> = Vec::new();
    let mut op_arr: Vec<Array> = Vec::new();
    for o in objs {
        let (em, diff, spec, expo) = o.material.shade_params();
        em_arr.push(arr3v(em));
        diff_arr.push(arr3v(diff));
        spec_arr.push(arr3v(spec));
        expo_arr.push(fs(expo));
        op_arr.push(fs(o.material.opacity));
    }
    let emissive = ck(ck(ops::stack(&em_arr, 0)).take_axis(&mat_safe, 0));
    let diff = ck(ck(ops::stack(&diff_arr, 0)).take_axis(&mat_safe, 0));
    let spec = ck(ck(ops::stack(&spec_arr, 0)).take_axis(&mat_safe, 0));
    let expo = ck(ck(ck(ops::stack(&expo_arr, 0)).take_axis(&mat_safe, 0)).expand_dims(1));
    let op_px = ck(ck(ops::stack(&op_arr, 0)).take_axis(&mat_safe, 0));

    let vv =
        ck(ck(pos.negative()).divide(&ck(ck(ck(pos.multiply(&pos)).sum_axes(&[-1], true)).sqrt())));
    let mut shaded = shade_batched(
        &emissive,
        &diff,
        &spec,
        &expo,
        &pos,
        &nrm,
        &vv,
        lit,
        ambient,
        &shadow_vis,
    );

    // 纹理（按对象 select，与光线路径 nearest 同模式）
    for (i, o) in objs.iter().enumerate() {
        if let Some(t) = &o.material.map {
            let sampled = ck(t
                .sample(&uvf, WrapMode::Repeat, WrapMode::Repeat)
                .take_axis(Array::from_slice(&[0i32, 1, 2], &[3]), 1));
            shaded = ck(ops::select(
                &ck(ck(mat.eq(Array::from_int(i as i32))).expand_dims(1)),
                &ck(shaded.multiply(&sampled)),
                &shaded,
            ));
        }
    }

    // 写回全分辨率缓冲（命中像素下标唯一，普通 scatter 即可）
    let color_full = ck(ops::zeros::<f32>(&[n, 3]));
    let color = ck(ops::indexing::scatter_single(
        &color_full,
        &idx_h,
        &ck(shaded.reshape(&[k as i32, 1, 3])),
        0,
    ));
    let op_full = ck(ops::zeros::<f32>(&[n]));
    let opacity = ck(ops::indexing::scatter_single(
        &op_full,
        &idx_h,
        &ck(op_px.reshape(&[k as i32, 1])),
        0,
    ));
    RastResult {
        depth,
        color: ck(color.reshape(&[h, w, 3])),
        opacity: ck(opacity.reshape(&[h, w])),
        hit: vis.hit,
    }
}

// ---------------------------------------------------------------------------
// CPU 参考实现：GPU 等价测试的 cross-check（#[cfg(test)]）。无阴影（vis 恒 1）、
// 背面剔除、近平面整面丢弃——语义差即 GPU 路径补全的 D1/D5/D6/D7。
// 未来 CPU 后端接管光栅时解除 cfg(test) 即可。
// ---------------------------------------------------------------------------

#[cfg(test)]
use self::rast_face::RastFace;
#[cfg(test)]
use crate::scene_graph::identity3;
#[cfg(test)]
use cga_core::Geometry;
#[cfg(test)]
use cga_core::{transform_point, TriUvs};

#[cfg(test)]
fn transform_normal(m: [f64; 16], p: [f64; 3]) -> [f64; 3] {
    [
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2],
    ]
}

#[cfg(test)]
#[allow(clippy::too_many_arguments)]
pub(crate) fn rasterize_meshes_cpu(
    objs: &[Mesh],
    camera: &PerspectiveCamera,
    w: i32,
    h: i32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    lit: &[Light],
    ambient: Option<Light>,
) -> RastResult {
    let pixel_count = (w * h) as usize;
    let mut depth = vec![f32::INFINITY; pixel_count];
    let mut pos = vec![0f32; pixel_count * 3];
    let mut nrm = vec![0f32; pixel_count * 3];
    let mut uvbuf = vec![0f32; pixel_count * 2];
    let mut matidx = vec![-1i32; pixel_count];

    let mut face_data: Vec<Vec<RastFace>> = Vec::new();
    for obj in objs {
        let geom = match &obj.geometry {
            Geometry::TrimeshGeometry(g) => g,
            _ => continue,
        };
        let wm = camera.motor.compose(&obj.motor());
        let (_, _, af) = wm.affine_from_motor(identity3());
        let mut faces: Vec<RastFace> = Vec::with_capacity(geom.n_faces as usize);
        for i in 0..geom.n_faces as usize {
            let a = transform_point(af, geom.v0[i]);
            let b = transform_point(
                af,
                [
                    geom.v0[i][0] + geom.e1[i][0],
                    geom.v0[i][1] + geom.e1[i][1],
                    geom.v0[i][2] + geom.e1[i][2],
                ],
            );
            let c = transform_point(
                af,
                [
                    geom.v0[i][0] + geom.e2[i][0],
                    geom.v0[i][1] + geom.e2[i][1],
                    geom.v0[i][2] + geom.e2[i][2],
                ],
            );
            let n = transform_normal(af, geom.nrm[i]);
            let has = !geom.uv.is_empty();
            let uv = if has {
                geom.uv[i]
            } else {
                TriUvs {
                    u0x: 0.0,
                    u0y: 0.0,
                    u1x: 0.0,
                    u1y: 0.0,
                    u2x: 0.0,
                    u2y: 0.0,
                }
            };
            faces.push(RastFace {
                a,
                b,
                c,
                n,
                uv,
                has_uv: has,
            });
        }
        face_data.push(faces);
    }

    for (mi, faces) in face_data.iter().enumerate() {
        for f in faces {
            let center = [
                (f.a[0] + f.b[0] + f.c[0]) / 3.0,
                (f.a[1] + f.b[1] + f.c[1]) / 3.0,
                (f.a[2] + f.b[2] + f.c[2]) / 3.0,
            ];
            if f.n[0] * center[0] + f.n[1] * center[1] + f.n[2] * center[2] >= 0.0 {
                continue;
            }

            if f.a[2] <= 1e-4 || f.b[2] <= 1e-4 || f.c[2] <= 1e-4 {
                continue;
            }
            let sax = fx * f.a[0] / f.a[2] + cx;
            let say = fy * f.a[1] / f.a[2] + cy;
            let sbx = fx * f.b[0] / f.b[2] + cx;
            let sby = fy * f.b[1] / f.b[2] + cy;
            let scx = fx * f.c[0] / f.c[2] + cx;
            let scy = fy * f.c[1] / f.c[2] + cy;
            let xmin = sax.min(sbx.min(scx)).floor().max(0.0) as i32;
            let xmax = sax.max(sbx.max(scx)).ceil().min(f64::from(w - 1)) as i32;
            let ymin = say.min(sby.min(scy)).floor().max(0.0) as i32;
            let ymax = say.max(sby.max(scy)).ceil().min(f64::from(h - 1)) as i32;
            let denom = (sbx - sax) * (scy - say) - (sby - say) * (scx - sax);
            if denom.abs() < 1e-12 {
                continue;
            }
            for py in ymin..ymax + 1 {
                let pyc = f64::from(py) + 0.5;
                for px in xmin..xmax + 1 {
                    let pxc = f64::from(px) + 0.5;

                    let wa = ((sbx - pxc) * (scy - pyc) - (sby - pyc) * (scx - pxc)) / denom;
                    let wb = ((scx - pxc) * (say - pyc) - (scy - pyc) * (sax - pxc)) / denom;
                    let wc = 1.0 - wa - wb;
                    if wa < 0.0 || wb < 0.0 || wc < 0.0 {
                        continue;
                    }

                    let za = f.a[2];
                    let zb = f.b[2];
                    let zc = f.c[2];
                    let inv_w = wa / za + wb / zb + wc / zc;
                    if inv_w <= 1e-12 {
                        continue;
                    }
                    let z = 1.0 / inv_w;
                    let off = (py * w + px) as usize;
                    if z >= f64::from(depth[off]) {
                        continue;
                    }
                    depth[off] = z as f32;
                    let ix = (wa * f.a[0] / za + wb * f.b[0] / zb + wc * f.c[0] / zc) / inv_w;
                    let iy = (wa * f.a[1] / za + wb * f.b[1] / zb + wc * f.c[1] / zc) / inv_w;
                    let iz = (wa * f.a[2] / za + wb * f.b[2] / zb + wc * f.c[2] / zc) / inv_w;
                    pos[off * 3] = ix as f32;
                    pos[off * 3 + 1] = iy as f32;
                    pos[off * 3 + 2] = iz as f32;
                    nrm[off * 3] = f.n[0] as f32;
                    nrm[off * 3 + 1] = f.n[1] as f32;
                    nrm[off * 3 + 2] = f.n[2] as f32;

                    if f.has_uv {
                        let uu =
                            (wa * f.uv.u0x / za + wb * f.uv.u1x / zb + wc * f.uv.u2x / zc) / inv_w;
                        let vv =
                            (wa * f.uv.u0y / za + wb * f.uv.u1y / zb + wc * f.uv.u2y / zc) / inv_w;
                        uvbuf[off * 2] = uu as f32;
                        uvbuf[off * 2 + 1] = vv as f32;
                    }
                    matidx[off] = mi as i32;
                }
            }
        }
    }

    let mut idxs: Vec<usize> = Vec::new();
    for (i, &mi) in matidx.iter().enumerate() {
        if mi >= 0 {
            idxs.push(i);
        }
    }
    let mut rast_hit_mask = vec![0f32; pixel_count];
    for &oi in &idxs {
        rast_hit_mask[oi] = 1.0;
    }
    let mut color = ck(ops::zeros::<f32>(&[h, w, 3]));

    if !idxs.is_empty() {
        let k = idxs.len() as i32;
        let pp = Array::from_slice(&f32s_from_pos(&pos, &idxs), &[k, 3]);
        let nn = Array::from_slice(&f32s_from_nrm(&nrm, &idxs), &[k, 3]);

        let vv =
            ck(ck(pp.negative()).divide(ck(ck(ck(pp.multiply(&pp)).sum_axes(&[-1], true)).sqrt())));

        let mut em_arr: Vec<Array> = Vec::new();
        let mut diff_arr: Vec<Array> = Vec::new();
        let mut spec_arr: Vec<Array> = Vec::new();
        let mut expo_arr: Vec<Array> = Vec::new();
        let mut valid_objs: Vec<&Mesh> = Vec::new();
        for o in objs {
            if matches!(o.geometry, Geometry::TrimeshGeometry(_)) {
                valid_objs.push(o);
            }
        }
        for o in &valid_objs {
            let (em, diff, spec, expo) = o.material.shade_params();
            em_arr.push(arr3v(em));
            diff_arr.push(arr3v(diff));
            spec_arr.push(arr3v(spec));
            expo_arr.push(fs(expo));
        }

        let mut mi_arr = vec![0i32; k as usize];
        for oi in 0..k as usize {
            mi_arr[oi] = matidx[idxs[oi]];
        }
        let midx = Array::from_slice(&mi_arr, &[k]);
        let emissive = ck(ck(ops::stack(&em_arr, 0)).take_axis(&midx, 0));
        let diff = ck(ck(ops::stack(&diff_arr, 0)).take_axis(&midx, 0));
        let spec = ck(ck(ops::stack(&spec_arr, 0)).take_axis(&midx, 0));
        let expo = ck(ck(ck(ops::stack(&expo_arr, 0)).take_axis(&midx, 0)).expand_dims(1));
        let vis: Vec<Array> = lit.iter().map(|_| ck(ops::ones::<f32>(&[k]))).collect();
        let shaded = shade_batched(
            &emissive, &diff, &spec, &expo, &pp, &nn, &vv, lit, ambient, &vis,
        );
        shaded.eval().unwrap();
        let mut sdata = shaded.as_slice::<f32>().to_vec();

        for (mi, o) in valid_objs.iter().enumerate() {
            if let Some(t) = &o.material.map {
                let mut uv_list: Vec<f32> = Vec::new();
                let mut bp: Vec<usize> = Vec::new();
                for oi in 0..k as usize {
                    if matidx[idxs[oi]] == mi as i32 {
                        bp.push(oi);
                        uv_list.push(uvbuf[idxs[oi] * 2]);
                        uv_list.push(uvbuf[idxs[oi] * 2 + 1]);
                    }
                }
                if !bp.is_empty() {
                    let uvarr = Array::from_slice(&uv_list, &[bp.len() as i32, 2]);
                    let samp = ck(t
                        .sample(&uvarr, WrapMode::Repeat, WrapMode::Repeat)
                        .take_axis(Array::from_slice(&[0i32, 1, 2], &[3]), 1));
                    samp.eval().unwrap();
                    let td = samp.as_slice::<f32>();
                    for j in 0..bp.len() {
                        let o3 = bp[j] * 3;
                        sdata[o3] *= td[j * 3];
                        sdata[o3 + 1] *= td[j * 3 + 1];
                        sdata[o3 + 2] *= td[j * 3 + 2];
                    }
                }
            }
        }

        let mut col_flat = vec![0f32; pixel_count * 3];
        for oi in 0..k as usize {
            let o = idxs[oi];
            col_flat[o * 3] = sdata[oi * 3];
            col_flat[o * 3 + 1] = sdata[oi * 3 + 1];
            col_flat[o * 3 + 2] = sdata[oi * 3 + 2];
        }
        color = Array::from_slice(&col_flat, &[h, w, 3]);
    }

    let hits = ck(Array::from_slice(&rast_hit_mask, &[h, w]).gt(fs(0.0)));
    RastResult {
        depth: Array::from_slice(&depth, &[h, w]),
        color,
        opacity: Array::from_slice(&rast_hit_mask, &[h, w]),
        hit: hits,
    }
}

#[cfg(test)]
fn f32s_from_pos(pos: &[f32], idxs: &[usize]) -> Vec<f32> {
    let mut out = vec![0f32; idxs.len() * 3];
    for (oi, &o) in idxs.iter().enumerate() {
        out[oi * 3] = pos[o * 3];
        out[oi * 3 + 1] = pos[o * 3 + 1];
        out[oi * 3 + 2] = pos[o * 3 + 2];
    }
    out
}

#[cfg(test)]
fn f32s_from_nrm(nrm: &[f32], idxs: &[usize]) -> Vec<f32> {
    let mut out = vec![0f32; idxs.len() * 3];
    for (oi, &o) in idxs.iter().enumerate() {
        out[oi * 3] = nrm[o * 3];
        out[oi * 3 + 1] = nrm[o * 3 + 1];
        out[oi * 3 + 2] = nrm[o * 3 + 2];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{MeshParams, PerspectiveCamera};
    use crate::scene_graph::Color;
    use crate::shading::{Material, MaterialParams};
    use cga_core::TrimeshGeometry;

    fn uv_sphere(stacks: usize, slices: usize) -> TrimeshGeometry {
        let mut verts: Vec<[f64; 3]> = Vec::new();
        let mut faces: Vec<[i32; 3]> = Vec::new();
        for i in 0..=stacks {
            let phi = std::f64::consts::PI * i as f64 / stacks as f64;
            for j in 0..slices {
                let th = 2.0 * std::f64::consts::PI * j as f64 / slices as f64;
                verts.push([
                    1.5 * phi.sin() * th.cos(),
                    1.5 * phi.cos(),
                    1.5 * phi.sin() * th.sin(),
                ]);
            }
        }
        for i in 0..stacks {
            for j in 0..slices {
                let a = (i * slices + j) as i32;
                let b = (i * slices + (j + 1) % slices) as i32;
                let c = ((i + 1) * slices + j) as i32;
                let d = ((i + 1) * slices + (j + 1) % slices) as i32;
                faces.push([a, b, c]);
                faces.push([b, d, c]);
            }
        }
        faces.retain(|f| {
            let (a, b, c) = (
                verts[f[0] as usize],
                verts[f[1] as usize],
                verts[f[2] as usize],
            );
            let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let cx = e1[1] * e2[2] - e1[2] * e2[1];
            let cy = e1[2] * e2[0] - e1[0] * e2[2];
            let cz = e1[0] * e2[1] - e1[1] * e2[0];
            cx * cx + cy * cy + cz * cz > 1e-20
        });
        TrimeshGeometry::new(&verts, &faces)
    }

    fn test_cam(w: i32, h: i32) -> PerspectiveCamera {
        let mut c = PerspectiveCamera::new(
            40.0,
            w as f64 / h as f64,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        c.look_at([0.0, 0.0, 0.0], None);
        c
    }

    fn red_mesh(g: TrimeshGeometry) -> Mesh {
        Mesh::new(MeshParams {
            geometry: Geometry::TrimeshGeometry(g),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0xC0392B),
                roughness: 0.3,
                metalness: 0.1,
                emissive: Color::from_hex(0x000000),
                opacity: 1.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        })
    }

    fn test_lights() -> Vec<Light> {
        vec![
            Light::directional(Color::from_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]),
            Light::ambient(Color::from_hex(0xFFFFFF), 0.3),
        ]
    }

    fn split_ambient(lit: &[Light]) -> (Vec<Light>, Option<Light>) {
        use crate::shading::LightKind;
        let mut d = Vec::new();
        let mut a = None;
        for l in lit {
            if l.kind == LightKind::Ambient {
                a = Some(*l);
            } else {
                d.push(*l);
            }
        }
        (d, a)
    }

    #[test]
    fn test_gpu_matches_cpu_closed_sphere() {
        // 封闭球网格：GPU 与 CPU 参考实现的可见性/深度/颜色一致。
        let (w, h) = (96i32, 72i32);
        let objs = vec![red_mesh(uv_sphere(10, 20))];
        let cam = test_cam(w, h);
        let lit = test_lights();
        let (dir, amb) = split_ambient(&lit);
        let fy = h as f64 / (2.0 * (40.0_f64.to_radians() / 2.0).tan());
        let fx = fy * cam.aspect;
        let (cx, cy) = ((w - 1) as f64 / 2.0, (h - 1) as f64 / 2.0);
        // GPU 路径传空遮挡表 → 阴影 vis 恒 1，与 CPU 参考语义对齐
        let gpu = rasterize_meshes(&objs, &cam, w, h, fx, fy, cx, cy, &dir, amb, &[], &[], &[]);
        let cpu = rasterize_meshes_cpu(&objs, &cam, w, h, fx, fy, cx, cy, &dir, amb);
        for a in [
            &gpu.depth, &gpu.color, &gpu.hit, &cpu.depth, &cpu.color, &cpu.hit,
        ] {
            a.eval().unwrap();
        }
        let gh = gpu.hit.as_slice::<bool>();
        let ch = cpu.hit.as_slice::<bool>();

        let agree = gh.iter().zip(ch).filter(|(a, b)| a == b).count();
        assert!(
            agree as f64 >= 0.995 * gh.len() as f64,
            "hit 掩码一致率 {:.4}",
            agree as f64 / gh.len() as f64
        );
        let gd = gpu.depth.as_slice::<f32>();
        let cd = cpu.depth.as_slice::<f32>();
        let mut max_rel = 0f64;
        for i in 0..gh.len() {
            if gh[i] && ch[i] {
                let rel = ((gd[i] - cd[i]) as f64).abs() / cd[i] as f64;
                max_rel = max_rel.max(rel);
            }
        }
        assert!(max_rel < 2e-3, "深度最大相对差 {max_rel}");
        let gc = gpu.color.as_slice::<f32>();
        let cc = cpu.color.as_slice::<f32>();
        let mut se = 0f64;
        let mut cnt = 0usize;
        for i in 0..gh.len() {
            if gh[i] && ch[i] {
                for c in 0..3 {
                    se += ((gc[i * 3 + c] - cc[i * 3 + c]) as f64).powi(2);
                    cnt += 1;
                }
            }
        }
        let rmse = (se / cnt as f64).sqrt();
        assert!(rmse < 0.01, "颜色 RMSE {rmse}");
    }

    #[test]
    fn test_gpu_double_sided_open_quad() {
        // D5：开放网格双面渲染——法线背对相机的面片仍可见，且光照按翻转法线。
        let (w, h) = (64i32, 64i32);
        let mk = |faces: &[[i32; 3]; 2]| {
            red_mesh(TrimeshGeometry::new(
                &[
                    [-1.0, -1.0, 2.0],
                    [1.0, -1.0, 2.0],
                    [1.0, 1.0, 2.0],
                    [-1.0, 1.0, 2.0],
                ],
                faces,
            ))
        };
        let front = mk(&[[0, 1, 2], [0, 2, 3]]); // 法线 -z（向相机）
        let back = mk(&[[0, 2, 1], [0, 3, 2]]); // 法线 +z（背相机）
        let cam = test_cam(w, h);
        let lit = test_lights();
        let (dir, amb) = split_ambient(&lit);
        let fy = h as f64 / (2.0 * (40.0_f64.to_radians() / 2.0).tan());
        let fx = fy * cam.aspect;
        let (cx, cy) = ((w - 1) as f64 / 2.0, (h - 1) as f64 / 2.0);
        let rf = rasterize_meshes(
            &[front],
            &cam,
            w,
            h,
            fx,
            fy,
            cx,
            cy,
            &dir,
            amb,
            &[],
            &[],
            &[],
        );
        let rb = rasterize_meshes(
            &[back],
            &cam,
            w,
            h,
            fx,
            fy,
            cx,
            cy,
            &dir,
            amb,
            &[],
            &[],
            &[],
        );
        rb.hit.eval().unwrap();
        assert!(
            rb.hit.as_slice::<bool>()[32 * 64 + 32],
            "背面面片中心应命中"
        );
        rf.color.eval().unwrap();
        rb.color.eval().unwrap();
        let cf = rf.color.as_slice::<f32>();
        let cb = rb.color.as_slice::<f32>();
        let o = (32 * 64 + 32) * 3;
        for c in 0..3 {
            assert!(
                (cf[o + c] - cb[o + c]).abs() < 0.02,
                "双面光照应一致：front={} back={}",
                cf[o + c],
                cb[o + c]
            );
        }
    }

    #[test]
    fn test_gpu_near_clip() {
        // D6：横跨近平面的三角形被裁剪而非整面丢弃。
        let (w, h) = (64i32, 64i32);
        // 大三角形，顶点 z = 2 / 2 / -5（穿过 z_near）
        let tri = red_mesh(TrimeshGeometry::new(
            &[[-4.0, -4.0, 2.0], [4.0, -4.0, 2.0], [0.0, 4.0, -5.0]],
            &[[0, 1, 2]],
        ));
        let cam = test_cam(w, h);
        let lit = test_lights();
        let (dir, amb) = split_ambient(&lit);
        let fy = h as f64 / (2.0 * (40.0_f64.to_radians() / 2.0).tan());
        let fx = fy * cam.aspect;
        let (cx, cy) = ((w - 1) as f64 / 2.0, (h - 1) as f64 / 2.0);
        let r = rasterize_meshes(&[tri], &cam, w, h, fx, fy, cx, cy, &dir, amb, &[], &[], &[]);
        r.hit.eval().unwrap();
        let hits = r.hit.as_slice::<bool>().iter().filter(|&&b| b).count();
        assert!(hits > 100, "近平面裁剪后应仍有大量命中，实际 {hits}");
    }
}
