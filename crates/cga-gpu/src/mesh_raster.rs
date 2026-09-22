use crate::mlxops::*;
use crate::scene::{Mesh, PerspectiveCamera};
use crate::scene_graph::identity3;
use crate::shading::{shade_batched, Light};
use crate::texture::WrapMode;
use cga_core::{affine_from_motor, transform_point, Geometry, TriUvs};
use mlx_rs::{ops, Array};

#[derive(Clone, Debug)]
pub struct RastResult {
    pub depth: Array,
    pub color: Array,
    pub hit: Array,
}

fn transform_normal(m: [f64; 16], p: [f64; 3]) -> [f64; 3] {
    [
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2],
    ]
}

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
        let (_, _, af) = affine_from_motor(wm, identity3());
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
        hit: hits,
    }
}

struct RastFace {
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
    n: [f64; 3],
    uv: TriUvs,
    has_uv: bool,
}

fn f32s_from_pos(pos: &[f32], idxs: &[usize]) -> Vec<f32> {
    let mut out = vec![0f32; idxs.len() * 3];
    for (oi, &o) in idxs.iter().enumerate() {
        out[oi * 3] = pos[o * 3];
        out[oi * 3 + 1] = pos[o * 3 + 1];
        out[oi * 3 + 2] = pos[o * 3 + 2];
    }
    out
}

fn f32s_from_nrm(nrm: &[f32], idxs: &[usize]) -> Vec<f32> {
    let mut out = vec![0f32; idxs.len() * 3];
    for (oi, &o) in idxs.iter().enumerate() {
        out[oi * 3] = nrm[o * 3];
        out[oi * 3 + 1] = nrm[o * 3 + 1];
        out[oi * 3 + 2] = nrm[o * 3 + 2];
    }
    out
}
