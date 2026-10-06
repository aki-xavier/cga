//! CPU 预处理：逐面变换到相机空间、近平面裁剪、投影、屏幕分桶。
//! 全是逐面标量工作（O(faces)），重活（覆盖测试/插值/着色）在 GPU。
use crate::scene::{Mesh, PerspectiveCamera};
use cga_core::{transform_point, Geometry, TriUvs, TrimeshGeometry};

/// 近平面：与光线路径的 t>1e-6 同量级（光线路径不看 camera.near）。
pub(crate) const Z_NEAR: f64 = 1e-4;

/// 单面片窗口上限（屏幕像素）。分桶窗口法对跨越整屏的大三角形会退化成
/// O(bbox 面积 × 面数)（行进四面体正好产生这种大面片），因此在 CPU 预处理
/// 时把大面片递归四分——相机空间位置与 uv 都是线性插值，细分无损。
const MAX_FACE_WINDOW_PX: f64 = 4096.0;
const MAX_SUBDIV_DEPTH: u8 = 6;

/// 全局面表（一个场景所有网格对象的面拼接）。
#[derive(Default)]
pub(crate) struct FacePrep {
    /// 相机空间三角形（每面 9 个 f32：x0,y0,z0,x1,...）
    pub cam_v: Vec<[f32; 9]>,
    /// 屏幕投影（每面 6 个 f32：sx0,sy0,sx1,sy1,sx2,sy2），super-res 像素单位
    pub scr: Vec<[f32; 6]>,
    /// 面法线（相机空间，未做朝向翻转）
    pub nrm: Vec<[f32; 3]>,
    /// 角点 uv（u0,v0,u1,v1,u2,v2）
    pub uv: Vec<[f32; 6]>,
    /// 面 → mesh_objs 下标
    pub obj: Vec<i32>,
    /// 屏幕 bbox [x0, y0, x1, y1]（含端点，已 clamp 到画面；空面不收入）
    pub bbox: Vec<[i32; 4]>,
}

/// Sutherland–Hodgman：三角形对平面 z >= zn 裁剪，输出 0..2 个三角形。
/// uv 沿边以同一 t 插值（uv 在三角形上是仿射的，相机空间线性插值精确）。
fn clip_tri_near(
    v: [[f64; 3]; 3],
    uv: [[f64; 2]; 3],
    zn: f64,
) -> Vec<([[f64; 3]; 3], [[f64; 2]; 3])> {
    let mut poly_v: Vec<[f64; 3]> = Vec::with_capacity(4);
    let mut poly_t: Vec<[f64; 2]> = Vec::with_capacity(4);
    for i in 0..3 {
        let a = v[i];
        let b = v[(i + 1) % 3];
        let (ta, tb) = (uv[i], uv[(i + 1) % 3]);
        let ain = a[2] >= zn;
        let bin = b[2] >= zn;
        if ain {
            poly_v.push(a);
            poly_t.push(ta);
        }
        if ain != bin {
            let t = (zn - a[2]) / (b[2] - a[2]);
            poly_v.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), zn]);
            poly_t.push([ta[0] + t * (tb[0] - ta[0]), ta[1] + t * (tb[1] - ta[1])]);
        }
    }
    let mut out = Vec::new();
    for i in 1..poly_v.len().saturating_sub(1) {
        out.push((
            [poly_v[0], poly_v[i], poly_v[i + 1]],
            [poly_t[0], poly_t[i], poly_t[i + 1]],
        ));
    }
    out
}

/// 把一个对象的面（对象空间）展开成 (三角, 法线, uv) 列表。
/// BezierPatch 走 tessellate()——与光线内核的 BezierParams::tessellated 同一份离散。
fn object_faces(obj: &Mesh) -> Vec<([[f64; 3]; 3], [f64; 3], [[f64; 2]; 3])> {
    let tri;
    let g = match &obj.geometry {
        Geometry::TrimeshGeometry(g) => g,
        Geometry::BezierPatchGeometry(b) => {
            let (verts, faces) = b.tessellate();
            tri = TrimeshGeometry::new(&verts, &faces);
            &tri
        }
        _ => return Vec::new(),
    };
    let zero_uv = [[0.0, 0.0]; 3];
    let mut out = Vec::with_capacity(g.n_faces as usize);
    for i in 0..g.n_faces as usize {
        let a = g.v0[i];
        let b = [
            g.v0[i][0] + g.e1[i][0],
            g.v0[i][1] + g.e1[i][1],
            g.v0[i][2] + g.e1[i][2],
        ];
        let c = [
            g.v0[i][0] + g.e2[i][0],
            g.v0[i][1] + g.e2[i][1],
            g.v0[i][2] + g.e2[i][2],
        ];
        let uv = if g.uv.is_empty() {
            zero_uv
        } else {
            let t: &TriUvs = &g.uv[i];
            [[t.u0x, t.u0y], [t.u1x, t.u1y], [t.u2x, t.u2y]]
        };
        out.push(([a, b, c], g.nrm[i], uv));
    }
    out
}

/// 逐面：对象空间 → 相机空间 → 近平面裁剪 → 投影 → bbox。
#[allow(clippy::too_many_arguments)]
pub(crate) fn prepare_faces(
    objs: &[Mesh],
    camera: &PerspectiveCamera,
    w: i32,
    h: i32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
) -> FacePrep {
    let mut out = FacePrep::default();
    for (oi, obj) in objs.iter().enumerate() {
        let wm = camera.motor.compose(&obj.motor());
        let (_, _, af) = wm.affine_from_motor(crate::scene_graph::identity3());
        for (tri, n, uv) in object_faces(obj) {
            let cv = [
                transform_point(af, tri[0]),
                transform_point(af, tri[1]),
                transform_point(af, tri[2]),
            ];
            // 法线只做线性部分（与旧光栅器一致）
            let cn = [
                af[0] * n[0] + af[1] * n[1] + af[2] * n[2],
                af[4] * n[0] + af[5] * n[1] + af[6] * n[2],
                af[8] * n[0] + af[9] * n[1] + af[10] * n[2],
            ];
            if cv[0][2] >= Z_NEAR && cv[1][2] >= Z_NEAR && cv[2][2] >= Z_NEAR {
                push_face(&mut out, cv, cn, uv, oi, w, h, fx, fy, cx, cy, 0);
            } else if cv[0][2] < Z_NEAR || cv[1][2] < Z_NEAR || cv[2][2] < Z_NEAR {
                for (ct, cu) in clip_tri_near(cv, uv, Z_NEAR) {
                    push_face(&mut out, ct, cn, cu, oi, w, h, fx, fy, cx, cy, 0);
                }
            }
        }
    }
    out
}

#[allow(clippy::too_many_arguments)]
fn push_face(
    out: &mut FacePrep,
    cv: [[f64; 3]; 3],
    cn: [f64; 3],
    uv: [[f64; 2]; 3],
    oi: usize,
    w: i32,
    h: i32,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    depth: u8,
) {
    let mut s = [0f64; 6];
    for i in 0..3 {
        s[i * 2] = fx * cv[i][0] / cv[i][2] + cx;
        s[i * 2 + 1] = fy * cv[i][1] / cv[i][2] + cy;
    }
    let x0 = s[0].min(s[2]).min(s[4]).floor().max(0.0) as i32;
    let y0 = s[1].min(s[3]).min(s[5]).floor().max(0.0) as i32;
    let x1 = s[0].max(s[2]).max(s[4]).ceil().min(f64::from(w - 1)) as i32;
    let y1 = s[1].max(s[3]).max(s[5]).ceil().min(f64::from(h - 1)) as i32;
    if x1 < x0 || y1 < y0 {
        return; // 完全屏外
    }
    // 大屏占面片：递归四分（无损），让窗口保持小
    let window_px = f64::from(x1 - x0 + 1) * f64::from(y1 - y0 + 1);
    if window_px > MAX_FACE_WINDOW_PX && depth < MAX_SUBDIV_DEPTH {
        let mid = |a: [f64; 3], b: [f64; 3]| {
            [
                0.5 * (a[0] + b[0]),
                0.5 * (a[1] + b[1]),
                0.5 * (a[2] + b[2]),
            ]
        };
        let mid2 = |a: [f64; 2], b: [f64; 2]| [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])];
        let m01 = (mid(cv[0], cv[1]), mid2(uv[0], uv[1]));
        let m12 = (mid(cv[1], cv[2]), mid2(uv[1], uv[2]));
        let m20 = (mid(cv[2], cv[0]), mid2(uv[2], uv[0]));
        for (tv, tu) in [
            ([cv[0], m01.0, m20.0], [uv[0], m01.1, m20.1]),
            ([m01.0, cv[1], m12.0], [m01.1, uv[1], m12.1]),
            ([m20.0, m12.0, cv[2]], [m20.1, m12.1, uv[2]]),
            ([m01.0, m12.0, m20.0], [m01.1, m12.1, m20.1]),
        ] {
            push_face(out, tv, cn, tu, oi, w, h, fx, fy, cx, cy, depth + 1);
        }
        return;
    }
    out.cam_v.push([
        cv[0][0] as f32,
        cv[0][1] as f32,
        cv[0][2] as f32,
        cv[1][0] as f32,
        cv[1][1] as f32,
        cv[1][2] as f32,
        cv[2][0] as f32,
        cv[2][1] as f32,
        cv[2][2] as f32,
    ]);
    out.scr.push([
        s[0] as f32,
        s[1] as f32,
        s[2] as f32,
        s[3] as f32,
        s[4] as f32,
        s[5] as f32,
    ]);
    out.nrm.push([cn[0] as f32, cn[1] as f32, cn[2] as f32]);
    out.uv.push([
        uv[0][0] as f32,
        uv[0][1] as f32,
        uv[1][0] as f32,
        uv[1][1] as f32,
        uv[2][0] as f32,
        uv[2][1] as f32,
    ]);
    out.obj.push(oi as i32);
    out.bbox.push([x0, y0, x1, y1]);
}
