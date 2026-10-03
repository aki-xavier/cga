//! P0 — 退化用例库：量化 `docs/freeform-robust-boolean.md` §1 的三个根本问题
//! （尺度依赖 / 二值无证据 / 相切盲区）。
//!
//! 每条用例断言的是**期望的正确行为**。P1（Lipschitz 界 + `Unknown` 三值逻辑）落地前
//! 会失败的用例带 `#[ignore]`，理由指向文档对应章节。
//! `cargo test -p cga-gpu -- --ignored` 即打印当前失败清单；P1 验收 = 清单清空。

use cga_core::{
    box_geometry, csg_geometry, cylinder_geometry, extrude, motor_identity, sphere_geometry,
    transformed_geometry, translator, trimesh_geometry, CsgOp, Geometry, GeometryParams, Mat3,
};
use mlx_rs::Array;

use crate::{geom_contains, geom_intersect, geom_to_camera};

const IDENT: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn cam(g: &Geometry) -> GeometryParams {
    geom_to_camera(g, &motor_identity())
}

fn vec3(v: [f64; 3]) -> Array {
    Array::from_slice(&[v[0] as f32, v[1] as f32, v[2] as f32], &[1, 3])
}

/// 单条射线的最近表面参数 t；`None` = 分类判定该射线没有打到实体。
fn hit(g: &Geometry, o: [f64; 3], d: [f64; 3]) -> Option<f32> {
    let p = cam(g);
    let (t, _n, mask) = geom_intersect(&p, &vec3(o), &vec3(d));
    t.eval().unwrap();
    mask.eval().unwrap();
    if mask.as_slice::<bool>()[0] {
        Some(t.item_cast::<f32>())
    } else {
        None
    }
}

fn contains(g: &Geometry, pts: &[[f64; 3]]) -> Vec<bool> {
    let p = cam(g);
    let flat: Vec<f32> = pts
        .iter()
        .flat_map(|q| [q[0] as f32, q[1] as f32, q[2] as f32])
        .collect();
    let pos = Array::from_slice(&flat, &[pts.len() as i32, 3]);
    let got = geom_contains(&p, &pos);
    got.eval().unwrap();
    got.as_slice::<bool>().to_vec()
}

/// 外盒 − 内盒的薄壳（壁厚 `wall`），世界单位。
fn thin_shell(wall: f64) -> Geometry {
    let outer = Geometry::BoxGeometry(box_geometry(1.0, 1.0, 1.0));
    let inner = Geometry::BoxGeometry(box_geometry(
        1.0 - 2.0 * wall,
        1.0 - 2.0 * wall,
        1.0 - 2.0 * wall,
    ));
    Geometry::CsgGeometry(csg_geometry(CsgOp::Difference, vec![outer, inner]))
}

// ── 1. 尺度依赖：delta = 1e-4 是世界坐标绝对常数 ────────────────────────

/// 壁厚 5e-5 < delta：t±δ 双侧探测跨越整面墙 → 翻转判不出来 → 第一外表面消失。
#[test]
#[ignore = "P1: delta=1e-4 跳过 <delta 的壁厚，见 freeform-robust-boolean.md §1"]
fn thin_shell_outer_face_visible() {
    let g = thin_shell(5e-5);
    let t = hit(&g, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| (t - 1.5).abs() < 1e-3),
        "期望外表面 x=-0.5 (t=1.5)，实际 {:?}",
        t
    );
}

/// **同一设计、两种单位**：壁厚恒为外形尺寸的 5e-5 倍。
/// 大单位（size=1000, wall=0.05）正常；小单位（size=1, wall=5e-5）整体消失 ——
/// 因为 δ=1e-4 是**绝对**容差，生死由"特征尺寸 vs δ"决定，与设计无关。
#[test]
#[ignore = "P1: δ 为绝对容差 → 同一设计换单位即改变行为，见 freeform-robust-boolean.md §1(1)"]
fn unit_scale_invariance() {
    for size in [1.0, 1000.0] {
        let wall = size * 5e-5;
        let g = Geometry::CsgGeometry(csg_geometry(
            CsgOp::Difference,
            vec![
                Geometry::BoxGeometry(box_geometry(size, size, size)),
                Geometry::BoxGeometry(box_geometry(
                    size - 2.0 * wall,
                    size - 2.0 * wall,
                    size - 2.0 * wall,
                )),
            ],
        ));
        let t = hit(&g, [-2.0 * size, 0.0, 0.0], [1.0, 0.0, 0.0]);
        let want = 1.5 * size;
        assert!(
            t.is_some_and(|t| (t as f64 - want).abs() <= 1e-3 * want),
            "外形尺寸 {}（壁厚 {:.3e}）：期望 t={}，实际 {:?}",
            size,
            wall,
            want,
            t
        );
    }
}

/// 正常尺度的对照组：壁厚 0.05 ≫ delta，P0 现状就通过（回归护栏）。
#[test]
fn thick_shell_outer_face_visible() {
    let g = thin_shell(0.05);
    let t = hit(&g, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| (t - 1.5).abs() < 1e-3),
        "期望 t=1.5，实际 {:?}",
        t
    );
}

// ── 2. 掠射：判别式守卫 + delta 双重吃掉近切交点 ────────────────────────

/// y = 1-1e-6 的掠射球：两次交点间距 ~2.8e-3 > 2δ → 现状命中。
#[test]
fn grazing_sphere_coarse_entry() {
    let g = Geometry::SphereGeometry(sphere_geometry(1.0));
    let t = hit(&g, [0.0, 1.0 - 1e-6, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| t > 0.0 && t < 1e-2),
        "期望掠射交点 t≈1.4e-3，实际 {:?}",
        t
    );
}

/// y = 1-1e-10：穿入深度 ~1.4e-5 < delta，翻转探测落在球外 → 漏判。
/// 对渲染不可见，对导出分类是**漏检表面**（P1 的 `Unknown` 必须能报出来）。
#[test]
#[ignore = "P1: delta 探测跨越 <delta 的穿入深度 → 漏判，见 freeform-robust-boolean.md §1(2)(3)"]
fn grazing_sphere_fine_entry() {
    let g = Geometry::SphereGeometry(sphere_geometry(1.0));
    let t = hit(&g, [0.0, 1.0 - 1e-10, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| t > 0.0 && t < 1e-4),
        "期望掠射交点 t≈1.4e-5，实际 {:?}",
        t
    );
}

// ── 3. 相切：双根处的成员关系不翻转（正确行为的护栏） ───────────────────

/// 两球外切于 x=1（球心 0 与 2、半径各 1），从球心向 +x：切点**不是**表面
/// （并集两侧都是实体），最近表面应是第二球的出口 x=3 → t=3。
#[test]
fn tangent_union_from_touch_point() {
    let ball = Geometry::SphereGeometry(sphere_geometry(1.0));
    let moved = Geometry::AffineGeometry(transformed_geometry(
        ball.clone(),
        translator([2.0, 0.0, 0.0]),
        IDENT,
    ));
    let g = Geometry::CsgGeometry(csg_geometry(CsgOp::Union, vec![ball, moved]));
    let t = hit(&g, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| (t - 3.0).abs() < 1e-3),
        "期望切点后最近表面 t=3，实际 {:?}",
        t
    );
}

// ── 4. CSG 正常路径护栏：现有能力不能退化 ──────────────────────────────

/// 法兰式：盒 − 圆柱孔（README `mechanical.cgs` 的同构缩影）。
#[test]
fn csg_box_minus_hole_still_hits() {
    let g = Geometry::CsgGeometry(csg_geometry(
        CsgOp::Difference,
        vec![
            Geometry::BoxGeometry(box_geometry(1.0, 1.0, 1.0)),
            Geometry::CylinderGeometry(cylinder_geometry(0.2, 2.0)),
        ],
    ));
    // 从 -x 轴打，先落在盒外壁 x=-0.5 → t=1.5
    let t = hit(&g, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| (t - 1.5).abs() < 1e-3),
        "期望 t=1.5，实际 {:?}",
        t
    );
    // 避开孔的偏轴射线（y=0.4 > 孔半径 0.2，且在 1x1x1 盒内）同样落在外壁
    let t2 = hit(&g, [-2.0, 0.4, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t2.is_some_and(|t| (t - 1.5).abs() < 1e-3),
        "期望 t=1.5，实际 {:?}",
        t2
    );
    // 孔内为空、孔外为实体
    assert_eq!(
        contains(&g, &[[0.0, 0.4, 0.0], [0.1, 0.0, 0.0]]),
        [true, false],
        "盒内非孔区应为实体，孔内应为空"
    );
}

// ── 5. 网格分类：奇偶计数对非水密网格无意义（P4 换 winding number） ────

/// 立方体抽掉 +x 面 → +x 方向奇偶计数在体内错位。
#[test]
#[ignore = "P4: trimesh_contains 用 +x 奇偶计数，非水密即错，见 freeform-robust-boolean.md §3.4"]
fn open_mesh_inside_still_classifies() {
    let (verts, faces) = extrude(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], 1.0);
    // 抽掉顶点 x≈1 的那一侧面
    let keep: Vec<[i32; 3]> = faces
        .iter()
        .filter(|f| {
            let cx =
                (verts[f[0] as usize][0] + verts[f[1] as usize][0] + verts[f[2] as usize][0]) / 3.0;
            cx < 0.9
        })
        .copied()
        .collect();
    let g = Geometry::TrimeshGeometry(trimesh_geometry(&verts, &keep));
    assert_eq!(
        contains(&g, &[[0.5, 0.5, 0.5], [1.5, 0.5, 0.5]]),
        [true, false],
        "缺面后体内点仍应判为实体"
    );
}
