use cga_core::{
    extrude, AffineGeometry, BoxGeometry, CsgGeometry, CsgOp, CylinderGeometry, Geometry,
    GeometryParams, Mat3, Multivector, SphereGeometry, TrimeshGeometry,
};
use mlx_rs::Array;

use crate::{geom_contains, geom_intersect, geom_to_camera};

const IDENT: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn cam(g: &Geometry) -> GeometryParams {
    geom_to_camera(g, &Multivector::identity())
}

fn vec3(v: [f64; 3]) -> Array {
    Array::from_slice(&[v[0] as f32, v[1] as f32, v[2] as f32], &[1, 3])
}

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

fn thin_shell(wall: f64) -> Geometry {
    let outer = Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0));
    let inner = Geometry::BoxGeometry(BoxGeometry::new(
        1.0 - 2.0 * wall,
        1.0 - 2.0 * wall,
        1.0 - 2.0 * wall,
    ));
    Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Difference, vec![outer, inner]))
}

#[test]
fn thin_shell_outer_face_visible() {
    let g = thin_shell(5e-5);
    let t = hit(&g, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| (t - 1.5).abs() < 1e-3),
        "期望外表面 x=-0.5 (t=1.5)，实际 {:?}",
        t
    );
}

#[test]
fn unit_scale_invariance() {
    for size in [1.0, 1000.0] {
        let wall = size * 5e-5;
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Difference,
            vec![
                Geometry::BoxGeometry(BoxGeometry::new(size, size, size)),
                Geometry::BoxGeometry(BoxGeometry::new(
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

#[test]
fn grazing_sphere_coarse_entry() {
    let g = Geometry::SphereGeometry(SphereGeometry::new(1.0));
    let t = hit(&g, [0.0, 1.0 - 1e-6, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| t > 0.0 && t < 1e-2),
        "期望掠射交点 t≈1.4e-3，实际 {:?}",
        t
    );
}

#[test]
fn csg_grazing_sliver_below_delta() {
    let ball = Geometry::SphereGeometry(SphereGeometry::new(0.01));
    let below = Geometry::PlaneGeometry(cga_core::PlaneGeometry::new([0.0, 1.0, 0.0], -10.0));
    let g = Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Union, vec![ball, below]));
    let t = hit(&g, [0.0, 0.01 - 1e-9, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| t > 0.0 && t < 1e-4),
        "期望掠射薄片交点 t≈5e-6，实际 {:?}",
        t
    );
}

#[test]
fn tangent_union_from_touch_point() {
    let ball = Geometry::SphereGeometry(SphereGeometry::new(1.0));
    let moved = Geometry::AffineGeometry(AffineGeometry::with_motor(
        ball.clone(),
        Multivector::translator([2.0, 0.0, 0.0]),
        IDENT,
    ));
    let g = Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Union, vec![ball, moved]));
    let t = hit(&g, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| (t - 3.0).abs() < 1e-3),
        "期望切点后最近表面 t=3，实际 {:?}",
        t
    );
}

#[test]
fn csg_box_minus_hole_still_hits() {
    let g = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Difference,
        vec![
            Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0)),
            Geometry::CylinderGeometry(CylinderGeometry::new(0.2, 2.0)),
        ],
    ));

    let t = hit(&g, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t.is_some_and(|t| (t - 1.5).abs() < 1e-3),
        "期望 t=1.5，实际 {:?}",
        t
    );

    let t2 = hit(&g, [-2.0, 0.4, 0.0], [1.0, 0.0, 0.0]);
    assert!(
        t2.is_some_and(|t| (t - 1.5).abs() < 1e-3),
        "期望 t=1.5，实际 {:?}",
        t2
    );

    assert_eq!(
        contains(&g, &[[0.0, 0.4, 0.0], [0.1, 0.0, 0.0]]),
        [true, false],
        "盒内非孔区应为实体，孔内应为空"
    );
}

#[test]
fn tangent_sphere_ray_deterministic() {
    let g = Geometry::SphereGeometry(SphereGeometry::new(1.0));
    let t = hit(&g, [0.0, 1.0, 2.0], [0.0, 0.0, -1.0]);
    assert_eq!(t, None, "相切射线穿深 0：分类应判为无穿越");

    assert_eq!(
        crate::certify::QuadRoots::solve(1.0, -4.0, 4.0),
        crate::certify::QuadRoots::Double(2.0)
    );
}

#[test]
fn open_mesh_inside_still_classifies() {
    let (verts, faces) = extrude(&[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], 1.0);

    let keep: Vec<[i32; 3]> = faces
        .iter()
        .filter(|f| {
            let cx =
                (verts[f[0] as usize][0] + verts[f[1] as usize][0] + verts[f[2] as usize][0]) / 3.0;
            cx < 0.9
        })
        .copied()
        .collect();
    let g = Geometry::TrimeshGeometry(TrimeshGeometry::new(&verts, &keep));
    assert_eq!(
        contains(&g, &[[0.5, 0.5, 0.5], [1.5, 0.5, 0.5]]),
        [true, false],
        "缺面后体内点仍应判为实体"
    );
}

#[test]
fn flipped_closed_mesh_still_classifies() {
    // 全局反转向封闭立方体：绕数取 |w|，整体翻转不改变内外判据。
    let v = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ];
    let f = [
        [0, 1, 2],
        [0, 2, 3],
        [4, 6, 5],
        [4, 7, 6],
        [0, 4, 5],
        [0, 5, 1],
        [1, 5, 6],
        [1, 6, 2],
        [2, 6, 7],
        [2, 7, 3],
        [3, 7, 4],
        [3, 4, 0],
    ];
    let g = Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f));
    assert_eq!(
        contains(&g, &[[0.5, 0.5, 0.5], [2.0, 0.5, 0.5]]),
        [true, false],
        "整体反向的封闭网格仍应正确分类"
    );
}
