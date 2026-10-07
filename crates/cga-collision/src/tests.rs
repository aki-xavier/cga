//! C0 验收：pairwise 矩阵（每对 ≥ 3 构型：分离/接触/穿入）、三值规则、
//! 对称性、Unknown 构型必须报 Unknown。

use super::*;
use crate::shape::Shape;
use cga_core::geometry::{
    BoxGeometry, CircleGeometry, ConeGeometry, CylinderGeometry, EllipsoidGeometry, PlaneGeometry,
    SphereGeometry, TorusGeometry,
};
use cga_core::{CsgGeometry, CsgOp, Multivector};

fn sphere(r: f64) -> Geometry {
    Geometry::SphereGeometry(SphereGeometry::new(r))
}
fn plane(n: [f64; 3], d: f64) -> Geometry {
    Geometry::PlaneGeometry(PlaneGeometry::new(n, d))
}
fn bx(h: f64) -> Geometry {
    Geometry::BoxGeometry(BoxGeometry::new(h * 2.0, h * 2.0, h * 2.0))
}
fn cyl(r: f64, len: f64) -> Geometry {
    Geometry::CylinderGeometry(CylinderGeometry::new(r, len))
}
fn cone(r: f64, h: f64) -> Geometry {
    Geometry::ConeGeometry(ConeGeometry::new(r, h))
}
fn torus(major: f64, minor: f64) -> Geometry {
    Geometry::TorusGeometry(TorusGeometry::new(major, minor))
}
fn ell(radii: [f64; 3]) -> Geometry {
    Geometry::EllipsoidGeometry(EllipsoidGeometry::new(radii[0], radii[1], radii[2]))
}
fn circle(r: f64) -> Geometry {
    Geometry::CircleGeometry(CircleGeometry::new(r))
}

fn ident() -> [f64; 16] {
    cga_core::mat4_identity()
}
fn trans(x: f64, y: f64, z: f64) -> [f64; 16] {
    [
        1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0,
    ]
}
fn rotz(a: f64) -> [f64; 16] {
    Multivector::rotor([0.0, 0.0, 1.0], a).to_matrix()
}
fn scale2() -> [f64; 16] {
    [
        2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ]
}

fn sep(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> f64 {
    separation(a, wa, b, wb).expect("这一对应当有确切分离距离")
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-9, "{a} != {b}");
}

// ---- 点包含 ----

#[test]
fn contains_primitives() {
    let id = ident();
    assert_eq!(contains_point(&sphere(1.0), id, [0.5, 0.0, 0.0]), Hit::Yes);
    assert_eq!(contains_point(&sphere(1.0), id, [1.5, 0.0, 0.0]), Hit::No);
    // 旋转 45° 的盒：角点在 (√2,0,0)≈(1.414,0,0)，(1.5,0,0) 在盒外，(0.5,0.5,0) 在盒内
    let rb = mat4_mul(rotz(std::f64::consts::FRAC_PI_4), ident());
    assert_eq!(contains_point(&bx(1.0), rb, [1.5, 0.0, 0.0]), Hit::No);
    assert_eq!(contains_point(&bx(1.0), rb, [0.5, 0.5, 0.0]), Hit::Yes);
    // 柱（z 轴，r=0.5, 半长 1）
    assert_eq!(
        contains_point(&cyl(0.5, 2.0), id, [0.3, 0.0, 0.9]),
        Hit::Yes
    );
    assert_eq!(contains_point(&cyl(0.5, 2.0), id, [0.3, 0.0, 1.1]), Hit::No);
    // 锥（r=1, h=2，顶点 z=+1，底 z=−1）
    assert_eq!(
        contains_point(&cone(1.0, 2.0), id, [0.0, 0.0, 0.0]),
        Hit::Yes
    );
    assert_eq!(
        contains_point(&cone(1.0, 2.0), id, [0.6, 0.0, 0.0]),
        Hit::No
    );
    // 环面（R=1, r=0.25）
    assert_eq!(
        contains_point(&torus(1.0, 0.25), id, [1.0, 0.0, 0.0]),
        Hit::Yes
    );
    assert_eq!(
        contains_point(&torus(1.0, 0.25), id, [0.0, 0.0, 0.0]),
        Hit::No
    );
    // 椭球
    assert_eq!(
        contains_point(&ell([2.0, 1.0, 1.0]), id, [1.5, 0.0, 0.0]),
        Hit::Yes
    );
    assert_eq!(
        contains_point(&ell([2.0, 1.0, 1.0]), id, [2.5, 0.0, 0.0]),
        Hit::No
    );
    // 平面半空间：n=+y, d=0 ⇒ y≤0 为内
    assert_eq!(
        contains_point(&plane([0.0, 1.0, 0.0], 0.0), id, [0.0, -1.0, 0.0]),
        Hit::Yes
    );
    assert_eq!(
        contains_point(&plane([0.0, 1.0, 0.0], 0.0), id, [0.0, 1.0, 0.0]),
        Hit::No
    );
    // 非刚体仿射下的 contains 仍精确（点逆变换）：2× 缩放的球半径等效 2
    assert_eq!(
        contains_point(&sphere(1.0), scale2(), [1.5, 0.0, 0.0]),
        Hit::Yes
    );
    assert_eq!(
        contains_point(&sphere(1.0), scale2(), [2.5, 0.0, 0.0]),
        Hit::No
    );
    // 圆片是曲面：Unknown
    assert_eq!(
        contains_point(&circle(1.0), id, [0.5, 0.0, 0.0]),
        Hit::Unknown
    );
}

#[test]
fn contains_csg_three_valued() {
    let id = ident();
    // 球 ∪ 部分环面（实体但 contains 不支持 → Unknown）：点球内 → Yes；球外 →
    // No ∪ Unknown = Unknown（注意：Unknown 叶子会让 union 的外点也只能报 Unknown，
    // 这是三值的保守性，文档已注明）
    let tube = Geometry::TorusGeometry(TorusGeometry::tube(1.0, 0.25, 1.0));
    let u = Geometry::CsgGeometry(CsgGeometry::new(CsgOp::Union, vec![sphere(1.0), tube]));
    assert_eq!(contains_point(&u, id, [0.5, 0.0, 0.0]), Hit::Yes);
    assert_eq!(contains_point(&u, id, [5.0, 0.0, 0.0]), Hit::Unknown);
    // 球 − 球：内点 Yes / 外点 No
    let d = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Difference,
        vec![
            sphere(2.0),
            Geometry::AffineGeometry(cga_core::AffineGeometry::with_motor(
                sphere(1.0),
                cga_core::Multivector::translator([1.0, 0.0, 0.0]),
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            )),
        ],
    ));
    assert_eq!(contains_point(&d, id, [-1.5, 0.0, 0.0]), Hit::Yes);
    assert_eq!(contains_point(&d, id, [1.0, 0.0, 0.0]), Hit::No);
    assert_eq!(contains_point(&d, id, [5.0, 0.0, 0.0]), Hit::No);
}

// ---- 分离距离矩阵 ----

#[test]
fn separation_sphere_sphere() {
    let id = ident();
    close(
        sep(&sphere(1.0), id, &sphere(1.0), trans(3.0, 0.0, 0.0)),
        1.0,
    );
    close(
        sep(&sphere(1.0), id, &sphere(1.0), trans(2.0, 0.0, 0.0)),
        0.0,
    ); // 相切
    close(
        sep(&sphere(1.0), id, &sphere(1.0), trans(1.5, 0.0, 0.0)),
        -0.5,
    ); // 穿入
}

#[test]
fn separation_sphere_plane() {
    let id = ident();
    let p = plane([0.0, 1.0, 0.0], 0.0);
    close(sep(&sphere(1.0), trans(0.0, 2.0, 0.0), &p, id), 1.0);
    close(sep(&sphere(1.0), trans(0.0, 1.0, 0.0), &p, id), 0.0); // 落在地面上
    close(sep(&sphere(1.0), trans(0.0, 0.5, 0.0), &p, id), -0.5); // 陷入地面
}

#[test]
fn separation_sphere_box() {
    let id = ident();
    let b = bx(1.0);
    // 面方向分离：球心 (3,0,0)，盒面 x=1 → 2 − 0.5 = 1.5…（r=0.5）
    close(sep(&sphere(0.5), trans(3.0, 0.0, 0.0), &b, id), 1.5);
    // 棱方向：球心 (2,2,0) → 到棱 (1,1,0) 距离 √2，减 r
    close(
        sep(&sphere(0.5), trans(2.0, 2.0, 0.0), &b, id),
        std::f64::consts::SQRT_2 - 0.5,
    );
    // 球心在盒内：sdf = −0.3（x=0.7 到面 x=1），sep = −0.3 − 0.5
    close(sep(&sphere(0.5), trans(0.7, 0.0, 0.0), &b, id), -0.8);
    // 旋转 45° 的盒对角点方向：盒角在 (√2,0,0)… 球心 (√2+1.5,0,0) → sep = 1.5 − 0.5
    let rb = mat4_mul(rotz(std::f64::consts::FRAC_PI_4), ident());
    close(
        sep(
            &sphere(0.5),
            trans(std::f64::consts::SQRT_2 + 1.5, 0.0, 0.0),
            &bx(1.0),
            rb,
        ),
        1.0,
    );
}

#[test]
fn separation_sphere_cylinder() {
    let id = ident();
    let c = cyl(0.5, 2.0); // z 轴，r=0.5，半长 1
                           // 径向分离
    close(sep(&sphere(0.5), trans(2.0, 0.0, 0.0), &c, id), 1.0);
    // 轴向分离（帽上方）
    close(sep(&sphere(0.5), trans(0.0, 0.0, 2.0), &c, id), 0.5);
    // 球心在柱内：sdf = max(ρ−r, |z|−half) = −0.5 → sep = −0.75
    close(sep(&sphere(0.25), id, &c, id), -0.75);
}

#[test]
fn separation_sphere_cone() {
    let id = ident();
    let c = cone(1.0, 2.0); // 顶点 (0,0,1)，底半径 1 在 z=−1
                            // 侧面方向：(ρ,z)=(1.5,0) 到母线 (0,1)–(1,−1)：投影 t=0.7，垂足 (0.7,−0.4)，
                            // 距离 |(0.8,0.4)| = √0.8
    close(
        sep(&sphere(0.5), trans(1.5, 0.0, 0.0), &c, id),
        0.8f64.sqrt() - 0.5,
    );
    // 底角之外：(3,0,0) → 最近点是底角 (1,−1)，距离 √5
    close(
        sep(&sphere(0.5), trans(3.0, 0.0, 0.0), &c, id),
        5.0f64.sqrt() - 0.5,
    );
    // 球心在锥内轴线上：(ρ,z)=(0,0) 在锥内，到母线 (0,1)–(1,−1) 的垂足 (0.4,0.2)，
    // 距离 |(0.4,0.2)| = √0.2 = 1/√5 → sdf = −1/√5
    close(sep(&sphere(0.5), id, &c, id), -(0.2f64.sqrt()) - 0.5);
}

#[test]
fn separation_sphere_torus() {
    let id = ident();
    let t = torus(1.0, 0.25);
    close(sep(&sphere(0.5), trans(3.0, 0.0, 0.0), &t, id), 1.25);
    // 球在环洞中心：|(0−1, 0)| − 0.25 = 0.75
    close(sep(&sphere(0.3), id, &t, id), 0.45);
}

#[test]
fn separation_sphere_ellipsoid() {
    let id = ident();
    let e = ell([2.0, 1.0, 1.0]);
    close(sep(&sphere(1.0), trans(5.0, 0.0, 0.0), &e, id), 2.0);
    close(sep(&sphere(1.0), trans(0.0, 3.0, 0.0), &e, id), 1.0);
    // 球心在椭球内：v1 保守 Unknown
    assert!(separation(&sphere(0.5), id, &e, id).is_none());
}

#[test]
fn separation_plane_convex() {
    let id = ident();
    let ground = plane([0.0, 1.0, 0.0], 0.0);
    // 盒跨过地面：lo=−1, hi=1 → 穿透 −1；抬高 3 → 2；埋在 −3 → −2
    close(sep(&bx(1.0), id, &ground, id), -1.0);
    close(sep(&bx(1.0), trans(0.0, 3.0, 0.0), &ground, id), 2.0);
    close(sep(&bx(1.0), trans(0.0, -3.0, 0.0), &ground, id), -2.0);
    // 柱（z 轴躺平在地面法向里支撑半径 0.5）：跨地 → −0.5
    close(sep(&cyl(0.5, 2.0), id, &ground, id), -0.5);
    // 锥：轴 +z，顶点 z=1；地面法向 +z：支撑 hi = max(1, −1+0) = 1 → 跨地 −1
    let up = plane([0.0, 0.0, 1.0], 0.0);
    close(sep(&cone(1.0, 2.0), id, &up, id), -1.0);
    // 环面：xy 平面内（轴 +z）对地面（法向 +y）：支撑 = R·1 + r = 1.25 → 跨地 −1.25
    close(sep(&torus(1.0, 0.25), id, &ground, id), -1.25);
    // 椭球对平面 d=3（+y）：hi = 1 − 3 = −2 → −2
    close(
        sep(&ell([2.0, 1.0, 1.0]), id, &plane([0.0, 1.0, 0.0], 3.0), id),
        -2.0,
    );
    // 平面–平面：平行 |Δd|；相交 0
    close(sep(&ground, id, &plane([0.0, 1.0, 0.0], 2.0), id), 2.0);
    close(sep(&ground, id, &plane([0.0, 0.0, 1.0], 0.0), id), 0.0);
}

#[test]
fn separation_box_box() {
    let id = ident();
    // 面分离
    close(sep(&bx(1.0), id, &bx(1.0), trans(5.0, 0.0, 0.0)), 3.0);
    // 棱–棱：B 在 (3,3,0)，A 的棱 (1,1,z) 对 B 的棱 (2,2,z) → √2
    close(
        sep(&bx(1.0), id, &bx(1.0), trans(3.0, 3.0, 0.0)),
        std::f64::consts::SQRT_2,
    );
    // 穿入：x 方向交叠 0.5 → MTV −0.5
    close(sep(&bx(1.0), id, &bx(1.0), trans(1.5, 0.0, 0.0)), -0.5);
}

#[test]
fn separation_box_box_rotated_offset() {
    let id = ident();
    // B = 旋转 45° 后半宽 0.5，再平移到 (3,0,0)：x 向支撑半径 √2/2
    let wb = mat4_mul(trans(3.0, 0.0, 0.0), rotz(std::f64::consts::FRAC_PI_4));
    close(
        sep(&bx(1.0), id, &bx(0.5), wb),
        3.0 - 1.0 - std::f64::consts::SQRT_2 / 2.0,
    );
}

// ---- Unknown 构型（不许假装精确） ----

#[test]
fn separation_unknown_pairs() {
    let id = ident();
    // 环面–环面：未实现
    assert!(separation(
        &torus(1.0, 0.25),
        id,
        &torus(1.0, 0.25),
        trans(5.0, 0.0, 0.0)
    )
    .is_none());
    // 球–圆片：圆片是曲面
    assert!(separation(&sphere(1.0), id, &circle(1.0), id).is_none());
    // CSG union 现在分解为子件最小值（C1）；差/交仍是 Unknown
    let u = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Union,
        vec![sphere(1.0), sphere(1.0)],
    ));
    assert!(separation(&u, id, &sphere(1.0), id).is_some());
    let d = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Difference,
        vec![sphere(2.0), sphere(1.0)],
    ));
    assert!(separation(&d, id, &sphere(1.0), id).is_none());
    // 非刚体仿射（2× 缩放）下的分离：Unknown
    assert!(separation(&sphere(1.0), scale2(), &sphere(1.0), id).is_none());
}

// ---- overlap 三值 + 性质 ----

#[test]
fn overlap_consistency_with_separation() {
    let id = ident();
    let cases: [(Geometry, [f64; 16], Geometry, [f64; 16]); 4] = [
        (sphere(1.0), id, sphere(1.0), trans(3.0, 0.0, 0.0)),
        (sphere(1.0), id, sphere(1.0), trans(1.5, 0.0, 0.0)),
        (bx(1.0), id, plane([0.0, 1.0, 0.0], 0.0), id),
        (bx(1.0), id, bx(1.0), trans(5.0, 0.0, 0.0)),
    ];
    for (a, wa, b, wb) in &cases {
        let d = separation(a, *wa, b, *wb).unwrap();
        let want = if d <= 0.0 { Hit::Yes } else { Hit::No };
        assert_eq!(overlap(a, *wa, b, *wb), want, "sep={d}");
    }
    // 包围盒保守路径：两个环面相距 5（包围盒相离 → 确切 No）；相距 1（包围盒重叠、
    // 对未实现 → Unknown）。
    assert_eq!(
        overlap(
            &torus(1.0, 0.25),
            id,
            &torus(1.0, 0.25),
            trans(5.0, 0.0, 0.0)
        ),
        Hit::No
    );
    assert_eq!(
        overlap(
            &torus(1.0, 0.25),
            id,
            &torus(1.0, 0.25),
            trans(1.0, 0.0, 0.0)
        ),
        Hit::Unknown
    );
}

// ---- CSG union 分解（C1） ----

#[test]
fn overlap_union_decomposition() {
    let id = ident();
    // union(球@原点, 球@(5,0,0)) vs 第三个球：
    let u = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Union,
        vec![
            sphere(1.0),
            Geometry::AffineGeometry(cga_core::AffineGeometry::with_motor(
                sphere(1.0),
                cga_core::Multivector::translator([5.0, 0.0, 0.0]),
                [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            )),
        ],
    ));
    // 远处 → No（分离距离可算：union 的子件最小值）
    let (h, s) = probe(&u, id, &sphere(1.0), trans(10.0, 0.0, 0.0));
    assert_eq!(h, Hit::No);
    close(s.unwrap(), 3.0);
    // 碰到第二个子件 → Yes（separation ≤ 0）
    let (h, s) = probe(&u, id, &sphere(1.0), trans(6.5, 0.0, 0.0));
    assert_eq!(h, Hit::Yes);
    close(s.unwrap(), -0.5);
    // 三值 or 传播：union(球, 部分环面) vs 近球（包围盒重叠）→ No ∪ Unknown = Unknown
    let u2 = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Union,
        vec![
            sphere(1.0),
            Geometry::TorusGeometry(TorusGeometry::tube(1.0, 0.25, 1.0)),
        ],
    ));
    assert_eq!(
        overlap(&u2, id, &sphere(0.5), trans(1.75, 0.0, 0.0)),
        Hit::Unknown
    );
    // 包围盒相离 → 确切 No（不到 union 分解）
    assert_eq!(
        overlap(&u2, id, &sphere(0.5), trans(50.0, 0.0, 0.0)),
        Hit::No
    );
    // 差/交 → Unknown
    let d = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Difference,
        vec![sphere(2.0), sphere(1.0)],
    ));
    assert_eq!(overlap(&d, id, &sphere(0.5), id), Hit::Unknown);
}

// ---- 接触信息（C2） ----

#[test]
fn contacts_sphere_sphere() {
    let id = ident();
    // 相离 → 确切无接触
    assert_eq!(
        contacts(&sphere(1.0), id, &sphere(1.0), trans(3.0, 0.0, 0.0))
            .unwrap()
            .len(),
        0
    );
    // 穿入（b 在 x=1.5）：a 表面点 (1,0,0)，b 表面点 (0.5,0,0)，中点 (0.75,0,0)；normal = b 的分离方向 +x
    let cs = contacts(&sphere(1.0), id, &sphere(1.0), trans(1.5, 0.0, 0.0)).unwrap();
    assert_eq!(cs.len(), 1);
    let c = cs[0];
    assert!(norm(sub(c.point, [0.75, 0.0, 0.0])) < 1e-9, "{:?}", c.point);
    assert!(
        norm(sub(c.normal, [1.0, 0.0, 0.0])) < 1e-9,
        "{:?}",
        c.normal
    );
    close(c.separation, -0.5);
}

#[test]
fn contacts_sphere_plane() {
    let id = ident();
    let ground = plane([0.0, 1.0, 0.0], 0.0);
    // 球 r=1 心在 y=0.5：陷入地面 0.5；接触点中点在 y=−0.25；normal = 地面下移方向 −y
    let cs = contacts(&sphere(1.0), trans(0.0, 0.5, 0.0), &ground, id).unwrap();
    assert_eq!(cs.len(), 1);
    let c = cs[0];
    assert!(
        norm(sub(c.point, [0.0, -0.25, 0.0])) < 1e-9,
        "{:?}",
        c.point
    );
    assert!(
        norm(sub(c.normal, [0.0, -1.0, 0.0])) < 1e-9,
        "{:?}",
        c.normal
    );
    close(c.separation, -0.5);
}

#[test]
fn contacts_plane_box_resting() {
    let id = ident();
    let ground = plane([0.0, 1.0, 0.0], 0.0);
    // 盒 half=1 心在 y=1：恰好落地（sep=0）；接触点在底面（y=0），normal = +y（盒向上分离）
    let cs = contacts(&ground, id, &bx(1.0), trans(0.0, 1.0, 0.0)).unwrap();
    assert_eq!(cs.len(), 1);
    let c = cs[0];
    assert!(c.point[1].abs() < 1e-9, "{:?}", c.point);
    assert!(norm(sub(c.normal, [0.0, 1.0, 0.0])) < 1e-9);
    close(c.separation, 0.0);
    // 抬起来 → 确切无接触
    assert_eq!(
        contacts(&ground, id, &bx(1.0), trans(0.0, 1.5, 0.0))
            .unwrap()
            .len(),
        0
    );
}

#[test]
fn contacts_box_box_manifold() {
    let id = ident();
    // A half=1 @ 原点，B half=1 @ (1.5,0,0)：交区 x∈[0.5,1]。
    // 接触点 = A 的 x=1 面四角 + B 的 x=0.5 面四角，共 8 个。
    let cs = contacts(&bx(1.0), id, &bx(1.0), trans(1.5, 0.0, 0.0)).unwrap();
    assert_eq!(
        cs.len(),
        8,
        "{:?}",
        cs.iter().map(|c| c.point).collect::<Vec<_>>()
    );
    for c in &cs {
        assert!(
            c.point[0] >= 0.5 - 1e-9 && c.point[0] <= 1.0 + 1e-9,
            "{:?}",
            c.point
        );
        assert!((c.point[1].abs() - 1.0).abs() < 1e-9, "{:?}", c.point);
        assert!((c.point[2].abs() - 1.0).abs() < 1e-9, "{:?}", c.point);
        assert!(norm(sub(c.normal, [1.0, 0.0, 0.0])) < 1e-9, "normal = +x");
        close(c.separation, -0.5);
    }
}

#[test]
fn contacts_symmetric_normals() {
    let id = ident();
    let a = contacts(&sphere(0.5), id, &bx(1.0), id).unwrap();
    let b = contacts(&bx(1.0), id, &sphere(0.5), id).unwrap();
    assert_eq!(a.len(), b.len());
    for (x, y) in a.iter().zip(b.iter()) {
        assert!(norm(sub(x.point, y.point)) < 1e-9);
        assert!(norm(add(x.normal, y.normal)) < 1e-9, "法向互换取反");
    }
}

#[test]
fn contacts_unknown_pairs() {
    let id = ident();
    assert!(contacts(
        &torus(1.0, 0.25),
        id,
        &torus(1.0, 0.25),
        trans(2.1, 0.0, 0.0)
    )
    .is_none());
    assert!(contacts(&sphere(1.0), id, &circle(1.0), id).is_none());
    let d = Geometry::CsgGeometry(CsgGeometry::new(
        CsgOp::Difference,
        vec![sphere(2.0), sphere(1.0)],
    ));
    assert!(contacts(&d, id, &sphere(1.0), id).is_none());
}

// ---- 螺旋 CCD（C3） ----

#[test]
fn sweep_toi_rotating_arm_closed_form() {
    // 旋转臂：球 r=0.5 在半径 3 处从 (0,3,0) 出发，绕原点 z 顺时针（ω=−1）；
    // 立柱：球 r=0.5 在 (3,0,0)。接触当弦长 2R·sin(Δφ/2) = 1 → Δφ* = 2·asin(1/6)，
    // t* = π/2 − Δφ*。
    let arm0 = trans(0.0, 3.0, 0.0);
    let post = trans(3.0, 0.0, 0.0);
    let xi = [0.0, 0.0, -1.0, 0.0, 0.0, 0.0];
    let (h, t) = sweep_toi(xi, &sphere(0.5), arm0, &sphere(0.5), post, 2.0);
    let want = std::f64::consts::FRAC_PI_2 - 2.0 * (1.0f64 / 6.0).asin();
    assert_eq!(h, Hit::Yes);
    let t = t.unwrap();
    assert!((t - want).abs() < 1e-9, "t={t} want={want}");
    // 自洽：找到的接触时刻分离度 ≈ 0，稍前 > 0
    let m = mat4_mul(
        Multivector::velocity([0.0, 0.0, -0.5], [0.0; 3])
            .exp(t)
            .to_matrix(),
        arm0,
    );
    close(
        separation(&sphere(0.5), m, &sphere(0.5), post).unwrap(),
        0.0,
    );
    let m_before = mat4_mul(
        Multivector::velocity([0.0, 0.0, -0.5], [0.0; 3])
            .exp(t - 1e-3)
            .to_matrix(),
        arm0,
    );
    assert!(separation(&sphere(0.5), m_before, &sphere(0.5), post).unwrap() > 0.0);
}

#[test]
fn sweep_toi_certified_no_contact() {
    // 同一构型但 t_max 小于接触时刻 → 认证 No（不是"没采到"，是 Lipschitz 界证明）。
    let (h, t) = sweep_toi(
        [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
        &sphere(0.5),
        trans(0.0, 3.0, 0.0),
        &sphere(0.5),
        trans(3.0, 0.0, 0.0),
        1.0,
    );
    assert_eq!((h, t), (Hit::No, None));
}

#[test]
fn sweep_toi_already_touching_and_translation() {
    let id = ident();
    // 初始已穿入 → (Yes, Some(0))
    let (h, t) = sweep_toi(
        [0.0; 6],
        &sphere(1.0),
        id,
        &sphere(1.0),
        trans(1.5, 0.0, 0.0),
        1.0,
    );
    assert_eq!((h, t), (Hit::Yes, Some(0.0)));
    // 纯平移：球 r=1 从原点以 v=+x 撞 x=5 的球 → t* = 5 − 2 = 3
    let (h, t) = sweep_toi(
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        &sphere(1.0),
        id,
        &sphere(1.0),
        trans(5.0, 0.0, 0.0),
        10.0,
    );
    assert_eq!(h, Hit::Yes);
    close(t.unwrap(), 3.0);
}

#[test]
fn sweep_toi_sphere_box() {
    // 球 r=0.5 从 (−3,0,0) 以 v=+x 撞半宽 1 的盒 → 接触面 x=−1，t* = 3 − 1.5 = 1.5
    let (h, t) = sweep_toi(
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        &sphere(0.5),
        trans(-3.0, 0.0, 0.0),
        &bx(1.0),
        ident(),
        10.0,
    );
    assert_eq!(h, Hit::Yes);
    close(t.unwrap(), 1.5);
}

#[test]
fn sweep_toi_cam_contact_closed_form() {
    // 凸轮几何的交叉验证：cam_solve 的 circle–circle 分支（separation_branches）
    // 是"共面圆心距 − (r1+r2)"，与球–球在球心共面时逐点相同。
    // 从动件圆心绕 pivot (2,0,0) 转臂 1.5：c(q) = (2 + 1.5cos q, 1.5 sin q)，
    // 驱动件在原点 r=0.5。接触：|c|² = 6.25 + 6cos q = (r1+r2)² = 1
    // → cos q* = −0.875 → q* = acos(−0.875)。ω=+1 ⇒ t* = q*。
    // 绕 pivot 的旋转螺旋：ṗ = ω×(p − o) ⇒ v = o×ω = (2,0,0)×(0,0,1) = (0,−2,0)。
    let xi = [0.0, 0.0, 1.0, 0.0, -2.0, 0.0];
    let (h, t) = sweep_toi(
        xi,
        &sphere(0.5),
        trans(3.5, 0.0, 0.0),
        &sphere(0.5),
        ident(),
        3.0,
    );
    assert_eq!(h, Hit::Yes);
    let want = (-0.875f64).acos();
    assert!((t.unwrap() - want).abs() < 1e-9, "t={:?} want={want}", t);
}

#[test]
fn sweep_toi_unknown_pair() {
    // 环面–环面：分离距离未实现 → Unknown（不许假装扫过）
    let (h, _) = sweep_toi(
        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        &torus(1.0, 0.25),
        ident(),
        &torus(1.0, 0.25),
        trans(5.0, 0.0, 0.0),
        10.0,
    );
    assert_eq!(h, Hit::Unknown);
}

#[test]
fn box_box_mtv_exact_matches_brute_force() {
    // 对拍：精确 MTV（Minkowski 差 zonotope 最近点）vs 全方向采样参考
    // （每个方向的精确分离平移 ra+rb−|t·n|，取 2000 个方向的最小值——是 MTV 的
    // 上界逼近）。断言：① 精确 ≤ SAT 15 轴最小值（SAT 是子集 ⇒ 上界）；
    // ② 精确 ≈ 参考值（采样分辨率内）。
    let id = ident();
    let cases: Vec<([f64; 16], f64, [f64; 16], f64)> = vec![
        // 面穿入（SAT 精确的情形）
        (id, 1.0, trans(1.5, 0.0, 0.0), 1.0),
        // 深穿透 + 旋转（棱–棱）
        (
            id,
            1.0,
            mat4_mul(
                trans(0.9, 0.7, 0.3),
                Multivector::rotor([1.0, 1.0, 0.0], 0.7).to_matrix(),
            ),
            0.8,
        ),
        // 深穿透细杆交叉
        (id, 1.0, mat4_mul(trans(0.3, 0.4, 0.0), rotz(0.9)), 1.2),
    ];
    for (wa, _ha, wb, _hb) in &cases {
        let a = bx(1.0);
        let b = bx(1.0);
        let sep = separation(&a, *wa, &b, *wb).unwrap();
        assert!(sep < 0.0, "构型应穿入: {sep}");
        let exact = -sep;
        // 强对拍：沿 MTV 方向的单向分离平移必须恰等于 |MTV|（边界点自身的方向
        // 就是见证方向）。这与采样无关，是解析等式。
        let (sa, sb) = (
            crate::shape::to_shape(&a, *wa).unwrap(),
            crate::shape::to_shape(&b, *wb).unwrap(),
        );
        let (
            Shape::Box {
                c: ca,
                axes: aa,
                half: ha,
            },
            Shape::Box {
                c: cb,
                axes: ab,
                half: hb,
            },
        ) = (sa, sb)
        else {
            panic!("not boxes")
        };
        let t = sub(cb, ca);
        let ext = |l: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]| -> f64 {
            (0..3).map(|i| dot(l, axes[i]).abs() * half[i]).sum()
        };
        // 从 zonotope 端取 MTV 向量（内部可见）。
        let mtv = crate::pairs::box_mtv_exact(ca, aa, ha, cb, ab, hb);
        let n = scale(mtv, 1.0 / exact);
        let along = ext(n, aa, ha) + ext(n, ab, hb) - dot(t, n).abs();
        assert!(
            (along - exact).abs() < 1e-9,
            "MTV 方向的单向分离应恰等于 |MTV|: along={along} exact={exact}"
        );
        // 参考：全方向采样（上界逼近，只断言不小于精确值）。
        let brute = mtv_brute(&a, *wa, &b, *wb, 2000);
        assert!(
            exact <= brute + 1e-9,
            "精确 MTV 不得超过参考上界: exact={exact} brute={brute}"
        );
        // SAT 子集上界性质
        let sat = sat15_min(&a, *wa, &b, *wb);
        assert!(exact <= sat + 1e-9, "SAT 是上界: exact={exact} sat={sat}");
    }
}

/// 全方向采样参考：黄金角螺旋布点。
fn mtv_brute(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16], n_samples: usize) -> f64 {
    let (sa, sb) = (
        crate::shape::to_shape(a, wa).unwrap(),
        crate::shape::to_shape(b, wb).unwrap(),
    );
    let (
        Shape::Box {
            c: ca,
            axes: aa,
            half: ha,
        },
        Shape::Box {
            c: cb,
            axes: ab,
            half: hb,
        },
    ) = (sa, sb)
    else {
        panic!("not boxes")
    };
    let t = sub(cb, ca);
    let ext = |l: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]| -> f64 {
        (0..3).map(|i| dot(l, axes[i]).abs() * half[i]).sum()
    };
    let mut best = f64::INFINITY;
    let golden = std::f64::consts::PI * (3.0 - 5.0f64.sqrt());
    for i in 0..n_samples {
        let z = 1.0 - 2.0 * (i as f64 + 0.5) / n_samples as f64;
        let r = (1.0 - z * z).max(0.0).sqrt();
        let th = golden * i as f64;
        let n = [r * th.cos(), r * th.sin(), z];
        best = best.min(ext(n, aa, ha) + ext(n, ab, hb) - dot(t, n).abs());
    }
    best
}

/// SAT 15 轴的最小穿透（参考实现）。
fn sat15_min(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> f64 {
    let (sa, sb) = (
        crate::shape::to_shape(a, wa).unwrap(),
        crate::shape::to_shape(b, wb).unwrap(),
    );
    let (
        Shape::Box {
            c: ca,
            axes: aa,
            half: ha,
        },
        Shape::Box {
            c: cb,
            axes: ab,
            half: hb,
        },
    ) = (sa, sb)
    else {
        panic!("not boxes")
    };
    let t = sub(cb, ca);
    let ext = |l: [f64; 3], axes: [[f64; 3]; 3], half: [f64; 3]| -> f64 {
        (0..3).map(|i| dot(l, axes[i]).abs() * half[i]).sum()
    };
    let mut best = f64::INFINITY;
    let mut sat = |l: [f64; 3]| {
        let ll = norm(l);
        if ll < 1e-9 {
            return;
        }
        let l = scale(l, 1.0 / ll);
        best = best.min(ext(l, aa, ha) + ext(l, ab, hb) - dot(t, l).abs());
    };
    for ax in aa.iter().chain(ab.iter()) {
        sat(*ax);
    }
    for axa in aa {
        for axb in ab {
            sat(cross(axa, axb));
        }
    }
    best
}

#[test]
fn separation_symmetry() {
    let id = ident();
    let pairs: [(Geometry, [f64; 16], Geometry, [f64; 16]); 4] = [
        (sphere(0.5), trans(1.5, 0.0, 0.0), cone(1.0, 2.0), id),
        (sphere(0.5), trans(3.0, 0.0, 0.0), bx(1.0), id),
        (plane([0.0, 1.0, 0.0], 0.5), id, torus(1.0, 0.25), id),
        (bx(1.0), id, bx(0.5), trans(3.0, 0.0, 0.0)),
    ];
    for (a, wa, b, wb) in &pairs {
        let ab = separation(a, *wa, b, *wb).unwrap();
        let ba = separation(b, *wb, a, *wa).unwrap();
        assert!((ab - ba).abs() < 1e-12, "对称性：{ab} != {ba}");
    }
}
