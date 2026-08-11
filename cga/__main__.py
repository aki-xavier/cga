"""CGA 包自检: python -m cga

验证核心代数与原语/versor 的正确性 (OOP API)。约定:
点/点对/线为直接形式, 关联判据 p.op(X) = 0; 平面/球/圆为对偶
形式, 关联判据 p.ip(X) = 0; versor 作用 M.apply(obj);
meet 接受直接形式输入。
"""

import math
from pathlib import Path

import mlx.core as mx

from cga import (
    Circle,
    Cylinder,
    Line,
    Motor,
    Multivector,
    Plane,
    Point,
    RenderPrimitive,
    Sphere,
    render_scene,
)
from cga.robot import RobotError, load_robot
from cga.urdf_io import crdf_to_urdf, urdf_to_crdf

MODEL_CRDF = Path(__file__).resolve().parent.parent / "models" / "z1_arm.crdf.yaml"

_ok = 0


def check(name: str, cond: bool) -> None:
    """断言一条检查并计数。"""
    global _ok
    assert cond, f"FAIL: {name}"
    _ok += 1
    print(f"  ok  {name}")


def close(a: float, b: float, tol: float = 1e-4) -> bool:
    """|a−b| < tol。"""
    return abs(float(a) - float(b)) < tol


def vmax(mv: Multivector) -> float:
    """分量的最大绝对值。"""
    return float(mx.abs(mv.values).max().item())


def diff_max(a: Multivector, b: Multivector) -> float:
    """两 multivector 逐分量差的最大绝对值。"""
    return float(mx.abs(a.values - b.values).max().item())


def main() -> None:
    """全部自检: 代数 / 图元 / versor / exp-log / 距离。"""
    # null 性 + 距离
    p1, p2 = Point(0, 0, 0), Point(1, 0, 0)
    check("point is null", close(p1.gp(p1).values[0], 0))
    check("dist_point_point", close(p1.dist(Point(3, 4, 0)), 5.0))

    # 关联判据: 线 (直接形式) op(p, X) = 0
    L = Line(p1, p2)
    check("point on line", close(vmax(Point(2, 0, 0).op(L)), 0))
    check("point off line", vmax(Point(0, 1, 0).op(L)) > 1e-3)

    # 关联判据: 平面/球/圆 (对偶形式) ip(p, X) = 0
    pi = Plane((0, 0, 1), 2.0)  # z = 2 平面
    check("point on plane", close(vmax(Point(0.3, -0.7, 2).ip(pi)), 0))
    check("point off plane", vmax(Point(0, 0, 0).ip(pi)) > 1e-3)

    s = Sphere((1, 2, 3), 2.0)
    check("point on sphere", close(vmax(Point(3, 2, 3).ip(s)), 0))
    check("point off sphere", vmax(Point(0, 0, 0).ip(s)) > 1e-3)

    c = Circle((0, 0, 0), 1.0, (0, 0, 1))
    check("point on circle", close(vmax(Point(0, 1, 0).ip(c)), 0))
    check("point off circle", vmax(Point(0, 0, 1).ip(c)) > 1e-3)
    # 非单位法向: d 须按单位法向计算 (回归: Plane 归一化 n 但不缩放 d)
    cnu = Circle((1, 2, 3), 2.0, (0, 0, 2))
    check("circle non-unit normal", close(vmax(Point(3, 2, 3).ip(cnu)), 0))

    # 退化输入守卫
    try:
        Plane((0, 0, 0), 1.0)
        raise AssertionError("FAIL: degenerate plane accepted")
    except ValueError:
        check("degenerate plane raises", True)
    try:
        Circle((0, 0, 0), 1.0, (0, 0, 0))
        raise AssertionError("FAIL: degenerate circle accepted")
    except ValueError:
        check("degenerate circle raises", True)
    try:
        Multivector.bivector([1.0, 2.0])
        raise AssertionError("FAIL: short bivector accepted")
    except ValueError:
        check("bivector length check", True)

    # 距离函数 (float64 欧氏公式)
    check("dist_point_plane", close(Point(0, 0, 5).dist(pi), 3.0))
    check("dist_point_plane (plane 侧)", close(pi.dist(Point(0, 0, 5)), 3.0))
    check("dist_point_sphere on", close(Point(3, 2, 3).dist(s), 0))
    check("dist_point_sphere out", Point(5, 2, 3).dist(s) > 0)
    check("dist_point_sphere in", Point(1, 2, 3).dist(s) < 0)

    # meet (直接形式输入; 对偶原语先过 dual()):
    # 两平面交线 / 线球交点对
    pi2 = Plane((0, 1, 0), 1.0)  # y = 1 平面
    Lm = pi.dual().meet(pi2.dual())  # 交线: y=1, z=2, 沿 x 方向
    check(
        "meet(plane,plane) = line",
        close(vmax(Point(0, 1, 2).op(Lm)), 0) and close(vmax(Point(5, 1, 2).op(Lm)), 0),
    )
    Lz = Line(Point(0, 0, -2), Point(0, 0, 2))  # z 轴
    unit_s = Sphere((0, 0, 0), 1.0)
    PPm = Lz.meet(unit_s.dual())  # 交于 (0,0,±1)
    check(
        "meet(line,sphere) = point pair",
        close(vmax(Point(0, 0, 1).op(PPm)), 0)
        and close(vmax(Point(0, 0, -1).op(PPm)), 0),
    )

    # motor: 平移 / 旋转 / 复合 (作用: M.apply(obj))
    T = Motor.translator((1, 2, 3))
    check("translator", close(T.apply(p1).dist(Point(1, 2, 3)), 0))
    R = Motor.rotor((0, 0, 1), math.pi / 2)
    check("rotor 90° z", close(R.apply(p2).dist(Point(0, 1, 0)), 0))
    M = Motor((0, 0, 1), math.pi / 2, (1, 0, 0))
    check(
        "motor rot+trans",
        close(M.apply(p2).dist(Point(1, 1, 0)), 0),
    )

    # versor 保持关联: 平移后的线仍过平移后的点
    Lm2 = T.apply(L)
    check(
        "motor preserves incidence",
        close(vmax(T.apply(Point(2, 0, 0)).op(Lm2)), 0),
    )

    # versor 保持 meet: 先交后变换 = 先变换后交
    lhs = T.apply(pi.dual().meet(pi2.dual()))
    rhs = T.apply(pi).dual().meet(T.apply(pi2).dual())
    a, b = lhs.values, rhs.values
    scale = float(mx.abs(b).max().item())
    check(
        "versor preserves meet",
        bool(mx.allclose(a / scale, b / scale, atol=1e-4).item())
        or bool(mx.allclose(a / scale, -b / scale, atol=1e-4).item()),
    )

    # to_matrix 与 sandwich 作用一致
    Hm = M.to_matrix()
    p_h = [Hm[i][0] + Hm[i][3] for i in range(3)]  # M·(1,0,0,1): R[i][0] + t[i]
    check(
        "motor_to_matrix",
        all(abs(p_h[i] - [1, 1, 0][i]) < 1e-4 for i in range(3)),
    )

    # exp/log roundtrip 与 motor 插值 (Motor.exp(B, s): exp(-s·B), s 带符号)
    R90 = Motor.rotor((0, 0, 1), math.pi / 2)
    R45 = Motor.rotor((0, 0, 1), math.pi / 4)
    B90 = R90.log()
    check(
        "exp∘log roundtrip",
        close(Motor.exp(B90).apply(p2).dist(R90.apply(p2)), 0),
    )
    check(
        "Motor.exp scale=0.5 = half motor",
        close(Motor.exp(B90, 0.5).apply(p2).dist(R45.apply(p2)), 0),
    )
    check(
        "Motor.exp negative scale = inverse",
        close(
            Motor.exp(B90, -0.5)
            .apply(p2)
            .dist(Motor.rotor((0, 0, 1), -math.pi / 4).apply(p2)),
            0,
        ),
    )
    check(
        "interpolate midpoint",
        close(
            Motor.identity().interpolate(R90, 0.5).apply(p2).dist(R45.apply(p2)),
            0,
        ),
    )

    # 远原点距离 (float32 conformal 内积会灾难性抵消 → 0, 回归检查)
    check(
        "dist far from origin",
        close(Point(1000, 0, 0).dist(Point(1001, 0, 0)), 1.0, tol=1e-2),
    )

    # 螺旋运动 (非零节距) exp∘log roundtrip
    M_screw = Motor((0, 0, 1), 0.4, (0.3, -0.2, 0.1))
    M_rt = Motor.exp(M_screw.log())
    check(
        "screw exp∘log roundtrip",
        close(M_rt.apply(p2).dist(M_screw.apply(p2)), 0),
    )

    # extract_velocity: 纯平移幅值 + 纯旋转符号 (body frame)
    dt = 0.1
    ID = Motor.identity()
    Mv = Motor.translator((0.03, 0.0, 0.0))
    (w_v, v_v) = Motor.extract_velocity(Mv.gp(ID), ID, dt)
    check(
        "extract_velocity translation",
        close(v_v[0], 0.3) and close(v_v[1], 0.0) and close(w_v[2], 0.0),
    )
    (w_r, v_r) = Motor.extract_velocity(Motor.rotor((0, 0, 1), 0.2).gp(ID), ID, dt)
    check("extract_velocity rotation sign", close(w_r[2], 2.0))

    # ── 逆渲染: 掩码 / z-buffer / motor 视角 / 区域裁剪 ──────────────
    K = (100.0, 100.0, 64.0, 48.0)
    H2, W2 = 96, 128
    yy2, xx2 = mx.meshgrid(mx.arange(H2), mx.arange(W2), indexing="ij")
    # 掩码模式: 地平面 z=2 (region 1) + 球 (0,0,3) r=0.5 (region 2)
    # 中心像素 → 球前表面 2.5; 圆盘外 → 平面 2.0
    prims = [
        RenderPrimitive("plane", Plane((0, 0, 1), 2.0), 1),
        RenderPrimitive("sphere", Sphere((0, 0, 3), 0.5), 2),
    ]
    disc2 = (xx2 - 64) ** 2 + (yy2 - 48) ** 2 <= 40**2
    reg2 = mx.where(disc2, 2, 1).astype(mx.int32)
    r = render_scene(prims, K, (H2, W2), regions=reg2)
    check("render masked sphere front", close(r.depth[48, 64], 2.5))
    check("render masked plane", close(r.depth[10, 10], 2.0))
    # 全量 z-buffer: 球在前 → 球像素 1.5 遮挡平面 4.0
    r2 = render_scene(
        [
            RenderPrimitive("sphere", Sphere((0, 0, 2), 0.5), 2),
            RenderPrimitive("plane", Plane((0, 0, 1), 4.0), 1),
        ],
        K,
        (H2, W2),
    )
    check("render full zbuffer front", close(r2.depth[48, 64], 1.5))
    check("render full zbuffer corner", close(r2.depth[5, 5], 4.0))
    # motor 视角: 相机前进 1m → 平面变近 1m (4.0→3.0)
    r3 = render_scene(
        [RenderPrimitive("plane", Plane((0, 0, 1), 4.0), 1)],
        K,
        (H2, W2),
        motor=Motor.translator((0.0, 0.0, -1.0)),
    )
    check("render motor view", close(r3.depth[48, 64], 3.0))
    # 掩码裁剪: 无图元区域深度 0
    half2 = mx.where(xx2 < W2 // 2, 1, 0).astype(mx.int32)
    r4 = render_scene(
        [RenderPrimitive("plane", Plane((0, 0, 1), 2.0), 1)],
        K,
        (H2, W2),
        regions=half2,
    )
    check("render masked clip", close(r4.depth[48, 100], 0.0))
    check("render masked keep", close(r4.depth[48, 10], 2.0))

    # ── 圆柱: 解析距离 + 轴上/柱内/柱外 ──────────────────────────────
    cy = Cylinder((0.0, 0.0, 2.0), (0.0, 1.0, 0.0), 1.0)  # 轴 ∥ Y 过 (0,0,2)
    check("cylinder on surface", close(cy.dist(Point(1.0, 5.0, 2.0)), 0.0))
    check("cylinder inside", close(cy.dist(Point(0.2, 0.0, 2.0)), -0.8))
    check("cylinder outside", close(cy.dist(Point(3.0, -2.0, 2.0)), 2.0))
    check("cylinder surface dist symmetric", close(cy.dist(Point(-1.0, 5.0, 2.0)), 0.0))

    # ── 渲染引擎 (cga.engine): from_matrix roundtrip / inverse / 方向光 ──
    from cga.engine import (
        AmbientLight,
        Color,
        CylinderGeometry,
        DirectionalLight,
        Mesh,
        MeshBasicMaterial,
        MeshStandardMaterial,
        PerspectiveCamera,
        PlaneGeometry,
        Scene,
        SphereGeometry,
        render_frame,
    )

    # from_matrix ∘ to_matrix roundtrip (符号约定钉死)
    for axis, ang, t in [
        ((0, 0, 1), 0.7, (1, 2, 3)),
        ((1, 1, 1), 2.1, (-0.5, 0.25, 0.1)),
        ((0.3, -0.8, 0.2), 3.14159, (0, 0, 0)),
    ]:
        t4 = Motor(axis, ang, t).to_matrix()
        r3 = [row[:3] for row in t4[:3]]
        m_rt = Motor.from_matrix(r3, (t4[0][3], t4[1][3], t4[2][3]))
        r4 = m_rt.to_matrix()
        ok_rt = all(
            close(r4[i][j], r3[i][j], tol=2e-3) for i in range(3) for j in range(3)
        ) and all(close(r4[i][3], t4[i][3], tol=2e-3) for i in range(3))
        check(f"from_matrix roundtrip ({axis}, {ang})", ok_rt)

    # Motor.inverse: M·M⁻¹ = identity (点不动), 二次 inverse 复原
    M2 = Motor((0.2, 0.5, -0.3), 1.3, (0.4, -0.1, 0.2))
    p0 = Point(0.7, -0.2, 0.5)
    check(
        "motor inverse point fixed",
        close(diff_max(M2.inverse().apply(M2.apply(p0)), p0), 0),
    )
    check(
        "motor inverse double",
        close(diff_max(M2.inverse().inverse().apply(p0), M2.apply(p0)), 0),
    )

    # 方向光: 方向向量共轭后 e1..e3 部分只由 rotor 决定 —— translator
    # 只向 e∞ 槽写入 (t·u) 杂散项, 方向语义不受影响 (无穷远点语义)。
    M3 = Motor((0, 0, 1), 0.6, (5, -3, 2))
    T3 = Motor.translator((2, 1, -4))
    d0 = Multivector.vector(0.3, -0.8, 0.5)
    # 方向向量部分只由 rotor 决定 (translator 向 e∞ 槽写杂散项, 方向语义
    # 只看 e1..e3 —— 走公开访问器 euclidean_vector)
    a_vec = M3.apply(d0).euclidean_vector()
    b_vec = T3.compose(M3).apply(d0).euclidean_vector()
    check(
        "direction vector part rotor-only",
        close(max(abs(x - y) for x, y in zip(a_vec, b_vec, strict=True)), 0, tol=1e-3),
    )

    # 引擎: 空场景背景 = 精确背景色; MeshBasicMaterial 无光照直出
    H3, W3 = 48, 64
    cam0 = PerspectiveCamera(fov=50, aspect=4 / 3, position=(0, 0, 5), target=(0, 0, 0))
    cam0.look_at((0, 0, 0))
    bg = render_frame(Scene(background=Color(0x87CEEB)), cam0, W3, H3)
    check("engine background", bg[4, 4].tolist() == [135, 206, 235, 255])

    sc_basic = Scene()
    sc_basic.add(
        Mesh(SphereGeometry(1.0), MeshBasicMaterial(Color(0xFF0000))),
        Mesh(PlaneGeometry((0, 1, 0), -1.0), MeshStandardMaterial(Color(0xAAAAAA))),
    )
    img_basic = render_frame(sc_basic, cam0, W3, H3)
    check(
        "engine basic material",
        img_basic[H3 // 2, W3 // 2][:3].tolist() == [255, 0, 0],
    )

    # 标准材质 + 平行光: 相机沿视轴平移, 同表面点着色不变 (方向光无距离)
    # 用奇尺寸 (47,63) 让中心像素恰在轴上, 两次命中同一前表面点
    def _lit_pixel(cam_z: float) -> list[int]:
        sc = Scene()
        sc.add(
            Mesh(
                SphereGeometry(1.0),
                MeshStandardMaterial(Color(0x00FF00), roughness=0.5),
            ),
            DirectionalLight(intensity=0.8, direction=(0, 0, 1)),
            AmbientLight(intensity=0.2),
        )
        cam = PerspectiveCamera(
            fov=50, aspect=4 / 3, position=(0, 0, cam_z), target=(0, 0, 0)
        )
        cam.look_at((0, 0, 0))
        return render_frame(sc, cam, 63, 47)[23, 31][:3].tolist()

    p5, p4 = _lit_pixel(5.0), _lit_pixel(4.0)
    check("directional invariant under camera move", p5 == p4)
    # 光沿视轴 → 前表面 N·L=1 且 H=L+V 平行 → 白色高光 + 绿色漫反射
    check("engine standard lit green", p5[1] > 240 and p5[1] > p5[0] and p5[1] > p5[2])

    # 圆柱: 视线平行于轴 → 中心像素 miss (背景)
    sc_cyl = Scene()
    sc_cyl.add(
        Mesh(CylinderGeometry(0.7), MeshStandardMaterial(Color(0xD4AC0D))),
        DirectionalLight(intensity=0.8, direction=(0.5, 1, 0.3)),
        AmbientLight(intensity=0.2),
    )
    img_cyl = render_frame(sc_cyl, cam0, 63, 47)
    check(
        "engine cylinder axial miss",
        img_cyl[23, 31][:3].tolist() == [135, 206, 235],
    )

    # 抗锯齿: aa=2 (2×2 亚像素采样平均) 平滑球轮廓, 内部像素不变
    sc_aa = Scene(background=Color(0x0000FF))
    sc_aa.add(Mesh(SphereGeometry(1.0), MeshBasicMaterial(Color(0xFF0000))))
    img_aa1 = render_frame(sc_aa, cam0, 63, 47)
    img_aa2 = render_frame(sc_aa, cam0, 63, 47, aa=2)
    check(
        "aa interior identical",
        img_aa2[23, 31].tolist() == img_aa1[23, 31].tolist(),
    )
    check("aa smooths silhouette", bool(mx.any(img_aa1 != img_aa2).item()))

    # 有限圆柱: 端盖之外的像素是背景 (无限圆柱则整条都命中)
    # 圆柱局部轴 Z, 旋转 90° 绕 X → 相机空间竖直条带; 上端盖之上应是背景
    sc_fc = Scene(background=Color(0x0000FF))
    sc_fc.add(
        Mesh(
            CylinderGeometry(0.3, length=1.0),
            MeshBasicMaterial(Color(0xFF0000)),
            motor=Motor.rotor((1, 0, 0), math.pi / 2),
        )
    )
    img_fc = render_frame(sc_fc, cam0, 63, 47)
    red_col = [
        i for i in range(47) if img_fc[i, 31][0] > 200 and img_fc[i, 31][2] < 100
    ]
    check("finite cylinder visible", len(red_col) > 3)
    check(
        "finite cylinder capped",
        red_col and img_fc[max(0, red_col[0] - 3), 31][:3].tolist() == [0, 0, 255],
    )
    # 无限圆柱对照: 同一像素位置是圆柱色 (证明 cap 检查真的在起作用)
    sc_inf = Scene(background=Color(0x0000FF))
    sc_inf.add(
        Mesh(
            CylinderGeometry(0.3),
            MeshBasicMaterial(Color(0xFF0000)),
            motor=Motor.rotor((1, 0, 0), math.pi / 2),
        )
    )
    img_inf = render_frame(sc_inf, cam0, 63, 47)
    check(
        "infinite cylinder uncapped",
        red_col and img_inf[max(0, red_col[0] - 3), 31][0] > 200,
    )

    # ── CRDF 机器人描述 (内联 YAML, 无文件依赖) ─────────────────
    _CRDF2 = """
robot:
  name: r2
  base: base
  links:
    - name: base
    - name: tip
      geometry:
        - blade: cylinder
          radius: 0.05
          length: 0.5
          origin: {xyz: [0, 0, 0.25]}
          role: [visual, collision]
  joints:
    - name: j
      type: revolute
      parent: base
      child: tip
      origin: {xyz: [0, 0, 0.5]}
      axis: [0, 1, 0]
      limit: {lower: -3.14, upper: 3.14}
"""
    r2 = load_robot(_CRDF2)
    m0 = r2.fk_list([0.0])["tip"].to_matrix()
    check("crdf fk q=0 tip z", close(float(m0[2][3]), 0.5, tol=1e-4))
    m90 = r2.fk_list([math.pi / 2])["tip"].to_matrix()
    # URDF 语义: 旋转绕 joint frame 原点原地转 (child 帧原点不动),
    # 局部 +Z 轴经 Rot(Y, π/2) 映到 +X
    check("crdf fk q=90 rotate in place", close(float(m90[2][3]), 0.5, tol=1e-4))
    check(
        "crdf fk q=90 local z->x",
        close(float(m90[0][2]), 1.0, tol=1e-4)
        and close(float(m90[2][0]), -1.0, tol=1e-4),
    )

    def _rejects(text: str) -> bool:
        try:
            load_robot(text)
            return False
        except RobotError:
            return True

    check("crdf reject zero axis", _rejects("""
robot:
  name: bad
  base: b
  links: [{name: b}, {name: c}]
  joints: [{name: j, type: revolute, parent: b, child: c, axis: [0, 0, 0]}]
"""))
    check("crdf reject lower>upper", _rejects("""
robot:
  name: bad
  base: b
  links: [{name: b}, {name: c}]
  joints: [{name: j, type: revolute, parent: b, child: c, axis: [0, 1, 0],
             limit: {lower: 1.0, upper: -1.0}}]
"""))
    check("crdf reject dangling parent", _rejects("""
robot:
  name: bad
  base: b
  links: [{name: b}, {name: c}]
  joints: [{name: j, type: revolute, parent: ghost, child: c, axis: [0, 1, 0]}]
"""))

    # URDF round-trip: 导出 → 再导入 → fk motor 一致 (revolute + prismatic)
    _CRDF_RT = """
robot:
  name: rt
  base: base
  links:
    - name: base
    - name: arm
    - name: tip
  joints:
    - name: j1
      type: revolute
      parent: base
      child: arm
      origin: {xyz: [0, 0, 0.3]}
      axis: [0, 0, 1]
    - name: j2
      type: prismatic
      parent: arm
      child: tip
      origin: {xyz: [0, 0, 0.2]}
      axis: [1, 0, 0]
      limit: {lower: 0, upper: 0.5}
"""
    rt1 = load_robot(_CRDF_RT)
    rt2 = load_robot(urdf_to_crdf(crdf_to_urdf(rt1)))
    a1 = rt1.fk_list([0.4, 0.15])["tip"].to_matrix()
    a2 = rt2.fk_list([0.4, 0.15])["tip"].to_matrix()
    dmax = max(abs(a1[i][j] - a2[i][j]) for i in range(4) for j in range(4))
    check("crdf urdf round-trip fk", close(dmax, 0.0, tol=1e-4))

    # ── CRDF mesh 引用 (URDF 式文件引用, 引擎不渲染, interop 用) ──
    _MESH_URDF = (
        '<robot name="m"><link name="b"/><link name="c">'
        '<visual><geometry><mesh filename="package://x/a.dae" scale="1 2 3"/>'
        "</geometry></visual></link>"
        '<joint name="j" type="fixed"><parent link="b"/><child link="c"/></joint>'
        "</robot>"
    )
    rm1 = load_robot(urdf_to_crdf(_MESH_URDF, mesh_policy="keep"))
    gm1 = rm1.link("c").geometry[0]
    check(
        "crdf mesh keep ref",
        gm1.blade == "mesh"
        and gm1.file == "package://x/a.dae"
        and gm1.scale == (1.0, 2.0, 3.0),
    )
    rm2 = load_robot(urdf_to_crdf(crdf_to_urdf(rm1), mesh_policy="keep"))
    gm2 = rm2.link("c").geometry[0]
    check("crdf mesh round-trip", gm2.file == gm1.file and gm2.scale == gm1.scale)
    check(
        "crdf mesh requires file",
        _rejects(
            """
robot:
  name: x
  base: b
  links:
    - name: b
      geometry:
        - blade: mesh
  joints: []
"""
        ),
    )

    # ── 刚体动力学 (DynamicsPlant, 移植 Drake MultibodyPlant 子集) ──
    from cga.dynamics import DynamicsPlant

    _DYN_PEND = """
robot:
  name: pend
  base: base
  links:
    - name: base
    - name: bob
      inertial:
        mass: 2.0
        com: [0, 0, 0.5]
        inertia: {ixx: 0.01, iyy: 0.01, izz: 0.01}
  joints:
    - name: j1
      type: revolute
      parent: base
      child: bob
      origin: {xyz: [0, 0, 0]}
      axis: [0, 1, 0]
"""
    pend = DynamicsPlant(load_robot(_DYN_PEND))
    check(
        "dyn pendulum gravity",
        close(
            pend.gravity_forces([0.7])[0],
            -2.0 * 9.81 * 0.5 * math.sin(0.7),
            tol=1e-3,
        ),
    )
    # 自由单摆必须下落 (抓 g 的全局符号 bug: 曾导致摆反向上摆)
    check("dyn pendulum falls", pend.forward_dynamics([0.5], [0.0], [0.0])[0] > 0.0)

    _DYN_DOUBLE = """
robot:
  name: dbl
  base: base
  links:
    - name: base
    - name: l1
      inertial:
        mass: 1.0
        com: [0, 0, 0.2]
        inertia: {ixx: 0.01, iyy: 0.01, izz: 0.01}
    - name: l2
      inertial:
        mass: 0.5
        com: [0, 0, 0.15]
        inertia: {ixx: 0.005, iyy: 0.005, izz: 0.005}
  joints:
    - name: j1
      type: revolute
      parent: base
      child: l1
      origin: {xyz: [0, 0, 0]}
      axis: [0, 1, 0]
    - name: j2
      type: revolute
      parent: l1
      child: l2
      origin: {xyz: [0, 0, 0.4]}
      axis: [0, 1, 0]
"""
    dbl = DynamicsPlant(load_robot(_DYN_DOUBLE))
    g = dbl.gravity_forces([0.3, 0.2])
    t1 = -(
        9.81 * (1.0 * 0.2 + 0.5 * 0.4) * math.sin(0.3)
        + 9.81 * 0.5 * 0.15 * math.sin(0.5)
    )
    t2 = -9.81 * 0.5 * 0.15 * math.sin(0.5)
    check("dyn double gravity", close(g[0], t1, tol=1e-3) and close(g[1], t2, tol=1e-3))

    z1d = DynamicsPlant(load_robot(MODEL_CRDF))
    qdyn = [0.3, 0.2, -0.1, 0.1, -0.2, 0.3]
    Mm = z1d.mass_matrix(qdyn)
    check(
        "dyn mass matrix symmetric",
        close(
            max(
                abs(Mm[i][j] - Mm[j][i]) for i in range(6) for j in range(6)
            ),
            0.0,
            tol=1e-6,
        ),
    )
    # 刚性 FK 与 Motor FK 一致 (动力学热路径 = 等价矩阵形式)
    w_rig = z1d.rigid_fk(qdyn)
    w_mot = z1d.robot.fk_list(qdyn)
    dmax = 0.0
    for lnk in z1d.links:
        Rm, t = w_rig[lnk.name]
        m = w_mot[lnk.name].to_matrix()
        for i in range(3):
            for j in range(3):
                dmax = max(dmax, abs(Rm[i][j] - m[i][j]))
            dmax = max(dmax, abs(t[i] - m[i][3]))
    check("dyn rigid fk == motor fk", close(dmax, 0.0, tol=1e-5))
    # forward ∘ inverse == qdd (动力学自洽)
    qdd_ref = [0.5, 0.1, -0.3, 0.2, 0.4, -0.2]
    tau = z1d.inverse_dynamics(
        qdyn, [0.3, -0.2, 0.1, 0.4, -0.1, 0.2], qdd_ref
    )
    qdd_rt = z1d.forward_dynamics(
        qdyn, [0.3, -0.2, 0.1, 0.4, -0.1, 0.2], tau
    )
    check(
        "dyn forward∘inverse",
        close(max(abs(a - b) for a, b in zip(qdd_ref, qdd_rt)), 0.0, tol=1e-6),
    )

    # ── 浮动基座 + 焊接 (Drake floating-base / WeldFrames) ─────
    zf = DynamicsPlant(load_robot(MODEL_CRDF), floating_base=True)
    check("float nq", zf.nq == 12)
    # 全焊接: 整机 = 刚体 (nq=6), 自由落体精确 (基座 z = 1 − ½gt², 无旋转)
    zw = DynamicsPlant(
        load_robot(MODEL_CRDF),
        floating_base=True,
        weld=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
    )
    check("weld rigid nq", zw.nq == 6)
    qw, qdw = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [0.0] * 6
    for _ in range(500):
        qw, qdw = zw.integrate(qw, qdw, [0.0] * 6, 1e-3)
    # 半隐式欧拉离散解: z = 1 − g·dt²·Σk = 1 − 9.81e-6·(500·501/2), vz = −g·t
    z_discrete = 1.0 - 9.81e-6 * (500 * 501 / 2)
    check(
        "weld rigid fall z",
        close(qw[2], z_discrete, tol=1e-3)
        and close(qdw[2], -9.81 * 0.5, tol=1e-3),
    )
    check(
        "weld rigid no rotation",
        max(abs(v) for v in qdw[3:6]) < 1e-3,
    )
    # 注: 浮动基座 + 自由臂在腕部小惯量 (1.8e-4) 处半隐式欧拉数值脆弱
    # (自由臂甩动发散) —— 实机场景用 weld 刚化或需要 LCP 接触求解器。

    # ── 接触 (ContactModel, 惩罚法 + 库仑摩擦) ─────────────────
    from cga.contact import ContactModel

    _DYN_PEND_C = """
robot:
  name: pnd
  base: base
  links:
    - name: base
    - name: rod
      inertial:
        mass: 1.5
        com: [0, 0, 0.3]
        inertia: {ixx: 0.05, iyy: 0.05, izz: 0.01}
      geometry:
        - blade: cylinder
          radius: 0.06
          length: 0.12
          origin: {xyz: [0, 0, 0.54]}
          role: [collision]
  joints:
    - name: j1
      type: revolute
      parent: base
      child: rod
      origin: {xyz: [0, 0, 0.5]}
      axis: [0, 1, 0]
"""
    pc = DynamicsPlant(load_robot(_DYN_PEND_C))
    cm = ContactModel()
    qc, qdc = [0.5], [0.0]
    for _ in range(2000):  # 4s: 摆落下并静止在接触角
        qc, qdc = pc.integrate(qc, qdc, cm.generalized_forces(pc, qc, qdc), 2e-3)
    q_touch = math.acos(-0.5 / 0.6)
    fn = cm.forces(pc, qc, qdc)[0][2]
    check("contact settle angle", close(qc[0], q_touch, tol=1e-2))
    check(
        "contact force equilibrium",
        close(fn, 1.5 * 9.81 * 0.3 / 0.6, tol=1e-1),
    )

    # ── systems 框架 (Diagram/ports: 轨迹→PD→plant→F/T 图组合) ─
    from cga.sensors import ForceTorqueSensor
    from cga.systems import (
        Diagram,
        DynamicsSystem,
        FtsSystem,
        PidController,
        Simulator,
        TrajectorySource,
    )

    _diag = Diagram()
    _i_traj = _diag.add(TrajectorySource([0.1], [0.9], 1.0))
    _i_pid = _diag.add(PidController(pc, kp=100.0, kd=20.0))
    _i_plant = _diag.add(DynamicsSystem(pc, q=[0.1], qd=[0.0]))
    _i_fts = _diag.add(FtsSystem(ForceTorqueSensor(pc, "rod")))
    _diag.connect(_i_traj, "q_des", _i_pid, "q_des")
    _diag.connect(_i_plant, "state", _i_pid, "state")
    _diag.connect(_i_pid, "tau", _i_plant, "tau")
    _diag.connect(_i_plant, "state", _i_fts, "state")
    _diag.connect(_i_plant, "qdd", _i_fts, "qdd")
    _sim = Simulator(_diag, dt=2e-3, trace_ports=[(_i_fts, "fts"), (_i_plant, "state")])
    _traces = _sim.advance_to(3.0)
    _q_end = _traces[f"{_i_plant}:state"][-1][0][0]
    _f_end = _traces[f"{_i_fts}:fts"][-1][0]
    _fm = (_f_end[0] ** 2 + _f_end[1] ** 2 + _f_end[2] ** 2) ** 0.5
    check("diagram closed loop target", close(_q_end, 0.9, tol=1e-2))
    check("diagram fts = weight", close(_fm, 1.5 * 9.81, tol=1e-2))

    # ── 传感器/驱动器 (sensors.py: F/T / 驱动器 / 关节状态) ────
    from cga.sensors import ForceTorqueSensor, JointActuator, JointStateSensor

    fts_rod = ForceTorqueSensor(pc, "rod", name="rod_fts")
    f_rest, _ = fts_rod.read([0.9], [0.0], [0.0])
    fmag = (f_rest[0] ** 2 + f_rest[1] ** 2 + f_rest[2] ** 2) ** 0.5
    check("fts magnitude = subtree weight", close(fmag, 1.5 * 9.81, tol=1e-3))
    act = JointActuator(pc, "j1", effort_limit=5.0)
    check("actuator saturation", close(act.saturate([10.0])[0], 5.0, tol=1e-9))
    sts = JointStateSensor(pc, "j1")
    check("joint state sensor", sts.read([0.7], [1.2], [3.0]) == (0.7, 1.2, 3.0))

    # ── prismatic 关节 (动力学: FK 平移 + 雅可比平动列 + RNEA) ─
    _PRISM_V = """
robot:
  name: pv
  base: base
  links:
    - name: base
    - name: mass
      inertial:
        {mass: 2.0, com: [0, 0, 0.1], inertia: {ixx: 0.01, iyy: 0.01, izz: 0.01}}
  joints:
    - name: s1
      type: prismatic
      parent: base
      child: mass
      axis: [0, 0, 1]
"""
    _PRISM_H = _PRISM_V.replace("axis: [0, 0, 1]", "axis: [1, 0, 0]")
    pv = DynamicsPlant(load_robot(_PRISM_V))
    check(
        "prismatic vertical fall",
        close(pv.forward_dynamics([0.0], [0.0], [0.0])[0], -9.81, tol=1e-3),
    )
    ph = DynamicsPlant(load_robot(_PRISM_H))
    check(
        "prismatic horizontal no gravity",
        close(ph.forward_dynamics([0.0], [0.0], [0.0])[0], 0.0, tol=1e-6),
    )
    _PRISM_TEL = """
robot:
  name: tel
  base: base
  links:
    - name: base
    - name: turret
      inertial:
        {mass: 3.0, com: [0, 0, 0.15], inertia: {ixx: 0.05, iyy: 0.05, izz: 0.04}}
    - name: boom
      inertial:
        {mass: 1.5, com: [0.2, 0, 0.3], inertia: {ixx: 0.01, iyy: 0.02, izz: 0.02}}
  joints:
    - name: yaw
      type: revolute
      parent: base
      child: turret
      origin: {xyz: [0, 0, 0]}
      axis: [0, 0, 1]
    - name: slide
      type: prismatic
      parent: turret
      child: boom
      origin: {xyz: [0, 0, 0.3]}
      axis: [1, 0, 0]
"""
    ptel = DynamicsPlant(load_robot(_PRISM_TEL))
    qa_, qda_ = [0.3, 0.1], [2.0, 0.8]
    Mt = ptel.mass_matrix(qa_)
    Ct = ptel.coriolis_forces(qa_, qda_)
    gt = ptel.gravity_forces(qa_)
    idt = ptel.inverse_dynamics(qa_, qda_, [1.5, -0.5])
    check(
        "prismatic ID consistent",
        max(
            abs(
                idt[i]
                - (Mt[i][0] * 1.5 + Mt[i][1] * (-0.5) + Ct[i] + gt[i])
            )
            for i in range(2)
        )
        < 1e-6,
    )
    et0 = ptel.kinetic_energy(qa_, qda_) + ptel.potential_energy(qa_)
    qe_, qde_ = list(qa_), list(qda_)
    for _ in range(500):
        qe_, qde_ = ptel.integrate_rk4(qe_, qde_, [0.0] * 2, 1e-3)
    et1 = ptel.kinetic_energy(qe_, qde_) + ptel.potential_energy(qe_)
    check("prismatic energy", abs(et1 - et0) / et0 < 0.01)

    # ── 积分器 (RK4 / 变步长 DP45) ───────────────────────────
    q_r, qd_r = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [0.0] * 6
    for _ in range(50):  # 0.5s @ dt=1e-2
        q_r, qd_r = zw.integrate_rk4(q_r, qd_r, [0.0] * 6, 1e-2)
    z_free = 1.0 - 0.5 * 9.81 * 0.25
    check("rk4 free fall exact", close(q_r[2], z_free, tol=1e-6))
    q_a, qd_a = zw.integrate_adaptive(
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [0.0] * 6, [0.0] * 6, 1.0
    )
    check("adaptive free fall", close(q_a[2], 1.0 - 4.905, tol=1e-3))

    # ── 隐式接触 (integrate_implicit: 速度脉冲 + 位置修正) ─────
    # 摆锤隐式法静止角 = 惩罚法相同 (回归)
    qp, qdp = [0.5], [0.0]
    for _ in range(3000):
        qp, qdp = cm.integrate_implicit(pc, qp, qdp, [0.0], 2e-3)
    check("implicit pendulum settle", close(qp[0], q_touch, tol=0.02))
    # 焊接刚体坠落 4s: 隐式接触稳定 (惩罚法撞击发散)
    zd = DynamicsPlant(
        load_robot(MODEL_CRDF),
        floating_base=True,
        weld=("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
    )
    qd_, qdd_ = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0], [0.0] * 6
    drop_ok = True
    for _ in range(2000):
        qd_, qdd_ = cm.integrate_implicit(zd, qd_, qdd_, [0.0] * 6, 2e-3)
        if not all(math.isfinite(v) for v in qd_):
            drop_ok = False
            break
    check("implicit drop stable", drop_ok and max(abs(v) for v in qdd_) < 1e6)
    # 自由自旋体: 能量守恒 (惯性张量旋转 R·I·Rᵀ 正确性)
    qs, qds = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 0.0, 3.0, 0.0]
    e0 = zd.kinetic_energy(qs, qds) + zd.potential_energy(qs)
    spin_ok = True
    for _ in range(1000):
        qs, qds = zd.integrate(qs, qds, [0.0] * 6, 2e-3)
        if not all(math.isfinite(v) for v in qs):
            spin_ok = False
            break
    e1 = zd.kinetic_energy(qs, qds) + zd.potential_energy(qs)
    check("float spin energy", spin_ok and abs(e1 - e0) / e0 < 0.1)

    # ── OOP 封装: 对偶球/平面提取走公开访问器 (motor 共轭后类型降级) ──
    s_cam = Motor.translator((1, 2, 3)).apply(Sphere((0, 0, 0), 0.5))
    (c0, c1, c2), rho = Sphere.from_dual(s_cam)
    check(
        "sphere from_dual center",
        close(c0, 1.0, tol=1e-4) and close(c1, 2.0, tol=1e-4)
        and close(c2, 3.0, tol=1e-4),
    )
    check("sphere from_dual radius", close(rho, 0.5, tol=1e-4))
    pi_cam = Motor.translator((1, 2, 3)).apply(Plane((0, 1, 0), 0.0))
    check("plane einf_coeff", close(float(pi_cam.einf_coeff()), 2.0, tol=1e-4))

    print(f"\nall {_ok} checks passed")


if __name__ == "__main__":
    main()
