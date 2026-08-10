"""逆渲染: 把 CGA 图元场景 (SceneModel / 任意 blade 列表) 渲染回 2D 图像。

用途: 重建闭环验证 (模型 → 图像 round-trip) 与 novel-view 预览。

约定 (与 scenegraph.export 一致):
  - 场景已是米制相机空间: X 右 / Y 下 / Z 前, 相机在原点, 针孔
    col = fx·X/Z + cx, row = fy·Y/Z + cy。
  - 可选 motor = 世界→相机变换: 先对每个 blade 做 versor 共轭
    (M.apply) 再求交 —— 换视角的唯一入口, 不传即恒等。

两种模式:
  - regions 给定 (掩码模式): 每像素只允许"自己区域的图元"命中
    (region 查表 → 对应图元)。忠实还原管线已知的区域分配, 无限
    平面被区域轮廓天然裁剪, round-trip 可断言。
  - regions=None (全量模式): 对全部图元 z-buffer 逐像素最近面。
    限制如实标注: SceneModel 无 3D 面片范围, 无限平面全量渲染会
    整片覆盖 —— novel-view 只对闭曲面 (球) 真正成立; 面片范围
    是留钩 (export 后续可附区域 bbox 反投影)。

求交走欧氏解析 (ray-plane 直线 + ray-sphere 二次), 不碰 conformal
内积 —— float32 的 conformal 距离在远原点灾难性抵消 (模块判例)。

着色不装物理: 管线无 albedo, 用 region 色盘平涂 + 固定方向
Lambert 几何明暗; rgb 只作可视化, 不作光学声明。
"""

import math
from collections.abc import Sequence
from typing import NamedTuple

import mlx.core as mx

from cga import Cylinder, Motor, Multivector, Plane, Sphere

# 区域色盘 (12 色, 区分相邻区域即可, 非语义色)
_PALETTE = [
    (244, 67, 54), (33, 150, 243), (76, 175, 80), (255, 193, 7),
    (156, 39, 176), (0, 188, 212), (255, 87, 34), (139, 195, 74),
    (63, 81, 181), (255, 235, 59), (0, 150, 136), (233, 30, 99),
]

_LIGHT = (0.3, 0.6, 1.0)  # 固定方向光 (仅明暗, 非物理)


class RenderPrimitive(NamedTuple):
    """渲染图元 (鸭子类型自容, 不依赖 scenegraph)。"""

    kind: str  # "plane" / "sphere"
    blade: Multivector  # 米制 Plane/Sphere blade
    region: int = 0  # 区域 id (掩码模式查表用)


class RenderResult(NamedTuple):
    """渲染结果。"""

    depth: mx.array  # (H,W) float32 相机 Z (无命中 = 0)
    rgb: mx.array  # (H,W,3) uint8 可视化


def _plane_params(b: Multivector) -> tuple[mx.array, float]:
    """Plane blade → (单位法向 (3,), 距离 d)。法向读取 e1..e3 系数
    (Plane 构造已归一化; versor 共轭保持单位)。"""
    n = b.values[1:4]
    return n, float(b.values[5])


def _sphere_params(b: Multivector) -> tuple[mx.array, float]:
    """Sphere blade → (球心 (3,), 半径)。对偶球 s = w·(up(c) − ½ρ²e∞):
    c = v/w, ρ² = |c|² − 2f/w (与 algebra.Sphere.dist 同公式)。"""
    w = float(b.values[4])
    if abs(w) < 1e-12:
        raise ValueError("sphere blade has no e0 component")
    v = b.values[1:4]
    c = v / w
    f = float(b.values[5])
    r2 = float(mx.sum(v * v)) / (w * w) - 2.0 * f / w
    return c, math.sqrt(max(0.0, r2))


def _cylinder_params(b: Multivector) -> tuple[mx.array, mx.array, float]:
    """Cylinder (轴 Line blade + slots) → (轴点 (3,), 轴向 (3,), 半径)。
    注意: Cylinder 的几何在对象的 slots 里, motor 共轭会丢 (apply
    返回纯 Multivector) —— 圆柱暂不支持 motor 视角, 注释在 render_scene。"""
    q = b._axis_point  # type: ignore[attr-defined]
    n = b._axis_dir  # type: ignore[attr-defined]
    return mx.array(q), mx.array(n), float(b.radius)  # type: ignore[attr-defined]


def render_scene(
    prims: Sequence[RenderPrimitive],
    K: tuple[float, float, float, float],
    shape: tuple[int, int],
    regions: mx.array | None = None,
    motor: Motor | None = None,
    near: float = 0.1,
    far: float = 1e4,
) -> RenderResult:
    """图元场景 → 深度图 + 可视化 rgb。

    prims: 米制相机空间图元 (或经 motor 变到相机空间)。
    K: (fx, fy, cx, cy)。
    regions: (H,W) int32 区域标签 (掩码模式); None = 全量 z-buffer。
    motor: 世界→相机 Motor, 先共轭到相机空间再求交。
    """
    fx, fy, cx, cy = K
    assert fx > 0 and fy > 0, f"无效焦距 {K}"
    H, W = shape
    yy, xx = mx.meshgrid(
        mx.arange(H, dtype=mx.float32), mx.arange(W, dtype=mx.float32),
        indexing="ij",
    )
    # 光线方向 (H,W,3), d_z = 1 → 命中参数 t 即相机深度 Z
    dirs = mx.stack(
        [(xx - cx) / fx, (yy - cy) / fy, mx.ones_like(xx)], axis=-1
    )
    cams = [
        motor.apply(p.blade) if motor is not None else p.blade for p in prims
    ]
    light = mx.array(_LIGHT, dtype=mx.float32)
    light = light / mx.linalg.norm(light)
    best = mx.full((H, W), float("inf"), dtype=mx.float32)
    best_rgb = mx.zeros((H, W, 3), dtype=mx.uint8)
    for p, b in zip(prims, cams, strict=True):
        if regions is None:
            sel = mx.ones((H, W), dtype=mx.bool_)
        else:
            sel = regions == p.region
        if p.kind == "plane":
            n, d = _plane_params(b)
            denom = mx.where(
                mx.abs(dirs @ n) > 1e-8, dirs @ n, float("inf")
            )
            t = d / denom
            nrm = mx.broadcast_to(n, (H, W, 3))
        elif p.kind == "cylinder":
            # 透视射线与无限柱 (轴 n̂ 过 q, 半径 ρ): |P(t·d−q)| = ρ,
            # P = I−n̂n̂ᵀ → a·t² + 2b·t + c = 0, 近根 (实心柱前表面)
            q_c, n_c, r_c = _cylinder_params(b)
            dn = mx.sum(dirs * n_c, axis=-1)
            qn = mx.sum(q_c * n_c)
            aq = mx.sum(dirs * dirs, axis=-1) - dn * dn
            bq = dn * qn - mx.sum(dirs * q_c, axis=-1)  # d·P·(−q)
            cq = (mx.sum(q_c * q_c) - qn * qn) - r_c * r_c
            disc = bq * bq - aq * cq
            t = mx.where(
                (aq > 1e-8) & (disc > 0.0),
                (-bq - mx.sqrt(mx.maximum(disc, 0.0))) / aq,
                float("inf"),
            )
            # 柱面法向 (用于 Lambert): 径向单位向量
            hit = t[..., None] * dirs - q_c
            rad = hit - mx.sum(hit * n_c, axis=-1, keepdims=True) * n_c
            nrm = rad / mx.maximum(
                mx.linalg.norm(rad, axis=-1, keepdims=True), 1e-8
            )
        else:  # sphere
            c, r = _sphere_params(b)
            a = mx.sum(dirs * dirs, axis=-1)
            bb = -2.0 * mx.sum(dirs * c, axis=-1)
            cc = mx.sum(c * c) - r * r
            disc = bb * bb - 4.0 * a * cc
            t = mx.where(
                disc > 0.0,
                (-bb - mx.sqrt(mx.maximum(disc, 0.0))) / (2.0 * a),
                float("inf"),
            )
            hit = t[..., None] * dirs - c
            nrm = hit / mx.maximum(
                mx.linalg.norm(hit, axis=-1, keepdims=True), 1e-8
            )
        t = mx.where((t > near) & (t < far), t, float("inf"))
        t = mx.where(sel, t, float("inf"))
        take = t < best
        best = mx.where(take, t, best)
        # region 色盘 + Lambert 明暗
        col = mx.array(_PALETTE[p.region % len(_PALETTE)], dtype=mx.float32)
        sh = mx.maximum(mx.sum(nrm * light, axis=-1), 0.0)
        rgb_p = (col[None, None, :] * (0.35 + 0.65 * sh[..., None]))
        best_rgb = mx.where(
            take[..., None], rgb_p.astype(mx.uint8), best_rgb
        )
    depth = mx.where(best < far, best, 0.0)
    return RenderResult(depth, best_rgb)





def _selftest_cylinder() -> None:
    """圆柱求交自检 (render.py __main__ 太短, 单独函数复用)。"""
    K = (100.0, 100.0, 64.0, 48.0)
    H, W = 96, 128
    yy, xx = mx.meshgrid(mx.arange(H), mx.arange(W), indexing="ij")
    # 竖柱 (轴 ∥ Y 过 (0, 0, 3), 半径 0.4) + 背景墙 z=5
    cy = Cylinder((0.0, 0.0, 3.0), (0.0, 1.0, 0.0), 0.4)
    prims = [
        RenderPrimitive("cylinder", cy, 1),
        RenderPrimitive("plane", Plane((0, 0, 1), 5.0), 2),
    ]
    # 全量模式: 中心像素 (64,48) 射线 (0,0,1) 命中柱前表面 z=3−0.4=2.6
    out = render_scene(prims, K, (H, W))
    assert abs(float(out.depth[48, 64]) - 2.6) < 1e-3, float(out.depth[48, 64])
    # 角落像素 (射线远离柱) → 墙 5.0
    assert abs(float(out.depth[5, 5]) - 5.0) < 1e-3, float(out.depth[5, 5])
    print("  ok  cylinder: 中心 2.6 (前表面) / 角落墙 5.0")
    # 掩码模式: 柱区域=柱像素 (射线命中柱的 u 范围), 其余墙
    # 柱像素: |u| < ρ/z_c·f ≈ 0.4/3·100/128... 用射线命中判定
    u_n = (xx - 64) / 128.0
    hit = mx.abs(u_n) < 0.13  # 透视柱宽近似 (±0.4/3)
    regions2 = mx.where(hit, 1, 2).astype(mx.int32)
    out2 = render_scene(prims, K, (H, W), regions=regions2)
    d_c = float(out2.depth[48, 64])
    assert abs(d_c - 2.6) < 0.05, d_c
    d_e = float(out2.depth[48, 100])
    assert abs(d_e - 5.0) < 0.05, d_e
    print(f"  ok  cylinder masked: 柱区 {d_c:.2f} / 墙区 {d_e:.2f}")

if __name__ == "__main__":
    # ── 自检: 合成场景 (掩码 + 全量 + motor + 裁剪) ─────────────────
    K = (100.0, 100.0, 64.0, 48.0)
    H, W = 96, 128

    # 场景: 地平面 z=2 (region 1) + 球心 (0,0,3) r=0.5 (region 2)
    plane = Plane((0.0, 0.0, 1.0), 2.0)
    sphere = Sphere((0.0, 0.0, 3.0), 0.5)
    prims = [RenderPrimitive("plane", plane, 1), RenderPrimitive("sphere", sphere, 2)]

    yy, xx = mx.meshgrid(mx.arange(H), mx.arange(W), indexing="ij")
    disc = (xx - 64) ** 2 + (yy - 48) ** 2 <= 40**2  # 球区域 = 圆盘
    regions = mx.where(disc, 2, 1).astype(mx.int32)

    # 掩码模式: 中心像素 → 球前表面 z = 3−0.5 = 2.5; 圆盘外 → 平面 2.0
    out = render_scene(prims, K, (H, W), regions=regions)
    assert abs(float(out.depth[48, 64]) - 2.5) < 1e-3, float(out.depth[48, 64])
    assert abs(float(out.depth[10, 10]) - 2.0) < 1e-3, float(out.depth[10, 10])
    print("  ok  masked: 球前表面 2.5 / 平面 2.0")

    # 全量 z-buffer: 球在平面前 → 球像素取球 (1.5), 角落取平面 (4.0)
    plane_far = Plane((0.0, 0.0, 1.0), 4.0)
    sphere_near = Sphere((0.0, 0.0, 2.0), 0.5)
    out2 = render_scene(
        [
            RenderPrimitive("sphere", sphere_near, 2),
            RenderPrimitive("plane", plane_far, 1),
        ],
        K, (H, W),
    )
    assert abs(float(out2.depth[48, 64]) - 1.5) < 1e-3, float(out2.depth[48, 64])
    assert abs(float(out2.depth[5, 5]) - 4.0) < 1e-3, float(out2.depth[5, 5])
    print("  ok  full z-buffer: 球 1.5 遮挡平面 4.0 / 角落 4.0")

    # motor: 世界→相机 = 对场景 blades 共轭; translator(−1·z) 把
    # 场景后移 1m (相机前进 1m) → 平面 4.0→3.0
    m = Motor.translator((0.0, 0.0, -1.0))
    out3 = render_scene(
        [RenderPrimitive("plane", plane_far, 1)], K, (H, W), motor=m
    )
    assert abs(float(out3.depth[48, 64]) - 3.0) < 1e-3, float(out3.depth[48, 64])
    print("  ok  motor: 相机前进 1m → 平面 4.0→3.0")

    # 掩码裁剪: region 0 无图元 → 右半深度 0
    half = mx.where(xx < W // 2, 1, 0).astype(mx.int32)
    out4 = render_scene([RenderPrimitive("plane", plane, 1)], K, (H, W), regions=half)
    assert float(out4.depth[48, 100]) == 0.0, float(out4.depth[48, 100])
    assert float(out4.depth[48, 10]) == 2.0, float(out4.depth[48, 10])
    print("  ok  masked clip: 无图元区域深度 0 / 图元区域 2.0")
    _selftest_cylinder()
    print("cga.render: 6 项自检 ✓")

