"""PrimitiveRenderer 自检: 掩码/z-buffer/motor 视角/裁剪/圆柱/alpha/空场景。"""

import mlx.core as mx

from cga import (
    Cylinder,
    Motor,
    Plane,
    PrimitiveRenderer,
    RenderPrimitive,
    Sphere,
)
from tests.checks import Checks


class TestPrimitiveRenderer(Checks):
    """逆渲染 (图元场景 → 深度/颜色) 的解析真值断言。"""

    K = (100.0, 100.0, 64.0, 48.0)
    H, W = 96, 128

    @staticmethod
    def render(prims, regions=None, motor=None):
        cls = TestPrimitiveRenderer
        return PrimitiveRenderer.render_scene(
            prims, cls.K, (cls.H, cls.W), regions, motor
        )

    def test_masked_mode(self):
        # 地平面 z=2 (region 1) + 球 (0,0,3) r=0.5 (region 2, 圆盘掩码)
        prims = [
            RenderPrimitive("plane", Plane((0, 0, 1), 2.0), 1),
            RenderPrimitive("sphere", Sphere((0, 0, 3), 0.5), 2),
        ]
        H, W = self.H, self.W
        yy, xx = mx.meshgrid(mx.arange(H), mx.arange(W), indexing="ij")
        disc = (xx - 64) ** 2 + (yy - 48) ** 2 <= 40**2
        regions = mx.where(disc, 2, 1).astype(mx.int32)
        out = self.render(prims, regions)
        assert self.close(out.depth[48, 64], 2.5)  # 中心 → 球前表面
        assert self.close(out.depth[10, 10], 2.0)  # 圆盘外 → 平面

    def test_full_zbuffer(self):
        # 全量模式: 球 (z=2, r=0.5) 遮挡平面 (z=4)
        out = self.render(
            [
                RenderPrimitive("sphere", Sphere((0, 0, 2), 0.5), 2),
                RenderPrimitive("plane", Plane((0, 0, 1), 4.0), 1),
            ]
        )
        assert self.close(out.depth[48, 64], 1.5)  # 球前表面
        assert self.close(out.depth[5, 5], 4.0)  # 角落平面

    def test_motor_view(self):
        # 世界→相机 translator(−1·z) = 相机前进 1m → 平面 4.0→3.0
        out = self.render(
            [RenderPrimitive("plane", Plane((0, 0, 1), 4.0), 1)],
            motor=Motor.translator((0.0, 0.0, -1.0)),
        )
        assert self.close(out.depth[48, 64], 3.0)

    def test_masked_clip(self):
        # region 0 无图元 → 右半深度 0, 左半平面 2.0
        H, W = self.H, self.W
        xx = mx.meshgrid(mx.arange(H), mx.arange(W), indexing="ij")[1]
        half = mx.where(xx < W // 2, 1, 0).astype(mx.int32)
        out = self.render([RenderPrimitive("plane", Plane((0, 0, 1), 2.0), 1)], half)
        assert float(out.depth[48, 100]) == 0.0
        assert self.close(out.depth[48, 10], 2.0)

    def test_cylinder_full_and_masked(self):
        # 竖柱 (轴 ∥ Y 过 (0,0,3), r=0.4) + 背景墙 z=5
        prims = [
            RenderPrimitive("cylinder", Cylinder((0, 0, 3), (0, 1, 0), 0.4), 1),
            RenderPrimitive("plane", Plane((0, 0, 1), 5.0), 2),
        ]
        out = self.render(prims)
        assert self.close(out.depth[48, 64], 2.6, tol=1e-3)  # 柱前表面 3−0.4
        assert self.close(out.depth[5, 5], 5.0, tol=1e-3)  # 角落墙
        # 掩码: 柱区 |u| < 0.13 命中柱, 其余墙
        H, W = self.H, self.W
        xx = mx.meshgrid(mx.arange(H), mx.arange(W), indexing="ij")[1]
        regions = mx.where(mx.abs((xx - 64) / 128.0) < 0.13, 1, 2).astype(mx.int32)
        out2 = self.render(prims, regions)
        assert self.close(out2.depth[48, 64], 2.6, tol=0.05)
        assert self.close(out2.depth[48, 100], 5.0, tol=0.05)

    def test_alpha_compositing(self):
        # 半透明球 (z=2, r=0.5) 在墙 (z=4) 前, alpha 扫描
        wall = RenderPrimitive("plane", Plane((0, 0, 1), 4.0), 1)
        sphere = Sphere((0, 0, 2), 0.5)
        ball = RenderPrimitive("sphere", sphere, 2)
        rgb_wall = self.render([wall]).rgb[48, 64]
        rgb_ball = self.render([ball, wall]).rgb[48, 64]
        # alpha=0 → 与纯墙逐位一致; alpha=1 → 与不透明球一致
        g0 = self.render([RenderPrimitive("sphere", sphere, 2, alpha=0.0), wall])
        assert g0.rgb[48, 64].tolist() == rgb_wall.tolist()
        g1 = self.render([RenderPrimitive("sphere", sphere, 2, alpha=1.0), wall])
        assert g1.rgb[48, 64].tolist() == rgb_ball.tolist()
        # alpha=0.5 → 通道级介于两端 (±2 容 uint8 截断)
        g5 = self.render([RenderPrimitive("sphere", sphere, 2, alpha=0.5), wall])
        px = g5.rgb[48, 64].astype(mx.float32)
        lo = mx.minimum(rgb_wall.astype(mx.float32), rgb_ball.astype(mx.float32)) - 2
        hi = mx.maximum(rgb_wall.astype(mx.float32), rgb_ball.astype(mx.float32)) + 2
        assert bool(mx.all((px >= lo) & (px <= hi)))
        # 深度语义不变: 半透明也取前表面 1.5
        assert self.close(g5.depth[48, 64], 1.5)

    def test_empty_scene(self):
        out = self.render([])
        assert float(mx.max(out.depth)) == 0.0
        assert int(mx.max(out.rgb)) == 0
