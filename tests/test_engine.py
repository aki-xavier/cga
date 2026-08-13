"""引擎基础自检: 背景/无光照材质/平行光不变量/抗锯齿/有限圆柱端盖。"""

import math

import mlx.core as mx

from cga import Motor
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
    Renderer,
    Scene,
    SphereGeometry,
)
from tests.checks import Checks


class TestEngine(Checks):
    """Renderer 的基础行为 (解析真值/逐位断言)。"""

    @staticmethod
    def cam0() -> PerspectiveCamera:
        cam = PerspectiveCamera(
            fov=50, aspect=4 / 3, position=(0, 0, 5), target=(0, 0, 0)
        )
        cam.look_at((0, 0, 0))
        return cam

    def test_background_exact(self):
        # 空场景背景 = 精确背景色 (sRGB roundtrip 恒等)
        img = Renderer.render_frame(
            Scene(background=Color(0x87CEEB)), self.cam0(), 64, 48
        )
        assert img[4, 4].tolist() == [135, 206, 235, 255]

    def test_basic_material_identity(self):
        # MeshBasicMaterial 无光照直出 (sRGB roundtrip 恒等)
        sc = Scene()
        sc.add(
            Mesh(SphereGeometry(1.0), MeshBasicMaterial(Color(0xFF0000))),
            Mesh(PlaneGeometry((0, 1, 0), -1.0), MeshStandardMaterial(Color(0xAAAAAA))),
        )
        img = Renderer.render_frame(sc, self.cam0(), 64, 48)
        assert img[24, 32][:3].tolist() == [255, 0, 0]

    @staticmethod
    def lit_pixel(cam_z: float) -> list[int]:
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
        # 奇尺寸 (47,63) 让中心像素恰在轴上, 两次命中同一前表面点
        return Renderer.render_frame(sc, cam, 63, 47)[23, 31][:3].tolist()

    def test_directional_invariant_under_camera_move(self):
        # 平行光无距离: 相机沿视轴平移, 同表面点着色不变
        assert self.lit_pixel(5.0) == self.lit_pixel(4.0)

    def test_standard_lit_green(self):
        # 光沿视轴 → 前表面 N·L=1 → 白色高光 + 绿色漫反射
        p5 = self.lit_pixel(5.0)
        assert p5[1] > 240 and p5[1] > p5[0] and p5[1] > p5[2]

    def test_cylinder_axial_miss(self):
        # 视线平行于柱轴 → 中心像素 miss (背景)
        sc = Scene()
        sc.add(
            Mesh(CylinderGeometry(0.7), MeshStandardMaterial(Color(0xD4AC0D))),
            DirectionalLight(intensity=0.8, direction=(0.5, 1, 0.3)),
            AmbientLight(intensity=0.2),
        )
        img = Renderer.render_frame(sc, self.cam0(), 63, 47)
        assert img[23, 31][:3].tolist() == [135, 206, 235]

    def test_aa(self):
        # aa=2 (2×2 亚像素采样平均) 平滑球轮廓, 内部像素不变
        sc = Scene(background=Color(0x0000FF))
        sc.add(Mesh(SphereGeometry(1.0), MeshBasicMaterial(Color(0xFF0000))))
        img1 = Renderer.render_frame(sc, self.cam0(), 63, 47)
        img2 = Renderer.render_frame(sc, self.cam0(), 63, 47, aa=2)
        assert img2[23, 31].tolist() == img1[23, 31].tolist()
        assert bool(mx.any(img1 != img2).item())

    def test_finite_cylinder_capped(self):
        # 圆柱局部轴 Z, 旋转 90° 绕 X → 相机空间竖直条带; 端盖之外是背景
        sc = Scene(background=Color(0x0000FF))
        sc.add(
            Mesh(
                CylinderGeometry(0.3, length=1.0),
                MeshBasicMaterial(Color(0xFF0000)),
                motor=Motor.rotor((1, 0, 0), math.pi / 2),
            )
        )
        img = Renderer.render_frame(sc, self.cam0(), 63, 47)
        red_col = [
            i for i in range(47) if img[i, 31][0] > 200 and img[i, 31][2] < 100
        ]
        assert len(red_col) > 3
        assert img[max(0, red_col[0] - 3), 31][:3].tolist() == [0, 0, 255]

    def test_infinite_cylinder_uncapped(self):
        # 无限圆柱对照: 同一像素位置命中圆柱 (证明 cap 检查真的在起作用)
        sc = Scene(background=Color(0x0000FF))
        sc.add(
            Mesh(
                CylinderGeometry(0.3),
                MeshBasicMaterial(Color(0xFF0000)),
                motor=Motor.rotor((1, 0, 0), math.pi / 2),
            )
        )
        img = Renderer.render_frame(sc, self.cam0(), 63, 47)
        # 与有限版同一条带, 上端盖之外仍命中
        sc_cap = Scene(background=Color(0x0000FF))
        sc_cap.add(
            Mesh(
                CylinderGeometry(0.3, length=1.0),
                MeshBasicMaterial(Color(0xFF0000)),
                motor=Motor.rotor((1, 0, 0), math.pi / 2),
            )
        )
        img_cap = Renderer.render_frame(sc_cap, self.cam0(), 63, 47)
        red_col = [
            i for i in range(47) if img_cap[i, 31][0] > 200 and img_cap[i, 31][2] < 100
        ]
        assert red_col and img[max(0, red_col[0] - 3), 31][0] > 200
