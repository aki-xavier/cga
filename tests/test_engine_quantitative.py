"""渲染管线定量自检: sRGB roundtrip / 阴影 / Beer 吸收 / Fresnel。

全部从第一性原理推导期望值 (管线数学: sRGB 解码 → 线性光照 →
输出端编码), 不依赖实现内部状态。
"""

import math

from cga.engine import (
    AmbientLight,
    BoxGeometry,
    Color,
    DirectionalLight,
    Mesh,
    MeshBasicMaterial,
    MeshStandardMaterial,
    PerspectiveCamera,
    PlaneGeometry,
    PointLight,
    Renderer,
    Scene,
    SphereGeometry,
)
from tests.checks import Checks


class TestRendererQuantitative(Checks):
    """Whitted 管线 (折射/Fresnel/Beer/阴影/sRGB) 的公式级断言。"""

    WALL = [0xCC / 255, 0x33 / 255, 0x33 / 255]  # 红墙 (sRGB 编码值)

    @staticmethod
    def head_on_cam() -> PerspectiveCamera:
        cam = PerspectiveCamera(
            fov=40, aspect=1.0, position=(0, 0, 0), target=(0, 0, 1)
        )
        cam.look_at((0, 0, 1))
        return cam

    @staticmethod
    def wall_scene() -> Scene:
        sc = Scene()
        sc.add(
            Mesh(
                PlaneGeometry((0, 0, -1), -4.0),
                MeshBasicMaterial(Color(0xCC3333)),
            )
        )
        return sc

    def test_srgb_roundtrip_identity(self):
        # Basic 材质/背景: 解码→编码 roundtrip 恒等
        img = Renderer(64, 64, aa=1).render(self.wall_scene(), self.head_on_cam())
        assert img[32, 32, :3].tolist() == [204, 51, 51]

    def test_ior1_invisible(self):
        # ior=1 & opacity=0 & absorption=0 → 逐位不可见 (F≡0, 无弯折)
        sc = self.wall_scene()
        sc.add(
            Mesh(
                SphereGeometry(0.8),
                MeshStandardMaterial(Color(0xAAD4FF), opacity=0.0, ior=1.0),
                position=(0, 0, 2.2),
            )
        )
        r = Renderer(64, 64, aa=1)
        cam = self.head_on_cam()
        a = Renderer.frame_to_bytes(r.render(sc, cam))
        b = Renderer.frame_to_bytes(r.render(self.wall_scene(), cam))
        assert a == b

    def test_fresnel_normal_incidence(self):
        # 纯玻璃平面 (opacity=0, ior=1.5) 正向入射: R0 = (0.5/2.5)² = 0.04
        # 像素 = enc(0.96·dec(墙) + 0.04·dec(背景)) (反射方向回相机 → bg)
        sc = self.wall_scene()
        sc.add(
            Mesh(
                PlaneGeometry((0, 0, -1), -2.0),
                MeshBasicMaterial(Color(0x00FF00), opacity=0.0),
            )
        )
        px = Renderer(64, 64, aa=1).render(sc, self.head_on_cam())[32, 32, :3].tolist()
        bg = Scene().background
        exp = [
            self.linear_to_srgb255(
                0.96 * self.srgb_to_linear(w) + 0.04 * self.srgb_to_linear(b)
            )
            for w, b in zip(self.WALL, (bg.r, bg.g, bg.b))
        ]
        assert all(abs(a - e) <= 2 for a, e in zip(px, exp)), f"{px} vs {exp}"

    def test_fresnel_grazing(self):
        # 掠射 75.4° (相机斜看平面, 命中点仍在墙前): 精确非偏振 F 定量
        target = (3.85, 0, 1)
        cam = PerspectiveCamera(fov=40, aspect=1.0, position=(0, 0, 0), target=target)
        cam.look_at(target)
        sc = self.wall_scene()
        sc.add(
            Mesh(
                PlaneGeometry((0, 0, -1), -2.0),
                MeshBasicMaterial(Color(0x00FF00), opacity=0.0),
            )
        )
        r = Renderer(64, 64, aa=1)
        bare = r.render(self.wall_scene(), cam)[32, 32, :3].tolist()
        px = r.render(sc, cam)[32, 32, :3].tolist()
        ci = 1.0 / math.sqrt(3.85**2 + 1)
        eta = 1 / 1.5
        ct = math.sqrt(1 - eta * eta * (1 - ci * ci))
        g = 1.5
        rs = (ci - g * ct) / (ci + g * ct)
        rp = (ct - g * ci) / (ct + g * ci)
        fres = 0.5 * (rs * rs + rp * rp)
        bg = Scene().background
        exp = [
            self.linear_to_srgb255(
                fres * self.srgb_to_linear(b)
                + (1 - fres) * self.srgb_to_linear(w / 255)
            )
            for w, b in zip(bare, (bg.r, bg.g, bg.b))
        ]
        assert all(abs(a - e) <= 3 for a, e in zip(px, exp)), f"{px} vs {exp}"

    def test_beer_absorption(self):
        # 玻璃盒 (ior=1 隔离 Fresnel/弯折): 像素 = enc(exp(−σ·厚)·dec(墙))
        def slab(depth: float, absorption: float) -> list[int]:
            sc = self.wall_scene()
            sc.add(
                Mesh(
                    BoxGeometry(3, 3, depth),
                    MeshStandardMaterial(
                        Color(0xFFFFFF), opacity=0.0, ior=1.0, absorption=absorption
                    ),
                    position=(0, 0, 2.5),
                )
            )
            img = Renderer(64, 64, aa=1).render(sc, self.head_on_cam())
            return img[32, 32, :3].tolist()

        for depth in (0.2, 1.0):
            px = slab(depth, 0.8)
            trans = math.exp(-0.8 * depth)
            exp = [
                self.linear_to_srgb255(trans * self.srgb_to_linear(w))
                for w in self.WALL
            ]
            assert all(abs(a - e) <= 2 for a, e in zip(px, exp)), (
                f"depth={depth}: {px} vs {exp}"
            )
        # σ=0 → 无衰减 (逐位墙面色)
        assert slab(1.0, 0.0) == [204, 51, 51]

    @staticmethod
    def shadow_scene(opacity: float | None) -> Scene:
        # 平行光垂直向下, 球在本影点正上方; 相机低角度斜视 (ndv≈0 抑制高光)
        sc = Scene()
        sc.add(
            Mesh(
                PlaneGeometry((0, 1, 0), 0.0),
                MeshStandardMaterial(Color(0xFFFFFF), roughness=1.0),
            ),
            DirectionalLight(intensity=0.8, direction=(0, 1, 0)),
            AmbientLight(intensity=0.2),
        )
        if opacity is not None:
            sc.add(
                Mesh(
                    SphereGeometry(0.5),
                    MeshStandardMaterial(
                        Color(0xFFFFFF), roughness=1.0, opacity=opacity, ior=1.0
                    ),
                    position=(2, 1.5, 3),
                )
            )
        return sc

    @staticmethod
    def shadow_cam() -> PerspectiveCamera:
        cam = PerspectiveCamera(
            fov=40, aspect=1.0, position=(0, 0.8, -1), target=(2, 0, 3)
        )
        cam.look_at((2, 0, 3))
        return cam

    def test_shadow_umbra_is_ambient_only(self):
        # 本影 = 仅环境光: 白地面 dec=1.0, L = 0.2
        px = Renderer(96, 96, aa=1).render(self.shadow_scene(1.0), self.shadow_cam())
        assert abs(px[48, 48, 0].item() - self.linear_to_srgb255(0.2)) <= 2

    def test_glass_occluder_partial_shadow(self):
        # 透明遮挡物按 (1−opacity) 透光: vis=0.7 → L = 0.2 + 0.8·0.7
        r = Renderer(96, 96, aa=1)
        cam = self.shadow_cam()
        px_opaque = r.render(self.shadow_scene(1.0), cam)[48, 48, 0].item()
        px_glass = r.render(self.shadow_scene(0.3), cam)[48, 48, 0].item()
        px_clear = r.render(self.shadow_scene(None), cam)[48, 48, 0].item()
        exp_glass = self.linear_to_srgb255(0.2 + 0.56)
        assert abs(px_glass - exp_glass) <= 8  # 残余高光由容差覆盖
        assert px_clear > px_glass > px_opaque + 30

    def test_point_light_shadow_range(self):
        # 点光源 far 截断: 光在球上方 → 本影; 光在球下方 → 无影
        def scene(light_y: float) -> Scene:
            sc = Scene()
            sc.add(
                Mesh(
                    PlaneGeometry((0, 1, 0), 0.0),
                    MeshStandardMaterial(Color(0xFFFFFF), roughness=1.0),
                ),
                PointLight(intensity=2.0, position=(2, light_y, 3)),
                Mesh(
                    SphereGeometry(0.5),
                    MeshStandardMaterial(Color(0xFFFFFF)),
                    position=(2, 1.5, 3),
                ),
            )
            return sc

        cam = self.shadow_cam()
        r = Renderer(64, 64, aa=1)
        above = r.render(scene(4.0), cam)[32, 32, 0].item()
        between = r.render(scene(0.5), cam)[32, 32, 0].item()
        assert above < 10 < between - 40
