"""CGS 场景语言自检: 图元/变换复合/块作用域/材质/灯光/相机/错误/金样等价。"""

import importlib.util
import math
from pathlib import Path

import pytest

from cga.engine import (
    AmbientLight,
    BoxGeometry,
    CircleGeometry,
    CylinderGeometry,
    DirectionalLight,
    MeshBasicMaterial,
    MeshStandardMaterial,
    PerspectiveCamera,
    PlaneGeometry,
    PointLight,
    Renderer,
    SphereGeometry,
)
from cga.motors import Motor
from cga.scene_lang import SceneLoader
from tests.checks import Checks

ROOT = Path(__file__).resolve().parent.parent


class TestSceneLang(Checks):
    """CGS (OpenSCAD 风格场景语言) 的解析与求值。"""

    def test_primitives(self):
        scene, _cam = SceneLoader.load(
            "sphere(r=1);"
            "plane(n=[0,1,0], d=0.5);"
            "cylinder(r=0.7);"
            "cylinder(r=0.3, h=2.0);"
            "box(s=[1, 2, 3]);"
            "circle(r=0.9);"
        )
        kinds = [type(o.geometry) for o in scene.objects]
        assert kinds == [
            SphereGeometry, PlaneGeometry, CylinderGeometry,
            CylinderGeometry, BoxGeometry, CircleGeometry,
        ]
        cy_inf, cy_fin = scene.objects[2].geometry, scene.objects[3].geometry
        assert isinstance(cy_inf, CylinderGeometry) and cy_inf.half is None
        assert isinstance(cy_fin, CylinderGeometry)
        assert self.close(cy_fin.half, 1.0)  # h=2 → half=1

    def test_transform_composition(self):
        # translate 在外 → 先旋后移: Motor = T·R
        scene, _cam = SceneLoader.load(
            "translate([1, 2, 3]) rotate(axis=[0, 0, 1], angle=1.5707963267948966)"
            "  sphere(r=1);"
        )
        got = scene.objects[0].motor().to_matrix()
        exp = Motor((0, 0, 1), math.pi / 2, (1, 2, 3)).to_matrix()
        assert all(
            self.close(got[i][j], exp[i][j], tol=1e-5)
            for i in range(4)
            for j in range(4)
        )

    def test_nested_block_scoping(self):
        # 块内平移/材质不外泄: 第二球在原点、默认材质
        scene, _cam = SceneLoader.load(
            "material(color=0xFF0000) { translate([1, 0, 0]) sphere(r=1); }"
            "sphere(r=1);"
        )
        first, second = scene.objects
        assert self.close(first.motor().to_matrix()[0][3], 1.0)
        assert self.close(second.motor().to_matrix()[0][3], 0.0)
        assert isinstance(first.material, MeshStandardMaterial)
        assert first.material.color.r == 1.0
        assert isinstance(second.material, MeshStandardMaterial)
        assert second.material.color.r == 1.0  # 默认白 (1.0)
        assert second.material.color.g == 1.0  # 未被块内红色污染

    def test_material_fields_and_unlit(self):
        scene, _cam = SceneLoader.load(
            "material(color=0x2980B9, roughness=0.15, metalness=0.35, "
            "opacity=0.5, ior=1.4, absorption=0.3) sphere(r=1);"
            "material(color=0x00FF00, unlit=true) sphere(r=1);"
        )
        std, basic = scene.objects
        assert isinstance(std.material, MeshStandardMaterial)
        assert self.close(std.material.roughness, 0.15)
        assert self.close(std.material.opacity, 0.5)
        assert self.close(std.material.ior, 1.4)
        assert self.close(std.material.absorption, 0.3)
        assert isinstance(basic.material, MeshBasicMaterial)

    def test_lights_background_camera(self):
        scene, cam = SceneLoader.load(
            "background(color=0x101418);"
            "directional_light(direction=[0, 1, 0], intensity=0.5);"
            "point_light(position=[0, 4, 3], intensity=0.7);"
            "ambient_light(intensity=0.2);"
            "camera(fov=40, aspect=1.0, position=[0, 0, 5], target=[0, 0, 0]);"
        )
        assert [type(light) for light in scene.lights] == [
            DirectionalLight, PointLight, AmbientLight,
        ]
        assert self.close(scene.background.r, 0x10 / 255)
        assert isinstance(cam, PerspectiveCamera)
        assert self.close(cam.fov, 40.0)

    def test_errors(self):
        with pytest.raises(ValueError, match="未知语句"):
            SceneLoader.load("cube(r=1);")
        with pytest.raises(ValueError, match="缺少参数"):
            SceneLoader.load("sphere();")
        with pytest.raises(ValueError, match="期望"):
            SceneLoader.load("sphere(r=1)")  # 缺分号
        with pytest.raises(ValueError, match="缺少目标语句"):
            SceneLoader.load("translate([1, 0, 0]);")
        with pytest.raises(ValueError, match="第2行"):
            SceneLoader.load("sphere(r=1);\nblah;")

    def test_orbit_example_matches_python_build(self):
        # 金样: examples/orbit.cgs 与 demo_engine 的 Python 场景逐位等价
        spec = importlib.util.spec_from_file_location(
            "demo_engine", ROOT / "demo_engine.py"
        )
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        text = (ROOT / "examples" / "orbit.cgs").read_text(encoding="utf-8")
        scene, camera = SceneLoader.load(text)
        py_scene = mod.OrbitDemo.build_scene()
        py_cam = PerspectiveCamera(
            fov=50, aspect=4 / 3, position=(0, 2.4, 6.2), target=(0, 0.8, 0)
        )
        py_cam.look_at((0, 0.8, 0))
        r = Renderer(360, 270, aa=2)
        a = Renderer.frame_to_bytes(r.render(scene, camera))
        b = Renderer.frame_to_bytes(r.render(py_scene, py_cam))
        assert a == b, "CGS 场景应与 Python 构建逐位一致"
