"""CGS v2 自检: 变量/表达式/数学函数/for+range/module/if-else/echo/union。"""

import pytest

from cga.engine import SphereGeometry
from cga.scene_lang import SceneLoader
from tests.checks import Checks


class TestSceneLangV2(Checks):
    """OpenSCAD 对齐特性 (v2) 的语义断言。"""

    def test_variables_and_expressions(self):
        scene, _cam = SceneLoader.load(
            "r = 0.4 + 0.1;\n"
            "x = 2 * (r + 0.5);\n"  # 优先级: 2*(1.0) = 2.0
            "translate([x, 0, 0]) sphere(r=r);"
        )
        obj = scene.objects[0]
        assert isinstance(obj.geometry, SphereGeometry)
        assert self.close(obj.motor().to_matrix()[0][3], 2.0)

    def test_unary_minus_and_vector_arith(self):
        scene, _cam = SceneLoader.load(
            "base = [1, 2, 3];\ntranslate([-1, 0, 0] + base * 2) sphere(r=0.5);"
        )
        # [-1,0,0] + [2,4,6] = [1,4,6]
        m = scene.objects[0].motor().to_matrix()
        assert self.close(m[0][3], 1.0)
        assert self.close(m[1][3], 4.0)
        assert self.close(m[2][3], 6.0)

    def test_math_functions(self):
        scene, _cam = SceneLoader.load(
            "translate([sin(pi / 2), sqrt(16), max(3, 7)]) sphere(r=1);"
        )
        m = scene.objects[0].motor().to_matrix()
        assert self.close(m[0][3], 1.0)
        assert self.close(m[1][3], 4.0)
        assert self.close(m[2][3], 7.0)

    def test_for_range(self):
        scene, _cam = SceneLoader.load(
            "for (i = [0:2]) translate([i, 0, 0]) sphere(r=0.1);"
            "for (j = [0:0.5:1]) translate([0, j, 0]) sphere(r=0.1);"
        )
        assert len(scene.objects) == 6  # [0:2]→3 个 + [0:0.5:1]→3 个
        xs = [o.motor().to_matrix()[0][3] for o in scene.objects[:3]]
        assert xs == [0.0, 1.0, 2.0]
        ys = [o.motor().to_matrix()[1][3] for o in scene.objects[3:]]
        assert all(self.close(a, b) for a, b in zip(ys, [0.0, 0.5, 1.0]))

    def test_module_with_defaults(self):
        scene, _cam = SceneLoader.load(
            "module bead(r, gap=1.0) { translate([r * gap, 0, 0]) sphere(r=r); }\n"
            "bead(0.5);\n"
            "bead(0.5, gap=4.0);\n"
            "bead(r=0.25);"
        )
        xs = [o.motor().to_matrix()[0][3] for o in scene.objects]
        assert all(self.close(a, b) for a, b in zip(xs, [0.5, 2.0, 0.25]))

    def test_module_inherits_caller_transform(self):
        # module 调用点的 motor 上下文正常传入
        scene, _cam = SceneLoader.load(
            "module ball() { sphere(r=1); }\ntranslate([3, 0, 0]) ball();"
        )
        assert self.close(scene.objects[0].motor().to_matrix()[0][3], 3.0)

    def test_if_else(self):
        scene, _cam = SceneLoader.load(
            "which = 2;\n"
            "if (which == 1) { sphere(r=1); } else { sphere(r=2); }\n"
            "if (which > 1) sphere(r=3);"
        )
        from cga.algebra import Sphere
        from cga.engine import SphereGeometry

        radii = []
        for o in scene.objects:
            assert isinstance(o.geometry, SphereGeometry)
            _c, r = Sphere.from_dual(o.geometry.blade)
            radii.append(r)
        assert all(self.close(a, b) for a, b in zip(radii, [2.0, 3.0]))

    def test_echo(self, capsys):
        SceneLoader.load("x = 1 + 2;\necho(x, [1, 2] * 2);")
        out = capsys.readouterr().out
        assert "ECHO: 3.0 [2.0, 4.0]" in out

    def test_union_is_grouping(self):
        scene, _cam = SceneLoader.load(
            "union() { sphere(r=1); translate([2, 0, 0]) sphere(r=1); }"
        )
        assert len(scene.objects) == 2

    def test_v2_errors(self):
        with pytest.raises(ValueError, match="未定义变量"):
            SceneLoader.load("sphere(r=nope);")
        with pytest.raises(ValueError, match="未知函数"):
            SceneLoader.load("sphere(r=frob(1));")
        with pytest.raises(ValueError, match="缺参数"):
            SceneLoader.load("module m(r) { sphere(r=r); }\nm();")
        with pytest.raises(ValueError, match="步长不能为 0"):
            SceneLoader.load("for (i = [0:0:1]) sphere(r=1);")  # 步长=0

    def test_examples_grid(self):
        # examples/grid.cgs: module + for 的阵列示例, 3×3 = 9 球
        from pathlib import Path

        root = Path(__file__).resolve().parent.parent
        text = (root / "examples" / "grid.cgs").read_text(encoding="utf-8")
        scene, _cam = SceneLoader.load(text)
        assert len(scene.objects) == 10  # 地面 + 3×3 球
