"""CSG (CsgGeometry) 与实体协议测试: 区间布尔数值断言。

覆盖: difference / intersection / union / 半空间剖切 / 嵌套 CSG /
仿射包装叶子 / 网格叶子 / 阴影一致性。
"""

import mlx.core as mx
import pytest

from cga.engine import (
    AffineGeometry,
    BoxGeometry,
    CylinderGeometry,
    MeshGeometry,
    PlaneGeometry,
    SphereGeometry,
)
from cga.engine.csg import CsgGeometry
from cga.modeling import extrude
from cga.motors import Motor

RAY_O = mx.array([[0.0, 0.0, 5.0]])
RAY_D = mx.array([[0.0, 0.0, -1.0]])


def _hit(geo, o=RAY_O, d=RAY_D):
    p = geo.to_camera(Motor.identity())
    return geo.intersect(p, o, d)


class TestCsgBasics:
    def test_difference_nearest_surface(self):
        # 球[−1,1] − 盒[−0.5,0.5]: 沿 z 轴最近表面 = 球进入面 t=4
        g = CsgGeometry("difference", [SphereGeometry(1.0), BoxGeometry(1, 1, 1)])
        t, _n, m = _hit(g)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(4.0, abs=1e-5)

    def test_intersection_interval(self):
        # 球 ∩ 盒: 区间 [4.5, 5.5] → t=4.5
        g = CsgGeometry("intersection", [SphereGeometry(1.0), BoxGeometry(1, 1, 1)])
        t, _n, m = _hit(g)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(4.5, abs=1e-5)

    def test_union_is_nearest_of_children(self):
        a = SphereGeometry(1.0)  # z ±1 → t=4
        g = CsgGeometry(
            "union",
            [
                a,
                AffineGeometry(SphereGeometry(1.0), ((1, 0, 0), (0, 1, 0), (0, 0, 3))),
            ],
        )
        # 并集: 拉伸球 (z 半轴 3) 的进入面 t=2 更近
        t, _n, m = _hit(g)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(2.0, abs=1e-4)

    def test_difference_through_hole_misses(self):
        # 盒[0,2]³ − 中心竖孔: 孔轴上的射线穿孔 (减到无限长圆柱即全穿)
        g = CsgGeometry(
            "difference",
            [BoxGeometry(2, 2, 2), CylinderGeometry(0.3, 10.0)],
        )
        t, _n, m = _hit(g)
        assert not m.tolist()[0]
        assert t.tolist()[0] == float("inf")

    def test_halfspace_section(self):
        # 球 ∩ (y<0 半空间): 从 +y 向 −y, 表面 = 剖切平面 y=0 → t=5
        g = CsgGeometry(
            "intersection", [SphereGeometry(1.0), PlaneGeometry((0, 1, 0), 0.0)]
        )
        o = mx.array([[0.0, 5.0, 0.0]])
        d = mx.array([[0.0, -1.0, 0.0]])
        t, _n, m = _hit(g, o, d)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(5.0, abs=1e-5)

    def test_nested_csg(self):
        # (球 − 盒) ∩ (y<0 半空间): 沿 z 轴 (略偏 y=−0.2 避开剖切面退化),
        # 球−盒 进入面 t=4 在半空间内 → t=4
        inner = CsgGeometry("difference", [SphereGeometry(1.0), BoxGeometry(1, 1, 1)])
        g = CsgGeometry("intersection", [inner, PlaneGeometry((0, 1, 0), 0.0)])
        # 球在 y=−0.2 处截面半径 sqrt(1−0.04) → t = 5 − sqrt(0.96)
        t, _n, m = _hit(g, mx.array([[0.0, -0.2, 5.0]]), RAY_D)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(5.0 - 0.96**0.5, abs=1e-4)

    def test_affine_leaf(self):
        # 缩放球 (x2) − 盒: 沿 z 不变 t=4; 沿 x 半宽 2
        g = CsgGeometry(
            "difference",
            [
                AffineGeometry(SphereGeometry(1.0), ((2, 0, 0), (0, 1, 0), (0, 0, 1))),
                AffineGeometry(SphereGeometry(0.5), ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
            ],
        )
        t, _n, m = _hit(g)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(4.0, abs=1e-4)

    def test_shadow_matches_intersect(self):
        g = CsgGeometry("difference", [SphereGeometry(1.0), BoxGeometry(1, 1, 1)])
        p = g.to_camera(Motor.identity())
        t1, _n, m1 = g.intersect(p, RAY_O, RAY_D)
        t2, m2 = g.intersect_shadow(p, RAY_O, RAY_D)
        assert t1.tolist() == t2.tolist()
        assert m1.tolist() == m2.tolist()


class TestCsgValidation:
    def test_rejects_non_solid_leaf(self):
        from cga.engine import CircleGeometry

        with pytest.raises(TypeError):
            CsgGeometry("difference", [SphereGeometry(1.0), CircleGeometry(0.5)])

    def test_rejects_bad_op_and_arity(self):
        with pytest.raises(ValueError, match="op"):
            CsgGeometry("xor", [SphereGeometry(1.0), SphereGeometry(2.0)])
        with pytest.raises(ValueError, match=">= 2"):
            CsgGeometry("union", [SphereGeometry(1.0)])


class TestCsgMeshLeaf:
    def test_mesh_difference(self):
        # 挤出块 [0,2]²×[0,1] − 角落圆柱: 块中心 t=4, 孔轴 miss
        block = MeshGeometry(*extrude([(0, 0), (2, 0), (2, 2), (0, 2)], 1.0))
        g = CsgGeometry("difference", [block, CylinderGeometry(0.4, 3.0)])
        p = g.to_camera(Motor.identity())
        o = mx.array([[1.0, 1.0, 5.0]])
        t, _n, m = g.intersect(p, o, RAY_D)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(4.0, abs=1e-4)
        o2 = mx.array([[0.0, 0.0, 5.0]])
        _t2, _n2, m2 = g.intersect(p, o2, RAY_D)
        assert not m2.tolist()[0]
