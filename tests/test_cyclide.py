"""Dupin cyclide 测试: blade 构造 (球族包络 / versor 反演) + 引擎解析求交。

守门: 环型 cyclide (a=1, b=0.98, d=0.3, c=√(a²−b²)≈0.199) 满足 c<d<a。
"""

import math

import mlx.core as mx
import pytest

from cga.algebra import DupinCyclide, Point, Sphere
from cga.engine import CsgGeometry, CyclideGeometry
from cga.engine.geometry_base import Solid
from cga.motors import Motor

A, B, D = 1.0, 0.98, 0.3
C = math.sqrt(A * A - B * B)  # ≈ 0.198997

# 规范环型 cyclide 与 x 轴的 4 交点 (降序)
X1 = A + D - C
X2 = A - D + C
X3 = D + C - A
X4 = -A - D - C


class TestAlgebraModel:
    def test_param_implicit_consistency(self):
        cy = DupinCyclide(A, B, D)
        for u, v in [(0.0, 0.0), (1.2, 2.3), (3.1, 5.4), (math.pi, 0.5)]:
            x, y, z = cy.surface(u, v)
            assert cy.implicit(x, y, z) == pytest.approx(0.0, abs=1e-10)

    def test_kind(self):
        assert DupinCyclide(A, B, D).kind == "ring"  # c < d < a
        assert DupinCyclide(1.0, 0.6, 1.5).kind == "spindle"  # d > a
        assert DupinCyclide(1.0, 0.6, 0.3).kind == "horn"  # d < c

    def test_invalid_params_rejected(self):
        with pytest.raises(ValueError):
            DupinCyclide(1.0, 1.2, 0.3)  # b > a
        with pytest.raises(ValueError):
            DupinCyclide(1.0, 0.6, -1.0)  # d <= 0

    def test_focal_sphere_tangency(self):
        cy = DupinCyclide(A, B, D)
        for u in [0.0, 1.0, 2.0, 4.0]:
            r1, r2 = cy.tangency_residual(u)
            assert r1 == pytest.approx(0.0, abs=1e-10)
            assert r2 == pytest.approx(0.0, abs=1e-10)

    def test_generator_sphere_is_blade(self):
        cy = DupinCyclide(A, B, D)
        s = cy.generator_sphere(0.7)
        ctr, r = Sphere.from_dual(s)  # float32 blade → ~1e-7 精度
        assert ctr == pytest.approx(cy.spine(0.7), abs=1e-5)
        assert r == pytest.approx(cy.radius(0.7), abs=1e-5)

    def test_focal_spheres_are_blades(self):
        cy = DupinCyclide(A, B, D)
        (s1, s2) = cy.focal_spheres()
        (c1, r1) = Sphere.from_dual(s1)
        (c2, r2) = Sphere.from_dual(s2)
        assert c1 == pytest.approx((C, 0.0, 0.0), abs=1e-5)
        assert r1 == pytest.approx(A - D, abs=1e-5)
        assert c2 == pytest.approx((-C, 0.0, 0.0), abs=1e-5)
        assert r2 == pytest.approx(A + D, abs=1e-5)

    def test_characteristic_circle_on_surface(self):
        """特征圆 = 相邻球 meet, 圆上点落在 cyclide 曲面 (隐式=0)。"""
        cy = DupinCyclide(A, B, D)
        u = 0.9
        E = cy.spine(u)
        Ep = (-cy.a * math.sin(u), cy.b * math.cos(u), 0.0)
        r = cy.radius(u)
        rp = cy.c * math.sin(u)
        ep2 = sum(e * e for e in Ep)
        lam = r * rp / ep2
        center = tuple(E[i] - lam * Ep[i] for i in range(3))
        radius = math.sqrt(r * r - lam * lam * ep2)
        nn = math.sqrt(ep2)
        n = tuple(e / nn for e in Ep)
        e1 = (n[1], -n[0], 0.0)  # 与 n (xy 平面内) 正交
        e2 = (
            n[1] * e1[2] - n[2] * e1[1],
            n[2] * e1[0] - n[0] * e1[2],
            n[0] * e1[1] - n[1] * e1[0],
        )
        cc = cy.characteristic_circle(u)
        for t in [0.0, 1.3, 2.5]:
            p = tuple(
                center[i] + radius * (e1[i] * math.cos(t) + e2[i] * math.sin(t))
                for i in range(3)
            )
            assert cy.implicit(*p) == pytest.approx(0.0, abs=1e-8)
            assert float(Point(*p).ip(cc).values[0]) == pytest.approx(0.0, abs=1e-6)

    def test_adjacent_spheres_meet_incident(self):
        """相邻球 S(u), S(u+du) 的 meet 与解析特征圆同圆 (关联判据)。"""
        cy = DupinCyclide(A, B, D)
        u = 0.9
        meet = cy.generator_sphere(u).dual().meet(cy.generator_sphere(u + 1e-5).dual())
        cc = cy.characteristic_circle(u)
        # 特征圆上一点既在 Circle blade 上, 也在 meet (直接形式) 上
        E = cy.spine(u)
        Ep = (-cy.a * math.sin(u), cy.b * math.cos(u), 0.0)
        r = cy.radius(u)
        lam = r * cy.c * math.sin(u) / sum(e * e for e in Ep)
        center = tuple(E[i] - lam * Ep[i] for i in range(3))
        # 单位化 Ep 作圆平面法向, 取一正交向量
        nn = math.sqrt(sum(e * e for e in Ep))
        e1 = (Ep[1] / nn, -Ep[0] / nn, 0.0)
        radius = math.sqrt(r * r - lam * lam * sum(e * e for e in Ep))
        p = tuple(center[i] + radius * e1[i] for i in range(3))
        pt = Point(*p)
        assert float(pt.ip(cc).values[0]) == pytest.approx(0.0, abs=1e-6)
        assert float(pt.op(meet).values[0]) == pytest.approx(0.0, abs=1e-6)

    def test_uv_roundtrip(self):
        cy = DupinCyclide(A, B, D)
        for u0, v0 in [(0.5, 1.0), (2.0, 4.0), (5.0, 0.2)]:
            x, y, z = cy.surface(u0, v0)
            u1, v1 = cy.uv(x, y, z)
            # u 精确 (atan2 归一到 (−π,π]), v 相对参数化差一个镜像
            assert (u1 - u0) % (2 * math.pi) == pytest.approx(0.0, abs=1e-6)
            assert (v1 + v0) % (2 * math.pi) == pytest.approx(0.0, abs=1e-6)

    def test_from_torus_inversion(self):
        cy = DupinCyclide.from_torus_inversion(2.0, 0.5, shift_x=1.0)
        assert cy.kind == "ring"

        def torus(u, v):
            R, r, s = 2.0, 0.5, 1.0
            return (
                s + (R + r * math.cos(v)) * math.cos(u),
                (R + r * math.cos(v)) * math.sin(u),
                r * math.sin(v),
            )

        for u, v in [(0.7, 1.9), (2.1, 0.3)]:
            tx, ty, tz = torus(u, v)
            n2 = tx * tx + ty * ty + tz * tz
            assert cy.implicit(tx / n2, ty / n2, tz / n2) == pytest.approx(
                0.0, abs=1e-9
            )

    def test_from_torus_inversion_centered_is_degenerate(self):
        with pytest.raises(ValueError, match="shift_x"):
            DupinCyclide.from_torus_inversion(2.0, 0.5)  # shift_x=0 → c=0 环面

    def test_inversion_versor(self):
        for p in [(2.0, 0.0, 0.0), (1.0, 2.0, 3.0), (-0.5, 0.25, 1.5)]:
            q = DupinCyclide.invert_point(Point(*p))  # float32 sandwich
            r2 = sum(x * x for x in p)
            assert q.coords() == pytest.approx(tuple(x / r2 for x in p), abs=1e-6)


class TestEngine:
    def test_ray_hits_implicit(self):
        g = CyclideGeometry(A, B, D)
        p = g.to_camera(Motor.identity())
        o = mx.array([[3.0, 0.0, 0.0]])
        d = mx.array([[-1.0, 0.0, 0.0]])
        t, _n, m = g.intersect(p, o, d)
        assert m.tolist()[0]
        cy = DupinCyclide(A, B, D)
        assert cy.implicit(3.0 - float(t[0]), 0.0, 0.0) == pytest.approx(0.0, abs=1e-3)

    def test_axis_ray_four_crossings(self):
        g = CyclideGeometry(A, B, D)
        p = g.to_camera(Motor.identity())
        ts, ns, valid = g.crossings(
            p, mx.array([[3.0, 0.0, 0.0]]), mx.array([[-1.0, 0.0, 0.0]])
        )
        assert valid.tolist()[0] == [True, True, True, True]
        exp = sorted([3.0 - x for x in (X1, X2, X3, X4)])
        assert [float(x) for x in ts[0]] == pytest.approx(exp, abs=1e-2)

    def test_center_hole_ray_misses(self):
        # 过中心的 +z 射线穿过孔, 不命中 (类似环面)
        g = CyclideGeometry(A, B, D)
        p = g.to_camera(Motor.identity())
        _t, _n, m = g.intersect(
            p, mx.array([[0.0, 0.0, 5.0]]), mx.array([[0.0, 0.0, -1.0]])
        )
        assert not m.tolist()[0]

    def test_contains(self):
        g = CyclideGeometry(A, B, D)
        p = g.to_camera(Motor.identity())
        res = g.contains(
            p, mx.array([[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
        )
        assert res.tolist() == [True, False, False]

    def test_shift(self):
        # 平移 shift 后的 cyclide: 中心孔 x 处 miss, 平移处 hit
        g = CyclideGeometry(A, B, D, shift=(1.0, 0.0, 0.0))
        p = g.to_camera(Motor.identity())
        inside = g.contains(p, mx.array([[2.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))
        assert inside.tolist() == [True, False]

    def test_bounds_contain_surface(self):
        g = CyclideGeometry(A, B, D)
        p = g.to_camera(Motor.identity())
        lo, hi = g.bounds_camera(p)
        cy = DupinCyclide(A, B, D)
        for i in range(7):
            for j in range(7):
                x, y, z = cy.surface(i * 2 * math.pi / 6, j * 2 * math.pi / 6)
                assert lo[0] <= x <= hi[0]
                assert lo[1] <= y <= hi[1]
                assert lo[2] <= z <= hi[2]

    def test_is_solid_and_csg_combines(self):
        g = CyclideGeometry(A, B, D)
        assert isinstance(g, Solid)
        csg = CsgGeometry("union", [CyclideGeometry(A, B, D), CyclideGeometry(A, B, D)])
        p = csg.to_camera(Motor.identity())
        t, _n, m = csg.intersect(
            p, mx.array([[3.0, 0.0, 0.0]]), mx.array([[-1.0, 0.0, 0.0]])
        )
        assert m.tolist()[0]
        assert float(t[0]) > 0.0


class TestCgs:
    def test_cyclide_primitive(self):
        from cga.scene_lang import SceneLoader

        sc, _ = SceneLoader.load("cyclide(a=1, b=0.98, d=0.3);")
        assert isinstance(sc.objects[0].geometry, CyclideGeometry)

    def test_cyclide_in_csg(self):
        from cga.engine.csg import CsgGeometry
        from cga.scene_lang import SceneLoader

        sc, _ = SceneLoader.load(
            "difference() { box(s=[4,4,4]); cyclide(a=1, b=0.98, d=0.3); }"
        )
        assert isinstance(sc.objects[0].geometry, CsgGeometry)
