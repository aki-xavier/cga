"""图元代数自检: null 性 / 关联判据 / 距离 / meet / 退化守卫 / from_dual。

约定: 点/点对/线为直接形式, 关联判据 p.op(X) = 0; 平面/球/圆为对偶
形式, 关联判据 p.ip(X) = 0。
"""

import pytest

from cga import Circle, Cylinder, Line, Motor, Multivector, Plane, Point, Sphere
from tests.checks import Checks


class TestPrimitives(Checks):
    """CGA 图元 (直接/对偶两种形式) 的代数性质。"""

    def test_point_null_and_dist(self):
        p1 = Point(0, 0, 0)
        assert self.close(p1.gp(p1).values[0], 0)  # null 向量 p·p = 0
        assert self.close(p1.dist(Point(3, 4, 0)), 5.0)

    def test_line_incidence(self):
        line = Line(Point(0, 0, 0), Point(1, 0, 0))
        assert self.close(self.vmax(Point(2, 0, 0).op(line)), 0)
        assert self.vmax(Point(0, 1, 0).op(line)) > 1e-3

    def test_plane_incidence(self):
        pi = Plane((0, 0, 1), 2.0)  # z = 2 平面
        assert self.close(self.vmax(Point(0.3, -0.7, 2).ip(pi)), 0)
        assert self.vmax(Point(0, 0, 0).ip(pi)) > 1e-3

    def test_sphere_incidence(self):
        s = Sphere((1, 2, 3), 2.0)
        assert self.close(self.vmax(Point(3, 2, 3).ip(s)), 0)
        assert self.vmax(Point(0, 0, 0).ip(s)) > 1e-3

    def test_circle_incidence(self):
        c = Circle((0, 0, 0), 1.0, (0, 0, 1))
        assert self.close(self.vmax(Point(0, 1, 0).ip(c)), 0)
        assert self.vmax(Point(0, 0, 1).ip(c)) > 1e-3
        # 非单位法向: d 须按单位法向计算 (回归: Plane 归一化 n 但不缩放 d)
        cnu = Circle((1, 2, 3), 2.0, (0, 0, 2))
        assert self.close(self.vmax(Point(3, 2, 3).ip(cnu)), 0)

    def test_degenerate_guards(self):
        with pytest.raises(ValueError):
            Plane((0, 0, 0), 1.0)  # 零法向
        with pytest.raises(ValueError):
            Circle((0, 0, 0), 1.0, (0, 0, 0))  # 零法向
        with pytest.raises(ValueError):
            Multivector.bivector([1.0, 2.0])  # 分量数不足

    def test_distances(self):
        pi = Plane((0, 0, 1), 2.0)
        s = Sphere((1, 2, 3), 2.0)
        assert self.close(Point(0, 0, 5).dist(pi), 3.0)
        assert self.close(pi.dist(Point(0, 0, 5)), 3.0)  # plane 侧调用
        assert self.close(Point(3, 2, 3).dist(s), 0)  # 球面上
        assert Point(5, 2, 3).dist(s) > 0  # 球外为正
        assert Point(1, 2, 3).dist(s) < 0  # 球内为负

    def test_meet_plane_plane(self):
        # 两平面交线: y=1, z=2, 沿 x 方向 (对偶原语先过 dual())
        pi = Plane((0, 0, 1), 2.0)
        pi2 = Plane((0, 1, 0), 1.0)
        lm = pi.dual().meet(pi2.dual())
        assert self.close(self.vmax(Point(0, 1, 2).op(lm)), 0)
        assert self.close(self.vmax(Point(5, 1, 2).op(lm)), 0)

    def test_meet_line_sphere(self):
        # z 轴与单位球交于 (0,0,±1)
        lz = Line(Point(0, 0, -2), Point(0, 0, 2))
        ppm = lz.meet(Sphere((0, 0, 0), 1.0).dual())
        assert self.close(self.vmax(Point(0, 0, 1).op(ppm)), 0)
        assert self.close(self.vmax(Point(0, 0, -1).op(ppm)), 0)

    def test_far_from_origin_dist(self):
        # float32 conformal 内积会灾难性抵消 → 0, 距离走 float64 欧氏公式
        assert self.close(Point(1000, 0, 0).dist(Point(1001, 0, 0)), 1.0, tol=1e-2)

    def test_cylinder_distances(self):
        cy = Cylinder((0.0, 0.0, 2.0), (0.0, 1.0, 0.0), 1.0)  # 轴 ∥ Y 过 (0,0,2)
        assert self.close(cy.dist(Point(1.0, 5.0, 2.0)), 0.0)  # 柱面上
        assert self.close(cy.dist(Point(0.2, 0.0, 2.0)), -0.8)  # 柱内
        assert self.close(cy.dist(Point(3.0, -2.0, 2.0)), 2.0)  # 柱外
        assert self.close(cy.dist(Point(-1.0, 5.0, 2.0)), 0.0)  # 对称侧

    def test_from_dual_after_motor(self):
        # motor 共轭后类型降级为 Multivector, 提取走公开访问器/类方法
        s_cam = Motor.translator((1, 2, 3)).apply(Sphere((0, 0, 0), 0.5))
        (c0, c1, c2), rho = Sphere.from_dual(s_cam)
        assert self.close(c0, 1.0) and self.close(c1, 2.0) and self.close(c2, 3.0)
        assert self.close(rho, 0.5)
        pi_cam = Motor.translator((1, 2, 3)).apply(Plane((0, 1, 0), 0.0))
        assert self.close(float(pi_cam.einf_coeff()), 2.0)
