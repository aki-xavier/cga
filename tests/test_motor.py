"""Motor versor 自检: 平移/旋转/复合/逆/exp-log/插值/速度/矩阵往返。"""

import math

import mlx.core as mx
import pytest

from cga import Motor, Multivector, Plane, Point
from tests.checks import Checks


class TestMotor(Checks):
    """刚体变换 versor (O' = M·O·M̃) 的代数性质。"""

    def test_translator(self):
        t = Motor.translator((1, 2, 3))
        assert self.close(t.apply(Point(0, 0, 0)).dist(Point(1, 2, 3)), 0)

    def test_rotor(self):
        r = Motor.rotor((0, 0, 1), math.pi / 2)
        assert self.close(r.apply(Point(1, 0, 0)).dist(Point(0, 1, 0)), 0)

    def test_compose_rot_trans(self):
        m = Motor((0, 0, 1), math.pi / 2, (1, 0, 0))  # M = T·R
        assert self.close(m.apply(Point(1, 0, 0)).dist(Point(1, 1, 0)), 0)

    def test_preserves_incidence(self):
        # versor 保持关联: 平移后的线仍过平移后的点
        from cga import Line

        line = Line(Point(0, 0, 0), Point(1, 0, 0))
        t = Motor.translator((1, 2, 3))
        assert self.close(self.vmax(t.apply(Point(2, 0, 0)).op(t.apply(line))), 0)

    def test_preserves_meet(self):
        # 先交后变换 = 先变换后交 (整体符号/尺度不定, 归一后比)
        pi = Plane((0, 0, 1), 2.0)
        pi2 = Plane((0, 1, 0), 1.0)
        t = Motor.translator((1, 2, 3))
        lhs = t.apply(pi.dual().meet(pi2.dual()))
        rhs = t.apply(pi).dual().meet(t.apply(pi2).dual())
        a, b = lhs.values, rhs.values
        scale = float(mx.abs(b).max().item())
        assert bool(mx.allclose(a / scale, b / scale, atol=1e-4).item()) or bool(
            mx.allclose(a / scale, -b / scale, atol=1e-4).item()
        )

    def test_to_matrix(self):
        m = Motor((0, 0, 1), math.pi / 2, (1, 0, 0))
        h = m.to_matrix()
        p_h = [h[i][0] + h[i][3] for i in range(3)]  # M·(1,0,0,1) = R[i][0] + t[i]
        assert all(abs(p_h[i] - [1, 1, 0][i]) < 1e-4 for i in range(3))

    @pytest.mark.parametrize(
        "axis,ang,t",
        [
            ((0, 0, 1), 0.7, (1, 2, 3)),
            ((1, 1, 1), 2.1, (-0.5, 0.25, 0.1)),
            ((0.3, -0.8, 0.2), 3.14159, (0, 0, 0)),
        ],
    )
    def test_from_matrix_roundtrip(self, axis, ang, t):
        # from_matrix ∘ to_matrix = 恒等 (符号约定钉死)
        t4 = Motor(axis, ang, t).to_matrix()
        r3 = [row[:3] for row in t4[:3]]
        m_rt = Motor.from_matrix(r3, (t4[0][3], t4[1][3], t4[2][3]))
        r4 = m_rt.to_matrix()
        assert all(
            self.close(r4[i][j], r3[i][j], tol=2e-3) for i in range(3) for j in range(3)
        )
        assert all(self.close(r4[i][3], t4[i][3], tol=2e-3) for i in range(3))

    def test_inverse(self):
        m2 = Motor((0.2, 0.5, -0.3), 1.3, (0.4, -0.1, 0.2))
        p0 = Point(0.7, -0.2, 0.5)
        # M·M⁻¹ = identity (点不动)
        assert self.close(self.diff_max(m2.inverse().apply(m2.apply(p0)), p0), 0)
        # 二次 inverse 复原
        assert self.close(
            self.diff_max(m2.inverse().inverse().apply(p0), m2.apply(p0)), 0
        )

    def test_exp_log_roundtrip(self):
        r90 = Motor.rotor((0, 0, 1), math.pi / 2)
        r45 = Motor.rotor((0, 0, 1), math.pi / 4)
        p2 = Point(1, 0, 0)
        b90 = r90.log()
        assert self.close(Motor.exp(b90).apply(p2).dist(r90.apply(p2)), 0)
        # Motor.exp(B, s) = exp(-s·B): s=0.5 → 半个 motor
        assert self.close(Motor.exp(b90, 0.5).apply(p2).dist(r45.apply(p2)), 0)
        # 负 scale = 逆
        r_neg45 = Motor.rotor((0, 0, 1), -math.pi / 4)
        assert self.close(Motor.exp(b90, -0.5).apply(p2).dist(r_neg45.apply(p2)), 0)

    def test_interpolate_midpoint(self):
        r90 = Motor.rotor((0, 0, 1), math.pi / 2)
        r45 = Motor.rotor((0, 0, 1), math.pi / 4)
        p2 = Point(1, 0, 0)
        mid = Motor.identity().interpolate(r90, 0.5)
        assert self.close(mid.apply(p2).dist(r45.apply(p2)), 0)

    def test_screw_exp_log_roundtrip(self):
        # 螺旋运动 (非零节距): SE(3) 指数/对数精确往返
        m_screw = Motor((0, 0, 1), 0.4, (0.3, -0.2, 0.1))
        m_rt = Motor.exp(m_screw.log())
        p2 = Point(1, 0, 0)
        assert self.close(m_rt.apply(p2).dist(m_screw.apply(p2)), 0)

    def test_extract_velocity(self):
        dt = 0.1
        ident = Motor.identity()
        # 纯平移: 幅值 0.03/0.1 = 0.3, 角速度零
        mv = Motor.translator((0.03, 0.0, 0.0))
        (w_v, v_v) = Motor.extract_velocity(mv.gp(ident), ident, dt)
        assert self.close(v_v[0], 0.3) and self.close(v_v[1], 0.0)
        assert self.close(w_v[2], 0.0)
        # 纯旋转: 0.2 rad / 0.1 s = 2.0 rad/s, 符号为正 (body frame)
        (w_r, _v_r) = Motor.extract_velocity(
            Motor.rotor((0, 0, 1), 0.2).gp(ident), ident, dt
        )
        assert self.close(w_r[2], 2.0)

    def test_direction_vector_rotor_only(self):
        # 方向向量共轭后 e1..e3 部分只由 rotor 决定 —— translator 只向
        # e∞ 槽写杂散项, 方向语义不受影响 (无穷远点语义)
        m3 = Motor((0, 0, 1), 0.6, (5, -3, 2))
        t3 = Motor.translator((2, 1, -4))
        d0 = Multivector.vector(0.3, -0.8, 0.5)
        a_vec = m3.apply(d0).euclidean_vector()
        b_vec = t3.compose(m3).apply(d0).euclidean_vector()
        assert self.close(
            max(abs(x - y) for x, y in zip(a_vec, b_vec, strict=True)),
            0,
            tol=1e-3,
        )
