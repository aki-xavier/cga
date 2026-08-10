"""CGA Motor — 共形空间的刚体变换 (OOP 表面)。

在 CGA 中, 刚体运动 (旋转 + 平移) 由偶次 grade 的 multivector
*motor* 表示。motor M 按 sandwich 积作用任意对象 O:

    O' = M O M̃   (M̃ = M.reverse())

基本 versor:
  - Rotor:      R = exp(-θ/2 · B)    绕二重向量平面 B 旋转 θ
  - Translator: T = 1 - (t/2)·e∞     平移向量 t
  - Motor:      M = T · R            复合刚体运动

刚体的速度状态是二重向量 V (6 自由度), 运动方程 dM/dt = -½·V·M。
"""

import math
from typing import TypeVar

import mlx.core as mx

from cga.algebra import E0, EINF, Point
from cga.multivector import NUM_COMPONENTS, Multivector

MV = TypeVar("MV", bound=Multivector)


class Motor(Multivector):
    """刚体变换 (偶次 versor): 作用 O' = M·O·M̃, 运动方程 dM/dt = −½·V·M。"""

    __slots__ = ()

    def __init__(
        self,
        rotation_axis: tuple[float, float, float] | None = None,
        rotation_angle: float = 0.0,
        translation: tuple[float, float, float] = (0, 0, 0),
    ):
        """M = T(translation)·R(axis, angle) (axis=None 或 angle=0 即无旋转)。"""
        if rotation_axis is not None and abs(rotation_angle) > 1e-12:
            ro = Motor.rotor(rotation_axis, rotation_angle)
        else:
            ro = Multivector.scalar(1.0)
        tx, ty, tz = translation
        if abs(tx) > 1e-12 or abs(ty) > 1e-12 or abs(tz) > 1e-12:
            tr = Motor.translator(translation)
        else:
            tr = Multivector.scalar(1.0)
        super().__init__(tr.gp(ro).values)  # M = T · R

    @classmethod
    def _wrap(cls, mv: Multivector) -> Motor:
        """把已有 multivector 标记为 Motor (不重算, 内部用)。"""
        self = cls.__new__(cls)
        self.values = mv.values
        return self

    # ── 构造 ──────────────────────────────────────────────────────

    @classmethod
    def identity(cls) -> Motor:
        """恒等 motor (无旋转无平移)。"""
        return cls._wrap(Multivector.scalar(1.0))

    @classmethod
    def rotor(cls, axis: tuple[float, float, float], angle: float) -> Motor:
        """绕 axis 旋转 angle (rad) 的 rotor (单位, grade 0+2)。

        R = cos(θ/2) − sin(θ/2)·(nx·e23 + ny·e31 + nz·e12), e31 = −e13。
        """
        ax, ay, az = axis
        norm_ax = math.sqrt(ax * ax + ay * ay + az * az)
        if norm_ax < 1e-12:
            return cls.identity()
        ax, ay, az = ax / norm_ax, ay / norm_ax, az / norm_ax
        half_angle = angle / 2.0
        s = math.cos(half_angle)
        sf = math.sin(half_angle)

        vals = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        vals[0] = s  # cos(θ/2)
        vals[6] = -sf * az  # e12: -nz
        vals[7] = sf * ay  # e13: +ny  (since -ny*e31 = +ny*e13)
        vals[10] = -sf * ax  # e23: -nx
        return cls._wrap(Multivector(vals))

    @classmethod
    def from_quaternion(cls, q: tuple[float, float, float, float]) -> Motor:
        """由 (w, x, y, z) 四元数 (MJCF 约定) 构造 rotor。"""
        w, x, y, z = q
        n = math.sqrt(w * w + x * x + y * y + z * z)
        if n < 1e-12:
            return cls.identity()
        w, x, y, z = w / n, x / n, y / n, z / n
        angle = 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), w)
        return cls.rotor((x, y, z), angle)

    @classmethod
    def translator(cls, displacement: tuple[float, float, float]) -> Motor:
        """平移 displacement 的 translator: T = 1 − (t ∧ e∞)/2。"""
        tx, ty, tz = displacement
        tv = Multivector.vector(tx, ty, tz)
        return cls._wrap(Multivector.scalar(1.0) - tv.op(EINF) * 0.5)

    @classmethod
    def from_matrix(cls, R, t) -> Motor:
        """由 3x3 旋转矩阵与平移向量构造: M = T(t)·R,
        保证 Motor.from_matrix(R, t).to_matrix() == [R|t]。"""
        tv = tuple(float(x) for x in mx.array(t, dtype=mx.float32))
        return cls._wrap(
            cls.translator(tv).gp(cls.from_quaternion(cls._matrix_to_quaternion(R)))
        )

    @classmethod
    def exp(cls, B: Multivector, scale: float = 1.0) -> Motor:
        """二重向量指数: exp(−scale·B), 结果是一个 motor。

        分解 B 的旋转/平移部分, 走 SE(3) 指数映射 (Rodrigues + SO(3)
        左雅可比), 对一般螺旋运动 (非零节距) 精确——闭式 B² 符号分类
        只对纯旋转/纯平移成立。B 的分量约定为半 twist:
        B = ½(ω̄_bivector + v̄∧e∞), 与运动方程 dM/dt = −½·V·M 一致。

          - 纯平移 (ω̄ = 0): 1 − scale·B    (B² = 0, 级数截断, 精确)
          - 纯旋转 (v̄ = 0): rotor(ω̄/|ω̄|, |ω̄|)
          - 一般螺旋:        T(V̄·v̄) · R(rodrigues(ω̄))
        """
        Bv = B * scale
        vals = Bv.values
        wx, wy, wz = float(vals[10]), -float(vals[7]), float(vals[6])  # e23,e31,e12
        vx, vy, vz = float(vals[9]), float(vals[12]), float(vals[14])  # e_i∧e∞

        w_bar = mx.array([2.0 * wx, 2.0 * wy, 2.0 * wz], dtype=mx.float32)
        v_bar = mx.array([2.0 * vx, 2.0 * vy, 2.0 * vz], dtype=mx.float32)

        theta = float(mx.sqrt((w_bar * w_bar).sum()))
        v_norm = float(mx.sqrt((v_bar * v_bar).sum()))
        if theta < 1e-12:
            if v_norm < 1e-12:
                return cls.identity()
            # 纯平移: Bv 幂零, 级数截断
            return cls._wrap(Multivector.scalar(1.0) - Bv)
        if v_norm < 1e-12:
            # 纯旋转 (过原点)
            axis = (w_bar / theta).tolist()
            return cls.rotor((axis[0], axis[1], axis[2]), theta)

        # 一般螺旋: Rodrigues + SO(3) 左雅可比
        bx, by, bz = float(w_bar[0]), float(w_bar[1]), float(w_bar[2])
        W = mx.array(
            [
                [0.0, -bz, by],
                [bz, 0.0, -bx],
                [-by, bx, 0.0],
            ],
            dtype=mx.float32,
        )
        WW = mx.matmul(W, W, stream=mx.cpu)  # CPU stream: GPU matmul 降精度
        theta2 = theta * theta
        sin_t, cos_t = math.sin(theta), math.cos(theta)
        a_r, b_r = sin_t / theta, (1.0 - cos_t) / theta2
        a_v, b_v = (1.0 - cos_t) / theta2, (theta - sin_t) / (theta2 * theta)
        eye = mx.eye(3)
        R = eye + a_r * W + b_r * WW
        V = eye + a_v * W + b_v * WW
        t = mx.matmul(V, v_bar, stream=mx.cpu).tolist()
        return cls.from_matrix(R, t)

    # ── 作用与提取 ────────────────────────────────────────────────

    def apply(self, obj: MV) -> MV:
        """O' = M·O·M̃ (M̃ = M.reverse())。versor 作用保持图元类型
        (变换后的点仍是点, 线仍是线), 图元子类输入返回同类。"""
        out = self.gp(obj).gp(self.reverse())
        cls = type(obj)
        if cls is Multivector:
            return out  # type: ignore[return-value]
        wrapped = cls.__new__(cls)
        wrapped.values = out.values
        return wrapped

    def log(self) -> Multivector:
        """对数: 二重向量 Bv 使 exp(−Bv) = self。

        走 SE(3) 矩阵对数 (含 θ≈π 的对称部分恢复分支), 对一般螺旋
        运动 (非零节距) 精确——"标量+二重向量"闭式只对纯旋转成立,
        纯平移 (幂零) 还会整体归零。结果分量约定为半 twist:
        Bv = ½(ω̄_bivector + v̄∧e∞), 与 dM/dt = −½·V·M 一致。
        """
        T = mx.array(self.to_matrix(), dtype=mx.float32)
        R = T[:3, :3]
        t = T[:3, 3]

        trace = float(mx.diagonal(R).sum())
        cos_theta = min(1.0, max(-1.0, (trace - 1.0) / 2.0))
        # θ 用 atan2 提取 (|antisym| = 2·sinθ): acos((trace−1)/2) 在
        # θ→0 时把 float32 trace 噪声 (~1e-7) 二次放大成 ~1e-4 的 θ 误差。
        antisym = mx.stack(
            [
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1],
            ]
        )
        sin_theta_abs = 0.5 * float(mx.sqrt((antisym * antisym).sum()))
        theta = math.atan2(sin_theta_abs, cos_theta)

        if theta < 1e-9:
            # 纯平移: v̄ = t
            w_bar = mx.zeros(3, dtype=mx.float32)
            v_bar = t
        else:
            sin_theta = math.sin(theta)
            if theta < math.pi - 1e-3:
                c = theta / (2.0 * sin_theta)
                w_bar = c * antisym
            else:
                # θ ≈ π: 反对称公式除以 sin θ 爆炸, 改从对称部分恢复转轴
                # (R = 2·aaᵀ − I → R[i][j] = 2·a_i·a_j, i≠j), 用最大的
                # 轴分量作符号参考 (对 float32 噪声最稳健)
                axis = mx.sqrt(mx.maximum((mx.diagonal(R) + 1.0) / 2.0, 0.0))
                axis_l = axis.tolist()
                R_l = R.tolist()
                ref = max(range(3), key=lambda i: abs(axis_l[i]))
                if ref == 0:
                    axis_l[1] = math.copysign(axis_l[1], R_l[0][1])
                    axis_l[2] = math.copysign(axis_l[2], R_l[0][2])
                elif ref == 1:
                    axis_l[0] = math.copysign(axis_l[0], R_l[0][1])
                    axis_l[2] = math.copysign(axis_l[2], R_l[1][2])
                else:
                    axis_l[0] = math.copysign(axis_l[0], R_l[0][2])
                    axis_l[1] = math.copysign(axis_l[1], R_l[1][2])
                w_bar = mx.array(axis_l, dtype=mx.float32) * theta

            # SO(3) 左雅可比的逆: v̄ = V̄⁻¹·t
            bx, by, bz = float(w_bar[0]), float(w_bar[1]), float(w_bar[2])
            wxm = mx.array(
                [
                    [0.0, -bz, by],
                    [bz, 0.0, -bx],
                    [-by, bx, 0.0],
                ],
                dtype=mx.float32,
            )
            wx2 = mx.matmul(wxm, wxm, stream=mx.cpu)
            theta2 = theta * theta
            coeff = 1.0 / theta2 - (1.0 + cos_theta) / (2.0 * theta * sin_theta)
            V_inv = mx.eye(3) - 0.5 * wxm + coeff * wx2
            v_bar = mx.matmul(V_inv, t, stream=mx.cpu)

        w_l = (w_bar / 2.0).tolist()
        v_l = (v_bar / 2.0).tolist()
        return Motor.velocity_bivector(
            (w_l[0], w_l[1], w_l[2]),
            (v_l[0], v_l[1], v_l[2]),
        )

    def to_matrix(self) -> list[list[float]]:
        """等效 4x4 齐次变换矩阵 [R|t] (传统渲染管线用)。

        motor M 对共形点的作用: p' = M·p·M̃。
        """
        origin_t = self.apply(E0)
        tx = float(origin_t.values[1])
        ty = float(origin_t.values[2])
        tz = float(origin_t.values[3])

        px_t = self.apply(Point(1, 0, 0))
        py_t = self.apply(Point(0, 1, 0))
        pz_t = self.apply(Point(0, 0, 1))

        r00 = float(px_t.values[1]) - tx
        r10 = float(px_t.values[2]) - ty
        r20 = float(px_t.values[3]) - tz

        r01 = float(py_t.values[1]) - tx
        r11 = float(py_t.values[2]) - ty
        r21 = float(py_t.values[3]) - tz

        r02 = float(pz_t.values[1]) - tx
        r12 = float(pz_t.values[2]) - ty
        r22 = float(pz_t.values[3]) - tz

        return [
            [r00, r01, r02, tx],
            [r10, r11, r12, ty],
            [r20, r21, r22, tz],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def interpolate(self, other: Multivector, t: float) -> Motor:
        """self → other 的插值: M(t) = self · exp(t · log(self⁻¹·other))。"""
        delta = Motor._wrap(self.reverse().gp(other))
        return Motor._wrap(self.gp(Motor.exp(delta.log(), t)))

    # ── 速度二重向量 (twist) ──────────────────────────────────────

    @staticmethod
    def velocity_bivector(
        angular: tuple[float, float, float], linear: tuple[float, float, float]
    ) -> Multivector:
        """角速度 + 线速度 → 速度二重向量 (twist)。

        V = ω + v ∧ e∞, ω = ωx·e23 + ωy·e31 + ωz·e12 为角速度二重
        向量, v 为线速度。运动方程: dM/dt = −½·V·M。
        """
        wx, wy, wz = angular
        vx_val, vy_val, vz_val = linear

        vals = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        # 角速度部分 (二重向量):
        vals[6] = wz  # e12
        vals[7] = -wy  # e13 (from ωy*e31 = -ωy*e13)
        vals[10] = wx  # e23

        rot = Multivector(vals)
        tv = Multivector.vector(vx_val, vy_val, vz_val)
        return rot + tv.op(EINF)

    @staticmethod
    def extract_velocity(
        M_current: Multivector, M_previous: Multivector, dt: float
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """由相邻两个 motor 提取角/线速度。

        V ≈ 2 · log(M_prev⁻¹ · M_curr) / dt
        (delta = exp(−½V·dt) → log(delta) = ½V·dt → V = 2·log(delta)/dt)

        坐标系约定: 相对 motor M_prev⁻¹·M_curr 表达在上一时刻的
        体坐标系, 返回的 twist 是上一时刻的体坐标速度, 不是世界系
        twist —— 喂给世界系消费者前先用 M_previous 变换。
        """
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
        delta = Motor._wrap(M_previous.reverse().gp(M_current))
        V = delta.log() * (2.0 / dt)

        vals = V.values
        wx = float(vals[10])  # e23
        wy = -float(vals[7])  # e13 (取负因 e31 = -e13)
        wz = float(vals[6])  # e12
        # v∧e∞ 分量槽位: (i, 4) = e_i∧e∞
        vx = float(vals[9])  # e1∧e∞
        vy = float(vals[12])  # e2∧e∞
        vz = float(vals[14])  # e3∧e∞

        return ((wx, wy, wz), (vx, vy, vz))

    # ── 内部助手 ──────────────────────────────────────────────────

    @staticmethod
    def _matrix_to_quaternion(matrix) -> tuple[float, float, float, float]:
        """3x3 旋转矩阵 → (w, x, y, z) 四元数。"""
        m = mx.array(matrix, dtype=mx.float32)
        if m.ndim == 1 and m.size == 9:
            m = m.reshape(3, 3)
        trace = float(mx.diagonal(m).sum())
        if trace > 0.0:
            s = math.sqrt(trace + 1.0) * 2.0
            return (
                0.25 * s,
                float(m[2, 1] - m[1, 2]) / s,
                float(m[0, 2] - m[2, 0]) / s,
                float(m[1, 0] - m[0, 1]) / s,
            )
        if float(m[0, 0]) > float(m[1, 1]) and float(m[0, 0]) > float(m[2, 2]):
            s = math.sqrt(1.0 + float(m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
            return (
                float(m[2, 1] - m[1, 2]) / s,
                0.25 * s,
                float(m[0, 1] + m[1, 0]) / s,
                float(m[0, 2] + m[2, 0]) / s,
            )
        if float(m[1, 1]) > float(m[2, 2]):
            s = math.sqrt(1.0 + float(m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
            return (
                float(m[0, 2] - m[2, 0]) / s,
                float(m[0, 1] + m[1, 0]) / s,
                0.25 * s,
                float(m[1, 2] + m[2, 1]) / s,
            )
        s = math.sqrt(1.0 + float(m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
        return (
            float(m[1, 0] - m[0, 1]) / s,
            float(m[0, 2] + m[2, 0]) / s,
            float(m[1, 2] + m[2, 1]) / s,
            0.25 * s,
        )
