"""传感器与驱动器 —— 移植 Drake 的 ForceTorqueSensor / JointActuator 语义。

- ForceTorqueSensor: 测量 link 上某帧的 6 轴力/力矩 (世界力/力矩 →
  传感器局部坐标)。读数 = RNEA 反推的该 link 子树支撑力 (结构力)
  传播到传感器帧: 力平移不变, 力矩 n + r×f。
- JointActuator: 关节驱动器, 力矩饱和 (effort 限幅) —— 反馈到
  integrate 的 tau。
- JointStateSensor: 关节位置/速度/力矩读数 (thin wrapper, 契约统一)。

传感器是"生产者": 只读 plant 状态, 不写状态 (低耦合, 依赖单向)。
"""

from __future__ import annotations

from dataclasses import dataclass

from cga.dynamics import DynamicsPlant
from cga.motors import Motor


def _mm3(A, B):
    return [
        [A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j] for j in range(3)]
        for i in range(3)
    ]


def _mv3(A, v):
    return (
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    )


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


@dataclass
class ForceTorqueSensor:
    """6 轴力/力矩传感器 (Drake ForceTorqueSensor 语义)。

    origin: 传感器帧相对 link 的 Motor (默认 link 原点)。
    read(q, qd, qdd): 传感器帧局部坐标的 (力, 力矩) —— 世界分量经
    传感器旋转矩阵转回局部。qdd=None 时按准静态 (q̈=0, 只含 C·q̇+g)。
    """

    plant: DynamicsPlant
    link: str
    name: str = "fts"
    origin: Motor | None = None

    def read(self, q, qd, qdd=None) -> tuple[tuple, tuple]:
        """传感器读数: (F_local, τ_local) 在传感器帧 (SI)。"""
        n = self.plant.nq
        if qdd is None:
            qdd = [0.0] * n
        wrenches = self.plant.reaction_wrenches(q, qd, qdd)
        if self.link not in wrenches:
            raise KeyError(f"F/T 传感器 link {self.link!r} 无动力学数据")
        f_w, n_w = wrenches[self.link]
        # 传感器帧位姿 (世界): R_s·(R,t) 链
        Rl, tl = self.plant.rigid_fk(q)[self.link]
        if self.origin is None:
            Rs, ts = Rl, tl
        else:
            mo = self.origin.to_matrix()
            Ro = [[float(mo[i][j]) for j in range(3)] for i in range(3)]
            to = (float(mo[0][3]), float(mo[1][3]), float(mo[2][3]))
            Rs = _mm3(Rl, Ro)
            ts = _add(tl, _mv3(Rl, to))
        # 力矩传播到传感器原点 (r = 传感器原点 − link 原点, 世界)
        r = _add(ts, (-tl[0], -tl[1], -tl[2]))
        n_s = _add(n_w, _cross(r, f_w))
        # 转回传感器局部坐标
        Rst = [[Rs[j][i] for j in range(3)] for i in range(3)]
        return _mv3(Rst, f_w), _mv3(Rst, n_s)


@dataclass
class JointActuator:
    """关节驱动器: 力矩饱和 (Drake JointActuator 的 effort_limit)。

    saturate(tau): 把整条广义力矩向量里本关节的分量限幅到 ±limit,
    并记录实际输出 (read_effort)。limit=None 不限幅。
    """

    plant: DynamicsPlant
    joint: str
    effort_limit: float | None = None
    name: str = "actuator"

    def __post_init__(self):
        self.plant.joint_indices(self.joint)

    def saturate(self, tau: list[float]) -> list[float]:
        """tau 中本关节分量的限幅 (返回新列表)。"""
        i, _ = self.plant.joint_indices(self.joint)
        out = list(tau)
        if self.effort_limit is not None:
            out[i] = max(-self.effort_limit, min(self.effort_limit, tau[i]))
        self._effort = out[i]
        return out

    def read_effort(self) -> float:
        return getattr(self, "_effort", 0.0)


@dataclass
class JointStateSensor:
    """关节状态读数: (q, q̇, τ) 按关节列 (Drake Joint get_positions 等)。"""

    plant: DynamicsPlant
    joint: str
    name: str = "state"

    def __post_init__(self):
        self.plant.joint_indices(self.joint)

    def read(self, q, qd, tau=None) -> tuple[float, float, float]:
        i, iq = self.plant.joint_indices(self.joint)
        return q[iq], qd[i], (tau[i] if tau is not None else 0.0)
