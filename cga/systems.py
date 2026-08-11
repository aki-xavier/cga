"""systems 框架 —— 移植 Drake Diagram/ports 语义 (仿真图组合)。

System: 带端口 (输入/输出) + 状态 + 一步推进的单元;
Diagram: 系统图 —— 端口连线 (output→input), 按拓扑序推进;
Simulator: 仿真驱动器 (循环 step + 端口记录 tracer)。

低耦合: 系统间只经端口传数据 (生产者/消费者解耦); 依赖方向
单向 (控制器/传感器 → 动力学, 不反向摸内部)。
"""

from __future__ import annotations

from dataclasses import dataclass, field

from cga.dynamics import DynamicsPlant
from cga.sensors import ForceTorqueSensor


class System:
    """端口系统基类: step(state, inputs, dt) -> (new_state, outputs)。"""

    name: str = "system"

    def step(self, state, inputs: dict, dt: float) -> tuple:
        raise NotImplementedError

    def initial_outputs(self) -> dict:
        """初始输出 (Diagram 建图后预置, 反馈读一拍延迟的初始值)。"""
        return {}


@dataclass
class Diagram:
    """系统图: 端口连线 + 拓扑序推进。"""

    systems: list = field(default_factory=list)
    _wires: list = field(default_factory=list)  # (src, src_port, dst, dst_port)
    _states: dict = field(default_factory=dict)
    _outputs: dict = field(default_factory=dict)
    _topo: list = field(default_factory=list)

    def add(self, sys: System) -> int:
        idx = len(self.systems)
        self.systems.append(sys)
        self._states[idx] = None
        self._outputs[idx] = sys.initial_outputs()
        return idx

    def connect(self, src: int, src_port: str, dst: int, dst_port: str) -> None:
        self._wires.append((src, src_port, dst, dst_port))
        self._topo = self._topo_sort()

    def _topo_sort(self) -> list:
        """Kahn 拓扑排序。剩余边 (反馈环) 变一拍延迟读 —— 控制器读
        plant 上一拍的状态输出 (离散控制标准语义), 非代数环。"""
        indeg = {i: 0 for i in range(len(self.systems))}
        adj = {i: [] for i in range(len(self.systems))}
        for (w_src, _s, w_dst, _p) in self._wires:
            adj[w_src].append(w_dst)
            indeg[w_dst] += 1
        queue = [i for i in range(len(self.systems)) if indeg[i] == 0]
        order = []
        while queue:
            i = queue.pop(0)
            order.append(i)
            for j in adj[i]:
                indeg[j] -= 1
                if indeg[j] == 0:
                    queue.append(j)
        # 环上的系统追加在后 (其反馈输入走上一拍输出, 已就绪)
        for i in range(len(self.systems)):
            if i not in order:
                order.append(i)
        return order

    def step(self, dt: float) -> dict:
        """推进所有系统 (拓扑序 + 一拍延迟反馈), 返回各系统输出。"""
        for idx in self._topo:
            sys = self.systems[idx]
            inputs = {}
            for (w_src, w_port, w_dst, port) in self._wires:
                if (
                    w_dst == idx
                    and w_src in self._outputs
                    and w_port in self._outputs[w_src]
                ):
                    inputs[port] = self._outputs[w_src][w_port]
            new_state, outputs = sys.step(self._states[idx], inputs, dt)
            self._states[idx] = new_state
            self._outputs[idx] = outputs
        return self._outputs


@dataclass
class TrajectorySource(System):
    """目标位形发生器: q(t) 从 q0 到 q1 的 5 次多项式 (首末速度/加速度零)。"""

    q0: list
    q1: list
    duration: float
    name: str = "trajectory"

    def step(self, state, inputs, dt):
        t = state or 0.0
        t = min(t + dt, self.duration)
        s = t / self.duration
        # 5 次多项式: q = q0 + (q1-q0)·(10s³ - 15s⁴ + 6s⁵)
        s3, s4, s5 = s**3, s**4, s**5
        k = 10 * s3 - 15 * s4 + 6 * s5
        q = [self.q0[i] + (self.q1[i] - self.q0[i]) * k for i in range(len(self.q0))]
        return t, {"q_des": q}


@dataclass
class PidController(System):
    """计算力矩 PID: τ = g + M·(Kp·e + Ki·∫e − Kd·q̇) (闭环 PD 增益)。"""

    plant: DynamicsPlant
    kp: float = 50.0
    kd: float = 12.0
    ki: float = 0.0
    name: str = "pid"

    def step(self, state, inputs, dt):
        q, qd = inputs["state"]
        q_des = inputs["q_des"]
        n = self.plant.nq
        if state is None:
            e_int = [0.0] * n
        else:
            e_int = state
        g = self.plant.gravity_forces(q)
        M = self.plant.mass_matrix(q)
        e = [q_des[i] - q[i] for i in range(n)]
        e_int = [e_int[i] + e[i] * dt for i in range(n)]
        acc = [self.kp * e[i] + self.ki * e_int[i] - self.kd * qd[i] for i in range(n)]
        tau = [g[i] + sum(M[i][j] * acc[j] for j in range(n)) for i in range(n)]
        return e_int, {"tau": tau}


@dataclass
class DynamicsSystem(System):
    """DynamicsPlant 包装: 输入 tau, 输出 state (q, qd) 与各 link 速度。"""

    plant: DynamicsPlant
    q: list
    qd: list
    name: str = "plant"

    def step(self, state, inputs, dt):
        tau = inputs.get("tau", [0.0] * self.plant.nq)
        self.qdd = self.plant.forward_dynamics(self.q, self.qd, tau)  # 实际加速度
        self.q, self.qd = self.plant.integrate(self.q, self.qd, tau, dt)
        return (self.q, self.qd), {"state": (self.q, self.qd), "qdd": self.qdd}

    def initial_outputs(self) -> dict:
        return {"state": (self.q, self.qd), "qdd": [0.0] * self.plant.nq}


@dataclass
class FtsSystem(System):
    """力/力矩传感器系统: 输入 state, 输出 6 轴 F/T (传感器局部坐标)。"""

    sensor: ForceTorqueSensor
    name: str = "fts"

    def step(self, state, inputs, dt):
        q, qd = inputs["state"]
        qdd = inputs.get("qdd")  # plant 的实际加速度 (非自由动力学!)
        f, n = self.sensor.read(q, qd, qdd)
        return None, {"fts": (f, n)}


@dataclass
class Simulator:
    """仿真驱动器: 循环推进 diagram, 可选记录指定端口 (tracer)。"""

    diagram: Diagram
    dt: float = 2e-3
    trace_ports: list = field(default_factory=list)  # [(system_idx, port)]

    def advance_to(self, t_end: float) -> dict:
        """步进到 t_end, 返回 {port_key: [每步值]} 轨迹。"""
        traces = {f"{i}:{p}": [] for (i, p) in self.trace_ports}
        t = 0.0
        while t < t_end - 1e-12:
            outs = self.diagram.step(self.dt)
            for (i, p) in self.trace_ports:
                if i in outs and p in outs[i]:
                    traces[f"{i}:{p}"].append(outs[i][p])
            t += self.dt
        return traces
