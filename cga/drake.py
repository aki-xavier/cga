"""Drake 物理后端 —— 直接接入 pydrake MultibodyPlant (替代自定义动力学移植)。

cga 保留 CRDF/渲染引擎, 物理全用 Drake (生产级):
- 离散 plant (time_step>0): 内置接触求解器 (TAMSI/约束), AdvanceOneStep 一步;
- 连续查询 (mass_matrix/gravity/coriolis/inverse/forward dynamics) 透传 Drake;
- 渲染用 poses() 取各 link 世界位姿 (R, t), 走 cga engine。

注: Drake API 强制 numpy (本项目 sanctioned interop 例外); 浮动基座
用 Drake 原生四元数关节; 关节驱动经 JointActuator + 驱动端口。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from pydrake.common.eigen_geometry import Quaternion  # type: ignore
from pydrake.geometry import Box
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    CoulombFriction,
)
from pydrake.multibody.tree import (
    MultibodyForces,
)
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

from cga.robot import Robot, load_robot
from cga.urdf_io import crdf_to_urdf


class DrakePlant:
    """pydrake MultibodyPlant 适配器 (面向渲染/控制/动力学查询)。"""

    def __init__(
        self,
        robot: Robot | str,
        dt: float = 1e-3,
        floating_base: bool = False,
        gravity: tuple = (0.0, 0.0, -9.81),
        stiffness: float | None = None,
        dissipation: float | None = None,
        friction: float | None = None,
        ground: bool = True,  # 世界加固定地面 (10×10×0.1, 顶面 z=0)
    ):
        self.robot = (
            load_robot(robot) if isinstance(robot, (str, Path)) else robot
        )
        self.dt = dt
        builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder, time_step=dt
        )
        urdf = crdf_to_urdf(self.robot)
        Parser(self.plant).AddModelsFromString(urdf, "robot.urdf")
        # cga 语义: robot.base 世界固定; floating_base=True 时基座浮动
        if not floating_base:
            try:
                self.plant.WeldFrames(
                    self.plant.world_frame(),
                    self.plant.GetFrameByName(self.robot.base),
                )
            except RuntimeError:
                pass  # URDF 已带 world→基座 固定关节
        # 关节驱动器 (URDF 无 <transmission> 时 Drake 不自动建)
        self._actuators = {}
        for j in self.robot.joints:
            if j.type in ("revolute", "continuous", "prismatic"):
                act = self.plant.AddJointActuator(
                    f"act_{j.name}", self.plant.GetJointByName(j.name)
                )
                self._actuators[j.name] = act
        if stiffness is not None:
            self.plant.set_penetration_allowance(stiffness)
        if dissipation is not None:
            self.plant.set_stiction_tolerance(dissipation)
        if friction is not None:
            self.plant.set_default_contact_material(
                friction_coefficients=(friction, friction)
            )
        if ground:
            self._add_ground(friction or 0.7)
        self.plant.Finalize()
        self.diagram = builder.Build()
        self.sim = Simulator(self.diagram)
        self.ctx = self.diagram.GetMutableSubsystemContext(
            self.plant, self.sim.get_mutable_context()
        )
        # 关节名 → Drake Joint (可动)
        self._joints = {
            j.name: self.plant.GetJointByName(j.name)
            for j in self.robot.joints
            if j.type in ("revolute", "continuous", "prismatic")
        }
        # 驱动端口 (tau 按 CRDF 关节序)
        self.actuation_port = self.plant.get_actuation_input_port()

    def _add_ground(self, mu: float) -> None:
        """固定地面: 10×10×0.1 盒, 顶面 z=0 (cga 的 planes 概念 → 真实几何)。"""
        inst = self.plant.AddModelInstance("ground")
        body = self.plant.AddRigidBody("ground_body", inst)
        self.plant.WeldFrames(
            self.plant.world_frame(), body.body_frame(),
            RigidTransform(p=[0.0, 0.0, -0.05]),
        )
        self.plant.RegisterCollisionGeometry(
            body,
            RigidTransform(),
            Box(10.0, 10.0, 0.1),
            "ground_collision",
            CoulombFriction(mu, mu),
        )
        self.plant.RegisterVisualGeometry(
            body,
            RigidTransform(),
            Box(10.0, 10.0, 0.1),
            "ground_visual",
            np.array([0.6, 0.63, 0.65, 1.0]),
        )

    # ── 状态读写 ───────────────────────────────────────────────

    def joint_state(self) -> tuple[list, list]:
        """(q, q̇) 按 CRDF 可动关节序。"""
        q = [self._joints[n].get_angle(self.ctx) for n in self._joints]
        qd = [self._joints[n].get_angular_rate(self.ctx) for n in self._joints]
        return q, qd

    def set_joint_state(self, q: list, qd: list | None = None) -> None:
        for n, val in zip(self._joints, q, strict=True):
            self._joints[n].set_angle(self.ctx, float(val))
        if qd is not None:
            for n, val in zip(self._joints, qd, strict=True):
                self._joints[n].set_angular_rate(self.ctx, float(val))

    def set_base_pose(self, p: tuple, quat: tuple) -> None:
        """浮动基座位姿 (世界 p + 四元数 (qw,qx,qy,qz), Drake 序)。"""
        body = self.plant.GetBodyByName(self.robot.base)
        X = RigidTransform(RotationMatrix(Quaternion(*quat)), p)  # type: ignore
        self.plant.SetFreeBodyPose(self.ctx, body, X)

    def set_base_twist(self, v: tuple, w: tuple) -> None:
        from pydrake.multibody.math import SpatialVelocity  # type: ignore

        body = self.plant.GetBodyByName(self.robot.base)
        self.plant.SetFreeBodySpatialVelocity(
            self.ctx, body, SpatialVelocity(w, v)  # type: ignore
        )

    # ── 仿真推进 ───────────────────────────────────────────────

    def step(self, tau: list | None = None, n: int = 1) -> None:
        """离散推进 n 步 (每步 dt), tau 按 CRDF 关节序 (None = 零)。"""
        if tau is None:
            tau = [0.0] * len(self._actuators)
        tau_vec = np.zeros(len(self._actuators))
        for i, name in enumerate(self._actuators):
            tau_vec[i] = tau[i]
        self.actuation_port.FixValue(self.ctx, tau_vec)
        for _ in range(n):
            self.sim.AdvanceTo(self.sim.get_context().get_time() + self.dt)

    def time(self) -> float:
        return float(self.sim.get_context().get_time())

    # ── 渲染 ───────────────────────────────────────────────────

    def poses(self) -> dict[str, tuple]:
        """各 link 世界位姿 {name: (R, t)} (R 3×3 list, t 3-tuple)。"""
        out = {}
        for lnk in self.robot.links:
            X = self.plant.GetBodyByName(lnk.name).EvalPoseInWorld(self.ctx)
            out[lnk.name] = (
                [[float(v) for v in row] for row in X.rotation().matrix()],
                tuple(float(v) for v in X.translation()),
            )
        return out

    # ── 动力学查询 (透传 Drake) ────────────────────────────────

    def _context_with(self, q: list, qd: list | None = None):
        ctx = self.plant.CreateDefaultContext()
        for n, val in zip(self._joints, q, strict=True):
            self._joints[n].set_angle(ctx, float(val))
        if qd is not None:
            for n, val in zip(self._joints, qd, strict=True):
                self._joints[n].set_angular_rate(ctx, float(val))
        return ctx

    def mass_matrix(self, q: list) -> list[list[float]]:
        return self.plant.CalcMassMatrix(self._context_with(q)).tolist()

    def gravity_forces(self, q: list) -> list[float]:
        return self.plant.CalcGravityGeneralizedForces(
            self._context_with(q)
        ).tolist()

    def coriolis_forces(self, q: list, qd: list) -> list[float]:
        return self.plant.CalcBiasTerm(
            self._context_with(q, qd)
        ).tolist()

    def inverse_dynamics(self, q, qd, qdd) -> list[float]:
        """τ = M·q̈ + C·v − Q (含重力 —— Drake 的 ID(空 forces) 不含,
        需显式减 Q 才对齐 EOM: M·q̈ + C·v − Q = τ)。"""
        ctx = self._context_with(q, qd)
        id_ = self.plant.CalcInverseDynamics(
            ctx, np.array(qdd), MultibodyForces(self.plant)
        )
        g = self.plant.CalcGravityGeneralizedForces(ctx)
        return (id_ - g).tolist()

    def forward_dynamics(self, q, qd, tau) -> list[float]:
        """q̈ = M⁻¹(τ − C·v + Q) (EOM: M·q̈ + C·v − Q = τ)。"""
        ctx = self._context_with(q, qd)
        M = self.plant.CalcMassMatrix(ctx)
        bias = self.plant.CalcBiasTerm(ctx)
        g = self.plant.CalcGravityGeneralizedForces(ctx)
        tau_vec = np.zeros(self.plant.num_actuated_dofs())
        for i, name in enumerate(self._actuators):
            tau_vec[i] = tau[i]
        return np.linalg.solve(M, tau_vec - bias + g).tolist()

    def kinetic_energy(self, q, qd) -> float:
        return float(self.plant.CalcKineticEnergy(self._context_with(q, qd)))

    def potential_energy(self, q) -> float:
        return float(self.plant.CalcPotentialEnergy(self._context_with(q)))
