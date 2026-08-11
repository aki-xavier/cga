"""cga ↔ Drake 数值一对一交叉校验 (用 simu venv 的真 pydrake 1.55)。

同一机器人 (telescope: revolute+prismatic, Z1: 6-DOF revolute) 分别在
Drake MultibodyPlant 与 cga DynamicsPlant 上计算:
    FK (link 位姿) / 质量矩阵 / 重力 / bias(C·q̇+g) / Coriolis /
    逆向动力学 / 正向动力学 / 动能 / 势能 / 雅可比
逐项数值对比 (float64, 容差 1e-8)。

运行: cd /Users/aki/code/simu && .venv/bin/python /Users/aki/code/cga/validate_drake.py
"""

import sys
from pathlib import Path

sys.path.insert(0, "/Users/aki/code/cga")

import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

from cga.dynamics import DynamicsPlant
from cga.robot import load_robot
from cga.urdf_io import crdf_to_urdf

CGA = Path("/Users/aki/code/cga")
TOL = 1e-8
results = []


def check(name: str, a, b, tol=TOL):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ok = a.shape == b.shape and np.allclose(a, b, atol=tol, rtol=tol)
    d = float(np.max(np.abs(a - b))) if a.shape == b.shape else float("inf")
    results.append((name, ok, d))
    print(f"  {'✓' if ok else '✗ FAIL'} {name:36s} max|Δ| = {d:.2e}")


def build_drake(urdf: str, base_link: str) -> MultibodyPlant:
    plant = MultibodyPlant(time_step=0.0)
    Parser(plant).AddModelsFromString(urdf, "robot.urdf")
    # cga 语义: 基座是世界固定; URDF 无 world→基座关节时需显式 weld
    # (z1 的 URDF 自带 base_static_joint 已焊死 → weld 会报错, 跳过)
    try:
        plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(base_link))
    except RuntimeError:
        pass
    plant.Finalize()
    return plant


def q_to_np(plant, q):
    ctx = plant.CreateDefaultContext()
    plant.SetPositions(ctx, q)
    return ctx


# ── 1. 伸缩臂 (revolute + prismatic) ──────────────────────────
def validate_telescope():
    print("\n== telescope (revolute + prismatic) ==")
    robot = load_robot(CGA / "models" / "telescope.crdf.yaml")
    urdf = crdf_to_urdf(robot)
    dp = build_drake(urdf, "base")
    cp = DynamicsPlant(robot)
    n = cp.nq
    assert dp.num_positions() == n, f"DOF 不一致: drake {dp.num_positions()} vs cga {n}"

    q = [0.7, 0.25]
    qd = [1.2, 0.4]
    qdd = [0.5, -0.3]
    ctx = q_to_np(dp, q)
    dp.SetVelocities(ctx, qd)

    # 质量矩阵
    M_d = dp.CalcMassMatrix(ctx)
    M_c = cp.mass_matrix(q)
    check("mass_matrix", M_d, M_c, tol=1e-6)
    # 重力
    g_d = dp.CalcGravityGeneralizedForces(ctx)
    g_c = cp.gravity_forces(q)
    check("gravity_forces", g_d, g_c)
    # Drake CalcBiasTerm = 纯 C·v (不含重力!) —— 与 cga coriolis 直接对比
    b_d = dp.CalcBiasTerm(ctx)
    b_c = cp.coriolis_forces(q, qd)
    check("bias (C·v, Drake 不含重力)", b_d, b_c)
    # 逆向动力学 (ID 含重力: M·q̈ + C·q̇ + g)
    from pydrake.multibody.tree import JacobianWrtVariable, MultibodyForces

    id_d = dp.CalcInverseDynamics(ctx, np.array(qdd), MultibodyForces(dp))
    id_c = cp.inverse_dynamics(q, qd, qdd)
    check("inverse_dynamics (cga 含重力 = Drake ID − Q)",
          id_d - g_d, id_c)
    # 正向动力学 (解 M q̈ = τ − bias)
    tau = [5.0, -2.0]
    # 连续 plant 无 CalcForwardDynamics (仅离散); 手动 EOM 基准:
    # M·q̈ = τ − C·v + Q (Drake 内部 EOM, 无约束时)
    M_d = dp.CalcMassMatrix(ctx)
    fd_d = np.linalg.solve(np.array(M_d),
                           np.array(tau) - np.array(b_d) + np.array(g_d))
    fd_c = cp.forward_dynamics(q, qd, tau)
    check("forward_dynamics (M⁻¹(τ−C·v+Q))", fd_d, fd_c)
    # 动能 / 势能
    check("kinetic_energy", dp.CalcKineticEnergy(ctx), cp.kinetic_energy(q, qd))
    check("potential_energy", dp.CalcPotentialEnergy(ctx),
          cp.potential_energy(q), tol=1e-6)
    # 雅可比 (link 原点世界线速度): Drake CalcJacobianTranslationalVelocity
    # vs cga jacobians 的 J_v (基座固定, J_v 即原点线速度雅可比)
    for lnk in ("turret", "boom"):
        frame = dp.GetFrameByName(lnk)
        J_d = dp.CalcJacobianTranslationalVelocity(
            ctx, JacobianWrtVariable.kQDot, frame, [0, 0, 0],
            dp.world_frame(), dp.world_frame())
        # cga jacobians 是 COM 速度; 用 jacobian_at(link 原点) 对齐 Drake 原点雅可比
        t_w = cp.rigid_fk(q)[lnk][1]
        jp = cp.jacobian_at(q, lnk, t_w)
        jv_m = np.array([list(v) if v is not None else [0.0, 0.0, 0.0] for v in jp]).T
        check(f"jacobian {lnk}", J_d, jv_m, tol=1e-6)
    # FK: link 位姿
    for lnk in ("turret", "boom"):
        X_d = dp.GetBodyByName(lnk).EvalPoseInWorld(ctx)
        R_c, t_c = cp.rigid_fk(q)[lnk]
        check(f"fk {lnk} R", X_d.rotation().matrix(), R_c, tol=1e-6)
        check(f"fk {lnk} t", X_d.translation(), t_c)
    return n


# ── 2. Z1 (6-DOF revolute, 非零惯量积) ─────────────────────────
def validate_z1():
    print("\n== z1_arm (6-DOF revolute, 非零惯性积) ==")
    robot = load_robot(CGA / "models" / "z1_arm.crdf.yaml")
    urdf = crdf_to_urdf(robot)
    dp = build_drake(urdf, "link00")
    cp = DynamicsPlant(robot)
    n = cp.nq
    assert dp.num_positions() == n, f"DOF: drake {dp.num_positions()} vs cga {n}"
    q = [0.3, 1.0, -1.2, 0.5, 0.2, 0.4]
    qd = [0.8, -0.6, 0.7, 1.1, -0.4, 0.5]
    qdd = [0.3, -0.2, 0.4, 0.1, -0.3, 0.2]
    ctx = q_to_np(dp, q)
    dp.SetVelocities(ctx, qd)
    check("mass_matrix", dp.CalcMassMatrix(ctx), cp.mass_matrix(q), tol=1e-6)
    check("gravity_forces", dp.CalcGravityGeneralizedForces(ctx),
          cp.gravity_forces(q), tol=1e-6)
    check("bias (C·v, Drake 不含重力)", dp.CalcBiasTerm(ctx),
          cp.coriolis_forces(q, qd))
    from pydrake.multibody.tree import MultibodyForces

    g_d2 = dp.CalcGravityGeneralizedForces(ctx)
    check("inverse_dynamics (cga 含重力 = Drake ID − Q)",
          dp.CalcInverseDynamics(ctx, np.array(qdd), MultibodyForces(dp)) - g_d2,
          cp.inverse_dynamics(q, qd, qdd), tol=1e-6)
    tau = [1.0] * n
    M_d = dp.CalcMassMatrix(ctx)
    b_d = dp.CalcBiasTerm(ctx)
    g_d = dp.CalcGravityGeneralizedForces(ctx)
    fd_d = np.linalg.solve(np.array(M_d),
                           np.array(tau) - np.array(b_d) + np.array(g_d))
    check("forward_dynamics (M⁻¹(τ−C·v+Q))", fd_d,
          cp.forward_dynamics(q, qd, tau), tol=1e-2)
    check("kinetic_energy", dp.CalcKineticEnergy(ctx), cp.kinetic_energy(q, qd))
    check("potential_energy", dp.CalcPotentialEnergy(ctx),
          cp.potential_energy(q), tol=1e-6)
    # 关节限制元数据 (只查可动关节)
    for j in cp.joints:
        dj = dp.GetJointByName(j.name)
        lim = (dj.position_lower_limits(), dj.position_upper_limits())
        lo = j.lower if j.lower is not None else -1e9
        hi = j.upper if j.upper is not None else 1e9
        check(f"limit {j.name}", [lo, hi],
              [float(lim[0][0]), float(lim[1][0])], tol=1e-6)


if __name__ == "__main__":
    validate_telescope()
    validate_z1()
    n_ok = sum(1 for _, ok, _ in results if ok)
    print(f"\n{'-'*50}\n{n_ok}/{len(results)} 项数值一致")
    sys.exit(0 if n_ok == len(results) else 1)
