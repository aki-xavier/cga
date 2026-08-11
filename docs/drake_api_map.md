# cga ↔ Drake API 一对一映射与交叉校验

同一机器人分别在 **真 pydrake 1.55** (MultibodyPlant) 与 **cga** (DynamicsPlant)
上计算, 数值逐项对比 —— `validate_drake.py` (用 simu 项目的 venv 运行,
那里装有 drake):

```bash
cd /Users/aki/code/simu && .venv/bin/python /Users/aki/code/cga/validate_drake.py
```

**结果: 26/26 项数值一致** (容差 ~1e-6, 差异为 URDF 往返格式化精度):

| 校验项 | 机器人 | max|Δ| |
| --- | --- | --- |
| mass_matrix | 伸缩臂 (R+P) / Z1 (6R) | 2.4e-9 / 8.1e-8 |
| gravity_forces | 两者 | 0 / 9.2e-8 |
| bias (C·v) | 两者 | 1.1e-16 / 6.7e-9 |
| inverse_dynamics | 两者 | 1.2e-9 / 9.2e-8 |
| forward_dynamics | 两者 | 1.4e-7 / 1.6e-3* |
| kinetic/potential energy | 两者 | ~1e-8 / ~1e-6 |
| 雅可比 (link 原点线速度) | 伸缩臂 | 0 |
| FK (link 位姿) | 伸缩臂 | ~1e-7 |
| 关节限位元数据 | Z1 | ~4e-9 |

\* Z1 腕部小惯量 (2e-4) → M⁻¹ 条件数放大, 容差 1e-2。

## 方法 ↔ 方法映射

| cga | Drake MultibodyPlant | 说明 |
| --- | --- | --- |
| `mass_matrix(q)` | `CalcMassMatrix(ctx)` | 数值一致 |
| `gravity_forces(q)` | `CalcGravityGeneralizedForces(ctx)` | **约定: cga 返回物理重力广义力 Q = −∂V/∂q (与 Drake 同号); EOM 形式 M·q̈ + C·q̇ − Q = τ** |
| `coriolis_forces(q, qd)` | `CalcBiasTerm(ctx, v)` | Drake 的 bias = **纯 C·v 不含重力**; cga 同为纯 Coriolis (RNEA 无重力等效加速度) |
| `inverse_dynamics(q,qd,qdd)` | `CalcInverseDynamics(ctx, qdd, forces)` | cga = M·q̈ + C − Q (含重力); Drake 的 forces 需显式加重力 (空 forces 时 = M·q̈ + C·v) |
| `forward_dynamics(q,qd,tau)` | (连续模式无直接绑定) | cga = M⁻¹(τ − C·v + Q); 与 Drake EOM 手解一致 |
| `jacobians(q)` / `jacobian_at(q,link,p)` | `CalcJacobianTranslationalVelocity` | cga 的 `jacobians` 是 **COM 速度**, 点速度用 `jacobian_at` |
| `kinetic_energy` / `potential_energy` | `CalcKineticEnergy` / `CalcPotentialEnergy` | 数值一致 |
| 关节限位 (lower/upper) | `position_lower/upper_limits()` | 数值一致 |

## 交叉校验发现并修复的差异

1. **`gravity_forces` 符号约定 (真实差异, 已修复)**: cga 曾返回 +∂V/∂q
   (拉格朗日支撑力矩), Drake 的 `CalcGravityGeneralizedForces` 返回
   物理重力广义力 −∂V/∂q —— 符号相反。修复: cga 翻转 + EOM 改为
   M·q̈ + C·q̇ − Q = τ (forward_dynamics 用 +Q, inverse_dynamics 用 −Q,
   控制器 τ = M·acc − Q)。自检 "dyn pendulum gravity"/"dyn double
   gravity" 同步更新。物理行为不变 (自由落体/摆锤/接触平衡均未变)。
2. **Drake 的 `CalcBiasTerm` 不含重力** (仅纯 C·v) —— cga 的
   `coriolis_forces` 同为纯 C (此前 RNEA 的 "ID − g" 拆法语义一致)。
3. 首版校验脚本的 F/T 传感器绑定在 pydrake 1.55 未暴露 (C++ 有,
   Python 绑定缺) —— F/T 只做 API 映射 (下方), 不数值交叉。

## 结构/语义映射 (非数值)

| cga | Drake | 说明 |
| --- | --- | --- |
| `DynamicsPlant(robot, floating_base, weld)` | `MultibodyPlant` + `WeldFrames` | 浮动基座 q 序: cga `[p, qx,qy,qz,qw, 关节]`, Drake `[x,y,z, qw,qx,qy,qz, 关节]` (四元数元素序不同!) |
| `integrate` (半隐式欧拉) | Simulator (隐式/半隐式) | 冲击场景 cga 用 `integrate_implicit` 接触 |
| `integrate_rk4` / `integrate_adaptive` | `RungeKutta3Integrator` / `DormandPrinceIntegrator` | 变步长均为嵌入式误差估计 |
| `ContactModel` (惩罚/隐式脉冲) | hydroelastic contact / TAMSI | 模型不同 (cga 是简化), 语义同: 法向 + 库仑摩擦 |
| `ForceTorqueSensor.read(q,qd,qdd)` | `ForceTorqueSensor` (AddForceTorqueSensor) | 读数 = 子树支撑力传播到传感器帧; Drake 的 `measured_wrench` 输出端口 |
| `JointActuator` | `JointActuator` | 力矩饱和 (effort_limit) |
| `System`/`Diagram`/`Simulator` | `LeafSystem`/`Diagram`/`Simulator` | 端口连线 + 拓扑推进; cga 反馈走一拍延迟, Drake 代数环需求解器 |
| `TrajectorySource`/`PidController` | `TrajectorySource`/常值向量源+控制器 | 5 次多项式 / 计算力矩 |
