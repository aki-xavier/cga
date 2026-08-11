"""刚体动力学 —— 移植 Drake MultibodyPlant 的 API 子集, CRDF + CGA 驱动。

API 镜像 Drake:
    DynamicsPlant(robot)    载入 CRDF 机器人
    mass_matrix(q)          质量矩阵 M(q)
    gravity_forces(q)       广义重力 g(q)
    coriolis_forces(q, qd)  Coriolis/离心力 C(q,q̇)q̇
    inverse_dynamics(q,qd,qdd) → τ = M·q̈ + C·q̇ + g
    forward_dynamics(q,qd,τ) → q̈ = M⁻¹(τ − C·q̇ − g)
    integrate(q, qd, τ, dt) 半隐式欧拉一步 (关节限位 + 阻尼)
    link_twists(q, qd)      每个 link 的空间速度 = twist 二重向量 (CGA)

算法: 拉格朗日 + 雅可比 ——
    M 由动能 K = ½q̇ᵀMq̇ 精确构造 (角/线雅可比 + 世界惯量);
    g 由势能 V = Σ m·g·z_com 有限差分;
    C 由 Christoffel 符号 (M 的有限差分)。
CGA 角色: 每帧 FK 走 motor 链 (robot.fk), link 姿态是 Motor, 速度是
二重向量 (motors.velocity_bivector)。

范围 (v1): 固定基座串联臂; 无接触/驱动器模型; 重力沿机器人 Z 轴
(Z-up 约定, g = −9.81); 关节限位与阻尼来自 CRDF (dynamics.damping)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from cga.motors import Motor
from cga.robot import (
    CONTINUOUS,
    FIXED,
    PRISMATIC,
    REVOLUTE,
    Robot,
    RobotError,
)

GRAVITY = 9.81  # Z-up 世界: 势能 V = Σ m·g·z (标准约定, g 向下为正)
# 注意: 此约定下 g(q) = ∂V/∂q 是"平衡重力所需的广义力"; 曾用 −9.81
# 导致 g 差全局符号, 自由动力学方向错误 (单摆不落反升)。


@dataclass
class _DynLink:
    """动力连杆: 质量/质心/惯量 (link frame, 关于质心)。"""

    name: str
    mass: float
    com: tuple[float, float, float]
    inertia: tuple[float, float, float, float, float, float]  # ixx..iyz
    chain: tuple = field(default=())  # 从 base 到该 link 的关节 (拓扑序)


class DynamicsPlant:
    """CRDF 机器人 → 固定基座刚体动力学 (镜像 Drake MultibodyPlant 子集)。"""

    def __init__(
        self,
        robot: Robot,
        floating_base: bool = False,
        weld: tuple[str, ...] = (),
        weld_pose: tuple[float, ...] | None = None,
    ):
        """浮动基座: 虚拟 6-DOF (线速度+角速度) + 四元数位姿接在基座 link 前。

        q = [px, py, pz, qx, qy, qz, qw, 关节...] (四元数), nq = 6 + 可动关节数,
        qd = [v, ω, 关节...]。基座 link = 与 robot.base 固定关节相连的首个 link
        (Z1: link00)。weld: 焊死关节名 (Drake WeldFrames 语义) —— 不做自由度,
        FK 当固定关节传播位姿; weld_pose: 焊死关节的固定角度 (默认 0)。
        全焊接可作刚体坠落。
        """
        self.robot = robot
        self.floating = floating_base
        self.weld = set(weld)
        self.weld_pose = dict(zip(weld, weld_pose or (0.0,) * len(weld)))
        self.movable = [
            j
            for j in robot.joints
            if j.type in (REVOLUTE, CONTINUOUS, PRISMATIC) and j.name not in self.weld
        ]
        # prismatic 支持: FK 平移 + 雅可比平动列 + RNEA 平动分支
        if floating_base:
            base_link = None
            for j in robot.joints:
                if j.type == FIXED and j.parent == robot.base:
                    base_link = j.child
                    break
            if base_link is None:
                raise RobotError("浮动基座需要 world→link 的固定关节")
            self.base_link = base_link
            self.root = base_link
            self.nq = 6 + len(self.movable)  # 速度状态: [v, ω, 关节]
            self.nq_pos = self.nq + 1  # 配置: [p, 四元数(qx,qy,qz,qw), 关节]
        else:
            self.base_link = None
            self.root = robot.base
            self.nq = len(self.movable)
            self.nq_pos = self.nq
        self.joints = self.movable

        # 拓扑: 每个 link 的父关节
        joint_of = {j.child: j for j in robot.joints}
        parent_of = {j.child: j.parent for j in robot.joints}
        # 每个动力 link 的关节链 (root → link; 浮动时从基座 link 起,
        # 不含 world 固定关节)
        self.links: list[_DynLink] = []
        for lnk in robot.links:
            inert = lnk.inertial
            if inert is None or inert.mass <= 0:
                continue
            chain = []
            node = lnk.name
            while node in joint_of and node != self.root:
                chain.append(joint_of[node])
                node = parent_of[node]
            chain.reverse()
            self.links.append(
                _DynLink(
                    name=lnk.name,
                    mass=inert.mass,
                    com=inert.com,
                    inertia=(
                        inert.ixx,
                        inert.iyy,
                        inert.izz,
                        inert.ixy,
                        inert.ixz,
                        inert.iyz,
                    ),
                    chain=tuple(chain),
                )
            )
        # 关节索引: _jidx = 速度列 (浮动基座前 6 列 v/ω), _jidxq = 配置位
        # (浮动基座前 7 位 p+四元数 —— 两者差 1, 勿混用)
        base_off = 6 if floating_base else 0
        self._jidx = {j.name: base_off + i for i, j in enumerate(self.movable)}
        self._jidxq = {
            j.name: (base_off + 1 if floating_base else 0) + i
            for i, j in enumerate(self.movable)
        }
        self._damping = {
            j.name: j.damping or 0.0 for j in self.joints
        }
        # 关节 origin motor 的 (R, t) 矩阵形式 (一次性, 热路径预计算;
        # 含固定关节 —— link00 经 base_static_joint 连到 world)
        self._origin_rt = {}
        for j in robot.joints:
            mt = j.origin.to_matrix()
            self._origin_rt[j.name] = (
                [[float(mt[i][k]) for k in range(3)] for i in range(3)],
                (float(mt[0][3]), float(mt[1][3]), float(mt[2][3])),
            )

    # ── 运动学: FK + 雅可比 ──────────────────────────────────────

    def rigid_fk(self, q: list[float]) -> dict[str, tuple]:
        """纯 Python 刚体 FK: link → (R, t)。

        动力学热路径不用 MLX Motor FK (每次 ~18-30 个 MLX op, 每 op
        ~10ms 同步开销, 热循环不可用)。这里用欧氏 (R,t) 链, 与
        Motor 链的 to_matrix() 数值等价 (自检覆盖一致性):
        M_child = M_parent·M_origin·Rot(axis,q) → R = Rp·Ro·Rq,
        t = tp + Rp·to (origin 的 (Ro,to) 在 __init__ 预计算)。
        """
        if self.floating:
            px, py, pz = q[0], q[1], q[2]
            R0 = _quat_to_R(q[3], q[4], q[5], q[6])
            world: dict = {self.base_link: (R0, (px, py, pz))}
            # 跳过 world→基座 的固定关节 (已被虚拟关节代替)
            pending = [
                j
                for j in self.robot.joints
                if not (j.type == FIXED and j.parent == self.robot.base)
            ]
        else:
            world: dict = {self.robot.base: (_I3(), (0.0, 0.0, 0.0))}
            pending = list(self.robot.joints)
        while pending:
            progressed = False
            for j in pending[:]:
                if j.parent not in world:
                    continue
                Rp, tp = world[j.parent]
                Ro, to = self._origin_rt[j.name]
                R_po = _mm3(Rp, Ro)
                if j.type == FIXED or j.name in self.weld:
                    ang = (
                        self.weld_pose.get(j.name, 0.0) if j.name in self.weld else 0.0
                    )
                    Rq = (
                        _rot_axis(j.axis or (0.0, 0.0, 1.0), ang)
                        if j.name in self.weld
                        else _I3()
                    )
                    Rj = _mm3(R_po, Rq)
                    tj = _add(tp, _mv3(Rp, to))
                elif j.type == PRISMATIC:
                    ax_w = _mv3(R_po, j.axis or (0.0, 0.0, 1.0))
                    Rj = R_po
                    tj = _add(
                        _add(tp, _mv3(Rp, to)),
                        _scale(ax_w, q[self._jidxq[j.name]]),
                    )
                else:
                    Rq = _rot_axis(j.axis or (0.0, 0.0, 1.0), q[self._jidxq[j.name]])
                    Rj = _mm3(R_po, Rq)
                    tj = _add(tp, _mv3(Rp, to))
                world[j.child] = (Rj, tj)
                pending.remove(j)
                progressed = True
            if not progressed:
                raise RobotError("动力学 FK 断链")
        return world

    def jacobians(self, q: list[float]) -> dict:
        """一次刚性 FK → 所有动力 link 的 (R, COM 世界位置, 角/线雅可比行)。

        雅可比: J_ω[i][j] = 关节 j 轴的世界方向 (link i 在其子树内);
        J_v[i][j] = 轴 × (com_i − 关节原点) (旋转关节)。
        """
        world = self.rigid_fk(q)
        axes: dict[str, tuple] = {}
        origins: dict[str, tuple] = {}
        for j in self.joints:
            Rc, tc = world[j.child]
            axes[j.name] = _mv3(Rc, j.axis or (0.0, 0.0, 1.0))
            origins[j.name] = tc
        n = self.nq
        # 浮动基座虚拟列 (速度状态): 0-2 线速度 (J_v = e_i, J_ω = 0),
        # 3-5 角速度 (J_ω = e_i 世界轴, J_v = e_i × (com − 基座原点))
        p0 = (0.0, 0.0, 0.0)
        if self.floating:
            _R0, p0 = world[self.base_link]
        eye = (
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
        out = {}
        for lnk in self.links:
            R, t = world[lnk.name]
            com_w = _apply_rot_trans(R, t, lnk.com)
            jw: list = [None] * n
            jv: list = [None] * n
            if self.floating:
                for k in range(6):
                    if k < 3:
                        jw[k] = (0.0, 0.0, 0.0)
                        jv[k] = eye[k]
                    else:
                        jw[k] = eye[k - 3]
                        jv[k] = _cross(eye[k - 3], _sub(com_w, p0))
            for j in lnk.chain:
                if j.type == FIXED or j.name in self.weld:
                    continue
                idx = self._jidx[j.name]
                a, o = axes[j.name], origins[j.name]
                if j.type == PRISMATIC:
                    jw[idx] = (0.0, 0.0, 0.0)
                    jv[idx] = a  # 平动: 线速度沿轴, 无角速度
                else:
                    jw[idx] = a
                    jv[idx] = _cross(a, _sub(com_w, o))
            out[lnk.name] = (R, com_w, jw, jv)
        return out

    # ── 质量矩阵 (动能精确构造, 一次 FK) ────────────────────────

    def mass_matrix(self, q: list[float]) -> list[list[float]]:
        n = self.nq
        M = [[0.0] * n for _ in range(n)]
        kin = self.jacobians(q)  # 一次 FK 算所有 link
        for lnk in self.links:
            R, _com, jw, jv = kin[lnk.name]
            Iw = _world_inertia(R, lnk.inertia)
            for i in range(n):
                if jw[i] is None:
                    continue
                for jj in range(n):
                    if jw[jj] is None:
                        continue
                    M[i][jj] += (
                        lnk.mass * _dot(jv[i], jv[jj]) + _quad(jw[i], Iw, jw[jj])
                    )
        return M

    # ── 势能与重力 (解析: τ_j = Σ m·g·J_v[z], 零额外 FK) ────────

    def _potential(self, q: list[float]) -> float:
        world = self.rigid_fk(q)
        v = 0.0
        for lnk in self.links:
            R, t = world[lnk.name]
            z = _apply_rot_trans(R, t, lnk.com)[2]
            v += lnk.mass * GRAVITY * z
        return v

    def gravity_forces(self, q: list[float]) -> list[float]:
        """广义重力 g(q): τ_j = Σ_i m_i·g·J_v_ij[z] (解析, 用 M 同款雅可比)。

        重力对 link i 的广义力 = J_v_ijᵀ·(0,0,m_i·g) = m_i·g·J_v_ij[z];
        J_v_ij 在子树内非零, 零额外 FK (与 mass_matrix 共用一次 FK)。"""
        kin = self.jacobians(q)
        n = self.nq
        g = [0.0] * n
        for lnk in self.links:
            _R, _com, _jw, jv = kin[lnk.name]
            for jj in range(n):
                row = jv[jj]
                if row is not None:
                    g[jj] += lnk.mass * GRAVITY * row[2]
        return g

    # ── Coriolis/离心 (RNEA, 精确单遍) ──────────────────────────

    def coriolis_forces(self, q: list[float], qd: list[float]) -> list[float]:
        """C(q,q̇)q̇。固定基座: RNEA (ID(q,q̇,0) − g, 单遍精确)。

        浮动基座: 数值 Christoffel C = Ṁ·q̇ (M 沿运动的有限差分)。
        RNEA 在浮动基座漏掉基座线速度×旋转的 Coriolis 耦合项
        (基座 v 是循环坐标, RNEA 按 q̈=0 算力不含 Ṁ_pω·v), 与 M 不一致
        → 能量注入爆炸 (实测: 自由自旋体 0.5s 内 E +300000%)。
        浮动基座 + 四元数下 M(q) 的导数必须显式计入; float64 差分
        噪声 ~1e-9 相对, 无旧 Christoffel 的 float32 问题。
        """
        n = self.nq
        if not self.floating:
            tau, _ = self._rne_id(q, qd, [0.0] * n)
            g = self.gravity_forces(q)
            return [tau[i] - g[i] for i in range(n)]
        h = 1e-4
        qp = self.advance_pose(q, qd, h)
        qm = self.advance_pose(q, qd, -h)
        Mp = self.mass_matrix(qp)
        Mm = self.mass_matrix(qm)
        return [
            sum((Mp[i][j] - Mm[i][j]) / (2.0 * h) * qd[j] for j in range(n))
            for i in range(n)
        ]

    def reaction_wrenches(self, q, qd, qdd) -> dict:
        """RNEA 反推: 每 link 子树在 link 原点处的支撑 (力, 力矩)。

        F/T 传感器读数的基础 (结构力, 世界坐标)。"""
        _tau, wrenches = self._rne_id(q, qd, qdd)
        return wrenches

    def joint_indices(self, name: str) -> tuple[int, int]:
        """关节的速度列索引与配置位索引 (传感器/驱动器跨模块访问)。"""
        if name not in self._jidx:
            raise RobotError(f"关节 {name!r} 不存在或已焊死")
        return self._jidx[name], self._jidxq[name]

    def _rne_id(
        self, q: list[float], qd: list[float], qdd: list[float]
    ) -> tuple[list[float], dict]:
        """递归牛顿-欧拉逆向动力学: τ = M·q̈ + C·q̇ + g (世界坐标)。

        浮动基座: q̈ 前 6 个是基座虚拟加速度 (ID 时给 0 → C q̇ + g),
        返回的 τ 前 6 个是基座广义力 (平移 = 基座原点合力, 旋转 = 力矩
        在 ZYX 中间轴上的投影)。
        """
        world = self.rigid_fk(q)
        n = self.nq
        # 关节序 (父先于子, 含固定关节 —— 它们只传播无速度) + children
        children: dict[str, list] = {lnk.name: [] for lnk in self.links}
        order: list[str] = []
        for lnk in self.links:
            for j in lnk.chain:
                if j.name in order:
                    continue
                order.append(j.name)
        for j in self.robot.joints:
            if j.child in children and j.parent in children:
                children[j.parent].append(j.child)
        # 每 link 的世界 (R, t), com 世界, 世界惯量; 根位姿 (固定基座 = world)
        R = {lnk.name: world[lnk.name][0] for lnk in self.links}
        t = {lnk.name: world[lnk.name][1] for lnk in self.links}
        if not self.floating:
            R[self.root] = _I3()
            t[self.root] = (0.0, 0.0, 0.0)
        com = {}
        Iw = {}
        for lnk in self.links:
            com[lnk.name] = _apply_rot_trans(R[lnk.name], t[lnk.name], lnk.com)
            Iw[lnk.name] = _world_inertia(R[lnk.name], lnk.inertia)
        mass = {lnk.name: lnk.mass for lnk in self.links}
        base = self.base_link if self.floating else self.root
        # 基座角速度 = 速度状态 qd[3..5] (世界 ω, 直接取; ID q̈=0 → α_0 = 0)
        if self.floating:
            w0 = (qd[3], qd[4], qd[5])
        else:
            w0 = (0.0, 0.0, 0.0)
        w = {base: w0}
        wd = {base: (0.0, 0.0, 0.0)}  # α_0 = 0 (q̈_base = 0)
        # 基座线加速度 = 重力等效 (0,0,+g): 基座以 g 向上加速 ≡ 重力向下
        # (标准 RNEA 技巧, 不显式加重力外力; 符号验证: 摆锤得平衡力矩)
        a = {base: (0.0, 0.0, GRAVITY)}
        # 正向: 速度/加速度传播
        ac = {}
        if base in mass:
            r_cc = _sub(com[base], t[base])
            ac[base] = _add(
                a[base],
                _cross(w[base], _cross(w[base], r_cc)),  # α=0 → 只有离心
            )
        jby_name = {j.name: j for j in self.robot.joints}
        jby = {j.child: j for j in self.robot.joints}
        for name in order:
            j = jby_name[name]
            parent = j.parent
            Rc = world[j.child][0]
            tc = world[j.child][1]
            axis_w = _mv3(Rc, j.axis or (0.0, 0.0, 1.0))
            r_pc = _sub(tc, t[parent])
            if j.type == FIXED or j.name in self.weld:
                qd_j = qdd_j = 0.0  # 固定/焊死: 无速度贡献, 只传播位姿
            else:
                col = self._jidx[j.name]
                qd_j, qdd_j = qd[col], qdd[col]

            if j.type == PRISMATIC:
                # 平动关节: 无旋转贡献; 加速度含 q̈ + 2·ω×s·q̇ (运动轴 Coriolis)
                w[j.child] = w[parent]
                wd[j.child] = wd[parent]
                s_vel = _scale(axis_w, qd_j)
                a[j.child] = _add(
                    _add(a[parent], _scale(axis_w, qdd_j)),
                    _add(
                        _scale(_cross(w[parent], s_vel), 2.0),
                        _add(
                            _cross(wd[parent], r_pc),
                            _cross(w[parent], _cross(w[parent], r_pc)),
                        ),
                    ),
                )
            else:
                wj = _scale(axis_w, qd_j)
                w[j.child] = _add(w[parent], wj)
                wd[j.child] = _add(
                    wd[parent], _add(_scale(axis_w, qdd_j), _cross(w[parent], wj))
                )
                a[j.child] = _add(
                    _add(a[parent], _cross(wd[parent], r_pc)),
                    _cross(w[parent], _cross(w[parent], r_pc)),
                )
            r_cc = _sub(com[j.child], tc)
            ac[j.child] = _add(
                _add(a[j.child], _cross(wd[j.child], r_cc)),
                _cross(w[j.child], _cross(w[j.child], r_cc)),
            )
        # 反向: 力/力矩传播 (从末端到基座, 含基座 link 自身)
        F, N, f, nn = {}, {}, {}, {}
        tau = [0.0] * n
        rev = [lnk.name for lnk in reversed(self.links)]
        for name in rev:
            m_i = mass[name]
            I_w = Iw[name]
            r_cc = _sub(com[name], t[name])
            F[name] = _scale(ac[name], m_i)  # 重力经基座等效加速度进入
            N[name] = _add(
                _imv3(I_w, wd[name]), _cross(w[name], _imv3(I_w, w[name]))
            )
            fi = _add(F[name], (0.0, 0.0, 0.0))
            nn_i = _add(N[name], _cross(r_cc, F[name]))
            for ch in children.get(name, []):
                if ch not in F:
                    continue
                r_ic = _sub(t[ch], t[name])
                fi = _add(fi, f[ch])
                nn_i = _add(nn_i, _add(nn[ch], _cross(r_ic, f[ch])))
            f[name] = fi
            nn[name] = nn_i
            if name != base:
                j = jby[name]
                if j.type != FIXED and j.name not in self.weld:
                    col = self._jidx[j.name]
                    axis_w = _mv3(world[j.child][0], j.axis or (0.0, 0.0, 1.0))
                    if j.type == PRISMATIC:
                        tau[col] = _dot(axis_w, f[name])  # 力沿轴
                    else:
                        tau[col] = _dot(axis_w, nn_i)  # 力矩沿轴
        # 基座虚拟广义力 (浮动): 平移 = 基座原点合力, 旋转 = 世界轴力矩
        if self.floating:
            fb = (0.0, 0.0, 0.0)
            nb = (0.0, 0.0, 0.0)
            if base in f:
                fb = f[base]
                nb = nn[base]
            tau[0], tau[1], tau[2] = fb
            tau[3], tau[4], tau[5] = nb
        return tau, {name: (f[name], nn[name]) for name in f}

    # ── Drake 风格 API ──────────────────────────────────────────

    def inverse_dynamics(self, q, qd, qdd) -> list[float]:
        """τ = M·q̈ + C·q̇ + g。"""
        M = self.mass_matrix(q)
        c = self.coriolis_forces(q, qd)
        g = self.gravity_forces(q)
        n = self.nq
        return [sum(M[i][j] * qdd[j] for j in range(n)) + c[i] + g[i] for i in range(n)]

    def forward_dynamics(self, q, qd, tau) -> list[float]:
        """q̈ = M⁻¹(τ − C·q̇ − g)。"""
        n = self.nq
        c = self.coriolis_forces(q, qd)
        g = self.gravity_forces(q)
        rhs = [tau[i] - c[i] - g[i] for i in range(n)]
        M = self.mass_matrix(q)
        return solve_linear(M, rhs)

    def advance_pose(
        self, q: list[float], qd: list[float], dt: float
    ) -> list[float]:
        """位姿推进 (半隐式, 用新速度): 平移 + 四元数旋转 + 关节。

        浮动基座 q = [p, 四元数(qx,qy,qz,qw), 关节], qd = [v, ω, 关节]。
        四元数积分 q̇ = ½·Ω(ω)·q 后归一化 —— 无欧拉角奇异, 大角度
        (倾倒/翻转) 稳定; 旋转无外部角速度守恒 (自由飞行不漂转)。"""
        q_new = list(q)
        if self.floating:
            for k in range(3):
                q_new[k] += qd[k] * dt
            dq = _quat_deriv(
                (qd[3], qd[4], qd[5]), (q[3], q[4], q[5], q[6])
            )
            for k in range(4):
                q_new[3 + k] += dq[k] * dt
            q_new[3], q_new[4], q_new[5], q_new[6] = _quat_normalize(
                (q_new[3], q_new[4], q_new[5], q_new[6])
            )
        for j in self.joints:
            iq = self._jidxq[j.name]
            q_new[iq] += qd[self._jidx[j.name]] * dt
        return q_new

    def _apply_damping(
        self, tau: list[float], qd: list[float]
    ) -> list[float]:
        """τ_eff = τ − b·q̇ (按关节列索引; 虚拟基座关节无阻尼)。"""
        tau_eff = list(tau)
        for j in self.joints:
            i = self._jidx[j.name]
            tau_eff[i] -= self._damping[j.name] * qd[i]
        return tau_eff

    def _clamp_limits(
        self, q_new: list[float], qd_new: list[float]
    ) -> tuple[list[float], list[float]]:
        """关节限位 (撞限位速度归零)。"""
        for j in self.joints:
            iq = self._jidxq[j.name]
            iv = self._jidx[j.name]
            lo, hi = j.lower, j.upper
            if lo is not None and q_new[iq] < lo:
                q_new[iq] = lo
                if qd_new[iv] < 0:
                    qd_new[iv] = 0.0
            if hi is not None and q_new[iq] > hi:
                q_new[iq] = hi
                if qd_new[iv] > 0:
                    qd_new[iv] = 0.0
        return q_new, qd_new

    def integrate(self, q, qd, tau, dt: float) -> tuple[list[float], list[float]]:
        """半隐式欧拉一步 + 关节限位 (撞限位速度归零) + CRDF 阻尼。"""
        n = self.nq
        tau_eff = self._apply_damping(tau, qd)
        qdd = self.forward_dynamics(q, qd, tau_eff)
        qd_new = [qd[i] + qdd[i] * dt for i in range(n)]
        q_new = self.advance_pose(q, qd_new, dt)
        return self._clamp_limits(q_new, qd_new)

    # ── RK4 与变步长 (Dormand-Prince 45, Drake 移植) ───────────

    def integrate_rk4(
        self, q, qd, tau, dt: float
    ) -> tuple[list[float], list[float]]:
        """经典 4 阶 Runge-Kutta 一步。能量漂移 O(dt⁴) ≪ 半隐式欧拉
        O(dt) (实测自由自旋 2.23% → 0.02% @dt=2e-3); 无耗散场景精度
        高。位姿走 advance_pose (四元数归一), 无欧拉角奇异。
        注意: RK4 假定动力学平滑 —— 接触/撞击 (不连续力) 场景用
        半隐式欧拉或隐式接触 (integrate_implicit)。"""
        n = self.nq
        tau_eff = self._apply_damping(tau, qd)

        def acc(qq, qqd):
            return self.forward_dynamics(qq, qqd, tau_eff)

        k1 = acc(q, qd)
        v2 = [qd[i] + k1[i] * dt / 2.0 for i in range(n)]
        k2 = acc(self.advance_pose(q, qd, dt / 2.0), v2)
        v3 = [qd[i] + k2[i] * dt / 2.0 for i in range(n)]
        k3 = acc(self.advance_pose(q, v2, dt / 2.0), v3)
        v4 = [qd[i] + k3[i] * dt for i in range(n)]
        k4 = acc(self.advance_pose(q, v3, dt), v4)
        v_new = [
            qd[i] + (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * dt / 6.0
            for i in range(n)
        ]
        # 位置用阶段速度加权平均 (非末速度): 常量加速度下精确
        v_avg = [
            (qd[i] + 2.0 * v2[i] + 2.0 * v3[i] + v4[i]) / 6.0 for i in range(n)
        ]
        q_new = self.advance_pose(q, v_avg, dt)
        return self._clamp_limits(q_new, v_new)

    def integrate_adaptive(
        self,
        q,
        qd,
        tau,
        t_end: float,
        dt0: float = 1e-2,
        rtol: float = 1e-4,
        atol: float = 1e-6,
    ) -> tuple[list[float], list[float]]:
        """变步长积分到 t_end (Dormand-Prince 45, 嵌入式 4/5 阶误差估计)。

        h 按 0.9·h·(tol/err)^(1/5) 自适应: 光滑段大步 (自由飞行),
        陡峭段自动加密。返回 t_end 处的 (q, qd)。与 RK4 同样要求
        动力学平滑 (接触用隐式接触/固定步长)。"""
        tau_eff = self._apply_damping(tau, qd)
        n = self.nq
        t = 0.0
        h = dt0
        while t < t_end - 1e-12:
            h = min(h, t_end - t)
            q5, qd5, err = self._dp45_step(q, qd, tau_eff, h)
            scale = [
                atol + rtol * max(abs(qd[i]), abs(qd5[i])) for i in range(n)
            ]
            err_norm = max(err[i] / scale[i] for i in range(n))
            if err_norm <= 1.0:
                q, qd = q5, qd5
                t += h
                h *= 2.0 if err_norm < 0.2 else 1.0
            else:
                h *= max(0.2, 0.9 * (1.0 / err_norm) ** 0.2)
        return q, qd

    def _dp45_step(self, q, qd, tau_eff, h):
        """Dormand-Prince 45 一步: (q5, v5, 误差估计) (v4/v5 差)。"""
        n = self.nq
        ks = []
        vs_all = []
        vs = qd
        for i in range(7):
            if i > 0:
                vs = [
                    qd[j] + sum(_DP_A[i][m] * ks[m][j] for m in range(i)) * h
                    for j in range(n)
                ]
                qs = self.advance_pose(q, vs, _DP_C[i] * h)
            else:
                qs = q
            vs_all.append(vs)
            ks.append(self.forward_dynamics(qs, vs, tau_eff))
        v5 = [
            qd[j] + sum(_DP_B5[m] * ks[m][j] for m in range(7)) * h
            for j in range(n)
        ]
        v4 = [
            qd[j] + sum(_DP_B4[m] * ks[m][j] for m in range(7)) * h
            for j in range(n)
        ]
        # 位置用 b5 加权阶段速度 (v_stage_m = qd + h·Σa·k 已记录),
        # 而非末速度 —— 常量加速度下精确
        v_avg = [
            sum(_DP_B5[m] * vs_all[m][j] for m in range(7)) for j in range(n)
        ]
        return (
            self.advance_pose(q, v_avg, h),
            v5,
            [abs(v5[j] - v4[j]) for j in range(n)],
        )

    def jacobian_at(
        self, q: list[float], link_name: str, p_world: tuple
    ) -> list[tuple[float, float, float] | None]:
        """世界点 p 的线速度雅可比 (接触投影用): J_p[j] = J_v[j] + J_ω[j]×(p−com)。

        接触力 F 作用于 p → 广义力 τ_c[j] = J_p[j]·F。
        """
        kin = self.jacobians(q)
        if link_name not in kin:
            raise RobotError(f"link {link_name!r} 无动力学数据")
        _R, com, jw, jv = kin[link_name]
        n = self.nq
        out = []
        for j in range(n):
            if jw[j] is None:
                out.append(None)
            else:
                out.append(_add(jv[j], _cross(jw[j], _sub(p_world, com))))
        return out

    def kinetic_energy(self, q, qd) -> float:
        """K = ½ q̇ᵀ M q̇ (验证用)。"""
        M = self.mass_matrix(q)
        n = self.nq
        return 0.5 * sum(M[i][j] * qd[i] * qd[j] for i in range(n) for j in range(n))

    def potential_energy(self, q) -> float:
        return self._potential(q)

    def link_twists(self, q, qd) -> dict[str, Motor]:
        """每个动力 link 的空间速度 = twist 二重向量 (CGA)。

        twist = ω (角速度) + v_com (质心线速度), velocity_bivector 构造。
        """
        kin = self.jacobians(q)
        twists = {}
        for lnk in self.links:
            k = kin[lnk.name]
            w = [0.0, 0.0, 0.0]
            v = [0.0, 0.0, 0.0]
            for i, j in enumerate(self.joints):
                jw, jv = k["Jw"][i], k["Jv"][i]
                if jw is None:
                    continue
                w = _add(w, _scale(jw, qd[i]))
                v = _add(v, _scale(jv, qd[i]))
            twists[lnk.name] = Motor.velocity_bivector(tuple(w), tuple(v))
        return twists


# ── 3D 向量/矩阵工具 (纯 Python, 无 numpy) ───────────────────────


def _I3() -> list[list[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _mm3(A, B) -> list[list[float]]:
    """3×3 矩阵乘。"""
    return [
        [
            A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]
            for j in range(3)
        ]
        for i in range(3)
    ]


def _mv3(A, v) -> tuple[float, float, float]:
    """3×3 × 向量。"""
    return (
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    )


# Dormand-Prince 45 Butcher 表 (嵌入式 4/5 阶误差估计)
_DP_A = (
    (),
    (1 / 5,),
    (3 / 40, 9 / 40),
    (44 / 45, -56 / 15, 32 / 9),
    (19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729),
    (9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656),
    (35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84),
)
_DP_B5 = (35 / 384, 0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84, 0)
_DP_B4 = (5179 / 57600, 0, 7571 / 16695, 393 / 640, -92097 / 339200, 187 / 2100, 1 / 40)
_DP_C = (0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1, 1)


def _quat_to_R(x, y, z, w) -> list[list[float]]:
    """四元数 (x,y,z,w) → 旋转矩阵 (Hamilton, 单位化)。"""
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0.0:
        return _I3()
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def _quat_deriv(omega, quat) -> tuple[float, float, float, float]:
    """q̇ = ½·q⊗ω̂ (世界角速度 ω, Hamilton 积)。"""
    wx, wy, wz = omega
    x, y, z, w = quat
    return (
        0.5 * (w * wx + y * wz - z * wy),
        0.5 * (w * wy + z * wx - x * wz),
        0.5 * (w * wz + x * wy - y * wx),
        0.5 * (-(x * wx + y * wy + z * wz)),
    )


def _quat_normalize(q) -> tuple[float, float, float, float]:
    x, y, z, w = q
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)  # 数值退化守卫: 回单位四元数
    return (x / n, y / n, z / n, w / n)


def _rot_axis(axis, angle) -> list[list[float]]:
    """Rodrigues: 绕单位轴转 angle 的旋转矩阵。"""
    ax, ay, az = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return [
        [c + ax * ax * C, ax * ay * C - az * s, ax * az * C + ay * s],
        [ay * ax * C + az * s, c + ay * ay * C, ay * az * C - ax * s],
        [az * ax * C - ay * s, az * ay * C + ax * s, c + az * az * C],
    ]


def _apply_rot_trans(R, t, v):
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2] + t[0],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2] + t[1],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2] + t[2],
    )


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _imv3(I6, v):
    """6 分量对称惯量 × 向量。"""
    ixx, iyy, izz, ixy, ixz, iyz = I6
    return (
        ixx * v[0] + ixy * v[1] + ixz * v[2],
        ixy * v[0] + iyy * v[1] + iyz * v[2],
        ixz * v[0] + iyz * v[1] + izz * v[2],
    )


def _quad(a, I6, b):
    """aᵀ·I·b (3×3 对称惯量, I6 = (ixx,iyy,izz,ixy,ixz,iyz))。"""
    Ib = (
        I6[0] * b[0] + I6[3] * b[1] + I6[4] * b[2],
        I6[3] * b[0] + I6[1] * b[1] + I6[5] * b[2],
        I6[4] * b[0] + I6[5] * b[1] + I6[2] * b[2],
    )
    return _dot(a, Ib)


def _world_inertia(R, I6) -> tuple:
    """I_world = R·I_local·Rᵀ (3×3 对称, 返回 6 分量)。

    曾写成 R·I·R (缺转置): 旋转态下是"双重旋转"的相似变换之误 ——
    惯量张量特征值被改变 → M 不定 (负特征值) → 浮动基座旋转态爆炸。
    固定基座小角度测试 R≈I 掩盖了此 bug。"""
    ixx, iyy, izz, ixy, ixz, iyz = I6
    il = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            s = 0.0
            for a in range(3):
                ra = R[i][a]
                for b in range(3):
                    s += ra * il[a][b] * R[j][b]
            out[i][j] = s
    return (out[0][0], out[1][1], out[2][2], out[0][1], out[0][2], out[1][2])


def solve_linear(A, b) -> list[float]:
    """高斯消元解 Ax = b (n×n, 纯 Python)。"""
    n = len(b)
    m = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise RobotError(f"质量矩阵奇异 (q 处退化?) col={col}")
        m[col], m[pivot] = m[pivot], m[col]
        inv = 1.0 / m[col][col]
        for j in range(col, n + 1):
            m[col][j] *= inv
        for r in range(n):
            if r != col:
                f = m[r][col]
                for j in range(col, n + 1):
                    m[r][j] -= f * m[col][j]
    return [m[i][n] for i in range(n)]
