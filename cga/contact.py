"""接触模型 —— 移植 Drake 惩罚接触 (compliant penalty contact)。

碰撞几何: CRDF 的 collision 圆柱 (blade=cylinder) 解析求最低点 vs 平面。
接触力: 法向惩罚弹簧-阻尼 + 库仑摩擦 (平滑饱和):
    f_n = max(0, k·δ − b·v_n)           v_n = 接触点沿法向速度
    f_t = −μ·f_n·sat(v_t/ε)             v_t = 切向速度
接触力经接触点线速度雅可比投影为广义力 (plant.jacobian_at):
    τ_c = J_pᵀ·F,  叠加进 forward_dynamics 的 τ。

平面: (n̂, d), n̂·x = d, n̂ 指向物体 (地面 = ((0,0,1), 0), 可加桌面等)。
"""

from __future__ import annotations

from dataclasses import dataclass

from cga.dynamics import DynamicsPlant, solve_linear


@dataclass
class ContactModel:
    """接触模型。

    两套方法 (Drake 的 penalty 与 impulse 两类接触):
    - `generalized_forces` + plant.integrate: 惩罚弹簧-阻尼 (轻接触/静止稳);
    - `integrate_implicit`: 速度级脉冲 + 位置修正 (Box2D 式, 冲击/翻滚稳)。
    惩罚法在撞击下失稳 (接触刚度×穿透指数发散), 坠落场景用隐式法。
    """

    stiffness: float = 2e4  # N/m (惩罚法): 4.3kg 臂静止穿透 ~2mm
    damping: float = 80.0  # N·s/m (惩罚法)
    friction: float = 0.7  # 库仑摩擦系数
    v_stick: float = 0.05  # 摩擦平滑阈值 (m/s, 惩罚法)
    slop: float = 0.005  # 允许穿透 (m, 隐式法): 接触判定/位置修正阈值
    pos_beta: float = 0.2  # 位置修正每步去除比例 (隐式法)
    max_corr: float = 0.005  # 位置修正单步封顶 (m): 防深穿透巨推放大
    planes: tuple = ((0.0, 0.0, 1.0, 0.0),)  # (n̂, d): 地面 z=0

    def integrate_implicit(
        self,
        plant: DynamicsPlant,
        q: list[float],
        qd: list[float],
        tau: list[float],
        dt: float,
    ) -> tuple[list[float], list[float]]:
        """隐式接触步 (速度级脉冲 + 位置修正, Box2D 式两阶段)。

        惩罚弹簧撞击失稳 (接触刚度×穿透指数发散, 实测翻滚体必炸),
        此法冲击稳定:
        1. 速度求解 (Gauss-Seidel 3 轮): 法向脉冲使接触点法向速度归零
           (e=0 非弹性, 不注入能量); 库仑摩擦脉冲带限幅 |λ_t| ≤ μ·λ_n;
        2. 位置修正: 深穿透 (δ > slop) 沿法向拉回, 只动位置不动速度。
        """
        n = plant.nq
        qdd_free = plant.forward_dynamics(q, qd, tau)
        qd_free = [qd[i] + qdd_free[i] * dt for i in range(n)]
        M = plant.mass_matrix(q)
        qd_new = self._impulse_solve(plant, M, q, qd_free)
        q_next = plant.advance_pose(q, qd_new, dt)
        # 位置修正: 用步初接触 (固定雅可比, 不重估几何 —— 重估会追逐
        # 被转动后的新穿透, 两轮叠加放大 → 爆炸), 修正量封顶。
        contacts = self._contact_points(plant, q)
        for _ in range(2):
            q_next = self._position_correct(plant, contacts, M, q_next)
        return q_next, qd_new

    def _contact_points(self, plant: DynamicsPlant, q: list[float]) -> list:
        """当前状态的全部接触: [(δ, 接触点, 法向, 点雅可比 J_p)]。"""
        n = plant.nq
        world = plant.rigid_fk(q)
        kin = plant.jacobians(q)
        out = []
        for (lnk_name, Ro, to, r, h) in self._collision_geoms(plant):
            Rl, tl = world[lnk_name]
            R = _mm3(Rl, Ro)
            c = _add(tl, _mv3(Rl, to))
            u = _mv3(R, (0.0, 0.0, 1.0))
            for (nx, ny, nz, d) in self.planes:
                hit = _cylinder_contact(c, u, r, h, (nx, ny, nz), d)
                if hit is None:
                    continue
                delta, p, nrm = hit
                _R, com, jw, jv = kin[lnk_name]
                jp: list = [None] * n
                for j in range(n):
                    if jw[j] is not None:
                        jp[j] = _add(jv[j], _cross(jw[j], _sub(p, com)))
                out.append((delta, p, nrm, jp))
        return out

    def _impulse_solve(
        self,
        plant: DynamicsPlant,
        M: list[list[float]],
        q: list[float],
        qd_free: list[float],
    ) -> list[float]:
        """Gauss-Seidel 接触脉冲: 法向 v_n→0 (e=0) + 库仑摩擦限幅。"""
        n = plant.nq
        qd_new = list(qd_free)
        contacts = [c for c in self._contact_points(plant, q) if c[0] > 0.0]
        if not contacts:
            return qd_new
        for _ in range(3):
            for (delta, p, nrm, jp) in contacts:
                jn = [
                    (_dot(jp[j], nrm) if jp[j] is not None else 0.0) for j in range(n)
                ]
                vn = sum(jn[j] * qd_new[j] for j in range(n))
                if vn > 0:  # 分离中, 无脉冲
                    continue
                x = solve_linear(M, jn)
                A = sum(jn[j] * x[j] for j in range(n))
                if A < 1e-12:
                    continue
                lam = -vn / A
                vp = [0.0, 0.0, 0.0]
                for j in range(n):
                    if jp[j] is not None:
                        vp = _add(vp, _scale(jp[j], qd_new[j]))
                vt = _sub(vp, _scale(nrm, vn))
                vt_mag = _norm(vt)
                for j in range(n):
                    qd_new[j] += x[j] * lam
                if vt_mag > 1e-9:  # 摩擦脉冲 (Coulomb 限幅)
                    t = _scale(vt, 1.0 / vt_mag)
                    jt = [
                        (_dot(jp[j], t) if jp[j] is not None else 0.0) for j in range(n)
                    ]
                    xt = solve_linear(M, jt)
                    At = sum(jt[j] * xt[j] for j in range(n))
                    if At > 1e-12:
                        lam_t = -sum(jt[j] * qd_new[j] for j in range(n)) / At
                        lam_t = max(
                            -self.friction * lam, min(self.friction * lam, lam_t)
                        )
                        for j in range(n):
                            qd_new[j] += xt[j] * lam_t
        return qd_new

    def _position_correct(
        self,
        plant: DynamicsPlant,
        contacts: list,
        M: list[list[float]],
        q: list[float],
    ) -> list[float]:
        """位置修正: 深穿透 (δ>slop) 沿法向拉回 (β 比例), 不动速度。

        contacts 来自步初 (固定雅可比); 修正以伪速度经 advance_pose
        施加 (四元数位姿不直接加分量, 保证归一); 每步封顶 max_corr,
        深穿透 (高速扫掠) 分多步解析而非一步巨推。"""
        n = plant.nq
        q2 = q
        for (delta, p, nrm, jp) in contacts:
            if delta <= self.slop:
                continue
            jn = [(_dot(jp[j], nrm) if jp[j] is not None else 0.0) for j in range(n)]
            x = solve_linear(M, jn)
            A = sum(jn[j] * x[j] for j in range(n))
            if A < 1e-12:
                continue
            corr = min(
                self.pos_beta * (delta - self.slop), self.max_corr
            )  # 正: 沿法向推出
            qd_corr = [x[j] * corr / A for j in range(n)]
            q2 = plant.advance_pose(q2, qd_corr, 1.0)
        return q2

    def generalized_forces(
        self, plant: DynamicsPlant, q: list[float], qd: list[float]
    ) -> list[float]:
        """所有 link 碰撞圆柱 vs 全部平面的接触力 → 广义力。

        热路径: 几何 origin (R,t) 静态预计算 (避免 MLX to_matrix 每步
        ~10ms×8); 雅可比一次算 (jacobian_at 每接触调一次是 ~0.1ms 重复)。"""
        n = plant.nq
        tau = [0.0] * n
        world = plant.rigid_fk(q)
        kin = plant.jacobians(q)
        geoms = self._collision_geoms(plant)
        for (lnk_name, Ro, to, r, h) in geoms:
            Rl, tl = world[lnk_name]
            R = _mm3(Rl, Ro)
            c = _add(tl, _mv3(Rl, to))
            u = _mv3(R, (0.0, 0.0, 1.0))
            for (nx, ny, nz, d) in self.planes:
                nrm = (nx, ny, nz)
                hit = _cylinder_contact(c, u, r, h, nrm, d)
                if hit is None:
                    continue
                delta, p, nrm = hit
                _R, com, jw, jv = kin[lnk_name]
                # 接触点线速度雅可比: J_p[j] = J_v[j] + J_ω[j]×(p−com)
                vp = [0.0, 0.0, 0.0]
                for j in range(n):
                    if jw[j] is not None:
                        jp = _add(jv[j], _cross(jw[j], _sub(p, com)))
                        vp = _add(vp, _scale(jp, qd[j]))
                vn = _dot(vp, nrm)
                fn = max(0.0, self.stiffness * delta - self.damping * vn)
                if fn <= 0:
                    continue
                vt = _sub(vp, _scale(nrm, vn))
                vt_mag = _norm(vt)
                ft = [0.0, 0.0, 0.0]
                if vt_mag > 1e-12:
                    s = self.friction * fn * min(1.0, vt_mag / self.v_stick)
                    ft = _scale(vt, -s / vt_mag)
                F = _add(_scale(nrm, fn), ft)
                for j in range(n):
                    if jw[j] is not None:
                        jp = _add(jv[j], _cross(jw[j], _sub(p, com)))
                        tau[j] += _dot(jp, F)
        return tau

    def _collision_geoms(self, plant: DynamicsPlant) -> list:
        """碰撞圆柱的静态参数 [(link, Ro, to, r, h)] (origin 预计算缓存)。"""
        key = id(plant)
        if getattr(self, "_geom_cache", None) and self._geom_cache[0] == key:
            return self._geom_cache[1]
        geoms = []
        for lnk in plant.robot.links:
            for g in lnk.geometry:
                if "collision" not in g.role or g.blade != "cylinder":
                    continue
                mt = g.origin.to_matrix()
                Ro = [[float(mt[i][j]) for j in range(3)] for i in range(3)]
                to = (float(mt[0][3]), float(mt[1][3]), float(mt[2][3]))
                geoms.append((lnk.name, Ro, to, g.radius, g.length / 2.0))
        self._geom_cache = (key, geoms)
        return geoms

    def forces(self, plant, q, qd) -> list[tuple]:
        """调试/自检: 返回 [(link, 穿透, 法向力, 法向速度)] 接触明细。"""
        out = []
        n = plant.nq
        world = plant.rigid_fk(q)
        for lnk in plant.robot.links:
            if lnk.name not in world:
                continue
            Rl, tl = world[lnk.name]
            for g in lnk.geometry:
                if "collision" not in g.role or g.blade != "cylinder":
                    continue
                mt = g.origin.to_matrix()
                Ro = [[float(mt[i][j]) for j in range(3)] for i in range(3)]
                to = (float(mt[0][3]), float(mt[1][3]), float(mt[2][3]))
                R = _mm3(Rl, Ro)
                c = _add(tl, _mv3(Rl, to))
                u = _mv3(R, (0.0, 0.0, 1.0))
                for (nx, ny, nz, d) in self.planes:
                    hit = _cylinder_contact(
                        c, u, g.radius, g.length / 2.0, (nx, ny, nz), d
                    )
                    if hit is None:
                        continue
                    delta, p, nrm = hit
                    jp = plant.jacobian_at(q, lnk.name, p)
                    vp = [0.0, 0.0, 0.0]
                    for j in range(n):
                        if jp[j] is not None:
                            vp = _add(vp, _scale(jp[j], qd[j]))
                    vn = _dot(vp, nrm)
                    fn = max(0.0, self.stiffness * delta - self.damping * vn)
                    if fn > 0:
                        out.append((lnk.name, delta, fn, vn))
        return out


    # ── 圆柱 vs 平面: 最低点解析求交 ─────────────────────────────────


def _cylinder_contact(c, u, r, h, n, d):
    """有限圆柱 (中心 c, 轴 u 单位, 半径 r, 半长 h) vs 平面 n̂·x = d。

    最低点候选: 侧面 (n̂·u ≠ ±1 时) + 两个端盖; 取 n̂·x − d 最小者。
    返回 (穿透 δ, 接触点, 法向 n̂) 或 None (未接触)。
    """
    du = _dot(n, u)
    candidates = []
    side = _sub(_scale(u, du), n)  # (n̂·u)u − n̂: 径向最深处
    sl = _norm(side)
    if sl > 1e-9:
        v = _scale(side, 1.0 / sl)
        candidates.append(_add(c, _scale(v, r)))
    candidates.append(_add(c, _scale(u, h)))
    candidates.append(_sub(c, _scale(u, h)))
    best = min(candidates, key=lambda p: _dot(n, p) - d)
    sdist = _dot(n, best) - d
    if sdist >= 0:
        return None
    return (-sdist, best, n)


# ── 3D 工具 (纯 Python, 与 dynamics 同族) ────────────────────────


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


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) ** 0.5
