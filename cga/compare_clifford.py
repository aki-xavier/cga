"""数值准确性对比: src/cga  vs  clifford 库 (pygae/clifford, g3c Cl(4,1))。

约定对齐 (已实测): clifford.g3c 的 eo = ½(e5−e4), einf = e4+e5 与本包
e0/e∞ 一致 (e0²=e∞²=0, e0·e∞=−1); 伪标量 I = e12345, I² = −1, 故
dual(A) = A·I⁻¹ = −A·I。基变换: e0 = ½(e5−e4), e∞ = e4+e5, 其余相同。

对比方式: 同一运算在两库各算一遍, 结果都映到本包的 32 分量向量
(经基变换矩阵的逆), 报 max|Δ|。本包是 float32 存储, clifford 是
float64 —— 误差量级本身就是对比结论的一部分。

运行: PYTHONPATH=src python src/cga/compare_clifford.py
依赖: uv sync --extra compare (clifford→numba 0.66 要求 numpy<2.5, 故
主依赖的 numpy 上限为 <2.5; 本对比脚本不用 numba 的 JIT 路径)。
"""

import math
from functools import reduce

import mlx.core as mx
import numpy as np
from clifford.g3c import blades

from cga import Circle, Line, Motor, Multivector, Plane, Point, Sphere
from cga.multivector import BASIS_BLADES

# ── 基变换: 本包 null 基 ↔ clifford e4/e5 基 ───────────────────────

e1, e2, e3, e4, e5 = (blades[k] for k in ("e1", "e2", "e3", "e4", "e5"))
EO = 0.5 * (e5 - e4)  # clifford 侧的原点 (e0)
EI = e4 + e5  # clifford 侧的无穷远点 (e∞)
VMAP = {0: e1, 1: e2, 2: e3, 3: EO, 4: EI}
ONE = blades[""]  # 标量 blade

_BLADE_CF = []  # 本包第 i 个基 blade 的 clifford 表示
for t in BASIS_BLADES:
    _BLADE_CF.append(reduce(lambda a, v: a ^ VMAP[v], t, ONE))

# 基变换矩阵 (clifford 32 ← 本包 32) 及其逆, float64
_A = np.stack([np.asarray(b.value, dtype=np.float64) for b in _BLADE_CF], axis=1)
_A_INV = np.linalg.inv(_A)


def ours_to_cf(mv: Multivector):
    """本包 Multivector → clifford MultiVector。"""
    vals = np.asarray(mv.values, dtype=np.float64)
    acc = 0.0 * ONE  # MultiVector 零 (sum() 的 int 起点会让 pyright 误判)
    for i in range(32):
        if vals[i] != 0:
            acc = acc + float(vals[i]) * _BLADE_CF[i]
    return acc


def cf_to_ours(cf) -> np.ndarray:
    """clifford MultiVector → 本包 32 分量向量 (numpy float64)。"""
    return _A_INV @ np.asarray(cf.value, dtype=np.float64)


def ours_vec(mv: Multivector) -> np.ndarray:
    """本包 Multivector → 32 分量 numpy 向量 (float64)。"""
    return np.asarray(mv.values, dtype=np.float64)


# ── 对比框架 ──────────────────────────────────────────────────────

_ok = 0


def check(name: str, a: np.ndarray, b: np.ndarray, tol: float = 2e-4) -> None:
    """max|Δ| 超 tol 即失败 (float32 vs float64 的正常量级 ~1e-6..1e-5)。"""
    global _ok
    err = float(np.max(np.abs(a - b))) if a.size else 0.0
    assert err < tol, f"FAIL: {name} max|Δ|={err:.3e} >= {tol:.0e}"
    _ok += 1
    print(f"  ok  {name:<42s} max|Δ| = {err:.2e}")


def rand_mv(seed: int, dense: bool = True) -> Multivector:
    """随机 multivector (dense=True 稠密; False 随机稀疏化 70%)。"""
    rng = np.random.default_rng(seed)
    v = rng.normal(size=32)
    if not dense:
        v[rng.random(32) < 0.7] = 0.0  # 稀疏混合 grade
    return Multivector(mx.array(v, dtype=mx.float32))


def main() -> None:
    """全面对比: 代数 / 图元 / versor / exp-log / 矩阵 / 距离。"""
    print("== 代数运算 (随机稠密/稀疏 multivector) ==")
    I_cf = blades["e12345"]
    for seed, dense in ((0, True), (1, True), (2, False)):
        a, b = rand_mv(seed, dense), rand_mv(seed + 100, dense)
        ac, bc = ours_to_cf(a), ours_to_cf(b)
        tag = f"seed{seed}{' dense' if dense else ' sparse'}"
        check(f"gp   {tag}", ours_vec(a.gp(b)), cf_to_ours(ac * bc))
        # ip 已与 clifford 的 | (fat dot/Hestenes, 含标量项归零)
        # 定义一致, 混合 grade 直接全面对比
        check(f"ip   {tag}", ours_vec(a.ip(b)), cf_to_ours(ac | bc))
        check(f"op   {tag}", ours_vec(a.op(b)), cf_to_ours(ac ^ bc))
        check(f"rev  {tag}", ours_vec(a.reverse()), cf_to_ours(~ac))
        check(f"dual {tag}", ours_vec(a.dual()), cf_to_ours(ac * (-I_cf)))
        check(
            f"meet {tag}",
            ours_vec(a.meet(b)),
            cf_to_ours(((ac * (-I_cf)) ^ (bc * (-I_cf))) * (-I_cf)),
        )
        n_ours = a.norm()
        n_cf = math.sqrt(abs(float((ac * ~ac).value[0])))
        check(f"norm {tag}", np.array([n_ours]), np.array([n_cf]))

    print("== 图元构造 (clifford 侧按约定手工构造) ==")
    for x, y, z in ((1.0, 2.0, 3.0), (-0.5, 0.25, 4.0)):
        p = Point(x, y, z)
        pc = EO + x * e1 + y * e2 + z * e3 + 0.5 * (x * x + y * y + z * z) * EI
        check(f"Point({x},{y},{z})", ours_vec(p), cf_to_ours(pc))

    p1, p2 = Point(0, 0, 0), Point(1, 2, 3)
    check(
        "PointPair",
        ours_vec(Multivector(p1.op(p2).values)),
        cf_to_ours(ours_to_cf(p1) ^ ours_to_cf(p2)),
    )
    check(
        "Line", ours_vec(Line(p1, p2)), cf_to_ours(ours_to_cf(p1) ^ ours_to_cf(p2) ^ EI)
    )

    pl = Plane((1.0, 2.0, 2.0), 3.0)  # 法向会被归一
    n = np.array([1.0, 2.0, 2.0]) / 3.0
    plc = n[0] * e1 + n[1] * e2 + n[2] * e3 + 3.0 * EI
    check("Plane", ours_vec(pl), cf_to_ours(plc))

    sp = Sphere((1.0, 2.0, 3.0), 2.0)
    cc = ours_to_cf(Point(1.0, 2.0, 3.0))
    check("Sphere", ours_vec(sp), cf_to_ours(cc - 0.5 * 4.0 * EI))

    ci = Circle((1.0, 2.0, 3.0), 2.0, (0.0, 0.0, 1.0))
    d = 3.0  # n·center
    cic = (cc - 2.0 * EI) ^ (e3 + d * EI)
    check("Circle", ours_vec(ci), cf_to_ours(cic))

    print("== versor: rotor / translator / motor 作用 ==")
    for axis, ang in (((0.0, 0.0, 1.0), math.pi / 2), ((1.0, 2.0, 3.0), 0.7)):
        ax = np.array(axis, dtype=np.float64)
        ax = ax / np.linalg.norm(ax)
        Bc = ax[0] * (e2 ^ e3) + ax[1] * (e3 ^ e1) + ax[2] * (e1 ^ e2)
        Rc = math.cos(ang / 2) - math.sin(ang / 2) * Bc
        check(f"rotor{axis}", ours_vec(Motor.rotor(axis, ang)), cf_to_ours(Rc))

    t = (0.3, -0.2, 0.1)
    Tc = 1.0 - 0.5 * ((t[0] * e1 + t[1] * e2 + t[2] * e3) ^ EI)
    check("translator", ours_vec(Motor.translator(t)), cf_to_ours(Tc))

    M = Motor((1.0, 2.0, 3.0), 0.7, t)
    Rc2 = ours_to_cf(Motor.rotor((1.0, 2.0, 3.0), 0.7))
    Mc = Tc * Rc2
    check("motor compose", ours_vec(M), cf_to_ours(Mc))

    for name, obj in (
        ("Point", Point(1.0, -1.0, 2.0)),
        ("Line", Line(Point(0, 0, 0), Point(1, 1, 1))),
        ("Sphere", Sphere((0.5, 0.5, 0.5), 1.5)),
    ):
        oc = ours_to_cf(obj)
        check(f"apply {name}", ours_vec(M.apply(obj)), cf_to_ours(Mc * oc * ~Mc))

    print("== exp/log (clifford 侧用级数独立展开) ==")

    def cf_series_exp(B, n: int = 16):
        """exp(−B) 的泰勒级数 (clifford 几何积, float64)。"""
        acc, term = ONE + 0.0 * e1, ONE + 0.0 * e1
        NB = -B
        for k in range(1, n):
            term = term * NB * (1.0 / k)
            acc = acc + term
        return acc

    for axis, ang, tr in (
        ((0.0, 0.0, 1.0), 0.6, (0.0, 0.0, 0.0)),  # 纯旋转
        ((0.0, 0.0, 0.0), 0.0, (0.3, -0.2, 0.1)),  # 纯平移
        ((1.0, 2.0, 3.0), 0.7, (0.3, -0.2, 0.1)),  # 一般螺旋
        ((0.0, 1.0, 0.0), math.pi - 0.01, (0.1, 0.0, 0.2)),  # θ≈π 分支
    ):
        Mo = Motor(axis if ang else None, ang, tr)
        B = Mo.log()  # 本包对数 (含 θ≈π 恢复分支)
        # 本包 exp(B) 对 clifford 直接映射的 M
        check(
            f"exp∘log {axis},{ang:.2f}",
            ours_vec(Motor.exp(B)),
            cf_to_ours(ours_to_cf(Mo)),
        )
        # clifford 级数 exp(−B) 独立验证 B 的正确性
        check(
            f"series exp(−B) {axis},{ang:.2f}",
            cf_to_ours(cf_series_exp(ours_to_cf(B))),
            cf_to_ours(ours_to_cf(Mo)),
            tol=2e-3,
        )

    print("== to_matrix vs clifford sandwich 提矩阵 ==")
    Hm = np.array(M.to_matrix())
    Hc = np.zeros((4, 4))
    for col, pt in enumerate(
        [
            np.zeros(3),
            np.array([1.0, 0, 0]),
            np.array([0, 1.0, 0]),
            np.array([0, 0, 1.0]),
        ]
    ):
        pc = EO + pt[0] * e1 + pt[1] * e2 + pt[2] * e3 + 0.5 * float(pt @ pt) * EI
        out = Mc * pc * ~Mc
        ov = cf_to_ours(out)
        w = ov[4]
        Hc[:3, col] = ov[1:4] / w
    Hc[3, 3] = 1.0
    Hc[:3, 1:] -= Hc[:3, 0:1]  # 旋转列 = 变换单位点 − 变换原点
    check("to_matrix R", Hm[:3, :3].reshape(-1), Hc[:3, 1:].reshape(-1))
    check("to_matrix t", Hm[:3, 3], Hc[:3, 0])

    print("== 距离 (float64 欧氏公式 vs 解析值; 远原点回归) ==")
    d_ours = Point(1000, 0, 0).dist(Point(1001, 0, 0))
    check("dist far-from-origin", np.array([d_ours]), np.array([1.0]), tol=1e-6)
    d_p = Point(0, 0, 5).dist(Plane((0, 0, 1), 2.0))
    check("dist point-plane", np.array([d_p]), np.array([3.0]), tol=1e-6)
    d_s = Point(5, 2, 3).dist(Sphere((1, 2, 3), 2.0))
    check("dist point-sphere", np.array([d_s]), np.array([2.0]), tol=1e-6)
    # 对照: clifford float64 conformal 内积在远原点同样损失精度,
    # 这正是本包距离走欧氏公式的原因 (展示, 不断言)
    p_far = EO + 1000 * e1 + 0.5 * 1e6 * EI
    p_far2 = EO + 1001 * e1 + 0.5 * (1001.0**2) * EI
    d_cf = math.sqrt(max(0.0, float(-2 * (p_far | p_far2).value[0])))
    print(
        f"  info clifford conformal 内积远原点距离 = {d_cf:.6f} "
        f"(真值 1.0, 本包 = {d_ours:.6f})"
    )

    print(f"\nall {_ok} comparisons passed")


if __name__ == "__main__":
    main()
