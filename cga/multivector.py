"""5D 共形几何代数 (CGA) 的 multivector 表示。

5D CGA 的 multivector 有 32 个分量, 按 grade 组织:
  Grade 0: 1 个标量
  Grade 1: 5 个向量   {e1, e2, e3, e0, e∞}
  Grade 2: 10 个二重向量
  Grade 3: 10 个三重向量
  Grade 4: 5 个四重向量
  Grade 5: 1 个伪标量

基取 null 基 {e1, e2, e3, e0, e∞}: e0² = e∞² = 0,
e0·e∞ = -1。e0 与 e∞ 非正交, 故 blade 的几何积不能用正交基的递归
公式——积表构建时对 blade_b 的全排列做反对称化 (见 _compute_gp)。
代价是建表稍慢 (一次性), 收益是 conformal 权重 (e0 系数) 显式存储,
远原点坐标提取无基换算抵消。所有分量存于 MLX 数组。
"""

import math

import mlx.core as mx

# ── 基 blade 定义 ──────────────────────────────────────────────────

# 32 个基 blade 的规范排序 (公开: 互操作/工具脚本
# 的合法元数据, 如 compare_clifford.py 的基变换)
# 每个 blade 是基向量下标元组: 0=e1, 1=e2, 2=e3, 3=e0, 4=e∞
BASIS_BLADES: list[tuple[int, ...]] = [
    # Grade 0 (1)
    (),
    # Grade 1 (5)
    (0,),
    (1,),
    (2,),
    (3,),
    (4,),
    # Grade 2 (10)
    (0, 1),
    (0, 2),
    (0, 3),
    (0, 4),
    (1, 2),
    (1, 3),
    (1, 4),
    (2, 3),
    (2, 4),
    (3, 4),
    # Grade 3 (10)
    (0, 1, 2),
    (0, 1, 3),
    (0, 1, 4),
    (0, 2, 3),
    (0, 2, 4),
    (0, 3, 4),
    (1, 2, 3),
    (1, 2, 4),
    (1, 3, 4),
    (2, 3, 4),
    # Grade 4 (5)
    (0, 1, 2, 3),
    (0, 1, 2, 4),
    (0, 1, 3, 4),
    (0, 2, 3, 4),
    (1, 2, 3, 4),
    # Grade 5 (1)
    (0, 1, 2, 3, 4),
]

NUM_COMPONENTS = 32
NUM_GRADES = 6

# blade 元组 → 下标的快速查表
_BLADE_TO_IDX = {blade: i for i, blade in enumerate(BASIS_BLADES)}

# 每个 blade 下标的 grade
_BLADE_GRADE = [len(blade) for blade in BASIS_BLADES]

# 按 grade 分组的下标
GRADE_INDICES: list[list[int]] = [[] for _ in range(NUM_GRADES)]
for i, g in enumerate(_BLADE_GRADE):
    GRADE_INDICES[g].append(i)

# 各 grade 的尺寸: [1, 5, 10, 10, 5, 1]
GRADE_SIZES = [len(g) for g in GRADE_INDICES]

# 各 grade 在扁平数组里的切片区间
_GRADE_SLICES: list[tuple[int, int]] = []
offset = 0
for size in GRADE_SIZES:
    _GRADE_SLICES.append((offset, offset + size))
    offset += size

# ── 基向量的度规 ───────────────────────────────────────────────────
# 下标: 0=e1, 1=e2, 2=e3, 3=e0, 4=e∞
# e1²=e2²=e3²=1, e0²=e∞²=0, e0·e∞ = e∞·e0 = −1
_VECTOR_METRIC = mx.array(
    [
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 0, -1],
        [0, 0, 0, -1, 0],
    ],
    dtype=mx.float32,
)


def _parity(seq: list[int]) -> int:
    """把 seq 排序所需的排列奇偶性, 返回 (-1)^交换次数。"""
    arr = list(seq)
    swaps = 0
    n = len(arr)
    for i in range(n):
        for j in range(n - 1 - i):
            if arr[j] > arr[j + 1]:
                arr[j], arr[j + 1] = arr[j + 1], arr[j]
                swaps += 1
    return -1 if swaps % 2 else 1


def _compute_gp(
    blade_a: tuple[int, ...], blade_b: tuple[int, ...]
) -> list[tuple[int, int]]:
    """计算两个基 blade 的几何积。

    blade_b 是外积 b_1 ∧ ... ∧ b_q。由于 e0 与 e∞ 非正交
    (e0·e∞ = -1), 外积不等于其向量序列的顺序几何积, 而是反对称化
    的几何积:

        b_1 ∧ ... ∧ b_q = (1/q!) Σ_σ sign(σ) b_σ(1) * ... * b_σ(q)

    每个排列后的向量序列用递归公式 A * v = A ⌋ v + A ∧ v 逐一乘过
    blade_a, 其中收缩为

        A ⌋ v = Σ_i (-1)^{|A|-i} g(a_i, v) * (a_1 ∧ ... ∧ â_i ∧ ... ∧ a_k)

    系数必须为整数 (blade 是带符号的排列积); 出现分数意味着建表
    本身出错, 故直接抛异常而不是静默取整。

    返回 (符号, 结果 blade 下标) 的列表。
    """
    import itertools
    import math

    # 以 {blade 元组: 累计系数} 累加结果
    results: dict[tuple[int, ...], float] = {}
    q = len(blade_b)
    for perm in itertools.permutations(blade_b):
        perm_sign = _parity(list(perm))  # blade_b 已排序: 即 σ 的奇偶性
        # 把 perm 的向量逐个乘过 blade_a
        partial = {blade_a: 1.0}
        for bv in perm:
            new_partial: dict[tuple[int, ...], float] = {}
            for cur_blade, cur_sign in partial.items():
                cur_list = list(cur_blade)
                k = len(cur_list)

                # 第 1 项: 收缩 —— 对 cur_blade 里的每个向量
                for i in range(k):
                    metric_val = float(_VECTOR_METRIC[cur_list[i], bv])
                    if metric_val == 0:
                        continue
                    contracted = tuple(cur_list[:i] + cur_list[i + 1 :])
                    term_sign = cur_sign * metric_val * ((-1) ** (k - i - 1))
                    new_partial[contracted] = (
                        new_partial.get(contracted, 0.0) + term_sign
                    )

                # 第 2 项: 外积 —— 追加 bv; 符号由后面的规范化
                # 奇偶性处理
                wedge_blade = tuple(cur_list + [bv])
                new_partial[wedge_blade] = new_partial.get(wedge_blade, 0.0) + cur_sign

            partial = new_partial

        scale = perm_sign / math.factorial(q)
        for blade, coef in partial.items():
            results[blade] = results.get(blade, 0.0) + scale * coef

    # 规范化结果: blade 排序, 先累计分数系数, 全部合并后再取整
    coef_by_idx: dict[int, float] = {}
    for blade, sign in results.items():
        if abs(sign) < 1e-12:
            continue
        blade_list = list(blade)
        parity = _parity(blade_list)
        blade_list.sort()
        canon_blade = tuple(blade_list)
        idx = _BLADE_TO_IDX.get(canon_blade)
        if idx is None:
            continue
        coef_by_idx[idx] = coef_by_idx.get(idx, 0.0) + sign * parity

    final = []
    for idx, coef in coef_by_idx.items():
        rounded = round(coef)
        if abs(coef - rounded) < 1e-9 and rounded != 0:
            final.append((rounded, idx))
        elif abs(coef) >= 1e-9:
            raise ArithmeticError(
                f"non-integer blade coefficient {coef} for blade index {idx}"
            )
    return final


def _build_gp_table() -> list[list[list[tuple[int, int]]]]:
    """构建几何积乘法表。"""
    gp_table = [[[] for _ in range(NUM_COMPONENTS)] for _ in range(NUM_COMPONENTS)]

    for i in range(NUM_COMPONENTS):
        for j in range(NUM_COMPONENTS):
            gp_table[i][j] = _compute_gp(BASIS_BLADES[i], BASIS_BLADES[j])

    return gp_table


# 模块加载时建表一次
_GP_TABLE = _build_gp_table()

# ── 供 MLX 用的预计算稀疏 GP 数组 ──────────────────────────────────

# 任一乘积的最大项数
_max_terms = max(len(terms) for row in _GP_TABLE for terms in row)

# 用 Python 列表构建补零的稠密下标/符号数组 (一次性开销)
_signs_list = [
    [[0.0] * _max_terms for _ in range(NUM_COMPONENTS)] for _ in range(NUM_COMPONENTS)
]
_indices_list = [
    [[0] * _max_terms for _ in range(NUM_COMPONENTS)] for _ in range(NUM_COMPONENTS)
]
_counts_list = [[0] * NUM_COMPONENTS for _ in range(NUM_COMPONENTS)]

for i in range(NUM_COMPONENTS):
    for j in range(NUM_COMPONENTS):
        terms = _GP_TABLE[i][j]
        _counts_list[i][j] = len(terms)
        for k, (sign, dst) in enumerate(terms):
            _signs_list[i][j][k] = float(sign)
            _indices_list[i][j][k] = dst

GP_SIGNS = mx.array(_signs_list, dtype=mx.float32)
GP_INDICES = mx.array(_indices_list, dtype=mx.int32)
GP_COUNTS = mx.array(_counts_list, dtype=mx.int32)

# ── grade 投影掩码 ─────────────────────────────────────────────────

_GRADE_MASKS = []
for g in range(NUM_GRADES):
    vals = [1.0 if i in GRADE_INDICES[g] else 0.0 for i in range(NUM_COMPONENTS)]
    _GRADE_MASKS.append(mx.array(vals, dtype=mx.float32))

GRADE_MASKS = _GRADE_MASKS


def _grade_signs(sign_of_grade) -> mx.array:
    """由"逐 grade 符号函数"构建逐分量 ±1 掩码。"""
    vals = [1.0] * NUM_COMPONENTS
    for g in range(NUM_GRADES):
        for idx in GRADE_INDICES[g]:
            vals[idx] = float(sign_of_grade(g))
    return mx.array(vals, dtype=mx.float32)


# 两种对合的符号掩码, 模块加载时预计算一次
_REVERSE_MASK = _grade_signs(lambda g: (-1) ** (g * (g - 1) // 2))
_INVOLUTION_MASK = _grade_signs(lambda g: -1 if g % 2 else 1)

# ── 供 scatter_add 式几何积用的扁平化 GP 表 ────────────────────────

# GP_MASK[i,j,k] = 符号 (若 GP_TABLE[i][j] 对 k 有贡献), 否则 0
_mask_list = [
    [[0.0] * NUM_COMPONENTS for _ in range(NUM_COMPONENTS)]
    for _ in range(NUM_COMPONENTS)
]
for i in range(NUM_COMPONENTS):
    for j in range(NUM_COMPONENTS):
        for sign, dst in _GP_TABLE[i][j]:
            _mask_list[i][j][dst] += float(sign)
GP_MASK = mx.array(_mask_list, dtype=mx.float32)

# 稀疏计算用的非零 (i,j) 对
_nz_i = []
_nz_j = []
for i in range(NUM_COMPONENTS):
    for j in range(NUM_COMPONENTS):
        if _GP_TABLE[i][j]:
            _nz_i.append(i)
            _nz_j.append(j)
GP_NONZERO_I = mx.array(_nz_i, dtype=mx.int32)
GP_NONZERO_J = mx.array(_nz_j, dtype=mx.int32)

# 模块级数组全部物化: 未求值的懒图携带创建线程的流, 后台线程
# (GroupingTracker worker) 使用它们会报 no Stream in current thread
mx.eval(
    GP_SIGNS,
    GP_INDICES,
    GP_COUNTS,
    GP_MASK,
    GP_NONZERO_I,
    GP_NONZERO_J,
    *GRADE_MASKS,
    _REVERSE_MASK,
    _INVOLUTION_MASK,
)


# ── Multivector 类 ─────────────────────────────────────────────────


class Multivector:
    """5D CGA 的 32 分量 multivector, 以 MLX 数组为后端。

    系数存储于 null 基 {e1, e2, e3, e0, e∞} 下 (槽 4 = e0, 槽 5 = e∞)。
    """

    __slots__ = ("values",)

    def __init__(self, values: mx.array | None = None):
        """由 32 分量数组构造; None 表示零 multivector。"""
        if values is None:
            self.values = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        elif isinstance(values, mx.array):
            if values.shape != (NUM_COMPONENTS,):
                raise ValueError(f"Expected shape (32,), got {values.shape}")
            self.values = values
        else:
            arr = mx.array(values, dtype=mx.float32)
            if arr.shape != (NUM_COMPONENTS,):
                raise ValueError(f"Expected shape (32,), got {arr.shape}")
            self.values = arr

    @staticmethod
    def zeros() -> Multivector:
        """零 multivector。"""
        return Multivector(mx.zeros(NUM_COMPONENTS, dtype=mx.float32))

    @staticmethod
    def scalar(s: float) -> Multivector:
        """标量 multivector (仅 grade-0 分量)。"""
        vals = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        vals[0] = s
        return Multivector(vals)

    @staticmethod
    def vector(
        v1: float, v2: float, v3: float, v0: float = 0.0, ve: float = 0.0
    ) -> Multivector:
        """由欧氏分量 (v1,v2,v3) + e0/e∞ 系数 (v0/ve) 构造向量。"""
        vals = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        vals[1] = v1
        vals[2] = v2
        vals[3] = v3
        vals[4] = v0
        vals[5] = ve
        return Multivector(vals)

    @staticmethod
    def bivector(components: list[float]) -> Multivector:
        """由 10 个 grade-2 分量构造二重向量。"""
        if len(components) != 10:
            raise ValueError(f"Expected 10 bivector components, got {len(components)}")
        vals = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        for i, v in enumerate(components):
            idx = GRADE_INDICES[2][i]
            vals[idx] = v
        return Multivector(vals)

    def grade(self, g: int) -> Multivector:
        """提取 grade-g 投影。"""
        mask = GRADE_MASKS[g]
        return Multivector(self.values * mask)

    def scalar_part(self) -> float:
        """标量部 (grade-0 分量)。"""
        return float(self.values[0])

    def vector_part(self) -> mx.array:
        """向量部 (grade-1 的 5 个分量)。"""
        start, end = _GRADE_SLICES[1]
        return self.values[start:end]

    def euclidean_vector(self) -> tuple[float, float, float]:
        """欧氏向量部 (e1/e2/e3 三个系数)。"""
        return (float(self.values[1]), float(self.values[2]), float(self.values[3]))

    def e0_coeff(self) -> float:
        """e0 (原点) 系数 —— conformal 权重, 显式存储于槽 4。"""
        return float(self.values[4])

    def einf_coeff(self) -> float:
        """e∞ (无穷远点) 系数, 显式存储于槽 5。"""
        return float(self.values[5])

    def bivector_part(self) -> mx.array:
        """二重向量部 (grade-2 的 10 个分量)。"""
        start, end = _GRADE_SLICES[2]
        return self.values[start:end]

    @property
    def is_zero(self) -> bool:
        """是否为零 multivector (所有分量≈0)。

        这不是 CGA 的 null 性 (v·v = 0)——conformal point 等非零向量
        也是 null; 判 null 请用 gp(v, v) 的标量部。
        """
        return bool(mx.all(mx.abs(self.values) < 1e-10).item())

    def __add__(self, other: Multivector) -> Multivector:
        """逐分量加法。"""
        return Multivector(self.values + other.values)

    def __sub__(self, other: Multivector) -> Multivector:
        """逐分量减法。"""
        return Multivector(self.values - other.values)

    def __mul__(self, scalar: float) -> Multivector:
        """标量乘法 (几何积请用 gp())。"""
        return Multivector(self.values * scalar)

    def __rmul__(self, scalar: float) -> Multivector:
        """右标量乘法。"""
        return Multivector(self.values * scalar)

    def __truediv__(self, scalar: float) -> Multivector:
        """标量除法。"""
        return Multivector(self.values / scalar)

    def __neg__(self) -> Multivector:
        """逐分量取负。"""
        return Multivector(-self.values)

    def __repr__(self) -> str:
        """按 grade 列出非零分量的可读表示。"""
        parts = []
        for g in range(NUM_GRADES):
            for idx in GRADE_INDICES[g]:
                v = float(self.values[idx])
                if abs(v) > 1e-10:
                    blade_name = _blade_name(idx)
                    if blade_name == "1":
                        parts.append(f"{v:.4f}")
                    else:
                        parts.append(f"{v:+.4f}*{blade_name}")
        if not parts:
            return "Multivector(0)"
        return "Multivector(" + " ".join(parts) + ")"

    def __eq__(self, other: object) -> bool:
        """近似相等 (allclose, atol=1e-6)。"""
        if not isinstance(other, Multivector):
            return False
        return bool(mx.allclose(self.values, other.values, atol=1e-6).item())

    def __hash__(self) -> int:
        """精确表示的哈希。注意 __eq__ 是近似比较 (atol=1e-6), 故
        "近似相等但非逐位相同"的 multivector 会有不同哈希——作
        dict key/set 成员时请自行量化或取整后再用。"""
        return hash(tuple(self.values.tolist()))

    def copy(self) -> Multivector:
        """拷贝 (新 MLX 数组)。"""
        return Multivector(mx.array(self.values))

    # ── 代数运算 (实现即验证过的原 cga.algebra 函数, 逐字搬入) ──────

    def gp(self, other: Multivector) -> Multivector:
        """几何积。result[k] = Σ GP_MASK[i,j,k]·self_i·other_j,
        用预计算的稀疏非零 (i,j) 对索引。"""
        prod = self.values[GP_NONZERO_I] * other.values[GP_NONZERO_J]
        mask_rows = GP_MASK[GP_NONZERO_I, GP_NONZERO_J, :]  # (N, 32)
        return Multivector((mask_rows * prod[:, None]).sum(axis=0))

    def ip(self, other: Multivector) -> Multivector:
        """内积 (fat dot / Hestenes, 与 clifford 库的 | 算子一致)。

        blade 规则对全 grade 对的线性扩张:
            A|B = Σ_{r,s≥1} ⟨ ⟨A⟩_r * ⟨B⟩_s ⟩_|r−s|
        含标量 (grade 0) 的项为零 (Hestenes 规则, 与 clifford 实测
        一致: 1|e12 = 0)。r>s 时非零 (对称内积, 与左收缩的区别);
        向量与 blade 间 (r=1≤s) 与左收缩相同, 故关联判据
        p.ip(X) = 0 的行为不受影响。对一般混合 grade multivector
        正确。"""
        result = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        for ga in range(1, NUM_GRADES):
            a_g = self.values * GRADE_MASKS[ga]
            if not bool(mx.any(a_g != 0).item()):
                continue
            for gb in range(1, NUM_GRADES):
                b_g = other.values * GRADE_MASKS[gb]
                if not bool(mx.any(b_g != 0).item()):
                    continue
                prod = Multivector(a_g).gp(Multivector(b_g))
                result = result + prod.values * GRADE_MASKS[abs(gb - ga)]
        return Multivector(result)

    def op(self, other: Multivector) -> Multivector:
        """外积 self ∧ other。

        blade 规则对全 grade 对的线性扩张:
            a ∧ b = Σ_{r,s} < <a>_r * <b>_s >_{r+s}
        对一般混合 grade multivector 正确。"""
        result = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        for ga in range(NUM_GRADES):
            a_g = self.values * GRADE_MASKS[ga]
            if not bool(mx.any(a_g != 0).item()):
                continue
            for gb in range(NUM_GRADES - ga):
                b_g = other.values * GRADE_MASKS[gb]
                if not bool(mx.any(b_g != 0).item()):
                    continue
                prod = Multivector(a_g).gp(Multivector(b_g))
                result = result + prod.values * GRADE_MASKS[ga + gb]
        return Multivector(result)

    def reverse(self) -> Multivector:
        """反转 involution: grade-k blade 乘 (-1)^{k(k-1)/2}。"""
        return Multivector(self.values * _REVERSE_MASK)

    def grade_involution(self) -> Multivector:
        """Grade involution: 奇 grade 分量取负。"""
        return Multivector(self.values * _INVOLUTION_MASK)

    def conjugate(self) -> Multivector:
        """Clifford 共轭: reverse + grade involution。"""
        return self.reverse().grade_involution()

    def dual(self) -> Multivector:
        """Hodge 对偶: 乘逆伪标量 I⁻¹。

        定向约定: I = e123 ∧ e∞ ∧ e0, 与 `clifford` 库的 conformal
        伪标量 e12345 一致 (e∞ ∧ e0 = +e45)。此定向下 I² = −1,
        故 I⁻¹ = −I, dual(A) = A · I⁻¹。
        """
        # I = e123∧e∞∧e0 = −(规范 blade 31);  I⁻¹ = −I = +blade31
        I_inv_vals = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        I_inv_vals[31] = 1.0
        return self.gp(Multivector(I_inv_vals))

    def undual(self) -> Multivector:
        """对偶的逆: dual(dual(x)) = −x (因 I⁻² = I² = −1), 故 undual = −dual。

        从直接形式还原对偶形式 (n + d·e∞ / up(c) − ½ρ²e∞) 时使用。
        """
        return -self.dual()

    def meet(self, other: Multivector) -> Multivector:
        """两个直接形式原语的交: self ∨ other = (self* ∧ other*)*。

        输入需为直接形式; 对偶形式的原语 (plane/sphere/circle) 先过
        dual() 再传入。例: π1.dual().meet(π2.dual()) = 交线 (直接形式)。
        """
        return self.dual().op(other.dual()).dual()

    def norm(self) -> float:
        """欧氏范数: sqrt(|⟨self · reverse(self)⟩₀|)。"""
        s = float(self.gp(self.reverse()).values[0])
        return math.sqrt(abs(s))

    def normalized(self) -> Multivector:
        """归一化到单位范数。"""
        n = self.norm()
        if n < 1e-12:
            return Multivector.zeros()
        return self / n

    def bulk(self) -> Multivector:
        """欧氏 (bulk) 部分: 不含 e0/e∞ 的分量。"""
        euc_indices = [0, 1, 2, 3, 6, 7, 10, 16]
        vals = mx.zeros(NUM_COMPONENTS, dtype=mx.float32)
        for idx in euc_indices:
            vals[idx] = self.values[idx]
        return Multivector(vals)

    def weight(self) -> Multivector:
        """Conformal (weight) 部分: 含 e0/e∞ 的分量。"""
        return self - self.bulk()


def _blade_name(idx: int) -> str:
    """blade 下标 → 可读名 (如 (0,1) → "e12", 标量 → "1")。"""
    blade = BASIS_BLADES[idx]
    if not blade:
        return "1"
    names = {0: "e1", 1: "e2", 2: "e3", 3: "e0", 4: "e∞"}
    return "".join(names[v] for v in blade)
