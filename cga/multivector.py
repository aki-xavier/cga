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
公式——积表构建时对 blade_b 的全排列做反对称化 (见 compute_gp)。
代价是建表稍慢 (一次性), 收益是 conformal 权重 (e0 系数) 显式存储,
远原点坐标提取无基换算抵消。所有分量存于 MLX 数组。

全部代数常量 (基表/积表/掩码/基向量) 是 Multivector 的类属性,
在类定义后由文件末尾的初始化块填充 (构建函数是静态方法)。
"""

import math
from typing import ClassVar

import mlx.core as mx


class Multivector:
    """5D CGA 的 32 分量 multivector, 以 MLX 数组为后端。

    系数存储于 null 基 {e1, e2, e3, e0, e∞} 下 (槽 4 = e0, 槽 5 = e∞)。

    类属性 (文件末尾初始化块填充):
      BASIS_BLADES: 32 个基 blade 的规范排序 (公开: 互操作/工具脚本的
        合法元数据; 排序经 clifford 库数值验证, 见 git 历史的
        compare_clifford.py); 每个 blade 是基向量下标元组
        (0=e1, 1=e2, 2=e3, 3=e0, 4=e∞)
      GP_*: 几何积的稀疏/稠密预计算表; GRADE_*: grade 分组与掩码
      E1/E2/E3/E0/EINF: 基向量 multivector
    """

    __slots__ = ("values",)

    # ── 类属性声明 (值在文件末尾初始化块填充) ─────────────────────
    BASIS_BLADES: ClassVar[list[tuple[int, ...]]]
    NUM_COMPONENTS: ClassVar[int]
    NUM_GRADES: ClassVar[int]
    BLADE_TO_IDX: ClassVar[dict[tuple[int, ...], int]]
    BLADE_GRADE: ClassVar[list[int]]
    GRADE_INDICES: ClassVar[list[list[int]]]
    GRADE_SIZES: ClassVar[list[int]]
    GRADE_SLICES: ClassVar[list[tuple[int, int]]]
    VECTOR_METRIC: ClassVar[mx.array]
    GP_TABLE: ClassVar[list[list[list[tuple[int, int]]]]]
    GP_SIGNS: ClassVar[mx.array]
    GP_INDICES: ClassVar[mx.array]
    GP_COUNTS: ClassVar[mx.array]
    GRADE_MASKS: ClassVar[list[mx.array]]
    REVERSE_MASK: ClassVar[mx.array]
    INVOLUTION_MASK: ClassVar[mx.array]
    GP_MASK: ClassVar[mx.array]
    GP_NONZERO_I: ClassVar[mx.array]
    GP_NONZERO_J: ClassVar[mx.array]
    E1: ClassVar[Multivector]
    E2: ClassVar[Multivector]
    E3: ClassVar[Multivector]
    E0: ClassVar[Multivector]
    EINF: ClassVar[Multivector]

    # ── 积表构建 (静态方法, 模块加载时由初始化块调用一次) ──────────

    @staticmethod
    def parity(seq: list[int]) -> int:
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

    @staticmethod
    def compute_gp(
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

        metric = Multivector.VECTOR_METRIC
        blade_to_idx = Multivector.BLADE_TO_IDX
        # 以 {blade 元组: 累计系数} 累加结果
        results: dict[tuple[int, ...], float] = {}
        q = len(blade_b)
        for perm in itertools.permutations(blade_b):
            perm_sign = Multivector.parity(list(perm))  # blade_b 已排序: σ 的奇偶性
            # 把 perm 的向量逐个乘过 blade_a
            partial = {blade_a: 1.0}
            for bv in perm:
                new_partial: dict[tuple[int, ...], float] = {}
                for cur_blade, cur_sign in partial.items():
                    cur_list = list(cur_blade)
                    k = len(cur_list)

                    # 第 1 项: 收缩 —— 对 cur_blade 里的每个向量
                    for i in range(k):
                        metric_val = float(metric[cur_list[i], bv])
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
                    new_partial[wedge_blade] = (
                        new_partial.get(wedge_blade, 0.0) + cur_sign
                    )

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
            parity = Multivector.parity(blade_list)
            blade_list.sort()
            canon_blade = tuple(blade_list)
            idx = blade_to_idx.get(canon_blade)
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

    @staticmethod
    def build_grade_indices() -> list[list[int]]:
        """按 grade 分组的 blade 下标。"""
        groups: list[list[int]] = [[] for _ in range(Multivector.NUM_GRADES)]
        for i, g in enumerate(Multivector.BLADE_GRADE):
            groups[g].append(i)
        return groups

    @staticmethod
    def build_grade_slices() -> list[tuple[int, int]]:
        """各 grade 在扁平数组里的切片区间。"""
        slices = []
        offset = 0
        for size in Multivector.GRADE_SIZES:
            slices.append((offset, offset + size))
            offset += size
        return slices

    @staticmethod
    def build_gp_table() -> list[list[list[tuple[int, int]]]]:
        """构建几何积乘法表。"""
        n = Multivector.NUM_COMPONENTS
        blades = Multivector.BASIS_BLADES
        gp_table = [[[] for _ in range(n)] for _ in range(n)]
        for i in range(n):
            for j in range(n):
                gp_table[i][j] = Multivector.compute_gp(blades[i], blades[j])
        return gp_table

    @staticmethod
    def build_gp_dense() -> tuple[mx.array, mx.array, mx.array]:
        """GP 表 → 补零稠密数组 (GP_SIGNS/GP_INDICES/GP_COUNTS)。"""
        n = Multivector.NUM_COMPONENTS
        table = Multivector.GP_TABLE
        max_terms = max(len(terms) for row in table for terms in row)
        signs_list = [[[0.0] * max_terms for _ in range(n)] for _ in range(n)]
        indices_list = [[[0] * max_terms for _ in range(n)] for _ in range(n)]
        counts_list = [[0] * n for _ in range(n)]
        for i in range(n):
            for j in range(n):
                terms = table[i][j]
                counts_list[i][j] = len(terms)
                for k, (sign, dst) in enumerate(terms):
                    signs_list[i][j][k] = float(sign)
                    indices_list[i][j][k] = dst
        return (
            mx.array(signs_list, dtype=mx.float32),
            mx.array(indices_list, dtype=mx.int32),
            mx.array(counts_list, dtype=mx.int32),
        )

    @staticmethod
    def build_grade_masks() -> list[mx.array]:
        """逐 grade 投影掩码 (GRADE_MASKS[g][i] = 1 若分量 i 属 grade g)。"""
        masks = []
        for g in range(Multivector.NUM_GRADES):
            vals = [
                1.0 if i in Multivector.GRADE_INDICES[g] else 0.0
                for i in range(Multivector.NUM_COMPONENTS)
            ]
            masks.append(mx.array(vals, dtype=mx.float32))
        return masks

    @staticmethod
    def grade_signs(sign_of_grade) -> mx.array:
        """由"逐 grade 符号函数"构建逐分量 ±1 掩码。"""
        vals = [1.0] * Multivector.NUM_COMPONENTS
        for g in range(Multivector.NUM_GRADES):
            for idx in Multivector.GRADE_INDICES[g]:
                vals[idx] = float(sign_of_grade(g))
        return mx.array(vals, dtype=mx.float32)

    @staticmethod
    def build_gp_sparse() -> tuple[mx.array, mx.array, mx.array]:
        """GP 表 → 扁平掩码 GP_MASK[i,j,k] 与非零 (i,j) 对。"""
        n = Multivector.NUM_COMPONENTS
        table = Multivector.GP_TABLE
        mask_list = [[[0.0] * n for _ in range(n)] for _ in range(n)]
        nz_i = []
        nz_j = []
        for i in range(n):
            for j in range(n):
                if table[i][j]:
                    nz_i.append(i)
                    nz_j.append(j)
                for sign, dst in table[i][j]:
                    mask_list[i][j][dst] += float(sign)
        return (
            mx.array(mask_list, dtype=mx.float32),
            mx.array(nz_i, dtype=mx.int32),
            mx.array(nz_j, dtype=mx.int32),
        )

    @staticmethod
    def blade_name(idx: int) -> str:
        """blade 下标 → 可读名 (如 (0,1) → "e12", 标量 → "1")。"""
        blade = Multivector.BASIS_BLADES[idx]
        if not blade:
            return "1"
        names = {0: "e1", 1: "e2", 2: "e3", 3: "e0", 4: "e∞"}
        return "".join(names[v] for v in blade)

    # ── 构造 ──────────────────────────────────────────────────────

    def __init__(self, values: mx.array | None = None):
        """由 32 分量数组构造; None 表示零 multivector。"""
        n = Multivector.NUM_COMPONENTS
        if values is None:
            self.values = mx.zeros(n, dtype=mx.float32)
        elif isinstance(values, mx.array):
            if values.shape != (n,):
                raise ValueError(f"Expected shape (32,), got {values.shape}")
            self.values = values
        else:
            arr = mx.array(values, dtype=mx.float32)
            if arr.shape != (n,):
                raise ValueError(f"Expected shape (32,), got {arr.shape}")
            self.values = arr

    @staticmethod
    def zeros() -> Multivector:
        """零 multivector。"""
        return Multivector(mx.zeros(Multivector.NUM_COMPONENTS, dtype=mx.float32))

    @staticmethod
    def scalar(s: float) -> Multivector:
        """标量 multivector (仅 grade-0 分量)。"""
        vals = mx.zeros(Multivector.NUM_COMPONENTS, dtype=mx.float32)
        vals[0] = s
        return Multivector(vals)

    @staticmethod
    def vector(
        v1: float, v2: float, v3: float, v0: float = 0.0, ve: float = 0.0
    ) -> Multivector:
        """由欧氏分量 (v1,v2,v3) + e0/e∞ 系数 (v0/ve) 构造向量。"""
        vals = mx.zeros(Multivector.NUM_COMPONENTS, dtype=mx.float32)
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
        vals = mx.zeros(Multivector.NUM_COMPONENTS, dtype=mx.float32)
        for i, v in enumerate(components):
            idx = Multivector.GRADE_INDICES[2][i]
            vals[idx] = v
        return Multivector(vals)

    # ── 分量访问 ──────────────────────────────────────────────────

    def grade(self, g: int) -> Multivector:
        """提取 grade-g 投影。"""
        mask = Multivector.GRADE_MASKS[g]
        return Multivector(self.values * mask)

    def scalar_part(self) -> float:
        """标量部 (grade-0 分量)。"""
        return float(self.values[0])

    def vector_part(self) -> mx.array:
        """向量部 (grade-1 的 5 个分量)。"""
        start, end = Multivector.GRADE_SLICES[1]
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
        start, end = Multivector.GRADE_SLICES[2]
        return self.values[start:end]

    @property
    def is_zero(self) -> bool:
        """是否为零 multivector (所有分量≈0)。

        这不是 CGA 的 null 性 (v·v = 0)——conformal point 等非零向量
        也是 null; 判 null 请用 gp(v, v) 的标量部。
        """
        return bool(mx.all(mx.abs(self.values) < 1e-10).item())

    # ── 运算符 ────────────────────────────────────────────────────

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
        for g in range(Multivector.NUM_GRADES):
            for idx in Multivector.GRADE_INDICES[g]:
                v = float(self.values[idx])
                if abs(v) > 1e-10:
                    blade_name = Multivector.blade_name(idx)
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
        prod = (
            self.values[Multivector.GP_NONZERO_I]
            * other.values[Multivector.GP_NONZERO_J]
        )
        mask_rows = Multivector.GP_MASK[
            Multivector.GP_NONZERO_I, Multivector.GP_NONZERO_J, :
        ]  # (N, 32)
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
        masks = Multivector.GRADE_MASKS
        result = mx.zeros(Multivector.NUM_COMPONENTS, dtype=mx.float32)
        for ga in range(1, Multivector.NUM_GRADES):
            a_g = self.values * masks[ga]
            if not bool(mx.any(a_g != 0).item()):
                continue
            for gb in range(1, Multivector.NUM_GRADES):
                b_g = other.values * masks[gb]
                if not bool(mx.any(b_g != 0).item()):
                    continue
                prod = Multivector(a_g).gp(Multivector(b_g))
                result = result + prod.values * masks[abs(gb - ga)]
        return Multivector(result)

    def op(self, other: Multivector) -> Multivector:
        """外积 self ∧ other。

        blade 规则对全 grade 对的线性扩张:
            a ∧ b = Σ_{r,s} < <a>_r * <b>_s >_{r+s}
        对一般混合 grade multivector 正确。"""
        masks = Multivector.GRADE_MASKS
        result = mx.zeros(Multivector.NUM_COMPONENTS, dtype=mx.float32)
        for ga in range(Multivector.NUM_GRADES):
            a_g = self.values * masks[ga]
            if not bool(mx.any(a_g != 0).item()):
                continue
            for gb in range(Multivector.NUM_GRADES - ga):
                b_g = other.values * masks[gb]
                if not bool(mx.any(b_g != 0).item()):
                    continue
                prod = Multivector(a_g).gp(Multivector(b_g))
                result = result + prod.values * masks[ga + gb]
        return Multivector(result)

    def reverse(self) -> Multivector:
        """反转 involution: grade-k blade 乘 (-1)^{k(k-1)/2}。"""
        return Multivector(self.values * Multivector.REVERSE_MASK)

    def grade_involution(self) -> Multivector:
        """Grade involution: 奇 grade 分量取负。"""
        return Multivector(self.values * Multivector.INVOLUTION_MASK)

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
        i_inv_vals = mx.zeros(Multivector.NUM_COMPONENTS, dtype=mx.float32)
        i_inv_vals[31] = 1.0
        return self.gp(Multivector(i_inv_vals))

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
        vals = mx.zeros(Multivector.NUM_COMPONENTS, dtype=mx.float32)
        for idx in euc_indices:
            vals[idx] = self.values[idx]
        return Multivector(vals)

    def weight(self) -> Multivector:
        """Conformal (weight) 部分: 含 e0/e∞ 的分量。"""
        return self - self.bulk()


# ── 类属性初始化块 (模块加载时执行一次) ────────────────────────────
# 表构建走上面的静态方法; 未求值的懒图携带创建线程的流, 故末尾
# mx.eval 物化 (后台线程使用安全)。

Multivector.BASIS_BLADES = [
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
Multivector.NUM_COMPONENTS = 32
Multivector.NUM_GRADES = 6
Multivector.BLADE_TO_IDX = {
    blade: i for i, blade in enumerate(Multivector.BASIS_BLADES)
}
Multivector.BLADE_GRADE = [len(blade) for blade in Multivector.BASIS_BLADES]
Multivector.GRADE_INDICES = Multivector.build_grade_indices()
Multivector.GRADE_SIZES = [len(g) for g in Multivector.GRADE_INDICES]
Multivector.GRADE_SLICES = Multivector.build_grade_slices()

# 基向量度规: 0=e1, 1=e2, 2=e3, 3=e0, 4=e∞
# e1²=e2²=e3²=1, e0²=e∞²=0, e0·e∞ = e∞·e0 = −1
Multivector.VECTOR_METRIC = mx.array(
    [
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 0, -1],
        [0, 0, 0, -1, 0],
    ],
    dtype=mx.float32,
)

Multivector.GP_TABLE = Multivector.build_gp_table()
(
    Multivector.GP_SIGNS,
    Multivector.GP_INDICES,
    Multivector.GP_COUNTS,
) = Multivector.build_gp_dense()
Multivector.GRADE_MASKS = Multivector.build_grade_masks()
# 两种对合的符号掩码
Multivector.REVERSE_MASK = Multivector.grade_signs(lambda g: (-1) ** (g * (g - 1) // 2))
Multivector.INVOLUTION_MASK = Multivector.grade_signs(lambda g: -1 if g % 2 else 1)
(
    Multivector.GP_MASK,
    Multivector.GP_NONZERO_I,
    Multivector.GP_NONZERO_J,
) = Multivector.build_gp_sparse()

# 基向量
Multivector.E1 = Multivector.vector(1, 0, 0, 0, 0)
Multivector.E2 = Multivector.vector(0, 1, 0, 0, 0)
Multivector.E3 = Multivector.vector(0, 0, 1, 0, 0)
Multivector.E0 = Multivector.vector(0, 0, 0, 1, 0)
Multivector.EINF = Multivector.vector(0, 0, 0, 0, 1)

# 物化: 未求值的懒图携带创建线程的流, 后台线程使用它们会报
# no Stream in current thread
mx.eval(
    Multivector.GP_SIGNS,
    Multivector.GP_INDICES,
    Multivector.GP_COUNTS,
    Multivector.GP_MASK,
    Multivector.GP_NONZERO_I,
    Multivector.GP_NONZERO_J,
    *Multivector.GRADE_MASKS,
    Multivector.REVERSE_MASK,
    Multivector.INVOLUTION_MASK,
    Multivector.E1.values,
    Multivector.E2.values,
    Multivector.E3.values,
    Multivector.E0.values,
    Multivector.EINF.values,
)
