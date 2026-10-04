//! P2 —— 可证明求根：判别式精确路径 + 四次根包（DK 初值 + 区间符号/单调认证）。
//!
//! 理论、验收与分层见 `docs/freeform-robust-boolean.md` §3.3、§6（P2 阶段）：
//!
//! - **判别式代数精确路径**（§3.3 路线 1）：[`quadratic`] 判定 `b² − 4ac` 的**精确符号**
//!   —— `b²` 与 `a·c` 各经 `two_prod` 精确分解为可表示的 `(hi, lo)` 对，再按词典序比较。
//!   没有 `disc > 1e-12` 式阈值：掠射（判别式为任意小正数）必然判为两根，
//!   精确重根必然落进 [`QuadRoots::Double`]，无实根必然 [`QuadRoots::None`]；
//!   输入尺度退化（非有限 / 下溢 / 恒等式）显式 [`QuadRoots::Indeterminate`]，不猜。
//! - **四次根包**（§3.3 路线 2）：[`quartic`] 用与 `geometry_extra::dk_roots` 同种子、
//!   同 50 次迭代的 Durand–Kerner 产生初值盒，逐盒做**区间符号/单调认证**
//!   （区间算术用 `inari`，IEEE 1788.1-2017）：端点**严格符号**反向 + `0 ∉ F′(X)`
//!   单调 → IVT 给"恰有一根"；点求值分辨歧义（`|P(t)|` 落进包络宽度）处用中值引理
//!   `d = E/g` 收口。**不使用非包络的 f64 点值**——fl-Horner 的舍入偏置会让
//!   Newton 在过估计盒上收敛到伪根。
//!   **无法证明唯一性（重根特征：`0 ∈ F′(X)`）→ 显式 [`QuarticRoots::Unknown`]**
//!   ——对应 §7「不在相切处猜」。四次的重根判定因此**不**走 256 项闭式判别式：
//!   f64 下它的舍入并不比 `0 ∈ F′` 证据更可靠，路线 2 已经覆盖。
//! - **完整性（无静默漏判）**：Cauchy 界 `1 + max|ci|` 圈定根的绝对范围；符号扫描
//!   （区间 Horner + 二分）对全范围收口——区间证明 `0 ∉ P(X)` 才剪枝（含根的盒子
//!   永远剪不掉；单调同号的过估计盒确证无根同样剪），叶宽极限处仍 `0 ∈ P` 的区域
//!   做最后一次盒认证，`Unknown` 且邻近无已证根 → `Unknown`。DK 漏掉的单根会被
//!   扫描补证；扫描预算耗尽同样 → `Unknown`（保守、可判定）。与已证根**容差盒**
//!   邻接/交叠的锁死叶并入该根（低于点求值分辨极限的根族，与 [`crate::tol`]
//!   `DEGENERATE_ULPS` 同一哲学）。
//!
//! **分层**（§6 性能约束）：本模块是 **CPU f64 权威路径**。GPU f32 快速路径
//! （`csg.rs` 的 `disc > 1e-12` 等守卫）本阶段不动——近切处 f32 判别式的符号本就
//! 不可信，放宽它需要整条 CPU 回落配合（接线随 P4 导出一并接）；管线分类正确性由
//! P1 的区间分类兜底，本模块提供可证明判据与三值语义的落点。
//!
//! 系数约定：首一四次 `t⁴ + c3 t³ + c2 t² + c1 t + c0`（与 `dk_roots` Horner 内层的
//! 隐式首项一致）；射线系数构造器要求 `d` 为单位向量（与 GPU 路径一致）。

use inari::Interval;

/// 运行时构造闭区间 `[a, b]`（`const_interval!` 只收常量表达式，这里走 `TryFrom`）。
/// 非法输入（`a > b` / 非有限）返回 `Interval::EMPTY`，由调用方的空集分支显式处理。
#[inline]
fn iv(a: f64, b: f64) -> Interval {
    Interval::try_from((a, b)).unwrap_or(Interval::EMPTY)
}

// ── 判别式精确路径（§3.3 路线 1） ─────────────────────────────────────

/// 二次方程根的**精确分类**（对应 §3.3 路线 1：判别式代数精确路径）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QuadRoots {
    /// 判别式精确 > 0 —— 恰有两个互异实根（升序）。
    Two(f64, f64),
    /// 判别式精确 = 0 —— 重根（相切），`t = −b / 2a`。
    Double(f64),
    /// 判别式精确 < 0 —— 无实根（射线不相交）。
    None,
    /// `a = 0 ∧ b ≠ 0` —— 退化为线性，单根 `−c / b`。
    One(f64),
    /// 系数非有限、乘积下溢、或整条射线贴在面上（`a = b = c = 0`）——
    /// 显式不可判定（§7：猜不出来就报，不猜）。
    Indeterminate,
}

/// 精确双乘积：`a · b = hi + lo`，`hi` 为舍入积、`lo` 为其**可表示**的精确残差
/// （fma 残差定理）。规格化尺度下成立；下溢 / 溢出 / 非规格化 → `None`。
fn two_prod(a: f64, b: f64) -> Option<(f64, f64)> {
    if a == 0.0 || b == 0.0 {
        return Some((0.0, 0.0));
    }
    let hi = a * b;
    if !hi.is_normal() {
        return None;
    }
    let lo = f64::mul_add(a, b, -hi);
    if !lo.is_finite() {
        return None;
    }
    Some((hi, lo))
}

/// 判别式 `b² − 4ac` 的**精确符号**（针对这组 f64 系数所定义的实系数二次方程）。
///
/// `b²`、`a·c` 各为精确 `(hi, lo)` 分解，`4·(a·c)` 是纯指数平移（缩放失真即返回
/// `None`）；两对按词典序比较是精确的——同一实值必得同一 `(fl, 残差)` 对，
/// 不同实值必先在 `hi` 分出高下（四舍五入到最近的保序性）。
/// 返回 `None` 当且仅当尺度退化（下溢 / 溢出），调用方转为显式不可判定。
fn discriminant_sign(a: f64, b: f64, c: f64) -> Option<i8> {
    use std::cmp::Ordering;
    let (bh, bl) = two_prod(b, b)?;
    let (ph, pl) = two_prod(a, c)?;
    let qh = 4.0 * ph;
    let ql = 4.0 * pl;
    if !qh.is_finite() || !ql.is_finite() || qh * 0.25 != ph || ql * 0.25 != pl {
        return None;
    }
    match bh.partial_cmp(&qh) {
        Some(Ordering::Greater) => Some(1),
        Some(Ordering::Less) => Some(-1),
        _ => match bl.partial_cmp(&ql) {
            Some(Ordering::Greater) => Some(1),
            Some(Ordering::Less) => Some(-1),
            Some(Ordering::Equal) => Some(0),
            None => None,
        },
    }
}

/// 求解 `a t² + b t + c = 0`，判别式走精确符号分类（无阈值魔数）。
///
/// 根值用稳定式（`q = −(b + sign(b)√disc)/2`，Fritsch 配对），仅在符号已由
/// [`discriminant_sign`] 锁定后计算。
pub fn quadratic(a: f64, b: f64, c: f64) -> QuadRoots {
    if !(a.is_finite() && b.is_finite() && c.is_finite()) {
        return QuadRoots::Indeterminate;
    }
    if a == 0.0 {
        return if b == 0.0 {
            if c == 0.0 {
                QuadRoots::Indeterminate // 恒等式：整条射线贴在面上
            } else {
                QuadRoots::None // 0 = c ≠ 0：无解
            }
        } else {
            // 线性单根；量级溢出 f64 同样显式不可判定
            let t = -c / b;
            if t.is_finite() {
                QuadRoots::One(t)
            } else {
                QuadRoots::Indeterminate
            }
        };
    }
    match discriminant_sign(a, b, c) {
        None => QuadRoots::Indeterminate,
        Some(1) => {
            // 与 discriminant_sign 同一 (hi, lo) 精确对：b² − 4ac = (bh − 4ph) + (bl − 4pl)，
            // 各项一次舍入。相消情形（bh ≈ 4ph）残差相减仍保留量值——比 mul_add 的
            // 单次舍入多扛住一层消去；结果非正 / 非有限才退到噪声尺度。
            let (bh, bl) = match two_prod(b, b) {
                Some(p) => p,
                None => return QuadRoots::Indeterminate,
            };
            let (ph, pl) = match two_prod(a, c) {
                Some(p) => p,
                None => return QuadRoots::Indeterminate,
            };
            let scale = bh.abs() + 4.0 * ph.abs();
            if !scale.is_finite() {
                return QuadRoots::Indeterminate; // 两个 ~1e308 量级：尺度退化
            }
            let mut disc = (bh - 4.0 * ph) + (bl - 4.0 * pl);
            if !disc.is_finite() || disc <= 0.0 {
                // 精确判别式为正而双精度差分落入舍入零域：根间距 ≤ √(ε·尺度)，
                // 用噪声尺度作代表量值（低于该分辨极限在 f64 内本就不可分辨）。
                disc = scale.max(f64::MIN_POSITIVE) * f64::EPSILON;
            }
            let sq = disc.sqrt();
            if b == 0.0 {
                let t = (-c / a).max(0.0).sqrt();
                return if t.is_finite() {
                    QuadRoots::Two(-t, t)
                } else {
                    QuadRoots::Indeterminate
                };
            }
            let q = -0.5 * (b + b.signum() * sq);
            let (t1, t2) = (q / a, c / q);
            // 根量级超出 f64（如 a 下溢、c 巨大）→ 显式不可判定，不吐 ±inf
            if t1.is_finite() && t2.is_finite() {
                QuadRoots::Two(t1.min(t2), t1.max(t2))
            } else {
                QuadRoots::Indeterminate
            }
        }
        Some(0) => {
            let t = -b / (2.0 * a);
            if t.is_finite() {
                QuadRoots::Double(t)
            } else {
                QuadRoots::Indeterminate
            }
        }
        _ => QuadRoots::None,
    }
}

// ── 四次根包：系数、DK 初值、interval Newton、完整性扫描 ─────────────────

/// 首一四次方程 `t⁴ + c3 t³ + c2 t² + c1 t + c0` 的系数
/// （与 `geometry_extra::dk_roots` 的 Horner 内层约定一致）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quartic {
    pub c3: f64,
    pub c2: f64,
    pub c1: f64,
    pub c0: f64,
}

impl Quartic {
    /// 区间包络（区间 Horner；`F(X) ⊇ {P(t) : t ∈ X}`，因此含根的盒子剪不掉）。
    fn eval_iv(&self, x: Interval) -> Interval {
        (((x + iv(self.c3, self.c3)) * x + iv(self.c2, self.c2)) * x + iv(self.c1, self.c1)) * x
            + iv(self.c0, self.c0)
    }

    /// 导数区间 `F′(X)`（Horner：`((4t + 3c3)t + 2c2)t + c1`）。
    fn deriv_iv(&self, x: Interval) -> Interval {
        (((x * iv(4.0, 4.0) + iv(3.0 * self.c3, 3.0 * self.c3)) * x
            + iv(2.0 * self.c2, 2.0 * self.c2))
            * x)
            + iv(self.c1, self.c1)
    }
}

/// 射线–圆环面（局部系，轴 = z）的首一四次系数；`d` 需为单位向量。
/// 与 `geometry_extra::torus_local_crossings` 同一推导的 f64 权威版。
pub fn torus_quartic(major: f64, minor: f64, o: [f64; 3], d: [f64; 3]) -> Quartic {
    let r2 = major * major;
    let oo = o[0] * o[0] + o[1] * o[1] + o[2] * o[2];
    let od = o[0] * d[0] + o[1] * d[1] + o[2] * d[2];
    let g = oo + r2 - minor * minor;
    Quartic {
        c3: 4.0 * od,
        c2: 2.0 * g + 4.0 * od * od - 4.0 * r2 * (d[0] * d[0] + d[1] * d[1]),
        c1: 4.0 * od * g - 8.0 * r2 * (o[0] * d[0] + o[1] * d[1]),
        c0: g * g - 4.0 * r2 * (o[0] * o[0] + o[1] * o[1]),
    }
}

/// 射线–cyclide（局部系）的首一四次系数；`d` 需为单位向量。
/// 与 `geometry_extra::cyclide_local_crossings` 同一推导的 f64 权威版。
#[allow(clippy::too_many_arguments)]
pub fn cyclide_quartic(
    a: f64,
    b: f64,
    dd: f64,
    c: f64,
    shift: [f64; 3],
    o: [f64; 3],
    d: [f64; 3],
) -> Quartic {
    let ox = o[0] - shift[0];
    let oy = o[1] - shift[1];
    let oz = o[2] - shift[2];
    let (dx, dy) = (d[0], d[1]);
    let big_a = ox * ox + oy * oy + oz * oz;
    let b1 = ox * dx + oy * dy + oz * d[2];
    let bb = b * b - dd * dd;
    let g = big_a + bb;
    let p0 = ox * a - c * dd;
    let p1 = dx * a;
    Quartic {
        c3: 4.0 * b1,
        c2: 2.0 * g + 4.0 * b1 * b1 - 4.0 * p1 * p1 - 4.0 * b * b * dy * dy,
        c1: 4.0 * b1 * g - 8.0 * p0 * p1 - 8.0 * b * b * oy * dy,
        c0: g * g - 4.0 * p0 * p0 - 4.0 * b * b * oy * oy,
    }
}

/// 四次方程实根的**认证结果**（§3.1 三值语义在根层面的落点）。
#[derive(Debug, Clone, PartialEq)]
pub enum QuarticRoots {
    /// 全部实根均经盒认证（严格符号/单调 + 歧义中值引理），且完整性扫描未发现遗漏
    /// （升序；空表 = 证明无实根）。
    Certified(Vec<f64>),
    /// 存在无法判定的区域（重根 / 相切 / 扫描预算耗尽）——显式 `Unknown`，
    /// `certified` 为已证根（可能不完整，调用方对整条射线按 Unknown 处理）。
    Unknown { certified: Vec<f64> },
}

impl QuarticRoots {
    /// 已证根（两种情形都返回；`Unknown` 时可能不完整）。
    pub fn roots(&self) -> &[f64] {
        match self {
            QuarticRoots::Certified(v) | QuarticRoots::Unknown { certified: v } => v,
        }
    }

    /// 是否存在无法判定的区域。
    pub fn is_unknown(&self) -> bool {
        matches!(self, QuarticRoots::Unknown { .. })
    }
}

// 复数小工具（DK 初值用；纯 f64，不进区间域）
type Cx = (f64, f64);

#[inline]
fn c_add(a: Cx, b: Cx) -> Cx {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: Cx, b: Cx) -> Cx {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: Cx, b: Cx) -> Cx {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_div(a: Cx, b: Cx) -> Cx {
    let m = b.0 * b.0 + b.1 * b.1;
    ((a.0 * b.0 + a.1 * b.1) / m, (a.1 * b.0 - a.0 * b.1) / m)
}

fn c_eval(q: &Quartic, z: Cx) -> Cx {
    let mut h = c_add(z, (q.c3, 0.0));
    h = c_add(c_mul(h, z), (q.c2, 0.0));
    h = c_add(c_mul(h, z), (q.c1, 0.0));
    c_add(c_mul(h, z), (q.c0, 0.0))
}

/// 与 `dk_roots` 完全相同的四个种子（缩放后）。
fn dk_seeds(rad: f64) -> [Cx; 4] {
    const S: [Cx; 4] = [(0.4, 0.9), (-0.65, 0.72), (-0.74, -0.67), (0.73, -0.68)];
    S.map(|(re, im)| (re * rad, im * rad))
}

/// 同步 Durand–Kerner 迭代（与 `dk_roots` 同为 50 次，全部初值用旧值同时更新）。
fn dk_iterate(q: &Quartic, z: &mut [Cx; 4]) {
    for _ in 0..50 {
        let mut next = *z;
        for (i, zi) in z.iter().enumerate() {
            let p = c_eval(q, *zi);
            let mut den: Cx = (1.0, 0.0);
            for (j, zj) in z.iter().enumerate() {
                if j != i {
                    den = c_mul(den, c_sub(*zi, *zj));
                }
            }
            next[i] = c_sub(*zi, c_div(p, den));
        }
        *z = next;
    }
}

/// x 处的 ulp（f64）；非有限 → `NaN`（使叶宽 / 邻近判断全部落空，交给预算收口）。
fn ulp64(x: f64) -> f64 {
    let a = x.abs();
    if a.is_normal() {
        a.next_up() - a
    } else if a.is_finite() {
        f64::MIN_POSITIVE
    } else {
        f64::NAN
    }
}

/// 收录已证根：(根值, 误差上界)；两证相距 ≤ 8·max(容差, ulp) 视为同一根并入
/// （容差取保守并——真根同时落在两张证书内），低于分辨极限的根族合并不重报。
fn push_root(certs: &mut Vec<(f64, f64)>, r: f64, t: f64) {
    for c in certs.iter_mut() {
        let scale = c.1.max(t).max(ulp64(c.0)).max(ulp64(r));
        if scale.is_finite() && (c.0 - r).abs() <= 8.0 * scale {
            c.1 = c.1.max(t);
            return;
        }
    }
    certs.push((r, t));
}

/// 盒探测结果（区间证明的三值语义在单个盒上的落点）。
#[derive(Debug)]
enum Probe {
    /// 盒内**恰有一**实根：(根值, 误差上界 ≤ 包络分辨率)。
    Root(f64, f64),
    /// **证明盒内无实根**（包络剪枝，或单调 + 端点严格同号）——不是"没找到"。
    NoRoot,
    /// 无法证明存在且唯一（`0 ∈ F′(X)`：重根特征，或歧义收口触及临界点）。
    Unknown,
}

/// 点 `t` 处 `P(t)` 的**严格符号**：点区间包络不含 0 才作数（含 0 = 点求值
/// 分辨歧义——`|P(t)|` 落在包络宽度内，f64 点求值无法判向）。
fn point_sign(q: &Quartic, t: f64) -> Option<i8> {
    let p = q.eval_iv(iv(t, t));
    if p.is_empty() {
        None
    } else if p.inf() > 0.0 {
        Some(1)
    } else if p.sup() < 0.0 {
        Some(-1)
    } else {
        None
    }
}

/// 分辨歧义处的存在唯一性收口（中值引理，区间 Newton 的存在性对偶形式）：
/// `|P(m)| ≤ E`（包络含 0）且 `[m±w]` 上 `|P′| ≥ g > 0` ⟹ 恰有一实根落在
/// `[m − E/g, m + E/g]`（MVT + 严格单调），返回 `(m, 上界)`。
/// `w` 从 ulp 起按 4× 阶梯放大取证——触到临界点（`0 ∈ F′`）后永远取不到 `g`
/// → `None`（近重根本就不可认证，交回扫描定 `Unknown`）。
fn ambiguity_lemma(q: &Quartic, m: f64) -> Option<(f64, f64)> {
    let pe = q.eval_iv(iv(m, m));
    if pe.is_empty() {
        return None;
    }
    let e = pe.inf().abs().max(pe.sup().abs()); // |P(m)| ≤ e（包络外舍入）
    let mut w = ulp64(m);
    if !w.is_finite() || w <= 0.0 {
        w = f64::MIN_POSITIVE;
    }
    for _ in 0..64 {
        if !w.is_finite() {
            return None;
        }
        let fpi = q.deriv_iv(iv(m - w, m + w));
        if !fpi.is_empty() && !fpi.contains(0.0) {
            let g = if fpi.inf() > 0.0 {
                fpi.inf()
            } else {
                -fpi.sup()
            };
            // 上舍：d ≥ E/g 真实值（e 外舍、g 内舍、乘 (1+8ε) 盖住一次除法舍入）
            let d = (e / g) * (1.0 + 8.0 * f64::EPSILON);
            if d <= w {
                // [m±d] ⊆ [m±w]（单调性已在此区域证明），根与 m 相距 ≤ d
                return Some((m, d));
            }
        }
        w *= 4.0;
    }
    None
}

/// 单盒认证：包络剪枝 → `0 ∈ F′(X)` 判 `Unknown`（重根特征）→ 端点严格符号
/// 分类（同号 → 无根；跨零 → 二分收窄至歧义引理或 8 ulp 分辨极限）。
fn probe_box(q: &Quartic, x: Interval) -> Probe {
    if x.is_empty() {
        return Probe::NoRoot;
    }
    let p = q.eval_iv(x);
    if p.is_empty() || p.inf() > 0.0 || p.sup() < 0.0 {
        return Probe::NoRoot; // 包络证明 0 ∉ P(X)
    }
    let fp = q.deriv_iv(x);
    if fp.is_empty() {
        return Probe::Unknown;
    }
    if fp.contains(0.0) {
        return Probe::Unknown; // 无法证明单调——重根/相切特征
    }
    let (mut lo, mut hi) = (x.inf(), x.sup());
    // 端点分类：歧义端点（含 0 的点包络）先走中值引理；同号 → 确证无根
    let a = match point_sign(q, lo) {
        Some(s) => s,
        None => {
            return match ambiguity_lemma(q, lo) {
                Some((r, t)) => Probe::Root(r, t),
                None => Probe::Unknown,
            };
        }
    };
    let b = match point_sign(q, hi) {
        Some(s) => s,
        None => {
            return match ambiguity_lemma(q, hi) {
                Some((r, t)) => Probe::Root(r, t),
                None => Probe::Unknown,
            };
        }
    };
    if a == b {
        return Probe::NoRoot; // 单调 + 严格同号：盒内确证无根（区间过估计）
    }
    // 严格跨零：二分保持跨零，直到端点进入分辨带（中值引理）或 8 ulp 极限
    for _ in 0..128 {
        let w = hi - lo;
        let mid = lo + w * 0.5;
        if w <= 8.0 * ulp64(mid) {
            return Probe::Root(mid, 0.5 * w);
        }
        match point_sign(q, mid) {
            None => {
                return match ambiguity_lemma(q, mid) {
                    Some((r, t)) => Probe::Root(r, t),
                    None => Probe::Unknown,
                };
            }
            Some(c) => {
                if c == a {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
        }
    }
    Probe::Unknown
}

/// 完整性扫描：Cauchy 界内二分，`0 ∉ P(X)` 才剪枝（单调同号的过估计盒提前
/// 确证无根）；叶锁死处做最后一次盒认证，`Unknown` 且邻近无已证根 → `unknown`。
/// 预算耗尽同样置 `unknown`（保守可判定）。
struct Sweep<'a> {
    q: &'a Quartic,
    certs: &'a mut Vec<(f64, f64)>,
    budget: u32,
    unknown: bool,
}

impl Sweep<'_> {
    fn visit(&mut self, x: Interval, depth: u32) {
        if self.unknown {
            return;
        }
        let p = self.q.eval_iv(x);
        if p.is_empty() || p.inf() > 0.0 || p.sup() < 0.0 {
            return; // 区间证明 0 ∉ P(X)：无根（含根的盒子永远剪不掉）
        }
        // 单调且端点严格同号：确证无根——剪掉区间过估计逼出来的伪下降
        let fp = self.q.deriv_iv(x);
        if !fp.is_empty() && !fp.contains(0.0) {
            if let (Some(a), Some(b)) = (point_sign(self.q, x.inf()), point_sign(self.q, x.sup())) {
                if a == b {
                    return;
                }
            }
        }
        if self.budget == 0 {
            self.unknown = true;
            return;
        }
        self.budget -= 1;
        let mid = x.mid();
        let w = x.sup() - x.inf();
        let u = ulp64(mid).max(ulp64(x.inf())).max(ulp64(x.sup()));
        if depth >= 128 || w <= 8.0 * u {
            // 叶：0 仍 ∈ P → 最后一次盒认证（DK 漏掉的单根在此补证）
            match probe_box(self.q, x) {
                Probe::Root(r, t) => push_root(self.certs, r, t),
                Probe::NoRoot => {}
                Probe::Unknown => {
                    // 与已证根的容差盒邻接/交叠 → 已解释（低于分辨极限的根族并入）
                    let explained = self.certs.iter().any(|&(c, t)| {
                        let slack = 8.0 * u + t;
                        slack.is_finite() && c - slack <= x.sup() && c + slack >= x.inf()
                    });
                    if !explained {
                        self.unknown = true;
                    }
                }
            }
            return;
        }
        let lo = iv(x.inf(), mid);
        let hi = iv(mid, x.sup());
        self.visit(lo, depth + 1);
        self.visit(hi, depth + 1);
    }
}

/// 完整性扫描的总区间预算（每次二分记一次；远超常规用例所需）。
const SWEEP_BUDGET: u32 = 16_384;

/// 认证求解首一四次 `t⁴ + c3 t³ + c2 t² + c1 t + c0`：
/// DK 初值盒 + 盒认证（区间符号/单调 + 歧义中值引理）+ Cauchy 界内完整性扫描。
/// 任何无法证明唯一性的区域 → [`QuarticRoots::Unknown`]，绝不静默漏判。
pub fn quartic(q: Quartic) -> QuarticRoots {
    if !(q.c3.is_finite() && q.c2.is_finite() && q.c1.is_finite() && q.c0.is_finite()) {
        return QuarticRoots::Unknown {
            certified: Vec::new(),
        };
    }
    let rad = 1.0 + q.c3.abs().max(q.c2.abs()).max(q.c1.abs()).max(q.c0.abs());
    let mut certs: Vec<(f64, f64)> = Vec::new();

    // 1) DK 初值（与 dk_roots 同种子、同步数）→ 逐初值盒认证
    let mut z = dk_seeds(rad);
    dk_iterate(&q, &mut z);
    for (zr, _) in z {
        if !zr.is_finite() {
            continue;
        }
        let r0 = (rad * 1e-6).max(8.0 * ulp64(zr));
        if !r0.is_finite() || !iv(zr - r0, zr + r0).is_common_interval() {
            continue;
        }
        if let Probe::Root(r, t) = probe_box(&q, iv(zr - r0, zr + r0)) {
            push_root(&mut certs, r, t);
        }
        // 无根 / 无法证唯一不在此定论——由完整性扫描收口
    }

    // 2) 完整性扫描：补 DK 之漏、锁死处定 Unknown
    let mut sweep = Sweep {
        q: &q,
        certs: &mut certs,
        budget: SWEEP_BUDGET,
        unknown: false,
    };
    sweep.visit(iv(-rad, rad), 0);
    let unknown = sweep.unknown;

    certs.sort_by(|a, b| a.0.total_cmp(&b.0));
    let certs: Vec<f64> = certs.into_iter().map(|(v, _)| v).collect();
    if unknown {
        QuarticRoots::Unknown { certified: certs }
    } else {
        QuarticRoots::Certified(certs)
    }
}

// ── P2 验收（docs/freeform-robust-boolean.md §6） ─────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::dupin_cyclide;

    // ── 判别式精确路径（§3.3 路线 1） ──────────────────────────────────

    /// 精确重根 → `Double`（相切射线的确定结果，不是静默丢弃）。
    #[test]
    fn quadratic_exact_double_root() {
        assert_eq!(quadratic(1.0, -4.0, 4.0), QuadRoots::Double(2.0));
        assert_eq!(quadratic(1.0, -2.0, 1.0), QuadRoots::Double(1.0));
    }

    /// 互异两根升序；精确负判别式 → `None`（射线确定不相交）。
    #[test]
    fn quadratic_two_roots_and_proven_none() {
        assert_eq!(quadratic(1.0, -3.0, 2.0), QuadRoots::Two(1.0, 2.0));
        assert_eq!(quadratic(1.0, 0.0, 1.0), QuadRoots::None);
        assert_eq!(quadratic(2.0, 4.0, 100.0), QuadRoots::None); // disc = −784
    }

    /// 掠射：判别式 4e-13 **小于** GPU 守卫阈值 1e-12——旧代码静默丢掉这对交点
    /// （§1「掠射静默丢交点」），精确路径必须给出两根并夹住切点 t = 1。
    #[test]
    fn grazing_pair_below_old_guard_threshold() {
        let (a, b, c) = (1.0, -2.0, 1.0 - 1e-13);
        let naive = b * b - 4.0 * (a * c);
        assert!(
            naive > 0.0 && naive < 1e-12,
            "前置条件：常规 disc 在旧守卫零域"
        );
        match quadratic(a, b, c) {
            QuadRoots::Two(t1, t2) => {
                assert!(
                    t1 < 1.0 && t2 > 1.0,
                    "两根应夹住切点 1.0，实际 ({t1}, {t2})"
                );
                assert!(
                    t2 - t1 < 1e-6,
                    "根间距应 ≈ √disc = 6.3e-7，实际 {}",
                    t2 - t1
                );
            }
            other => panic!("期望 Two，实际 {other:?}"),
        }
    }

    /// 亚 ulp 正判别式：a = next_up(1)、c = next_down²(1) → 精确 disc = 2⁻¹⁰²，
    /// 而 `b*b − 4(a*c)` 恰好算成 0（单次舍入的零域）——双对差分仍给出准确根。
    #[test]
    fn sub_ulp_positive_discriminant_exact_roots() {
        let a = 1.0f64.next_up();
        let c = 1.0f64.next_down().next_down(); // 1 − 2⁻⁵²
        let b = -2.0;
        // (1+2⁻⁵²)(1−2⁻⁵²) = 1 − 2⁻¹⁰⁴ → 精确 disc = 4 − 4(1−2⁻¹⁰⁴) = 2⁻¹⁰² > 0
        assert_eq!(b * b - 4.0 * (a * c), 0.0, "前置条件：常规计算落入零域");
        match quadratic(a, b, c) {
            QuadRoots::Two(t1, t2) => {
                // 真根 = 1 与 (1−2⁻⁵²)/(1+2⁻⁵²) ≈ 1 − 2⁻⁵¹
                assert_eq!(t2, 1.0, "较大根应精确为 1");
                let gap = t2 - t1;
                assert!(
                    gap > 1e-17 && gap < 1e-15,
                    "根间距应 ≈ 2⁻⁵¹ = 4.4e-16，实际 {gap}"
                );
            }
            other => panic!("期望 Two，实际 {other:?}"),
        }
    }

    /// 退化输入全部显式：线性单根 / 常数无解 / 恒等式 / 非有限 / 尺度溢出。
    #[test]
    fn quadratic_degenerate_inputs_are_explicit() {
        assert_eq!(quadratic(0.0, 2.0, -4.0), QuadRoots::One(2.0));
        assert_eq!(quadratic(0.0, 0.0, 1.0), QuadRoots::None);
        assert_eq!(quadratic(0.0, 0.0, 0.0), QuadRoots::Indeterminate);
        assert_eq!(quadratic(f64::NAN, 1.0, 1.0), QuadRoots::Indeterminate);
        assert_eq!(quadratic(1.0, f64::INFINITY, 1.0), QuadRoots::Indeterminate);
        // a·c 溢出：two_prod 失败 → 尺度退化，显式不可判定
        assert_eq!(quadratic(1e308, 1.0, 1e308), QuadRoots::Indeterminate);
        // a 为最小非规格化数：乘积下溢 → 显式不可判定（不猜"约等于线性"）
        assert_eq!(
            quadratic(f64::from_bits(1), 1.0, 1.0),
            QuadRoots::Indeterminate
        );
    }

    /// property：600 组同指数族（含构造的近相切 / 精确重根），判别式符号与
    /// i128 整数精确参照逐一相等；分类与符号一致。LCG 确定性，无随机源。
    #[test]
    fn discriminant_sign_matches_exact_integer_reference() {
        let mut st: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut rnd = move || {
            st = st
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (st >> 33) as u32
        };
        for case in 0..600 {
            let e = (rnd() % 61) as i32 - 30; // 2^e ∈ [2⁻³⁰, 2³⁰]，全程规格化
            let m: i64 = (1 << 20) + (rnd() as i64 % (1 << 21));
            let n: i64 = (1 << 20) + (rnd() as i64 % (1 << 21));
            // 构造 mb² − 4·ma·mc ∈ {0, ∓4m², 4mn+1}：精确重根与近相切各半
            let (ma, mc, mb): (i64, i64, i64) = match rnd() % 4 {
                0 => (m * m, n * n, 2 * m * n),
                1 => (m * m, n * n + 1, 2 * m * n),
                2 => (m * m, n * n - 1, 2 * m * n),
                _ => (m * m, n * n, 2 * m * n + 1),
            };
            let (sa, sc): (i64, i64) = (
                if rnd() & 1 == 1 { -1 } else { 1 },
                if rnd() & 1 == 1 { -1 } else { 1 },
            );
            // 精确参照：D = mb² − 4·(sa·sc)·ma·mc（2^e 平方为正，不改符号）
            let d_int = (mb as i128) * (mb as i128)
                - 4 * (sa as i128) * (sc as i128) * (ma as i128) * (mc as i128);
            let want = d_int.signum() as i8;
            let pow = (2.0f64).powi(e);
            let a = (sa * ma) as f64 * pow;
            let b = (mb as f64) * pow * if rnd() & 1 == 1 { -1.0 } else { 1.0 };
            let c = (sc * mc) as f64 * pow;

            assert_eq!(
                discriminant_sign(a, b, c),
                Some(want),
                "case {case}: D_int = {d_int}"
            );
            match (want, quadratic(a, b, c)) {
                (1, QuadRoots::Two(t1, t2)) => assert!(t1 < t2 && t1.is_finite() && t2.is_finite()),
                (0, QuadRoots::Double(t)) => assert!(t.is_finite()),
                (-1, QuadRoots::None) => {}
                (_, other) => panic!("case {case}: sign {want} 与分类 {other:?} 不一致"),
            }
        }
    }

    // ── 四次根包（§3.3 路线 2） ────────────────────────────────────────

    /// 四个互异单根全部经证明 → `Certified`（升序、无重复）。
    #[test]
    fn quartic_four_simple_roots_certified() {
        // (t−1)(t−2)(t−3)(t−4)
        let q = Quartic {
            c3: -10.0,
            c2: 35.0,
            c1: -50.0,
            c0: 24.0,
        };
        match quartic(q) {
            QuarticRoots::Certified(rs) => {
                assert_eq!(rs.len(), 4, "四根都应被证明：{rs:?}");
                for (got, want) in rs.iter().zip([1.0, 2.0, 3.0, 4.0]) {
                    assert!((got - want).abs() < 1e-9, "根 {got} ≠ {want}");
                }
            }
            other => panic!("期望 Certified([1,2,3,4])，实际 {other:?}"),
        }
    }

    /// 无实根 → `Certified([])`：完整性扫描证明根域内 0 ∉ P（不是"没找到"）。
    #[test]
    fn quartic_no_real_roots_certified_empty() {
        let q = Quartic {
            c3: 0.0,
            c2: 0.0,
            c1: 0.0,
            c0: 1.0, // t⁴ + 1，四根全在复平面
        };
        assert_eq!(quartic(q), QuarticRoots::Certified(Vec::new()));
    }

    /// 重根 t=1：`0 ∈ F′(X)` 无法证唯一 → 显式 `Unknown`；
    /// 单根 2、3 仍被证明（`Unknown` 的已证部分不丢）。
    #[test]
    fn quartic_double_root_reports_unknown() {
        // (t−1)²(t−2)(t−3) = t⁴ − 7t³ + 17t² − 17t + 6
        let q = Quartic {
            c3: -7.0,
            c2: 17.0,
            c1: -17.0,
            c0: 6.0,
        };
        let r = quartic(q);
        assert!(r.is_unknown(), "重根必须显式 Unknown，实际 {r:?}");
        let rs = r.roots();
        assert_eq!(rs.len(), 2, "单根 2、3 应已证：{rs:?}");
        assert!((rs[0] - 2.0).abs() < 1e-9 && (rs[1] - 3.0).abs() < 1e-9);
    }

    /// 环面相切射线（R=2, r=1，o=(3,0,5)，d=(0,0,−1)）：唯一实根 t=5 是重根 →
    /// `Unknown{[]}`（§7「不在相切处猜」），绝不把"没证出来"混同"没有交点"。
    #[test]
    fn quartic_tangent_torus_ray_reports_unknown() {
        let q = torus_quartic(2.0, 1.0, [3.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert_eq!(
            (q.c3, q.c2, q.c1, q.c0),
            (-20.0, 174.0, -740.0, 1225.0),
            "与 geometry_extra 推导同式"
        );
        let r = quartic(q);
        assert!(r.is_unknown(), "相切必须 Unknown，实际 {r:?}");
        assert!(r.roots().is_empty(), "唯一实根是重根，无可证根：{r:?}");
    }

    /// 环面横截射线（o=(2,0,5)）：两个单根 4、6 全部证明 → `Certified`。
    #[test]
    fn quartic_transversal_torus_ray_certified() {
        let q = torus_quartic(2.0, 1.0, [2.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert_eq!((q.c3, q.c2, q.c1, q.c0), (-20.0, 164.0, -640.0, 960.0));
        match quartic(q) {
            QuarticRoots::Certified(rs) => {
                assert_eq!(rs.len(), 2, "实根恰为 4、6：{rs:?}");
                assert!((rs[0] - 4.0).abs() < 1e-9 && (rs[1] - 6.0).abs() < 1e-9);
            }
            other => panic!("期望 Certified([4,6])，实际 {other:?}"),
        }
    }

    /// cyclide 射线端到端：过已知表面点的射线必须证出 t=4，且全部已证根回代
    /// `implicit` 残差在相对量级内（系数构造、DK、Newton、扫描全链路一致）。
    #[test]
    fn quartic_cyclide_ray_roots_satisfy_implicit() {
        let cy = dupin_cyclide(1.0, 0.6, 1.5, [0.0, 0.0, 0.0]);
        let s = cy.surface(0.7, 1.1);
        assert!(
            cy.implicit(s[0], s[1], s[2]).abs() < 1e-10,
            "表面点前置条件"
        );
        let o = [s[0], s[1], s[2] + 4.0];
        let d = [0.0, 0.0, -1.0]; // 单位向量，t=4 处恰为表面点 s
        let q = cyclide_quartic(cy.a, cy.b, cy.d, cy.c(), cy.shift, o, d);
        let r = quartic(q);
        assert!(!r.is_unknown(), "横截 cyclide 射线应可完全证明，实际 {r:?}");
        let rs = r.roots();
        assert!(
            rs.iter().any(|&t| (t - 4.0).abs() < 1e-9),
            "应证出构造根 t=4：{rs:?}"
        );
        for &t in rs {
            let (x, y, z) = (o[0], o[1], o[2] + d[2] * t);
            let f = cy.implicit(x, y, z);
            // 相对残差：以隐式方程三项的量级为尺度
            let (bb, sx, sy) = (cy.b * cy.b - cy.d * cy.d, x - cy.shift[0], y - cy.shift[1]);
            let rho = sx * sx + sy * sy + (z - cy.shift[2]) * (z - cy.shift[2]);
            let lin = cy.a * sx - cy.c() * cy.d;
            let scale = (rho + bb) * (rho + bb) + 4.0 * lin * lin + 4.0 * cy.b * cy.b * sy * sy;
            assert!(
                f.abs() <= 1e-9 * scale + 1e-9,
                "根 t={t} 回代残差 {f} 超出相对量级 {scale}"
            );
        }
    }

    /// 非有限系数 → `Unknown`（显式，不进区间域）。
    #[test]
    fn quartic_nonfinite_coeffs_unknown() {
        let q = Quartic {
            c3: f64::NAN,
            c2: 1.0,
            c1: 1.0,
            c0: 1.0,
        };
        assert_eq!(
            quartic(q),
            QuarticRoots::Unknown {
                certified: Vec::new()
            }
        );
    }
}
