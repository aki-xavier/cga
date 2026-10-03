# 自由曲面 + 可证明布尔：补 CGA 内核的两块理论缺口

状态：提案定稿，P0（诊断 + 退化用例库）已落地；P1–P4 待实施。
适用：`crates/cga-gpu/src/{geom_kernels,csg,geometry_extra}.rs`、`crates/cga-core/src/{geometry,modeling}.rs`。
诊断行号为 2026-10-03 快照，随代码漂移，以符号名为准。

## 1. 诊断：容差现在散在哪几处

| 位置（文件:符号） | 常数 | 实际作用 | 问题 |
|---|---|---|---|
| `geom_kernels.rs::csg_nearest_surface` | `delta = 1e-4` | 在 t±δ 双侧探 `csg_contains`，靠**是否翻转**认定边界 | 绝对容差：在图元被求值的空间里量取，与特征尺寸无关 |
| `geom_kernels.rs::csg_uv` | `delta = 1e-4` | 判定"落点踩在哪个子面上" | 同上，且与 nearest_surface 用同一 δ，错会一起错 |
| `csg.rs::sphere_crossings` | `disc > 1e-12` | 判别式非负守卫 | 掠射（双根间距 < 2√δ）静默丢交点 |
| `csg.rs::plane_crossings` / `cylinder_crossings` | `|denom| > 1e-9` | 防除零 | 平行判定与模型尺度无关 |
| `csg.rs::cylinder_crossings` | `a > 1e-12 ∧ disc > 1e-12` | 侧壁二次方程有效性 | 小半径圆柱在大尺度坐标下失效 |
| `geometry_extra.rs::dk_roots` | 迭代容差 | Durand–Kerner 解三次/四次 | 无根隔离、无重根语义、无误差证 |
| `trimesh.rs::trimesh_contains` | — | +x 射线**奇偶计数** | 非水密网格/开边 → 分类无意义 |
| 全局 | f64 代数核 / f32 渲染 | Metal 无 f64 | GPU 上一切阈值都是 f32 语义 |

三个根本问题：

1. **绝对容差 = 单位依赖**：`delta = 1e-4` 是写死的绝对距离，生死由"特征尺寸 vs δ"决定。
   同一设计以两种单位表达（壁厚恒为外形的 5e-5 倍），大单位活、小单位死。
2. **二值且无证据**：flip 判不出来时结果**静默错误**（洞、闪面、整块消失），没有任何状态能告诉你"这一段我不确定"。
3. **相切/掠射是盲区**：双根让 t±δ 两次都翻或都不翻，δ 取任何值都会在某个特征尺寸上失效。

### 1.1 P0 实测（`crates/cga-gpu/src/degenerate.rs`，2026-10-03）

`cargo test -p cga-gpu degenerate -- --include-ignored` → 4 通过 / 4 失败（失败即 `#[ignore]` 清单）。

| 用例 | 期望 | 实测 | 结论 |
|---|---|---|---|
| 薄壳 wall=5e-5，外形 1 | t=1.5 | **None（整块消失）** | δ 吞掉 <δ 的壁 |
| 薄壳 wall=5e-4（=5δ） | t=1.5 | 命中 | 阈值恰在 δ 附近 |
| 薄壳 wall=0.05（=500δ） | t=1.5 | 命中 | 正常路径护栏 |
| 同设计换单位 size=1 / 1000 | 均命中 | size=1 → None；size=1000 → 命中 | 绝对容差 = 单位依赖 |
| 掠射球 y=1−1e−6（穿深 ~2.8e−3） | 命中 | 命中 | 2δ 以内才安全 |
| 掠射球 y=1−1e−10（穿深 ~1.4e−5） | 命中 | **None** | <2δ 的穿入深度漏判 |
| 相切并集，从球心向 +x | t=3（切点非表面） | t=3 | 相切处的正确行为护栏 |
| 盒 − 圆柱孔（正常路径） | t=1.5 / contains 正确 | 命中且正确 | 回归护栏 |
| 缺面立方体，体内点 | `true` | **`false`** | 奇偶计数对非水密网格必错（P4） |

## 2. 转向：要补的不是 B-rep 容差，是可证明分类

B-rep 那套公差理论（SSI 曲线求交、拓扑重建、healing、公差带）是为**显式共享的边和面**服务的。本仓库的布尔是**逐射线分类**：`geom_crossings` 收集交点 → 排序 → `csg_contains` 双侧探测找翻转 → 最近表面。没有共享拓扑结构可以"容忍误差"，也就不该去补 B-rep 的理论。

> 所以问题要重新表述：**把 δ 从经验常数，改写成由 Lipschitz 界 + 区间算术推导出来的可证明余量（certified margin）。**

这是 verified numerics（Moore / Kearfott 区间 Newton）的路子，不是 OCCT 的路子。不需要 SSI，需要区间算术。

## 3. 理论骨架

### 3.1 三值逻辑取代二值

```
classify(segment) ∈ { In, Out, Unknown }
Unknown → 细分消除；渲染可弃权（交抗锯齿），导出不允许
```

整套理论的唯一卖点是**可证明**：猜不出来就报 `Unknown`，绝不静默猜。

### 3.2 Lipschitz 界 —— δ 的合法来源

每个原语补两个接口：

```rust
trait CertifiedPrimitive {
    fn value(&self, x: [f64; 3]) -> f64;              // 隐式函数值（已有）
    fn lipschitz(&self, lo: [f64;3], hi: [f64;3]) -> f64; // 新增：梯度范数上界
}
```

- 硬 `min`/`max` CSG **保持 1-Lipschitz**（Bálint 2019）→ union/intersection/difference 继承界；**`smooth_min` 会破坏它**，要保证就只能硬切换。
- CGA 的 `d²` 型量不是 1-Lipschitz（梯度 2d）→ 改用 `d = |x−c| − r`，或按位置给 L。
- 段上界（保守 sphere tracing）：`|f(t)| ≥ |f(t0)| − L·Δt` → 两端判号即覆盖整段。

**固定 `delta = 1e-4` 的直接替代**：在子面 i 的交点 t*（f_i = 0）处，其余子面 f_j 的梯度界 L_j → 取 `ε = 1 / max(L_j)`。若对所有 j 有 `|f_j(t*)| > L_j·ε` → 符号**已证**；某个 j 落进带内 → `Unknown` → 细分或提精度。这一步单独就能换掉现有 flip 逻辑，架构不动。

### 3.3 根隔离 —— IVT 抓不到重根

- 变号根：端点异号 + 定向舍入 → 存在性平凡成立。
- **相切（重根）**：不变号，IVT 永远看不见。这才是"理论短板"真正所在。三条路并用：
  1. 代数原语（球/柱/锥/环/cyclide）→ 判别式 / 结式**精确**分类（判别式是现有四次方程系数的副产品）；
  2. `interval Newton` 证明盒子内存在且唯一，重根时报告"无唯一解" → 转 `Unknown`；
  3. 真判不出 → 显式报告该像素不可判定。
- `dk_roots` 保留做初值，外面套区间封装（Rust 参考 Guillemot 2024 *Validated Numerics for Algebraic Path Tracking*；区间算术用 `inari`，IEEE 1788.1-2017）。

### 3.4 导出保证

- 分类可证明 → marching cubes 只输出**全部 8 角点已判定**的 cell → 水密是定理不是运气。
- 网格后端的奇偶计数换**广义环绕数**（Jacobson 2013）→ 非水密/开边也能分类，仍挂进 `contains` 协议。

## 4. 自由曲面：三条路线

| 路线 | 做法 | 判定 |
|---|---|---|
| **A. 参数曲面进现有协议** | Bézier clipping + **几何分离界**（Sederberg "Complete Subdivision Algorithms" 一支）隔离射线-曲面全部交点，interval Newton 精化；曲面作为一个有 `crossings`/`contains` 的叶子挂进 `geom_crossings` | ✅ **首选**。`crossings + contains` 协议天生就是为它准备的；`AffineGeometry` 的射线逆变换已解决变换问题 |
| **B. 射影化**（NURBS → rational/PGA，Hildenbrand rational GA 一线） | 想让 CGA"原生"表达有理曲面 | ❌ 放弃。节点向量、G1/G2 连续、裁剪是**独立体系**，GA 不提供也换不来，投入产出最差 |
| **C. 距离场化** | 用控制点凸包给出 L-Lipschitz 的保守距离下界 | ⚠️ **降级为"给界的手段"**：只用于 §3.2 给 Lipschitz 常数，不作为几何真相 |

**架构结论**：CGA 继续管运动 / 原语 / 距离（blade 的闭式交与 L 是它真正的优势），自由曲面走标准参数几何，靠 motor 射线逆变换 + `crossings`/`contains` 协议统一。**不把 NURBS 塞进 blade** —— §2 的边界线把"自由曲面理论短板"绕过去了，代价是承认 CGA 不是全能表示。

## 5. CGA 在这套理论里的位置

值得记下来的**研究空白**：CGA 文献做交 / 距离 / 运动的很多，做 verified numerics 的几乎没有。把"blade 的闭式距离 + 区间算术"合成**可证明 CSG**，是没人占的位置——本仓库已有 32 分量 f64 核和四次根求解器，起步成本比从零写低一个量级。

CGA 真正给到的：

- 原语的闭式距离 / 交（球、圆、平面、motor）→ L 可解析求得，不用数值微分；
- versor 变换下**界可随变换搬运**（共轭后重新解析求界，不必重新采样）；
- 四次方程系数已有 → 判别式分类零成本。

CGA 给不了的（交回通用几何层）：参数曲面、拓扑、裁剪、容差管理。

## 6. 分阶段落地

| 阶段 | 内容 | 验收 |
|---|---|---|
| **P0** ✅ | `delta`/`1e-*` 语义盘点 + 退化用例库 | `degenerate.rs` 跑出**当前失败清单**，每条带 `#[ignore]` 注释指向本文 |
| **P1** | 每原语实现 `lipschitz()`；`csg_nearest_surface` 的 flip 改成"ε 由界推导 + `Unknown` 细分"；常数收进 `Tol { abs, rel, min_feature }`，尺度相对化 | P0 的 `#[ignore]` 用例全部转正；同设计跨单位（size = 1 / 1000）行为一致 |
| **P2** | 四次根包 interval Newton（DK 做初值）；判别式走代数精确路径 | 相切/重根用例有确定结果（命中或 `Unknown`），无静默漏判 |
| **P3** | Bézier/NURBS patch 实现 `crossings`（分离界 + clip），`contains` 用闭壳奇偶或 winding | 自由曲面可进 CSG，导出测试通过 |
| **P4** | 导出：MC 全判定 cell → 水密保证；mesh 后端换 winding number | 水密性作为断言进 CI（体积/表面积/欧拉示性数校验） |

**性能约束**：Metal 无 f64，区间求值不能下 GPU。分层——GPU 只走"界已在 CPU 端算好/已证"的快速路径，出现 `Unknown` 时回落 f64 CPU 路径（渲染则直接弃权交抗锯齿），不把区间算术塞进 kernel。

## 7. 明确不做的三件事

1. **不追 B-rep / STEP / PMI**：P0–P4 全做完也不改变这一点，别把它当目标。
2. **不用 `smooth_min`**：它悄悄偷走 Lipschitz 保证，得不偿失。
3. **不在相切处猜**：猜不出来就报 `Unknown`。

## 8. 参考文献

- Bálint & Várkonyi, *Operations on Signed Distance Function Estimates*, Acta Cybernetica 2019 — min/max 保持 1-Lipschitz。
- Matt Keeter, *Gradients are the new intervals* (2025) — 用梯度构造伪区间，图形学版的轻量区间。
- Sederberg et al., *Complete Subdivision Algorithms* — 几何分离界的射线-曲线/曲面求交。
- Nishita / Sederberg, *Bézier Clipping* 及后续稳健化改进（数值稳定的 ray-patch 求交）。
- Moore/Kearfott, *Introduction to Interval Analysis*；CAPD — interval Newton 存在性/唯一性证明。
- Guillemot et al., *Validated Numerics for Algebraic Path Tracking* (2024, Rust 实现)。
- `inari` crate — IEEE 1788.1-2017 区间算术，纯 Rust。
- Jacobson et al., *Robust Inside-Outside Segmentation using Generalized Winding Numbers*, SIGGRAPH 2013。
- Urick et al., *Watertight Boolean Operations*, CAD 2019 — trimmed B-rep 在 SSI 表达上的固有缺陷。
- nTop, *Implicit modeling vs B-rep* — 隐式表示为何"构造即鲁棒"。
