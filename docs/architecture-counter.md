<!-- markdownlint-configure-file {"MD013": false} -->
# 反面架构评审（counter-position，2026-10-12）

状态：**一次刻意站在对立面的评审**。本文不是计划、不是路线图、也不是决策记录——它是「若要论证 cga 的架构不该这样建」时能拿出的最强论据与其证据。§5 的两条真错已修（有失败前置证据），其余条目是判断，不是工单。

方法：只读代码调查（4 条并行线索：核心代数 / GPU 与后端 / 宿主 / 碰撞·网格·仓库治理）+ 一次性 out-of-tree crate 复现（已删）+ 逐条落码核对。凡推断标 `[INFERENCE]`，凡未证实标「未证实」。**文档宣称不当证据**：下文多处列出「文档说的 ≠ 代码做的」。

---

## 0. 立场摘要

cga 不是「用 CGA 做几何引擎」，而是「一个自研几何 + 宿主技术栈，顺带用了 CGA 的表示法」。它的复杂度由**内部抽象自洽**驱动，而不是由消费者、性能预算或平台约束驱动：DAG 干净、文档 161 KB、351 个测试，但唯一外部消费者只用到渲染核心的三个类型。

最有说服力的一条不在意见层，在验收层：**即使按项目自己的闭式验证标准，公开 API 上也存在两条未被测出的真错**（§5，已复现、已修）。

---

## 1. 反证一：CGA 只到「表示法」层，没到「架构」层

| 宣称（README） | 代码事实 |
| --- | --- |
| `op`/`ip` 两个关联谓词是核心能力（README:116） | 生产代码零使用；唯一外部调用在测试 `cga-gpu/src/geometry_extra.rs:847` |
| 32 分量 multivector 是统一表示 | 非测试代码里被显式读写的分量是 **13/32**（0-7、9、10、12、14、31）；其余 19 个（含**全部** grade-3/4）全仓零访问（`values[n]` 在 core 之外 0 命中） |
| 统一代数 | 116 个公开项；其中 **~25/40 个 `Multivector` 方法在 core 之外无生产调用者**（`meet/join/dual/undual/proj/rej/reflect/norm/inverse/grade/op/ip/extract_velocity`…）。`docs/roadmap.md:110` 把 `extract_velocity` 当「现成能力」，实际只有它自己的测试 `motors.rs:413` 调用 |
| 图元即 blade | 9 个图元里 4 个（cone/torus/ellipsoid/cyclide）不是 blade（README:244 自认）；`scale/mirror` 不可能由 versor 表达（README:343）；渲染内核拿到 blade 立刻降级为标量参数（`SphereParams{c,r,axes}`，`to_sphere()/euclidean_vector()/einf_coeff()`） |
| 单一变换机制 | 生产路径用 `Mat4`（`cga-scene/src/object.rs`、渲染/碰撞/质量全走 `world: [f64;16]`）；`motor` 出场后立刻 `to_matrix()`——`cga-host/src/collision.rs` 有 13 处 `motor().to_matrix()` 组合 |

代价（按代码逐条计数的算术推断 `[INFERENCE]`）：

| 操作 | 条目数/flops | 位置 |
| --- | --- | --- |
| `motor.apply` = `reverse` + `2×gp` | **2048 条目**（1024 MAC + 1024 ±1 乘 + 2048 比较 + ~3072 间接取数）+ 每次重算 `reverse` | `multivector.rs:153-169` + `motors.rs:225-227` |
| 4×4 点变换 | **18 flops**，0 分支 0 间接 | `mat4.rs:23-29` |
| `decompose_rigid` + `to_matrix`（= `collision::inv_affine4`，每次点包含一次） | ≈13k MAC（固定 30 次极迭代，**无收敛判据**） | `affine.rs:61-101`、`collision/lib.rs:84-87,122` |

**反方结论**：把 blade 换成 `(params + Mat4)`，只把 `motor` 留在真需要 `exp/log/interpolate` 的运动学层，行为等价——除非有人真用 `op/ip/meet/join`，而生产里没人用。那 GA 就是品牌而不是负载。**GPU 侧不构成反驳**：渲染热点在 GPU，这 114× 只在 CPU 热点（碰撞批处理、运动学求解、质量属性）兑现。

## 2. 反证二：同一几何知识有 4–6 份副本，「零依赖代数核心」是杂烩

| 知识 | 副本数 | 位置 |
| --- | --- | --- |
| 射线–图元求交 | 4 份独立 + MSL 第 5 份 | `geometry_ops.rs`、`geometry_extra.rs`、`fallback.rs`(f64)、`csg.rs`(crossings)、`bvh_trace.rs:41-282`(MSL 字符串) |
| 点包含 | 5 份 | `csg.rs:39-300`、`collision/lib.rs:122-169`、`pairs.rs:36-57`(sdf)、`bake.rs:19-70`(field)、`core/cyclide.rs:145` |
| 环面隐式/四次式 | 6 份 | `geometry_extra.rs:368`、`certify/quartic.rs:25`、`fallback.rs:379`、`bake.rs:57`、`pairs.rs:48`、`lib.rs:156` |
| 圆锥 `k2 = 1+(r/h)²` | 5–6 份 | MSL `bvh_trace.rs:193`、`geometry_extra.rs:145`、`fallback.rs:277`、`csg.rs:223`、`bake.rs:44` + 碰撞另一种写法 |
| 包围盒 | 3 份 | `geom_bounds`、`bake.rs:144`、`world_aabb`（host 再包一层） |
| 求根器 | 4 套 | 区间+i128 精确判别 DK、数组版 DK、`Quartic`、采样+二分×2 |

后果不是「难看」而是**修正不可达**：改一处公式要同步 4–6 个文件，而它们的精度与 guard 常数各不相同（MSL 里手抄 1e-6/1e-9/1e-12；f32 数组 vs f64 认证），测试只覆盖其中一部分（MSL 副本只有「整帧容差 `max_diff ≤ 8`」）。

**阶段 1 已收敛（2026-10-12，`docs/geometry-unification.md`）**：点包含（`field`/`contains`）、
精确欧氏距离/最近点/支撑/包围盒现在只有一份实现（`cga-core::query`，blade 图元锚在 GA 关联谓词上），
网格化与碰撞改为分发，core 里此前**零调用**的 `DupinCyclide::implicit`/`Cylinder::dist`/`plane_dist`/`sphere_dist`
已接线；GPU 的 f32 数组内核与 MSL 保留为性能实现，由逐图元漂移测试钉住（数组 ≡ 标量定义）。
**仍未收敛**：射线求交（数组 f32 / MSL / f64 `fallback` / `certify` 共 4 份）与参数化 seam
（`identity_params` vs `geom_to_camera` 两份 11 变体 switch，对 cylinder/box/circle 给出不同 `q/u/axes`）
——阶段 2/3 的验收条件写在 `docs/geometry-unification.md` §4。

另一边，号称「纯代数边界」的 `cga-core` 实际装着：GPU kernel 参数 ABI（`geometry/identity_params.rs` 100 行，被 4 个 crate 调用）、4×4 矩阵工具（`mat4.rs` 自述是 mesh_io 残留）、场景枚举（`Geometry/CsgGeometry/AffineGeometry`——而真正的场景图在 `cga-scene`，反过来 `pub use cga_core::{Geometry, Multivector}`）。`docs/module-review.md:20` 的「cga-core 纯代数边界无污染」与代码不符。

项目自己认过一次同类问题：`docs/module-review.md:67` 记录「同一公式散在 5 个 crate、14 份定义」，2026-10-12 才收敛 vec/mat（`vec3.rs`）。而第 3 节里的重复是同一模式的**存续**证据。

## 3. 反证三：GPU 路径的设计假设已被自己的补丁推翻

- **「一帧一次内核批处理」是描述性幻觉**：数组路径逐对象发一次求交（`renderer.rs:1166,1193`），阴影再 `每光源×每对象`（`:1287,1303`）；主循环里**逐对象 `eval()+as_slice()` 同步回读**（`:83,93,121,131,998-999`）——GPU 的批量优势被切成 N 段流水线气泡。100 对象/2 光源 ≈ 300 次往返/帧 `[INFERENCE]`。
- **BVH 每帧重建 + 整块重传**（`bvh_trace.rs:517-645`，上传在 `:633-641`），场景不变也传（≈204 KB/帧 @1000 图元 `[INFERENCE]`）；不存在「场景未变就复用」的双帧路径。
- **CSG 逐像素区间采样**把中间张量撑到 ≈483 MB f32（`geom_kernels.rs:452-457`；对应 README:158 的 460 ms）。
- **精度被迫双轨**：f32 内核 + `certify`/`fallback` 的第三份 f64 标量实现（`fallback.rs:1-8` 自述：判别式落在相消带内的光线改由 CPU f64 重判）。
- **平台**：`mlx-rs` 非 optional、无 feature、无 CPU 渲染回退；`cga-host`(`Cargo.toml:11`) 与 `cga-examples`(`:13`) 无条件依赖 cga-gpu → 锁定面比 `docs/cross-platform.md` 声称的「唯一锁定点 = cga-gpu 13 文件」大一层（宿主与全部 CLI 也锁）；该文档还写着「**没有自定义 Metal kernel**」，而 `bvh_trace.rs:37-372` 有两条 JIT MSL kernel、`cga-fastmetal` 直接调 mlx-c `fast.metal_kernel_*`。文档的 stage 1/2/3（`GpuOps` trait / `cpu-backend` feature / wgpu-CubeCL 后端）在仓库里**零落地**（全仓 `GpuOps` 零命中、无任何 `[features]`）。

对象数、精度、平台三个方向都被现实打穿，补丁全加在架构边缘（BVH kernel、f64 认证、手写 MSL）而不是重画架构——这正是「设计假设错了」的指纹，而不是「还没优化」。

## 4. 反证四：宿主层是自造浏览器子集，且没有消费者

`cga-host` 15 220 行（最大 crate）里，属于本领域（CGA 场景）的只有 builder/scene_build/kinematics 的几何部分；其余是在重造别人已有的东西：

| 自造 | 规模 | 仓库里已有的替代 |
| --- | --- | --- |
| CSS 选择器解析 + 匹配 + 级联 + 继承 + 值转换 | ≈885 行（`jsx/css.rs:155-486,580-760` + `builder.rs:226-350`） | lightningcss 只被用作词法/颜色归一化（14 处调用） |
| `.jsx` 模块图 DFS + 循环检测 + 命名空间包装 + 别名重写 | ≈250 行（`jsx/compile.rs:154-400`） | 仓库里已经在用 esbuild（`scripts/vendor-react.mjs:44-57`） |
| 「结构化参数 → JSX 文本」生成器（再喂 swc→boa→JSON 回到 Rust） | 469 行（`jsx_gen.rs` + 4 个 spec） | 直接构造 `SceneRun` 即可省掉两次序列化 |

接口契约也在漂：`HostCall` 21 变体（`react.rs:296-337`）↔ JS `HANDLERS` 19 项（`react-host.js:693-712`），且已有**死面**（JS `dispatch` 无 Rust 发送方）；`docs/jsx-css-host.md` §8 说「新增宿主方法两端各加一处」，实际 3–4 处（Rust 变体 + `to_json` + JS handler + JS session），加一个 tag 要改 5 处/2 种语言；元素词汇表 3 份拷贝（`scene-prelude.js` ↔ `builder.rs` ↔ `session.rs`）且无跨语言一致性测试（`sig_names` 还漏了 `union`）。

消费者面（这是反方最省力的一击）：

- 唯一外部消费者是 `/Users/aki/code/r3d`（`Cargo.toml:74-76` path 依赖 `cga-core/cga-gpu/cga-scene`），只用 `Renderer::new` / `Scene` / `PerspectiveCamera`——**不消费 cga-host / cga-collision / cga-mesh**。
- `cga-collision` 与 `cga-mesh` 在仓库内各只有 `cga-host` 一个消费者。
- README:250 声明的「library-level entry for programmatic callers (LLM codegen, external GUIs)」在仓库内无对应实体，而且**紧接着的示例编译不过**：示例调 `cga_gpu::gen_flange_assembly` / `render_jsx_png` / `run_jsx`，而 `crates/cga-gpu/Cargo.toml:6-13` 不含 cga-host，这三个符号在 `cga-gpu/src` 零命中（实测）。

## 5. 反证五：验证文化在架构接缝上漏了真错（已复现、已修）

用一次性 out-of-tree crate 调公开 API `mass_properties`（`cga-mesh`）：

| 用例 | 独立解析期望 | 修复前实测 | 倍数 |
| --- | --- | --- | --- |
| 球 r=1、ρ=1 的 `inertia[0][0]` | `2/5·m·r² = 1.6755` | **7.0184** | ×4.1888 = **体积 v** |
| 网格路径（仿射包装同一球） | 1.6755 | **6.9941** | ×4.174 ≈ v |
| 既有唯一绝对量级惯性测试用的盒（`BoxGeometry::new(4,0.5,0.5)`） | v=1 | 正确 | 因子恰为 v=1 → **测不出** |
| 仿射缩放锥（`scale=2`）的 `cog.z` | **−1.0** | **−2.0076** | 线性部分被作用两次 |

两条根因都在接缝上，而不是在某个函数内部：

1. **量纲**：`to_world`（`mass.rs:107-139`）用 `m = density·v` 去乘**已经含 v 的单位密度二阶矩**（`closed_form` 注释自己写着「单位密度：I = 2/5 m r²」）⇒ 全仓 `inertia` 偏大 v 倍。
2. **契约**：`mass.rs:81` 明写「`world` 必须是刚体的（发射路径已经分解过）」，而唯一调用方 `builder.rs:378-389` 把**完整 `world`** 传给已经折进 `lin` 的 `AffineGeometry` ⇒ `cog`/`inertia` 被线性部分作用两次。

为什么 351 个绿测试留不住：`cross_check`（`mass.rs:238-262`）两侧都是**单位密度**约定；唯一的绝对量级惯性测试体积恰好 = 1；`MCP scene_mass` 与报告只输出 `mass/cog`；无 CI。受影响面是真实的：`cog` 进 `scene_report`、MCP、以及 C2 静力学（`U = −Σ m·g·cog`）。

**修复（2026-10-12）**：`to_world` 只乘密度；`builder.rs` 在 `lin ≠ I` 时把 `motor.to_matrix()` 作为质量属性的 world。回归测试（失败前置均有记录）：

| 测试 | 拦什么 |
| --- | --- |
| `mass::tests::inertia_scales_with_density_not_with_volume` | 体积 ≠ 1 的闭式量纲（球 r=1 报 7.0184、盒 ρ=3 报 128） |
| `mass::tests::mesh_path_inertia_shares_the_closed_form_convention` | 网格路径同一约定（旧值 6.994 vs 期望 1.673） |
| `mass::tests::world_rotation_rotates_inertia`（**加强**：盒由体积 1 改为 4） | 旋转路径的量纲（旧值 I_xy −10 vs −2.5） |
| `jsx::tests::test_mass_scaled_object_uses_rigid_world_only` | 端到端缩放锥 `cog.z`（旧值 −2.0076 vs −1.0） |

## 6. 治理：声称与代码脱钩

| 声称 | 实测 |
| --- | --- |
| README:94 教 CI 用 `CGA_REQUIRE_GPU=1` | 仓库**没有 CI 配置**（`.github/**` 不存在），唯一自动化是 `Makefile`（test/run/fmt/vendor-react，靠人手动跑） |
| README:157「30–60×」（BVH） | 唯一计时脚本 `bench_ray.rs` **不切 BVH**（开关 `CGA_NO_BVH/CGA_BVH_FORCE` 只在 `bvh_trace.rs:712-720` 被读）→ 数字要人工设环境变量才能复现，无脚本无断言无 bench crate |
| README:412「goldens have RMSE = 0」 | 金样是 PNG `fnv1a` 哈希且只覆盖 **3/8** 场景（`jsx/tests.rs:99-118`）；全仓**没有任何算 RMSE 的代码** |
| 测试总数 | 修正前：README/`module-review.md` 348、`test-design-audit.md:4` 347、`css-conformance.md:4,321` 189、"cga-host 142" —— 四处互斥 |
| `docs/collision-plan.md:5` | 描述的是**已被删除**的依赖边（`cga-gpu::scene` → 现为 `cga_scene`，`module-review.md:57` 记录过拆分但该文档没同步） |
| `crates/cga-core/gen_tables.py` | 「生成文件」的输入（`table_source`）**不在仓库里**（脚本自述 kept for provenance），且签入文件已被 rustfmt 重排 → 表不可重放 |
| mesh 的「认证水密」 | `is_watertight()`/`topology_report()` **只在测试里被调用**（`bake.rs:878-883`）；导出/质量/宽相三条生产路径都不验；质量步长只是包围盒最大边/48（`mass.rs:136-147`），无收敛/误差界 |
| 碰撞的「宽相」 | 宽相在 host（`collision.rs:51` 阈值 64），且**不削减输出**：测试自己断言 `hits.len() == n(n-1)/2`（`collision.rs:1054-1059`）→ O(n²) 时间 + O(n²) 分配 + 每帧 `Debug` 字符串指纹（`collision.rs:73-79`） |
| 窄相覆盖率 | 14/28 图元对（`pairs.rs:14-31`），**所有自配对**（柱–柱、锥–锥、环面–环面、椭球–椭球）都缺；真实机械场景里最常见的柱–柱、柱–盒恰好是缺口 |

## 7. 反方的替代设计（若目标是产品而不是研究）

1. **一个 primitive 真源**：`trait Primitive { intersect / contains / bounds / normal / support / field }`，后端三份（f64 标量参考、SIMD/CPU、GPU）由同一源生成 MSL/表 → 直接消灭 §2 的 4–6 份副本。
2. **场景表示去 blade**：`params + Mat4`；`motor` 留在运动学/插值。
3. **渲染后端 trait 化**：把 `cross-platform.md` 的阶段 1 真正做掉（今日零落地），先有可跑的 CPU 参考后端，再谈性能后端。
4. **宿主层收缩**：只留 `SceneRun` IR + 生成器；JSX/CSS 交给外部 JS 栈经 IR 交接，等第二个消费者出现再长。
5. **碰撞**：复用同一 primitive 表（sdf/支撑），输出**稀疏**（只发 Yes/Unknown），去掉每帧字符串指纹与全矩阵发射；空间索引用渲染侧已有的 BVH 思路而不是另一套 AABB 网格。
6. **治理**：CI（含 `CGA_REQUIRE_GPU=1` 的 Linux/macOS 两档）+ bench 脚本把 README 的性能数字变成产物 + 金样扩容 + 让测试总数只有一个版本。

## 8. 反方站不住的地方（诚实边界）

- 三值 `Unknown`、闭式验证、逐位一致（增量/DOF/opaque）是真优点，不是架构缺陷；本文的指控是「**复杂度分配错了**」，不是「工程不行」。
- 114× 的代数量级只在 CPU 热点兑现；渲染热点在 GPU，所以 GA 的 CPU 开销在渲染路径上不构成瓶颈（但在碰撞/运动学/质量属性的 f64 热点上是实测量级）。
- 一次反例：曾有一条「生成表里 e0/einf 行有填充项、必须靠配对项抵消」的指控——解码 `gp_j/gp_dst/gp_sign` 后**抵消对为 0**，那是退化度量（`e0·einf = −1`，乘积两分量）的合法编码。这类「看着致命」的指控必须逐条落码，否则反方自己先失信。
- 本文没有跑渲染/金标对比，也没跑 JSX 端到端复现 §5 的旧值（会拉 MLX 构建）；§5 的证据是库级实测 + 调用点读证 + 修复后的端到端回归测试。

## 9. 结论与优先级的反方排序

最强的反面立场不是「选错了代数」，而是：**这个仓库缺少外部锚点（消费者、性能预算、平台、CI），于是复杂度向内部生长——N 份重复实现、自造子系统、文档漂移；而它最擅长的验证恰好停在架构接缝之外**（§5 就是接缝上的漏网错，20 行一次性代码即可抓到）。

反方排序：① §5（真错，已完成）→ ② §2（副本收敛到单一 primitive 表）→ ③ §3/§4（后端 trait 化、宿主收缩）→ ④ §6（治理：CI/bench/金样/文档同步）。

**开放项（本文不改，留给立项）**：README:412 的 RMSE=0 表述与 `artifacts/tests/` 的实际内容不符（且 artifacts 被 gitignore，无法在 CI 里核对）；`docs/collision-plan.md:5` 的过时依赖图；`gen_tables.py` 的输入不在库；窄相 14/28 的缺口清单是否需要按真实用例排序。
