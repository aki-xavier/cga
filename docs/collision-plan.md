<!-- markdownlint-configure-file {"MD013": false} -->
# 碰撞检测：定位与开发计划

状态：**C0 + C1 已实施（2026-10-07）**，C2–C4 计划。适用：`cga-core/src/collision/`、`cga-host/src/collision.rs`（场景扫描）、`cga-host`（JSX 查询、报告、运动学闭环）。

## 0. 结论与定位

**应该在本项目实现，但范围严格限定在"检测与查询"**：重叠判定、分离距离、接触信息（点/法向/分离度）、螺旋扫掠 TOI（连续碰撞检测）、关节行程的干涉扫描。

**不做动力学**（力/冲量/约束求解器/时间步进）：那是另一个数量级的项目，与"确定性几何内核"的定位不同，需要时另行立项。**不做网格碰撞**（项目红线：场景无网格，网格只出现在导出）。

理由：

1. **同源**：渲染 = 光线 vs 场景的碰撞检测；碰撞 = 物体 vs 物体。`crossings`/`contains` 协议、degenerate 库、certify 认证求根全部直接复用。
2. **运动学闭环**：关节（P1）/ 齿轮（P2）/ 凸轮接触（P3）齐了；没有碰撞检测，机械场景只能摆 pose，不能验证运动合法性。URDF 导出在 → 机器人干涉检查是标准需求（motion planning 的前提）。
3. **差异化**：解析实体 + CGA 螺旋运动 + 认证数值。网格引擎（GJK/EPA）给近似；本项目给确切答案，或明确的"不知道"。
4. `dist()` 查询在 `docs/jsx-css-host.md` v1 边界里显式推迟、"需要时单独立项"——本计划就是它的立项。

## 1. 现状证据（可复用的件）

| 件 | 位置 | 给碰撞检测什么 |
| --- | --- | --- |
| `geom_crossings` / `geom_contains` | `cga-gpu/src/csg.rs` | 实体成员关系协议（CSG 布尔就是靠它） |
| degenerate 库 + 区间分类 + 认证求根 | `cga-gpu/src/degenerate.rs`、`certify/` | 相切/退化接触的认证路径（`QuarticRoots::is_unknown` 是三值先例） |
| `is_convex_inner` | `cga-gpu/src/geom_kernels.rs` | 凸性分类：凸对走精确路径，非凸走保守路径 |
| `cam_solve` + `separation_branches` | `cga-host/src/scene_build/kinematics.rs` | 接触求解原型：分离函数 + 采样 bracket + 二分至 1e-12 + 唯一性认证 |
| motor `exp` / `log` / `velocity` / `extract_velocity` | `cga-core/src/motors.rs` | 螺旋运动：CCD 的**精确**轨迹（不是线性插值近似） |
| `geom_bounds` / `aabb_sphere` / 屏幕剔除 | `cga-gpu/src/geometry_ops.rs`、`renderer.rs` | broad phase 的包围盒/包围球 |
| 帧间指纹 diff / 分组 | `IncrementalRenderer`、`Object::group` | 碰撞对缓存的失效依据：指纹没变 ⇒ 碰撞结果不用重算 |
| tag 注册表 + `face()/center()` 查询 | `cga-host/src/jsx/mod.rs`、`scene_build` | JSX 侧按名字引用对象的方式（`collides("gripper", "part")`） |
| `solve` 宿主原生函数 | `react-host.js` / `jsx/mod.rs` | JSX → Rust 标量查询的既有模式，`collides()` 照抄 |

## 2. 设计决策

- **D1 只做检测**：动力学（力/冲量/约束求解/时间步进）不在本计划内。
- **D2 三值结果**：`Hit::Yes / No / Unknown`。CSG 差/交的重叠判定不装精确：保守规则给不出确切答案时返回 `Unknown`，调用方自行降级（当作 Yes 报警 / 当作 No 放行）——显式三值，不许静默。
- **D3 窄相在 `cga-core`**：纯 CPU f64 标量数学，不绑 MLX（碰撞查询是逐对的控制流密集代码，不是渲染那种逐光线数据流；GPU 批化以后有需求再说）。
- **D4 CCD 用螺旋运动精确轨迹**：`q(t) = exp(t·ξ)·q₀`，TOI 用 cam_solve 同款「采样 bracket → 唯一性认证 → 二分」；不做线性插值近似的扫掠。
- **D5 JSX 落点**：`collides(ofA, ofB)` / `clearance(ofA, ofB)` 宿主查询；结果进场景报告。v1 不加新元素类型。
  - C0 实施修正：`solve()` 是**模块求值期**的纯数学宿主函数，那时场景还没建成；碰撞查询需要建成后的场景，必须走 `{__q}` **惰性查询**通道（与 `face()/center()` 同款，构建期解析），不能模仿 `solve()`。JSX 面因此整体移到 C1。
- **D6 穿透深度/MTV 只对凸对**（球/盒/柱/锥/椭球及其仿射包装，`is_convex_inner` 同款分类）；非凸/CSG → `Unknown`。
- **D7 broad phase = 包围盒 + 指纹缓存**：复用 `geom_bounds` 剪枝与增量渲染的指纹失效；对象数量级（几十到几百）不需要空间索引，需要时再立项。

## 3. API 草案

Rust（`cga-core/src/collision/`，新模块）：

```rust
/// 三值判定（D2）：确切命中 / 确切不命中 / 认证路径也给不出确切答案。
pub enum Hit { Yes, No, Unknown }

/// 一个接触：点、法向（从 b 指向 a）、带符号分离度（< 0 = 穿入）。
pub struct Contact { pub point: [f64; 3], pub normal: [f64; 3], pub separation: f64 }

/// 两个实体（几何 + 世界变换）是否重叠。
pub fn overlap(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> Hit;
/// 分离距离（最近点对距离；穿入时 < 0）。None = Unknown（非凸/CSG 无确切解）。
pub fn separation(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> Option<f64>;
/// 接触信息（接触/穿入时非空）。Unknown 时为 None。
pub fn contacts(a: &Geometry, wa: [f64; 16], b: &Geometry, wb: [f64; 16]) -> Option<Vec<Contact>>;
/// 螺旋扫掠 TOI：a 以螺旋速度 xi 从 wa 出发，t ∈ [0, t_max] 内首次接触 b 的时刻。
pub fn sweep_toi(xi: [f64; 6], a: &Geometry, wa: [f64; 16],
                 b: &Geometry, wb: [f64; 16], t_max: f64) -> (Hit, Option<f64>);
```

JSX（跟随 `solve()` 模式，构建期求值，可进 props）：

```jsx
const hit = collides("gripper", "part");      // → true / false / null（Unknown）
const gap = clearance("gripper", "part");     // → 分离度或 null
```

场景报告：接触/干涉表（对象对、分离度、接触点、Unknown 单列）。

## 4. 阶段

每阶段独立可验收、可提交；全部保守——拿不准就 `Unknown` / 全量。

### C0 · 点与距离原语

**已实施（2026-10-07）**：`cga-core/src/collision/`（`Hit` 三值、`contains_point`、
`separation`、`overlap`、`world_aabb`；纯 CPU f64）。已交付：

- `contains_point`：全图元（球/平面半空间/盒/柱含无限长/锥/环面/椭球）+ CSG
  三值递归（union/intersection/difference 的 and/or/not）+ 任意仿射精确
  （点逆变换，不要求刚体）。圆片/环纹面/部分环面 → `Unknown`。
- `separation` 两两表（世界空间形状，仅刚体含镜像；非刚体仿射 → `Unknown`）：
  球–{球/平面/盒/柱/锥/环面/椭球(球心在外)}、平面–{平面/盒/柱/锥/环面/椭球}、
  盒–盒（SAT 15 轴 + 最近特征对精确距离 + 穿透轴深度）。锥 SDF 走截面三角形
  的 2D 点–边距离（精确）；椭球外部点走单调函数对分（内部点 v1 `Unknown`）。
- `overlap`：分离距离优先；算不了时世界包围盒保守判定（相离 ⇒ `No`，否则
  `Unknown`）。
- 验收：15 个测试（`cga-core::collision::tests`）——pairwise 矩阵每对 ≥ 3 构型
  （分离/相切/穿入，解析距离断言）、CSG 三值、Unknown 构型必须 Unknown、
  对称性、overlap 与 separation 一致性、包围盒保守路径。
- 与计划的偏差：JSX `inside()` 移到 C1（见 D5 修正）。

<details><summary>C0 原始计划条文（已按上表交付，留档）</summary>

- 点级查询：`contains(geo, world, point)`（`geom_contains` 的 CPU 标量版）→ JSX `inside(of, p)`。
- 凸对分离距离表：球-球、球-平面、球-盒、球-柱、盒-盒（OBB SAT）、球-锥、柱-柱、球-椭球；仿射包装一律先变到局部。柱-锥-环面等非平凡对用「采样 + Newton 最近点 + 认证回退」，认证不了就 `Unknown`。
- 验收：pairwise 矩阵单测（每对 ≥ 3 构型：分离/接触/穿入，距离有解析解）；性质测试（对称性、三角不等式抽查）。

</details>

### C1 · 重叠判定 + broad phase + JSX helper

**已实施（2026-10-07）**。已交付：

- `overlap()`/`probe()` 判定阶梯：包围盒相离 ⇒ `No` → 两两分离距离表 →
  CSG union 递归分解（三值 or；`separation` 同步分解为子件最小值）→
  差/交 ⇒ `Unknown`。
- `cga-host/src/collision.rs`：`CollisionScan` 全对扫描——同组（`group` 非 0
  且相等）免检跳过；指纹（几何 + 世界变换）缓存跨帧复用（`cache_hits` /
  `computed` 统计）。`SceneSession::collisions()` 持持久扫描器。
- JSX 惰性标量查询（构建期解析，`{__q}` 通道）：`clearance(a,b)` → 分离距离、
  `collides(a,b)` → 1/0、`inside(of, p)` → 1/0；`qadd/qsub/qmul/qdiv` 组合
  （JS 算术对查询对象是字符串拼接——不透明对象不能加）。**Unknown 一律构建
  报错**（三值不许变成数字）。tag 引用先定义后使用（与 `instances`/`when` 同）。
  落点：图元的数值参数（`build_geo`）与 `<rotate angle>`（`p_num_lazy`）。
  注：`build_geo` 此前把 `{__q}` 对象静默成 0——现在要么给真值要么报错。
- 报告：`collide <i> <j> yes|unknown sep=<d>` 行（只列非 No 的对，没有则整节省略，
  既有金标不受影响）。
- 验收：`overlap_union_decomposition`（union 分解 + 三值传播 + 包围盒提前 No）、
  `scan_pairs_and_group_skip`、`scan_cache_reuse_across_scans`（动一个对象只重算
  含它的对）、`test_jsx_collision_queries`（clearance/collides/inside 全解析正确）、
  `test_jsx_collision_query_unknown_errors`（环面–环面 → Unknown 报错）、
  `session_collisions_cache_across_frames`（跨帧缓存）、`test_report_contacts`、
  `gallery_collision_scan_baseline`（8 场景 Yes/Unknown 计数基线 +
  animation 太阳球陷入地面 0.05 的语义抽查——检测器抓到的真实穿入）。

### C2 · 接触信息

- `contacts()`：凸对的最近点对 + 法向；平面接触给接触面中心；多点接触（盒-盒面面）给凸包角点。
- 接触标记可视化（小marker球/法向短线，走普通对象通道）+ 渲染 golden。
- 验收：接触点坐标解析断言；golden PNG。

### C3 · 螺旋 CCD

- `sweep_toi()`：螺旋轨迹上的分离函数符号 bracket + 唯一性认证 + 二分（cam_solve 同款）；运动中的物体对场景中每个静止物逐一求 TOI，取最小。
- 验收：旋转臂 vs 立柱——接触角有闭式解，对比；`cam_solve` 的凸轮接触用 `sweep_toi`/`contacts` 复现（交叉验证）。

### C4 · 运动学闭环与文档

- 关节行程干涉扫描：`q ∈ limit` 区间上扫 `sweep_toi`/分离度，报告"行程内何处干涉"。
- 文档：本文件状态改"已实施"，§5 勾选；`jsx-css-host.md` v1 边界移除 `dist()`；README 特性表加碰撞检测行。
- 验收：法兰/机械臂示例场景行程扫描输出干涉区间，与解析预期一致。

## 5. 测试计划

| 层 | 内容 |
| --- | --- |
| 单元 | pairwise 矩阵（每对分离/接触/穿入三构型 + 解析距离）；三值规则的 Unknown 构型 |
| 性质 | `overlap(A,B) == overlap(B,A)`；`Miss ⇔ separation > 0`；对称接触法向反向 |
| 认证 | 相切/退化构型走 degenerate/certify 路径，不许 panic 不许静默错 |
| 一致 | 接触标记渲染 golden；`cam_solve` 接触用 `contacts()` 复现 |
| 端到端 | JSX helper 在 `SceneSession` 多帧下正确失效（指纹变了才重算）；行程扫描已知结果 |
| 回归 | 现有 200 测试 + 3 金标逐位不变；`cargo fmt --check`；clippy 无新增警告 |

## 6. 风险

| 风险 | 对策 |
| --- | --- |
| CSG 精确重叠是研究坑（差/交的曲面交线） | D2 三值逃生门：v1 只承诺凸对与 union 分解的确切性 |
| 非凸穿透深度无定义良好的解析解 | D6：只承诺凸对 MTV，其余 `Unknown` |
| O(n²) 对象对 | D7：包围盒剪枝 + 指纹缓存；对象数上量后再立空间索引项 |
| 相切/退化是数值重灾区 | degenerate/certify 路径从 C0 就接入，不后补 |
| 范围蠕变（"顺便做个动力学吧"） | D1 写在最上面；动力学需要时另行立项 |
