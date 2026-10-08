<!-- markdownlint-configure-file {"MD013": false} -->
# 碰撞检测：定位与开发计划

状态：**C0–C4 全部已实施（2026-10-07）**；同日窄相拆为独立 crate。适用：`crates/cga-collision/`（窄相：判定/分离/接触/螺旋 CCD，纯 f64 CPU，只依赖 `cga-core`）、`cga-host/src/collision.rs`（场景扫描/标记/关节扫描，依赖 `cga-gpu` 的 `Scene`/`Object`）、`cga-host`（JSX 查询、报告、运动学闭环）。

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
- **D3 窄相是纯 CPU f64 标量数学**，不绑 MLX（碰撞查询是逐对的控制流密集代码，不是渲染那种逐光线数据流；GPU 批化以后有需求再说）。
  - C4 后修正（2026-10-07，用户拍板）：窄相拆为**独立 crate `cga-collision`**（只依赖 `cga-core`，全部走 pub API，`cga-core` 零改动）；场景扫描/标记/关节扫描留在 `cga-host`（依赖 `cga-gpu` 的 `Scene`/`Object`，进纯 crate 会把 MLX 拖进依赖图）。
- **D4 CCD 用螺旋运动精确轨迹**：`q(t) = exp(t·ξ)·q₀`，TOI 用 cam_solve 同款「采样 bracket → 唯一性认证 → 二分」；不做线性插值近似的扫掠。
- **D5 JSX 落点**：`collides(ofA, ofB)` / `clearance(ofA, ofB)` 宿主查询；结果进场景报告。v1 不加新元素类型。
  - C0 实施修正：`solve()` 是**模块求值期**的纯数学宿主函数，那时场景还没建成；碰撞查询需要建成后的场景，必须走 `{__q}` **惰性查询**通道（与 `face()/center()` 同款，构建期解析），不能模仿 `solve()`。JSX 面因此整体移到 C1。
- **D6 穿透深度/MTV 只对凸对**（球/盒/柱/锥/椭球及其仿射包装，`is_convex_inner` 同款分类）；非凸/CSG → `Unknown`。
- **D7 broad phase = 包围盒 + 指纹缓存**：复用 `geom_bounds` 剪枝与增量渲染的指纹失效；对象数量级（几十到几百）不需要空间索引，需要时再立项。

## 3. API 草案

Rust（`crates/cga-collision/`，独立 crate）：

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

**已实施（2026-10-07）**：`crates/cga-collision/`（`Hit` 三值、`contains_point`、
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
- 验收：15 个测试（`cga-collision 的 tests`）——pairwise 矩阵每对 ≥ 3 构型
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
  落点：图元的数值参数（`build_geo`，构造时把 `{__q}` 解析成数值）。变换 prop
  里 `t`/`scale`/`mirror` 走 `p_vec3_lazy`（向量位的 `vadd`/`face` 等），`rotate`
  是四元列表走 `p_num_list`——**只吃普通数值**，标量查询不落在变换上。
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

**已实施（2026-10-07）**。已交付：

- `cga_core::collision::contacts(a, wa, b, wb) -> Option<Vec<Contact>>`：域与
  `separation` 一致（`None` = Unknown）。`Contact { point, normal, separation }`：
  normal 是 **b 的分离方向**（穿入时平移 b 即可分开；相离时从 a 最近点指向 b
  最近点）；接触点取两侧最近表面点的中点。
- 各形状的最近点/支撑点：球/平面/盒/柱（含无限长）/锥（截面三角形最近点映回
  3D）/环面（截面圆）/椭球（外部对分；内部 → Unknown）。平面–X 给最深支撑点；
  平面–平面 v1 不给（接触是条线）。
- 盒–盒接触流形：A 内顶点 + B 内顶点 + 边×面交点，1e-9 去重；面–面穿入给
  交区矩形的 8 角点（有解析断言）。
- `CollisionScan::scan_contacts`（接触解只跑在 Yes 对上）；`contact_markers`
  （红色发光小球 + 法向黄色细柱，普通对象通道直接渲染）。
- 验收：接触点坐标解析断言（球-球中点、球-面穿入深度中点、盒落地支撑点、
  盒-盒 8 角点流形）、法向互换取反的对称性、Unknown 域与 separation 一致、
  接触标记渲染金标（fnv1a 钉死）。

<details><summary>C2 原始计划条文（已按上表交付，留档）</summary>

- `contacts()`：凸对的最近点对 + 法向；平面接触给接触面中心；多点接触（盒-盒面面）给凸包角点。
- 接触标记可视化（小marker球/法向短线，走普通对象通道）+ 渲染 golden。
- 验收：接触点坐标解析断言；golden PNG。

</details>

### C3 · 螺旋 CCD

**已实施（2026-10-07）**。已交付：

- `cga_core::collision::sweep_toi(xi, a, wa, b, wb, t_max)`：`xi = [ω; v]` 物理
  螺旋速度（注意 `Multivector::velocity` 存二重矢量系数，内部喂一半），
  轨迹 `M(t) = exp(t·ξ)·wa` 精确螺旋（非线性插值近似）。
- 认证（不报假阴性/假阳性）：`f(t)` = 分离距离是 1-Lipschitz 的，
  `|f'| ≤ vmax`（vmax = 螺旋最大点速度上界 = 轴向节距速度 + |ω|·包围盒角点到
  螺旋轴的最大距离，沿轨迹不变）。区间下界 > 0 ⇒ 认证无接触；符号翻转 ⇒
  对分求根；8 分细分到深度上限仍无法认证（擦边退化）⇒ `Unknown`；几何对
  不支持 ⇒ `Unknown`；初始已接触 ⇒ `(Yes, Some(0.0))`。
- 场景版 `sweep_scene(xi, i, scene, t_max) -> SweepOutcome { first, unknown }`
  （同组跳过；`unknown` 列出无法判定的对象——它们可能在 first 之前接触）。
- 验收（全部闭式/解析断言，1e-9）：旋转臂 vs 立柱（接触角闭式解）+
  接触时刻分离度自洽（≈0 且稍前 > 0）、认证无接触（t_max 截断）、初始穿入、
  纯平移、球–盒、**凸轮交叉验证**（cam_solve 的 circle–circle 分支 = 球心共面
  时的球–球：`cos q* = −0.875` 闭式解逐位吻合）、环面–环面 Unknown、
  `sweep_scene` 最早接触与同组跳过。
- 注意（已写进模块文档）：擦边（grazing）接触在认证框架下是 `Unknown`——
  这是诚实的边界情形，不是漏报。

<details><summary>C3 原始计划条文（已按上表交付，留档）</summary>

- `sweep_toi()`：螺旋轨迹上的分离函数符号 bracket + 唯一性认证 + 二分（cam_solve 同款）；运动中的物体对场景中每个静止物逐一求 TOI，取最小。
- 验收：旋转臂 vs 立柱——接触角有闭式解，对比；`cam_solve` 的凸轮接触用 `sweep_toi`/`contacts` 复现（交叉验证）。

</details>

### C4 · 运动学闭环与文档

**已实施（2026-10-07）**。已交付：

- `sweep_joint(run, joint_name) -> JointSweepOutcome`：1-DOF 关节（revolute /
  continuous / prismatic / helical）的 q 从行程低端扫到高端（continuous 扫
  一整圈）。关节 frame 的世界矩阵 `F = world·M(q)⁻¹`，螺旋轴过 F 原点、
  方向 `F·axis`；helical 加节距平移，prismatic 纯平移。各 mesh 先退到
  `q=lo`（`F·M(lo)·M(q)⁻¹·F⁻¹` 共轭），再走 `sweep_toi` 的认证螺旋扫掠。
- 嵌套闭包：扫父关节时全部后代关节的 meshes 刚体随动（`parent` 链 BFS）；
  运动集合内部不互相误报；同组免检。
- v1 边界：其余关节**冻结**（齿轮/凸轮联动扫描另行立项）；多 DOF / fixed /
  无 limit → `skipped` 注明原因。
- 验收：闭式解（单关节臂撞立柱 `q* = β − acos((ρ²+d²−s²)/(2ρd))`，1e-9）、
  嵌套闭包（base/elbow 各扫各的，各自的闭式接触角都吻合）、跳过路径、
  无干涉干净返回。
- 文档：本文件状态改"全部已实施"；`jsx-css-host.md` v1 边界已在 C1 移除
  `dist()`；README 特性表与覆盖率已含碰撞检测。

<details><summary>C4 原始计划条文（已按上表交付，留档）</summary>

- 关节行程干涉扫描：`q ∈ limit` 区间上扫 `sweep_toi`/分离度，报告"行程内何处干涉"。
- 文档：本文件状态改"已实施"，§5 勾选；`jsx-css-host.md` v1 边界移除 `dist()`；README 特性表加碰撞检测行。
- 验收：法兰/机械臂示例场景行程扫描输出干涉区间，与解析预期一致。

</details>

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

## 7. 第一性原理正确性审计（2026-10-07）

起因：不能用「与渲染器一致」当正确性证据——渲染器可能也错。以下每条公式从
定义推导，与任何既有实现无关。

**定义**：`separation(A,B)` 相离时 = `min_{a∈∂A,b∈∂B}|a−b|`；穿入时的理想
语义 = 最小平移量（MTV）取负。

**从定义可证精确的**：

- 球–球：对称性 ⇒ 最近点在两心连线（穿入时也是精确 MTV）。
- 球–X：`dist(ball(c,r), X) = dist(c,X) − r` 对**任意**集合成立（球是 c 的
  r-邻域）——环面非凸也成立。前提是 X 的 SDF 精确。
- 盒/带帽柱/锥/环面 SDF：盒是精确欧氏；带帽柱的棱缘由截面的 `max` 组合覆盖；
  锥/环面用旋转体子午面归约（最近点与查询点同方位角）。锥 SDF 的截面三角形
  **不含轴边**（轴边是实体内部，不是边界）——审计时验证过这个细节。
- 椭球外部：Eberly 方程单调性可证 ⇒ 对分收敛到真根；内部报 Unknown（诚实）。
- 平面–凸体：半空间的 MTV 必沿法向，`min(−lo, hi)` 精确。
- 盒–盒相离：凸多面体最近特征必为顶点–面/边–边；顶点–顶点对由边–边端点
  覆盖（审计时推了覆盖性证明）。
- CSG contains 三值：De Morgan 三值逻辑的保守方向可证健全。
- AABB：union 取包 / intersection 取交 / difference 取左，对「相离 ⇒ No」
  方向可证健全。
- 扫掠认证：点到集合距离是 1-Lipschitz ⇒ 复合螺旋速度界合法；union-min 与
  SAT-min 都是同常数 Lipschitz 函数的 min，界仍然成立。

**语义边界/近似（诚实标注，不是错误）**：

- ~~盒–盒穿入深度~~：**已实现精确 MTV**（Minkowski 差 = 6 生成元 zonotope，
  原点到边界的最近点；面枚举 + 2D zonotope 多边形链构造）。
  强对拍：沿 MTV 方向的单向分离平移恰等于 |MTV|（1e-9 解析断言）；
  5000 个伪随机深穿透构型上 SAT 与精确值差异 ≤ 1e-15——审计的担心在
  盒–盒上未兑现（zonotope 面法向全在 SAT 轴集内），但现在是认证实现，
  不靠经验。盒–盒接触法向同步改用精确 MTV 方向。
- **平面–平面相交返回 0**：两个相交半空间重叠体积无限，MTV 无定义；返回 0
  是表面距离语义（表面相交 = 接触），不是穿入深度。
- **盒–盒接触法向**：同取 SAT 最小轴——深穿透时是 MTV 方向的近似。
- **平面是半空间实体**：与渲染器的「无厚度曲面」刻意分叉（球在地面之下 =
  穿入），是碰撞侧的正确建模选择。
- **椭球内部点**、**擦边（grazing）扫掠**：报 Unknown，不装精确。

**审计结论**：布尔接触/重叠判定、相离距离、穿入深度全部从定义可证精确
（盒–盒穿入在审计后升级为 Minkowski 差 zonotope 精确 MTV；平面–平面相交
的 0 是表面距离语义——两半空间 MTV 无定义）。

## 8. D1+D2 实施记录（2026-10-08）

- **D1 接触解帧间缓存**：`CollisionScan.contact_cache` 与 probe 同一套指纹键
  （几何+世界变换的 Debug 散列）；第二帧起 Yes 对的接触解零重算。
  测试：两次 `scan_contacts` 逐位一致、缓存命中计数、动一个对象只重算相关对。
- **D2 齿轮联动行程扫描**（`sweep_joint` 扩展）：
  - 联动集合 = 从被扫 pair 出发沿 gear 关系 BFS（`dq_b = ratio·dq_a`，反向除
    ratio；ratio=0 的从动端锁在 offset 不动，被扫端锁死则 skipped；环路比率
    冲突 skipped）。`JointSweepOutcome.coupled` 报告随动 pair 名。
  - 每个随动 pair 的子树按**自己的世界螺旋**（速率 = Δ·rate_x）走闭式 TOI，
    退位矩阵把子树从当前姿态退回 q_p(t=0) = q_p_cur + Δ·(lo − q_x_cur)。
    嵌套检查保证随动子树互不相交（兄弟分支型联动）→ 每个 mesh 恰好一个驱动
    pair，单螺旋精确。
  - 随动对互碰：**同轴**（轴平行且轴心连线共线）→ 相对 twist 差仍是同轴螺旋，
    闭式精确；**不同轴** → probe 取 t=0 间隔，与全程相对接近速度上界
    （|v|+|ω|·(轴距+局部包围半径)，无界几何 → ∞）比较：严格大于 = 认证不碰，
    否则进 `unknown`；t=0 已接触报 q=lo。三值语义保持：绝不假装。
  - 诚实边界（skipped 注明原因）：cam 联动（cam 的 a/b 是 link 名，关联 link
    落在任一随动子树里即拒——从动 q 是接触解，非线性，不是螺旋）；嵌套 gear
    联动（一个随动 pair 在另一个的子树里——复合运动 exp(xi₁t)·exp(xi₂t) 不是
    单螺旋，闭式 TOI 不适用）。
  - 闭式验证：反向联动臂撞柱 q*=asin(0.98)（1e-9）；同轴互碰 q*=acos(0.1)
    （1e-9，相对螺旋路径）；认证不碰（间隔 9.8 > 速度界 1.1）first=None 且
    unknown 为空；保守界认证不了时 unknown 恰好含随动对两球。
