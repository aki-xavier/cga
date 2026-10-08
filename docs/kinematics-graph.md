<!-- markdownlint-configure-file {"MD013": false} -->
# 运动学图模型：无因果（acausal）建模的设计建议

状态：**G0–G5 全部已实施（2026-10-08）**。用户拍板：**不保留包含式**（`<joint>` 嵌套写法已删除，显式报错指向本文档）。本文档是调研记录 + 建模语言设计 + 阶段计划。适用：`cga-host/src/scene_build/kinematics.rs`（`solve_graph` + `lm_solve`）、`cga-host/src/jsx/`、`jsx_gen`、URDF 导出。

## 0. 调研：Modelica 的无因果建模（1998–2008）

### 核心事实

1. **连接器（connector）两类变量**：across/势变量（电压、温度、位姿）在连接点**相等**；through/流变量（电流、力）在连接点**和为零**（广义基尔霍夫）。`connect(a,b)` 不是赋值，是**方程**。
2. **多体库（MultiBody）的关节定义**（官方原文）：joints are *"idealized, massless elements that constrain the motion between frames"*——**约束，不是容器**。frame_a/frame_b 是连接器（位姿为势变量、力/力矩为流变量），关节把两个 frame 的相对运动约束到一个流形上。谁父谁子不存在。
3. **因果方向由求解器推导**：同一个电阻模型，电流已知求电压、电压已知求电流都行——方程不写方向。这就是"URDF 形式不对"的学理对应物：URDF 把求解器的定向（生成树方向）写进了模型语法。
4. **闭链 = 生成树 + cut-joint**：树关节照常前向传播；cut-joint 提供**位置级约束方程**补上环路（`RevolutePlanarLoopConstraint` 等先例）。环路从不靠"选个父"解决，靠方程解决。
5. **平衡模型（Modelica 3.0）**：编译期结构检查——未知数个数 = 方程个数。构图期报错，不拖到求解期。

### 对本项目的映射

我们是**纯运动学**（没有力/力矩/动力学），所以连接器的流变量侧整体退化：没有 DAE、没有指标约化、没有 Pantelides。剩下的全是可搬的：

- 位姿相等（frame 重合）⇒ 我们的 `world(F_a)∘M(q) = world(F_b)` 约束；
- 关节 = 约束元件 ⇒ 我们的 `<pair>`；
- 生成树 + cut-joint ⇒ 我们的树求解 + 将来的 `<closure>`；
- 平衡模型 ⇒ 我们的构图期诊断；
- 无因果 ⇒ gear/cam 的 driver/driven 命名废弃（q 方程无方向）。

## 1. 设计决策

- **D1 约束无向**：`<pair>` 的 `a`/`b` 对称。求解器从锚点 BFS 定向生成树；传播方向反了的边在矩阵上取逆（文档写明逆变换规则）。
- **D2 不保留包含式**：`<joint>…</joint>` 嵌套写法删除。画廊/示例/生成器全部迁移到 `<link>`/`<pair>`（G1 的验收：迁移后渲染逐位不变）。
- **D3 frame 约定**：pair 的 `at`/`axis`/`rpy` 表达在 **`a` 侧连杆 frame**；`q` 的语义方向 = 从 F_a 看 F_b 的正向运动；反向传播时用逆。多 DOF 关节的 `M(q)` 沿用现有 `joint_motion`。
- **D4 高副无因果化**：`<gear a="p1" b="p2" ratio offset>`（`q_b = ratio·q_a + offset`，谁解谁由求解器定）；`<cam a b aProfile bProfile>`（接触约束对称，求解方向求解器定）。
- **D5 锚定唯一**：`<anchor link="base"/>` 至多一个；缺锚 → 报错（位姿无参考系）。孤岛 link（与锚不连通）→ 报错。
- **D6 渲染树只承担几何归属**：`<link>` 是元素树里的几何容器（CSS/材质/增量/分组照旧），frame 来自图求解，不再来自树嵌套。
- **D7 闭链另行立项**：`<closure joints=[...]/>`（位置级约束方程 + 牛顿/认证求根）是后续阶段；G1 只覆盖树机构。图里有环且无 closure → 构图期报错（不许静默走生成树丢掉约束）。
- **D8 错误契约**：未知引用、重名 link/pair、多锚、孤岛、环无 closure、q 越限——全部构图期报错并带名字。

## 2. 语言形式

```jsx
{/* 体：平铺。几何 + 公共属性（t/rotate/scale/mirror/tag/class/材质）照旧 */}
<link name="base"><box s={[0.4, 0.4, 0.1]} /></link>
<link name="arm1"><cylinder r={0.05} h={1} /></link>
<link name="arm2"><cylinder r={0.05} h={1} /></link>

{/* 副：无向约束。at/axis 写在 a 侧 frame；q 初值/限位照旧 */}
<pair kind="revolute" a="base" b="arm1" at={[0, 0, 0.05]} axis={[0, 1, 0]} limit={[-1.5, 1.5]} />
<pair kind="revolute" a="arm1" b="arm2" at={[0, 0, 1]} axis={[0, 1, 0]} />

{/* 世界锚定（唯一） */}
<anchor link="base" />

{/* 高副：q 图上的方程，无方向 */}
<gear a="elbow_drive" b="elbow" ratio={-2} />

{/* 闭链（后续立项）：位置级约束方程 */}
<closure joints={["j1", "j2", "j3", "j4"]} />
```

约束方程：`world(F_a) ∘ M(q) ∘ F_b⁻¹ = world(B)`——F_a = T(at)·R(rpy)（a 侧 frame），F_b 默认单位。生成树反向边按逆变换传播。

## 3. 求解器（树机构，G1）

1. 构图：links = 节点，pairs = 边，校验（D8）。
2. 定向：从锚 BFS 生成树；每条边记方向（正向/反向）。
3. 传播：`world(B) = world(A) ∘ F_a ∘ M(q) ∘ F_b⁻¹`（反向边：对 q 取逆变换）。
4. 高副方程：gear 直接代入；cam 沿用认证求根（`cam_solve` 的 bracket+bisect，方向改为"由已解侧解未解侧"）。
5. q 的来源：prop 初值 / pose 覆盖 / 高副方程解出。

## 4. 阶段与验收

**已实施（2026-10-08）**：

- **G1 ✓**：`<Link>/<Pair>/<Anchor>` 元素 + 两遍构建（pass 1 `collect_decl` 收集
  声明 → `solve_graph` 图求解 → pass 2 注入 `link_worlds` 的正式 walk）。
  生成树 BFS 定向（反向边矩阵取逆）、q 赋值（pose > prop > 0）、gear 无因果
  不动点（a 侧缺省取 0 为种子，与旧模型一致）、cam 逐个解（自由 q = anchor→b
  路径上第一个未固定 pair）。`<joint>` 已删除（显式迁移报错）。
  **最强验收**：`demo_pairs` 再生的 `pairs.png` 与迁移前**逐位一致**。
- **G2 ✓**：`<Gear a b>` / `<Cam a b aProfile bProfile>` 无因果命名；
  gear 双侧给定查矛盾、ratio=0 反解报错；cam 的"谁解谁"由求解器从固定状态推导。
- **G3 ✓**：构图期诊断全集（重名 link/pair、未知引用、缺锚、多锚、孤岛、
  闭环无 closure、q 越限（pin 预检 + 终检）、gear 矛盾、限位格式）。
- **G4 ✓（同批）**：URDF 从定向生成树导出（反向边 = 显式 G1 边界错误）；
  报告输出 link/pair/anchor/gear/cam 行；`sweep_joint` 适配 pair +
  tree_down 子树；`kinematics-pairs.js` 组件改 pair 形式（不再有 children）。
- **G5（立项）**：`<closure>` 闭链约束求解（计划见 §3 与 docs/collision-plan.md）。

- **G5 ✓（2026-10-08）**：`<closure a b at bAt axis>` 环路闭包。语义：a 侧 `at`
  点与 b 侧 `bAt` 点重合 + 闭合轴对齐（revolute 闭环）。求解：闭链的未知量是
  环路上未固定的 1-DOF pair（closure 副自己的 q 是闭合物不是未知量）；
  残差 = 点重合（3）+ 轴对齐（叉积 3）；Levenberg–Marquardt + 有限差分雅可比 +
  部分主元高斯消元（小规模手写，不引依赖）；不收敛 / 收敛残差超界 → 显式报错。
  pair 新增 `guess` prop（求解器初值选支；不被求解器触碰则报错）。
  验收：矩形四连杆闭式解（q1=q2=−π/2）1e-6 吻合 + 摇杆端点落点断言 +
  同输入同解 + 错误路径（未知引用/无自由 q/装不上）+ 报告 closure 行。

- **G5+ ✓（2026-10-08，空间/多自由度闭链）**：闭链自由变量 = 路径上未固定 pair
  的全部 q 分量（圆柱副/球副/平面副按分量展开，旧「不是 1-DOF 另行立项」拒绝
  删除）。雅可比从 FD 升级为 **B 螺旋列的解析导数**（`screws_world_at`：
  d(pa)=w×pa+v，d(û)=w×û，驱动侧由生成树子树关系定）；`lm_solve_j`（解析）与
  `lm_solve`（FD）并存，FD 对拍为裁判（`closure_jacobian_fd_check` 6×5 逐元素）。
  `guess` prop 支持数组（分量数 = 副自由度数）。`closure_residual` /
  `closure_jacobian` 抽出为 pub(crate) 可测单元。闭式验证：圆柱副+球副空间闭环
  零位形天然闭合、偏移 guess 收回零（1e-6）、秩 5 钉死（mobility dof 0）；
  四连杆（1-DOF）结果与 FD 版逐位一致。`solved` 名单：1-DOF 裸名、多自由度
  `name[i]`。

<details><summary>原计划阶段表（留档）</summary>

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| G0 | 本文档 + `Kinematics` 图数据结构（nodes/edges/anchor） | 文档评审 |
| G1 | `<link>/<pair>/<anchor>` + 生成树定向传播；画廊与生成器迁移；**包含式删除** | 迁移后 8 场景渲染逐位不变；全测试绿 |
| G2 | gear/cam 无因果化（a/b 命名 + 求解方向推导） | 现有 gear/cam 测试改写法后全过 |
| G3 | 构图期诊断全集（D8） | 每类错误一个测试，错误带名字 |
| G4 | URDF 导出与报告适配（图 → URDF 树需选根定向，内部细节） | 导出金标不变 |
| G5（立项） | `<closure>` 闭链约束求解 | 四连杆闭式验证 |

</details>

## 5. 风险

| 风险 | 对策 |
| --- | --- |
| 迁移破坏画廊金标 | G1 验收就是逐位一致；先迁生成器再迁手写场景 |
| 反向传播的 q 符号错误 | 每类关节一个"正向写反向算"的等价测试 |
| 用户已习惯包含式写法 | 用户已拍板删除；文档给迁移对照表 |
| 闭链误用（环无 closure） | 构图期报错（D7），不许静默 |

## 6. 参考资料

- Elmqvist 1998, *A Unified Object-Oriented Language for Physical Systems Modeling*（无因果方程的原始设计 rationale）
- Modelica MultiBody Joints 文档："joints constrain motion between frames"；`RevolutePlanarLoopConstraint`（cut-joint 位置级约束先例）
- ModelicaAdditions.MultiBody：Tree-Joints / Cut-Joints 分解
- Olsson 2008, *Balanced Models in Modelica 3.0*（方程数 = 未知数的编译期结构检查）
- mbe.modelica.university, Connectors 章（across/through 变量与连接语义）
