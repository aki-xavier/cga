<!-- markdownlint-configure-file {"MD013": false} -->
# 路线图（roadmap）

状态：**当前（2026-10-08）**。本文档是方向的依赖关系与排序；每个方向落地时各自单独立项（参照 `css-conformance.md` / `collision-plan.md` / `kinematics-graph.md` 的做法：计划文档 → 阶段 → 验收）。

## 1. 已建成（全部推送，332 测试全绿）

- **渲染器**：解析光线追踪（全图元 + CSG + 仿射 + 认证数值回退）、帧间像素级增量渲染（与全帧逐位一致）。
- **JSX+CSS 宿主**：真 React 19（hooks/context/memo/key/事件派发/宿主输入）、CSS 真匹配真级联（含错误契约）、帧间子树复用、分组（`Object::group`）、碰撞查询惰性求值。
- **碰撞检测**（`cga-collision`）：三值重叠判定、凸对分离距离（含精确 MTV）、接触流形、螺旋 CCD（认证无接触）、关节行程干涉扫描。
- **运动学图模型**：`<link>/<pair>/<anchor>` 无因果建模（Modelica 调研后设计）、生成树定向传播、gear/cam 无方向、`<closure>` 闭链 LM 求解、`guess` 选支。
- **crate 卫生**：cga-core 零依赖、cga-collision / cga-mesh 独立、cga-gpu（MLX）/ cga-host（boa/swc）/ cga-examples。

## 2. 方向与依赖

```
A 交互闭环（拾取→事件→增量渲染）   ← 唯一串起所有已建成子系统的方向
B 速度级运动学（twist/雅可比）     ← 依赖：图模型（已有）
C 物理属性 + 静力学               ← 依赖：bake 体积（已有）+ B 的 wrench 对偶
D 碰撞性能打磨                    ← 随时可做，无依赖
E React 差距（优先级分层/lazy）    ← 没有消费者，不做
G5+ 空间闭链 / 多自由度环路         ← 依赖：B（雅可比复用）
```

**明确不做**（记录在案，不重复讨论）：接触动力学（冲量/摩擦/时间积分）。理由：与本项目"确切或 Unknown"的验证文化冲突（摩擦是模型、积分有离散误差、混沌敏感钉不住金标），且是另一个产品。真有需求时是独立 crate，把 `cga-collision` 当库用。

## 3. 各方向展开

### A. 交互闭环（推荐先做）

**已实施（2026-10-08）**：

- **拾取**：`cga_gpu::pick(scene, camera, x, y, w, h) -> Option<PickHit>`——像素一条
  光线的解析求交（最近 t 胜出，含遮挡），命中点/法线回世界系。渲染器每像素
  已有 `best_idx`，独立拾取不依赖整帧渲染。
- **接线**：快照节点带 `__id`（schema v3）；`SceneRun.object_instances` 与
  `scene.objects` 平行（复用的克隆继承来源 id——实例 id 跨帧稳定）；
  `SceneSession::pick`（命中 + 实例 id）/ `click`（→ `dispatch(id, "onClick",
  payload)`，payload 带命中点/法线/对象下标）。
- **验收链**（`session_click_drives_state_and_incremental_render`）：像素坐标
  投影断言命中对象下标 → click → onClick → setState 跨帧累积（r: 0.2→0.3→0.4）
  → 增量渲染与全帧**逐位一致** → 未命中点击 → None。GPU 侧三测试：最近命中与
  遮挡、未命中、平面拾取。
- 边界（记录在案）：处理器必须在**宿主元素**上（组件不产生宿主实例——冒泡走
  实例树祖先链，组件上的 onClick 不会落到任何实例；组件要显式转发给宿主子元素）。
- **抓取驱动（U1，docs/ue58-inspirations.md，2026-10-09）**：`SceneSession::drag`
  /`drag_pick`——抓住 link 上一点拖到世界目标点，`drag_solve` 用 LM + 路径螺旋
  列解析雅可比解自由 1-DOF pair 的 q，经 pose 通道写回（`drag_owned` 支持连续
  拖拽）；V1 边界（cam/gear/闭链/多 DOF/不可达/越限）全部显式 Err 且场景不变。
  顺带修复反向边螺旋列符号三处（`closure_jacobian`/`jacobian`/`propagate_with_q`，
  探针 + FD 对拍证实）。验收：闭式断言 13 项 + 会话层逐位一致。
- **位姿抓取（U1 借清单第 1 件，2026-10-09）**：`drag_pose_solve` /
  `SceneSession::drag_pose`——link frame 拖到目标位姿；残差 6 行 = 位置差 +
  rotvec(R_c·R_tᵀ)（对照 control-ga-pid 的 rotvec_between，无对跖退化）；朝向行
  雅可比用 SO(3) 左雅可比逆**闭式**（任意残差处精确，FD 对拍含非零残差构型）；
  target 旋转不正规显式 Err。ga-pid 对照分析与 Plant 后端差距记录在
  `docs/ue58-inspirations.md` §3.U1 后续边界。
- **MCP 工具层（U2，docs/ue58-inspirations.md，2026-10-09）**：
  `cga-examples --bin cga_mcp`——stdio MCP 服务器（手写 JSON-RPC 子集，零新
  框架依赖），10 个场景工具（open/render/pick/drag/drag_pose/collisions/
  mass/mobility/report/close），渲染回 PNG image content，cga 的 Err 以
  `isError` + 原文透传。验收：子进程端到端 2 测试（握手/工具名单/闭式拖拽
  收回 q*/PNG 魔数/认证碰撞分离度/闭式质量/五条错误路径）。
- **诊断渲染通道（U3，2026-10-09）**：`cga_gpu::render_diagnostic` +
  `DiagChannel`（ObjectId 调色板 / Normal 相机系法线 / Depth 灰度 1/(1+t)），
  主光线副产品逐位确定；`SceneSession::render_diagnostic` + MCP `channel`
  参数。验收：GPU 4 测试（调色板逐像素/平面法线常数/深度 vs pick t 交叉/
  逐位确定）+ 会话 1（id 图恰好 4 色）+ MCP 端到端。螺旋轴/碰撞线叠加层
  形态不同，有消费者再立项。

<details><summary>原始计划（留档）</summary>

- 拾取：渲染器每像素已算 `best_idx`（命中对象下标）——拾取 = 一条光线的解析求交，几乎免费。API：`pick(scene, camera, x, y) -> Option<(object_index, point, normal, instance_id?)>`。
- 接线：对象下标 → React 实例（发射时记下实例 id；需要对象→实例的反向映射）→ `dispatch(id, "onClick", payload)`。
- 验收链：点选坐标 → 对象下标的解析断言；onClick → setState → 局部重建（create=0）→ 增量渲染与全帧逐位一致。
- 形态：无头可交互查看器（宿主驱动，不是窗口程序）；事件源抽象（先支持 pick，指针移动/拖拽后续）。

</details>

### B. 速度级运动学

**已实施（2026-10-08）**。`cga-host/src/scene_build/kinematics.rs`：

- `pair_screws_world`：副在当前位姿的世界螺旋轴列（全部 8 种副类型；twist 约定
  `[ω; v]` 物理量纲，与 sweep_toi 一致）。平面副的平移列不经 θ 旋转、旋转轴过
  当前平移点——FD 对拍抓到过这个错。
- `jacobian`（列带 `(pair 名, q 分量)` 标签）/ `link_twist` / `point_velocity`。
- `is_singular`：路径雅可比列秩 < 自由度数（同轴双副 = 奇异，有测试）。
- `mobility`：Grübler 口径——gross = Σq 维数 + closure 自带旋转 DOF；pins =
  作者给定/pose + gear + cam + Σ(closure 残差雅可比的秩 + 1)。四连杆：不钉输入
  dof=1，钉曲柄 dof=0（与 Grübler 一致）。
- 验收：全部副类型的点速度与位姿有限差分对拍（1e-4；FD 是裁判，不是手推导）、
  奇异/非奇异判定、活动度两组值。

<details><summary>原始计划（留档）</summary>

- twist 沿生成树传播（`Multivector::velocity` / `extract_velocity` 现成）；雅可比矩阵（连杆末端 twist 对 q 的偏导）；活动度 = 约束秩分析（构图期诊断奇异/冗余约束）。
- 仍是几何不是动力学：可闭式验证（旋转副的雅可比列有解析形式）。
- 为 C 与 G5+ 供底座。

</details>

### C. 物理属性 + 静力学（已完成 C1+C2）

- **C1 质量属性**：`cga_mesh::mass_properties`——图元闭式解（球/盒/柱/锥/整环面/椭球），CSG/仿射/环纹面/部分环面走水密网格的 Mirtich 体积分（`bake` 保证水密 + 一致朝向；步长自适应包围盒）；平面/圆片/无限长柱 → `None`（无有限体积，诚实跳过）。闭式与网格积分互相交叉验证（锥/环面的横向惯性公式由对拍裁决，不凭记忆）。JSX：`density` prop 沿子树下传（独立通道，不进材质键），每对象 `SceneRun.mass_props`，报告 `mass i m=… cog=…` + 汇总 `total_mass=`。
- **C2 准静态平衡**：`settle(kin, link_masses, gravity, free)`——势能 U = −Σ m·g·cog，自由 q 上最小化；带阻尼牛顿（FD 梯度/Hessian + 步长截断 + U 下降接受准则；不是平衡方程迭代——起点恰在驻点时梯度法的 Hessian 奇异，势能最小化没有这个病）。无时间积分：这是静力学不是动力学。单摆/双摆闭式平衡姿态验证。
- 不含接触动力学（见 §2 明确不做）。装配静力校核（wrench 投影到副方向）留到有消费者。

### D. 碰撞性能打磨（已完成 D1+D2）

- **D1 ✓**：`scan_contacts` 的 Yes 对接触解帧间缓存——与 probe 同一套指纹键
  （几何+世界变换），第二帧起接触解零重算、结果逐位一致（测试断言）。
- **D2 ✓**：齿轮联动行程扫描。被扫 pair 经 gear 关系（dq_b = ratio·dq_a）带动的
  pair 一并随动（`JointSweepOutcome.coupled` 列出随动 pair）；各自子树按自己的
  螺旋走闭式 TOI（兄弟分支型联动精确）。随动对互碰：**同轴 → 相对螺旋闭式精确**；
  不同轴 → t=0 间隔 vs 全程速度上界（|v|+|ω|·(轴距+包围半径)）认证不碰，
  认证不了进 `unknown`（绝不假装）。诚实边界：cam 联动（从动 q 非线性）与
  嵌套联动（复合运动非单螺旋）→ `skipped` 注明原因。闭式验证：反向联动臂撞柱
  q*=asin(0.98)、同轴互碰 q*=acos(0.1) 均 1e-9。
- **D3 ✓ 空间索引**：对象数 > 64 启用世界 AABB 宽相——AABB 不相交 ⇒ 认证 No
  （精确判定，`separation=None` 诚实不造假距离），跳过指纹缓存+窄相；无界几何
  不剪。小场景保持全对精确距离（报告/金标不变）。测试：100 球场景 5151 对
  判定与逐对 probe 参考全一致，剪掉 >90% 窄相。

### E. React 差距（已完成，2026-10-08）

- **E1 优先级分层**：dispatch（click 等离散事件）期间 `setCurrentUpdatePriority
  (Discrete)`，`resolveUpdatePriority` 返回被跟踪车道并计数（`lanes()` 可观测，
  测试断言 click 的 setState 计在 discrete）。
- **E2 `lazy()`/动态 `import()`**：bundler 把字面量规格的 `import("./x.jsx")`
  编期打包（`__cga_dyn_import(id)` → 已求值模块命名空间的 Promise；非字面量/
  缺文件编期报错）。配套修了真 bug：**boa 的 promise 任务队列只能由 Rust 推进**，
  `drain`/`frame` 现在交织 `run_jobs` 与 JS 调度器到两连静止（lazy+Suspense
  的 retry 就卡在这）。静态 import 既有，语义不变。
- **E3 DevTools 握手**：装 `__REACT_DEVTOOLS_GLOBAL_HOOK__` 记录型桩 +
  显式 `reconciler.injectIntoDevTools()`（渲染器侧义务，ReactDOM 同款）；
  `devtools()` 可观测注册/提交。检查器协议（浏览器扩展后端）不在渲染器侧范围。

### G5+. 空间闭链 / 多自由度环路（已完成）

- 闭链自由变量 = 路径上未固定 pair 的**全部 q 分量**（圆柱副/球副/平面副按分量展开）；
  雅可比 = B 螺旋列的解析导数（d(pa)=w×pa+v，d(û)=w×û），FD 对拍为裁判
  （`closure_jacobian_fd_check` 逐元素 6×5）；`lm_solve_j` 解析版与 `lm_solve`（FD）并存。
- `guess` prop 支持数组（分量数 = 副的自由度数）。
- 闭式验证：圆柱副+球副空间闭环，零位形天然闭合，偏移 guess 收回零（1e-6），
  秩 5 钉死（dof 0）；矩形四连杆（1-DOF 路径）在解析雅可比下结果不变。
- 闭链 `solved` 名单：1-DOF 保持裸名，多自由度按 `name[i]` 展开。

## 4. 推荐顺序

**A → B → C**，D 穿插，E/G5+ 由真实需求驱动。理由：A 把项目从"渲染器 + 库"变成"可交互应用"，且验收链与既有纪律同构（解析断言 + 逐位一致）；B、C 逐级抬高机构分析的语义层，全部可验证。
