<!-- markdownlint-configure-file {"MD013": false} -->
# UE 5.8 设计调研：对本项目的启发

状态：**调研完成（2026-10-09）**。本文档是调研记录 + 候选方向清单；每个方向落地时按惯例单独立项（参照 `collision-plan.md` / `kinematics-graph.md` 的做法：计划文档 → 阶段 → 验收）。本文档本身不改 roadmap。

## 0. 调研对象与方法

- Unreal Engine 5.8，2026-06-17 发布，**UE5 最后一个主版本**，Epic 此后转入 UE6（UE5 仅维护性支持）。
- 来源：Epic 官方公告（unrealengine.com）、官方论坛发布帖、CG Channel / GamesBeat / Digital Production / PC Gamer 报道。官方 release notes 页需登录（403），细节以公告 + 多家报道交叉核实为准。
- 只记录与本项目（解析几何内核 + GPU 渲染 + JSX/CSS 宿主 + 运动学/碰撞）有对照价值的内容；纯游戏/影视工作流（nDisplay、VCam、音频等）略。

## 1. UE 5.8 核心事实（相关部分）

| 系统 | 状态 | 事实 |
| --- | --- | --- |
| Mesh Terrain | Experimental | 真 3D 网格地形，替代 2.5D heightfield：悬垂、浮岛、隧道可表达。非破坏修饰器（挪动地貌特征自动重生）。与 PCG / World Partition / OFPA 互操作 |
| PCG | 增强 | **程序结果上可手工编辑，底层程序逻辑不断开**；嵌入子图：技术美术把复杂度藏进子图，只向艺术家暴露参数 |
| MegaLights | → Production-Ready | 大量动态投影面光源。转正的关键叙事不只是性能（本代主机 60fps），还有**诊断工具**：Light Finder、Ray Visualizer（可视化光线迭代），让人"敢放灯" |
| Lumen Lite | 新 | 中质量 GI 模式（辐照度场 + 探针遮挡），成本约为高质量 Lumen 一半。质量/性能分层显式化 |
| Toon Shader | Experimental | 基于 Substrate 的 NPR 着色模型，全目标平台 |
| Movie Render Graph | → Production-Ready | 渲染设置图化；**逐 shot 隔离灯光**（同一场景不同灯光取值出多镜）；Accumulation DoF：接近路径追踪的景深质量、实时成本 |
| Direct Mesh Controls (DMC) | Experimental | Control Rig 控制器直接表示在骨骼网格表面上，动画师**拖表面摆姿势**，不操作抽象控制器 |
| Control Rig Physics / Dynamics | Beta / Experimental | 物理 rig 模块化、可叠在既有动画上、权重可关键帧；Dynamics 是粒子求解器，**5× 速度换精度**，运行时取向。双求解器并存，用户按场景选 |
| MetaHuman Collections | Experimental | 人群按相机距离在**高保真个体 Actor ↔ 低保真实例化蒙皮网格（ISKM）间无缝切换**；Mass 编排、Nanite 渲染。保真是阶梯，不是开关 |
| MCP 插件 | Experimental | 任意 LLM 经 Model Context Protocol 连接引擎核心系统（蓝图/资产/关卡/材质/网格），做资产创建/测试/优化。官方演示：Claude Code 从资产库取物、摆场景、对齐参考图调光。Epic 明言 UE6 中心 = LLM 进创作管线 |
| Sandboxes | Experimental | 隔离实验环境，选择性合并回主项目，可分享给协作者 |
| Dataflow | → Production-Ready | 物理资产（破碎/布料）的节点图，非破坏：改动即时重生，迭代不丢上游 |
| 其他 | — | Gizmo 系统统一；移动端增量 cook 提速迭代 |

## 2. 三条设计信号

1. **LLM 进创作管线**：MCP 插件 + UE6 宣言。引擎的职责从"给人用的编辑器"扩为"给 LLM 可编程操作、且结果可检查的后端"。
2. **一切程序化皆非破坏**：PCG 手工编辑不断开逻辑、Dataflow 转正、Mesh Terrain 修饰器。规则：**生成结果可以被覆盖，覆盖本身是独立层，上游逻辑永远可重生**。
3. **精度/性能分层显式化 + 诊断驱动信任**：Lumen Lite、Control Rig Dynamics、MetaHuman Collections 都是"同一语义、多档实现、按场景选档"；MegaLights 的转正叙事说明：让用户敢用高级特性的关键是**可观测的诊断工具**，不是文档承诺。

三条信号与本项目的对应：① JSX/CSS 纯文本场景 + 可验证反馈 = LLM-native 程度高于 UE；② CSS 级联 + `pose > prop > solved > 0` 已是非破坏覆盖链；③ FD/解析双雅可比、碰撞认证回退已是分层求解。详见 §4、§5。

## 3. 候选方向（按"已有零件复用度 / 价值"排序）

### U1 抓取驱动运动学（直接操纵，对应 DMC）

**已实施（2026-10-09）**。

**UE 做法**：动画师直接在角色网格表面拖拽摆姿势（DMC）。

**本项目现状：拼图已齐，只差拼起来**：

| 件 | 位置 | 给 U1 什么 |
| --- | --- | --- |
| 解析拾取（命中点/法线/实例 id） | `cga_gpu::pick`、`SceneSession::pick` | 抓住"表面上哪一点" |
| 事件派发 + 优先级车道 | `dispatch`、E1 | 拖拽事件流进 React 状态 |
| 增量渲染逐位一致 | `IncrementalRenderer` | 拖拽过程只重算脏像素 |
| 雅可比（螺旋列 + 解析导数） | `kinematics.rs::jacobian` | 抓取点速度对 q 的偏导 |
| LM 求解器（解析/FD 并存） | `lm_solve_j` | "让被抓点跟随光标"= 位置级约束求解，与 `<closure>` 同构 |

**落点**：抓住 link 表面一点拖动 → 每帧把"被抓点到目标点"写成残差（复用 `closure_residual` 形态），LM 解 q，渲染增量更新。这是把方向 A（交互）、B（雅可比）、G5+（闭链求解）串成产品的最短路径。

**验收**（与既有纪律同构）：解析位形（如四连杆）拖到闭式已知落点做断言；全程增量渲染与全帧逐位一致。

**诚实边界**：拖拽目标不可达 / LM 不收敛 → 停在最后合法位形并显式报告，不假装跟随；拖拽路径上撞限位 → 停在限位。

**实施记录（2026-10-09）**：

- **核心**：`cga-host/src/scene_build/kinematics.rs` 的 `drag_solve(kin, link,
  grab_local, target, extra_free)`——anchor→link 树路径上自由 1-DOF pair 的 q
  由 LM（`lm_solve_j` 复用）解"抓取点−target"3 行残差；雅可比 = 路径螺旋列
  解析点导数（`drag_path_walk` / `drag_jacobian`，pub(crate) 可测）。
- **会话**：`SceneSession::drag(link, from, to)`（世界点 → link 局部系 → 求解
  → q 经 pose 通道写回 → 重建一帧；失败回滚 pose）与 `drag_pick(x,y,w,h,to)`
  （拾取 → 命中对象所属 link → drag）。`drag_owned` 集合记录上轮写回的 pair，
  连续拖拽时它们仍是自由变量。
- **诚实边界（V1，全部显式 Err 不静默）**：多 DOF 副保持当前 q（pose 写回
  只支持 1-DOF）；gear 耦合 pair 固定；路径触碰闭链求解 pair → 拒绝（闭链
  机构拖拽 = 拖动+闭链联解，另行立项）；含 cam 机构 → 拒绝；不可达 / 不
  收敛 / 解出 q 越限 → Err 且场景不变。
- **顺带修复的真 bug（探针先行，FD 裁判证实）**：`closure_jacobian`、
  `jacobian()`/`link_twist`/`point_velocity`、`propagate_with_q`（settle 用）
  对**反向边**（pair 朝 anchor 写，生成树反向传播）漏取螺旋列负号/逆变换。
  既有测试全部正向声明所以从未暴露；`closure_jacobian_fd_check_reversed_edges`
  探针给出 −1.3308 vs +1.3308 的实证后修复。
- **验收**：闭式断言 11 项（单摆 π/2 正反两向、滑动副 1.5、2R 臂 FK 往返、
  球副保持不动、gear/闭链/cam 拒绝、extra_free 连续拖拽、不可达/越限 Err）+
  混合正反向链雅可比 FD 对拍 + 会话层 2 项（闭式拖拽 + 增量渲染逐位一致 +
  失败场景不变；像素抓取 drag_pick）。cga-host 142 测试全绿。

**后续边界（2026-10-09，对照 `code/control` 的 control-ga-pid 框架）**：

- **ga-pid 是什么**：Khatib 操作空间控制——τ = J′·f + C·q̇ + g，任务是逐平面
  （点 3 通道 / frame 6 通道）PID，设计面是极点配置（`k = ωn² − k_eff`、
  `d = 2ζωn − b_eff`，`wn_for_settling`/`zeta_from_overshoot` 闭式反解），
  Λ = (J·M⁻¹·J′)⁻¹ 惯量整形，PGA motor 表示位姿 + `rotvec_between` 朝向误差。
  它的前提是**动力学模型 + 时间积分 + 执行器 + 控制周期**——本项目 roadmap §2
  记录在案不做的"另一个产品"。它也没有"取消 IK"：位置级求根换成了带动力学的
  渐近跟踪，雅可比全程在用。分工判定：cga 的 LM 拖拽回答"什么位形恰好满足
  约束"（代数，要确切）；ga-pid 回答"什么力矩让植物以指定极点跟踪"（控制，
  要渐近）。直接把 dt/积分器/稳态误差搬进确定性引擎是净损失，不搬。
- **可借清单**（不越界，按序）：
  1. **motor-log 残差**（`frame_motor` + `rotvec_between` 形态）：位姿抓取
     （6 行：位置 + 朝向）的残差用 rotvec(R_c·R_tᵀ) + 位置差，不用点差+轴叉积
     （叉积在对跖朝向退化）。**已实施（2026-10-09）**：`drag_pose_solve` /
     `SceneSession::drag_pose`，见下。
  2. `TaskSpec` 逐通道规格（wn/zeta/ti/deadband/lead 每通道一份）：6 通道
     抓取扩展时的规格格式，有消费者再做。
  3. 二阶参考滤波器的设计面（`wn_for_settling`/`zeta_from_overshoot` 反解
     ωn/ζ）：拖拽目标不瞬移、按指定沉降时间平滑到位的"手感层"。线性、
     响应闭式、固定 dt 下逐位确定（宿主 drain/shim 本来就是确定性时间），
     金标可钉——ga-pid 设计面在纯运动学里的合法移植。
  4. keep-out 目标整形（escape.rs 的 TaskAvoidance）：拖拽加避障时把目标
     投影出禁区；认证碰撞距离现成。全身逃逸（逐 link 雅可比 + DLS）是 scope
     扩张，有消费者再立项。
- **反向整合远景（记录，不立项）**：`Plant` trait 引擎无关，cga 当 Plant
  后端的差距 = CRBA 质量矩阵 + Coriolis + 时间积分。其中 M(q) = Σ J_i′·I_i·J_i
  的原料**已在**（全 8 种副的解析螺旋雅可比 + C1 闭式质量属性），缺的主要是
  Coriolis（RNE/Christoffel）。这就是"另一个产品"的准确形状，等 z1-arm 类
  消费者出现时另行立项。

**位姿抓取实施记录（2026-10-09，借清单第 1 件）**：

- **核心**：`drag_pose_solve(kin, link, target: [f64;16], extra_free)`——把
  link 的 frame（位姿，不只是点）拖到目标位姿。残差 6 行 = 位置差（3）+
  rotvec(R_c·R_tᵀ)（3，rotor 对数形式，无叉积的对跖退化）；雅可比 = 路径
  螺旋列（位置行 w×p+v、朝向行 w，r=0 处精确，远离时是标准几何 IK 一阶
  模型——LM 接受准则仍是残差下降，收敛由终检 1e-6 钉死，与闭链同一诚实
  形态）。target 旋转部分不正规 → 显式 Err。
- **会话**：`SceneSession::drag_pose(link, target)`，写回/回滚/连续拖拽与
  抓点版同一套（`drag_owned` 共享）。
- **验收**：闭式断言（单摆 π/2 正/反两向、2R 臂位姿 FK 往返收回 q*、
  朝向不可达 Err、非正规 target Err）+ r=0 处雅可比 FD 对拍（混合正反向链）
  + 会话层闭式拖拽 + 增量渲染逐位一致。

### U2 MCP 工具层（LLM-native 几何后端）

**已实施（2026-10-09）**：`crates/cga-examples/src/bin/cga_mcp.rs`（stdio MCP
服务器）+ `crates/cga-examples/tests/mcp_stdio.rs`（子进程端到端 2 测试）。

**UE 做法**：实验性 MCP 插件把引擎核心系统暴露给任意 LLM；官方演示 LLM 摆场景、对齐参考图调光。UE 的场景状态是二进制资产，LLM 读写隔一层编辑器插件。

**本项目的适配性高于 UE**：场景即纯文本（JSX+CSS，LLM 原生可读写）；全部操作是无头宿主函数；**每个动作有可验证反馈**——金标图、解析断言、`probe` 三值判定、`mass`/`mobility` 报告。这正是 LLM 闭环需要的"动作 → 机器可检查的结果 → 纠错"，也是 LLM 生成 3D 内容当前最大的痛点。

**实施记录**：

- **协议**：stdio，一行一个 JSON-RPC 消息；协议子集手写（initialize / ping /
  tools/list / tools/call），不引 MCP SDK（协议面小，零新框架依赖；serde_json
  是既有依赖）。cga 的 Err → `isError: true` + **错误原文透传**（不假装成功）。
- **工具 10 个**：`scene_open`（JSX+CSS 源码直接传入，返回摘要+活动度）/
  `scene_close` / `scene_report` / `scene_render`（PNG image content + 增量
  统计）/ `scene_pick` / `scene_drag` / `scene_drag_pose` / `scene_collisions`
  （三值判定 + 缓存统计）/ `scene_mass` / `scene_mobility`。
- **验收**（子进程逐行 JSON-RPC）：握手 + 10 工具名单 + 未 open 的指导性
  错误 + 闭式拖拽收回 q* + PNG 回图（base64 魔数断言）+ 认证碰撞（球穿盒
  Yes 且分离度 −0.05/−0.07 解析断言）+ 闭式质量（1e-6）+ 坏 JSX/未知工具/
  未知方法/缺参数/不可达拖拽五条错误路径（错误原文透出、场景不变）。
- 边界（记录在案）：单会话（多会话/命名会话有消费者再做）；`scene_open`
  失败时旧会话保持；asset_root 固定 "."（内联源码不需要磁盘解析）。

**落点（原文）**：把 `render_jsx` / `pick` / `scan_contacts` / `mass_properties` / `mobility` 包成 MCP 工具。本项目"不确定就报 Unknown、不支持就报错带原文"的纪律恰好是 LLM 后端最需要的品质：绝不假装精确。

### U3 诊断渲染通道（对应 MegaLights 的诊断叙事）

**UE 做法**：MegaLights 转正的关键不止是性能，还有 Light Finder / Ray Visualizer——可观测性让用户敢用高级特性。

**落点**：渲染器每像素已有 `best_idx`，诊断输出是内核副产品，成本接近零：blade id 图、法线图、命中深度图、运动学螺旋轴可视化、碰撞对连线。headless 引擎没有编辑器视口，**诊断图就是视口**。与 E1/E3 把 `lanes()`/`devtools()` 做成可观测的思路一致。

### U4 渲染侧宽相 + 实例批处理（对应 Collections 的保真阶梯）

**UE 做法**：对象按距离在高/低保真表示间切换，保真是显式的资产级阶梯。

**本项目现状**：碰撞侧 D3 已做（对象数 > 64 启用世界 AABB 宽相，剪掉 >90% 窄相，判定与逐对参考全一致）；**渲染侧仍每帧遍历全部图元**（README 自述约十个）。`module`+`for` 的实例场景长大（grid.jsx 才 3×3）时，渲染需要同构的宽相：AABB 视锥剔除 + 同形实例共享求交参数。

**附带的表示阶梯**：blade（解析）↔ bake 网格（Mirtich 体积分 / 导出）目前按查询类型隐式选择；UE 的启发是把它升级为带规则的显式策略层。

### U5 解析轮廓 NPR（对应 Toon Shader，且是 UE 做不到的）

**UE 做法**：Substrate NPR 卡通着色。网格引擎的描边是屏幕空间后处理（法线/深度不连续检测，本质是猜）。

**本项目的独有优势**：CGA 里**球的轮廓是圆、圆柱的轮廓是直线——轮廓本身是 blade**，可解析提取、无走样。这是"用代数而不是用 hacks"的又一例，与"宁可解析不可近似"的气质吻合，且是 UE 架构上做不到的效果。

### U6 光圈景深（对应 Accumulation DoF）

解析光线追踪里 DoF = 光圈抖动采样，与已有 aa 超采样同构，成本接近零。UE 把它当卖点（"接近路径追踪质量、实时成本"）说明这类"采样层小改动、观感大提升"的特性性价比高。低优先，随时可做。

### U7 参数 provenance 报告（对应 PCG 手工编辑）

**UE 做法**：程序结果被手工覆盖，覆盖层独立、程序逻辑不断开。

**本项目已有同构物**：CSS 级联（覆盖是独立层）、`pose > prop > solved > 0` 的取值链。可补一层报告：每个生效参数标注来源（author / CSS / solve / default）。低优先，有消费者再做。

## 4. UE 5.8 为既有决策的背书

以下方向本项目已实施，UE 5.8 相当于工业界投了赞成票，不再展开：

| UE 5.8 | 本项目已有 |
| --- | --- |
| PCG 手工编辑不断开程序逻辑 | CSS 级联、`pose > prop > solved > 0` |
| Control Rig Dynamics：双求解器按场景选档 | FD/解析双雅可比并存、FD 当裁判；碰撞认证数值回退 |
| Dataflow：非破坏节点图管理物理资产 | CSG 树不 bake 即可渲染，bake 只服务体积积分 |
| Mesh Terrain：抛弃 2.5D heightfield 假设 | 抛弃"万物皆三角网格"，用 blade。推论：未来加地形/植被类几何时先找解析/半解析表示，网格是回退不是默认 |
| Movie Render Graph 逐 shot 隔离灯光 | 即 CSS 级联在灯光上的自然延伸（记住，无消费者不做） |
| 增量 cook 提速迭代 | 帧间子树复用 + 像素级增量渲染 |

## 5. 明确不搬

- **Chaos 接触动力学 / 布料 / 破碎**：`roadmap.md` §2 记录在案不做（与验证文化冲突，且是另一个产品），UE 做得再好不改变这个判断。
- **World Partition / 流式加载**：场景规模（十级图元）远没到需要它的量级。
- **MetaHuman / 动捕全家桶**：域外。
- **Sandboxes**：协作工具，单人项目无消费者；git worktree 已覆盖其实验隔离语义。

## 6. 参考资料

- [Unreal Engine 5.8 is now available](https://www.unrealengine.com/news/unreal-engine-5-8-is-now-available)（官方公告，主来源）
- [Unreal Engine 5.8 Released（官方论坛发布帖）](https://forums.unrealengine.com/t/unreal-engine-5-8-released/2729274)
- [Unreal Engine 5.8 Preview（官方论坛）](https://forums.unrealengine.com/t/unreal-engine-5-8-preview/2721597)
- [MetaHuman 5.8 Released!（官方论坛）](https://forums.unrealengine.com/t/metahuman-5-8-released/2729288)
- [CG Channel: 5 key features for CG artists in UE 5.8](https://www.cgchannel.com/2026/06/see-5-key-features-for-cg-artists-in-unreal-engine-5-8/)
- [GamesBeat: Epic Games launches Unreal Engine 5.8](https://gamesbeat.com/epic-games-launches-unreal-engine-5-8/)
- [Digital Production: Unreal Engine 5.8 Tidies Up](https://digitalproduction.com/2026/06/18/unreal-engine-5-8-tidies-up/)
- [PC Gamer: UE 5.8 launches…](https://www.pcgamer.com/software/unreal-engine-5-8-launches-with-improved-terrain-and-vegetation-tools-a-lumen-lite-option-for-faster-global-illumination-and-for-the-times-we-now-live-in-an-open-standard-plugin-for-llm-systems/)
- [Digital Trends: Epic takes a big step toward AI-built games](https://www.digitaltrends.com/gaming/epic-games-just-took-a-big-step-toward-ai-built-games-with-unreal-engine-5-8/)
