<!-- markdownlint-configure-file {"MD013": false} -->
# JSX + CSS 场景宿主：React 模式前端

状态：已实现（2026-10-06，路线 A：真 JS；2026-10-07，P1：改用**真 React 运行时**）。适用：`crates/cga-host/src/jsx/`、`crates/cga-host/src/react/`、CLI `render_jsx` / `report_jsx`。
R3/R4 已完成（2026-10-06）：生成管线（`jsx_gen`、`jsx_to_urdf`）改出 JSX，且使用与宿主一致的 PascalCase 元素名（`<Scene>` / `<Rotate>` / `<Difference>` …）；互操作**仅导出**（URDF / STL），不提供导入；CGS parser 已退役删除（`scene_lang` → `scene_build`，只剩 builder/kinematics/报告支撑），CGS 文本语法不再被解析。
决策记录：用户拍板"以 React + CSS 模式为主，本仓库红线（确定性错误契约/单遍/文本即真相）可以不管"。因此不走"CGS 降级为 IR"的保守路线，直接内嵌真 JS 引擎。

## 1. 技术栈与管线

| 件 | 选型 | 职责 |
| --- | --- | --- |
| JSX → JS | swc（`swc_core`，classic 运行时，pragma `h`） | 解析 + 变换，错误带 span（报 `JSX line N: …`） |
| JS 执行 | boa（纯 Rust，无 IO 沙箱） | 真 JS 语义：组件、`map`、三元、`Math`、任意表达式 |
| React 运行时 | 内置 react@19.3.0 + react-reconciler@0.34.0 + scheduler@0.28.0（`crates/cga-host/assets/react-runtime.{dev,prod}.js`，`make vendor-react` 生成） | 组件模型：hooks / context / memo / effects / 协调与局部更新 |
| 平台层 | `crates/cga-host/assets/react-shim.js`（我们维护） | 确定性 `setTimeout`/microtask/`performance`/`console`：时间只由 `drain` 推进，无事件循环 |
| 渲染器适配层 | `crates/cga-host/assets/react-host.js`（我们维护） | host config → 场景实例树 → `{t,p,c}` 快照（与旧元素树同形）；统一 RPC 入口 `__sess.call` |
| 场景预置 | `crates/cga-host/assets/scene-prelude.js`（我们维护） | 元素名常量 + 查询辅助 + `h`/`Fragment`，编译期 `include_str!` 注入模块最前面 |
| CSS | lightningcss | 规则解析（tag / `.class` / `#id` / `:root` / `scene`） |
| 场景构建 | 复用 `scene_build` 的 `build_geometry`/`build_material`/kinematics | 语义单一来源 |

管线：`.jsx` → swc 变换（`export default` → `const __scene`）→ 在常驻会话里以 `new Function('P', 模块)` 求值一次 → React 挂载 → `drain` 跑到不动点 → 快照已提交的场景实例树 → Rust 建 `SceneRun`（Scene + camera + tags + kinematics，报告管线原样可用）。

会话按线程复用，运行时只解析一次（进程内）；每次渲染只付"模块求值 + React 挂载 + 快照"。引擎跑在自带 64 MB 栈的专用线程上（boa 的解析是递归的，深嵌套场景在 debug 下会顶爆默认栈）。

## 2. 约定

- 场景 = 文件末尾的 `export default <element>`。
- 内置元素 PascalCase 或全小写均可（`<Sphere r={1}/>` ≡ `<sphere r={1}/>`）。
- 组件就是 React 组件（首字母大写）：hooks、`memo`、`Fragment`、`key` 都可用。模块只求值一次，组件身份跨帧稳定，因此 hook 状态得以保留。
- CSS 只认单层选择器（tag / `.class` / `#id` / `:root` / `scene`），材质属性沿元素树继承，内联 prop 优先于 CSS 规则，CSS 按源码顺序级联。
- `import './x.css'` 由宿主跟随装载；其他 import 报错。
- 材质键：`color roughness metalness emissive opacity ior absorption map`；JSX 元素可直接带这些 prop，也可由 CSS 类命中。
- 驼峰/蛇形双写兼容（`driverProfile` ≡ `driver_profile`）。

## 3. React 能力边界

**可用**（真 React 提供，均有测试看守）：`useState useReducer useMemo useRef useContext useEffect`（依赖与清理顺序正确）、`memo`、`createContext`、`Fragment`、`key` 协调；`Suspense` / error boundary / `useTransition` / `useSyncExternalStore` / `ref` / `forwardRef` 的机制都在（需要时自行给数据或驱动）。

**协调语义**：只有变化的实例被重建 —— 子组件自身 `setState` → 新建 0 / 更新 1；keyed 插入 → 新建 1、其余实例身份不变；卸载跑副作用清理。

**确定性**：调度器的时间由宿主 `drain` 推进（微任务 + 定时器，有界轮数），"挂载 → 提交 → 副作用"在没有事件循环的情况下跑到不动点；同输入同输出（有测试）。

**注意"局部"的边界**：局部重渲染是**场景级**的——只重建发生变化的宿主实例，整棵树的快照与 `SceneRun` 每帧重算；渲染仍是整帧（逐像素增量重绘是渲染器的另一课题）。

**没有落点**（不是 React 的限制，是本项目尚无宿主）：DOM（`document`/`window`）、真实事件源与布局、portal 到 DOM。`onClick` 之类会被 React 正常装上，但需要宿主派发事件并长驻会话——交互运行时属于后续阶段；`lazy()` / 动态 `import()` 还需要一个模块加载 shim。

**多帧与事件（宿主驱动）**：`SceneSession` 让一个模块跨多帧存活。
- **宿主输入（props 监听）**：输入放在 React context（`useContext(HostInput)`），宿主 `set_input(json)` 后 `update()`；只有消费者重渲染，其余子树 bailout。`render_frames` 每帧推 `IN.t`，示例场景 `animation.jsx` 首帧 create=13、之后每帧 **create=0 / update=8**。
- **事件派发**：自定义渲染器不会自动派发事件，由宿主决定"命中谁、派发什么"：`dispatch(instance_id, prop, payload)` 沿祖先链找第一个处理器并调用（简化版冒泡），随后 `drain`。测试里 `onClick` → `useState` → 局部更新（create=0，状态跨帧累积）。
- 没有落点的是**事件源**（拾取/指针）：渲染器已经是光线追踪，`scene_session.instances()` 给出可命中的实例，把它接到拾取上即可做成查看器——属于后续工作。

**版本适配**：host config 面按 React 版本固定（19 移除了 `prepareUpdate`、把 diff 交给 `commitUpdate(instance, type, prevProps, nextProps)`、元素标记改名 `react.transitional.element`、dev 构建额外要求性能追踪/View Transition/test selector 一组键）。升级 React 只需改 `react-host.js` 并重跑 `make vendor-react`。

## 4. 元素 → 语义映射

| JSX | 语义 |
| --- | --- |
| `<Sphere r Plane Box Cylinder Circle Cone Torus Cyclide Ellipsoid>` | 图元（参数校验复用 scene_build builder）。网格/曲面（mesh/bezier/extrude/loft）与烘焙已随网格支持一并移除 |
| `<Translate t>` `<Rotate axis angle>` `<Scale s>` `<Mirror axis>` | 修饰符（组合 4×4 ctx） |
| `<Material …>` | 材质合并作用于子树 |
| `<Union/Difference/Intersection>` | CSG 块（≥2 子几何，纯解析图元的递归布尔） |
| `<AmbientLight/DirectionalLight/PointLight/Camera/Background>` | 灯光与相机 |
| `<Joint name type axis at rpy q limit pitch>` | P1 关节（嵌套成父子树） |
| `<Revolute/Continuous/Prismatic/Helical/Cylindrical/Spherical/Planar/Fixed …>` | 关节副的**组件写法**（类型进组件名），等价于 `<joint type=…>`；其余 props/children 原样转发 |
| `<Gear driver driven ratio offset>` | P2 齿轮耦合（高副）；也可写 `<gear>` |
| `<Cam driver driven driverProfile drivenProfile>` | P3 凸轮接触求解（高副）；也可写 `<cam>`。profile 是普通对象 `{kind:"circle",c,n,r}` / `{kind:"plane",n,d}` |
| `<Tag name>` | 标签注册表 |

运动副也可用 **React 组件**写法：`Revolute/Continuous/Prismatic/Helical/Cylindrical/Spherical/Planar/Fixed`（低副，`<joint type=…>` 的别名）与 `Gear`/`Cam`（高副）。它们定义在 `assets/kinematics-pairs.js`（随预置一起注入），只是把类型写进组件名、再展开成同样的 `{t:'joint'|'gear'|'cam'}` 节点，因此**代数核心与求解仍在 Rust**（`scene_build/kinematics.rs`），报告 / URDF / pose 覆盖不受影响。`q` 在 1-DOF 关节是数字、在多 DOF 关节是数组（cylindrical `[qr,qp]`，spherical / planar 三个数）。生成器 `jsx_gen::gen_pairs_showcase()` 一次输出全部 8 种关节 + 齿轮/凸轮高副（见 `demo_pairs`）。

v1 边界（显式不做）：`dist()` 查询、`echo`、CSS 后代/子代组合选择器。需要时各自单独立项。

**透明度与渲染模式**：材质的 `opacity`/`ior`/`absorption` 在**正常模式**下驱动 Whitted 反射/折射（`opacity<1` 的表面发射次级光线），半透明遮挡物按 `1-opacity` 削弱阴影；**忽略透明度模式**（`RenderMode::IgnoreOpacity`）把一切当不透明——不发射次级光线、透明遮挡物按实心投影，用于实体预览与提速。两种模式在不含透明度的场景上输出**逐位一致**（有测试看守）。

已平权的原 CGS 高级特性（JSX 形态）：

| 原 CGS | JSX |
| --- | --- |
| `constrain(x) { lhs == rhs; } solve;` | `const [x] = solve([x0], [v => eq(...), v => le(...)])`（数值残差 + Levenberg-GN） |
| `drill(r=…, through=…, axis=…)` | `<Drill r through axis from to />`（through 吃 tag 名或元素值） |
| `instances("n")` + `if(len(…)==k)` | `<Instances of="n"/>` 重放 + `<When of="n" count={k}>` 门控 |
| `face/fnrm/center/lo/hi/size/xdir…` | 同名惰性查询函数（prop 位置解析；向量运算用 `vadd/vsub/vscale`） |
| `--set` / 原 cgs_pose | `run_jsx_pose` + 全局 `P` 约定（`P.x` 读覆盖）+ 关节名覆盖 |

## 5. 验收

- 平权验收（R2 时点，CGS 尚在）：orbit/assembly/画廊六场景的 JSX 版与原 CGS 版渲染**逐字节相同**（96×72 PNG）。R4 后 CGS 文件已删除，测试现为 JSX 冒烟。
- React 迁移验收（P1）：React 路径与旧的一次性求值路径对 7 个画廊场景渲染**逐字节相同**；旧路径删除后固化为渲染金标（orbit / mechanical / assembly 三场景，覆盖透明、CSG、solve 约束 + tag/when + 嵌套元素属性）。
- 跨二进制核对：迁移前后两个 CLI 对 7 个场景（640×480 aa=2）输出逐位一致，端到端 +80…+240 ms/进程。
- React 运行时测试：hooks 求值、局部更新计数、keyed 身份稳定、effect 依赖与清理顺序、卸载清理、同输入同输出、作者错误（语法/运行时）回传。
- 多帧/事件测试：`SceneSession` 输入变化只让消费者平移（create=0、恰好 1 个对象移动）、`onClick` 派发使状态跨三帧累积（create=0）、同输入序列同输出。
- 组件 + hooks（状态/记忆/上下文/副作用）+ `map` + 三元控制流出 4 个对象。
- CSS 类命中材质（color/roughness 断言）。
- JSX 关节 + gear 推导 q，对象归属记录正确，报告含 joint/gear 行。
- 多 DOF 关节的 `q` 数组（cylindrical `[qr,qp]`、spherical/planar 三个数）；`gen_pairs_showcase` 覆盖全部 8 种关节 + 齿轮/凸轮高副，报告含全部 `type=` 与 gear/cam 行。
- 错误：JSX 语法错带行号；未知元素报 `unknown primitive frob`（复用 scene_build 文本）；缺 `export default` 显式报错。
- CLI：`render_jsx examples/jsx/orbit.jsx out.png 320 240 2` 出图正常。

## 6. 架构终态

.jsx`+`.css` 是唯一的场景作者格式。`scene_build` 是语义底座（builder、错误文本、报告），`jsx` 宿主把 React 已提交的实例树翻译成它的调用。下游（渲染/报告/URDF、STL 导出）只认 `SceneRun`。CGS 文本语法已删除（2026-10-06，R4）；一次性求值路径已由真 React 运行时取代（2026-10-07，P1）。

## 7. Rust ↔ JS 引擎桥

- **引擎隔离**：boa（纯 Rust，无 FFI）跑在专用线程（64 MB 栈）。跨线程只传 `Send` 命令闭包；`!Send` 的 `Context` 只活在引擎线程，所有对 JS 的接触都在那里。
- **统一下发**：`HostCall`（`src/react/mod.rs`）是唯一的方法/参数编码点；JS 侧只有 `__sess.call(payloadJson)` 一个入口，方法表 `HANDLERS` 在 `react-host.js`。新增宿主方法 = 两端各加一处，不再三处手写字符串调用。
- **结构化错误**：回传一律是信封字符串 `{"ok":true,"value":…}` / `{"ok":false,"kind","message","stack"}`，`kind` ∈ syntax/runtime/internal。Rust `parse_envelope` 解构，`ReactError` 保留 `kind`；引擎级异常对象永不跨边界。
- **帧事务**：`ReactSession::frame(FrameAction)` 一次完成动作（mount / input / event / none）+ 重渲染 + drain + 快照 + React 错误，单次跨线程；`SceneSession` 直接用返回的快照重建。
- **契约版本**：JS `__sess.schema()` 报告 `{t,p,c}` schema 版本，建会话时由 `SCENE_SCHEMA` 断言——两端 IR 漂移立即失败（渲染金标是第二道防线）。
- **原生回调**：`register_global` 注入 boa 原生函数（`solve`）；外层 `catch_unwind` 把 panic 转成 JS 异常，不让它跨 FFI 边界 unwind。
- **会话类型**：`ReactSession<Owned>`（长驻交互，`SceneSession`）与 `ReactSession<Pooled>`（线程本地池，`with_session`）在类型上区分，池只会存放 Pooled——长驻会话不可能被误复用。
- **场景预置**：`assets/scene-prelude.js`，编译期 `include_str!` 嵌入。
- **沙箱（可选）**：`CGA_SANDBOX=1`，或 `SessionOptions { sandbox: Sandbox::On }` / `run_jsx_pose_sandbox` / `SceneSession::open_pose_sandbox`。模块求值前冻结宿主能力全局（`__sess`/`React`/`CGA_*`/`console`…），每帧后删除作者新增的全局。这是**纵深防御而非硬边界**：单帧内作者仍能读到未冻结的宿主对象；彻底隔离需要独立 realm（后续项）。

桥测试（`src/react/mod.rs`）：信封错误分类（语法/运行时）、帧事务一次挂载、原生函数 panic 被捕获、沙箱冻结宿主全局并清理作者全局、非沙箱保留作者全局；`src/jsx/mod.rs`：沙箱下照常渲染。
