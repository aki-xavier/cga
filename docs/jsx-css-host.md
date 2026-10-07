<!-- markdownlint-configure-file {"MD013": false} -->
# JSX + CSS 场景宿主：React 模式前端

状态：已实现（2026-10-06，路线 A：真 JS；2026-10-07，P1：改用**真 React 运行时**；2026-10-07，帧间增量 §6：版本化 + 子树复用 + 像素级增量渲染）。适用：`crates/cga-host/src/jsx/`、`crates/cga-host/src/react/`、CLI `render_jsx` / `report_jsx` / `render_frames`。
R3/R4 已完成（2026-10-06）：生成管线（`jsx_gen`、`jsx_to_urdf`）改出 JSX，且使用与宿主一致的 PascalCase 元素名（`<Scene>` / `<Rotate>` / `<Difference>` …）；互操作**仅导出**（URDF / STL），不提供导入；文本语法解析器已删除，只剩 builder/kinematics/报告支撑（`scene_build`）。
决策记录：用户拍板"以 React + CSS 模式为主，本仓库红线（确定性错误契约/单遍/文本即真相）可以不管"。因此不走"降级为 IR"的保守路线，直接内嵌真 JS 引擎。

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
- CSS 是真匹配 + 真级联：选择器支持 `.class` / `#id` / `[attr]` / `*` / tag / `:root` 与后代、子代 `>`、相邻 `+`、普通 `~` 四类组合器，按「`!important`/内联层级 → 特异性 → 源码顺序」级联；材质键与 `--*` 变量沿元素树继承，`var(--x, fallback)` 在取值前文本替换。不支持的构造（交互伪类、伪元素、`@` 规则、CSS 嵌套、`calc()`、带单位的数值）报错并带原文，未知属性名静默忽略（与浏览器一致）。能力面、差距清单与阶段验收的完整记录见 `docs/css-conformance.md`。
- `import './x.css'` 由宿主跟随装载；`.jsx` 模块导入见 §2.1（编译期打包）；其他 import 报错。
- 材质键：`color roughness metalness emissive opacity ior absorption map`；JSX 元素可直接带这些 prop，也可由 CSS 类命中。
- 分组：`<Group name="…">` 元素或任意元素的 `group="…"` prop，子树产出的对象带分组 id（`SceneRun::groups` 注册表，`Object::group`）。分组是**缓存/失效与拾取的元数据单元，不是 z-index**——对象前后关系永远由求交决定。
- 驼峰/蛇形双写兼容（`driverProfile` ≡ `driver_profile`）。
- 碰撞查询（构建期惰性标量，`{__q}` 通道）：`clearance(a,b)` 分离距离、`collides(a,b)` / `inside(of,[x,y,z])` → 1/0，`qadd/qsub/qmul/qdiv` 组合（JS 算术对查询对象无效）。Unknown（不支持的几何对）报构建错误——三值不许变成数字。引用是先定义后使用的 tag 名或内联元素。场景级扫描见 `SceneSession::collisions()` 与报告的 `collide` 行。

## 2.1 .jsx 模块导入（2026-10-07）

场景可以由多个 `.jsx` 文件组成：入口文件 `import` 其他模块，导入的模块当子模块用——
默认导出是组件就 `<Dial />` 用，是元素值就 `{dial}` 用；命名导出 `import { a, b as c } from './m.jsx'`。

```jsx
// dial.jsx
export default function Dial(props) {
  return <translate t={[props.x || 0, 0, 0]}><sphere r={0.5} /></translate>;
}
// scene.jsx（入口）
import Dial from './dial.jsx';
export default (<scene><camera /><Dial x={2} /></scene>);
```

实现是**编译期打包**（`compile_jsx_with`，不走运行时模块加载）：依赖图 DFS
递归编译，每个被导入模块包成 `globalThis.__exp_N = (() => { …; return {default, …named}; })()`
（真 JS 函数作用域，模块间顶层名永不冲突），导入点改写为普通别名常量
（`const Dial = globalThis.__exp_N.default;`），依赖序后序排放、查重记忆化
（钻石依赖只求值一次）。入口照旧 `export default` → `const __scene`。

- **解析来源**两种：`run_jsx*` / `SceneSession::open*` 用 `asset_root` 下的磁盘文件；
  `SceneSession::open_modules(entry, css, &[(name, src)])` 用调用方给的内存模块表
  （编辑器多标签页场景，specifier `"./dial.jsx"` 与 `"dial.jsx"` 等价）。
- **bundle 期错误**（都带 `JSX:` 前缀，进同一条错误契约）：循环导入
  （`circular import: a.jsx -> b.jsx -> a.jsx`）、模块找不到（`cannot import`）、
  目标没有默认导出、目标没有该命名导出、`export *` / re-export（不支持）。
- 支持的导出形式：`export default <expr>`、`export default function/class`（具名或匿名）、
  `export const/function/class`、`export { a, b as c }`。
- 单文件语义不变：不含 `.jsx` import 的场景编译产物与打包器引入前完全一致。

## 3. React 能力边界

**可用**（真 React 提供，均有测试看守）：`useState useReducer useMemo useRef useContext useEffect`（依赖与清理顺序正确）、`memo`、`createContext`、`Fragment`、`key` 协调；`Suspense` / error boundary / `useTransition` / `useSyncExternalStore` / `ref` / `forwardRef` 的机制都在（需要时自行给数据或驱动）。

**协调语义**：只有变化的实例被重建 —— 子组件自身 `setState` → 新建 0 / 更新 1；keyed 插入 → 新建 1、其余实例身份不变；卸载跑副作用清理。

**确定性**：调度器的时间由宿主 `drain` 推进（微任务 + 定时器，有界轮数），"挂载 → 提交 → 副作用"在没有事件循环的情况下跑到不动点；同输入同输出（有测试）。

**"局部"的三段式落点**（帧间增量见 §6）：React 协调只重建变化的宿主实例；构建按子树版本整棵复用未变子树的产出对象；渲染只重追值可能变化的光线。三段都保守：拿不准就全量。

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
| `<Group name>` / `group="…"` prop | 分组：子树产出对象的缓存/失效/拾取单元（**不是**渲染层叠，见 §6 档 2） |

运动副也可用 **React 组件**写法：`Revolute/Continuous/Prismatic/Helical/Cylindrical/Spherical/Planar/Fixed`（低副，`<joint type=…>` 的别名）与 `Gear`/`Cam`（高副）。它们定义在 `assets/kinematics-pairs.js`（随预置一起注入），只是把类型写进组件名、再展开成同样的 `{t:'joint'|'gear'|'cam'}` 节点，因此**代数核心与求解仍在 Rust**（`scene_build/kinematics.rs`），报告 / URDF / pose 覆盖不受影响。`q` 在 1-DOF 关节是数字、在多 DOF 关节是数组（cylindrical `[qr,qp]`，spherical / planar 三个数）。生成器 `jsx_gen::gen_pairs_showcase()` 一次输出全部 8 种关节 + 齿轮/凸轮高副（见 `demo_pairs`）。

v1 边界（显式不做）：`echo`；CSS 侧的布局/盒模型、伪元素、交互伪类、`@` 规则（`@media` / `@import` / `@keyframes` 等）、CSS 嵌套、`calc()` 与数学函数、`style={{…}}`（用 JSX prop 承担）。需要时各自单独立项。CSS 的差距清单与已实施的收敛记录见 `docs/css-conformance.md`。（原 `dist()` 边界已由碰撞检测落地：`clearance()`/`collides()`/`inside()`，见 `docs/collision-plan.md`。）

**透明度与渲染模式**：材质的 `opacity`/`ior`/`absorption` 在**正常模式**下驱动 Whitted 反射/折射（`opacity<1` 的表面发射次级光线），半透明遮挡物按 `1-opacity` 削弱阴影；**忽略透明度模式**（`RenderMode::IgnoreOpacity`）把一切当不透明——不发射次级光线、透明遮挡物按实心投影，用于实体预览与提速。两种模式在不含透明度的场景上输出**逐位一致**（有测试看守）。

## 5. 验收

- 画廊冒烟：orbit/assembly 等场景均渲染出合法 PNG（96×72）。
- React 迁移验收（P1）：React 路径与旧的一次性求值路径对 7 个画廊场景渲染**逐字节相同**；旧路径删除后固化为渲染金标（orbit / mechanical / assembly 三场景，覆盖透明、CSG、solve 约束 + tag/when + 嵌套元素属性）。
- 跨二进制核对：迁移前后两个 CLI 对 7 个场景（640×480 aa=2）输出逐位一致，端到端 +80…+240 ms/进程。
- React 运行时测试：hooks 求值、局部更新计数、keyed 身份稳定、effect 依赖与清理顺序、卸载清理、同输入同输出、作者错误（语法/运行时）回传。
- 多帧/事件测试：`SceneSession` 输入变化只让消费者平移（create=0、恰好 1 个对象移动）、`onClick` 派发使状态跨三帧累积（create=0）、同输入序列同输出。
- 组件 + hooks（状态/记忆/上下文/副作用）+ `map` + 三元控制流出 4 个对象。
- CSS 验收（`test_css_*`，全集见 `docs/css-conformance.md` §8）：31 行选择器匹配表（复合多类 / 四类组合器 / `*` / `[attr]` / `:root` 根限定 / tag 大小写 / 组合链）、10 格级联矩阵（`#id` > `.class` > `tag` > `*`、源码顺序、`!important` 跨规则与压过内联普通声明）、值转换（`rgb`/`hsl`/具名/hex/八位 hex → opacity、百分比、`var()` 与 fallback、`--x` 别名、未知属性静默）、错误契约（`@` 规则、CSS 嵌套、伪类/伪元素、不支持的值与单位、缺失变量）、以及父规则声明向子几何的继承。
- JSX 关节 + gear 推导 q，对象归属记录正确，报告含 joint/gear 行。
- 多 DOF 关节的 `q` 数组（cylindrical `[qr,qp]`、spherical/planar 三个数）；`gen_pairs_showcase` 覆盖全部 8 种关节 + 齿轮/凸轮高副，报告含全部 `type=` 与 gear/cam 行。
- 错误：JSX 语法错带行号；未知元素报 `unknown primitive frob`（复用 scene_build 文本）；缺 `export default` 显式报错。
- 帧间增量（§6）：实例版本随创建 / props 更新 / 结构变更正确抬升（`react::tests::instance_versions_track_changes`）；复用构建与全量构建逐字段一致（`incremental_build_reuses_unchanged_subtrees`）；兄弟组合器样式表关闭复用（`incremental_build_disabled_by_sibling_rules`）；增量渲染与全帧渲染逐位一致（cga-gpu `test_incremental_*` ×5 + `session_render_incremental_is_bitexact`）；分组流向 `Object::group`（`group_prop_and_element`）。
- 碰撞（`docs/collision-plan.md` C0–C1）：`clearance`/`collides`/`inside` 惰性查询解析正确（含 `qadd` 组合），Unknown 报构建错误；`CollisionScan` 同组免检 + 跨帧指纹缓存（动一个对象只重算含它的对）；报告 `collide` 行；画廊 8 场景 Yes/Unknown 计数基线 + animation 太阳球陷入地面 0.05 的语义抽查。
- CLI：`render_jsx examples/jsx/orbit.jsx out.png 320 240 2` 出图正常；`render_frames` 逐帧动画走增量构建 + 增量渲染并打印每帧统计。

## 6. 帧间增量：版本化、子树复用与像素级增量渲染

三段各有独立的失效依据，全部保守（拿不准就全量）：

**档 2 · 分组（作者标注的失效/拾取单元）**。`<Group name>` 或 `group="…"` prop 把子树产出的对象打进同一个分组（`Object::group` → `SceneRun::groups` 的下标）。注册表 append-only，跨帧 id 稳定（复用的对象带旧 id）。分组**不是 z-index**：渲染器是光线追踪，前后关系由求交决定；分组只做缓存失效与拾取归属的元数据。

**档 0 · 子树复用（构建侧）**。React 实例带两个版本：自身版本 `__v`（创建 / props 更新 / 子节点增删移动时递增）与子树版本 `__s`（后代最大值），随快照下发（schema v2）。`annotate_reuse` 把「祖先链 `__v` 全不变（上下文：变换/继承样式/兄弟位置不变）+ 自身 `__s` 不变（子树内容不变）+ 子树纯（无灯光/相机/joint/tag/when/instances 等副作用元素，props 里无 `{__q}` 惰性查询）+ 不在 tag/joint/when 动态上下文之下」的子树标上上一帧的对象区间，`walk` 整棵克隆复用。三条保守闸：样式表含兄弟组合器（`+`/`~`——兄弟的 class 变化抬不动本节点版本号）时整个会话关闭复用；含惰性查询的节点会**污染路径**——它的输出随 tag 注册表漂移但版本不变，其子树一律不得复用（有测试看守：查询跟随者必须跟到目标的新位置）；CSG 块是最小复用单元（其子树不单独复用）。`SceneSession::build_stats()` 报告复用率；正确性由「复用构建 ≡ 全量构建逐字段相等」的测试看守。

**档 1 · 像素级增量（渲染侧）**。`IncrementalRenderer`（cga-gpu）缓存上一帧的光线级结果，帧间按对象指纹 diff：相机 / 灯光 / 背景 / 对象数变化 → 全帧；否则脏集 = 变化对象的新旧屏幕包围盒 ∪ 阴影级联（形状/透明度变化的对象在新旧位置上能投到的接收者；地面等无界平面解析阴影足迹，无法界定则全帧）∪ 透明级联（正常模式下场景含透明对象时，其像素全脏——反射/折射次级光线可达任意对象）；脏光线 ≥ 总数一半 → 全帧（沿用渲染器既有的 2× 回退）。子集追踪与全帧追踪同一光线取值逐位一致（光线互相独立，渲染器既有的剔除机制已依赖这一点），测试逐位断言看守；无变化帧直接返回上一帧图像（dirty=0）。`SceneSession::render_incremental(w,h,aa,mode)` 是会话侧入口；`render_frames` 示例已切到增量渲染并打印每帧统计（`animation.jsx` 320×240 aa=2：脏光线约 18–25%）。

明确不做：**没有 z-index 分层合成**。层间互相遮挡/投影/折射在光线追踪里无法按层拆开；帧间复用的单位是像素值与对象产出，不是"图层"。

## 7. 架构终态

.jsx`+`.css` 是唯一的场景作者格式。`scene_build` 是语义底座（builder、错误文本、报告），`jsx` 宿主把 React 已提交的实例树翻译成它的调用。下游（渲染/报告/URDF、STL 导出）只认 `SceneRun`。文本语法已删除（2026-10-06，R4）；一次性求值路径已由真 React 运行时取代（2026-10-07，P1）。

## 8. Rust ↔ JS 引擎桥

- **引擎隔离**：boa（纯 Rust，无 FFI）跑在专用线程（64 MB 栈）。跨线程只传 `Send` 命令闭包；`!Send` 的 `Context` 只活在引擎线程，所有对 JS 的接触都在那里。
- **统一下发**：`HostCall`（`src/react/mod.rs`）是唯一的方法/参数编码点；JS 侧只有 `__sess.call(payloadJson)` 一个入口，方法表 `HANDLERS` 在 `react-host.js`。新增宿主方法 = 两端各加一处，不再三处手写字符串调用。
- **结构化错误**：回传一律是信封字符串 `{"ok":true,"value":…}` / `{"ok":false,"kind","message","stack"}`，`kind` ∈ syntax/runtime/internal。Rust `parse_envelope` 解构，`ReactError` 保留 `kind`；引擎级异常对象永不跨边界。
- **帧事务**：`ReactSession::frame(FrameAction)` 一次完成动作（mount / input / event / none）+ 重渲染 + drain + 快照 + React 错误，单次跨线程；`SceneSession` 直接用返回的快照重建。
- **契约版本**：JS `__sess.schema()` 报告 `{t,p,c}` schema 版本，建会话时由 `SCENE_SCHEMA` 断言——两端 IR 漂移立即失败（渲染金标是第二道防线）。v2（2026-10-07）：每个节点带 `__v`（自身版本）/`__s`（子树版本），增量构建的复用依据（§6 档 0）。
- **原生回调**：`register_global` 注入 boa 原生函数（`solve`）；外层 `catch_unwind` 把 panic 转成 JS 异常，不让它跨 FFI 边界 unwind。
- **会话类型**：`ReactSession<Owned>`（长驻交互，`SceneSession`）与 `ReactSession<Pooled>`（线程本地池，`with_session`）在类型上区分，池只会存放 Pooled——长驻会话不可能被误复用。
- **场景预置**：`assets/scene-prelude.js`，编译期 `include_str!` 嵌入。
- **沙箱（可选）**：`CGA_SANDBOX=1`，或 `SessionOptions { sandbox: Sandbox::On }` / `run_jsx_pose_sandbox` / `SceneSession::open_pose_sandbox`。模块求值前冻结宿主能力全局（`__sess`/`React`/`CGA_*`/`console`…），每帧后删除作者新增的全局。这是**纵深防御而非硬边界**：单帧内作者仍能读到未冻结的宿主对象；彻底隔离需要独立 realm（后续项）。

桥测试（`src/react/mod.rs`）：信封错误分类（语法/运行时）、帧事务一次挂载、原生函数 panic 被捕获、沙箱冻结宿主全局并清理作者全局、非沙箱保留作者全局；`src/jsx/mod.rs`：沙箱下照常渲染。
