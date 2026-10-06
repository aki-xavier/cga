<!-- markdownlint-configure-file {"MD013": false} -->
# JSX + CSS 场景宿主：React 模式前端

状态：已实现（2026-10-06，路线 A：真 JS）。适用：`crates/cga-host/src/jsx/`、CLI `render_jsx` / `report_jsx`。
R3/R4 已完成（2026-10-06）：生成管线（`jsx_gen`、`jsx_to_urdf`）改出 JSX；互操作**仅导出**（URDF / STL），不提供导入；CGS parser 已退役删除（`scene_lang` → `scene_build`，只剩 builder/kinematics/报告支撑），CGS 文本语法不再被解析。
决策记录：用户拍板"以 React + CSS 模式为主，本仓库红线（确定性错误契约/单遍/文本即真相）可以不管"。因此不走"CGS 降级为 IR"的保守路线，直接内嵌真 JS 引擎。

## 1. 技术栈与管线

| 件 | 选型 | 职责 |
| --- | --- | --- |
| JSX → JS | swc（`swc_core`，classic 运行时，pragma `h`） | 解析 + 变换，错误带 span（报 `JSX line N: …`） |
| JS 执行 | boa（纯 Rust，无 IO 沙箱） | 真 JS 语义：函数组件、`map`、三元、`Math`、任意表达式 |
| CSS | lightningcss | 规则解析（tag / `.class` / `#id` / `:root` / `scene`） |
| 场景构建 | 复用 `scene_build` 的 `build_geometry`/`build_material`/kinematics | 语义单一来源 |

管线：`.jsx` → swc 变换 → boa 执行（宿主 `h` 是纯 JS prelude）→ 元素树经 `JSON.stringify` 回传 → Rust 建 `SceneRun`（Scene + camera + tags + kinematics，报告管线原样可用）。

## 2. 约定

- 场景 = 文件末尾的 `export default <element>`。
- 内置元素 PascalCase 或全小写均可（`<Sphere r={1}/>` ≡ `<sphere r={1}/>`）。
- 函数组件首字母大写，按 React 约定调用。
- CSS 只认单层选择器（tag / `.class` / `#id` / `:root` / `scene`），材质属性沿元素树继承，内联 prop 优先于 CSS 规则，CSS 按源码顺序级联。
- `import './x.css'` 由宿主跟随装载；其他 import 报错。
- 材质键：`color roughness metalness emissive opacity ior absorption map`；JSX 元素可直接带这些 prop，也可由 CSS 类命中。
- 驼峰/蛇形双写兼容（`driverProfile` ≡ `driver_profile`）。

## 3. 元素 → 语义映射

| JSX | 语义 |
| --- | --- |
| `<Sphere r Plane Box Cylinder Circle Cone Torus Cyclide Ellipsoid>` | 图元（参数校验复用 scene_build builder）。网格/曲面（mesh/bezier/extrude/loft）与烘焙已随网格支持一并移除 |
| `<Translate t>` `<Rotate axis angle>` `<Scale s>` `<Mirror axis>` | 修饰符（组合 4×4 ctx） |
| `<Material …>` | 材质合并作用于子树 |
| `<Union/Difference/Intersection>` | CSG 块（≥2 子几何，纯解析图元的递归布尔） |
| `<AmbientLight/DirectionalLight/PointLight/Camera/Background>` | 灯光与相机 |
| `<Joint name type axis at rpy q limit pitch>` | P1 关节（嵌套成父子树） |
| `<Gear driver driven ratio offset>` | P2 齿轮耦合 |
| `<Cam driver driven driverProfile drivenProfile>` | P3 凸轮接触求解（profile 是普通对象 `{kind:"circle",c,n,r}` / `{kind:"plane",n,d}`） |
| `<Tag name>` | 标签注册表 |

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

## 4. 验收

- 平权验收（R2 时点，CGS 尚在）：orbit/assembly/画廊六场景的 JSX 版与原 CGS 版渲染**逐字节相同**（96×72 PNG）。R4 后 CGS 文件已删除，测试现为 JSX 冒烟。
- 函数组件 + `map` + 三元控制流出 4 个对象。
- CSS 类命中材质（color/roughness 断言）。
- JSX 关节 + gear 推导 q，对象归属记录正确，报告含 joint/gear 行。
- 错误：JSX 语法错带行号；未知元素报 `unknown primitive frob`（复用 scene_build 文本）；缺 `export default` 显式报错。
- CLI：`render_jsx examples/jsx/orbit.jsx out.png 320 240 2` 出图正常。

## 5. 架构终态

`.jsx`+`.css` 是唯一的场景作者格式。`scene_build` 是语义底座（builder、错误文本、报告），`jsx` 宿主把 JS 执行结果翻译成它的调用。下游（渲染/报告/URDF 互转）只认 `SceneRun`。CGS 文本语法已删除（2026-10-06，R4）。
