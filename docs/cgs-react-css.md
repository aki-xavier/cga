<!-- markdownlint-configure-file {"MD013": false} -->
# JSX + CSS 场景宿主：React 模式的 CGS 前端

状态：已实现（2026-10-06，路线 A：真 JS）。适用：`crates/cga-gpu/src/jsx/`、CLI `render_jsx`。
决策记录：用户拍板"以 React + CSS 模式为主，本仓库红线（确定性错误契约/单遍/文本即真相）可以不管"。因此不走"CGS 降级为 IR"的保守路线，直接内嵌真 JS 引擎。

## 1. 技术栈与管线

| 件 | 选型 | 职责 |
| --- | --- | --- |
| JSX → JS | swc（`swc_core`，classic 运行时，pragma `h`） | 解析 + 变换，错误带 span（报 `JSX line N: …`） |
| JS 执行 | boa（纯 Rust，无 IO 沙箱） | 真 JS 语义：函数组件、`map`、三元、`Math`、任意表达式 |
| CSS | lightningcss | 规则解析（tag / `.class` / `#id` / `:root` / `scene`） |
| 场景构建 | 复用 CGS 的 `build_geometry`/`build_material`/kinematics | 语义单一来源 |

管线：`.jsx` → swc 变换 → boa 执行（宿主 `h` 是纯 JS prelude）→ 元素树经 `JSON.stringify` 回传 → Rust 建 `CgsRun`（Scene + camera + tags + kinematics，报告管线原样可用）。

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
| `<Sphere r Plane Box Cylinder Circle Cone Torus Cyclide Ellipsoid Bezier Extrude Loft Mesh>` | 图元（参数校验复用 CGS builder） |
| `<Translate t>` `<Rotate axis angle>` `<Scale s>` `<Mirror axis>` | 修饰符（组合 4×4 ctx） |
| `<Material …>` | 材质合并作用于子树 |
| `<Union/Difference/Intersection>` | CSG 块（≥2 子几何） |
| `<AmbientLight/DirectionalLight/PointLight/Camera/Background>` | 灯光与相机 |
| `<Joint name type axis at rpy q limit pitch>` | P1 关节（嵌套成父子树） |
| `<Gear driver driven ratio offset>` | P2 齿轮耦合 |
| `<Cam driver driven driverProfile drivenProfile>` | P3 凸轮接触求解（profile 是普通对象 `{kind:"circle",c,n,r}` / `{kind:"plane",n,d}`） |
| `<Tag name>` | 标签注册表 |

v1 边界（显式不做）：`drill`、`instances()` 集合选择、`constrain` 方程块、pose 覆盖、CSS 后代/子代组合选择器。需要时各自单独立项。

## 4. 验收

- `test_jsx_orbit_parity_with_cgs`：`examples/jsx/orbit.jsx` + `orbit.css` 与 `examples/cgs/orbit.cgs` 渲染**逐字节相同**（96×72 PNG）。
- 函数组件 + `map` + 三元控制流出 4 个对象。
- CSS 类命中材质（color/roughness 断言）。
- JSX 关节 + gear 推导 q，mesh 归属记录正确，报告含 joint/gear 行。
- 错误：JSX 语法错带行号；未知元素报 `unknown primitive frob`（复用 CGS 文本）；缺 `export default` 显式报错。
- CLI：`render_jsx examples/jsx/orbit.jsx out.png 320 240 2` 出图正常。

## 5. 与 CGS 的关系

CGS 保留为底层语义来源（builder、错误文本、报告）。JSX 是并列的前端宿主，不是 IR 翻译——两条路径在同一个 `CgsRun` 上汇合，下游（渲染/报告/bake/URDF）无差别。错误契约在 JSX 侧降级为尽力而为（boa/swc 消息 + 行号），CGS 侧契约不变。
