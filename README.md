# cga — 共形几何代数 (Conformal Geometric Algebra) 实验场

5D 共形几何代数核心 + three.js 风格渲染引擎 + MLX/Metal GPU 批量光线追踪。
**本项目为纯 Rust 实现**，GPU 计算依赖 [`mlx-rs`](https://github.com/oxideai/mlx-rs)
（Apple MLX 的 Rust 绑定，Metal 后端）。

把欧氏 3D 空间嵌入共形空间（基 `{e1, e2, e3, e0, e∞}`），点 / 线 / 面 / 圆 /
球与刚体运动 (motor) 统一为代数元素：场景里的每一个对象都是一个 CGA blade，
相机是一次 versor 共轭，渲染就是对 blade 的 GPU 批量求交。

## 渲染结果

`demo_engine` 的轨道动画（地面 + 红/蓝球 + 金柱 + 绿盒 + 紫圆盘 +
折射玻璃球，平行光 + 点光 + 环境光，硬阴影，`aa=2` 超采样抗锯齿）：

![轨道渲染 demo](examples/engine/orbit.gif)

重新生成（`encode_gif_rgba` 直接合成 GIF，不落 PNG，见 `crates/cga-core/src/gif.rs`）：

```bash
cargo run --release -p cga-examples --bin demo_engine -- 90
```

## 特性

| 层 | 内容 |
| --- | --- |
| **CGA 核心** | 32 分量 multivector（纯 `[f64; 32]`，CPU 双精度）；Motor versor 变换 (gp/reverse/log/velocity)；exp/log/插值；直接形式 `op` 与对偶形式 `ip` 两种关联判据 |
| **渲染引擎** | three.js 命名 API：Scene / PerspectiveCamera / Mesh / Sphere·Plane·Cylinder·Box·Circle Geometry / MeshStandard Material / Ambient·Directional·Point Light / Renderer.render / OrbitControls；场景对象 = CGA blade，变换 = Motor 共轭；超采样抗锯齿 `renderer(w, h, aa, n)` |
| **复杂建模** | **CSG**（union/difference/intersection 递归真布尔，实体协议 crossings/contains）；**仿射扩展**（scale/mirror 射线逆变换 + Newton 极分解）；**新图元** cone/torus/ellipsoid/cyclide；**网格**（Möller–Trumbore 批量求交 + extrude/loft + OBJ/glTF/GLB 互操作） |
| **MLX GPU** | 每像素向量化解析求交，全分辨率单帧一次 kernel 批量（mlx-rs / Metal）；相机空间 X 右 / Y 下 / Z 前 |

## 快速开始

要求：Rust stable（1.8x+）、macOS Apple Silicon。`mlx-rs` 首次构建会编译
MLX C++ 核心（一次性，约几分钟），之后全部走缓存：

```bash
make test     # cargo test --workspace（208 个测试全过）
make run      # 渲染 smoke 场景 → render_smoke.png
make fmt      # cargo fmt --all
```

## CGS 场景语言 (OpenSCAD 风格)

`examples/cgs/orbit.cgs`（与上面的演示场景逐位等价，测试有金样断言）：

```text
material(color=0xB0B0B0, roughness=0.7) plane(n=[0, 1, 0], d=0);
translate([0, 1, 0])
  material(color=0xC0392B, roughness=0.25, metalness=0.25) sphere(r=1);

directional_light(direction=[0.4, 1.0, 0.35], intensity=0.38);
camera(fov=50, position=[0, 2.4, 6.2], target=[0, 0.8, 0]);
```

修饰符（`translate`/`rotate`/`scale`/`mirror`/`material`）对紧随的语句或 `{}`
块生效，可嵌套；图元：sphere/plane/cylinder/box/circle/cone/torus/cyclide/
ellipsoid/extrude/loft/mesh。渲染：

```bash
cargo run --release -p cga-examples --bin render_cgs -- examples/cgs/orbit.cgs orbit.png 640 480 2
```

支持变量/表达式/数学函数/for+range/module/if-else/echo，以及 CSG
（union/difference/intersection）；完整语法见 `crates/cga-gpu/src/scene_lang.rs`
文件头注释。示例：`examples/cgs/grid.cgs`（module + for 的 3×3 球阵）、
`examples/cgs/building.cgs`（CSG 开窗建筑）、`examples/cgs/mechanical.cgs`
（CSG 钻孔装配）。

**语句/表达式边界与错误契约** — 语句分发是单遍的，顺序：赋值
（`name = expr;`，**最先**检查 ⇒ 任何关键字都可被变量遮蔽，如 `echo = 5;`、
`for = 6;`）→ `module/for/if/echo/show/tag/drill/var/constrain` → CSG →
语句修饰语（`translate/rotate/scale/mirror/material`，缺目标 → `modifier
missing target statement`）→ 属性语句（`background/camera/*_light`）→
图元语句。`background` 两种写法等价：`background = 0x2B3138;`（属性赋值，
之后可用 `bg = background;` 读回）与 `background(color=0x2B3138);`。

语句与表达式错位时按组给出确定错误文本（LLM Verifier 可直接断言）：

| 错位情形 | 规范错误文本 |
| --- | --- |
| 语句关键字/修饰语/属性语句出现在表达式中（`g = show(…)`、`g = translate(…)`、`g = camera(…)`） | `CGS line N: {name} is a statement and cannot be used in an expression` |
| 表达式中出现赋值（`y = x = 2`） | `CGS line N: assignment is a statement and cannot be used in an expression` |
| 数学/查询函数用作语句（`len([1,2]);`、`center("x");`） | `CGS line N: {name} is an expression function and cannot be used as a statement` |
| 索引记号 `a[…]`（语句或表达式位置） | `CGS line N: indexing is not supported — use comp(vector, index)` |
| 集合选择拿到非引用名实参（`instances(5)`） | `CGS line N: instances needs a reference name, got {v}` |
| constrain 方程体关系符错位（`x != 1`；`a <= b == c` 多顶层关系） | `CGS line N: constrain does not support != — use ==, <= or >=`；多关系 → `CGS line N: one relation per constrain equation` |
| 未知语句名（`blah();`） | `CGS line N: unknown primitive {name}` |

P4 方法链（`docs/cgs-v3.md`）**+0 行**：`g.show()`、`g.at()`、`g.5` 等链式拼法
经同一分发落回本表与 `expect`/`geom_val` 的既有文本（等价金样与
`assert_eq(链式, 函数式)` 逐字对照断言继承）。

Scene Report（`docs/scene-report.md`）是全函数，本表 **+0 行**：报告生成不产生
任何诊断文本；`report_cgs` CLI 的 `eprintln`+`exit(1)` 属 CLI 层，不进语言契约。

列表取分量用 `comp(list, i)`（不支持 `a[i]` 索引）。

**CGS v2 关联/求解能力**（`docs/cgs-v2.md`；加法改造，既有 `.cgs` 与既有测试
逐位不动）：

- **几何值与稳定引用** — `g = box(…); show(g);` 表达式几何值（帧无关绑定，
  `show` 时套用语句帧）；`tag("name") stmt` 注册具名实例；`center/lo/hi/
  size/dist/xdir/ydir/zdir` 查询（Str 实例 / Geom 值分派）。
- **派生跟随（G3）** — `drill(r=…, through=…, axis=…)` 贯穿切割器，轴向范围
  取目标包围盒（改厚度孔自动跟随）；`from/to` 可为数字或 `"名:key"` 面引用
  （`drill(r=0.2, from="base:+z", to="top:-z");` 端面到端面贯穿、可跨实例，
  `axis` 由面键推断）。
- **关系式定位** — 表达式 `at/rot/scaled(g, …)`、`polar(r, a)`、`comp(v, i)`。
- **编译期约束求解（P2）** — `var x = …; constrain(x) { lhs == rhs; … } solve;`
  Levenberg 阻尼 Gauss–Newton，语句执行时刻解出、烘焙回 scope（不收敛 = 编译
  错误）；方程须直接引用未知量表达式。
- **面引用（P3）** — `face(x, "+z")` 面心 / `fnrm(x, "+z")` 法向：box/圆柱/
  圆锥/球/椭球精确，其余 AABB 面心退化；供装配/URDF 挂点——标签绑在构造节点
  上，布尔重算不产生欧氏 CAD 的拓扑命名漂移。

**CGS v3 P4 后缀方法链**（`docs/cgs-v3.md`；纯语法糖，去糖后走既有分发）—
`g.at([1,0,0]).rot([0,1,0], 45)` ≡ `rot(at(g, [1,0,0]), [0,1,0], 45)`：接收者
插为第一实参，方法名后就是普通调用（`expect(lparen)` → 实参 → 同一三路分发），
边界/未知函数/变元数/类型错误逐字继承既有文本；语句位识别为表达式语句
（`box(…).at(…);`、`g.at(…);` → `expr → geom_val → ; → add_geometry`，与 CSG
表达式形式同一条渲染路），关键字语句（`for (…)`、`difference(a, b);` 等）不受
影响；`1.5` 等浮点词法不变。

**CGS v3 P5 集合选择 `instances()`**（`docs/cgs-v3.md` §4；集合是一等值，
不引入 SELECT/FROM/WHERE 表面语法）— `instances("hole") → List[Geom]`（元素帧 =
世界帧、顺序 = 注册表发射序），`len` 计数、`for` 遍历、`if` 过滤、既有
`center/lo/hi` 聚合并存，元素可直接作 `drill through=` 实参。SQL 对照：
`SELECT * → instances("t")`、`COUNT(*) → len(…)`、`WHERE → for + if`、
`UPDATE → 拒绝`（正向替代见 §7）；语句位命中 expression-only 契约。错误契约
**+1 行**（`instances needs a reference name, got {v}`）；可选子项一并落地：
`face("plate:+z")` 单串面字面量 ≡ `face("plate", "+z")`（零新增文本）。

**CGS v3 P6 不等式约束**（`docs/cgs-v3.md` §5；解算器侧，语法零新面）—
`constrain(x) { x <= 5; … } solve;` 方程体顶层关系符 `== / <= / >= / < / >`
分类进同一 GN 最小二乘：等式残差 `u−v`，不等式 hinge `max(0, u−v)`（满足 ⇒
残差与梯度同为 0，只裁剪可行域、不牵引解）；`>=`/`>` 解析期翻转为 `<=`、`<`
视同闭包，向量逐分量。`!=` 拒绝、多顶层关系命中
`one relation per constrain equation`（错误契约 **+1 行**）；不可行仍显式
`did not converge`，不静默折中。

**Scene Report 执行结果文本报告**（`docs/scene-report.md`；语言的**输出轴**，
与 v2/v3 输入轴正交）— `cgs_report(text, asset_root)` / `report_cgs` CLI 把执行后
的场景格式化为**确定性逐行文本**，供 LLM 逐行断言结果：

- 场景级 `background`/`camera`（七键照实）/灯行 + 每对象
  `object <i> <帧前缀> material(…) <几何>;` + `bounds <i>` 行；
- 几何参数树按 CGS 形打印（`sphere(r=…)`、`box(s=2·half)`、无界 `cylinder(h=-1)`、
  CSG 名 = 存储序递归），帧键进 `frame(t=…, axis=…, angle=…, lin=…)` token、
  恒等键与恒等帧整体省略；数值统一六位规范化（`pi/2 → 1.570796`、任意零 → `0`），
  旋转经矩阵→四元数提取、`angle ∈ [0, π]` 唯一化；
- tag 注册表段（名按字典序、实例按发射序，即 P5 `instances()` 读取的那份数据）与
  `summary`（对象/灯/无包围盒计数 + 全场 bbox 并集）；
- API：`cgs_run_result → CgsRun { scene, camera, tags }`（`cgs_load*` 改为委托、
  签名与行为零变）；错误契约 **+0 行**。

## 复杂建模能力

面向建筑/机械参数化建模的完整工具链：

**CSG 真布尔** — `difference()` / `intersection()` 递归组合（任意嵌套）：
收集子树全部边界穿越点 → 逐段成员测试（δ 双侧采样）→ 最近实体表面。叶子 =
全部实体图元（sphere/box/cylinder/cone/torus/ellipsoid/extrude/loft/mesh 与
plane 半空间 —— 半空间交 = 剖切视图）：

![CSG 布尔并排](examples/csg/demo_csg.png)

**建筑：CSG 开窗 + 纹理** — `examples/cgs/building.cgs`：参数化板式办公楼，
砖墙用 `difference()` 开真窗洞，`material(map=...)` 贴砖纹。

**机械：CSG 钻孔装配** — `examples/cgs/mechanical.cgs`：法兰螺栓节圆阵列真钻孔 +
中心孔、沉头锥孔、环面垫圈、径向齿阵列。

**新图元** — cone（凸体区间裁剪）/ torus（Durand-Kerner 复数迭代解四次方程）/
ellipsoid（= 仿射缩放球）/ **cyclide**（Dupin cyclide，四次曲面，Durand-Kerner
求根）。四者非 CGA blade，经射线逆变换接入；cyclide 模型见
`crates/cga-core/src/cyclide.rs`。

**仿射扩展** — scale/mirror 经 AffineGeometry 射线逆变换（非 versor 可达；法向
走逆置变换，det<0 镜像自动正确）。上下文为全 4×4 仿射，几何落点 Newton 极分解
为 motor·linear，rotate 与 scale/mirror 任意嵌套顺序均正确。

**网格与互操作** — MeshGeometry（Möller–Trumbore 批量求交，平坦法向，无 BVH）；
`modeling.rs` 的 extrude（耳切凹轮廓三角化）与 loft（等点数多截面）；
`mesh_io.rs` 纯 stdlib OBJ 读写，`mesh_io_gltf.rs` glTF/GLB 读写
（节点变换/层级/材质色）。

**精度** — 代数核心（multivector/motor/图元）是纯 CPU `[f64; 32]` 双精度，
远原点 conformal 抵消不受 float32 精度限制。渲染内核仍是 float32
（MLX/Metal 无 float64，参数进相机空间后 near-origin）。

**运动学** — `demo_kinematics`：齿轮副（16:8 齿数比 → 角速度比精确
−1:2）、曲柄滑块、螺旋轨迹 `M(s) = M₀·exp(s·log(M₀⁻¹M₁))` —— 全部 Motor 直写，
无矩阵分解/四元数换算层，直接合成 `examples/kinematics/kinematics.gif`（不落 PNG）：

![运动学 demo](examples/kinematics/kinematics.gif)

## 生成 / 无头渲染 / 网格烘焙

给程序化调用方（LLM codegen、ForgeCAD 等外部 GUI）的三个库级入口：

**CGS 生成**（`crates/cga-gpu/src/cgs_gen.rs`）— 结构化参数 → CGS 源码，
输出逐行平铺无 `for`（确定性、可 diff、必然可解析，生成即回环测试）：

```rust
let text = cga_gpu::gen_flange_assembly(
    &FlangeSpec::default(), &BoltCircleSpec::default(),
    &GearSpec::default(), &BasePlateSpec::default(),
); // -> background+camera+lights, 底板 4 沉头孔, 法兰 8 孔, 齿轮, 垫圈...
```

**无头渲染**（`crates/cga-gpu/src/headless.rs`）— CGS 文本 → PNG 字节，
无窗口无 CLI，供 Verifier / 外部 GUI 预览：

```rust
let out = cga_gpu::render_cgs_png(&text, ".", 640, 480, 2)?;
std::fs::write("preview.png", out.png)?;
```

**网格烘焙**（`crates/cga-core/src/bake.rs`）— 任意 CSG 隐式体 → 三角网格，
**纯 CPU f64 无 GPU 依赖**（Linux/CI 可用）：有符号场（与 GPU `*_contains`
同号约定）→ marching tetrahedra（Kuhn 六四面体拆分）→ 数值梯度定向法向。

```rust
let m = cga_core::bake(&world_params, 0.1)?;   // vertices + faces
let v = cga_core::mesh_volume(&m.vertices, &m.faces); // 体积自检
```

CLI：`cargo run --release -p cga-examples --bin bake_cgs -- examples/cgs/mechanical.cgs out.obj 0.1`
（step 可调；地面等无界平面自动跳过）。OBJ/GLB 复用 `mesh_io` / `save_glb`。
限制：分辨率换质量（细于 step 的特征丢失）；相切/共面退化继承 CSG 采样语义；
trimesh 场 O(F)/点。烘焙 mesh 适合仿真碰撞与预览，STEP 精确导出不在其列。

## 场景代码

```rust
use cga_core::*;
use cga_gpu::*;

let mut scene = scene(None);
scene.add_mesh(mesh(MeshParams {
    geometry: Geometry::PlaneGeometry(plane_geometry([0.0, 1.0, 0.0], 0.0)),
    material: standard_material(MaterialParams {
        color: color_hex(0xB0B0B0), roughness: 0.7, metalness: 0.0,
        emissive: color_hex(0x000000), opacity: 1.0, ior: 1.5, absorption: 0.0,
    }),
    position: [0.0, 0.0, 0.0], rotation_axis: [0.0, 0.0, 1.0],
    rotation_angle: 0.0, motor: None,
}));                                          // 地面: 对偶平面 blade (y=0)
scene.add_mesh(mesh(MeshParams {
    geometry: Geometry::SphereGeometry(sphere_geometry(1.0)),
    material: standard_material(MaterialParams {
        color: color_hex(0xC0392B), roughness: 0.25, metalness: 0.25,
        emissive: color_hex(0x000000), opacity: 1.0, ior: 1.5, absorption: 0.0,
    }),
    position: [0.0, 1.0, 0.0], rotation_axis: [0.0, 0.0, 1.0],
    rotation_angle: 0.0, motor: None,
}));                                          // 球: 对偶球 blade, 半径即尺寸
scene.add_light(directional_light(color_hex(0xFFFFFF), 0.38, [0.4, 1.0, 0.35]));
scene.add_light(ambient_light(color_hex(0xFFFFFF), 0.34));

let mut camera = perspective_camera(50.0, 4.0 / 3.0, 0.1, 100.0,
    [0.0, 2.4, 6.2], [0.0, 0.8, 0.0], [0.0, 1.0, 0.0]);
camera.look_at([0.0, 0.8, 0.0], None);

let mut renderer = renderer(360, 270, 2, 3);
let img = renderer.render(scene, camera);       // (H, W, 4) uint8 RGBA
save_frame_png("out.png", &img);
```

（精确签名见 `crates/cga-gpu/src/scene.rs` / `renderer.rs` / `shading.rs`。）

## 架构

![cga 架构图](docs/cga-architecture.svg)

关键设计：图元级（blade）建模而非三角网格 —— 球/圆柱没有细分数，尺寸全部在
geometry 构造参数里；像素级计算全部在 mlx-rs GPU 上批量进行，Rust 层每帧只循环
图元（~10 个）。代数核心跑在 CPU 的 `[f64; 32]` 上（比 float32 更准），
`mlx-rs` 只用在逐像素渲染内核（`renderer.rs` / `geometry_ops.rs` / `shading.rs`），
标量助手 `s_add`/`s_mul`/`s_clip`… 自由函数由 `mlxops.rs` 提供。

## CGA 建模 vs 传统欧氏建模（渲染视角）

| 维度 | 传统欧氏建模 (three.js/网格) | 本项目 CGA 建模 |
| --- | --- | --- |
| **几何表示** | 三角网格：球/圆柱靠细分数逼近，永远是多边形近似 | 隐式 blade：球/圆柱/平面/圆/盒的解析方程，精确无细分数 |
| **尺寸/精度** | 细分数决定精度与内存；距离近了能看到面片棱角 | 尺寸 = geometry 构造参数（如 `sphere_geometry(1.0)`），任意距离渲染一致 |
| **变换机制** | 4×4 矩阵（平移+旋转分算），连乘浮点误差破坏正交性 | Motor versor 共轭 `X' = M·X·M̃`；任意 motor 乘积仍是 motor，逆 = reverse |
| **相机** | 独立的 view/projection 矩阵 | 相机 pose 也是 Motor，与物体变换同机制 |
| **渲染管线** | 光栅化：顶点着色器投影 → 片段插值 → z-buffer | 光线追踪：每像素对 blade 解析求交，mlx-rs GPU 逐像素批量 |
| **统一性** | 几何、变换、渲染是三套独立机制 | 点/线/面/圆/球与刚体运动同属一个 5D 代数，关联判据 op/ip、求交 meet 统一 |
| **动画/插值** | 矩阵无直接插值语义，需分解位置+四元数 | `Motor.exp/log`：直接插值 motor、提取速度二重向量 |

实际后果：

- **球/圆柱无多边形** —— 渲染质量不随相机距离恶化；代价是隐式几何没有顶点/拓扑，
  网格编辑类建模工具用不上。
- **无限几何天然成立** —— 无限平面、无限圆柱是代数对象本身的属性，无需裁剪。
- **变换与几何同构** —— motor 与 blade 是同一类对象，没有矩阵-四元数-轴角换算层。
- **代价在别处** —— 解析求交的渲染内核仍是 float32；有限圆柱带端盖；无限圆柱/平面
  在相机位于退化位形时需内核特殊处理。

## 范围声明 (与 three.js 的差距)

如实标注：

- 阴影：每光源一条遮挡射线（硬阴影）；无软阴影。无后处理 / tonemap。
- 纹理：`material(map=...)` 解析 UV；无 mipmap/过滤控制。
- scale/mirror：经 AffineGeometry 射线逆变换（非 versor 可达）；blade 语义
  （meet/关联判据）不适用于仿射形变后的图元。
- CSG：相切/共面退化配置依赖 δ=1e-4 双侧采样；CSG 节点单材质；circle 非实体。
- cone/torus/ellipsoid/cyclide/网格非 CGA blade，经射线逆变换接入。
- cyclide：环型 (c<d<a) 是光滑亏格-1 曲面；尖型自交，CSG 成员性语义退化。
- 网格：暴力 O(N·F) 无 BVH；平坦法向；无纹理坐标；glTF 导入暂限单 primitive。
- 精度：代数核心恒为 CPU float64；渲染内核 float32。
- 无 envMap/IBL：高 metalness 材质会显黑，demo 因此压低金属度。

## 机器人领域潜在应用

CGA 建模 + Motor + GPU 光线追踪 + 逆渲染回环：

![CGA 机器人应用](docs/cga-robotics.svg)

- **仿真与合成数据**：光线追踪合成深度/RGB，逐像素 GPU 批量，改视角只需重设相机
  Motor —— 适合感知训练数据的域随机化批量生成（带精确深度真值）。
- **运动学与轨迹**：Motor 是 SE(3) 的 versor 表示，已有 `exp`/`log`/
  `velocity_bivector`/`interpolate`；`motor = exp(s·log(M₀·M₁⁻¹))` 生成平滑刚体
  路径，无需矩阵分解。
- **几何感知输出**：图元 blade 本身就是操作对象语义（平面法向 = 抓取姿态 z 轴、
  圆柱轴线+半径 = 夹爪开度）。
- **重建回环验证**：重建的 blade 场景渲染回原视角（`renderer.rs`），与原帧深度对比
  发现漂移。
- **统一坐标变换**：相机、机械臂、工件全用同一代数，多坐标系链式变换收敛到一个
  表示。

## 项目布局

```text
Cargo.toml                 # workspace（4 个 crate）
crates/
  cga-core/                # 纯 f64 CPU 代数核心
    src/multivector.rs     32 分量多重向量 (gp/ip/op/reverse/dual/meet/norm)
    src/tables.rs          GP 积表（由 gen_tables.py 从积表定义重新生成）
    gen_tables.py          积表生成器（一次性脚本，输出 tables.rs）
    src/motors.rs          Motor: rotor/translator/exp/log/interpolate/velocity
    src/primitives.rs      图元: point/point_pair/line/plane/sphere/circle/cylinder + 距离
    src/cyclide.rs         Dupin cyclide 模型（球族包络 + versor 反演）
    src/geometry.rs        Geometry 类型 + 相机空间参数结构
    src/affine.rs + affine_geom.rs
                           仿射扩展（scale/mirror 射线逆变换 + Newton 极分解）
    src/csg_node.rs        CSG 树节点（union/difference/intersection）
    src/modeling.rs        耳切三角化 + extrude + loft
    src/bake.rs            有符号隐式场 + marching tetrahedra 网格烘焙 (纯 CPU)
    src/mesh_io.rs         OBJ 读写 + 4x4 矩阵助手
    src/gif.rs             动画 GIF89a 编码（中位切分配色 + LZW，纯 stdlib）
  cga-gpu/                 # mlx-rs/Metal GPU 内核
    src/mlxops.rs          标量广播助手（fs/s_add/s_clip/…，原 mlx_ops 模块）
    src/scene_graph.rs     Vec3/Color/Object3D
    src/scene.rs           Mesh/Scene/PerspectiveCamera/OrbitControls
    src/geometry_ops.rs + geometry_extra.rs + geom_kernels.rs
                           blade 几何批量解析求交（含 cone/torus/ellipsoid/cyclide）
    src/trimesh.rs         Möller–Trumbore 批量网格求交
    src/csg.rs             递归 CSG 布尔（crossings/contains 实体协议）
    src/shading.rs         材质/灯光 + 批量 Blinn-Phong
    src/texture.rs         纹理解码 + bilinear map 采样
    src/image_io.rs        PNG 读写（image crate）
    src/renderer.rs        mlx-rs GPU 批量光线追踪（SSAA/硬阴影/Whitted 折射）
    src/mesh_raster.rs     CPU 网格光栅化（与光线追踪层深度合成）
    src/mesh_io_gltf.rs    glTF/GLB 读写（save_glb / load_gltf）
    src/scene_lang.rs      CGS 场景语言（lexer + 单遍 parser/evaluator）
    src/scene_report.rs    Scene Report：执行结果确定性逐行文本（docs/scene-report.md）
    src/cgs_gen.rs         CGS 文本生成 (参数 → 源码, LLM codegen 靶)
    src/headless.rs        无头渲染 (CGS 文本 → PNG 字节)
  cga-examples/            演示 CLI（src/bin/*.rs，见下）
examples/                  .cgs 示例 (cgs/orbit/grid/building/mechanical) + cgs/assets/ 纹理
                           + 各 demo 输出金样图（README 插图）
artifacts/tests/           测试金样图（cgs_orbit / cone / cyclide / ...，gitignored）
docs/                      架构图 / 机器人应用图 (svg)
```

演示 CLI（`cargo run --release -p cga-examples --bin <name>`）：

- `demo_engine` —— 轨道动画 → `examples/engine/orbit.gif`（可选帧数参数，默认 90）
- `demo_advantage` —— 无多边形/无限几何/变换同构三面板 → `advantage_{a,b,c}.png`
- `demo_kinematics` —— 齿轮副/曲柄滑块/螺旋插值 → `examples/kinematics/kinematics.gif`
- `demo_csg` —— difference/intersection/union 并排 → `examples/csg/demo_csg.png`
- `demo_gltf` —— extrude L 形 → 存 `.glb` → 重载 → 渲染 → `demo_gltf.{glb,png}`
- `demo_helmet` —— DamagedHelmet.glb 加载渲染 → `examples/helmet/demo_helmet.png`
- `render_cgs <file.cgs> [out.png] [w h aa]` —— CGS→PNG CLI
- `bake_cgs <file.cgs> [out.obj|out.glb] [step]` —— CGS→三角网格烘焙（无界面自动跳过）
- `report_cgs <file.cgs>` —— CGS→Scene Report（stdout 逐行可断言文本，错误 stderr + exit 1）
- `stereo_pair [seed] [out_dir] [w h] [baseline]` —— 随机三维场景的**双目渲染**：两个朝向完全相同、
  只沿 x 差一个基线的相机，产出一对严格校正的左右图（`left.png` / `right.png`）与几何真值
  （`truth.txt`：焦距、基线、每物件的深度与视差）。供 r3d 的 `stereo` / `depth` 直接读取；
  相机空间约定与 r3d 的 `Depth` 对齐（左图在前、经校正视差只在 x 上）

## 质量

- `make test`（`cargo test --workspace`）：208 个测试全过 —— 代数恒等式 /
  图元关联判据 / versor 往返 / exp-log 往返 / 距离公式 / 抗锯齿 / 引擎渲染定量 /
  CSG 布尔 / 仿射 / 新图元 / cyclide / 网格与互操作 / CGS / CGS v2（关联查询、
  drill 面引用、constrain 求解、语句边界错误契约）/ CGS v3（P4 后缀方法链：
  等价金样、错误继承、语句位链；P5 集合选择：顺序金样、计数条件、聚合互通；
  P6 不等式约束：hinge 满足/违约金样、混合可行域、关系符错误）/
  Scene Report（报告格式化、确定性、帧规范化）/
  自由曲面布尔退化用例库 + 交点区间分类 + 可证明求根（判别式精确符号、
  四次根 DK 初值 + 区间符号/单调认证）/
  位移曲面烘焙 / CGS 生成回环 / 无头渲染 / 网格烘焙（体积金样）
  （cga-core 46 + cga-gpu 162；另有 1 条退化用例 `#[ignore]`，P4 网格分类落地后清空）。
- 测试会把渲染金样图写到 `artifacts/tests/`（cgs_orbit / cone / cyclide /
  ellipsoid / sphere / textured_box / torus / trimesh）。
- 渲染结果与金样逐像素一致（sphere/cone/ellipsoid/cyclide/torus/textured_box/
  helmet/csg 金样 RMSE = 0）。

## License

MIT，见 [LICENSE](LICENSE)。
