<!-- markdownlint-disable MD033 -->
<!-- markdownlint-configure-file {"MD013": false} -->
# cga — 共形几何代数（CGA）

cga 是一个 5D 共形几何代数引擎。纯 Rust 实现。它包含三部分：

- 代数核心：CPU 运行，float64 精度，32 分量 multivector。
- 渲染引擎：GPU 运行（MLX/Metal，经 `mlx-rs`）。API 命名对齐 three.js。
- CGS 场景语言：OpenSCAD 风格。

cga 把欧氏 3D 嵌入共形空间（基 `{e1, e2, e3, e0, e∞}`）。点、线、面、圆、球和刚体运动（motor）是同一个代数的元素。场景中每个对象是一个 blade。相机位姿是一次 versor 共轭。渲染是 GPU 上的批量射线-blade 求交。

<p align="center"><img src="examples/engine/orbit.gif" width="580" alt="demo_engine 轨道动画"></p>
<p align="center"><sub><code>demo_engine</code> 轨道动画：地面、红球、蓝球、金柱、绿盒、紫圆盘、折射玻璃球。平行光 + 点光 + 环境光。硬阴影。aa=2 超采样。</sub></p>

## 渲染画廊

八张 CGS 场景（`examples/cgs/*.cgs`）。每张图由一条 `render_cgs` 命令生成：

<table>
  <tr>
    <td align="center" width="50%">
      <img src="examples/cgs/examples_cgs/orbit.png" width="440" alt="orbit.cgs 渲染图"><br>
      <sub><b>orbit.cgs</b> — 折射玻璃球 + 反射/漫反射多材质。测试金样场景。</sub>
    </td>
    <td align="center" width="50%">
      <img src="examples/cgs/examples_cgs/grid.png" width="440" alt="grid.cgs 3×3 球阵"><br>
      <sub><b>grid.cgs</b> — <code>module</code> + <code>for</code> 的 3×3 球阵。</sub>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="examples/cgs/examples_cgs/building.png" width="440" alt="building.cgs 砖纹建筑"><br>
      <sub><b>building.cgs</b> — CSG 开真窗洞。<code>map=</code> 贴砖纹。</sub>
    </td>
    <td align="center" width="50%">
      <img src="examples/cgs/examples_cgs/mechanical.png" width="440" alt="mechanical.cgs 机械件"><br>
      <sub><b>mechanical.cgs</b> — 节圆阵列钻孔。沉头锥孔。环面垫圈。齿阵列。</sub>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="examples/cgs/examples_cgs/primitives.png" width="440" alt="primitives.cgs 图元全家福"><br>
      <sub><b>primitives.cgs</b> — 图元全家福：sphere·box·cylinder·cone（blade）+ torus·tube·ellipsoid·cyclide·circle（射线逆变换）。</sub>
    </td>
    <td align="center" width="50%">
      <img src="examples/cgs/examples_cgs/affine.png" width="440" alt="affine.cgs 仿射扩展"><br>
      <sub><b>affine.cgs</b> — 非均匀 <code>scale</code>。<code>mirror</code> 镜像对：偏心孔和角标球随体翻转。</sub>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="examples/cgs/examples_cgs/assembly.png" width="440" alt="assembly.cgs 关联装配"><br>
      <sub><b>assembly.cgs</b> — v2/v3 汇演：<code>constrain</code> 解孔位。<code>drill</code> 贯穿。<code>face</code> 挂立柱。<code>instances</code> 门控顶梁。全程方法链。</sub>
    </td>
    <td align="center" width="50%">
      <img src="examples/cgs/examples_cgs/freeform.png" width="440" alt="freeform.cgs 自由曲面"><br>
      <sub><b>freeform.cgs</b> — 双三次 Bézier 曲面罩。厚壳曲面开槽（CSG 叶）。</sub>
    </td>
  </tr>
</table>

| 互操作 | 运动学 |
| --- | --- |
| <img src="examples/helmet/demo_helmet.png" width="360" alt="DamagedHelmet glTF 载入渲染"><br><sub>载入 glTF <code>DamagedHelmet.glb</code> 并渲染。</sub><br><img src="examples/gltf/demo_gltf.png" width="360" alt="extrude L 形 glTF 往返渲染"><br><sub>extrude L 形 → 写 <code>.glb</code> → 读回 → 渲染。</sub> | <img src="examples/kinematics/kinematics.gif" width="360" alt="demo_kinematics 运动学动画"><br><sub><code>demo_kinematics</code> — 齿轮副（16:8 → −1:2）。曲柄滑块。螺旋路径 <code>M(s)=M₀·exp(s·log(M₀⁻¹M₁))</code>。全部用 motor 直写。</sub> |

## 架构

![cga 架构图](docs/cga-architecture.svg)

- cga 建模用 **blade，不用三角网格**。球和圆柱没有细分数。尺寸就是构造参数。每帧 Rust 层只循环约十个图元。逐像素计算全部在 GPU 上一次批量完成。
- 代数核心恒为 CPU float64。远原点的共形抵消不受 float32 限制。`mlx-rs` 只用于逐像素内核（`renderer` / `geometry_ops` / `shading`）。标量助手 `s_add`/`s_mul`/… 由 `mlxops` 提供。

## 要求

- Rust stable 1.8x 或更高版本。
- macOS，Apple Silicon。

注意：首次构建会编译一次 MLX C++ 核心。之后的构建使用缓存。

## 快速开始

1. 运行测试：

   ```bash
   make test
   ```

   结果：236 个测试全部通过。

2. 渲染冒烟场景：

   ```bash
   make run
   ```

   结果：生成 `render_smoke.png`。

3. 格式化代码：

   ```bash
   make fmt
   ```

## 特性总表

| 层 | 内容 |
| --- | --- |
| **CGA 核心** | 32 分量 multivector（纯 `[f64; 32]`）。motor versor 变换（gp/reverse/log/velocity）。exp/log/插值。关联判据两种：直接形式 `op`，对偶形式 `ip`。 |
| **渲染引擎** | three.js 命名：Scene / PerspectiveCamera / Mesh / Sphere·Plane·Cylinder·Box·Circle Geometry / MeshStandard Material / Ambient·Directional·Point Light / Renderer.render / OrbitControls。对象 = blade。变换 = motor 共轭。`Renderer::new(w, h, aa, n)` 超采样。 |
| **复杂建模** | **CSG**：递归真布尔（crossings/contains 实体协议）。**仿射扩展**：scale/mirror 射线逆变换 + Newton 极分解。**新图元**：cone/torus（`arc<2π` 即 tube 弧环管）/ellipsoid/cyclide。**网格**：Möller–Trumbore 批量求交 + extrude/loft + OBJ/glTF/GLB。 |
| **MLX GPU** | 每像素向量化解析求交。全分辨率单帧一次 kernel 批量（mlx-rs / Metal）。相机空间：X 右、Y 下、Z 前。 |

## CGS 场景语言（OpenSCAD 风格）

场景示例：

```text
material(color=0xB0B0B0, roughness=0.7) plane(n=[0, 1, 0], d=0);
translate([0, 1, 0])
  material(color=0xC0392B, roughness=0.25, metalness=0.25) sphere(r=1);

directional_light(direction=[0.4, 1.0, 0.35], intensity=0.38);
camera(fov=50, position=[0, 2.4, 6.2], target=[0, 0.8, 0]);
```

渲染场景文件：

```bash
cargo run --release -p cga-examples --bin render_cgs -- examples/cgs/orbit.cgs orbit.png 640 480 2
```

规则：

- 修饰符 `translate/rotate/scale/mirror/material` 作用于紧随的语句或 `{}` 块。修饰符可以嵌套。
- 图元：`sphere/plane/cylinder/box/circle/cone/torus/cyclide/ellipsoid/bezier/extrude/loft/mesh`。
- `bezier(points=16个[x,y,z], thickness=0, div=8)`：`thickness=0` 是渲染面。`thickness>0` 是水密厚壳，可进 CSG 和烘焙。
- `torus(R, r, arc=2π)`：`arc<2π` 时是部分弧环管（tube）。
- 语言能力：变量、表达式、数学函数、`for`+`range`、`module`、`if-else`、`echo`、CSG 运算符 `union/difference/intersection`。完整语法见 `crates/cga-gpu/src/scene_lang.rs` 文件头。
- 列表取分量用 `comp(list, i)`。不支持 `a[i]` 索引。

### 错误契约

错误文本是确定的逐行文本。LLM 可以直接断言。

语句分发是单遍的。顺序如下：**赋值最先**（变量可以遮蔽关键字，`echo = 5;` 合法）→ `module/for/if/echo/show/tag/drill/var/constrain` → CSG → 语句修饰语 → 属性语句 → 图元。`background = 0x2B3138;` 与 `background(color=0x2B3138);` 等价。

| 错位情形 | 规范错误文本 |
| --- | --- |
| 语句关键字/修饰语/属性语句出现在表达式中 | `CGS line N: {name} is a statement and cannot be used in an expression` |
| 表达式中出现赋值（`y = x = 2`） | `CGS line N: assignment is a statement and cannot be used in an expression` |
| 表达式函数用作语句（`len([1,2]);`） | `CGS line N: {name} is an expression function and cannot be used as a statement` |
| 索引记号 `a[…]` | `CGS line N: indexing is not supported — use comp(vector, index)` |
| 集合选择拿到非引用名实参 | `CGS line N: instances needs a reference name, got {v}` |
| constrain 方程体关系符错位 / 多顶层关系 | `CGS line N: constrain does not support != — use ==, <= or >=` · `CGS line N: one relation per constrain equation` |
| 未知语句名（`blah();`） | `CGS line N: unknown primitive {name}` |

### 能力矩阵

| 阶段 | 能力 | 语义 |
| --- | --- | --- |
| **v2** | 几何值与稳定引用 | `g = box(…); show(g);` 是表达式几何值。`tag("n") stmt` 注册具名实例。查询：`center/lo/hi/size/dist/xdir/ydir/zdir`。 |
| **v2** | 派生跟随 (G3) | `drill(r=…, through=…, axis=…)` 是贯穿切割器。轴向范围取目标包围盒。改厚度时孔自动跟随。`from/to` 可为数字或 `"名:key"` 面引用。 |
| **v2** | 关系式定位 | 表达式 `at/rot/scaled(g, …)`、`polar(r, a)`、`comp(v, i)`。 |
| **v2 P2** | 编译期约束求解 | `var x = …; constrain(x) { lhs == rhs; … } solve;`。Levenberg 阻尼 Gauss–Newton。语句执行时刻解出，烘焙回 scope。不收敛 = 编译错误。 |
| **v2 P3** | 面引用 | `face(x, "+z")` 是面心。`fnrm(x, "+z")` 是法向。box/圆柱/圆锥/球/椭球精确。供装配与 URDF 挂点。 |
| **v3 P4** | 后缀方法链 | `g.at([1,0,0]).rot([0,1,0], 45)` ≡ `rot(at(g, [1,0,0]), [0,1,0], 45)`。接收者插为第一实参。纯去糖。错误文本逐字继承。语句位识别为表达式语句。 |
| **v3 P5** | 集合选择 `instances()` | `instances("hole") → List[Geom]`。`len` 计数。`for` 遍历。`if` 过滤。`center/lo/hi` 聚合。不引入 SELECT/FROM/WHERE 表面语法。 |
| **v3 P6** | 不等式约束 | 方程体 `== / <= / >= / < / >` 进同一 GN 最小二乘。等式残差 `u−v`。不等式 hinge `max(0, u−v)`：满足时残差与梯度为 0，只裁剪可行域。`!=` 拒绝。不可行仍显式报 `did not converge`。 |
| **Report** | 执行结果文本 | `cgs_report(text, asset_root)` / `report_cgs` CLI：场景级 background/camera/灯行 + 每对象一行 `object <i> …` + `bounds` + 几何参数树 + tag 注册表 + `summary`。数值六位规范化。帧唯一化。错误契约 **+0 行**。 |

详见 [`docs/cgs-v2.md`](docs/cgs-v2.md)、[`docs/cgs-v3.md`](docs/cgs-v3.md)、[`docs/scene-report.md`](docs/scene-report.md)。画廊 `assembly.cgs`（32 行，`examples/cgs/assembly.cgs`）联合汇演 v2/v3 全部特性：`constrain` 解孔位 → `drill` 贯穿 → `face` 面心挂立柱 → `instances` 计数架顶梁。

## CGA 建模 vs 传统欧氏建模

| 维度 | 传统欧氏建模 (three.js/网格) | 本项目 CGA 建模 |
| --- | --- | --- |
| **几何表示** | 三角网格。球/圆柱靠细分数逼近。 | 隐式 blade 解析方程。精确。无细分数。 |
| **尺寸/精度** | 细分数决定精度。凑近可见棱角。 | 尺寸 = 构造参数。任意距离渲染一致。 |
| **变换机制** | 4×4 矩阵连乘。浮点误差破坏正交性。 | `X' = M·X·M̃`。任意 motor 乘积仍是 motor。逆 = reverse。 |
| **相机** | 独立 view/projection 矩阵。 | 相机位姿也是 motor。 |
| **渲染管线** | 光栅化：顶点着色 → 插值 → z-buffer。 | 光线追踪：每像素对 blade 解析求交。 |
| **统一性** | 几何/变换/渲染三套机制。 | 点线面圆球与刚体运动同属一个 5D 代数。 |
| **动画** | 矩阵无插值语义。需分解 + 四元数。 | `Motor.exp/log` 直接插值。可直接提取速度二重向量。 |

**三个直接后果：**

| <img src="examples/advantage/advantage_a.png" width="300" alt="无多边形对比图"><br>① 无多边形 | <img src="examples/advantage/advantage_b.png" width="300" alt="无限几何对比图"><br>② 无限几何 | <img src="examples/advantage/advantage_c.png" width="300" alt="变换同构对比图"><br>③ 变换同构 |
| --- | --- | --- |
| 球/圆柱无多边形。渲染质量不随相机距离恶化。 | 无限平面、无限圆柱是代数对象自身属性。无需裁剪（b 图圆柱直抵地平线）。 | motor 与 blade 是同类对象。没有矩阵–四元数–轴角换算层。 |

## 复杂建模能力

**CSG 真布尔** — `difference()` / `intersection()` 递归组合。可以任意嵌套。内核收集子树的全部边界穿越点，逐区间测试成员关系，取最近的实体表面。叶子 = 全部实体图元 + `plane` 半空间。半空间交 = 剖切视图。

![CSG 布尔并排：并 / 差 / 交](examples/csg/demo_csg.png)

**新图元** — cone（凸体区间裁剪）/ torus（Durand–Kerner 解四次方程；`arc<2π` 时是部分弧环管 tube，CGS 与 Rust API 同源）/ ellipsoid（仿射缩放球）/ cyclide（Dupin cyclide 四次曲面）。这四者不是 CGA blade。它们经射线逆变换接入。图见画廊 `primitives.cgs`。

**仿射扩展** — scale/mirror 经 AffineGeometry 射线逆变换接入。versor 无法表达这两类变换。法向走逆置变换。det<0 时镜像自动正确。上下文是全 4×4 仿射。几何落点经 Newton 极分解为 motor·linear。`rotate` 与 `scale/mirror` 任意嵌套顺序都正确。图见画廊 `affine.cgs` 的镜像对。

**网格与互操作** — `MeshGeometry`：Möller–Trumbore 批量求交。平坦法向。无 BVH。`modeling.rs`：extrude（耳切凹轮廓三角化）与 loft（等点数多截面）。`mesh_io.rs`：纯 stdlib OBJ 读写。`mesh_io_gltf.rs`：glTF/GLB 读写（节点变换/层级/材质色）。

**自由曲面（P3）** — `bezier(points=16, thickness, div)` 是有理双三次 Bézier 补丁。图见画廊 `freeform.cgs` 的曲面罩与曲面开槽。它的 `crossings`/`contains`/`field`/`bounds` 复用网格 MT 内核（均匀 `div×div` 剖分，解析弦高界在测试中断言）。`thickness>0` 时缝合水密偏置厚壳（top/bottom/侧壁共享索引，全边 ×2 断言），是真实体，可进 CSG 与烘焙。`thickness=0` 时是渲染面。CSG 与烘焙显式拒绝它（与 `circle` 同族）。完整示例见 `examples/cgs/freeform.cgs`。实测约束：CSG×网格内存按 O(射线 × 交点 × 三角) 增长。区间分类器对每条射线采样 (k+1) 个点。`contains` 已按 256 MB 临时量预算分块求值，防止 OOM。高 `div` 必须配低分辨率/aa。

**烘焙** — `bake` 把任意 CSG 隐式体变成三角网格。**纯 CPU float64，无 GPU 依赖。Linux/CI 可运行。** 流程：有符号场（与 GPU `*_contains` 同号）→ marching tetrahedra（Kuhn 六四面体剖分）→ 位精确顶点焊接 + 分量级一致定向。**水密是构造性结论，不是运气。** `topology_report()` 把边界边、非流形边、欧拉示性数放进 CI 断言。场值非有限时显式报错：导出不许弃权。下图是 `demo_bake` 输出。左：隐式 CSG 原件（解析光滑）。右：烘焙网格（可见刻面）。

![隐式 CSG 与烘焙网格并排](examples/bake/demo_bake.png)

```text
$ cargo run --release -p cga-examples --bin demo_bake
topology: TopologyReport { vertices: 40868, faces: 81744, degenerate_faces: 0,
  boundary_edges: 0, nonmanifold_edges: 0, inconsistent_edges: 0, euler: -4 }
volume: 1.3911
saved examples/bake/demo_bake.png + demo_bake.obj
```

```bash
cargo run --release -p cga-examples --bin bake_cgs -- examples/cgs/mechanical.cgs out.obj 0.1
```

注意：粗 step 会丢失细于 step 的特征。相切/共面退化继承 CSG 采样语义。烘焙网格适合仿真碰撞与预览。精确 STEP 导出不在范围内。

## 生成 / 无头渲染

这是给程序化调用方（LLM codegen、外部 GUI）的库级入口。`demo_lang` 一次跑通三件事：生成 → 无头渲染 → 场景报告。

![gen_flange_assembly 生成并无头渲染的法兰装配](examples/lang/demo_lang.png)

```rust
// CGS 生成：结构化参数 → 逐行平铺源码（无 for、确定性、可 diff、必然可解析）
let text = cga_gpu::gen_flange_assembly(
    &FlangeSpec::default(), &BoltCircleSpec::default(),
    &GearSpec::default(), &BasePlateSpec::default());

// 无头渲染：CGS 文本 → PNG 字节。无窗口，无 CLI。
let out = cga_gpu::render_cgs_png(&text, ".", 640, 480, 2)?;
std::fs::write("preview.png", out.png)?;

// 场景报告：执行结果的确定性逐行文本。LLM Verifier 可直接断言。
let report = cga_gpu::cgs_report(&text, ".")?;

// 网格烘焙：CSG → 三角网格（纯 CPU f64）
let m = world_params.bake(0.1)?;
let v = m.volume();
```

`examples/lang/report.txt`（`demo_lang` 产物）节选。每个对象一行几何参数树 + 包围盒。数值六位规范化：

```text
scene version=1
camera(fov=45, aspect=1.777778, position=[7.5,6.5,9.5], target=[0,1.8,0], ...);
object 1 material(...) difference(frame(t=[0,0.55,0], axis=[-1,0,0], angle=1.570796, cylinder(r=2.6, h=0.5)), ...);
bounds 1 lo=[-2.6,-2.3,-2.6] hi=[2.6,3.4,2.6]
...
summary objects=41 lights=3 no_bounds=1 bbox_lo=[-3.5,-2.3,-3.5] bbox_hi=[3.5,5.55,3.5]
```

## 场景代码

```rust
use cga_core::*;
use cga_gpu::*;

let mut scene = Scene::new(None);
scene.add_mesh(Mesh::new(MeshParams {
    geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
    material: Material::standard(MaterialParams {
        color: Color::from_hex(0xB0B0B0), roughness: 0.7, metalness: 0.0,
        emissive: Color::from_hex(0x000000), opacity: 1.0, ior: 1.5, absorption: 0.0,
    }),
    position: [0.0, 0.0, 0.0], rotation_axis: [0.0, 0.0, 1.0],
    rotation_angle: 0.0, motor: None,
}));                                          // 地面：对偶平面 blade (y=0)
scene.add_mesh(Mesh::new(MeshParams {
    geometry: Geometry::SphereGeometry(SphereGeometry::new(1.0)),
    material: Material::standard(MaterialParams {
        color: Color::from_hex(0xC0392B), roughness: 0.25, metalness: 0.25,
        emissive: Color::from_hex(0x000000), opacity: 1.0, ior: 1.5, absorption: 0.0,
    }),
    position: [0.0, 1.0, 0.0], rotation_axis: [0.0, 0.0, 1.0],
    rotation_angle: 0.0, motor: None,
}));                                          // 球：对偶球 blade。半径即尺寸。
scene.add_light(Light::directional(Color::from_hex(0xFFFFFF), 0.38, [0.4, 1.0, 0.35]));
scene.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.34));

let mut camera = PerspectiveCamera::new(50.0, 4.0 / 3.0, 0.1, 100.0,
    [0.0, 2.4, 6.2], [0.0, 0.8, 0.0], [0.0, 1.0, 0.0]);
camera.look_at([0.0, 0.8, 0.0], None);

let mut renderer = Renderer::new(360, 270, 2, 3);
let img = renderer.render(scene, camera);       // (H, W, 4) uint8 RGBA
save_frame_png("out.png", &img);
```

签名见 `crates/cga-gpu/src/scene.rs` / `renderer.rs` / `shading.rs`。

## 双目渲染

`stereo_pair` 用两个相机渲染严格校正的左右图。两个相机朝向完全相同，只差一个基线。同时输出几何真值：

<table>
  <tr>
    <td align="center" width="50%"><img src="examples/stereo/left.png" width="440" alt="双目左图"><br><sub>左图 <code>left.png</code></sub></td>
    <td align="center" width="50%"><img src="examples/stereo/right.png" width="440" alt="双目右图"><br><sub>右图 <code>right.png</code>。仅 x 向基线位移。</sub></td>
  </tr>
</table>

同时输出 `truth.txt`：焦距、基线、每物件深度与视差。相机空间约定与 r3d 的 `Depth` 对齐（左图在前；经校正的视差只在 x 上）。`stereo` / `depth` 可直接读取。

```bash
cargo run --release -p cga-examples --bin stereo_pair -- 7 examples/stereo 480 360 0.10
```

## 已知限制（与 three.js 的差距）

- **阴影**：每光源一条遮挡射线（硬阴影）。无软阴影。无后处理。无 tonemap。无 envMap/IBL。高 metalness 材质会显黑。demo 因此压低金属度。
- **纹理**：`material(map=...)` 做解析 UV 采样。无 mipmap。无过滤控制。
- **scale/mirror**：经 AffineGeometry 射线逆变换接入。versor 无法表达。blade 语义（meet/关联判据）不适用于仿射形变后的图元。
- **CSG**：分类用交点间区间（无 δ 采样）。相切/重根走判别式精确路径。判不出时显式报 `Unknown`，不静默猜测（见 `docs/freeform-robust-boolean.md` P0–P4）。节点单材质。`circle` 非实体。
- **非 blade 图元**：cone/torus/ellipsoid/cyclide/网格经射线逆变换接入。cyclide 环型是光滑亏格-1 曲面。尖型自交，CSG 成员性语义退化。
- **网格**：暴力 O(N·F)。无 BVH。平坦法向。无纹理坐标。glTF 导入暂限单 primitive。内外分类用广义绕数：开网格与整体反向可用；混合朝向无意义。CSG 复合时 `contains` 按 256 MB 临时量预算分块求值（防 OOM，代价是逐块 GPU 同步）。`crossings` 尚未分块。
- **精度**：代数核心恒为 CPU float64。渲染内核为 float32（MLX/Metal 无 float64）。参数进入相机空间后接近原点。

## 机器人领域潜在应用

![CGA 机器人应用](docs/cga-robotics.svg)

| 方向 | 依托 |
| --- | --- |
| 仿真与合成数据 | 光线追踪合成深度/RGB。改视角只需重设相机 motor。适合域随机化批量生成（带精确深度真值）。 |
| 运动学与轨迹 | motor 是 SE(3) 的 versor 表示。`exp`/`log`/`velocity_bivector`/`interpolate` 已有。`motor = exp(s·log(M₀·M₁⁻¹))` 生成平滑刚体路径。 |
| 几何感知输出 | 图元 blade 即操作对象语义。平面法向 = 抓取姿态 z 轴。圆柱轴线+半径 = 夹爪开度。 |
| 重建回环验证 | 重建场景渲染回原视角。与原帧深度对比可发现漂移。 |
| 统一坐标变换 | 相机、机械臂、工件共用同一代数。多坐标系链式变换收敛到一个表示。 |

## 项目布局

```text
crates/
  cga-core/                 纯 f64 CPU 代数核心
    multivector / tables    32 分量 MVP + GP 积表（gen_tables.py 生成）
    motors / primitives     motor、图元关联判据与距离
    cyclide / geometry      Dupin cyclide、Geometry 与相机参数
    affine / affine_geom    仿射扩展（scale/mirror 逆变换 + Newton 极分解）
    csg_node / modeling     CSG 树、耳切三角化 + extrude + loft
    bake / mesh_io / gif    网格烘焙（marching tetrahedra）、OBJ、GIF89a 编码
  cga-gpu/                  mlx-rs/Metal GPU 内核
    mlxops / shading        标量广播助手、材质灯光与批量 Blinn-Phong
    scene / scene_graph     Mesh·Scene·PerspectiveCamera·OrbitControls、Vec3/Color
    geometry_ops / geometry_extra / geom_kernels   blade 解析求交（含 cone/torus/ellipsoid/cyclide）
    trimesh / csg / certify 求交、递归 CSG、可证明求根与区间分类
    texture / image_io / shading / mesh_raster     纹理、PNG、CPU 光栅合成
    mesh_io_gltf            glTF/GLB 读写
    scene_lang / scene_report / cgs_gen / headless CGS 语言、报告、生成、无头渲染
    renderer                mlx-rs GPU 批量光线追踪（SSAA/硬阴影/Whitted 折射）
  cga-examples/             演示 CLI（src/bin/*.rs）
examples/                   .cgs 场景（含 primitives/affine/assembly/freeform）+ assets 纹理 + 各 demo 输出图（README 插图；bake/lang 为 P4 与生成管线汇演）
docs/                       架构图、机器人图、cgs-v2/v3、scene-report、freeform-robust-boolean
```

演示 CLI（`cargo run --release -p cga-examples --bin <name>`）：

| CLI | 输出 |
| --- | --- |
| `demo_engine [frames]` | `examples/engine/orbit.gif` |
| `demo_advantage` | `examples/advantage/advantage_{a,b,c}.png` 三面板 |
| `demo_kinematics` | `examples/kinematics/kinematics.gif` |
| `demo_csg` | `examples/csg/demo_csg.png`（并/差/交并排） |
| `demo_bake` | `examples/bake/demo_bake.{png,obj}`（隐式 CSG 与水密烘焙网格并排） |
| `demo_lang` | `examples/lang/{generated_flange.cgs, demo_lang.png, report.txt}`（生成 → 无头渲染 → 报告） |
| `demo_gltf` | `examples/gltf/demo_gltf.{glb,png}` |
| `demo_helmet` | `examples/helmet/demo_helmet.png` |
| `render_cgs <file.cgs> [out.png] [w h aa]` | CGS → PNG |
| `bake_cgs <file.cgs> [out.obj\|out.glb] [step]` | CGS → 三角网格（自动跳过无界平面） |
| `report_cgs <file.cgs>` | CGS → 场景报告（stdout 逐行可断言文本；错误走 stderr，exit 1） |
| `stereo_pair [seed] [out_dir] [w h] [baseline]` | `left.png` / `right.png` / `truth.txt` 双目对 |

## 质量

- `make test`：**236 个测试全部通过**（cga-core 62 + cga-gpu 174，无 `#[ignore]`）。覆盖：代数恒等式、图元关联判据、versor·exp-log 往返、抗锯齿、引擎渲染定量、CSG、仿射、新图元、cyclide、网格互操作（绕数分类：开网格、整体反向、分块一致性）、CGS 全阶段（v2 关联查询、drill 面引用、constrain 求解、语句边界错误契约；v3 P4–P6 金样）、场景报告、自由曲面退化用例库 + 区间分类 + 可证明求根 + Bézier 补丁叶（求值/弦高界/水密壳/CSG/烘焙体积/CGS 错误契约）、烘焙体积与水密拓扑（边界边/非流形边/欧拉示性数）金样。
- 渲染金样写入 `artifacts/tests/`（gitignored）。sphere/cone/ellipsoid/cyclide/torus/textured_box/helmet/csg 金样 **RMSE = 0**。

## License

MIT。见 [LICENSE](LICENSE)。
