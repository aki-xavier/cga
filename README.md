# cga — 共形几何代数 (Conformal Geometric Algebra) 实验场

5D 共形几何代数核心 + three.js 风格渲染引擎 + MLX/Metal GPU 批量光线追踪。
**本项目已整体移植到 V (vlang)**，依赖 [`mlx-v`](https://github.com/) 绑定
（`~/code/mlx-v`）；原 Python 实现已移除。

把欧氏 3D 空间嵌入共形空间（基 `{e1, e2, e3, e0, e∞}`），点 / 线 / 面 / 圆 /
球与刚体运动 (motor) 统一为代数元素：场景里的每一个对象都是一个 CGA blade，
相机是一次 versor 共轭，渲染就是对 blade 的 GPU 批量求交。

## 渲染结果

`examples/demo_engine.v` 的轨道动画（地面 + 红/蓝球 + 金柱 + 绿盒 + 紫圆盘 +
折射玻璃球，平行光 + 点光 + 环境光，硬阴影，`aa=2` 超采样抗锯齿）：

![轨道渲染 demo](examples/artifacts/orbit.gif)

重新生成（直接由 V 合成 `examples/artifacts/orbit.gif`，不落 PNG，见 `gif.v`）：

```bash
v -gc boehm run examples/demo_engine.v 90
```

## 特性

| 层 | 内容 |
| --- | --- |
| **CGA 核心** | 32 分量 multivector（纯 `[32]f64`，CPU 双精度）；Motor versor 变换 (gp/reverse/log/velocity)；exp/log/插值；直接形式 `op` 与对偶形式 `ip` 两种关联判据 |
| **渲染引擎** | three.js 命名 API：Scene / PerspectiveCamera / Mesh / Sphere·Plane·Cylinder·Box·Circle Geometry / MeshStandard Material / Ambient·Directional·Point Light / Renderer.render / OrbitControls；场景对象 = CGA blade，变换 = Motor 共轭；超采样抗锯齿 `renderer(w, h, aa, n)` |
| **复杂建模** | **CSG**（union/difference/intersection 递归真布尔，实体协议 crossings/contains）；**仿射扩展**（scale/mirror 射线逆变换 + Newton 极分解）；**新图元** cone/torus/ellipsoid/cyclide；**网格**（Möller–Trumbore 批量求交 + extrude/loft + OBJ/glTF/GLB 互操作） |
| **MLX GPU** | 每像素向量化解析求交，全分辨率单帧一次 kernel 批量；相机空间 X 右 / Y 下 / Z 前 |

## 快速开始

要求：V 0.5.x、macOS Apple Silicon、`mlx-c`（Homebrew）与 `mlx-v` 仓库
（见 `~/code/mlx-v/README.md`）。`mlx` / `cga` 模块经 V 默认模块路径
`~/.vmodules` 解析，一次性链好符号链接后**无需任何环境变量**：

```bash
# 一次性：把两个模块链进 V 的默认模块路径
ln -s ~/code/mlx-v ~/.vmodules/mlx
ln -s "$(pwd)"     ~/.vmodules/cga

make test     # 跑全部 19 个测试文件（-no-memory-limit，见 Makefile）
make run      # 渲染 smoke 场景 → examples/artifacts/render_smoke.png
make editor   # 启动 CGS 网页编辑器 → http://127.0.0.1:8123
make fmt      # v fmt -w .
```

直接调用 `v` 即可（例如 `v -no-memory-limit test .`）。构建可执行文件
（demo / `render_cgs` / `render_smoke`）请加 `-gc boehm`：V 0.5.2 默认的
`boehm_full_opt` GC 生成的 closure 代码在 macOS 上编译失败，会触发一次
虚假的「C compiler bug report」与回退重编译（产物仍正确，但很吵）。

## CGS 场景语言 (OpenSCAD 风格)

`examples/orbit.cgs`（与上面的演示场景逐位等价，测试有金样断言）：

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
v -gc boehm run examples/render_cgs.v examples/orbit.cgs orbit.png 640 480 2
```

支持变量/表达式/数学函数/for+range/module/if-else/echo，以及 CSG
（union/difference/intersection）；完整语法见 `scene_lang.v`
文件头注释。示例：`examples/grid.cgs`（module + for 的 3×3 球阵）、
`examples/building.cgs`（CSG 开窗建筑）、`examples/mechanical.cgs`
（CSG 钻孔装配）。

### 实时预览编辑器 (V web)

`editor/` 是网页版实时预览编辑器（V `net.http` 服务，替代原 Rust/gpui 版本）：
左侧代码编辑，右侧实时预览（编辑防抖 → `POST /render` → PNG），解析错误返回
HTTP 400 并在界面提示（Result 式解析，不会崩掉服务）。

- **CGS 语法高亮**：手写词法分析器（`editor/highlight.v`：关键字 / 图元 /
  修饰符 / 函数 / 数字与色值 / 注释 / 运算符）。
- **拖拽调参**：`editor/params.v` 把每个数值字面量（如 `sphere.r`、
  `translate.t[1]`）映射为滑块，拖动直写回源文本并实时重渲染。
- 路由：`GET /`（编辑器页面）、`POST /render?w=&h=&aa=`（CGS→PNG）、
  `POST /params`（提取可拖动参数 JSON）、`GET /health`。

```bash
make editor     # 或：v -gc boehm run editor/
# open http://127.0.0.1:8123
```

## 复杂建模能力

面向建筑/机械参数化建模的完整工具链：

**CSG 真布尔** — `difference()` / `intersection()` 递归组合（任意嵌套）：
收集子树全部边界穿越点 → 逐段成员测试（δ 双侧采样）→ 最近实体表面。叶子 =
全部实体图元（sphere/box/cylinder/cone/torus/ellipsoid/extrude/loft/mesh 与
plane 半空间 —— 半空间交 = 剖切视图）：

![CSG 布尔并排](examples/artifacts/demo_csg.png)

**建筑：CSG 开窗 + 纹理** — `examples/building.cgs`：参数化板式办公楼，
砖墙用 `difference()` 开真窗洞，`material(map=...)` 贴砖纹：

![参数化建筑](examples/artifacts/building.png)

**机械：CSG 钻孔装配** — `examples/mechanical.cgs`：法兰螺栓节圆阵列真钻孔 +
中心孔、沉头锥孔、环面垫圈、径向齿阵列：

![机械装配体](examples/artifacts/mechanical.png)

**新图元** — cone（凸体区间裁剪）/ torus（Durand-Kerner 复数迭代解四次方程）/
ellipsoid（= 仿射缩放球）/ **cyclide**（Dupin cyclide，四次曲面，Durand-Kerner
求根）。四者非 CGA blade，经射线逆变换接入；cyclide 模型见 `cyclide.v`。

**仿射扩展** — scale/mirror 经 AffineGeometry 射线逆变换（非 versor 可达；法向
走逆置变换，det<0 镜像自动正确）。上下文为全 4×4 仿射，几何落点 Newton 极分解
为 motor·linear，rotate 与 scale/mirror 任意嵌套顺序均正确。

**位移曲面（基元 + 残差）** — `displaced.v`：一般曲面表示为
`F(x) = d_base(x) − scale·r(uv(x))`，基元取 sphere/plane/无限 cylinder/cyclide，
r 为双线性残差网格（u 周期缠绕）。求交 = 基元解析括段（半径按 max|残差| 膨胀）+
段内 128 步符号扫描 + 8 次二分细化（非自由空间 ray marching）；法向为 F 的中心
差分。`bake_residual` 沿基元节点法向批量投射射线把任意目标几何（含 trimesh）
烘焙成残差网格，闭合「任意曲面 → CGA 基元」的表达回环。示例
`examples/demo_displace.v`（loft 花瓶 → 圆柱，PSNR ~34 dB）。

**网格与互操作** — MeshGeometry（Möller–Trumbore 批量求交，平坦法向，无 BVH）；
`modeling.v` 的 extrude（耳切凹轮廓三角化）与 loft（等点数多截面）；`mesh_io.v`
纯 stdlib OBJ 读写，`mesh_io_gltf.v` glTF/GLB 读写（节点变换/层级/材质色）。

**精度** — 代数核心（multivector/motor/图元）在 V 里是纯 CPU `[32]f64`，比
Python 参考的默认 float32 更精确，远原点 conformal 抵消不再受 float32 精度
限制。渲染内核仍是 float32（MLX/Metal 无 float64，参数进相机空间后
near-origin）。

**运动学** — `examples/demo_kinematics.v`：齿轮副（16:8 齿数比 → 角速度比精确
−1:2）、曲柄滑块、螺旋轨迹 `M(s) = M₀·exp(s·log(M₀⁻¹M₁))` —— 全部 Motor 直写，
无矩阵分解/四元数换算层，直接由 V 合成 `examples/artifacts/kinematics.gif`（不落 PNG）：

![运动学 demo](examples/artifacts/kinematics.gif)

## 场景代码

```v
import cga

mut scene := cga.scene(none)
scene.add_mesh(cga.mesh(cga.plane_geometry([0.0, 1.0, 0.0]!, 0.0),
  cga.standard_material(cga.color_hex(0xB0B0B0), 0.7, 0.0,
  cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.0, 0.0, 0.0]!,
  [0.0, 0.0, 1.0]!, 0.0, none))              // 地面: 对偶平面 blade (y=0)
scene.add_mesh(cga.mesh(cga.sphere_geometry(1.0),
  cga.standard_material(cga.color_hex(0xC0392B), 0.25, 0.25,
  cga.color_hex(0x000000), 1.0, 1.5, 0.0), [0.0, 1.0, 0.0]!,
  [0.0, 0.0, 1.0]!, 0.0, none))              // 球: 对偶球 blade, 半径即尺寸
scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.38, [0.4, 1.0, 0.35]!))
scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.34))

mut camera := cga.perspective_camera(50.0, 4.0 / 3.0, 0.1, 100.0,
  [0.0, 2.4, 6.2]!, [0.0, 0.8, 0.0]!, [0.0, 1.0, 0.0]!)
camera.look_at([0.0, 0.8, 0.0]!, none)

mut renderer := cga.renderer(360, 270, 2, 3)
img := renderer.render(scene, camera)          // (H, W, 4) uint8 RGBA
cga.save_frame_png('out.png', img)
img.free()
```

## 架构

```mermaid
flowchart LR
    subgraph CGA核心
        MV["Multivector [32]f64"] --> PRIM[图元类<br/>Point/Line/Plane/Sphere/Circle/Cylinder]
        MOT[Motor versor] --> TRANS["X_cam = M·X·M̃"]
        PRIM --> TRANS
    end
    subgraph 渲染引擎
        SC[Scene/Mesh/Geometry] --> WMC["每帧: 图元共轭进相机空间 (CPU f64)"]
        WMC --> HIT["mlx-v GPU 批量求交 (N=H·W 向量化 float32)"]
        HIT --> SH[Blinn-Phong 着色]
        SH --> PIX[(RGBA 帧)]
        CAM[PerspectiveCamera/OrbitControls] --> WMC
        LGT[3 种灯光] --> SH
    end
    CGA核心 --> 渲染引擎
```

关键设计：图元级（blade）建模而非三角网格 —— 球/圆柱没有细分数，尺寸全部在
geometry 构造参数里；像素级计算全部在 mlx-v GPU 上批量进行，V 层每帧只循环
图元（~10 个）。代数核心跑在 CPU 的 `[32]f64` 上（比 Python 默认 float32 更准），
`mlx-v` 只用在逐像素渲染内核（`renderer.v` / `geometry_ops.v` / `shading.v`），
V 无标量运算符重载，标量助手 `s_add`/`s_mul`/`s_clip`… 自由函数由 `mlx-v` 提供。

## CGA 建模 vs 传统欧氏建模（渲染视角）

| 维度 | 传统欧氏建模 (three.js/网格) | 本项目 CGA 建模 |
| --- | --- | --- |
| **几何表示** | 三角网格：球/圆柱靠细分数逼近，永远是多边形近似 | 隐式 blade：球/圆柱/平面/圆/盒的解析方程，精确无细分数 |
| **尺寸/精度** | 细分数决定精度与内存；距离近了能看到面片棱角 | 尺寸 = geometry 构造参数（如 `sphere_geometry(1.0)`），任意距离渲染一致 |
| **变换机制** | 4×4 矩阵（平移+旋转分算），连乘浮点误差破坏正交性 | Motor versor 共轭 `X' = M·X·M̃`；任意 motor 乘积仍是 motor，逆 = reverse |
| **相机** | 独立的 view/projection 矩阵 | 相机 pose 也是 Motor，与物体变换同机制 |
| **渲染管线** | 光栅化：顶点着色器投影 → 片段插值 → z-buffer | 光线追踪：每像素对 blade 解析求交，mlx-v GPU 逐像素批量 |
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

```mermaid
flowchart LR
    subgraph CGA核心[代数层: 同一套对象]
        BL[blade 图元<br/>平面/球/圆柱/圆/盒]
        MO[Motor versor<br/>compose/inverse/log/velocity]
    end
    subgraph 渲染层
        RT[mlx-v GPU 光线追踪<br/>合成深度/RGB]
        IR[逆渲染 round-trip<br/>模型→图像验证]
    end
    subgraph 机器人应用
        A[仿真与合成数据]
        B[运动学与轨迹]
        C[几何感知输出]
        D[重建回环验证]
        E[统一坐标变换]
    end
    BL --> A & C
    MO --> B & E
    RT --> A
    IR --> D
```

- **仿真与合成数据**：光线追踪合成深度/RGB，逐像素 GPU 批量，改视角只需重设相机
  Motor —— 适合感知训练数据的域随机化批量生成（带精确深度真值）。
- **运动学与轨迹**：Motor 是 SE(3) 的 versor 表示，已有 `exp`/`log`/
  `velocity_bivector`/`interpolate`；`motor = exp(s·log(M₀·M₁⁻¹))` 生成平滑刚体
  路径，无需矩阵分解。
- **几何感知输出**：图元 blade 本身就是操作对象语义（平面法向 = 抓取姿态 z 轴、
  圆柱轴线+半径 = 夹爪开度）。
- **重建回环验证**：重建的 blade 场景渲染回原视角（`renderer.v`），与原帧深度对比
  发现漂移。
- **统一坐标变换**：相机、机械臂、工件全用同一代数，多坐标系链式变换收敛到一个
  表示。

## 项目布局

```text
cga/                       # 平铺 `module cga`（V 文件全在仓库根目录）
  multivector.v            32 分量多重向量 (gp/ip/op/reverse/dual/meet/norm)，[32]f64
  multivector_tables.v     GP 积表（由 gen/gen_tables.py 生成，与 Python 逐位一致）
  motors.v                 Motor: rotor/translator/exp/log/interpolate/velocity
  primitives.v             图元: point/point_pair/line/plane/sphere/circle/cylinder + 距离
  cyclide.v                Dupin cyclide 模型（球族包络 + versor 反演）
  scene_graph.v            Vec3/Color/Object3D
  scene.v                  Mesh/Scene/PerspectiveCamera/OrbitControls
  geometry.v + geometry_ops.v + geometry_extra.v
                           blade 几何 + 解析求交（cone/torus/ellipsoid/cyclide/trimesh）
  shading.v                材质/灯光 + 批量 Blinn-Phong
  texture.v                PNG 解码 + bilinear map 采样
  renderer.v               mlx-v GPU 批量光线追踪（SSAA/硬阴影/Whitted 折射）
  displaced.v + displaced_kernel.v
                           基元+残差位移曲面（bracket+march 求交 / 残差烘焙）
  csg.v + csg_node.v       递归 CSG 布尔（crossings/contains 实体协议）
  affine.v + affine_geom.v 仿射扩展（scale/mirror 射线逆变换 + Newton 极分解）
  modeling.v               耳切三角化 + extrude + loft
  mesh_io.v                OBJ 读写 + 4x4 矩阵助手
  mesh_io_gltf.v           glTF/GLB 读写（save_glb / load_gltf）
  image_io.v               PNG 读写
  gif.v                    动画 GIF89a 编码（中位切分配色 + LZW，纯 stdlib）
  scene_lang.v             CGS 场景语言（lexer + 单遍 parser/evaluator）
  *_test.v                 17 个根目录测试文件
  editor/                  CGS 网页编辑器（server.v/params.v/highlight.v + web/）
                           + params_test.v / highlight_test.v
  examples/                .cgs 示例 (orbit/grid/building/mechanical) + assets/ 纹理
                           + .v 演示 CLI（见下）
  gen/gen_tables.py        GP 积表生成器（生成 multivector_tables.v）
  examples/artifacts/      demo 输出（README 插图）
  artifacts/tests/         测试金样图（cgs_orbit / cone / cyclide / ...）
```

演示 CLI（`v -gc boehm run examples/<name>.v`）：

- `demo_engine.v` —— 轨道动画 → `orbit.gif`
- `demo_advantage.v` —— 无多边形/无限几何/变换同构三面板 → `advantage_{a,b,c}.png`
- `demo_kinematics.v` —— 齿轮副/曲柄滑块/螺旋插值 → `examples/artifacts/kinematics.gif`
- `demo_csg.v` —— difference/intersection/union 并排 → `demo_csg.png`
- `demo_gltf.v` —— extrude L 形 → 存 `.glb` → 重载 → 渲染 → `demo_gltf.{glb,png}`
- `render_smoke.v` —— smoke 场景 → `render_smoke.png`（即 `make run`）
- `render_cgs.v <file.cgs> [out.png] [w h aa]` —— CGS→PNG CLI
- `demo_displace.v` —— loft 星形花瓶烘焙到圆柱基元 → `displace_{target,displaced,diff}.png`

## 质量

- `make test`（`v -no-memory-limit test .`）：19 个测试文件全过 —— 代数恒等式 /
  图元关联判据 / versor 往返 / exp-log 往返 / 距离公式 / 抗锯齿 / 引擎渲染定量 /
  CSG 布尔 / 仿射 / 新图元 / cyclide / 网格与互操作 / CGS / 位移曲面烘焙 /
  编辑器 params+highlight。
- 测试会把渲染金样图写到 `artifacts/tests/`（cgs_orbit / cone / cyclide /
  ellipsoid / sphere / textured_box / torus / trimesh）。
- `v test .` 零警告、零 notice。

## License

MIT，见 [LICENSE](LICENSE)。
