# cga — 共形几何代数 (Conformal Geometric Algebra) 实验场

5D 共形几何代数核心 + three.js 风格渲染引擎 + MLX/Metal GPU 批量光线追踪。

把欧氏 3D 空间嵌入共形空间 (基 `{e1, e2, e3, e0, e∞}`), 点 / 线 / 面 / 圆 / 球与刚体
运动 (motor) 统一为代数元素: 场景里的每一个对象都是一个 CGA blade, 相机是一次
versor 共轭, 渲染就是对 blade 的 GPU 批量求交。

## 渲染结果

`demo_engine.py` 的轨道动画 (90 帧, ~30ms/帧, 360×270, `aa=2` 超采样抗锯齿):
地面 + 红/蓝球 + 金柱 + 绿盒 + 紫圆盘 + 折射玻璃球 (Fresnel/Beer), 平行光 + 点光 + 正面补光 + 环境光 (硬阴影),
OrbitControls 环绕一周:

![轨道渲染 demo](docs/orbit.gif)

重新生成: `uv run python demo_engine.py 90` (默认输出到 `artifacts/`)。

## 特性

| 层 | 内容 |
| --- | --- |
| **CGA 核心** | 32 分量 multivector; 图元类 Point / PointPair / Line / Plane / Sphere / Circle / Cylinder; Motor versor 变换 (gp/reverse/log/velocity_bivector); exp/log/插值; 直接形式 `op` 与对偶形式 `ip` 两种关联判据; **float32/float64 双精度** (`set_precision`, 见下) |
| **渲染引擎** | three.js 命名 API: Scene / PerspectiveCamera / Mesh / Sphere·Plane·Cylinder·Box·Circle Geometry / MeshBasic·Standard Material / Ambient·Directional·Point Light / Renderer.render / OrbitControls; 场景对象 = CGA blade, 变换 = Motor 共轭 (相机也是 Motor); 超采样抗锯齿 `Renderer(aa=N)` |
| **复杂建模** | **CSG** (difference/intersection 递归真布尔, 实体协议 crossings/contains); **仿射扩展** (scale/mirror/shear 射线逆变换); **新图元** cone/torus/ellipsoid; **网格** (Möller–Trumbore 批量求交 + extrude/loft 构建器 + OBJ/glTF 互操作) |
| **MLX GPU** | 每像素向量化解析求交, 全分辨率单帧一次 kernel 批量; 相机空间 X 右 / Y 下 / Z 前 |

## 快速开始

```bash
uv run python demo_engine.py 90        # 渲染轨道动画 → artifacts/orbit.gif
uv run pytest                          # 测试套件 (tests/: 代数/图元/versor/引擎/逆渲染定量)
```

## CGS 场景语言 (OpenSCAD 风格)

`examples/orbit.cgs` (与上面的 Python 场景逐位等价, tests 有金样断言):

```text
material(color=0xB0B0B0, roughness=0.7) plane(n=[0, 1, 0], d=0);
translate([0, 1, 0])
  material(color=0xC0392B, roughness=0.25, metalness=0.25) sphere(r=1);

directional_light(direction=[0.4, 1.0, 0.35], intensity=0.38);
camera(fov=50, position=[0, 2.4, 6.2], target=[0, 0.8, 0]);
```

修饰符 (`translate`/`rotate`/`material`) 对紧随的语句或 `{}` 块生效,
可嵌套; 图元: sphere/plane/cylinder/box/circle。渲染:

```bash
uv run python -m cga.scene_lang examples/orbit.cgs orbit.png 640 480 2
```

支持变量/表达式/数学函数/for+range/module/if-else/echo;
完整语法见 `cga/scene_lang/__init__.py` 文档头 (v3: scale/mirror/
difference/intersection/cone/torus/ellipsoid/extrude/loft/mesh/precision)。
阵列示例: `examples/grid.cgs` (module + for 的 3×3 球阵);
领域示例: `examples/building.cgs` (CSG 开窗建筑) /
`examples/mechanical.cgs` (CSG 钻孔装配)。

### 实时预览编辑器 (Rust/GPUI)

`editor/` 是 OpenSCAD 风格的桌面编辑器 (gpui-component): 左侧代码编辑,
右侧实时预览 (编辑防抖 300ms → 常驻渲染服务 → PNG), 解析错误上状态栏。

- **多文件标签页**: 顶部标签栏, 启动自动打开 `examples/*.cgs`, 支持
  新建 / 打开 (系统对话框, 多选) / 保存 (未命名走另存为) / 关闭切换。
- **CGS 语法高亮**: 手写词法分析器 (关键字 / 图元 / 修饰符 / 函数 /
  数字与色值 / 注释 / 运算符), One-Dark 配色 + 大括号折叠。
- **拖拽调参**: 右侧参数面板把每个数值字面量 (如 `sphere.r`、
  `translate.t[1]`、`for.i.start`) 映射为滑块, 拖动直写回源文本并
  实时重渲染; 色值不纳入拖拽。

```bash
cd editor && cargo run    # 首次构建较久; 渲染服务自动拉起
```

架构: 编辑器 (Rust) ↔ HTTP ↔ `cga.scene_lang.render_server` (Python/MLX
常驻进程, 避免冷启动)。需要 Xcode 的 metal 工具链:
`DEVELOPER_DIR=/Applications/Xcode.app/Contents/Developer cargo run`。

## 复杂建模能力 (v3)

面向建筑/机械参数化建模的完整工具链:

**CSG 真布尔** — `difference()` / `intersection()` 递归组合 (任意嵌套):
收集子树全部边界穿越点 → 逐段成员测试 (δ 双侧采样) → 最近实体表面。
叶子 = 全部实体图元 (sphere/box/cylinder/cone/torus/ellipsoid/extrude/
loft/mesh 与 plane 半空间 —— 半空间交 = 剖切视图)。机械装配体的
钻孔/沉头孔/键槽由此解锁:

![CSG 球减三向方孔](docs/csg.png)

**建筑: CSG 开窗 + 纹理** — `examples/building.cgs`: 参数化板式办公楼
(层数/开间/进深全参数), 砖墙用 `difference()` 开真窗洞,
`material(map=...)` 贴砖纹 (box 立方体投影 UV):

![参数化建筑](docs/building.png)

**机械: CSG 钻孔装配** — `examples/mechanical.cgs`: 法兰螺栓节圆阵列
真钻孔 + 中心孔、沉头锥孔、环面垫圈、花纹板贴图、径向齿阵列
(rotate+translate 的 Motor 复合):

![机械装配体](docs/mechanical.png)

**新图元** — cone (凸体区间裁剪) / torus (Durand-Kerner 复数迭代解射线
四次方程) / ellipsoid (= 仿射缩放球, 零新求交数学)。三者非 CGA blade,
经射线逆变换接入 (如实标注)。

**仿射扩展** — `Object3D.linear` (3x3): scale/mirror/shear 经
AffineGeometry 射线逆变换 (非 versor 可达; 法向走逆置变换, det<0
镜像自动正确)。CGS 里 `scale(s)` / `mirror(axis=...)` 修饰符,
上下文为全 4x4 仿射, 几何落点 Newton 极分解为 motor·linear,
rotate 与 scale/mirror 任意嵌套顺序均正确。

**网格与互操作** — MeshGeometry (Möller–Trumbore 分块批量求交, 平坦
法向, 无 BVH 如实标注); `cga.modeling` 的 extrude (耳切凹轮廓三角化)
与 loft (等点数多截面); `cga.mesh_io` 纯 stdlib OBJ 与 glTF/GLB 读写
(节点变换/层级/材质色)。

**精度** — `set_precision("float64")` (或 CGS `precision("float64");`):
代数核心 (multivector/motor/图元) 切到 CPU float64 —— 远原点 conformal
抵消从 ~1e-4 压到 ~1e-13 相对量级, 场景坐标可远超 ±20 (实测 x=±400
渲染与原点渲染逐像素差 ≤1/255); 渲染内核仍 float32 (参数进相机空间后
near-origin)。Metal 无 float64: Multivector/Motor/图元的全部方法经
`wrap_cpu_f64` 守卫自动切 CPU stream, 调用方透明。

**运动学** — `demo_kinematics.py` (`.venv/bin/python demo_kinematics.py`):
齿轮副 (16:8 齿数比 → 角速度比精确 −1:2)、曲柄滑块 (连杆 pose 每帧由
两端点轴角转子解算)、螺旋轨迹 `M(s) = M₀·exp(s·log(M₀⁻¹M₁))` ——
全部 Motor 直写, 无矩阵分解/四元数换算层:

![运动学 demo](docs/kinematics.gif)

## 场景代码

```python
from cga.engine import *

scene = Scene()
scene.add(
    Mesh(PlaneGeometry((0, 1, 0), 0.0),             # 地面: 对偶平面 blade (y=0)
         MeshStandardMaterial(Color(0xB0B0B0), roughness=0.7)),
    Mesh(SphereGeometry(1.0),                        # 球: 对偶球 blade, 半径即尺寸
         MeshStandardMaterial(Color(0xC0392B), roughness=0.25, metalness=0.25),
         position=(0, 1, 0)),
    Mesh(BoxGeometry(0.9, 0.9, 0.9),                 # 盒: 3 对平面 slab
         MeshStandardMaterial(Color(0x27AE60), roughness=0.6),
         position=(0.8, 0.45, 1.8)),
    DirectionalLight(intensity=0.38, direction=(0.4, 1.0, 0.35)),  # 主光
    PointLight(intensity=0.7, position=(0, 4, 3.5)),
    AmbientLight(intensity=0.34),
)

camera = PerspectiveCamera(fov=50, aspect=4 / 3, position=(0, 2.4, 6.2), target=(0, 0.8, 0))
camera.look_at((0, 0.8, 0))
controls = OrbitControls(camera, target=(0, 0.8, 0), radius=6.6, elevation=0.42)
controls.azimuth = 2 * 3.14159 * 0.5   # 半圈
controls.update()

renderer = Renderer(360, 270)
img = renderer.render(scene, camera)    # (H, W, 4) uint8 RGBA
```

## 架构

```mermaid
flowchart LR
    subgraph CGA核心
        MV[Multivector 32分量] --> PRIM[图元类<br/>Point/Line/Plane/Sphere/Circle/Cylinder]
        MOT[Motor versor] --> TRANS["X_cam = M·X·M̃"]
        PRIM --> TRANS
    end
    subgraph 渲染引擎
        SC[Scene/Mesh/Geometry] --> WMC["每帧: 图元共轭进相机空间 (CPU)"]
        WMC --> HIT["MLX GPU 批量求交 (N=H·W 向量化)"]
        HIT --> SH[Blinn-Phong 着色]
        SH --> PIX[(RGBA 帧)]
        CAM[PerspectiveCamera/OrbitControls] --> WMC
        LGT[3 种灯光] --> SH
    end
    CGA核心 --> 渲染引擎
```

关键设计: 图元级 (blade) 建模而非三角网格 —— 球/圆柱没有细分数, 尺寸全部在
geometry 构造参数里; 像素级计算全部在 MLX GPU 上批量进行, Python 层每帧只循环
图元 (~10 个)。

## CGA 建模 vs 传统欧氏建模（渲染视角）

本项目的建模方式与传统欧氏引擎 (three.js + 三角网格 + 矩阵变换) 有根本区别:

| 维度 | 传统欧氏建模 (three.js/网格) | 本项目 CGA 建模 |
| --- | --- | --- |
| **几何表示** | 三角网格: 球/圆柱靠细分数逼近, 永远是多边形近似 | 隐式 blade: 球/圆柱/平面/圆/盒的解析方程, 精确无细分数 |
| **尺寸/精度** | 细分数决定精度与内存; 距离近了能看到面片棱角 | 尺寸 = geometry 构造参数 (如 `SphereGeometry(1.0)`), 任意距离渲染一致 |
| **变换机制** | 4×4 矩阵 (平移+旋转分算), 连乘浮点误差破坏正交性, 需重新正交化 | Motor versor 共轭 `X' = M·X·M̃`; 任意 motor 乘积仍是 motor (刚体保真), 逆 = reverse |
| **相机** | 独立的 view/projection 矩阵, 与物体变换机制分离 | 相机 pose 也是 Motor, 与物体变换同机制 (`X_cam = M_cam·X·M̃_cam`) |
| **渲染管线** | 光栅化: 顶点着色器投影 → 片段插值 → z-buffer | 光线追踪: 每像素对 blade 解析求交 (ray-sphere 二次 / ray-plane 一次 / ray-cylinder 二次 / 盒 slab 法), MLX GPU 逐像素批量 |
| **统一性** | 几何、变换、渲染是三套独立机制 | 点/线/面/圆/球与刚体运动同属一个 5D 代数 (都是 multivector), 关联判据 op/ip、求交 meet 统一 |
| **动画/插值** | 矩阵无直接插值语义, 需分解位置+四元数 | `Motor.exp/log`: 可直接插值 motor、提取速度二重向量 (运动学闭环) |

这带来几个实际后果:

- **球/圆柱无多边形** —— 渲染质量不随相机距离恶化, 近看不会暴露三角面片; 代价是隐式几何没有顶点/拓扑, 网格编辑类建模工具 (细分、挤出) 用不上。
- **无限几何天然成立** —— 无限平面 (地平线无限延伸)、无限圆柱是代数对象本身的属性, 无需裁剪; 想要有限片状面需自行按区域掩码裁剪 (见 `cga/render.py`)。
- **变换与几何同构** —— motor 与 blade 是同一类对象, `Motor.compose` / `inverse` / `log`
  直接作用于任何图元, 没有矩阵-四元数-轴角之间的换算层。
- **代价在别处** —— 解析求交的 float32 blade 共轭限制场景尺度 (坐标宜 ±20 内);
  无纹理坐标概念 (v1 无纹理); 有限圆柱经 `CylinderGeometry(length=...)`
  带端盖, 无限圆柱/平面在相机位于退化位形时需内核特殊处理 (见下节)。

### 渲染示例: 三个优势的可视化

`demo_advantage.py` 把上面三个实际后果各渲染成一张独立图像
(`uv run python demo_advantage.py` → `artifacts/advantage_{a,b,c}.png`, 均以
`aa=2` 超采样抗锯齿渲染, 轮廓无锯齿):

**无多边形** — 相机贴脸 (≈2.7× 半径) 的大球 + 无限圆柱: 轮廓是完美圆弧、
高光无棱角; 同等距离的网格球/圆柱已能看到三角面片棱角。

![无多边形](docs/advantage_a.png)

**无限几何** — 地面是无限平面 (直达地平线不裁剪), 金柱是无限圆柱
(无顶/底盖), 红球作尺度参照 (底切正好在地面)。

![无限几何](docs/advantage_b.png)

**变换与几何同构** — 同一个 Motor 沿 `exp(s·log(M0⁻¹M1))` 插值
(SE(3) 螺旋), 一路驱动 6 个小球轨迹图元; 终点绿盒的旋转也由同一
motor 给出 —— 无矩阵分解、无位置/四元数换算层。

![变换与几何同构](docs/advantage_c.png)

## 范围声明 (与 three.js 的差距)

如实标注, 见 `cga/engine/__init__.py` 顶部注释:

- 阴影: 每光源一条遮挡射线 (硬阴影); 无软阴影。无后处理 / tonemap;
  >1 高光硬截断。
- 纹理: `material(map=...)` 解析 UV (v3 起); 无 mipmap/过滤控制。
- scale/mirror: v3 起经 AffineGeometry 射线逆变换 (非 versor 可达的
  仿射扩展); blade 语义 (meet/关联判据) 不适用于仿射形变后的图元。
- CSG: 相切/共面退化配置依赖 δ=1e-4 双侧采样 (间距 < 2δ 可能漏翻转);
  CSG 节点单材质; circle 非实体不能作叶子。
- cone/torus/ellipsoid/网格非 CGA blade, 经射线逆变换接入。
- 网格: 暴力 O(N·F) 无 BVH (定位中小网格); 平坦法向; 无纹理坐标;
  glTF 导入暂限单 primitive; 无 IFC/STEP (B-rep 内核超出范围, 交换
  格式走网格)。
- 无限平面 / 圆柱无面片裁剪; 相机在柱内等退化情形按内核处理。
- 精度: 默认 float32 (blade 共轭场景坐标宜 ±20); `set_precision`
  ("float64") 后代数核心走 CPU float64, 坐标可远超 ±20 (渲染内核仍
  float32)。
- 无 envMap/IBL: 高 metalness 材质会显黑 (three.js 同样问题), demo 因此压低金属度。

## 机器人领域潜在应用

CGA 建模 + Motor + GPU 光线追踪 + 逆渲染回环, 四条能力线分别对机器人
的仿真、运动学、感知与验证需求:

```mermaid
flowchart LR
    subgraph CGA核心[代数层: 同一套对象]
        BL[blade 图元<br/>平面/球/圆柱/圆/盒]
        MO[Motor versor<br/>compose/inverse/log/velocity]
    end
    subgraph 渲染层
        RT[MLX GPU 光线追踪<br/>合成深度/RGB]
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

### 仿真与合成数据

光线追踪合成深度图/RGB, **逐像素 GPU 批量** (H×W 一次 kernel), 改视角只需
重设相机 Motor (OrbitControls/相机共轭) —— 适合感知训练数据的域随机化批量
生成 (带精确深度真值), 以及解析求交的仿真传感器 (比光栅化深度更准, 无面片
误差)。

> 诚实标注: v1 无阴影/纹理/物理光照, 做视觉训练数据可用, 照片级仿真需扩展。

### 运动学与轨迹

Motor 是 SE(3) 的 versor 表示, 本包已有 `exp` / `log` / `velocity_bivector` /
`compose` / `inverse`:

- **Cartesian 轨迹插值**: `motor = exp(s·log(M₀·M₁⁻¹))` 生成平滑刚体路径
  (机械臂末端/移动底座), 无需矩阵分解成平移+四元数。
- **Twist 提取**: `velocity_bivector` 直接给速度二重向量 (6-DOF 旋量),
  控制律 (导纳/阻抗) 的关节空间映射上游。
- **手眼标定**: AX=XB 的 A/X/B 全是 motor, 同一代数免来回转换。
- **刚体保真**: motor 连乘不破坏正交性 (vs 4×4 矩阵浮点漂移),
  长链变换 (base→tool→camera) 不累积退化。

### 几何感知输出

图元 blade 本身就是机器人的操作对象语义 (感知前端的平面/圆柱拟合与深度
估计在配套仓库, 导出为米制 SceneModel 衔接):

- **平面** (桌面/墙/货架面/传送带): 法向即抓取/放置姿态 z 轴, 米制 blade + 协方差直接进操作规划。
- **圆柱** (管件/瓶罐/轴): 轴线 + 半径 = 抓取中心与夹爪开度参数。
- **球** (球体工件) / **圆** (开口、法兰、孔): 近圆目标识别与对准。
- 逆渲染 (`cga/render.py`) 把模型渲染回 2D 核对重建质量 —— 感知闭环可信度机制。

### 重建回环验证

模型→图像 round-trip 是重建系统少见的闭环检查: 重建的 blade 场景渲染回原
视角, 与原帧深度对比可发现漂移; novel-view 预览给操作规划做预期视图。

### 统一坐标变换

相机、机械臂、工件全用同一代数: `X_cam = M_cam·X·M̃_cam` 与
`X_base = M_tool·X` 无机制差异 —— 多坐标系链式变换、手眼标定、双相机几何
收敛到一个表示, 减少跨库转换 bug。

## 项目布局

```text
cga/
  multivector.py   32 分量多重向量, 代数运算 (gp/ip/op/dual/meet/norm/...);
                   积表/掩码/基向量全是类属性; DTYPE 精度开关 + wrap_cpu_f64
  algebra/         图元类与距离 (point/line/plane/sphere/cylinder/circle, 每类一文件)
  motors.py        Motor: 刚体变换 versor, exp/log/插值/速度提取
  engine/          three.js 风格渲染引擎 (MLX 批量光线追踪, 每类一文件):
                   scene/mesh/camera/renderer + 5 种 blade 几何 + 材质 + 灯光
                   + affine_geometry (scale/mirror 射线逆变换, 极分解)
                   + csg (递归布尔) + cone/torus/ellipsoid/trimesh 图元
  modeling/        网格构建器: 耳切三角化 + extrude + loft (纯数据变换)
  mesh_io/         OBJ / glTF/GLB 读写 (纯 stdlib, 无第三方依赖)
  render/          逆渲染: 图元场景 → 2D 深度/颜色 (PrimitiveRenderer)
  scene_lang/      CGS 场景语言 (OpenSCAD 风格, Lexer + SceneLoader + 渲染服务)
tests/           pytest 测试套件 (checks.py 共享断言 + test_* 每类一文件)
editor/            cgs-editor (Rust/GPUI): .cgs 实时预览编辑器 (左代码右渲染)
examples/          .cgs 示例 (orbit/grid/building/mechanical) + assets/ 纹理
demo_engine.py     轨道动画 demo (OrbitDemo) → PNG 帧 + GIF
demo_advantage.py  优势渲染 demo (AdvantageDemo) → 三张独立图
demo_kinematics.py 运动学 demo (齿轮副/曲柄滑块/螺旋插值) → GIF
```

## 质量

- `uv run pytest`: 106 项测试全过 (代数恒等式 / 图元关联判据 / versor 往返 /
  exp-log 往返 / 距离公式 / 抗锯齿 / 引擎渲染与 Whitted 管线定量 / CSG 区间
  布尔 / 仿射 / 新图元 / 网格与互操作 / CGS v3, 见 `tests/`)。
- ruff (E/F/I/UP) 与 pyright (strict) 零告警。

## License

MIT, 见 [LICENSE](LICENSE)。
