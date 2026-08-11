# cga — 共形几何代数 (Conformal Geometric Algebra) 实验场

5D 共形几何代数核心 + three.js 风格渲染引擎 + MLX/Metal GPU 批量光线追踪。

把欧氏 3D 空间嵌入共形空间 (基 `{e1, e2, e3, e0, e∞}`), 点 / 线 / 面 / 圆 / 球与刚体
运动 (motor) 统一为代数元素: 场景里的每一个对象都是一个 CGA blade, 相机是一次
versor 共轭, 渲染就是对 blade 的 GPU 批量求交。

## 渲染结果

`demo_engine.py` 的轨道动画 (90 帧, ~30ms/帧, 360×270, `aa=2` 超采样抗锯齿):
地面 + 红/蓝球 + 金柱 + 绿盒 + 紫圆盘, 平行光 + 点光 + 正面补光 + 环境光,
OrbitControls 环绕一周:

![轨道渲染 demo](docs/orbit.gif)

重新生成: `uv run python demo_engine.py 90` (默认输出到 `artifacts/`)。

## 特性

| 层 | 内容 |
| --- | --- |
| **CGA 核心** | 32 分量 multivector; 图元类 Point / PointPair / Line / Plane / Sphere / Circle / Cylinder; Motor versor 变换 (gp/reverse/log/velocity_bivector); exp/log/插值; 直接形式 `op` 与对偶形式 `ip` 两种关联判据 |
| **渲染引擎** | three.js 命名 API: Scene / PerspectiveCamera / Mesh / Sphere·Plane·Cylinder·Box·Circle Geometry / MeshBasic·Standard Material / Ambient·Directional·Point Light / Renderer.render / OrbitControls; 场景对象 = CGA blade, 变换 = Motor 共轭 (相机也是 Motor); 超采样抗锯齿 `Renderer(aa=N)` (每像素 N×N 条亚像素射线平均) |
| **MLX GPU** | 每像素向量化解析求交 (5 种隐式几何), 全分辨率单帧一次 kernel 批量; 相机空间 X 右 / Y 下 / Z 前 |

## 快速开始

```bash
uv run python demo_engine.py 90        # 渲染轨道动画 → artifacts/orbit.gif
uv run python -m cga                   # 包自检: 103 项断言 (代数/图元/versor/距离/动力学/接触/浮动基座/隐式接触/积分器/prismatic/传感器/systems/球体回弹)
```

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

## 范围声明 (v1 与 three.js 的差距)

如实标注, 见 `cga/engine.py` 顶部注释:

- 无阴影 / 纹理 / 后处理 / tonemap; 颜色线性 I/O, 不做 sRGB 编码。
- Object3D 无 scale: motor 是刚体变换, 尺寸全走 geometry 参数。
- 无限平面 / 圆柱无面片裁剪; 相机在柱内等退化情形按内核处理。
- float32: blade 共轭在 float32 下进行, 场景坐标宜控制在 ±20 内 (远原点
  conformal 抵消是本包已知限制)。
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

### CRDF — CGA 机器人描述格式

URDF 的 YAML 版: 链接树 + 关节 + 几何 (blade) + 惯量。帧语义与 URDF
完全一致 (joint.origin = 父 frame → joint frame, axis 在 joint frame,
几何/质心 origin 在 link frame, 单位 SI), 但可读性、去重和代数表达
更好:

| 维度 | URDF (XML) | CRDF (YAML) |
| --- | --- | --- |
| **语法** | XML 标签嵌套, 一个链接几十行 | YAML 映射, 一个链接几行; 注释随便写 |
| **origin** | `xyz + rpy` 字符串 | `xyz + rpy` (URDF 兼容) 或 `motor: {axis, angle, t}` (CGA 签名原样) |
| **几何** | mesh 文件引用或基本体, visual/collision 分写两遍 | blade 图元 (cylinder/box/sphere/plane/circle), `role: [visual, collision]` 一份复用 |
| **变换** | 4×4 矩阵 | Motor versor; FK = Motor 链, 长链不积累正交性漂移 |
| **运动学** | 无内置 | `fk(q)`: revolute = `M_origin·Rot(axis,q)`, prismatic = `·Trans(axis·q)` |

示例 (`models/z1_arm.crdf.yaml` 由宇树官方 URDF 导入; 视觉 mesh 跳过,
7 条碰撞圆柱提升为双角色 blade):

```yaml
robot:
  name: z1_description
  base: world
  links:
    - name: link00
      geometry:
        - blade: cylinder
          radius: 0.0325
          length: 0.051
          origin: {xyz: [0, 0, 0.0255]}
          role: [visual, collision]
      inertial: {mass: 0.472, com: [-0.0033, -0.0001, 0.025], inertia: {ixx: 0.0004, iyy: 0.0004, izz: 0.0005}}
  joints:
    - name: joint1
      type: revolute
      parent: link00
      child: link01
      origin: {xyz: [0, 0, 0.0585]}
      axis: [0, 0, 1]
      limit: {lower: -2.618, upper: 2.618, effort: 30, velocity: 3.1415}
```

用法 (加载 → FK → 直接进 CGA 引擎渲染, 无网格无中间表示):

```python
from cga.robot import load_robot
from cga.urdf_io import crdf_to_urdf, urdf_to_crdf  # 双向转换 (pydrake/UrdfScene 互操作)

robot = load_robot("models/z1_arm.crdf.yaml")
world = robot.fk_list([0.0, 1.1, -1.2, 0.8, 0.4, 0.3])  # link → Motor
```

`demo_robot.py` 渲染该描述 (臂底恰落地; 仅根级 Z-up(URDF)→Y-up(引擎)
转换, 数据文件保持 URDF 语义):

![CRDF 机器人渲染](docs/robot_z1.png)

范围声明 (v1): mesh 不建模 —— 导入时**忽略** (`mesh_policy='skip'`, 默认;
某 link 的 visual 全被跳过时, 其 collision 基本体提升为 `[visual, collision]`
保证渲染不空)。可选 `keep` (保留 `blade: mesh, file, scale` 文件引用,
opaque 字符串可含 `package://` URI, interop round-trip 无损, 引擎不渲染)
或 `error` (严格模式, 遇 mesh 报错)。

惯量全 6 分量张量; 无 SRDF/transmission/gazebo 语义; URDF 的
floating/planar 关节不支持。

## 项目布局

```text
cga/
  multivector.py   32 分量多重向量, 代数运算 (gp/ip/op/dual/meet/norm/...)
  algebra.py       图元类与距离 (直接/对偶两种形式)
  motors.py        Motor: 刚体变换 versor, exp/log/插值/速度提取
  engine.py        three.js 风格渲染引擎 (MLX 批量光线追踪)
  render.py        逆渲染: SceneModel → 2D 深度/颜色 (反投影工具)
  compare_clifford.py  已删除 (2026-08): clifford/numpy 依赖移除, 数值验证见 git 历史
  robot.py          CRDF 机器人描述: YAML 解析 + 校验 + Motor FK
  urdf_io.py        URDF ⇄ CRDF 双向转换 (pydrake/UrdfScene 互操作)
  drake.py         Drake 物理后端 (直接接入 pydrake MultibodyPlant: 离散推进+接触求解器, 动力学查询透传)
  __main__.py      包自检: python -m cga (79 项断言)
demo_engine.py     轨道动画 demo → PNG 帧 + GIF
demo_advantage.py   优势渲染 demo → 无多边形/无限几何/motor 插值三张独立图
demo_robot.py      CRDF 渲染 demo: YAML 机器人描述 → FK → 引擎渲染
demo_dynamics.py   动力学 demo (Drake): Z1 计算力矩 PD 姿态跟踪 → 帧/GIF
demo_bounce.py      物理 demo (Drake): 球 1m 坠落触地静止 → 帧/GIF
models/            CRDF 机器人描述文件 (z1_arm / bounce_ball)
```

## WebGL 渲染器 (浏览器端 CGA)

`webgl/` 把同一个 CGA 代数跑进浏览器 —— 浏览器**直接解析 CRDF 的
`.yaml` 文件**, 无需任何转换/导出步骤:

- **原生 CGA** (`cga.js`): 基 blade = bitmask, 几何积用递归恒等式
  (向量·blade = 左收缩 + 楔积; (a∧W)·B = a·(W·B) − (a⌋W)·B), 度量
  e1-3²=1、e0²=e∞²=0、m(e0,e∞)=−1 —— 已对照 Python 端 32×32 基乘积表
  逐项验证 (0/1024 不匹配), 无需导出乘法表。
- **YAML 解析** (`yaml.js`): 覆盖 CRDF 子集 (块映射/同缩进序列/流式
  集合/注释); `model.js` 镜像 `cga/robot.py` 载入语义 (origin 的
  xyz+rpy / motor 两种写法 → Motor versor)。
- **渲染**: WebGL2 片段着色器逐像素解析求交 (球/平面/有限圆柱/盒/圆盘,
  与 `cga.engine` 同族数学) + Blinn-Phong; 每帧 FK (versor 乘积) →
  link motor · 几何 origin → (R,t) → 世界→相机刚体变换 → uniform 数组。
- **交互**: 关节滑块 (FK 实时), 轨道相机 (拖拽旋转/滚轮缩放, 根级
  Z-up→Y-up 与 demo_robot 一致)。
- **验证**: `webgl/verify.js` 用 headless Chromium 检查 console 错误 +
  像素读回 + 滑块联动 (需本机有 playwright/chromium)。

```bash
cd ~/code/cga && python3 -m http.server 8000
# 打开 http://localhost:8000/webgl/  (页面直接 fetch ../models/z1_arm.crdf.yaml)
```

![WebGL 渲染](docs/webgl_z1.png)

## 物理仿真 (直接接入 Drake)

`cga/drake.py` 直接接入 **pydrake MultibodyPlant** (生产级物理引擎),
替代此前自研的动力学移植 (已删除: 自研接触/浮动基座/积分器对复杂场景
脆弱, 且维护成本高):

```python
from cga.drake import DrakePlant

plant = DrakePlant("models/z1_arm.crdf.yaml")          # CRDF → URDF → Drake
plant.set_joint_state(q, qd)                            # 关节状态
plant.step(tau)                                         # 离散推进 (内置接触求解器)
poses = plant.poses()                                   # 渲染用各 link 位姿
M = plant.mass_matrix(q)                                # 动力学查询 (透传 Drake)
```

- **物理全用 Drake**: 离散 plant (time_step>0) 内置接触求解器 (TAMSI/
  约束), 浮动基座 (原生四元数关节), 关节驱动 (JointActuator + 驱动端口)。
- **渲染仍走 cga engine** (CGA 光线追踪): `poses()` 取位姿 → WORLD_UP
  变换 → 引擎渲染; CRDF/URDF 转换复用 (`crdf_to_urdf`)。
- **动力查询透传**: mass_matrix / gravity_forces / coriolis_forces /
  inverse_dynamics / forward_dynamics (EOM: M·q̈ + C·v − Q = τ,
  Q = CalcGravityGeneralizedForces)。
- 世界自带固定地面 (10×10×0.1 盒, 顶面 z=0); 默认点接触 (耗散性,
  真实物理 —— 小球落地静止, 无自定义回弹)。
- 依赖: `drake>=1.55` (重依赖 ~1GB, 物理互操作是 sanctioned 例外,
  API 强制 numpy)。

`demo_bounce.py`: 球从 1m 坠落, Drake 接触求解器处理触地, 静止在球心
z=0.06:

![回弹仿真](docs/bounce_ball.gif)

`demo_dynamics.py`: Z1 计算力矩 PD (τ = M·(Kp·e − Kd·q̇) − Q) 折叠 →
舒展:

![动力学仿真](docs/dynamics_z1.gif)

## 质量

- `python -m cga`: 103 项自检全过 (代数恒等式 / 图元关联判据 / versor 往返 /
  exp-log 往返 / 距离公式 / 抗锯齿 / CRDF FK·校验·round-trip·mesh,
  见 `cga/__main__.py`)。
- ruff (E/F/I/UP) 与 pyright (strict) 零告警。

## License

MIT, 见 [LICENSE](LICENSE)。
