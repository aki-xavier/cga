<!-- markdownlint-configure-file {"MD013": false} -->
# 模块划分评审（2026-10-09）

状态：**评审完成；问题 1、2 已落地（2026-10-09），仅剩顺手项**。范围：7 个 crate 的依赖图、体量、跨 crate 引用。

## 结论：整体合理，两处结构性问题

依赖 DAG 干净（最硬的证据）：

```text
cga-core（零依赖，2620 行/38 文件，一图元一文件）
  ├─ cga-collision（窄相 probe 只吃 cga-core 的 Geometry+世界矩阵 ✓）
  ├─ cga-mesh（烘焙/质量属性/STL ✓）
cga-fastmetal（355 行单文件，Metal kernel 包装 ✓）
cga-gpu ← core + fastmetal（9845 行，renderer/bvh/certify/csg/shading 清晰）
cga-host ← core + collision + mesh + gpu（JSX/React/运动学/碰撞适配/导出）
cga-examples ← core + gpu + host（终端 CLI，无反向依赖 ✓）
```

无环、无终端反向依赖、窄相与烘焙独立性正确、cga-core 纯代数边界无污染。

## 问题 1（结构性，主要）：场景数据模型住在 cga-gpu

`Scene` / `Object` / `Material` / `Light` / `PerspectiveCamera` / `Color` 定义在
cga-gpu（`scene/`、`scene_graph/`、`shading/`），但它们是**场景表示**不是
Metal 渲染器。实证三处：

- `cga-host` 对 cga-gpu 的引用主体是 `scene::{Object, Scene, PerspectiveCamera}`、
  `shading::{Light, Material}`——要的是模型不是渲染器；
- `cga-host/src/collision.rs` 被迫 `use cga_gpu::scene::Scene`（窄相 crate 本身
  是对的，场景侧适配被 gpu 类型绑架）；
- `docs/cross-platform.md` 的跨平台愿景下，纯 CPU 场景消费者无法在不拖
  mlx-sys 的情况下使用场景模型。

**对策**：抽 `cga-scene` crate（Scene/Object/Material/Light/Camera/Color/
纹理），gpu 与 host 改为依赖它；cga-gpu 内 re-export 保持现有 API 路径不破坏。

## 问题 2（内聚性）：`jsx/mod.rs` 是 God file —— 已解决

原 4124 行非测试代码承担至少 6 类职责：swc 编译、CSS 匹配/级联、元素 walk/
几何构建、SceneSession（帧事务/增量）、拖拽与拾取 API、tag/face/center 查询。
已按流水线切成五层（见行动表），`cga_host::*` 的 pub API 一行未变。

## 轻微项（记录在案）

- `renderer/mod.rs` = 43 行 helper + 1336 行测试集中地，名不副实（测试可挪
  `renderer/tests.rs`）；
- `kinematics.rs` 非测试 1014 行内聚良好，测试/代码 2:1 是文化使然；
- `v3_*` 小助手在 kinematics.rs 与 scene_graph 各一份，host 侧可收敛到
  cga-core 的 `vec3_dot` 等。

## 行动表

| 项 | 动作 | 成本 | 状态 |
| --- | --- | --- | --- |
| 场景模型出 gpu | 抽 `cga-scene` crate | 中 | **已完成（2026-10-09）**：Scene/Object/Object3D/Color/Camera/OrbitControls/Light/Material/CPU 纹理/PNG 编解码全部迁入 `crates/cga-scene`（零 MLX 依赖）；`Texture.pixels` 改 `Vec<f32>`，GPU 采样走 cga-gpu 的 `texture::GpuTexture`（按次转换，缓存另行立项）；`Light::direction_at/far` 改自由函数 `shading::{light_direction_at, light_far}`；cga-gpu 四个模块改路径兼容 shim（旧 `cga_gpu::scene::*` 等路径不变）；cga-host 场景模型引用全面切到 `cga_scene`。344 测试全绿，重构无行为变化（纹理测试曾误改场景参数，已按原版恢复）。 |
| jsx/mod.rs 拆分 | 按职责拆子模块 | 中 | **已完成（2026-10-09）**：按流水线切五层——`compile`（swc）/ `element`（El+prop）/ `css` / `builder`（Builder）/ `session`（boa+Session），测试独立成 `tests.rs`；pub API 不变。唯一实质改动：`Builder` 字段不再被 session 直读，改为 `Builder::start(BuilderStart)` + `build_all() -> BuiltRun`（走树→统计→默认相机→SceneRun 的编排收进 Builder，字段保持私有）。逐行多重集比对确认零语义漂移，344 测试全绿。 |
| renderer/mod.rs 改名 | 测试挪 tests.rs | 小 | 顺手 |
| v3 helper 收敛 | host 改用 cga-core | 小 | 顺手 |
