<!-- markdownlint-configure-file {"MD013": false} -->
# 模块划分评审（2026-10-09）

状态：**评审全部落地（2026-10-09）**：问题 1、2 + 两个顺手项均已完成，行动表无未完成项。范围：7 个 crate 的依赖图、体量、跨 crate 引用。

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

## 轻微项（已处理）

- `renderer/mod.rs` 曾是 43 行 helper + 1336 行测试集中地，名不副实 → 测试已挪
  `renderer/tests.rs`；
- `kinematics.rs` 非测试 1014 行内聚良好，测试/代码 2:1 是文化使然（保留）；
- `v3_*` 小助手重复 → `v3_cross` 已收敛到 `cga_core::vec3_cross`；`v3_unit`
  保留本地（退化语义不同，见行动表）。

## 行动表

| 项 | 动作 | 成本 | 状态 |
| --- | --- | --- | --- |
| 场景模型出 gpu | 抽 `cga-scene` crate | 中 | **已完成（2026-10-09）**：Scene/Object/Object3D/Color/Camera/OrbitControls/Light/Material/CPU 纹理/PNG 编解码全部迁入 `crates/cga-scene`（零 MLX 依赖）；`Texture.pixels` 改 `Vec<f32>`，GPU 采样走 cga-gpu 的 `texture::GpuTexture`（按次转换，缓存另行立项）；`Light::direction_at/far` 改自由函数 `shading::{light_direction_at, light_far}`；cga-gpu 四个模块改路径兼容 shim（旧 `cga_gpu::scene::*` 等路径不变）；cga-host 场景模型引用全面切到 `cga_scene`。344 测试全绿，重构无行为变化（纹理测试曾误改场景参数，已按原版恢复）。**后续（2026-10-12 精简遍）：这些路径兼容 shim 已整体删除**（见文末）。 |
| jsx/mod.rs 拆分 | 按职责拆子模块 | 中 | **已完成（2026-10-09）**：按流水线切五层——`compile`（swc）/ `element`（El+prop）/ `css` / `builder`（Builder）/ `session`（boa+Session），测试独立成 `tests.rs`；pub API 不变。唯一实质改动：`Builder` 字段不再被 session 直读，改为 `Builder::start(BuilderStart)` + `build_all() -> BuiltRun`（走树→统计→默认相机→SceneRun 的编排收进 Builder，字段保持私有）。逐行多重集比对确认零语义漂移，344 测试全绿。 |
| renderer/mod.rs 改名 | 测试挪 tests.rs | 小 | **已完成（2026-10-09）**：31 个测试挪到 `renderer/tests.rs`，mod.rs 回到 45 行纯 helper |
| v3 helper 收敛 | host 改用 cga-core | 小 | **部分完成（2026-10-09）**：`v3_cross` 收敛到 `cga_core::vec3_cross`（公式逐字相同，15 处调用点）；`v3_unit` **故意不合并**——近零退化语义是 `[0,0,0]`，与 `cga_scene::vec3_unit` 的 `[0,0,1]` 不同，合并会改相机/法线兜底方向。已在代码注释写明。 |

## 精简遍（2026-10-12）

目标：把整个工作区压到「不能再精简」——重复**定义**、死代码、无意义间接层、机械冗余清零；行为零变化。
验收（全绿）：**348 测试通过**（cga-core 29 + cga-collision 29 + cga-mesh 17 + cga-gpu 102 + cga-host 159 + cga-scene 1 + cga-fastmetal 7 + cga-examples 4）、**clippy 全工作区 0 警告**（原 34 + `--fix` 清掉的 ~150）、**7 张 gallery PNG 逐字节一致**（320×240 aa=2，基线/终态 sha256 全同）。Rust 净删 459 行（35559 → 35100）。

### 1. vec/mat 数学收敛到 cga-core（唯一定义处）
新增 `crates/cga-core/src/vec3.rs`：`vec3_dot / vec3_scale / vec3_cross / vec3_add / vec3_sub / vec3_len`（`vec3_cross` 自 `geometry/mod.rs` 迁入）；`mat4.rs` 新增 `transform_dir`。此前同一公式散在 5 个 crate、14 份定义（`dot`/`v3_dot`/`vdot`/`scale`/`v3_scale`/`vscale`/`v3scale`/`cross`/`v3cross`/`xform_point`/`xform_dir`/`vnorm`/`v3_norm`/`v3_len`）。19 处内联长度公式 `(v[0]*v[0] + …).sqrt()` 统一为 `vec3_len`。
保留的差异：`cga_scene::vec3_unit` 与各处本地 `*_unit` 近零退化兜底方向不同（`[0,0,1]` vs `[0,0,0]`，已注释）；`cga-collision` 用 `use cga_core::{vec3_add as add, …}` 短别名（该 crate 的 `sub/add/norm` 有 130+ 处调用，逐处改长只会更难读）。

### 2. 删掉路径兼容 shim（承接 10-09 的"抽 cga-scene"）
`crates/cga-gpu/src/scene/`、`scene_graph/` 两个纯重导出模块整体删除；`shading/`、`texture/` 里的重导出行删除。引用方全部改到 `cga_scene::`：cga-gpu 内部（renderer/geom_kernels/texture/bvh_trace）、cga-host、cga-examples 的 6 个 `use cga_gpu::*` bin（Cargo.toml 补 `cga-scene` 依赖）。

### 3. cga-gpu 三套平行射线内核去重
`geometry_ops.rs` / `geometry_extra.rs` / `csg.rs` 里同一图元的「射线—图元前导 + 根求解」原本各有 2~3 份逐字拷贝（全仓最大的重复块）。提取 12 个 `#[inline]` helper、30 个调用点复用（`sphere_ray_quad`、`near_root`、`plane_ray`、`circle_ray`、`cylinder_local`、`cylinder_quad`、`cylinder_caps`、`box_slab`、`box_hit_root`、`ellipsoid_ray_quad`、`local_nearest_hit`、`local_shadow_hit`），净 -60 行。**不为去重多算任何算子**：`*_intersect` 只取近根、`*_crossings` 要两根、`*_shadow` 只要 t——只在三处本来就算同一批算子时才合并。

### 4. 死代码
- cga-core：`multivector::{bivector, vector_part, bivector_part, copy, normalized, weight, bulk, conjugate}`、`primitives::point_pair`；`geometry/geometry.rs` 的 11 变体枚举并入 `geometry/mod.rs`（消除 `clippy::module_inception`）；删空目录 `src/collision`。
- cga-gpu：`geom_kernels::to_f32_3`（原被 `#[allow(dead_code)]` 掩盖）、`mlxops::s_min`、`tol::degenerate_tol`。
- cga-host：`builder::{mat4_mul, translate4, mat4_inv}` 三个纯转发层、`urdf::{xform_point, xform_dir, rigid_inv}`、`scene_build::fmt_f64`（并入 `jsx_gen::fmt_num`）、`react::apply_input`。
- cga-scene：`texture_from_bytes` / `texture_from_png_bytes`。
- cga-examples：bin 内不可达的 `pub fn load_scene`。

### 5. CLI / 测试助手去重
- 5 个 example bin 的 25 行「读 JSX + 跟随 `import './x.css'`」与「换扩展名」惯用法 → `cga-examples/src/lib.rs` 的 `load_scene` / `replace_ext`（含 `render_frames` 的变形版；其原本 `unwrap_or_default()` 吞错改为与其他四个一致 panic，已 smoke 验证帧输出不变）。
- `fnv1a`（collision/xs tests）、`ray`/`ray3`/`tf_ray`/`em_ray`、`contains`(tf/em/cc)、`csg_hit`/`cyc_hit` 各多份 → cga-gpu `lib.rs` 的 `#[cfg(test)]` `test_ray` / `test_contains` / `test_hit`；host 的 `closure_jacobian_fd_check(_reversed_edges)` 参数化合并。

### 6. 刻意不合并（附理由）
| 对 | 理由 |
| --- | --- |
| `csg::plane_crossings` vs `plane_ray` | 算子序列分叉（csg 把 denom 夹进 1e-9、valid 形状不同、另走 `plane_fallback`） |
| `kinematics::lm_solve` vs `lm_solve_j` | 停滞分支语义不同（前者一律 Err，后者判驻点返回 Ok） |
| 各处 `v3_unit`/`vunit` vs `vec3_unit` | 近零兜底方向语义不同（`[0,0,0]` vs `[0,0,1]`） |
| `data_f32`（cga-examples + cga-gpu tests） | 后者是 `#[cfg(test)]`，跨 crate 无法共享 |
| `geom_kernels::tf_hit` vs `em_hit` | 只差 trajectory/euclidean 入口的测试包装，合并需再引入一层间接 |
| `tables.rs` 的 `gp_grade` | 该文件是 `gen_tables.py` 生成的数学常量表（文件头写着 do not edit by hand），模块级 `#[allow(dead_code)]` 只掩盖它一项，不动 |
| `v3_add`/`v3_sub` 别名 vs canonical 名 | 见第 1 节：collision 用短别名，其余 crate 用 canonical 名 |
| `examples/jsx/*.jsx` 之间 0.86 相似度 | 是各演示场景共享的 `<scene>/<camera>/<ground>` 写法，合并会毁掉「每个文件独立可读」的展示价值 |
