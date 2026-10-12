<!-- markdownlint-configure-file {"MD013": false} -->
# 图元几何的统一定义（CGA 单一来源）

状态：**阶段 1 已落地（2026-10-12）**。范围：新增 `crates/cga-core/src/query.rs`，并改写
`cga-mesh` / `cga-collision` / `cga-gpu` / `cga-host` 的消费者。阶段 2/3（参数化 seam
合并、射线层）见 §4，未做。

## 0. 问题（实测清点）

同一几何知识此前分散在 **5 条独立管线**：

| 管线 | 位置 | 覆盖 |
| --- | --- | --- |
| cga-core 参数层 | `geometry/identity_params.rs`、`primitives.rs`、`cyclide.rs` | 唯一的参数化；但 `plane_dist`/`sphere_dist`/`Cylinder::dist`/`DupinCyclide` 生产零调用 |
| cga-gpu f32 数组 | `geometry_ops.rs`、`geometry_extra.rs`、`csg.rs` | 求交/包含/UV/包围盒（全 f32） |
| cga-gpu f64 认证 | `fallback.rs`、`certify/` | 同一批公式的第二份 f64 标量实现 |
| cga-gpu MSL | `bvh_trace.rs`（MSL 字符串） | 第三份 f32 求交，5 个图元 |
| cga-host / cga-collision / cga-mesh | `collision.rs`、`lib.rs`/`pairs.rs`/`contacts.rs`、`bake.rs` | 点包含、SDF、支撑、最近点、隐式场、包围盒 |

量化（改造前）：点包含 **5 份**、光线求交 **4 份 + MSL 第 5 份**、环面隐式/四次式 **6 份**、
圆锥 `k2` **5–6 份**、包围盒 **3 份**、点–形状 SDF/支撑/最近点各 **2–3 份**。

**关键区分（决定了"统一"不等于"合并成一份"）**：`field`（符号隐式）与 `sdf`（精确欧氏距离）
**不是同一个函数**——sphere/plane 上同号，box/cylinder/cone 上是 max 型隐式 vs 精确距离
（`bake.rs` 对照 `pairs.rs`）。所以统一的目标是：**每一类函数各自只有一份**，语义不混。

## 1. 决定

1. **唯一实现在 `cga_core::query`**（代数核心 crate 内），其余 crate 只做分发与坐标系搬运。
2. **CGA 锚点**：
   - blade 图元（sphere/plane/circle/line）的 `field` 就是它的 GA 关联谓词——
     `X·ip(S) = −(|p−c|²−r²)/2`（球，`field = −2·ip`）、`X·ip(P) = 有符号距离`（平面，`field = ip`），
     由 `tests::blade_predicates_agree_with_field` 钉死；算术形式只是这些谓词的快速取值。
   - `box` = 6 个半空间 blade 的交（`box_field` 即逐轴 max 折叠）；`cylinder` 的轴 = line blade。
   - `cone`/`torus`/`ellipsoid`/`cyclide` 用 `(a_inv3, t_inv)` 的共形仿射系进入规范系
     （与渲染器同一套 `affine_from_motor` 约定）；环纹面的场直接调用 `DupinCyclide::implicit`
     （cga-core 里早已完整、此前生产零调用——本次接线）。
3. **三类语义分开、各自一份**（§3）。
4. **GPU 的 f32 数组内核与 MSL 是性能实现，不是第二份定义**：
   - 数组侧由漂移测试钉住：`cga-gpu::csg::tests::test_array_contains_agrees_with_core_sign_definition`
     —— 10 个图元配置 × 729 采样点，数组判定必须与 `GeometryParams::field` 同号
     （贴面样本跳过：f32 舍入 + 数组侧严格 `<` / 标量侧 `<=` 的边界约定差异）。
   - MSL 侧由既有的 40 对象整帧等价测试看守；逐图元逐光线的漂移测试列入阶段 2。
5. 消费者侧：**不允许再出现图元公式**（只允许坐标系搬运、三值到 `Hit` 的映射、
   碰撞特有的保守策略）。

## 2. 阶段 1 落地清单

| 位置 | 动作 |
| --- | --- |
| `cga-core/src/query.rs`（新增） | `Membership`（三值 Kleene 组合子）、`to_local`/`a_fwd_point`、`GeometryParams::{field, contains, distance, bounds}`、精确欧氏族（`sdf_*`、`closest_point_*`、`support_point_*`、`support_range_*`、`ellipsoid_closest*`、`dist_seg`）+ 11 个闭式测试 |
| `cga-mesh/src/bake.rs` | 删 `to_local` + 9 个 `*_field` + `csg_field` + `affine_field` + `bounds` + `apply_fwd`/`corners_bounds`/`combine_bounds`/`Fold`；`bake` 只留四面体 marching / 焊接 / 朝向 |
| `cga-collision/src/lib.rs` | `local_contains`/`bool_hit` 删除 → `fig.identity_params().contains(p)`（三值映射到 `Hit`）；`local_bounds` 只留两处**碰撞策略**（圆片零厚度板、环纹面 None），其余走 core 的 `bounds` |
| `cga-collision/src/pairs.rs` | `sdf`/`sdf_box`/`sdf_cylinder`/`sdf_cone`/`dist_seg`/`support_range` 全部改为分发 core |
| `cga-collision/src/contacts.rs` | `closest_point`/`closest_on_seg`/`support_point`/`ellipsoid_closest` 全部改为分发 core |
| `cga-core/src/primitives.rs` | GA 侧 `plane_dist`/`sphere_dist`/`Cylinder::dist` 接到 `query`（此前三份独立公式，其中 `Cylinder::dist` 生产零调用） |
| `cga-host/src/collision.rs` | 两处 `BakeExt::bounds` UFCS 调用改为固有方法（`identity_params().bounds()`） |

## 3. 语义三分（表）

| 函数 | 语义 | 唯一实现 | 消费者 |
| --- | --- | --- | --- |
| `field` | 符号隐式（<0 内 / >0 外 / =0 面），量纲不承诺 | `query.rs` | 网格化（marching）、CSG 折叠、GPU 数组内核（漂移测试）、碰撞判号的符号 |
| `contains` | `field` 的符号化，**三值**（圆片/环纹面/部分环面/未定 → `Unknown`） | `query.rs` | 碰撞 `contains_point`、JSX `${...}` 查询 |
| `distance` / `closest_point` / `support_point` / `support_range` | **精确欧氏**量（无闭式的图元 → `None`，绝不近似冒充） | `query.rs` | 碰撞窄相（球–X、平面–X、接触流形） |
| `bounds` | 保守轴向包围盒（`None` = 无界） | `query.rs` | 网格/质量属性/导出的包围盒；碰撞宽相（+ 两处碰撞策略） |

## 4. 阶段 2/3（未做）

**阶段 2 — 参数化 seam 合并**。`Geometry::identity_params()`（core）与 `geom_to_camera()`
（gpu）是同一件事的两份 11 变体 switch：前者给 cylinder/box/circle 硬编码原点/`+z`，
后者把电机折进参数（`m.apply(blade)` / `m.apply(e1/e2/e3)`）。合并成 core 的一个
`Geometry::params_in_frame(motor, linear)`，gpu/host 调用它。验收：gpu 侧所有渲染金标不变，
且 `identity_params() == params_in_frame(identity)`（逐字段相等测试）。

**阶段 3 — 射线层**。求交仍有 4 份：数组 f32、MSL、f64 `fallback`、`certify` 的系数提取。
canonical 定为 **f64 标量**（`fallback.rs` 的 `*_rows` 上移到 core），数组与 MSL 作为性能
实现由**逐图元逐光线**的漂移测试钉住（随机光线集合上 `(t, 命中)` 一致，t 相对 1e-5）。
MSL 与前两者语言不同、无法共享源码 —— 它的统一形态就是"同构 + 测试"，并在 `bvh_trace.rs`
的注释里写明与数组内核的对应关系。验收：三个实现 × 9 个图元的随机光线一致性测试 + 现有
整帧金标不变。

## 5. 明确拒绝的替代方案（留档，避免重复讨论）

| 方案 | 为什么不 |
| --- | --- |
| MSL 里直接实现 32 分量 GA（`gp` 表驱动）做求交 | 把每像素的 18 flops 级判定膨胀成上千次查表乘法，且二次曲面求根并不因 GA 变简单；渲染成本数量级上涨，收益只有"叫法统一" |
| 数组内核逐光线调用标量 CGA | 丢掉 MLX 的向量化 ⇒ 等于删掉渲染器 |
| 把 4 个非 blade 图元（cone/torus/ellipsoid/cyclide）硬改成 blade | 它们在 CGA 里是退化二次曲面/共形包络，改写会改渲染语义与金标；README:244 的现状是**有意**的，收益只是命名 |
| `build.rs` 代码生成（一份 spec 生成 MLX + MSL） | 引入不可重放脚本（`gen_tables.py` 的教训：输入不在库）与构建期工具链；漂移测试已能拦住失配。**复核点**：若 MSL 覆盖的图元数从 5 增加，重新评估 |

## 6. 验收（阶段 1）

- `cargo test --workspace` 全绿（363 测试）；`cargo clippy --workspace --all-targets` 0 警告。
- 渲染路径未改：7 张 gallery 金标逐字节不变（改动只碰质量/碰撞/网格/查询路径）。
- 漂移测试：数组 f32 ↔ 标量定义同号（10 图元 × 729 采样点，含 CSG 差集与仿射包装）。
- 新增闭式断言（cga-core，11 个测试）：GA 谓词恒等（球/平面）、球/平面/盒/柱/锥/环/椭球/环纹面/圆片
  的 field·contains·distance、CSG 三值折叠与符号折叠、仿射折叠、包围盒闭式、支撑/最近点闭式。
