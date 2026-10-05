<!-- markdownlint-configure-file {"MD013": false} -->
# CGS 关节化调研：层级场景图 / 姿态参数 / URDF 互转

状态：调研完成（2026-10-06）；P1.1（rpy 关节系）、P4（姿态参数）、P5（URDF 导出）、P6（URDF 导入）已实现，验收见 §4。
适用：`crates/cga-gpu/src/scene_lang/`（关节注册表）、`scene/`（场景结构）、`scene_report.rs`。
前序：`docs/cgs-joint.md`（P1–P3 已实现：joint 八型 / gear 耦合 / cam 接触求解）。

## 0. 起点：P1–P3 给了什么

- `Kinematics` 注册表：`JointDef` 已含 `parent` 链与 `world` 变换（构建期组合）。
- `Scene` 仍是平铺 `Vec<Mesh>`；每个 Mesh 的 `Object3D` 已带 `motor: Option<Multivector>` 字段。
- q 在执行时刻定死，但 q 本来就是表达式（`q=pi/4`、`q=theta` 引用 scope 变量均可）。
- 互操作件全部在位：bake（CSG→水密网格+体积）、OBJ/glTF 读写、无头渲染、可断言报告。

## 1. 层级场景图

现状：joint 嵌套在构建期已组合成树（`joint_stack` → parent 链 + world 矩阵链），但树不持久——落进 `Scene` 后是平铺 mesh 列表，层级只留在报告文本里。

| 路线 | 做法 | 成本 | 评价 |
| --- | --- | --- | --- |
| **(a) 不建树：重求值驱动** | 姿态变化 = 用新 q 重跑 CGS 求值（当前规模毫秒级）。层级只存在为求值顺序 | 极低 | 与"文本即真相/单遍"红线零冲突 |
| **(b) Kinematics 升级持久树** | JointDef 加 `meshes: 区间`，改姿 = 沿链重乘 world 写回各 mesh 的 motor。Scene 不动 | 中 | 支撑关节动画直放；gear/cam 求解器不重跑 |
| **(c) Scene 改真树** | Mesh 挂节点、节点挂节点 | 高 | 推翻渲染输入格式与报告格式，当前无必要 |

结论：**(a) 为主，(b) 推迟**到有外部驱动/长动画需求（JointDef 已留好接口），(c) 不做。

## 2. 姿态参数

现状：q 已是表达式，缺的只是覆盖入口。

方案：`cgs_pose(text, asset_root, overrides)` / CLI `--set name=value`：

- **变量级覆盖**：root scope 在赋值点生效（执行到 `theta = 0.5;` 时写入覆盖值）。零新语法，表达式与 `constrain` 照常工作。
- **关节级覆盖**：joint 语句查覆盖表优先于字面 q；gear/cam 驱动的关节禁止覆盖（显式报错）。覆盖限 1-DOF 关节。

关键设计约束：**gear/cam 链必须在覆盖下重新推导**（driven q 从 driver q 来）——覆盖必须发生在求值时（路线 (a) 的重执行），不能事后改场景。报告加 `pose name=value` 行保持可断言。动画 = 外部循环逐帧 `cgs_pose` + 渲染（`demo_engine` 模式），GPU 是瓶颈，求值开销可忽略。

## 3. URDF 导入/导出

生态（已核实）：Rust 用 `urdf-rs`（openrr，serde-xml 读写，覆盖 link/joint/visual/collision/inertial/limit/mimic）；校验用 `check_urdf`（ROS urdfdom）或本机 urdf skill。不手写 XML 解析。导出侧直接生成 XML 文本（确定性、可断言，与报告同哲学）。

导出（CGS → URDF）映射：

| URDF | 来源 | 备注 |
| --- | --- | --- |
| joint type | JointDef.kind | revolute/prismatic/continuous/fixed/planar 直映；helical → revolute + mimic(prismatic) 双关节；cylindrical → 串联 R+P；spherical → 串联三 R（欧拉奇异要写明） |
| origin xyz | JointDef.at | 直映 |
| origin rpy | axis → z 的对齐旋转 | 需从 axis 反解 |
| axis / limit / mimic | JointDef.axis / limit / GearRel | 直映 |
| visual/collision | joint body 的 CSG → `bake(step)` → 水密 STL/OBJ 引用 | 恰好匹配时用 URDF 原生 sphere/box/cylinder |
| inertial | bake 体积 × 密度 | 惯量张量需在网格上积分，v1 跳过 |

导入（URDF → CGS）：urdf-rs 解析 → 每 joint 生成 `joint(...)` 语句、mesh 引用走现有 `mesh(file=…)`、原生几何直映 CGS 图元（两侧 cylinder 轴向同为 z）。生成走 `cgs_gen` 风格（结构化参数 → 平铺可 diff 源码）。

**导入倒逼回来的缺口**：URDF `origin` 是 `T(xyz)·R(rpy)`，而 CGS joint 的 `at` 只有平移部，关节系无旋转。无损往返需要 **P1.1：`joint` 加 `rpy=` 参数**（关节系 = `ctx·T(at)·R(rpy)`，axis 恒为关节系内值；rpy 为 0 时与现状逐字节一致）。

## 4. 实施顺序与验收

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| **P1.1** ✅ | `joint` 加 `rpy=[roll,pitch,yaw]`（URDF 固定轴约定 R=Rz(yaw)Ry(pitch)Rx(roll)） | rpy 姿态断言（roll 把 ŷ 转 ẑ）；rpy=0 与缺省逐字节一致（报告断言）；元数错误文本 `CGS line N: joint.rpy must be [roll, pitch, yaw]` |
| **P4** ✅ | 姿态参数：`cgs_pose` 库 API + `render_cgs`/`report_cgs` 的 `--set name=value` + 报告 `pose` 行 | 变量覆盖改姿态、关节覆盖改 q、gear 链随动（ratio·覆盖值）、driven/字面 q 并存/多 DOF/越限四条错误文本、报告行可断言 |
| **P5** ✅ | URDF 导出 `cgs_to_urdf`：关节树 + 降级规则（helical→R+mimic(P)、cylindrical→R+P、spherical→3R 串联）+ 原生几何（sphere/box/cylinder）+ 其余烘焙 OBJ + gear→mimic | urdf-rs 读回逐字段比对（往返金样）；helical 降级断言；CSG 体烘焙出非空 OBJ；缺 limit 显式报错 |
| **P6** ✅ | URDF 导入 `urdf_to_cgs`：urdf-rs 解析 → `cgs_gen` 风格 CGS 文本（拓扑排序、mimic→gear、floating 拒绝） | 导入→解析→关节树逐字段一致；floating 显式报错；导入的圆柱体进场景 |
