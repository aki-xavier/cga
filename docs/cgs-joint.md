<!-- markdownlint-configure-file {"MD013": false} -->
# CGS 关节提案：运动副声明、齿轮耦合与凸轮接触（P1→P3）

状态：P1（六型低副 + 报告）、P2（gear 耦合）、P3（cam 接触求解）已实现。
适用：`crates/cga-gpu/src/scene_lang/`（语句分发、注册表）、`scene_report.rs`（报告）。
前序：v2 的 `tag`/`face` 挂点与 `constrain` 求解、v3 的方法链。本提案只加声明与静态求解，不引入运行时求解器。

## 1. 动机

URDF 的 `joint` 是"自由度 + 轴 + 锚点 + 限位"的声明。CGS 此前只有静态平铺场景：
`constrain` 解的是装配位置（解出即烘焙），motor 运动学在 cga-core 但没接进语言。
本提案把运动副接进 CGS，保持两条红线：

- **静态场景，单遍前向**：关节角 q 在语句执行时刻确定（字面值、gear 推导、cam 求解），
  场景只渲染该姿态。文本即真相：没有"先发射后改姿"的回溯。
- **可断言**：关节树、耦合关系、求解结果全部进 `report_cgs` 逐行文本，错误走契约。

## 2. P1：`joint` 语句（六型低副 + fixed/continuous）

语法（语句修饰符，作用于紧随的语句或 `{}` 块，同 `tag`）：

```text
joint("shoulder", type="revolute", axis=[0, 0, 1], at=[0, 0, 0.3], q=0.5, limit=[-1.57, 1.57])
  cylinder(r=0.05, h=0.4);
```

参数：

| 参数 | 默认 | 语义 |
| --- | --- | --- |
| 名（位置参） | 必填 | 关节名，全场景唯一。嵌套 joint 形成父子树 |
| `type` | 必填 | `"revolute"/"continuous"/"prismatic"/"helical"/"cylindrical"/"spherical"/"planar"/"fixed"` |
| `axis` | `[0,0,1]` | 父系中的关节轴（单位化）。prismatic=移动方向；planar=平面法向；spherical/fixed 忽略 |
| `at` | `[0,0,0]` | 父系中的锚点（URDF joint origin 的平移部）。可吃 `face(x,"+z")` 表达式 |
| `q` | 0 | 关节坐标：R/P/H 标量；cylindrical `[qr,qp]`；planar `[x,y,theta]`；spherical `[rx,ry,rz]`（内旋 XYZ） |
| `limit` | 无 | `[lo,hi]`，仅 1-DOF 型（R/P/H）；`continuous` 显式无限位。q 越界 = 编译错误 |
| `pitch` | 无 | helical 必填，每弧度平移量 |

语义（去糖式）：子内容的世界变换 `ctx' = ctx · T(at) · M(q)`，`M(q)` 按类型：

| type | M(q) |
| --- | --- |
| revolute/continuous | Rot(axis, q) |
| prismatic | T(q·axis) |
| helical | T(pitch·q·axis)·Rot(axis, q) |
| cylindrical | T(qp·axis)·Rot(axis, qr) |
| spherical | Rot(ex,rx)·Rot(ey,ry)·Rot(ez,rz)（锚点处内旋 XYZ） |
| planar | T(x·e1+y·e2)·Rot(axis, theta)，e1/e2 ⊥ axis |
| fixed | I（q 必须缺省） |

嵌套 joint 的 body 内层 joint 自动以最近外层 joint 为 parent，注册进 `Kinematics.joints`。

报告（每关节一行，数值六位规范化）：

```text
joint 0 "shoulder" type=revolute parent="root" axis=[0,0,1] at=[0,0,0.3] q=0.5 limit=[-1.57,1.57]
...
summary objects=.. lights=.. joints=2 gears=1 cams=0 ..
```

错误契约（逐字）：

| 错位 | 文本 |
| --- | --- |
| 名缺失/类型非法 | `CGS line N: joint needs a name string` · `CGS line N: joint.type must be "revolute", "continuous", "prismatic", "helical", "cylindrical", "spherical", "planar" or "fixed"` |
| 重名 | `CGS line N: duplicate joint name {s}` |
| 轴为零 | `CGS line N: joint.axis must be nonzero` |
| q 元数错 | `CGS line N: cylindrical joint q must be [qr, qp]`（planar/spherical 同理） |
| q 越界 | `CGS line N: joint {s} q={q} outside limit [{lo}, {hi}]` |
| helical 缺 pitch | `CGS line N: helical joint needs pitch=` |
| fixed 带 q | `CGS line N: fixed joint takes no q` |
| continuous 带 limit | `CGS line N: continuous joint takes no limit` |
| limit 元数错 | `CGS line N: joint.limit must be [lo, hi]` |

planar 基约定（确定性）：`e1 = axis × x̂`（axis 接近 x̂ 时改取 `axis × ŷ`），`e2 = axis × e1`。

## 3. P2：`gear` 语句（匀速比耦合）

```text
gear("driver", "driven", ratio=-0.5, offset=0.0);
```

纯语句（分号结尾，无 body）。语义对齐 URDF `mimic` + 传动比：
`q_driven = ratio · q_driver + offset`。单遍约束：

- driver 必须**已声明**（q 已知）；driven 必须**后声明**，且其 `joint` 语句**省略 q**。
- 两者必须 1-DOF（revolute/continuous/prismatic/helical）。
- 推导出的 q 照常过 limit 检查。

错误契约：

| 错位 | 文本 |
| --- | --- |
| driver 未声明 | `CGS line N: unknown joint {s}` |
| 非 1-DOF | `CGS line N: gear needs 1-DOF joints, got {kind}` |
| driven 自带 q | `CGS line N: joint {s} is driven by gear, q must be omitted` |
| 重复驱动 | `CGS line N: joint {s} is already driven` |
| driven 已声明（次序颠倒） | `CGS line N: gear must precede the driven joint {s}` |

报告：`gear 0 driver="a" driven="b" ratio=-0.5 offset=0` 行。

## 4. P3：`cam` 语句（接触求解，限 blade 轮廓）

```text
cam("jcam", "jfollower",
    driver_profile=circle(c=[0.05, 0, 0], n=[0, 0, 1], r=0.05),
    driven_profile=circle(c=[0, 0, 0], n=[0, 0, 1], r=0.02));
```

纯语句，位置在 driver joint（q 已定）之后、driven joint（省略 q）之前。
执行时刻对 driven 的 q 做 1 维求解：两轮廓在各自关节系给出，
driver 轮廓经 `T1 = ctx₁·T(at₁)·M(q₁)` 就位（已知），
driven 轮廓经 `T2(q) = ctx₂·T(at₂)·M(q)`，求 `q ∈ limit` 使轮廓**外切且不穿透**。

轮廓类型（P3 边界，显式拒绝其余）：

- `circle(c, n, r)`：滚子/偏心圆凸轮。圆-圆接触退化到 ⊥n 过圆心平面内的 2D 外切：
  `|c₁−c₂| = r₁+r₂`。两面法向必须平行于各自关节轴（平面机构），否则报错。
- `plane(n, d)`：平底从动件。圆-平面接触：圆心到平面有符号距离 = r。

求解：f64 CPU，在 `[lo, hi]` 上扫符号变化 + 二分。解唯一性强制：

| 情形 | 文本 |
| --- | --- |
| 无解 | `CGS line N: cam found no contact in limit [{lo}, {hi}]` |
| 多解 | `CGS line N: cam contact is not unique in limit [{lo}, {hi}]` |
| 轮廓类型越界 | `CGS line N: cam profiles must be circle(...) or plane(...)` |
| 轮廓带缩放 | `CGS line N: cam profile must not be scaled` |
| 非平面机构 | `CGS line N: cam profile normal must be parallel to its joint axis` |
| driven 自带 q | `CGS line N: joint {s} is driven by cam, q must be omitted` |
| driven 缺 limit | `CGS line N: cam driven joint needs limit=` |
| driven 非 1-DOF | `CGS line N: cam needs a 1-DOF driven joint, got {kind}` |
| driven 已声明（次序颠倒） | `CGS line N: cam must precede the driven joint {s}` |

报告：`cam 0 driver="jcam" driven="jfollower" q={solved}` 行。

相切语义与 `certify` 同源：接触 = 判别式零集，求解只找根、不猜穿透侧；
driven 的 limit 给出搜索区间，区间内无根即显式失败（不静默取端点）。

## 5. 分阶段验收

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| **P1** ✅ | `joint` 八型 + 嵌套父子 + 报告 joint 行 + summary joints= | 六型姿态正确性（世界锚点/轴向断言）、嵌套 parent 链、limit 越界/重名/零轴/元数/缺 pitch/fixed 带 q 七条错误文本、报告金样 |
| **P2** ✅ | `gear` 语句 + 推导 q + 报告 gear 行 | 双滚筒 −1:2 姿态断言；未声明/非 1-DOF/自带 q/重复驱动四条错误文本 |
| **P3** ✅ | `cam` 圆/平面轮廓接触求解 + 报告 cam 行 | 偏心圆凸轮+滚子从动件解 = 解析值；平底从动件（圆-平面）；无解/多解/轮廓越界/非平面机构/自带 q 五条错误文本 |

明确不做：运行时关节动画（场景只渲染声明姿态）、非平面机构凸轮、
轮廓穿透区间分析（接触是单边约束，解即接触点）、URDF 文件互转（后续单独立项）。
