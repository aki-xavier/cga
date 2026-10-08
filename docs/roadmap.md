<!-- markdownlint-configure-file {"MD013": false} -->
# 路线图（roadmap）

状态：**当前（2026-10-08）**。本文档是方向的依赖关系与排序；每个方向落地时各自单独立项（参照 `css-conformance.md` / `collision-plan.md` / `kinematics-graph.md` 的做法：计划文档 → 阶段 → 验收）。

## 1. 已建成（全部推送，253 测试全绿）

- **渲染器**：解析光线追踪（全图元 + CSG + 仿射 + 认证数值回退）、帧间像素级增量渲染（与全帧逐位一致）。
- **JSX+CSS 宿主**：真 React 19（hooks/context/memo/key/事件派发/宿主输入）、CSS 真匹配真级联（含错误契约）、帧间子树复用、分组（`Object::group`）、碰撞查询惰性求值。
- **碰撞检测**（`cga-collision`）：三值重叠判定、凸对分离距离（含精确 MTV）、接触流形、螺旋 CCD（认证无接触）、关节行程干涉扫描。
- **运动学图模型**：`<link>/<pair>/<anchor>` 无因果建模（Modelica 调研后设计）、生成树定向传播、gear/cam 无方向、`<closure>` 闭链 LM 求解、`guess` 选支。
- **crate 卫生**：cga-core 零依赖、cga-collision / cga-mesh 独立、cga-gpu（MLX）/ cga-host（boa/swc）/ cga-examples。

## 2. 方向与依赖

```
A 交互闭环（拾取→事件→增量渲染）   ← 唯一串起所有已建成子系统的方向
B 速度级运动学（twist/雅可比）     ← 依赖：图模型（已有）
C 物理属性 + 静力学               ← 依赖：bake 体积（已有）+ B 的 wrench 对偶
D 碰撞性能打磨                    ← 随时可做，无依赖
E React 差距（优先级分层/lazy）    ← 没有消费者，不做
G5+ 空间闭链 / 多自由度环路         ← 依赖：B（雅可比复用）
```

**明确不做**（记录在案，不重复讨论）：接触动力学（冲量/摩擦/时间积分）。理由：与本项目"确切或 Unknown"的验证文化冲突（摩擦是模型、积分有离散误差、混沌敏感钉不住金标），且是另一个产品。真有需求时是独立 crate，把 `cga-collision` 当库用。

## 3. 各方向展开

### A. 交互闭环（推荐先做）

- 拾取：渲染器每像素已算 `best_idx`（命中对象下标）——拾取 = 一条光线的解析求交，几乎免费。API：`pick(scene, camera, x, y) -> Option<(object_index, point, normal, instance_id?)>`。
- 接线：对象下标 → React 实例（发射时记下实例 id；需要对象→实例的反向映射）→ `dispatch(id, "onClick", payload)`。
- 验收链：点选坐标 → 对象下标的解析断言；onClick → setState → 局部重建（create=0）→ 增量渲染与全帧逐位一致。
- 形态：无头可交互查看器（宿主驱动，不是窗口程序）；事件源抽象（先支持 pick，指针移动/拖拽后续）。

### B. 速度级运动学

- twist 沿生成树传播（`Multivector::velocity` / `extract_velocity` 现成）；雅可比矩阵（连杆末端 twist 对 q 的偏导）；活动度 = 约束秩分析（构图期诊断奇异/冗余约束）。
- 仍是几何不是动力学：可闭式验证（旋转副的雅可比列有解析形式）。
- 为 C 与 G5+ 供底座。

### C. 物理属性 + 静力学

- 质量属性：解析实体闭式（球/盒/柱/锥/椭球），CSG 体积分（与 `bake` 水密网格体积交叉验证）。`density` prop + 报告行。
- 准静态：重力下静平衡（认证求根，无时间积分）；装配静力校核（wrench 是 twist 对偶）。
- 不含接触动力学（见 §2 明确不做）。

### D. 碰撞性能打磨

- `scan_contacts` 的 Yes 对接触解帧间缓存（现在每帧重解）；
- 齿轮/凸轮联动的行程扫描（`sweep_joint` 目前是单副冻结其余）；
- 对象数量上量后的空间索引（包围盒剪枝不够时）。

### E. React 差距（不做，直到有消费者）

优先级分层（`resolveUpdatePriority` 恒 Default）、`lazy()`/动态 `import()`（无模块加载 shim）、DevTools。没有落点的机制不做。

### G5+. 空间闭链 / 多自由度环路

- 现 LM 只解 1-DOF pair；多自由度环路（球副/平面副在环上）需要速度级雅可比（依赖 B）。
- Stewart 平台类：位置级约束 + 分析雅可比。

## 4. 推荐顺序

**A → B → C**，D 穿插，E/G5+ 由真实需求驱动。理由：A 把项目从"渲染器 + 库"变成"可交互应用"，且验收链与既有纪律同构（解析断言 + 逐位一致）；B、C 逐级抬高机构分析的语义层，全部可验证。
