# Scene Report 提案：CGS 执行结果的可断言文本格式

状态：提案（未实施），待评审。
适用：`crates/cga-gpu/src/scene.rs`、`scene_lang.rs`（4462 行快照）、新模块 `scene_report.rs`。
前序：`docs/cgs-v2.md`（P0–P3 已实施）、`docs/cgs-v3.md`（P4–P6 提案）。本提案与两者正交——
v2/v3 改语言的输入轴，本提案定义执行结果的输出轴，不改任何加载语义。
诊断行号为 2026-10-04 快照（测试基线 166），随代码漂移，以符号名为准。

## 1. 动机与目标

CGS 脚本执行后，结果今天只有三种出口：

1. **PNG 渲染**（`render_cgs`）——像素级，LLM 无法逐值断言；
2. **OBJ/GLB bake**（`bake_cgs`）——三角汤，体量大到不可断言；
3. **Rust 内部断言**——`assert!((sc.objects[0].position[0] - 1.0).abs() < 1e-9)`
   式的逐字段检查，只活在测试代码里，LLM 拿不到可比对的文本。

本提案定义**第四种出口：Scene Report**——把一个 `Scene`（+相机+tag 注册表）格式化为
确定性、逐行可断言的文本，直接交给 LLM 做结果检验（golden 比对、逐行子串断言、
"孔在 x=1.2" / "物体 min y=0 坐在地上" 式几何验证）。

**目标：**

- **确定性**：同一场景两次生成逐字节相同；HashMap 迭代序等非确定源全部消除。
- **完整语义**：背景/相机/灯光 + 每个对象的几何参数树（CSG 递归）、世界变换、
  材质，全部显式打印（默认值也打印，断言不需要猜）。
- **tag 注册表**：`tag("x")` 注册的实例（世界帧 + 局部几何）进报告——这正是
  P5 `instances()` 读取的那份数据，报告先给它一个出口。
- **语法按可回读设计**：本期只实现输出（无 parser），但字面量、参数名、函数名
  与 CGS 逐位对齐（`0x` 颜色、`[x,y,z]` 列表、`sphere(r=…)` 形），使未来回读器
  可以直接复用 CGS 词法与函数表（§7）。
- **零新增错误契约**：报告生成是全函数（total），不产生任何新错误文本；加载错误
  沿用 `cgs_load_result` 原路（§9）。

**非目标：**

- 不写回读 parser（未来项，本提案只保证语法同形）；
- 不输出体积/网格顶点/纹理像素/相机 motor/`Inst.rel`（见 §7 差距表）；
- 不引入 JSON/serde 双格式（文本第一；JSON 若需要是另一个视图）；
- 不动 `cgs_load` / `cgs_load_result` 的签名与行为（加法改造，166 测试逐位不动）。

## 2. 现状事实表（全部实测）

| 事实 | 位置 | 对提案的意义 |
|---|---|---|
| `Scene { objects: Vec<Mesh>, lights, background }`；`Mesh { base: Object3D, geometry, material }` | `scene.rs:50`、`scene.rs:6` | 报告字段的唯一数据来源 |
| `Object3D { position, rotation_axis, rotation_angle, motor_override, linear }`；`motor()` 统一返回刚体 motor | `scene_graph.rs:76`、`113` | 世界变换统一走 `motor()`（CGS 路径恒为 `Some(motor)`） |
| `Geometry` 12 变体；`CylinderGeometry.half = -1` 表示无界；`TorusGeometry.arc` 默认 TAU；`TrimeshGeometry` 自带 `lo/hi/n_faces`；`AffineGeometry { inner, linear, motor }`；`CsgGeometry { op, children }` | `cga-core/geometry.rs:194`、`136–157`、`348`、`220`、`affine_geom.rs:6`、`csg_node.rs:11` | §5.4 逐变体映射；无界/默认值语义直接来自存储 |
| `add_geometry`：`decompose_rigid(ctx)` → 刚体进 mesh motor，`linear≠I` 时几何包 `AffineGeometry`；`mesh()` 的 `linear` 恒 `identity3()` | `scene_lang.rs:2914–2940`、`scene.rs:28` | 对象行 = 刚体语句前缀 + `frame(lin=…)` 包裹（分解顺序 `world = motor∘linear`，见 `affine_from_motor` `affine.rs:44`） |
| `csg_block`：每个子几何**无条件**包 `transformed_geometry(geo, cm, cl)`；结果几何是世界系（发射上下文已烤入）、实例槽 identity、mesh motor=identity | `scene_lang.rs:2962–2995` | CSG 子节点必出 `frame()` token；CSG 对象行无前缀、无平移键 |
| `register` → `Inst { geo, world, rel }` 存进 `named: HashMap<String, Vec<Inst>>`，`Vec` push 序 = 发射序 | `scene_lang.rs:1698`、`1066`、`1060` | tag 段：名按字典序（消 HashMap 序），实例按发射序；`rel` 不入报告 |
| `cgs_load_result` 出口把 `named` 整个丢弃 | `scene_lang.rs:1077–1121` | 需新增 `CgsRun` 出口承载注册表；`cgs_load*` 改为委托、签名不动 |
| `plane(n, d)` 构造 blade = `mv_vector(nx,ny,nz, 0, d)`；`geom_to_camera` 反解 `d = pi.einf_coeff()` | `primitives.rs:16–28`、`geom_kernels.rs:301–307` | `plane(n=…, d=…)` 提取与构造互逆（局部系、恒等 motor 下） |
| `geom_to_camera` 覆盖全部 12 变体（含 `AffineGeometry`/`CsgGeometry` 递归） | `geom_kernels.rs:286–403` | 世界系参数与 bbox 的现成通路（`bake_cgs.rs:41` 同款用法） |
| `bounds_of`：plane→None、circle→None、`h<0` cylinder→None；Difference→**首子** bounds、Union→并、Intersection→交 | `bake.rs:468–541` | `no_bounds=N` 计数与 summary bbox 语义按此定义 |
| motor→轴角现成件：`motor.to_matrix()` → `matrix_to_quaternion` → `2·atan2(‖xyz‖, w)` 范式 | `motors.rs:103`、`48` | 对象/实例旋转输出的唯一提取路径（经矩阵，约定无歧义） |
| `basic_material` 存 `kind=Basic, roughness=0, metalness=0, emissive=黑, ior=1.5, absorption=0, map=None` | `shading.rs:50–62` | `unlit=true` 行的字段值就是存储值，回读自洽 |
| CGS kw 表：`sphere r` / `plane n(+d默认0)` / `cylinder r(+h默认-1)` / `box s` / `cone r,h` / `torus R,r` / `cyclide a,b,d` / `ellipsoid radii` / `translate t` / `rotate axis,angle` / `background color`；material kw = color/map/unlit/roughness/metalness/emissive/opacity/ior/absorption；camera 语句只读 fov/aspect/position/target 四键 | `scene_lang.rs:3448–3525`、`2451–2489` | 报告 kw 名与 CGS **逐位一致**（`cgs_sig_names`/`cgs_sig_defaults` 是命名权威） |
| CSG 语句位两形：块 `difference() { … }`、表达式 `difference(a, b);`（2348–2355）；`translate/rotate/material/background/camera` 均 statement-only | `scene_lang.rs:2330–2355`、`717–735` | 载荷行 = 语句位形状；`examples/cgs/*.cgs` 实证语法 |
| `cgs_gen` 生成的 CGS 已用 `0xRRGGBB` 字面量 | `cgs_gen.rs:120` | 0x 颜色走既有词法，回读有据 |
| `Texture { pixels, width, height, is_linear }`——**不存源路径** | `texture.rs:14` | `map=WxH` 差距的根因 |
| 工作树并行未提交改动：`degenerate.rs`、`geom_kernels.rs`、`lib.rs`、`docs/freeform-robust-boolean.md`（M）、`tol.rs`（??） | git status | §11 实施风险（lib.rs 挂模块须选择性暂存） |

## 3. 报告语法总览

报告是**逐行、LF 结尾、无缩进**的文本，行分三类（按行首分派）：

1. **纯载荷行**——`background(…)` / `camera(…)` / 灯等场景级记录，行尾 `;`。
2. **框架前缀 + 载荷行**——`object <i> <载荷>;` 与 `tag "<名>" <j> <载荷>;`，
   行尾 `;`；载荷形状是 CGS 语句（去框架前缀后可直接当语句读）。
3. **纯框架行**——`scene …`、`bounds <i> …`、`tag "<名>" count=K`、`summary …`，
   **无分号**；给 LLM 定位与计数。框架词 ∈ { `scene`, `object`, `bounds`, `tag`,
   `summary` }，均非 CGS 函数名（`tag` 框架行实参带引号，与 `tag(…)` 调用形可区分）。

完整示例（源脚本 → 报告）：

```
// ---- 源 (illustrative) ----
background(color=0x20313A);
camera(fov=50, aspect=1.777778, position=[0,0,5], target=[0,0,0]);
ambient_light(intensity=0.3);
directional_light(direction=[0,-1,0]);
tag("hole") translate([1.2,0,0]) rotate(axis=[0,0,1], angle=pi/2)
  material(color=0xFF0000) sphere(r=0.5);
difference() {
  translate([0.5,0,0]) box(s=[2,2,2]);
  sphere(r=0.5);
}
```

```
scene version=1
background(color=0x20313A);
camera(fov=50, aspect=1.777778, position=[0,0,5], target=[0,0,0], up=[0,1,0], near=0.1, far=100);
ambient_light(color=0xFFFFFF, intensity=0.3);
directional_light(direction=[0,-1,0], color=0xFFFFFF, intensity=1);
object 0 translate([1.2,0,0]) rotate(axis=[0,0,1], angle=1.570796) material(color=0xFF0000, roughness=0.5, metalness=0, emissive=0x000000, opacity=1, ior=1.5, absorption=0) sphere(r=0.5);
bounds 0 lo=[0.7,-0.5,-0.5] hi=[1.7,0.5,0.5]
object 1 material(color=0xFFFFFF, roughness=0.5, metalness=0, emissive=0x000000, opacity=1, ior=1.5, absorption=0) difference(frame(t=[0.5,0,0], box(s=[2,2,2])), sphere(r=0.5));
bounds 1 lo=[-0.5,-1,-1] hi=[1.5,1,1]
tag "hole" count=1
tag "hole" 0 translate([1.2,0,0]) rotate(axis=[0,0,1], angle=1.570796) sphere(r=0.5);
summary objects=2 lights=2 no_bounds=0 bbox_lo=[-0.5,-1,-1] bbox_hi=[1.7,1,1]
```

要点（与逐字段规范 §5 对应）：

- `object <i>` 行：`[刚体语句前缀] material(…) <几何表达式>;`，紧随其后一行
  `bounds <i> …` 给该对象的世界系包围盒（`bounds_of=None` 时为 `bounds <i> none`）。
- CSG 对象的实例槽是 identity（§2 第 5 行），所以 `object 1` 没有前缀；子节点帧进
  `frame(…)` token：`frame(t=[0.5,0,0], box(s=[2,2,2]))`，恒等子帧不打 token（直接裸几何）。
- tag 段两行式：`tag "<名>" count=<K>` + K 行 `tag "<名>" <序> <与对象同形的帧+几何>;`。
  实例无 material（材质不在注册表里）。
- `summary` 收尾：对象/灯计数、无包围盒对象数、全场 bbox 并集。

## 4. 行序与确定性

报告行序固定如下（任何场景都是这个骨架，缺项跳过）：

1. `scene version=1`
2. `background(color=0x…);`（缺省时打印 `scene()` 的默认 `0x87CEEB`，同样打印）
3. `camera(…);`——**总是存在**：`cgs_load_result` 无 camera 语句时合成默认相机
   （fov 50、aspect 16/9、position [0,0,5]），照实打印
4. 灯行 ×N，按 `scene.lights` 发射序
5. 每个对象：`object <i> …;` 行 + `bounds <i> …` 行，`i` = `scene.objects` 下标
6. tag 段：tag 名按**字典序**（`BTreeMap`），名内实例按发射序
7. `summary objects=… lights=… no_bounds=… bbox_lo=… bbox_hi=…`

其他确定性规则：

- 颜色 = 存储值反解 8bit/通道 hex（`round(c*255)`，裁剪 0..255），`0xRRGGBB` 大写。
- 全部键序固定（material：color, roughness, metalness, emissive, opacity, ior,
  absorption[, map][, unlit]；camera：fov, aspect, position, target, up, near, far；
  灯：direction|position, color, intensity；frame：t, axis, angle, lin）。
- 向量/矩阵无空格：`[1.2,0,0]`、`[[2,0,0],[0,1,0],[0,0,1]]`。
- 空 tag 名单（count=0）不可能出现——`named` 的 Vec 只在 push 时创建。

## 5. 字段规范

### 5.1 数值格式（六位规范化）

对每个 f64 `v`：

1. `format!("{:.6}", v)`；
2. 去尾部 `0`，再若以 `.` 结尾去 `.`；
3. 结果解析回数值若为 0（含 `-0.000000`、`-0`）→ 输出 `0`；
4. 非有限值输出 `nan` / `inf` / `-inf`（诊断用，回读差距 §7）。

因此：`pi/2 → 1.570796`、`16/9 → 1.777778`、`1.0 → 1`、`1e-9 → 0`、`TAU → 6.283185`。
**精度契约**：每个数值绝对误差 ≤ 5e-7；报告文本相等 ⇒ 数值在 1e-6 级一致。
不产生科学计数法。

### 5.2 变换（刚体前缀与 frame token）

- **提取路径**：`motor().to_matrix()` → 平移 = `[m[3], m[7], m[11]]`；旋转 = 左上 3×3
  → `matrix_to_quaternion` → `angle = 2·atan2(‖xyz‖, w)`；`angle > π` 时取
  `2π − angle` 并轴取反 ⇒ 规范 `angle ∈ [0, π]`，轴单位化（与输入符号歧义唯一化）。
- **语句槽**（对象行前缀、tag 实例行前缀）：`translate([x,y,z]) rotate(axis=[a,b,c], angle=A)`，
  即 CGS 语句链形状；顺序恒为 translate 在前（`world = t∘r`，先转后移）。
- **表达式槽**（CSG 子节点、Affine 内层、对象行的 `mesh.linear≠I` 情形）用
  `frame(…)` token：键序 `t`（平移三元组）、`axis`+`angle`（旋转，成对出现）、
  `lin`（3×3 行列表）。
- **恒等省略（结构性，非语义参数）**：平移三元组六位规范化后全 0 → 省 `t`；
  角度规范化后为 0 → 省 `axis`+`angle`；`lin` 六位后等于单位阵 → 省 `lin`；
  **键全空 → 整个 `frame(...)` 不出现**（裸几何）。对象行的**前缀**只来自
  `mesh.motor`（CGS 路径恒有、可为 identity）；`frame(lin=…)` 出现在几何槽时来自
  `AffineGeometry`（`scale`/非均匀变换路径，CSG 子节点同理），或手工构造场景的
  `mesh.linear≠I`。
- 语义参数**不做恒等省略**（torus 的 `arc`、cyclide 的 `shift` 即使是默认值也打印，
  见 §5.4）——省略只发生在帧键，避免把"形状参数"和"坐标帧"两种省略混为一谈。

### 5.3 场景级载荷行

- `background(color=0xRRGGBB);`
- `camera(fov=…, aspect=…, position=[…], target=[…], up=[…], near=…, far=…);`
  —— CGS `camera` 语句只读 fov/aspect/position/target 四键，near/far/up 固定
  0.1/100/[0,1,0]（`scene_lang.rs:2486`）；报告照实打印全部七键，此三键为信息性
  （CGS 路径恒同值），回读差距见 §7。
- 灯：`ambient_light(color=…, intensity=…);`、
  `directional_light(direction=[…], color=…, intensity=…);`、
  `point_light(position=[…], color=…, intensity=…);`——kw 名即 CGS 函数名与参数名，
  `kind` 不另设键（由函数名承载）。

### 5.4 几何表达式（12 变体 → token）

参数名取 `cgs_sig_names`；值按 §5.1/§5.2 格式；CSG 递归、单行平铺：

| 存储 | 输出 token | 说明 |
|---|---|---|
| `SphereGeometry{radius}` | `sphere(r=…)` | |
| `PlaneGeometry{blade}` | `plane(n=[…], d=…)` | 局部系反解：`n = unit(blade.euclidean_vector())`、`d = blade.einf_coeff()`（§2 互逆对） |
| `CylinderGeometry{radius, half}` | `cylinder(r=…, h=…)` | `h = -1`（无界，存储 `half=-1`）否则 `h = 2·half`（CGS 全长语义） |
| `BoxGeometry{half}` | `box(s=[…])` | `s = 2·half`（CGS 全长语义，二进制精确） |
| `CircleGeometry{radius}` | `circle(r=…)` | |
| `ConeGeometry{radius, height}` | `cone(r=…, h=…)` | |
| `TorusGeometry{major, minor, arc}` | `torus(R=…, r=…, arc=…)` | `arc` 恒打印（CGS 默认 TAU → `6.283185`；arc≠TAU 来自非 CGS 路径，§7） |
| `EllipsoidGeometry{radii}` | `ellipsoid(radii=[…])` | |
| `CyclideGeometry{a,b,d,shift}` | `cyclide(a=…, b=…, d=…, shift=[…])` | `shift` 恒打印（CGS 恒 `[0,0,0]`） |
| `TrimeshGeometry{n_faces, lo, hi}` | `trimesh(faces=…, lo=[…], hi=[…])` | 局部系；extrude/loft/mesh 导入的落点；顶点不打印（§7） |
| `CsgGeometry{op, children}` | `union(…)` / `difference(…)` / `intersection(…)` | 名 = `csg_op_name`（`scene_lang.rs:3311`）；子节点按**存储序（发射序）**打印，参数逗号空格分隔、递归同形 |
| `AffineGeometry{inner, linear, motor}` | 外层刚体帧 + `frame(lin=[[…]], …)` 包内层 | 与 §5.2 规则合成：motor 部分按所在槽位（语句槽→前缀 / 表达式槽→frame 键）处理，linear 部分恒为 `frame(lin=…)` |

### 5.5 材质

对象行内、几何表达式**之前**（与 CGS 语句链同序：帧 → material → 几何）：

```
material(color=0x…, roughness=…, metalness=…, emissive=0x…, opacity=…, ior=…, absorption=…);
```

- 全字段恒打印（断言不猜默认）；有纹理追加 `, map=<W>x<H>`；`kind=Basic` 追加 `, unlit=true`
  （round-trip：`unlit=true` → `basic_material`，其存储值即打印值，自洽）。
- tag 实例行无 material（注册表只存几何+帧）。

### 5.6 tag 注册表

- 头行 `tag "<名>" count=<K>`（K≥1，不可能为 0，§4）。
- 实例行 `tag "<名>" <j> <帧前缀> <几何表达式>;`，`j` 从 0 发射序递增。
- 帧 = `Inst.world`；CSG 实例的 world 恒为 identity（`register(&result, mat4_identity(), ctx)`
  ——世界系已烤入几何），故该行无前缀、几何是世界系 CSG 树。
- `Inst.rel` 不打印（`drill` 复用的内部量，报告是结果验证面，§7 差距）。
- 名含 `"` 或 `\` 时按 CGS 字符串词法转义输出（实施时以 `cgs_lex` 实测为准；常规名无此问题）。

### 5.7 summary

`summary objects=<N> lights=<L> no_bounds=<U> bbox_lo=[…] bbox_hi=[…]`

- `no_bounds` = `bounds_of(geom_to_camera(obj))` 为 None 的对象数（plane、无界 cylinder、
  circle——`bake.rs:474/476/539`）。
- `bbox_*` = 有界对象世界 bbox 的并集；当 `N=0` 或全部对象无界时**省略两个 bbox 键**。
- 语义完全锚定 `bounds_of`（CSG Difference 取**首子** bounds 等），与 `bake_cgs` 的
  skip 判定同源。

## 6. 与 P4–P6 的关系

- 正交：v3 是输入轴（语言怎么写），本提案是输出轴（结果怎么验），互不依赖，实施顺序任意。
- 与 P5 共享数据：tag 注册表就是 `named`；本提案先给它公开出口（`CgsRun.tags`），
  P5 的 `instances()` 仍然直接读 `named`，两者不冲突、不互相等待。
- 错误契约零交集：报告不新增任何诊断文本（§9）。

## 7. 可回读性：对齐点与差距表

**对齐点（未来 parser 可直接复用 CGS 设施）：**

- 字面量：`0xRRGGBB` 颜色、`[x,y,z]` 列表、`-1.570796` 数字——全部走 `cgs_lex` 既有分支；
- kw/函数名：与 `cgs_sig_names`、几何构造器、灯/背景/相机语句逐位一致；
- 载荷行形状 = CGS 语句位形状（帧链 + material + 几何），CSG 用表达式形
  `difference(a, b)`（`scene_lang.rs:2348` 已支持）；
- 框架词与 CGS 无冲突（`scene/object/bounds/summary` 非 CGS 名；`tag` 框架行带引号实参，
  与 `tag(…)` 调用形可区分）。

**差距表（本期明示、不隐藏）：**

| 项 | 状态 | 原因/去向 |
|---|---|---|
| `frame(...)` token | 回读 ✗ | CGS 无对应字面量；回读器可映射为 motor∘linear 矩阵或 `at/rot/scaled` 组合 |
| `trimesh(faces=…, lo=…, hi=…)` | 回读 ✗ | 无顶点即无法重建网格；验证面就是 faces+bbox |
| `bounds <i> …` / `summary` / `scene` / `tag …` 框架行 | 回读 n/a | 报告元数据，非场景内容 |
| camera 的 `near/far/up` 键 | 回读 ⚠ | CGS `camera` 语句不读这三键（丢弃）；但 CGS 重建时固定输出 0.1/100/[0,1,0]，与 CGS 路径报告值恒同——巧合可回读，不作承诺；非 CGS 相机值会丢 |
| material 的 `map=WxH` | 回读 ✗ | `Texture` 不存源路径，只剩尺寸（`texture.rs:14`） |
| torus `arc≠TAU`、cyclide `shift≠0` | 回读 ✗ | 构造器只读 `R/r` 与 `a/b/d`（`scene_lang.rs:3073–3107`），多余键**静默丢弃**——报告值等于 CGS 默认（TAU / `[0,0,0]`）时丢弃无损，偏离默认即丢值 |
| `nan`/`inf` | 回读 ✗ | 非法 CGS 数字；仅作诊断输出 |
| tag 实例的 `rel` 帧 | 不输出 | `drill` 内部量，非结果验证面 |
| 数值 6dp | 有损 | 回读值与原值差 ≤5e-7（§5.1 精度契约） |
| 旋转轴符号 | 规范化 | `angle ∈ [0, π]` 强制唯一（源写 `angle=-0.4` 会输出 `axis` 取反 + `angle=0.4`，语义等价） |

**载荷可执行性**：对象/tag 载荷行去掉框架前缀后，在**不含 `frame(...)`/`trimesh(...)`
token** 时逐字节是合法 CGS 语句（分号已在行尾）；含上述 token 时形状仍统一、待回读器扩展。

## 8. API 设计

新模块 `crates/cga-gpu/src/scene_report.rs`；注册表出口放 `scene_lang.rs`（`Inst` 私有，
转换就地做）：

```rust
// scene_lang.rs（新增；cgs_load/cgs_load_result 签名与行为不变，改为委托）
#[derive(Clone, Debug)]
pub struct TagInstance { pub geo: Geometry, pub world: [f64; 16], pub rel: [f64; 16] }
pub type TagRegistry = std::collections::BTreeMap<String, Vec<TagInstance>>; // 名字典序

pub struct CgsRun {
    pub scene: Scene,
    pub camera: PerspectiveCamera,
    pub tags: TagRegistry,
}
pub fn cgs_run_result(text: &str, asset_root: &str) -> Result<CgsRun, String>; // 真实实现
pub fn cgs_run(text: &str, asset_root: &str) -> CgsRun;                       // panic 镜像 cgs_load

// scene_report.rs
pub fn scene_report(scene: &Scene, camera: &PerspectiveCamera, tags: &TagRegistry) -> String;
pub fn cgs_report(text: &str, asset_root: &str) -> Result<String, String>;     // run + report

// cga-examples/src/bin/report_cgs.rs（新 CLI，stdout）
// report_cgs <file.cgs>    —— 错误 eprintln + exit(1)（新 bin 自选，不进既有契约）
```

- `scene_report` 对手工构造场景同样可用（无 tag 时传 `&TagRegistry::new()`）。
- 实现依赖全部是既有 pub 件：`geom_to_camera`、`bounds_of`、`matrix_to_quaternion`、
  `motor().to_matrix()`、`csg_op_name`（`scene_lang` 内改 `pub(crate)` 或报告模块内
  三臂重实现，实施时取更小改动者）。

## 9. 错误契约：零新增

- 报告生成是全函数：无输入校验、无失败分支、无新错误格式串。README 错误契约表
  **本特性 +0 行**（明写，防误加）。
- 加载错误（语法/arity/边界）完全沿 `cgs_run_result` → 与 `cgs_load_result` 同一
  代码路径、同一文本、同一行号——166 个既有测试（含全部错误契约金样）逐位不动。
- 唯一"新增输出"是 `report_cgs` bin 的 `eprintln`+`exit(1)`，属 CLI 层，不属于语言契约。

## 10. 验收测试清单（8 项，166 → 174）

全部放 `scene_report.rs` 的 `#[cfg(test)]`（cga-gpu 120 → 128，cga-core 46 不动）。
金样沿用既有风格：精确字符串/整行 `assert_eq!`，数值全部来自 §5.1 规则手推并运行复核。

| # | 测试 | 断言意图 |
|---|---|---|
| T1 | `test_report_minimal_golden` | 无 background/camera 语句的单 `translate+material+sphere`：**整份报告**逐字节 golden（含默认背景 `0x87CEEB`、合成默认相机行、object/bounds 对、summary 含 bbox） |
| T2 | `test_report_rotation_canonical` | `angle=pi/2` → `angle=1.570796` 轴不反；`angle=-pi/2` → 轴反号+正值；`angle=0` 省 rotate 子句；`translate([1e-9,0,0])` 省前缀；数值 `-0.0` 输出 `0`（选帧省略后仍可见的槽位断言，如 `plane d=-0.0`） |
| T3 | `test_report_csg_nested_frames` | `difference(){ translate box; scale sphere; }`：CSG 对象无前缀、子节点 `frame(t=…)` / `frame(lin=[[2,0,0],…])`、恒等子裸几何、嵌套 difference 递归同形 |
| T4 | `test_report_unbounded_bounds` | 无界 `cylinder(r=1)` → `h=-1`、`bounds i none`、`no_bounds=1`；plane 同样计入 `no_bounds`；summary bbox 只由有界对象给出；含 scale 对象走 `Affine → a_fwd` bbox 通路 |
| T5 | `test_report_tags` | 两名多实例：名**字典序**、`count=`、实例发射序、CSG 实例 identity 帧、无 material；同输入两次调用逐字节相等（消 HashMap 序） |
| T6 | `test_report_trimesh` | `extrude(profile, h)` → `trimesh(faces=N, lo=[…], hi=[…])` 整行精确（faces 数以运行为准人工复核） |
| T7 | `test_report_material_variants` | `unlit` 材质行（`unlit=true` 且字段为 `basic_material` 存储值）；手工构造 `Texture` 的 `map=WxH` 分支；emissive `0x…` |
| T8 | `test_report_lights_camera` | 三种灯各一行（kw 序精确）+ 自定义 camera 七键行（`16/9 → 1.777778` 规则实证） |

既有 166 项零改动即为回归网（`make test` 全绿是硬验收）。

## 11. 实施步骤与风险

分步（每步全绿再前进，最后独立提交）：

1. `scene_lang.rs`：`TagInstance`/`TagRegistry`/`CgsRun` + `cgs_run_result`/`cgs_run`，
   `cgs_load*` 改委托（行为零变化）→ 166 全绿。
2. `scene_report.rs`：`fmt_num`/`fmt_vec3`/`fmt_color`/`fmt_axis_angle`/`fmt_frame`/
   `fmt_geom`/`fmt_material` + `scene_report`/`cgs_report`。
3. `lib.rs` 挂 `pub mod scene_report;` + re-export。
4. T1–T8（金样先跑出实际输出、人工核对语义后固化）。
5. `report_cgs` bin。
6. `cargo fmt --all` + `make test` → **174 全绿**。
7. README 同步（同提交）：
   - `make test` 行与测试段：166 → **174**（新增"场景报告格式化/确定性/帧规范化"类别词）；
   - 特性段（CGS v2 段附近）加「Scene Report 执行结果文本报告」小节，链 `docs/scene-report.md`；
   - 源码树加 `src/scene_report.rs` 行；
   - CLI 列表加 `report_cgs <file.cgs>`（`render_cgs`/`bake_cgs` 行旁）；
   - 错误契约表：**+0 行**（并在表注写明 Scene Report 无诊断文本）。

风险与对策：

| 风险 | 对策 |
|---|---|
| `lib.rs` 有并行会话未提交改动 | 只在其中追加本特性两行；`git add -p` 仅暂存本特性 hunk（或先与并行会话协调），绝不整文件带入对方改动 |
| `geom_kernels.rs`（`geom_to_camera`）在并行修改 | 只用其 pub 签名；bbox 金样只用 sphere/box/cylinder/plane 等简单图元，避开自由曲面/退化路径 |
| bbox 断言受并行会话影响 | T4 数值用手推值（§5.7 语义锚定 `bounds_of` 文本），若并行改动波及会在本提交前暴露 |
| 旋转提取符号/约定歧义 | 统一经 `to_matrix → matrix_to_quaternion`（矩阵真值，无 rotor 符号约定问题），T2 专测负角/零角 |
| 测试基线漂移 | 以提交时 `make test` 实数为准更新 README 两处计数 |
