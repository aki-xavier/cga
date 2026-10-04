# CGS v2 语法提案：从"构造语义"到"构造 + 关联语义"

状态：提案定稿，按 P0→P3 分阶段实施。
适用：`crates/cga-gpu/src/scene_lang.rs`（lexer + 单遍 parser/evaluator，约 2300 行）。
后续演进：P4–P6（jQuery 式后缀方法链、`instances()` 集合选择、不等式约束）见 `docs/cgs-v3.md`。

## 1. 动机：CGS 相对欧氏建模的三个语义缺口

CGS（本仓库的 OpenSCAD 风格场景语言）在**构造语义**上比欧氏特征建模直观：
`difference(){ 毛坯; 钻孔; }` 读代码即读工艺卡，无隐藏求解器状态，文本可 diff、
可重生成、可单测——这正是 LLM-first CAD 需要的性质。但它比欧氏 CAD 少三样东西：

| # | 缺口 | 欧氏 CAD 的解法 | CGS 现状（实测） |
|---|------|----------------|----------------|
| G1 | **稳定引用**：无法命名对象、无法"选中"后续操作的目标 | 特征/拓扑 ID + 面引用 | 无几何值类型；`CgsValue` 只有 Num/Bool/Str/List/Vec3，几何只在语句执行瞬间落入 scene |
| G2 | **关联语义**：位置写死坐标，改一个尺寸要手动级联 | 尺寸约束 + 求解器 | `translate([2.0, 0, 0])` 字面量；变量是文本替换，无"相对某对象"的定位 |
| G3 | **派生特征跟随**：`hole through all` 自动贯穿 | 派生范围引用 | 钻孔深度是模型单位写死（`cylinder(h=0.35)`） |

另外实测发现一个既有的组合缺口（也是本提案的直接动因之一）：

```
difference() { box(...); difference() { sphere(...); box(...); } }
→ CGS line 1: difference needs >= 2 geometry children   // 嵌套 CSG 实际不工作
```

语句形式的内层 `difference` 直接 `scene.add_mesh`，不回流到外层的 collect，
外层只收集到 1 个孩子而报错（README 宣称的"任意嵌套"并不成立）。

## 2. 设计原则

1. **加法改造**：现有全部 `.cgs` 文件原样可跑，已有金样测试逐位不动。新语法全部
   是新语句/新函数/新参数，不改任何既有语句的语义。
2. **文本即真相**：一切新语义在**单遍前向求值**中完成——约束求解发生在
   `constrain` 语句执行的那一刻，解出的数值直接写回 scope，后续语句读到的已是
   烘焙好的常量。没有独立编译产物，`源码 + 确定性求值` 仍是唯一事实。
3. **语句 = 渲染，表达式 = 值**：语句位置产生 scene 副作用；表达式位置产生
   `CgsValue::Geom`（几何值）。两者共享同一套构造函数，形态对偶
   （`difference(){...}` 与 `difference(a, b);`）。
4. **帧语义最小惊讶**：几何值**默认不捕获语句上下文**（frameless），
   放置永远来自"渲染它的地方"或显式的 `at/rot`；具名实例（`tag`）才获得
   世界帧，供跨语句关联查询。详见 §4.2——这是本提案最容易出错的地方，
   规则集中在这里一次讲清。
5. **失败显式**：查询打在无界几何（plane）、类型不符、约束不收敛时返回带行号的
   `Err`，绝不静默猜测（LLM Verifier 依赖确定的错误文本）。

## 3. 现有执行模型（改造的基座）

```
cgs_lex → SceneLoader::run_tokens → statement() 单遍分发
  ├─ 修饰语句 translate/rotate/scale/mirror/material → body() 下钻（ctx/mat 累积）
  ├─ difference/union/intersection → csg_block(): 开 collect 收集子语句的
  │    (geo, m4)，折叠成 CsgGeometry 一次 add_mesh
  ├─ add_geometry(): collecting 时 push 到 collect，否则 decompose(ctx) 后 add_mesh
  ├─ 赋值 `name = expr;` → scope.insert（expr 是纯数值表达式）——**分发表
  │    最先**：关键字可被变量遮蔽（`echo = 5;`）；`background = 0x…;` 同时
  │    写入场景背景（与 `background(color=0x…);` 等价，属性变量语义）；表达式
  │    中的赋值/索引/语句调用按组给出统一错误文本（见 README 错误契约）
  ├─ module: 存 token 体，调用时用参数 scope 重放（宏语义，看不到调用者 scope）
  └─ for/if/echo: 控制流
```

关键既有资产：
- `collect: Vec<CollectedGeom { geo, m4 }>` —— 事实上的构造 IR，本提案的
  `tag`/实例注册/表达式 CSG 都挂在这条通路上。
- `cgs_sig_names/cgs_sig_defaults` —— 图元签名表，兼作"这个名字是图元还是数学函数"
  的判别表。
- 模块参数可携带任意 `CgsValue` ⇒ P0 落地后几何值可直接作为模块实参传递。

## 4. P0：几何值 + 具名实例 + 引用查询（解决 G1）

### 4.1 值域扩展

```rust
enum CgsValue {
    Num(f64), Bool(bool), Str(String), List(Vec<CgsValue>), Vec3(CgsVec3),
    Geom(GeomVal),                    // 新增
}
struct GeomVal { geo: Geometry, m4: [f64; 16] }   // 形状 + 自身局部帧
```

### 4.2 帧语义（三条规则，一锤定音）

1. **绑定是 frameless 的**：表达式位置的图元（`g = cylinder(r=1, h=2);`）构造出
   `GeomVal { m4: identity }`——不捕获语句的 translate/rotate 上下文。
   这消除了"绑定处 ctx 和渲染处 ctx 相同 → 平方"的双重施加陷阱：
   `translate(T){ g = box(); show(g); }` 渲染在 T，不是 T∘T。
2. **`show(g)` 在 `ctx ∘ g.m4` 处渲染**（g 自带的 `at/rot` 帧 × 当前语句上下文）。
   绑定在别处、渲染在这里（实例化），或绑定渲染同处（原位）——两种直觉都对。
3. **`tag` 注册的世界帧与相对帧**：`tag("name") stmt` 把该语句产生的每个实例
   记录进注册表，每个实例存两个帧：
   - `world` = 注册时刻的绝对帧 → 供**关联查询**（`center("name")` 跨语句、
     跨装配引用"真实位置"）；
   - `rel`  = `world ∘ tag语句ctx⁻¹` → 供 P1 的 `drill` 在**另一个 ctx** 里
     复现"该实例在自己语句里的摆法"，避免双重施加。

查询函数接受两种引用：`Str` = 具名实例（用 `world` 帧），`Geom` = 几何值
（用自身 `m4` 帧）。一个函数、按类型分派。

### 4.3 新语法

```ebnf
(* 赋值扩展：右端可为几何表达式 —— 现有赋值分发零改动 *)
assignment   = ident "=" geom_expr ";" ;

geom_expr    = prim_call                (* 已知图元签名 → Geom 值，见签名表 *)
             | csg_call                 (* difference(a, b, c) → Geom 值 *)
             | "at"     "(" geom_expr "," expr ")"          (* 平移帧 *)
             | "rot"    "(" geom_expr "," expr "," expr ")" (* 旋转帧 axis,angle *)
             | "scaled" "(" geom_expr "," expr ")"          (* 缩放帧 *)
             | ident                    (* scope 中的 Geom 变量 *)
             | "(" geom_expr ")" ;

csg_call     = ("difference"|"intersection"|"union") "(" geom_expr ("," geom_expr)+ ")" ;

(* 新语句 *)
statement   += "show"  "(" geom_expr ")" ";"                (* 渲染几何值 *)
             | "tag"   "(" str ")" statement                 (* 具名实例前缀修饰 *)
             | csg_call ";"                                  (* 表达式 CSG 直接渲染 *)
             | ("difference"|"intersection"|"union") "(" args ")"   (* 赋值语句形态 *)
                                                       ";"  (* = csg_call 渲染 *)
```

**表达式调用支持命名参数**：`cylinder(r=1, h=2)` 在表达式位置目前会报
`undefined variable r`——P0 统一表达式调用的实参解析为语句路径的
`call_args`（pos + kw），数学函数只接受位置参数，图元接受 kw。

**同名形态对偶**（互不冲突：语句分发看语句首，表达式分发看调用首）：

```cgs
difference() { box(s=[2,2,2]); sphere(r=3); }   // 语句块（既有）
difference(box(s=[2,2,2]), sphere(r=3));        // 表达式（新，可赋值可嵌套）
difference(a, b);                               // 表达式当语句 = 渲染（新）
```

### 4.4 修嵌套 CSG（既有 bug，并入 P0）

`csg_block` 收尾时 `if was_collecting { collect.push(结果, m4=identity) }`
否则 `scene.add_mesh`。内层结果的几何已吸收子帧（世界坐标），故回流时 m4
恒为 identity——与外层把叶子按各自 m4 施加的规则一致（外层 ctx 已包含在
子节点 m4 里，推导见实施注记）。语句嵌套与表达式嵌套此后走同一语义。

### 4.5 引用查询函数（表达式位置，Str/Geom 分派）

| 函数 | 返回 | 说明 |
|------|------|------|
| `center(x)` | Vec3 | AABB 中心（Str: 世界帧；Geom: 自身帧；同名多实例取并集） |
| `lo(x)` / `hi(x)` | Vec3 | AABB 两端 |
| `size(x)` | Vec3 | AABB 尺寸 |
| `xdir(x)` / `ydir(x)` / `zdir(x)` | Vec3 | 实例/值局部轴的世界方向（m4 旋转列，URDF 关节帧直接可用） |
| `dist(a, b)` | Num | 两点/两引用 AABB 中心距（可与 Vec3 混用） |

`center` 等对无界几何（plane）报错：`CGS line N: <name> has no finite bounds`。

### 4.6 P0 落点与测试

| 位置 | 改动 |
|------|------|
| `scene_lang.rs` `CgsValue` | 加 `Geom(GeomVal)` 变体 + PartialEq/Display |
| `primary()` | 调用首分派：图元签名表 → `build_geometry`；查询函数 → `self.eval_query`；kw 实参解析 |
| `statement()` | `show` / `tag` / 表达式 CSG 分支 |
| `add_geometry` / `csg_block` | 实例注册钩子（pending_tag 栈）；嵌套回流 |
| `SceneLoader` 新字段 | `named: HashMap<String, Vec<Inst { geo, world, rel }>>`, `pending_tag: Vec<String>` |

测试（风格复用既有金样断言：解析文本 → 断言 objects 数/几何参数/position）：

1. `g = cylinder(r=1, h=2); show(g);` → 1 mesh，参数正确（绑定不渲染，show 才渲染）
2. `translate([3,0,0]) { g = box(...); show(g); }` → position=3（frameless 无平方）
3. `difference(a, b);` 表达式渲染 1 mesh；`x = difference(a, b); show(x);` 同
4. 语句嵌套 `difference(){ A; difference(){B;C;} }` → 1 mesh（bug 修复）
5. `tag("p") translate([5,0,0]) sphere(r=1); translate(center("p")) sphere(r=1);`
   → 第二球 position=5（关联查询闭环）
6. `lo/hi/size/xdir/dist` 数值经 `translate(center(...))` 落位断言
7. 错误分支：`center("missing")`、`center(plane(...))`、类型错

## 5. P1：派生跟随 + 关系式定位（解决 G3 与 G2 的免求解档）

### 5.1 `drill` —— 贯穿切割器（G3 的核心）

```ebnf
statement  += "drill" "(" drill_kw ("," drill_kw)* ")" ";" ;
drill_kw    = "r" "=" expr            (* 刀具半径 *)
            | "through" "=" (str | geom_expr)   (* 目标：具名实例 或 几何值 *)
            | "axis" "=" expr         (* 0/1/2 = X/Y/Z，必填，无隐含默认 *)
            | "from" "=" expr | "to" "=" expr   (* 可选：轴向数字限位（F 帧内坐标） *)
```

语义：在 `through` 引用的**相对帧**（§4.2 规则 3 的 `rel` / Geom 的 `m4`）内取
目标包围盒（多实例取世界盒并集后变换回切割器帧），沿 `axis` 从 `lo` 贯到 `hi`
（`from/to` 可替换两端），在 `ctx ∘ rel` 帧内构造圆柱刀具——目标变厚/变薄，
孔自动跟随；切割器出现在 `difference` 子块中即为贯穿孔，出现在并集中即为凸台轴。

```cgs
tag("plate") box(s=[7, 0.3, 7]);
difference() {
  show(plate_shape);
  drill(r=0.16, through="plate", axis=1);      // 底板改厚 → 孔自动贯穿
}
```

实现要点：圆柱局部轴为 +Z（与既有 `cylinder` 一致），`axis≠2` 时在 rel 帧内
补一个旋转换向；多实例 tag 取并集盒（变换到第一个实例的 rel 帧内求并）。

### 5.2 关系式定位小工具（表达式，纯函数）

| 函数 | 返回 | 说明 |
|------|------|------|
| `polar(r, a)` | Vec3 | `[r·cos a, r·sin a, 0]`——螺栓圈/均布阵列免手写三角 |
| `comp(v, i)` | Num | 向量分量访问（词法器无 `.`，`v.y` 不可行，用 `comp(v,1)`） |

有 `center/lo/hi/size + 元素级向量算术 + polar/comp`，"相对参照物定位"全部
可以在表达式里级联，无需任何求解器——这是 G2 的**免求解档**，覆盖八成场景：

```cgs
// 齿轮孔圈：均布在法兰上（关联：全部引用 "flange"，改法兰自动跟随）
for (i = [0:7]) {
  a = i * pi / 4;
  translate(center("flange") + polar(2.0, a)) drill(r=0.16, through="flange", axis=1);
}
```

## 6. P2：`constrain … solve` —— 编译期约束求解（解决 G2 的求解档）

只在残余的**欠定/联立**场景使用（例：两孔中心距给定求偏移、皮带轮中心距由
带长反解）。原则（§2.2）：求解发生在语句执行时刻，解烘焙进 scope，运行语义不变。

### 6.1 语法

```ebnf
statement  += "var" ident "=" expr ";"                    (* 声明可解变量 *)
             | "constrain" "(" ident_list ")"
               "{" constraint* "}" "solve" ";" ;
constraint  = expr "==" expr ";"                          (* 仅等式，v1 *)
```

- `var`：标记"求解未知量"。普通赋值 `x = ...` 不受影响。`var` 可重新声明
  （for 循环体每次迭代重置初值再解）。
- `constrain(v1,…,vk)`：显式列出未知量——必须是 scope 中的 `Num`。显式优于
  隐式推断：意图对 LLM/人可读，不会误劫持 `pi` 这类常量。
- 约束体逐条按 token 捕获（复用 `module_def` 的 token 捕获先例），每条必须是
  顶层 `==`；两侧求值为 Num 或 Vec3/List（Vec3/List 按分量展平成多条残差，
  两侧展平长度必须一致）。
- **方程必须直接引用未知量**：重估只覆写 scope 中的未知量标识符——几何值在
  绑定时烘焙、具名实例在渲染时烘焙，`center(b)` 这类查询在求解期间是常量。
  `center("a")` 等常量项可正常参与（作已知偏置）；若某未知量不出现在任何
  方程中（雅可比列全零），求解不收敛并显式报错。

### 6.2 求解器

- 残差 `r(x)`：以试探值覆写 scope 中的未知量，重估两侧 token 片段
  （`eval_token_expr`：换入片段 token、解析后恢复），`r = lhs - rhs` 展平。
- 算法：Levenberg–damped Gauss–Newton。数值雅可比（`h = 1e-6·max(1,|xⱼ|)`），
  `Δ = −(JᵀJ + λI)⁻¹ Jᵀ r`，n×n 高斯消元（n = 未知量个数，很小），
  步后 `|r|∞` 不降则 λ×10 重试。
  λ 随失败增大、随成功减小 → 秩亏（欠定/冗余）自动退化为最小模步。
- 收敛：`|r|∞ < 1e-10`，最多 50 次迭代。
- 失败：`CGS line N: constrain did not converge (|r|=…, iter=…, var=…)` ——
  显式错误，供 Verifier 归因；不收敛 = 编译错误而非渲染崩溃。
- 成功：`scope.insert(未知量, 解)` ——后续语句读到的即烘焙常量。
- v1 只支持 `==`；`<=`/`>=`（不等式约束）的求解已由 `docs/cgs-v3.md` §5 兑现
  （hinge 残差进同一 GN，`<`/`>` 视同闭包，`!=` 拒绝），本文的占位报错文本
  不再出现。

### 6.3 测试

1. 单方程：`var d = 1.0; constrain(d){ dist(center("a"), [d + 3, 0, 0]) == 4; } solve;`
   → `d` 解出；solve 前渲染的实例保持初值、solve 后语句读到烘焙值
   （对照断言：同一 `d` 在 solve 前后落位不同）
2. Vec3 联立：三轴同时约束 → 断言三个分量
3. 初值无关性：两个不同初值收敛到同一解
4. 无解：矛盾方程 → 精确错误文本断言
5. 欠定：两个未知量一个方程 → 收敛到某解（不报错），初值影响解
6. for 循环内重解：每迭代初值重置

## 7. P3：面引用（装配/URDF 挂点）

```ebnf
(* 表达式查询，Str/Geom 分派，同 §4.5 *)
face (x, key) -> Vec3    (* 面心的世界坐标 *)
fnrm(x, key) -> Vec3    (* 面法向的世界方向 *)
```

`key` ∈ `"+x","-x","+y","-y","+z","-z"`。解析规则（v1，文档明示精度）：

- **box**：AABB 面心 —— 精确；
- **cylinder/cone**：端面（±轴向）精确取自几何参数；侧面取轴向中点处的
  切向点 —— 圆柱精确在面上，圆锥取中点半径保证在锥面上（不用 AABB 面心，
  那会浮空）；圆锥侧面法向取生成线的垂线（外向、朝锥尖倾斜，精确）；
- **sphere/ellipsoid**：`key` 方向与椭球面交点（精确）；
- **CSG 结果 / 未知类型**：退化为局部 AABB 面心（近似），`echo` 级别的
  文档警告；精确的"边界归属分类"（哪一面属于哪个孩子）留到 v2——
  归类机制可复用渲染器 `csg_uv` 的 child-boundary 判定。
- **plane（无界）** → `no finite bounds` 错误；具名引用取**首个实例**
  （多实例 tag 的"面"有歧义——bbox 类查询仍取并集）。
- 法向按逆置 3×3 变换后归一（非均匀缩放下仍正确）；点按世界矩阵变换。

`drill` 扩展 P3 kw：`from`/`to` 接受 `"名:key"` 面引用——
`drill(r=…, from="base:+z", to="top:-z");` 端面到端面贯穿（可跨两个实例）。
语义（实现口径）：

- 面引用解析为**世界**坐标点，再变换回切割器帧 `F = ctx ∘ rel`（`through`
  给出）或 `F = ctx`（`through` 省略——两端齐备时 `through` 可省，包围盒
  不再需要）；
- `axis` 可省略：由面键推断（`±z` → 2）；显式给出时必须与面键一致，否则
  `does not match axis`；两端面键轴不一致 → `different axes`；
- 端点仍可为数字（F 帧内坐标），与面引用可混用（缺的一端回落包围盒，
  此时 `through` 必填）。

**引申（供新 GUI 项目）**：`xdir/ydir/zdir + face/fnrm` 已足够给每个具名实例
产出 URDF 关节帧（位置 + 朝向），无需等 B-rep 拓扑名——这正是"标签绑在构造
节点上、不绑拓扑 ID"的结构性优势：布尔重算不会让 `face("flange","+z")` 漂移，
而欧氏 CAD 的拓扑命名问题（面 ID 跨编辑漂移）在此不存在。

## 8. 执行模型总览（改造前后）

```
改造前:  lex → 单遍 statement 分发 → collect/折叠 → scene
改造后:  lex → 单遍 statement 分发
           ├─ 表达式求值: 数值 + Geom 值（递归组合，帧由 at/rot/show 规则决定）
           ├─ tag: 实例注册 (world, rel)         [P0]
           ├─ drill: rel 帧内取盒 → 刀具几何      [P1]
           ├─ constrain: token 捕获 → 现场 GN 求解 → 写回 scope [P2]
           └─ face/fnrm: 注册表 + 逐图元面解析     [P3]
         → collect/折叠（含嵌套回流） → scene
```

单遍前向求值保持不变；新语义全部是"执行到该语句时做一次额外工作"，
没有第二遍编译，没有 IR 重写，`cgs_load_result` 签名不变。

## 9. 与欧氏 CAD 的关系（结论）

- P0/P1 = **免求解关联**：引用 + 表达式级联，覆盖"同心/均布/偏置/贯穿"
  这类日常意图，保持 CGS 的确定性与可 diff；
- P2 = **联立求解兜底**，严格限定在语句执行时刻，失败即编译错误；
- P3 = **稳定面引用**，为装配/出图/URDF 提供挂点，且因标签绑定在构造节点上，
  结构性避免欧氏 CAD 的拓扑命名漂移。
- 欧氏 B-rep 的精确曲面（NURBS 导出、公差配合）仍由 truck 内核承担
  （见 `docs/competitors.md` 的定位结论）：CGS 是 authoring/关联层，
  不是替代精确表示。

## 10. 实施顺序与验收

| 阶段 | 内容 | 验收 |
|------|------|------|
| P0 | Geom 值 + show/tag + 表达式 CSG + 嵌套修复 + 查询函数 | 新增 9 测试；全部既有金样不动；`make test` 绿 |
| P1 | drill + polar/comp | 新增 4 测试（含"改厚度孔跟随"金样） |
| P2 | var + constrain/solve + GN 求解器 | 新增 7 测试（收敛/不收敛/欠定/循环重解/错误组） |
| 边界 | 语句/表达式边界统一错误 + 赋值分发前置 + background 属性化 + 索引拒绝 | 新增 1 测试（错误契约分组断言） |
| P3 | face/fnrm + drill 面引用 | 新增 4 测试（逐图元面心/法向 + 面引用贯穿 + 错误组） |

每阶段独立提交点：`cargo test --workspace` + `cargo fmt --all` 全绿后进入下一阶段。
