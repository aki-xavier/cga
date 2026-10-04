# CGS v3 提案：吸收 jQuery 链式与 SQL 集合语义（P4→P6）

状态：P4–P6 全部实施（2026-10-04，194 测试全绿：46 + 148）。
适用：`crates/cga-gpu/src/scene_lang.rs`（lexer + 单遍 parser/evaluator）。
前序：`docs/cgs-v2.md`（P0–P3 已实施，166 测试全绿）。本提案只加糖与缺件，不改 v2 语义。
诊断行号为 2026-10-04 快照，随代码漂移，以符号名为准。

## 1. 动机：三语法选型的结论

对三种候选表面语法按 LLM CAD 的三标准（LLM 语料相容性 / 可验证性 / 完备性）比较：

| 维度 | CGS 语句式（现状） | jQuery 链式 `.f().g()` | SQL 集合式 `SELECT…WHERE` |
|---|---|---|---|
| LLM 语料相容性 | 高（OpenSCAD 系构造 + C 系表达式） | 很高（JS 语料高频） | 很高（SQL 语料高频） |
| 可验证性 | 高：确定错误文本、语句时执行、单遍前向 | 中：本身无错误契约，须去糖后才可断言 | 低：选择集隐含"先求值后过滤"的优化器语义，UPDATE 还需回溯 |
| 完备性（CAD 表达力） | 构造+引用+约束 ✓；缺集合选择、深嵌套可读性差 | 改善嵌套可读性；不解决集合 | 集合选择强；构造、帧语义、约束全缺 |

**结论：CGS ≳ jQuery 链式 > SQL 式。** 构造继续用语句、约束继续用声明式方程块
（v2 已定），只吸收两个长处、拒绝一个短处：

1. **吸收 jQuery 的"接收者显式、从左到右"** → P4 后缀方法链（§3）。纯语法糖。
2. **吸收 SQL 的"集合是一等值、选择是读操作"** → P5 `instances()`（§4）。纯数据 +
   既有控制流，不引入 SELECT 关键字。
3. **拒绝 SQL 的回溯式 `UPDATE … WHERE`**（§7）：违背文本即真相与单遍执行，
   且撞上未解决的 CSG 边界归属。

贯穿红线（v2 §2 原则的延伸）：

- **糖不是新语义**：任何新表面形式都必须先写出去糖式，去糖后走既有函数表与全部
  规范错误——错误文本因此几乎零新增（§6 盘点）。
- **语义必须正向**：单遍、语句时执行；选择永远发生在发射前，不存在"发射后回头
  改写场景"。

## 2. 现状落地条件（词法/分发实测）

P4–P6 的可行性依赖以下实测事实（全部已核实）：

| 事实 | 位置 | 对提案的意义 |
|---|---|---|
| `.` 落入词法器末尾 `illegal character` 分支 | `cgs_lex` 253 | `Dot` 是空闲词法位，新增零冲突；现有语料/测试无游离 `.` |
| `punct_kind` 未列 `.`；`TokenKind` 无 Dot 变体 | 91–104、`TokenKind` Display 69 | 需加 `TokenKind::Dot` + Display `"dot"`（喂给既有 `expect`/`bad expression start` 文本） |
| 数字扫描器在数字**内部**消化 `1.5`（含一个点） | `cgs_lex` 224–251 | `1.5.abs()` 词法正常；`1.abs()` 的点被吞 → 见 §3.2 陷阱 |
| 单字符 `\|` 同为非法（`\|\|` 才合并成 Op） | `cgs_lex` 152 附近 | `\|` 空闲但与方法链重复，**不引入**（不双轨） |
| 表达式分发是单入口三路：`is_statement_only` → `geom_expr_call` → `cgs_call_fn` | `primary` 1223、1255–1282 | 链式去糖汇入此口即自动继承全部错误契约 |
| 查询函数表 `QUERY_FNS` 同时驱动表达式分发与语句位契约 | 710、742、1457 | P5 新函数只进这一个表即得全套行为 |
| 语句分发顺序：赋值(2276) → 索引拒绝(2288) → 关键字(2294) → CSG(2330) → 通用调用(2358) | `statement` 2255 | P4 语句位判链插在赋值/索引判据后、关键字前（§3.4） |
| `for` 已遍历 `CgsValue::List`；`len` 已支持列表 | `for_loop` 2524 | P5 返回 List 即零改动接入 |
| 实例注册表 `named: HashMap<String, Vec<Inst{geo, world, rel}>>`，`Vec` push 序 = 发射序 | 1060、1066、1698 | `instances()` 的确定性顺序免费获得 |
| 约束解析已识别 `< > <= >= !=` 但统一报占位错误 | 2060、2069 | P6 只需改分类与残差，语法面零新增 |
| GN 求解器对残差形态不可知（前向差分 + Levenberg 阻尼） | `constrain_solve` 2160–2247 | hinge 残差直接进同一求解器，求解器本体不动 |

## 3. P4：后缀方法链（jQuery 之形，零新语义）

### 3.1 规则

```
e.f(a, b)   ≡   f(e, a, b)        // 接收者 = 第一个位置实参
```

链式与函数式**逐位等价**：同一 scope、同一求值序（接收者先于实参）、同一错误文本。
链是后缀运算符，优先级最高：

```
box(s=[2, 2, 2]).at([1, 0, 0]).rot([0, 1, 0], 45)   ≡  rot(at(box(s=[2, 2, 2]), [1, 0, 0]), [0, 1, 0], 45)
"plate".face("+z")                                  ≡  face("plate", "+z")
center("a").dist(center("b"))                       ≡  dist(center("a"), center("b"))
-x.abs()                                            ≡  -(abs(x))       // 一元作用于整条链
var d = center("a").dist(center("b"));             // 链可用于任意表达式位（var/约束/实参）
```

### 3.2 词法：`TokenKind::Dot`，为何零冲突

- `punct_kind` 增加 `b'.' => Dot`，Display `"dot"`。token 头部为 `.` 时命中标记；
  token 头部为数字时先走数字扫描器，`1.5` 的点在扫描器内部消化（224–251），
  **两种情况互不经过对方分支**。语料与测试无游离 `.`（`illegal character` 文本
  无任何测试断言）。
- **数字字面量头的陷阱**（已知边界，写入文档）：

| 写法 | 结果 |
|---|---|
| `1.5.abs()` / `1.0.abs()` | ✓ 正常（扫描器只吞一个点，第二个点成 Dot） |
| `1.abs()` | ✗ 扫描器吞 `1.` → 表达式位 `x = 1.abs();` 并置成 `1 abs()` → `expected semi, got ident`；改写 `(1).abs()` |
| `.5` | `bad expression start dot`（既有格式串） |
| `g.5` | `expected ident, got number`（既有 `expect` 格式串） |

- `g.foo`（无括号）→ `expected lparen, got semi`：**不做属性访问**，`v.y` 继续写
  `comp(v, 1)`——保持唯一去糖规则（`ident(args)`），不引入第二套成员语义。
- 单 `|` 词法空闲但与方法链表达力重复，**不引入**（不双轨）。

### 3.3 分发：汇入既有单入口，错误全继承

`primary()` 求值完原子后做后缀循环，伪码：

```rust
let mut v = <atom 值>;                     // 数字/字符串/变量/括号/列表/函数调用
while self.peek().kind == TokenKind::Dot {
    self.take();                            // `.`
    let m = self.take();                    // 方法名，非 Ident → expected ident, got …
    self.expect(TokenKind::Lparen)?;        // 无括号 → expected lparen, got …（属性访问到此为止）
    let (mut pos, kw) = self.paren_args(scope)?;
    pos.insert(0, v);                       // 接收者前插为第一位置实参
    // 与 `m(…)` 完全相同的三路分发：is_statement_only → 边界错误；
    // GEOM_EXPR_NAMES/QUERY_FNS/at/rot/scaled/CSG → geom_expr_call；否则 cgs_call_fn
    v = <分发>(m.text, pos, kw, m.line)?;
}
Ok(v)
```

错误契约继承表（每一行都是既有规范文本，P4 不新增任何错误格式串）：

| 链式写法 | 去糖为 | 继承的规范错误文本 |
|---|---|---|
| `g.show()` / `g.translate(…)` | `show(g)`（表达式位） | `CGS line N: show is a statement and cannot be used in an expression`（1257） |
| `g.frobnicate()` | `frobnicate(g)` | `CGS line N: unknown function frobnicate`（1028） |
| `g.at()` | `at(g)` | `CGS line N: at needs (geometry, offset)`（arity，既有） |
| `g.scaled(k=2)` | — | `CGS line N: scaled takes positional arguments only`（1378，既有） |
| `g.center(k=1)` | — | `CGS line N: center takes no named arguments`（1586，既有） |
| `g.center`（属性式） | — | `CGS line N: expected lparen, got semi`（`expect` 1158） |
| `g.5` | — | `CGS line N: expected ident, got number` |
| `(5).at(…)`（接收者非 Geom） | `at(5, …)` | `CGS line N: at needs a geometry value, got 5`（`geom_val` 688；裸 `5.at` 的点被扫描器吞，先撞 §3.2 数字陷阱） |
| `mymod().at(…)`（头是模块名） | 表达式路径 | `CGS line N: unknown function mymod`（表达式位无模块可见性，既有行为） |
| `mymod();`（未定义名，无链对照） | 通用语句路径 | `CGS line N: unknown primitive mymod`（既有，不变；若 `mymod` 已 `module` 定义则正常执行） |

### 3.4 语句位：判链规则与渲染

**判链规则**（插在赋值判据 2276、索引拒绝 2288 之后、关键字分发 2294 之前）：

- `ident` 后紧跟 `Dot`，或 `ident` 后是 `(` 且括号平衡配对后的下一 token 是 `Dot`
  → 整条按**表达式语句**求值：`expr()` → `geom_val`（须为 Geom）→ `expect(Semi)`
  → `add_geometry(geo, ctx ∘ m4, mat)`（与 CSG 表达式形式 2350–2355 同一条渲染路）。

零冲突论证：

- 关键字语句 `translate(…)/for(…)/echo(…)` 的配对 `)` 后永远不是 `.`（其后是 `{`、
  语句或 `;`）→ 不触发，落回既有分发；`difference(a, b);` 配对后是 `;` → 落回
  CSG 分支 2330，行为逐位不变。
- 赋值判据在前：`g = …` 不受影响；`background = 0x…;` 属性赋值不变。
- 语句位链头**必须是 ident**（变量或函数名）：`("x").face("+z");` 仍报
  `expected statement name, got lparen`（既有）。括号头写 `g = ("x").face("+z");`。

**语句位的错误**（全部既有格式串）：

- 求值结果非 Geom → 复用 `geom_val` 格式，`what` 槽取 `expression statement`：
  `CGS line N: expression statement needs a geometry value, got 3.5`。
- 链头是 statement-only 函数 → **设计决定**：链是表达式糖，去糖位置即表达式位，
  故 `g.show();` 报 `show is a statement and cannot be used in an expression`
  （要渲染写 `show(g);`）。同一条规则在两个位置给出同一文本，可断言性最好。

### 3.5 明确非目标

- **无属性访问**：`v.y` → `comp(v, 1)`（词法器无点的历史约束在语义层继续成立）。
- **无 `.end()` 回退栈**：Geom 值不可变、分支即重新绑定，jQuery 的栈回退在 CGS 无
  对应物，不需要。
- **无 `|` 管道**：与方法链重复，不双轨（§3.2）。

### 3.6 验收（P4 新增 6 测试）

1. 链式/函数式**等价金样**：同一场景两种写法，`scene.objects` 逐位相同
   （实施：以 `scene_report` 确定性文本逐位断言，连帧、参数、包围盒一起覆盖）。
2. 错误继承分组断言：`g.show()`/`g.frobnicate()`/`g.at()`/`g.5`/`g.center` 五条
   文本各命中一行（风格沿 `test_cgs_constrain_errors` 的 `v2_err` 断言）。
   实施注记：`cgs_call_fn` 的 1 元路径先过 `cgs_num` 类型分派——`g.frobnicate()`
   实得 `frobnicate needs a number, got <geom>`，测试以 `assert_eq` 与函数式
   拼法 `frobnicate(g)` **逐字对照**（继承不变式即验收本体）；
   `unknown function frobnicate` 由 Number 接收者 `(1).frobnicate()` 命中。
3. 语句位链渲染：`box(s=[1,1,1]).at([1,0,0]);` 落位正确；`s = sphere(r=1); s.size();`
   报 `expression statement needs a geometry value, got …`（链先求值，查询 `size(s)`
   返 Vec3 再撞 `geom_val`）。
4. Str 头链 `"plate".face("+z")` 与两参形式等价。
5. 变量头链：`g = box(…); g.at(…); g.rot(…);` 渲染等价。
6. 词法 case：`1.5.abs()` 正常、`1.abs()` 确定报错、`.5` 报
   `bad expression start dot`。

既有 166 测试逐位不动；`cargo fmt --all` + `make test` 全绿。
**实施结果（2026-10-04）**：6 条新测试落地为 `test_cgs_p4_chain_{equivalence,
errors_inherited,statement_render,var_head,str_head}` + `test_cgs_p4_lex_dot`，
`make test` 183 全绿（cga-core 46 + cga-gpu 137）。

## 4. P5：集合选择 `instances()`（SQL 之义，无 SELECT 语法）

### 4.1 语义

```
instances("hole") -> List[Geom]      // 该 tag 名下全部实例，元素帧 = Inst.world
```

- **元素帧 = 世界帧**：与 Str 形式的查询（`center("hole")` 并集包围盒用
  `i.world`，1543）同一坐标系；顶层 `show(h)` 复现原位，套新语句帧时按 v2 §4.2
  规则合成 `ctx ∘ world`。
- **顺序确定**：注册表按名保存 `Vec<Inst>`，push 序 = 发射序（1698–1705），
  `for` 遍历即按发射顺序落位。
- **挂点 = `QUERY_FNS` 一个表**（710）：语句位 `instances("x");` 自动命中
  `is_expr_only_fn`（742）→ `instances is an expression function and cannot be
  used as a statement`；参数个数/命名参数自动走 `eval_query` 既有检查
  （`needs 1 argument(s)` / `takes no named arguments`）。
- **错误文本**：未知名或空列表 → 复用 `CGS line N: unknown reference "hole"`
  （1536 同款）；非 Str 实参 → **P5 新增一条**：
  `CGS line N: instances needs a reference name, got {v}`（与 `query_target`
  1569 家族同构）。

示例：

```cgs
tag("hole") { for (i = [0:2]) translate([i * 1.2, 0, 0]) cylinder(r=0.3, h=5); }

hs = instances("hole");        // SELECT * FROM holes  → 集合是一等值
n  = len(hs);                  // COUNT(*)             → len
for (h = hs) show(h);          // 遍历（发射顺序）
if (n > 2) echo("many holes"); // WHERE                → if
c  = center("hole");           // 聚合（并集包围盒 lo/hi/center，v2 已有）
for (h = hs) drill(r=0.2, through=h, axis=2);  // 集合元素（Geom）可作既有 through= 实参
```

> **实施注记（2026-10-04）**：示例原写 `through=comp(hs, 0)`——实测 `comp` 是
> 向量取分量（3 元数字列表，`cgs_vec3` 语义），泛化到任意列表需要一个新的越界
> 错误文本，超出 §6 对 P5「仅 1 条新增」的盘点，故元素抽取以 `for (h = hs)`
> 遍历为规范写法（§4.5 test 4 即按此验收）；`drill through=` 本就接受 Geom，
> 零改动。

### 4.2 SQL → CGS 映射表（写进 LLM 提示词的对照）

| SQL | CGS |
|---|---|
| `SELECT * FROM t` | `instances("t")` |
| `SELECT x FROM t WHERE p(x)` | `for (x = instances("t")) { if (p(x)) { … } }` |
| `SELECT COUNT(*) FROM t` | `len(instances("t"))` |
| `MIN/MAX/CENTER …`（聚合） | `lo("t")` / `hi("t")` / `center("t")`（并集语义，v2 已有）；逐元素数值聚合用 `for` + 变量 |
| `UPDATE t SET … WHERE p` | ✗ 拒绝，正向替代见 §7 |
| `JOIN` / 子查询 / `ORDER BY` | 非目标（CGS 无关系代数；集合是 List，可用既有表达式组合） |

### 4.3 不引入 SELECT/FROM/WHERE 表面语法的理由

1. **`select/from/where/count` 保持词法空闲**：不占关键字 ⇒ 变量遮蔽规则零变化
   （`from = 1;` 照常可用），LLM 语料里 SQL 的"声明选择集"直觉不会与 CGS 的
   单遍模型打架。
2. **等价性**：`SELECT…WHERE` 的全部只读语义已被"一个返回 List 的函数 + 既有
   for/if"覆盖，引入关键字只会多一套解析面与错误族——违反"糖不是新语义"的
   反面教材：这里连糖都算不上，是纯冗余。
3. **SQL 强的是数据模型（集合是一等值），不是关键字**——P5 只取前者。

### 4.4 （可选子项）`face/fnrm` 单串面字面量

统一为与 `drill from/to` 同形的 `"名:key"`：

```cgs
face("plate:+z")     ≡  face("plate", "+z")
```

- 实现：`eval_query` 中 face/fnrm 分支先看 `pos.len()==1 && Str 含 ':'` → 拆成
  两参再走原路；不含 `:` → 落到既有 `face needs 2 argument(s)`。
- 错误全复用：坏 key → `parse_face_key` 既有文本；未知名 → `unknown reference`。
- 零新增错误文本；可独立延后实施。

### 4.5 验收（P5 新增 5 测试）

1. **顺序金样**：`for (i=[0:2]) tag("hole") translate([i,0,0]) …` 注册三实例，
   再 `for (h = instances("hole")) show(h);` → 落位 x=0,1,2（列表逆序会立刻
   被断言抓住）。
2. 计数与条件：`len(instances("hole"))` + `if` 组合落位。
3. 错误组：`instances("nope")` → `unknown reference`；`instances(5)` →
   `instances needs a reference name, got 5`；`instances("x");`（语句位）→
   expression-only 契约。
4. 聚合与元素互通：`center("hole")` 并集与 `for` 逐元素 `center(h)` 的几何一致；
   集合元素作 `drill through=` 实参。
5. （若实施 4.4）单串与两参形式等价金样。

**实施结果（2026-10-04）**：§4.4 一并实施；5 条新测试落地为
`test_cgs_p5_instances_{order,count_if,errors,aggregate_element}` +
`test_cgs_p5_face_single_string`，`make test` 188 全绿（cga-core 46 +
cga-gpu 142）；P5 实际新增错误文本恰 1 条（§6 盘点兑现）。

## 5. P6：不等式约束（解算器侧，语法零新面）

### 5.1 语义

`constrain` 方程体的顶层关系符分类（每个分号段**恰一个**，深度 0 检测沿用
2046–2063 的括号深度计数）：

| 关系 | 残差分量（进同一 GN 最小二乘） |
|---|---|
| `lhs == rhs` | `lhs − rhs`（既有，逐分量 `flatten`） |
| `lhs <= rhs` | `max(0, lhs − rhs)`（hinge：满足 ⇒ 残差 0 且梯度 0） |
| `lhs >= rhs` | 解析期翻转为 `rhs <= lhs`，同上 |
| `<` `>` | **视同闭包** `<=` `>=`（前向差分与 hinge 无法区分严格性；落点可在边界上） |
| `!=` | 拒绝，**P6 新增文本**：`CGS line N: constrain does not support != — use ==, <= or >=` |
| 多于一个顶层关系（含 `==`+`<=` 混写、`a <= b <= c`） | 泛化既有文本为 `CGS line N: one relation per constrain equation`（原文本 `one == per…` 2055 无任何测试/文档断言，安全替换） |
| 无关系符（`x;`） | 既有文本不动：`constrain equations must be \`lhs == rhs;\``（2073，测试 4194 断言逐位保留） |

**语义性质**（写进文档的三条，皆由 hinge 结构直接推出）：

1. **不等式只裁剪、不指定**：满足时残差与梯度同为 0，不牵引解 ⇒ 初值已可行则
   迭代 0 收敛、初值不动；合法集内的落点由初值与等式决定（与"文本即真相"一致）。
2. **可行 ⇔ 收敛**：`|r|∞ < 1e-10`（既有判据 2225）此时等价于"等式精确 +
   不等式违约量 ≤ 1e-10"；可行问题精确落解。
3. **不可行 ⇒ 显式失败**：残差下确界 > tol ⇒ 永不收敛 ⇒ 既有
   `CGS line N: constrain did not converge (|r|=…, iter=…, var=…)`（2247）。
   求解器不会静默给出最小折中解——与 v2 失败显式原则一致。

### 5.2 求解器改动清单（纯解算器侧）

| 位置 | 改动 |
|---|---|
| `constrain_stmt` 关系分类 2046–2076 | 按上表分类；`!=` 与多关系符分支 |
| `constrain_residual` 2131–2157 | 每个分量按关系推 `u−v` 或 `max(0, u−v)`；`flatten` 双侧与形状匹配检查沿用（向量不等式**逐分量**，如 `size(a) <= [1,2,3]`） |
| `constrain_solve` 2160–2247 | **不动**。GN/前向差分/阻尼对残差形态不可知；hinge 的 0 梯度天然给出"满足不牵引" |
| 语法面 | 零新增 token/语句/关键字（`< <= > >=` 词法早已支持） |

### 5.3 既有测试影响（唯一触点）

`test_cgs_constrain_errors`（4191–4192）当前断言 `x < 1` 报
`inequality solving is planned` —— P6 兑现该占位，此行改为求解成功断言
（`var x = 0.0; constrain(x) { x < 1; } solve;` → 初值可行、x 不动、不报错），
并补 `x != 1` 断言新文本。**这是全部 166 个测试中唯一需要改动的断言**；
`must be \`lhs == rhs;\``（4194）等其余断言逐位不动。

文档同步：`docs/cgs-v2.md` §6.3 提及 `inequality solving is planned` 的一行
改为指向本提案 §5；`README.md` 错误契约表 +2 行（§9）。

### 5.4 验收（P6 新增 6 测试 + 替换 1 断言）

1. 满足不动：`var x = 0.0; constrain(x) { x <= 5; } solve;` → 收敛于初值。
2. 违约推边：`var x = 5.0; constrain(x) { x <= 1; } solve;` → x≈1（闭包）。
3. 等式+不等式混合：`var x = 0.0; var y = 0.0; constrain(x, y) { x + y == 10; x >= 0; y >= 0; }` 可行收敛。
4. 不可行：`var x = 0.0; constrain(x) { x == 6; x <= 5; }` → `did not converge`（既有文本）。
5. `x != 1` → 新文本；`a <= b == c` 多关系符 → `one relation per constrain equation`。
6. `<` 视同 `<=` + 向量逐分量不等式金样。

**实施结果（2026-10-04）**：6 条新测试落地为 `test_cgs_p6_inequality_{
satisfied_no_pull,violated_to_boundary}`、`test_cgs_p6_mixed_eq_inequality`、
`test_cgs_p6_infeasible_explicit`、`test_cgs_p6_relation_errors`、
`test_cgs_p6_lt_closed_vector_components`；§5.3 的唯一触点断言已替换
（`x < 1` 占位错误 → 初值可行、x 不动的成功断言），`must be \`lhs == rhs;\``
等其余断言逐位不动。`make test` 194 全绿（cga-core 46 + cga-gpu 148）；
P6 实际新增错误文本恰 1 条、安全泛化 1 条（§6 盘点兑现）。

## 6. 错误契约继承总表（P4–P6 新增/复用盘点）

| 阶段 | 新增规范文本 | 复用既有 |
|---|---|---|
| P4 | **0 个新格式串**：语句位非 Geom 走 `geom_val` 既有格式（`what` 槽 = `expression statement`）；Dot 相关错误全落 `expect`/`bad expression start` 既有格式 | statement-only / expression-only / `unknown function` / 各 arity 与命名参数检查 / `geom_val` |
| P5 | 1 个：`instances needs a reference name, got {v}` | `unknown reference "x"` / `takes no named arguments` / `needs 1 argument(s)` / expression-only 契约 |
| P6 | 1 个新（`!=` 拒绝）+ 1 个安全泛化（`one == per…` → `one relation per…`） | `did not converge` / `must be \`lhs == rhs;\`` / `unknown … is not defined` / `must be a number` |

→ README 错误契约表在 P5/P6 落地时各 +1 行；P4 零新增行。

## 7. 明确拒绝：回溯式 `UPDATE … WHERE`（及一切回溯写选择）

| # | 理由 |
|---|---|
| 1 | **破坏文本即真相/语句时执行**：`UPDATE` 要求"发射后再回头改写场景对象"，实现上要么第二遍遍历、要么维护 scene 反向索引，执行模型从前向单遍变成有状态重放——v2 §2 原则 2 的反面 |
| 2 | **撞上未解决的 CSG 边界归属**：发射后的叶子实例在 `union/difference` 内部的可寻址性未解决（P0 的 `tag` 注册点在进入 CSG 收集之前；"选中布尔结果里的哪些面/体"是 `docs/freeform-robust-boolean.md` 未完成的拓扑问题）。`UPDATE…WHERE` 的谓词无论写什么都会踩在这个缺口上 |
| 3 | **语料直觉冲突且更冗长**：SQL 的 UPDATE 是"声明选择集 + 写操作"两段式；在 CGS 里任何等价物最终都展开成发射点的 for/if/参数——直接写正向形式更短、无二义、可断言 |

**正向替代**（P5 已备齐）：

- **读选择** = `instances()` + `for/if`（选择是求值，产出 List）。
- **写选择** = 发射点修饰语——`for/if` 决定哪些实例以何参数发射、`tag/translate/
  material` 决定它们的帧与外观。**选择永远发生在发射前，不存在回溯。**

同批拒绝：`SELECT/FROM/WHERE` 表面语法（§4.3）、jQuery `.end()` 回退栈与单 `|`
管道（§3.5）。

## 8. 收益对照（三标准）

| 标准 | P4 链式 | P5 集合 | P6 不等式 |
|---|---|---|---|
| LLM 语料相容性 | JS 高频模式，嵌套调用读序从"内外"变"左右"，prompt 更短 | SQL 高频的 SELECT 直觉有了对照表（§4.2），不再诱导出 UPDATE | CAD 尺寸/公差的自然表达（"厚度 ≤ 2"），减少 LLM 硬凑等式的绕路 |
| 可验证性 | 糖=去糖由等价金样断言；错误文本全部既有 | 顺序/计数/错误均可断言；关键字零新增 ⇒ 遮蔽规则不变 | 失败仍显式（`did not converge`）；新增文本仅 1 条 |
| 完备性 | 表达式位可用任意链，var/约束/实参全打通 | G1 的引用有了集合形态，COUNT/WHERE/聚合齐 | P2 从等式扩到闭不等式，可行域语义完整 |

## 9. 实施顺序与验收

| 阶段 | 内容 | 验收 |
|---|---|---|
| P4 | `TokenKind::Dot` 词法 + `primary` 后缀链 + 语句位判链 | 已实施 ✓（2026-10-04）：新增 6 测试（§3.6）；既有测试逐位不动，183 全绿 |
| P5 | `instances()` 入 `QUERY_FNS` + SQL 映射文档化 +（可选）face 单串 | 已实施 ✓（2026-10-04）：新增 5 测试（§4.5，含 §4.4）；既有测试不动，188 全绿 |
| P6 | 关系分类 + hinge 残差 + `!=` 拒绝 | 已实施 ✓（2026-10-04）：新增 6 测试 + 替换 1 断言（§5.4）；既有其余测试逐位不动，194 全绿 |
| 文档 | README（测试数、错误契约表 +2 行、CGS v3 特性段）、`cgs-v2.md` §6.3 占位行改指针、`scene_lang.rs` 文件头 `//!` 注释同步语法 | 已完成 ✓（P4–P6 三提交各自携带；`//!` 语法同步随 P4，`cgs-v2.md` 前向指针随提案提交） |

每阶段独立提交点：`cargo fmt --all` + `make test` 全绿后进入下一阶段
（沿 `cgs-v2.md` §10 惯例）。
