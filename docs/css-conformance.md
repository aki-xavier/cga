<!-- markdownlint-configure-file {"MD013": false} -->
# CSS 收敛计划：与真 CSS 对齐

状态：**已实施（2026-10-07）**，P0–P4 全部落地，`cargo test --workspace` 全绿（189 = cga-core 44 + cga-gpu 82 + cga-host 63），3 个金标 PNG 逐位不变。
适用：`crates/cga-host/src/jsx/mod.rs` 的 CSS 段（`// ---- CSS ----`：`parse_selector` / `sel_matches` / `parse_css` / `css_value` / `css_color` / `css_number` / `var_substitute`，以及 `Builder::style_for` / `Builder::build_material`、`build_scene_run` 的根作用域分支），以及 `docs/jsx-css-host.md` §2 / §4 / §5。
行号为 2026-10-07 快照，随代码漂移；**以符号名为准**。§2 的探针表是**实施前**的基线证据，用来解释为什么要做这些改动；实施后的行为以 §6 各阶段的验收与 `mod tests` 里的 `test_css_*` 为准。

本文回答两件事：现状到底差在哪（§2、§3 全部是**实测**结论，不是推断），以及按什么顺序修、每步怎么验收（§6–§8）。

## 1. 一句话现状

解析层一直是真 lightningcss（语法错误真报错、`!important` 能识别、值会被归一化）；**匹配层和取值层原本是自研极简子集**，只干一件事——给几何元素挑材质。真 CSS 的选择器组合、特异性、继承、`!important` 优先级、颜色与关键字取值，在这一层要么被静默改写，要么被静默丢弃。

最危险的不是"不支持"，而是**静默退化**：一条 `.a > .b` 写错了上下文，结果它退化成 `.b`，照样命中——作者以为约束生效了，其实没有。

实施后：选择器编译成 `ComplexSelector` 右到左匹配，级联是显式的五层阶梯，值由 `CssColor` / 单位检查 / `var()` 替换给出，不支持的构造一律报错。

## 2. 实测证据（探针）

对 `parse_css` / `css_match`（**实施前的符号**）直接跑输入，输出如下（探针为一次性测试，已删除；复现方式见 §8）。

| 输入 CSS | 实测输出 | 结论 |
| --- | --- | --- |
| `.red { color: red; }` | `("color","red")` | 具名颜色按字面输出，我方只认 `#hex` → **静默丢弃** |
| `.red { color: rgb(255, 0, 0); }` | `("color","red")` | lightningcss 会把 `rgb()` 归一化成具名，仍非 hex → 丢弃 |
| `.red { color: rgb(255 0 0 / 50%); }` | `("color","#ff000080")` | 8 位 hex → `i32::from_str_radix` 溢出 → 丢弃 |
| `.red { color: blorble; }` | 正常解析，`("color","blorble")` | **lightningcss 不校验属性值**，校验责任全在我方 |
| `.red { color: #FFF; }` | `("color","#fff")` | 大小写与格式归一化正常 ✓ |
| `.red { opacity: 50%; }` | `("opacity",".5")` | 百分比已被归一化成小数 → **能用** ✓（靠运气，不是靠我方） |
| `.red { unlit: true; }` | `("unlit","true")` | 关键字非数值 → 丢弃；必须写 `unlit: 1` |
| `.red { background: blue; }` | `("background","#00f")` | 归一化形式不稳定：`blue` → `#00f`，而 `red` → `red` |
| `.red:hover { … }` | `class: "red:hover"` | 伪类被并进 class 名 → **永不匹配** |
| `.a.b { … }` | `class: "b"` | **复合类塌缩成最后一个** |
| `.a > .b { … }` / `.a + .b { … }` / `.p .c { … }` | `class: "b"` / `class: "b"` / `class: "c"` | **组合器被吃掉，规则退化成最后一个简单选择器 → 过度匹配** |
| `div .c { … }` | `tag: "div "`、`class: "c"` | 空格留在 tag 里 → **永不匹配**（同一条规则的另一种失败方式） |
| `* { … }` | `tag: "*"` | 永不匹配 |
| `@media (…) { … }` | 0 条规则 | 静默丢弃 |
| `@import './x.css';` | 0 条规则，其后规则正常 | 静默忽略，**不跟随** |
| `@keyframes k { … }` | 0 条规则 | 静默丢弃 |
| `{ { {` | `CSS: Invalid empty selector at :0:1` | 语法错误真报错 ✓（带位置） |
| `.red { color:#000 !important; color:#fff; }` | `[("color","#fff"), ("color","#000")]` | `!important` 只被挪到**本规则**末尾；跨规则仍是源码顺序 |
| `.red { foo: bar; }` | `("foo","bar")` | 未知属性被忽略 ✓（与浏览器一致） |
| `:root { color: … }` 对 `<sphere class="x">` | **命中** | `:root`/`scene` 的 tag/class/id 全为 `None` → **匹配所有元素** |
| `scene { … }` | `scene: true` | `scene` 既是选择器别名，又是真实元素 tag，二者语义冲突 |
| `.a.b` 对 `class="a b"` | **未命中** | 多类属性整串比较 → 永不命中 |
| `Sphere { … }` 对 `el.tag="Sphere"` | 未命中（对 `"sphere"` 命中） | 选择器被 lowercased、`el.tag` 原样比较；内建元素经预置已是小写，实际不受影响，组件自造的混合大小写 tag 会漏 |

## 3. 差距总表

性质列：**bug** = 静默给出错误结果；**缺口** = 缺能力但结果正确或无影响；**一致** = 与真 CSS 相同。

| # | 项 | 真 CSS | 现状 | 性质 | 阶段 |
| --- | --- | --- | --- | --- | --- |
| G1 | `:root` 作用域 | 只作用根元素 | 匹配**所有**元素 | bug | P0 |
| G2 | 复合类 `.a.b` | 需同时具备 | 塌缩成 `.b`（过度匹配） | bug | P0 |
| G3 | 多类属性 `class="a b"` | 分词逐个匹配 | 整串比较，永不命中 | bug | P0 |
| G4 | 组合器 ` ` `>` `+` `~` | 正确匹配 | 退化成最后一个简单选择器（过度匹配）；带 tag 时永不匹配 | bug | P0 |
| G5 | `*`、`[attr]` | 支持 | `*` 永不命中；`[attr]` 被并进 tag 名 | 缺口 | P0 |
| G6 | 交互/动态伪类、伪元素 | 支持 | 被并进 class/tag 名 → 永不命中，且不报错 | 缺口 | P0 改为报错 |
| G7 | `@media` / `@keyframes` / `@import` / 嵌套 | 支持 | 静默丢弃 | 缺口 | P0 改为报错 |
| G8 | `scene` 选择器与元素 tag 冲突 | — | 同一词两种语义 | bug | P0 |
| G9 | 特异性 (a,b,c,d) | 有 | 无，只有源码顺序 | 缺口 | P1 |
| G10 | `!important` | 压过所有普通声明 | 只在本规则内排最后 | 缺口 | P1 |
| G11 | inline 与 important 的相对顺序 | inline normal < author important < inline important | 写死为 inline 永远最大 | 缺口 | P1 |
| G12 | 任意元素参与匹配 | 任意元素 | `<material class="x">` 不参与 CSS 匹配 | 缺口 | P1 |
| G13 | CSS 继承 | 按属性白名单继承 | 只有 `<material>` 祖先这一条通道 | 缺口 | P2 |
| G14 | `style={{…}}` | 支持 | 不支持（用 JSX prop 承担） | 缺口 | P2 可选 |
| G15 | 颜色值 | named / `rgb()` / `hsl()` / hex / alpha | 仅 `#rgb` / `#rrggbb` | 缺口 | P3 |
| G16 | 数值与单位 | 含 `%` 与各类单位 | 纯 `f64` 字面量，部分靠 lightningcss 归一化侥幸可用 | 缺口 | P3 |
| G17 | 关键字 / 布尔 | 支持 | `unlit: true` 不可用 | 缺口 | P3 |
| G18 | `var()` 自定义属性 | 替换 | `--x` 被当材质键**别名**，`var()` 不替换 | 缺口（需重新定义） | P3 |
| G19 | 未知属性 | 忽略 | 忽略 | 一致 ✓ | — |
| G20 | 语法错误 | 报错 | 报错（带位置） | 一致 ✓ | — |

**实施状态（2026-10-07）**：表里"现状"列描述的都是**实施前**的行为。

- **已修复并有测试**：G1–G13、G15–G18。
- **原就一致**：G19、G20。
- **明确不做**：G14（`style={{…}}`，§12-O4 = B，用 JSX prop 承担）。
- G14 之外，"缺口"行里所有报错化的项都已按 §7 落地：不支持的构造报错，未知属性仍静默。

## 4. 目标与非目标

**目标**

1. **写什么匹配什么**：选择器语义与 CSS 一致，绝不静默退化；不支持的**明确报错**。
2. **级联按规范**：特异性 + 源码顺序 + `!important` + inline 的相对顺序可预测、可测试。
3. **值可用**：材质键范围内的颜色、数值、关键字与浏览器行为一致。
4. **零回归**：P0/P1 完成后，8 个画廊场景（`test_jsx_gallery_smoke`）全部可渲染，3 个金标哈希（`gallery_render_golden`：orbit / mechanical / assembly）**逐位不变**。

**非目标（显式不做，不做就不报错也不实现）**

布局与盒模型、字体与文本、阴影/滤镜/`transform`（CSS 版）、动画与过渡、`::before`/`::after` 伪元素、交互伪类（`hover`/`focus`/`active`，本项目无事件源）、`calc()`/`min()`/`max()`/`clamp()`、`@supports`/`@layer`/`@charset`、CSS 预处理器/模块化、CSS 动态重算（样式在场景构建时求值一次，随 `SceneSession` rebuild 重算）、把 CSS 扩展到几何/变换/灯光（仍由 JSX prop 承担）。

## 5. 设计决策

- **D1 解析不自写**。继续用 lightningcss（`1.0.0-alpha.72`），我方只做「选择器匹配 + 级联 + 值转换」。语法层已经是对的，不要碰。
- **D2 选择器编译成结构，右到左匹配**（已实施）。

  ```rust
  enum Combinator { Descendant, Child, NextSibling, LaterSibling }   // 与左侧兄弟/祖先的连接
  struct Compound {
      root: bool,                                // :root 或裸 scene：只匹配深度 0
      tag: Option<String>, classes: Vec<String>, id: Option<String>,
      attrs: Vec<(String, Option<String>)>,      // [attr] / [attr=v]
  }
  struct ComplexSelector {
      parts: Vec<(Combinator, Compound)>,        // 最右为匹配目标
      specificity: Specificity(u16, u16, u16, u16),
      scene_scope: bool,                         // 目标复合选择器带根标记
      text: String,                              // 原文，用于错误信息
  }
  struct Frame<'a> { el: &'a El, index: usize }  // 遍历位置：根→当前
  struct Node<'a>  { el: &'a El, depth: usize, index: usize }  // 匹配中的任意节点
  ```

  组合器匹配需要**祖先与兄弟**：`walk` 维护 `Vec<Frame>`（根到当前），兄弟节点用 `Node{el,depth,index}` 表示（不一定在栈上）。`sel_matches` 从 `parts.len()-1` 向左递归，后代/普通兄弟带回溯。这是 P0 的结构改动，也是唯一波及几何发射路径的地方（`walk` 拆成 `walk` / `walk_pushed` / `walk_inner`，前者压栈并算 `style_for`，后两者分派）。
- **D3 级联是显式五层阶梯**（已实施，等价于 CSS 作者层的规则，只是没有 UA 层）：

  ```
  0 继承 < 1 样式表普通 < 2 内联普通 < 3 样式表 !important < 4 内联 !important
  ```

  同层内按 `specificity`，再按 `order`（源码顺序）。`cascade_rank(important, inline) -> u8` 是这条阶梯的唯一入口。`Inherited` 层由 `style_for` 先铺（只铺 `MATERIAL_KEYS` + `--*`），因此不需要参与排序。


  与 CSS 的差异只有一处：没有 UA 层。`!important` 的优先级关系（inline normal < author important < inline important）与浏览器一致。
- **D4 `:root` 与 `scene` 限定到根**（已实施）。`:root` 与**整条**选择器恰为 `scene` 的写法编译成 `Compound { root: true }`，`compound_matches` 要求 `depth == 0`。`scene_scope` 取自**目标**复合选择器的 `root` 标记，`build_scene_run` 只消费 `sel.scene_scope` 规则的 `background`。`scene .a` 这类复合写法里的 `scene` 仍是普通类型选择器，语义不再冲突。
- **D5 不支持就报错，未知属性仍静默**（已实施）。选择器里出现交互伪类 / 伪元素 / 无法处理的 `@` 规则 / CSS 嵌套 → 报错；已知材质键取到无法解析的值 → 报错；**属性名未知 → 忽略**（与浏览器一致）。详见 §7。
- **D6 取值不依赖字符串字面形式**（spike 已完成）。探针证明 `value_to_css_string` 的输出不稳定（`rgb(255,0,0)` → `red`、`blue` → `#00f`）。`alpha.72` 的类型化入口可用：`lightningcss::values::color::CssColor::parse_string`（`impl Parse` 于 `traits`）一个入口覆盖 named / `rgb()` / `hsl()` / `hwb()` / hex / `transparent`，`RGBA.alpha` 是 `u8`（`÷255` 得 alpha）。**没有自建颜色表**；`currentcolor` 与 `lab()/oklch()` 等色空间 → 报错。数值、单位、关键字与 `var()` 仍走字符串（`css_number` / `css_bool` / `var_substitute`），因为这些本来就由我方语义决定。
- **D7 八位 hex 的 alpha 语义**（已实施）：拆成 RGB 与 alpha；**alpha 仅在最终样式里没有 `opacity` 时生效**（`opacity = alpha`），来自显式声明或继承的 `opacity` 都会压过它。
- **D8 `--x` 双写兼容**（已实施，§12-O5 = A）：`--name` 若 `name` 是材质键，**继续按材质键生效**（`style_for` 同时推入 `--name` 与 `name` 两条声明，参与同一套级联），同时存为自定义属性供 `var(--name)` 替换；`var()` 替换在值解析前做文本代入（`var_substitute`，支持 `var(--x, fallback)`）。旧用法不破坏，新语义可用。
- **D9 tag 比较 ASCII 大小写不敏感**（已实施）：CSS 类型选择器本就不区分大小写，`compound_matches` 用 `el.tag.eq_ignore_ascii_case(tag)`；选择器侧不 lowercased，而是比较时不敏感（两者等价，且不破坏 tag 原文用于错误信息）。
- **D10 不加宽容/严格双模式**（已拍板，§12-O1 = A）：只有一条路线，不实现 `CGA_CSS_LENIENT`。存量 `examples/**/*.css` 已扫描，没有任何写法会被新报错规则命中。

## 6. 分阶段实施

**全部完成（2026-10-07）。** 下表的"预估"与各阶段的 `改动点` 文件行号是计划期的快照；实施后的符号名见该阶段的**实施记录**。

| 阶段 | 内容 | 预估 | 依赖 | 状态 |
| --- | --- | --- | --- | --- |
| **P0** | 选择器匹配正确性 + 静默退化改报错 | ~350 行（含测试） | 无 | ✅ 已完成 |
| **P1** | 特异性、`!important`、inline 顺序、`<material>` 参与匹配 | ~200 行 | P0 | ✅ 已完成 |
| **P2** | CSS 继承（属性白名单）、可选 `style={{}}` | ~250 行 | P1 | ✅ 已完成（`style={{}}` 按 O4 不做） |
| **P3** | 颜色/数值/关键字/`var()` 取值 | ~350 行 | P0（与 P1/P2 可并行） | ✅ 已完成 |
| **P4** | `@` 规则策略收口、`@import` 决策 | ~80 行 | P0 已把"报错"兜底 | ✅ 已完成（`@import` 报错，O3 = B） |

### P0 — 选择器匹配正确性

**改动点**

1. `parse_selector_part`（`jsx/mod.rs:435`）→ `parse_selector(text) -> Result<ComplexSelector, String>`：识别组合器、复合选择器、多类、属性选择器、`*`、`:root`；无法识别的（交互伪类、伪元素）返回错误并带上原文。
2. `SimpleSel` / `StyleRule`（`:421`–`:433`）→ `Compound` / `ComplexSelector` / `StyleRule { sel, props, source }`。
3. `css_match`（`:494`）→ `matches(el, ctx) -> bool`，右到左求值；`ctx` 含祖先栈、前兄弟、`is_root`。
4. `material_for`（`:560`）与 `Builder::walk` 签名同步扩上下文（**唯一波及几何发射的地方**）。
5. `parse_css`（`:460`）：非 `CssRule::Style` 的规则 → 报错（不再 `else { continue; }`）。
6. `build_scene_run` 的 `:root` 分支（`:1662`）保留（仍取 `background`），但不再与 `css_match` 的全命中耦合。
7. D9 大小写不敏感。

**验收**

- 选择器表驱动测试 ≥ 25 条：复合类、多类、四类组合器、`*`、`[attr]`、`:root` 根限定、`scene` 不再全命中、带 tag 的后代选择器。
- 报错测试 ≥ 6 条：`:hover`、`::before`、`@media`、`@import`、`@keyframes`，错误文本含选择器原文。
- **金标复验**：8 个 `examples/jsx/*.css` 场景照常渲染，3 个金标（orbit / mechanical / assembly）PNG 逐位不变。
- 现有 `test_jsx_css_material` 与全部 189 测试保持通过（cga-core 44 + cga-gpu 82 + cga-host 63）。

**风险**：`material_for` 签名变动触及 CSG 与图元两条发射路径，需覆盖两者的行为测试。

**实施记录（2026-10-07）**

- `parse_selector_part` → `parse_selector(text) -> Result<ComplexSelector, String>`：手写扫描器处理 `.class` / `#id` / `*` / `[attr]` / `[attr=v]` / `:root` 与四类组合器；`PSEUDO_ELEMENT_NAMES` 按名字识别伪元素（lightningcss 会把 `::before` 归一成 `:before`，所以 `::` 检测只兜住没被归一的情况）。
- `SimpleSel` / `css_match` → `Compound` / `ComplexSelector` / `sel_matches` / `match_from`：右到左递归，后代与普通兄弟带回溯；位置用 `Frame`（栈上，根→当前）与 `Node{el,depth,index}`（栈外的兄弟节点）表示。
- `material_for` 消失，改由 `style_for` + `build_material` 承担；`walk` 拆成 `walk`（压栈）→ `walk_pushed`（算 `style_for`）→ `walk_inner`（分派），`primitive_el` / `joint_el` / CSG / `drill` / `instances` 全部改吃已经算好的 `style`，CSG 与图元两条发射路径都被覆盖。
- `parse_css` 对非 `CssRule::Style` 报错（按变体给出 `@media` / `@import` / `@keyframes` / `@supports` / `@font-face` 名字，其余归 `an at-rule`）；`style.rules` 非空（CSS 嵌套）→ 报错。
- D9：`compound_matches` 用 `el.tag.eq_ignore_ascii_case(tag)`。
- 测试：`test_css_combinators`（后代 / 子代 / 相邻兄弟 / 普通兄弟 / 复合多类 / 通配 / 大小写）、`test_css_selector_table`（31 行表驱动）、`test_css_attribute_selector`、`test_css_selector_list_split`、`test_css_root_scope_only`、`test_css_unsupported_constructs_error`。
- 金标复验：`test_jsx_gallery_smoke` 8 场景 + `gallery_render_golden` 3 金标逐位不变；全量 189 测试通过。

### P1 — 级联

**改动点**

1. `parse_selector` 内计算 specificity；`css_match` → `cascade(el, ctx) -> Vec<Decl>`，按 D3 排序。
2. `material_for`（`:560`）改用 cascade 结果；inline prop 的优先级改为 D3 定义的 origin 顺序（P0 期间保持现状，本阶段一次性切换）。
3. `Builder::walk` 的 `"material"` 分支（`:652`）也走 cascade —— 修复 G12。
4. 单条规则内 `important_declarations` 的位置不再用 `chain()` 凑，改用 `important` 标志参与排序。

**验收**

- 级联优先级矩阵 ≥ 12 条：`#id` > `.class` > `tag`、同特异性看源码顺序、`!important` 压过后续普通规则、inline normal < author important < inline important、`<material class>` 命中。
- 金标复验（P1 不应改变任何现有场景输出——现用 CSS 全是单类 + 源码顺序，无 `!important`）。

**风险**：inline 顺序切换可能影响"祖先 `<material>` 与元素 CSS 规则"同时存在的场景；现用示例里 `<material>` 只带 inline prop，理论不受影响，靠测试确认。

**实施记录（2026-10-07）**

- specificity 在 `parse_selector` 内累积：`#id` → a，`.class` / `[attr]` / `:root` → b，type → c，`*` → 0（`Specificity(u16,u16,u16,u16)`）。
- 级联没有单独的 `cascade()`：`Builder::style_for` 一次收集样式表声明与内联 prop 成 `Vec<PendingDecl>`，`pending.sort_by(rank → specificity → order)` 后顺序应用；P3 的值转换走同一条列表，所以只遍历一次。
- `important` 来自 `StyleDecl.important`（`parse_css` 分别遍历 `declarations` 与 `important_declarations`），不再用 `chain()` 凑位置。
- G12：任何元素（含 `<material>`）的内联材质 prop 都以 `Inline` origin 进入同一条级联；`walk_inner` 的 `"material"` 分支不再自己合并，直接把 `style` 下传。
- 测试（合计 12 条，跨三个用例）：`test_css_cascade_order` 10 条矩阵（`#id` > `.class` > `tag` > `*`、同特异性看源码顺序、同 important 看源码顺序、`!important` 跨规则与压过后写普通规则、`#id !important` > `.class !important`、inline 普通 vs 样式表普通、样式表 important vs inline 普通）+ `test_css_root_scope_only`（继承 < 本元素声明）+ `test_css_inherits_through_containers`（`<material class>` 参与匹配，G12）。
- 金标不变（现用 CSS 全是单类 + 源码顺序，无 `!important`）。
- 注：阶梯的第 4 层（内联 `!important`）目前**不可书写**——JSX prop 没有 `!important` 语法，`style={{}}` 也按 O4 不做；这一层是为将来保留的，无测试。

### P2 — 继承与作用域

**改动点**

1. CSS 声明纳入继承源：walk 维护"继承中的材质"（属性白名单：`color roughness metalness emissive opacity ior absorption map unlit`），与 `<material>` 祖先通道合并，合并顺序按 D3（`Inherited` 最低）。
2. 根作用域属性白名单：`background`（+ 可选 `background-color`）只在根作用域生效，不进元素继承。
3. 可选：`style={{…}}` 内联对象 → `Inline` origin（与 JSX prop 等价）。

**验收**

- 继承链测试：父规则声明被子几何继承、子元素匹配规则时覆盖继承值、`<material>` 与 CSS 混合的三层优先级。
- **本阶段可能改变渲染输出**（子元素此前不继承 CSS 声明）：金标复验若发现漂移，先量化差异，再决定重生成金标还是收窄继承白名单 —— 见 §12-O2。

**风险**：继承是唯一会改变现有画面的阶段；把它单独隔离，便于回退。

**实施记录（2026-10-07，O2 = B）**

- 继承源是 `style_for(el, stack, inherited)` 的第一步：**只铺 `MATERIAL_KEYS` 与 `--*`**（白名单即 §12-O2 的"收窄"；`background` 等展示属性不进元素继承）。这一步决定了 P2 不改变画面。
- `<material>` 祖先通道与 CSS 声明合并成一条链：`walk` 的 `mat` 参数就是**父元素的计算样式**，`"material"` 分支不再自己 `merged`，内联 prop 统一走 `style_for` 的 `Inline` 级。所以"祖先 `<material>` → CSS → 内联 prop"三层优先级不再靠两条通道拼，而是同一条级联。
- 根作用域 `background` 单独在 `build_scene_run` 消费（`sel.scene_scope` 规则，含 `var()` 替换与 `css_color` 校验），不进元素继承。
- `style={{…}}` 不做（O4 = B）。
- 测试：`test_css_inherits_through_containers`（父规则声明被子几何继承）、`test_css_root_scope_only`（根作用域 `background` 只进 `scene.background`）。
- 金标不变：`examples/**/*.jsx` 里没有任何容器元素携带材质 prop，白名单收窄后行为与实施前一致。

### P3 — 值

**改动点**

1. `css_value_to_arg`（`:511`）→ `css_value(name, raw) -> Result<ArgValue, String>`，按属性分派；先做 D6 的 API spike。
2. 颜色：named / `rgb()` / `rgba()` / `hsl()` / `hsla()` / `#rgb` / `#rrggbb` / `#rrggbbaa` / `transparent`；D7 的 alpha → opacity。
3. 数值：`<number>` 与 `<percentage>`（`25%` → `0.25`，仅对 0–1 量纲的键）；带 `px`/`em`/`rem`/`deg` 的值 → 报错（材质键无量纲）。
4. 关键字：`unlit: true/false`、`yes/no` 可选；`color: currentcolor` 明确报错。
5. `var(--x)` 文本代入（D8），无 fallback 的缺失变量 → 报错；`var(--x, fallback)` 支持 fallback。
6. 未列入材质键的属性 → 静默忽略（维持现状）。

**验收**

- 值转换表 ≥ 30 条（含每个颜色语法 1 条、`50%`、`unlit: true`、`var()` 三条、错误文本 ≥ 8 条）。
- `examples/**/*.css` 扫描（8 个文件，**无 `@` 规则**，选择器只有 `:root` + 单类，属性 10 个且全部属于 `MATERIAL_KEYS`/`background`，值只有 `#hex`、纯数、`url()`）：存量写法全部继续可用。
- 金标复验逐位不变。

**风险**：D6 的类型化 API 在 `alpha.72` 上是否可用未核实，spike 失败则退回字符串解析（代价：自己实现 named/rgb/hsl 表）。

**实施记录（2026-10-07）**

- `css_value_to_arg` → `css_value(name, raw) -> Result<ArgValue, String>`，按属性分派：颜色走 `css_color`，`map` 走 `url()` 剥离，`unlit` 走 `css_bool`，其余走 `css_number`。
- 颜色由 `CssColor::parse_string` 一个入口覆盖 named / `rgb()` / `rgba()` / `hsl()` / `hsla()` / `hwb()` / `#rgb` / `#rgba` / `#rrggbb` / `#rrggbbaa` / `transparent`；`currentcolor` 与 `lab()/oklch()/display-p3` 等色空间 → 报错（没有自建颜色表）。
- alpha：`css_color` 返回 `(hex, rgba.alpha as f64 / 255.0)`（`RGBA.alpha` 是 `u8`）；`style_for` 末尾只在最终样式里没有 `opacity` 时写入（D7）。
- 数值：`css_number` 先挡 `calc()/min()/max()/clamp()` → 报错，再把 `25%` 折成 `0.25`，再用 `CSS_UNITS` 表挡 `px/em/rem/deg/…` → 报错，最后 `parse::<f64>`。
- `var_substitute`：括号配对找 `var(...)`、顶层逗号切 fallback、未定义且无 fallback → 报错；自定义属性在 `style_for` 的 pass A 先落地（含继承来的 `--*`），pass B 才转常规值。
- 关键字：`css_bool` 认 `true/false/yes/no/on/off/1/0`。
- 未知属性在**值转换之前**就被过滤，所以 `width: 10px` 静默忽略而 `roughness: 1px` 报错——这条界线是 §7 的核心。
- 测试：`test_css_values`（rgb / hsl / named / 八位 hex → opacity / `50%` 与 `25%` / `unlit: true` / `var()` 与 fallback / `--x` 别名 / 未知属性静默）+ `test_css_value_errors`（`blorble`、单位、缺失变量）。
- 金标不变；`examples/**/*.css` 存量写法（`#hex`、纯数、`url()`）全部继续可用。

### P4 — `@` 规则收口

**改动点**

1. 确认 P0 的报错策略后，逐条决定：`@import` 是否跟随（相对路径，与 `import './x.css'` 同一解析规则）还是保持报错；`@media` 是否恒定不支持（报错）。
2. CSS 嵌套规则（lightningcss 能解析的 `CssRule::Nesting`）：P0 已按"非 Style 规则"报错，此处确认覆盖到。

**验收**：错误文本表补 3–5 条；`examples` 无 `@` 规则（已扫描确认），不受影响。

**实施记录（2026-10-07，O3 = B）**

- `parse_css` 对所有非 `CssRule::Style` 的规则报错，按变体给出名字：`@media` / `@import` / `@keyframes` / `@supports` / `@font-face` / `a nested rule`，其余归 `an at-rule`。
- CSS 嵌套不走 `CssRule::Nesting`：lightningcss 把嵌套规则挂在 `StyleRule.rules`，所以判据是 `!style.rules.0.is_empty()` → 报错。
- `@import` 保持报错（不跟随）：JS 侧 `import './x.css'` 已覆盖"拆分多份样式表"的需求。
- 测试：`test_css_unsupported_constructs_error` 6 条（`:hover` / `::before` / `@media` / `@import` / `@keyframes` / 嵌套）。
- `examples/**/*.css` 无 `@` 规则，不受影响。

## 7. 错误契约

统一前缀 `CSS:`，错误文本必须含触发的**原文**。具体格式见本节末尾。

| 情况 | 行为 |
| --- | --- |
| 语法错误 | 报错（现状已正确，`lightningcss` 带位置） |
| 选择器含交互/动态伪类（`:hover` 等）、伪元素 | 报错（`::before` 被 lightningcss 归一成 `:before` 时按 `PSEUDO_ELEMENT_NAMES` 识别；未归一的 `::` 直接报） |
| `@media` / `@keyframes` / `@supports` / `@font-face` / 其它 `@` 规则 | 报错（实施前：静默丢弃） |
| `@import` | **报错**（O3 = B，不跟随；JS 侧 `import './x.css'` 覆盖该需求） |
| CSS 嵌套（`StyleRule.rules` 非空） | 报错（实施前：静默丢弃） |
| 已知材质键取到无法解析的值（`roughness: blorble`、`roughness: 1px`、`roughness: calc(...)`） | 报错 |
| `color: currentcolor`、`lab()/oklch()` 等非 RGB 色空间 | 报错 |
| `var(--x)` 无 fallback 且未定义 | 报错（有 fallback 则用 fallback） |
| 未知属性名（`foo: bar`、`width: 10px`） | **静默忽略**（与浏览器一致）；判据是**属性名**，不是值 |
| `background`/`background-color` 写在非根作用域规则里 | 静默忽略（对元素没有渲染意义）；写在 `:root`/`scene` 规则里则校验并作用于 `scene.background` |

报错必须可发现且可定位：错误文本含触发的**原文**，便于在多规则 `.css` 里一眼找到。格式：

```
CSS: <选择器原文> <属性名>: `<值原文>` <原因>          例：CSS: .red color: `blorble` is not a valid color
CSS: <属性名>: <原因>                                例：CSS: color: var(--brand) is not defined
CSS: <lightningcss 已给位置时原样透传>                例：CSS: Invalid empty selector at :0:1
```

## 8. 测试计划与阶段门

**测试结构**（已实现，在 `src/jsx/mod.rs` 的 `mod tests` 内，与 `test_jsx_css_material` 同处）

| 测试 | 覆盖 |
| --- | --- |
| `test_css_selector_table` | 31 行表驱动：类 / 复合多类 / 重复类 / type / 大小写 / `*` / `#id` / `:root` / `scene` / 后代 / 子代 / 相邻兄弟 / 普通兄弟 / 属性选择器 / 组合链 |
| `test_css_combinators` | 四类组合器各一条 + 旧的"退化成最后一个 simple selector"回归 + `*` + tag 大小写 |
| `test_css_attribute_selector` | `[attr=v]` 命中 / 不命中 / 不支持的运算符报错 |
| `test_css_selector_list_split` | 选择器列表按**顶层**逗号切（`[class="a,b"]` 里的逗号不算）；尾逗号报错 |
| `test_css_root_scope_only` | `:root` 只命中根、根 `background` 只进 `scene.background`、根 `color` 的继承 |
| `test_css_cascade_order` | 10 条级联矩阵（见 P1 实施记录） |
| `test_css_values` | 颜色 7 种语法、`%`、关键字、`var()` + fallback、`--x` 别名、未知属性静默 |
| `test_css_value_errors` | `blorble` / 单位 / `calc()` / 缺失变量 / `currentcolor` |
| `test_css_unsupported_constructs_error` | `:hover` / 伪元素 / `@media` / `@import` / `@keyframes` / CSS 嵌套 |
| `test_css_inherits_through_containers` | 父规则声明被子几何继承（G12 + G13） |

**每个阶段的硬门槛**

1. `cargo test --workspace` 全绿（当前 **189** = cga-core 44 + cga-gpu 82 + cga-host 63；新增测试后 README 的计数要同步更新）。
2. `cargo fmt --all -- --check` 干净；clippy 无新增警告。
3. 金标复验：P0/P1/P3 必须逐位一致；P2 若不一致，按 §12-O2 决策后再动。**实施结果：8 个画廊场景 + 3 个金标逐位不变。**
4. `examples/**/*.css` 全量扫描，确认没有被新报错规则命中的存量写法。

**探针复现**（供后续核对本文 §2）：在 `mod tests` 里临时加一个打印 `parse_css(...)` 结果的测试，`cargo test -p cga-host --lib <name> -- --nocapture`，看完删除，不入库。

## 9. 风险（已按实施结论更新）

- **签名波及面**（✅ 已消解）：`material_for` 已删除，改由 `walk` → `walk_pushed` → `walk_inner` 一次性把算好的 `style` 传给 CSG / 图元 / `drill` / `instances` / `joint` 五条路径，不存在"漏一条就有一类元素不继承"的可能；`test_css_inherits_through_containers` 与 `test_css_selector_table` 各自覆盖容器与图元。
- **金标漂移**（✅ 未发生）：P2 用"只继承 `MATERIAL_KEYS` + `--*`"的白名单实现，`examples` 里没有容器元素携带材质 prop，行为与实施前一致；8 个画廊场景 + 3 个金标逐位不变。
- **lightningcss 版本**（✅ spike 成功）：`alpha.72` 的 `CssColor::parse_string` 可用，没有自建颜色表；`RGBA.alpha` 是 `u8`（需 `÷255`）。遗留边界：`lab()/oklch()/display-p3` 等色空间 → 报错而不是转换。
- **报错收紧**（✅ 无命中）：`examples/**/*.css` 8 个文件全量扫描，无 `@` 规则、只有 `:root` + 单类选择器、值只有 `#hex` / 纯数 / `url()`，没有写法被新报错规则命中。
- **性能**（✅ 无感）：匹配是"每元素×每规则×选择器链长"+ 一次稳定排序，规则约 10 条、元素数十至数百；`test_jsx_gallery_smoke` + `gallery_render_golden` 的整体耗时仍在原量级。
- **遗留**：阶梯第 4 层（内联 `!important`）当前不可书写（无 `style={{}}`，JSX prop 无 `!important`）；`:first-child` 等结构伪类按 D5 报错，需要时另开阶段。

## 10. 明确不做

布局/盒模型/字体/文本/阴影/滤镜、动画与过渡、`::` 伪元素、交互伪类、`calc()` 与数学函数、`@supports`/`@layer`/`@charset`、CSS 动态重算（样式随 `SceneSession` rebuild 一次性重算）、把 CSS 扩展到几何/变换/灯光、CSS 预处理器与模块化。

## 11. 文档同步（已完成）

- `docs/jsx-css-host.md`：
  - §2 约定的"CSS 只认单层选择器"改写为最终能力面（组合器、特异性、`!important`、继承、`var()`、报错边界），并指向本文；
  - §4 `v1 边界` 中"CSS 后代/子代组合选择器"一条改为**已支持**，其余边界（伪类/伪元素、`@` 规则、`calc()`、`style={{}}`）明确列出；
  - §5 验收补：选择器匹配表（`test_css_selector_table`）、级联矩阵（`test_css_cascade_order`）、值转换与错误文本（`test_css_values` / `test_css_value_errors`）、`@` 规则报错（`test_css_unsupported_constructs_error`）、金标复验。
- `README.md`：L114 / L118 的描述从 "CSS rules apply materials by tag/class/id" 更新为真实能力面，并指向本文。
- 测试计数：README 与本文 §8 同步为 **189 = 44 + 82 + 63**。
- 本文件：状态已改为**已实施**并标注日期，每个阶段补了实施记录。

## 12. 决策（已全部拍板，按此实施）

| # | 问题 | 选项 | 决定 | 实施结果 |
| --- | --- | --- | --- | --- |
| O1 | 不支持的构造报错还是忽略 | A 报错 / B 忽略 / C `CGA_CSS_LENIENT` 开关 | **A**（本仓库确定性错误契约） | 按 A 实现，未加开关 |
| O2 | P2 继承若改变金标 | A 重生成金标 / B 收窄继承白名单保持画面 / C 取消 P2 | **B 优先** | 按 B 实现（只继承 `MATERIAL_KEYS` + `--*`），金标未变，无需 A |
| O3 | `@import` | A 跟随（与 JS import 同规则）/ B 报错 | **B**，JS 侧 `import './x.css'` 已覆盖该需求 | 按 B 实现 |
| O4 | `style={{…}}` | A 做 / B 不做（用 JSX prop） | **B**，与"材质即 prop"的既有约定一致 | 未实现；副作用是阶梯第 4 层暂不可书写 |
| O5 | `--x` 别名 | A 双写兼容（D8）/ B 立即切换为纯变量语义 | **A**，零迁移 | 按 A 实现：`--name` 同时写入材质键与自定义属性 |
