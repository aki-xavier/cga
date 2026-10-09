//! CSS 层：选择器编译、匹配、级联、值转换（语法与值的归一化交给
//! lightningcss，差距清单与阶段见 docs/css-conformance.md）。

use std::collections::HashMap;

use serde_json::Value;

use crate::scene_build::ArgValue;

use super::element::El;

// ---- CSS ----
//
// 解析交给 lightningcss（语法与值的归一化都是真 CSS）；这里只做三件事：
// 选择器编译、级联、值转换。差距清单与阶段见 docs/css-conformance.md。

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
pub(crate) struct Specificity(u16, u16, u16, u16);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Combinator {
    Descendant,
    Child,
    NextSibling,
    LaterSibling,
}

/// 一个复合选择器：type / `.class` / `#id` / `[attr]` / `:root` / `*` 的组合。
#[derive(Clone, Debug, Default)]
pub(crate) struct Compound {
    /// `:root` 或裸 `scene`：只匹配深度为 0 的根元素。
    pub(crate) root: bool,
    pub(crate) tag: Option<String>,
    pub(crate) classes: Vec<String>,
    pub(crate) id: Option<String>,
    pub(crate) attrs: Vec<(String, Option<String>)>,
}

/// 复杂选择器。`parts` 最右是匹配目标；`parts[i].0` 描述它与左侧
/// （祖先或前兄弟）之间的连接。
#[derive(Clone, Debug)]
pub(crate) struct ComplexSelector {
    pub(crate) parts: Vec<(Combinator, Compound)>,
    pub(crate) specificity: Specificity,
    /// 目标复合选择器带根标记 → 根作用域规则（背景等由 `build_scene_run` 消费）。
    pub(crate) scene_scope: bool,
    pub(crate) text: String,
}

#[derive(Clone, Debug)]
pub(crate) struct StyleDecl {
    pub(crate) name: String,
    pub(crate) value: String,
    pub(crate) important: bool,
}

#[derive(Clone, Debug)]
pub(crate) struct StyleRule {
    pub(crate) sel: ComplexSelector,
    pub(crate) decls: Vec<StyleDecl>,
}

pub(crate) fn css_err(what: &str, msg: &str) -> String {
    format!("CSS: {what}: {msg}")
}

/// lightningcss 会把 `::before` 归一成 `:before`，所以伪元素按名字识别。
pub(crate) const PSEUDO_ELEMENT_NAMES: &[&str] = &["before", "after", "first-line", "first-letter"];

pub(crate) fn skip_ws(b: &[u8], i: &mut usize) -> bool {
    let start = *i;
    while *i < b.len() && b[*i].is_ascii_whitespace() {
        *i += 1;
    }
    *i > start
}

pub(crate) fn scan_ident(b: &[u8], i: &mut usize) -> Option<String> {
    let start = *i;
    while *i < b.len() {
        let c = b[*i];
        if c.is_ascii_alphanumeric() || c == b'-' || c == b'_' || c >= 0x80 {
            *i += 1;
        } else {
            break;
        }
    }
    if *i == start {
        None
    } else {
        Some(String::from_utf8_lossy(&b[start..*i]).into_owned())
    }
}

pub(crate) fn parse_attr(
    b: &[u8],
    i: &mut usize,
    sel: &str,
    c: &mut Compound,
) -> Result<(), String> {
    *i += 1; // '['
    skip_ws(b, i);
    let name =
        scan_ident(b, i).ok_or_else(|| css_err(sel, "expected an attribute name after '['"))?;
    skip_ws(b, i);
    if *i < b.len() && b[*i] == b'=' {
        *i += 1;
        skip_ws(b, i);
        if *i >= b.len() {
            return Err(css_err(sel, "unterminated attribute selector"));
        }
        let value = if b[*i] == b'"' || b[*i] == b'\'' {
            let quote = b[*i];
            *i += 1;
            let start = *i;
            while *i < b.len() && b[*i] != quote {
                *i += 1;
            }
            if *i >= b.len() {
                return Err(css_err(sel, "unterminated attribute value"));
            }
            let v = String::from_utf8_lossy(&b[start..*i]).into_owned();
            *i += 1;
            v
        } else {
            let start = *i;
            while *i < b.len() && b[*i] != b']' {
                *i += 1;
            }
            String::from_utf8_lossy(&b[start..*i]).trim().to_string()
        };
        skip_ws(b, i);
        if *i >= b.len() || b[*i] != b']' {
            return Err(css_err(
                sel,
                "unsupported attribute operator: only [attr] and [attr=value] are supported",
            ));
        }
        *i += 1;
        c.attrs.push((name, Some(value)));
    } else if *i < b.len() && b[*i] == b']' {
        *i += 1;
        c.attrs.push((name, None));
    } else {
        return Err(css_err(
            sel,
            "unsupported attribute selector or operator: only [attr] and [attr=value] are supported",
        ));
    }
    Ok(())
}

/// 把 lightningcss 归一化过的（逗号分隔的）单条选择器编译成结构化的
/// `ComplexSelector`。无法识别的构造一律报错，不做静默降级。
pub(crate) fn parse_selector(text: &str) -> Result<ComplexSelector, String> {
    let s = text.trim();
    if s == ":root" || s == "scene" {
        let specificity = if s == ":root" {
            Specificity(0, 1, 0, 0)
        } else {
            Specificity(0, 0, 1, 0)
        };
        return Ok(ComplexSelector {
            parts: vec![(
                Combinator::Descendant,
                Compound {
                    root: true,
                    ..Compound::default()
                },
            )],
            specificity,
            scene_scope: true,
            text: s.to_string(),
        });
    }

    let b = s.as_bytes();
    let mut i = 0usize;
    let mut parts: Vec<(Combinator, Compound)> = Vec::new();
    let mut specificity = Specificity::default();
    let mut pending = Combinator::Descendant;
    skip_ws(b, &mut i);
    if i >= b.len() {
        return Err(css_err(s, "empty selector"));
    }

    loop {
        // 一个复合选择器
        let mut c = Compound::default();
        let mut any = false;
        loop {
            if i >= b.len() {
                break;
            }
            match b[i] {
                b'.' => {
                    i += 1;
                    let n = scan_ident(b, &mut i)
                        .ok_or_else(|| css_err(s, "expected a class name after '.'"))?;
                    c.classes.push(n);
                    specificity.1 += 1;
                    any = true;
                }
                b'#' => {
                    i += 1;
                    let n = scan_ident(b, &mut i)
                        .ok_or_else(|| css_err(s, "expected an id after '#'"))?;
                    c.id = Some(n);
                    specificity.0 += 1;
                    any = true;
                }
                b'*' => {
                    i += 1;
                    any = true;
                }
                b'[' => {
                    parse_attr(b, &mut i, s, &mut c)?;
                    specificity.1 += 1;
                    any = true;
                }
                b':' => {
                    if i + 1 < b.len() && b[i + 1] == b':' {
                        return Err(css_err(s, "pseudo-elements (::) are not supported"));
                    }
                    i += 1;
                    let n = scan_ident(b, &mut i)
                        .ok_or_else(|| css_err(s, "expected a pseudo-class name after ':'"))?;
                    if n.eq_ignore_ascii_case("root") {
                        c.root = true;
                        specificity.1 += 1;
                    } else if PSEUDO_ELEMENT_NAMES
                        .iter()
                        .any(|p| p.eq_ignore_ascii_case(&n))
                    {
                        // lightningcss 会把 `::before` 归一成 `:before`，这里按名字识别。
                        return Err(css_err(s, &format!("pseudo-element :{n} is not supported")));
                    } else {
                        return Err(css_err(
                            s,
                            &format!("unsupported pseudo-class :{n}: only :root is supported"),
                        ));
                    }
                    any = true;
                }
                b'>' | b'+' | b'~' => break,
                ch if ch.is_ascii_whitespace() => break,
                _ => {
                    if c.root
                        || c.tag.is_some()
                        || !c.classes.is_empty()
                        || c.id.is_some()
                        || !c.attrs.is_empty()
                    {
                        return Err(css_err(
                            s,
                            "a type selector must come first in a compound selector",
                        ));
                    }
                    let n =
                        scan_ident(b, &mut i).ok_or_else(|| css_err(s, "expected a type name"))?;
                    c.tag = Some(n.to_ascii_lowercase());
                    specificity.2 += 1;
                    any = true;
                }
            }
        }
        if !any {
            return Err(css_err(s, "expected a selector"));
        }
        parts.push((pending, c));

        let saw_ws = skip_ws(b, &mut i);
        if i >= b.len() {
            break;
        }
        match b[i] {
            b'>' => {
                i += 1;
                pending = Combinator::Child;
            }
            b'+' => {
                i += 1;
                pending = Combinator::NextSibling;
            }
            b'~' => {
                i += 1;
                pending = Combinator::LaterSibling;
            }
            _ => {
                if !saw_ws {
                    return Err(css_err(s, "expected a combinator between selectors"));
                }
                pending = Combinator::Descendant;
            }
        }
        skip_ws(b, &mut i);
        if i >= b.len() {
            return Err(css_err(s, "dangling combinator at the end of the selector"));
        }
    }

    let scene_scope = parts.last().map(|(_, c)| c.root).unwrap_or(false);
    Ok(ComplexSelector {
        parts,
        specificity,
        scene_scope,
        text: s.to_string(),
    })
}

/// 遍历时为选择器匹配维护的位置。`stack` 从根元素到当前元素，
/// `stack[d].el` 是深度 `d` 的元素，`stack[d].index` 是它在其父节点中的下标。
#[derive(Clone, Copy)]
pub(crate) struct Frame<'a> {
    pub(crate) el: &'a El,
    pub(crate) index: usize,
}

impl<'a> Frame<'a> {
    pub(crate) fn new(el: &'a El, index: usize) -> Self {
        Frame { el, index }
    }
}

/// 匹配过程中的任意节点：可能不在 `stack` 上（兄弟节点），所以自带 `el`。
#[derive(Clone, Copy)]
pub(crate) struct Node<'a> {
    pub(crate) el: &'a El,
    pub(crate) depth: usize,
    pub(crate) index: usize,
}

pub(crate) fn sel_matches<'a>(sel: &ComplexSelector, stack: &[Frame<'a>]) -> bool {
    let Some(last) = stack.last() else {
        return false;
    };
    let cur = Node {
        el: last.el,
        depth: stack.len() - 1,
        index: last.index,
    };
    match_from(sel, stack, sel.parts.len() - 1, &cur)
}

/// 从右向左匹配：`parts[i]` 已经落在 `n` 上，再向左找 `parts[i-1]`。
pub(crate) fn match_from<'a>(
    sel: &ComplexSelector,
    stack: &[Frame<'a>],
    i: usize,
    n: &Node<'a>,
) -> bool {
    if !compound_matches(&sel.parts[i].1, n.el, n.depth) {
        return false;
    }
    if i == 0 {
        return true;
    }
    let target = i - 1;
    match sel.parts[i].0 {
        Combinator::Child => {
            parent_node(n, stack).is_some_and(|p| match_from(sel, stack, target, &p))
        }
        Combinator::Descendant => (0..n.depth).rev().any(|d| {
            let a = Node {
                el: stack[d].el,
                depth: d,
                index: stack[d].index,
            };
            match_from(sel, stack, target, &a)
        }),
        Combinator::NextSibling => {
            prev_sibling(n, stack).is_some_and(|p| match_from(sel, stack, target, &p))
        }
        Combinator::LaterSibling => prev_siblings(n, stack)
            .into_iter()
            .any(|p| match_from(sel, stack, target, &p)),
    }
}

pub(crate) fn parent_node<'a>(n: &Node<'a>, stack: &[Frame<'a>]) -> Option<Node<'a>> {
    if n.depth == 0 {
        return None;
    }
    Some(Node {
        el: stack[n.depth - 1].el,
        depth: n.depth - 1,
        index: stack[n.depth - 1].index,
    })
}

pub(crate) fn prev_sibling<'a>(n: &Node<'a>, stack: &[Frame<'a>]) -> Option<Node<'a>> {
    if n.depth == 0 || n.index == 0 {
        return None;
    }
    let parent = stack[n.depth - 1].el;
    Some(Node {
        el: &parent.children[n.index - 1],
        depth: n.depth,
        index: n.index - 1,
    })
}

pub(crate) fn prev_siblings<'a>(n: &Node<'a>, stack: &[Frame<'a>]) -> Vec<Node<'a>> {
    if n.depth == 0 {
        return Vec::new();
    }
    let parent = stack[n.depth - 1].el;
    (0..n.index)
        .rev()
        .map(|k| Node {
            el: &parent.children[k],
            depth: n.depth,
            index: k,
        })
        .collect()
}

pub(crate) fn compound_matches(c: &Compound, el: &El, depth: usize) -> bool {
    if c.root && depth != 0 {
        return false;
    }
    if let Some(tag) = &c.tag {
        if !el.tag.eq_ignore_ascii_case(tag) {
            return false;
        }
    }
    if let Some(id) = &c.id {
        match el.props.get("id").and_then(prop_scalar) {
            Some(got) if got == *id => {}
            _ => return false,
        }
    }
    if !c.classes.is_empty() {
        let mine = el_classes(el);
        for cl in &c.classes {
            if !mine.iter().any(|m| m == cl) {
                return false;
            }
        }
    }
    for (key, want) in &c.attrs {
        match el.props.get(key) {
            None | Some(Value::Null) => return false,
            Some(v) => {
                if let Some(want) = want {
                    match prop_scalar(v) {
                        Some(got) if got == *want => {}
                        _ => return false,
                    }
                }
            }
        }
    }
    true
}

pub(crate) fn el_classes(el: &El) -> Vec<String> {
    let raw = el.props.get("class").or_else(|| el.props.get("className"));
    match raw {
        Some(Value::String(s)) => s.split_whitespace().map(str::to_string).collect(),
        _ => Vec::new(),
    }
}

/// 属性选择器里把标量属性变成可比较的字符串。
pub(crate) fn prop_scalar(v: &Value) -> Option<String> {
    match v {
        Value::String(s) => Some(s.clone()),
        Value::Bool(b) => Some(if *b { "true" } else { "false" }.to_string()),
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                Some(i.to_string())
            } else if let Some(u) = n.as_u64() {
                Some(u.to_string())
            } else {
                let f = n.as_f64()?;
                if f.fract() == 0.0 && f.abs() < 1e15 {
                    Some(format!("{}", f as i64))
                } else {
                    Some(format!("{f}"))
                }
            }
        }
        _ => None,
    }
}

pub(crate) fn parse_css(src: &str) -> Result<Vec<StyleRule>, String> {
    let ss = lightningcss::stylesheet::StyleSheet::parse(
        src,
        lightningcss::stylesheet::ParserOptions::default(),
    )
    .map_err(|e| css_err("", &format!("syntax error: {e}")))?;
    let mut rules = Vec::new();
    for rule in &ss.rules.0 {
        let lightningcss::rules::CssRule::Style(style) = rule else {
            let kind = match rule {
                lightningcss::rules::CssRule::Media(_) => "@media",
                lightningcss::rules::CssRule::Import(_) => "@import",
                lightningcss::rules::CssRule::Keyframes(_) => "@keyframes",
                lightningcss::rules::CssRule::Supports(_) => "@supports",
                lightningcss::rules::CssRule::Nesting(_) => "a nested rule",
                lightningcss::rules::CssRule::FontFace(_) => "@font-face",
                _ => "an at-rule",
            };
            return Err(css_err(
                kind,
                "not supported: only plain style rules are supported here",
            ));
        };
        let sel_text = style.selectors.to_string();
        if !style.rules.0.is_empty() {
            return Err(css_err(
                &sel_text,
                "CSS nesting is not supported: only flat style rules are supported",
            ));
        }
        let mut decls = Vec::new();
        for (list, important) in [
            (&style.declarations.declarations, false),
            (&style.declarations.important_declarations, true),
        ] {
            for d in list {
                let name = d.property_id().name().to_string();
                // 序列化失败不能静默丢（docs/css-conformance.md §7）。
                let value = d
                    .value_to_css_string(lightningcss::printer::PrinterOptions::default())
                    .map_err(|e| css_err(&sel_text, &format!("cannot serialize `{name}`: {e}")))?;
                decls.push(StyleDecl {
                    name,
                    value,
                    important,
                });
            }
        }
        for part in split_selector_list(&sel_text) {
            let sel = parse_selector(&part)?;
            rules.push(StyleRule {
                sel,
                decls: decls.clone(),
            });
        }
    }
    Ok(rules)
}

/// 按顶层逗号切分选择器列表：属性/函数里的逗号（`[class="a,b"]`、`rgb(1,2,3)`）不算。
pub(crate) fn split_selector_list(s: &str) -> Vec<String> {
    let b = s.as_bytes();
    let mut out = Vec::new();
    let mut start = 0usize;
    let mut quote = 0u8;
    let mut depth = 0usize;
    for (i, &c) in b.iter().enumerate() {
        if quote != 0 {
            if c == quote && b.get(i.wrapping_sub(1)) != Some(&b'\\') {
                quote = 0;
            }
            continue;
        }
        match c {
            b'"' | b'\'' => quote = c,
            b'[' | b'(' => depth += 1,
            b']' | b')' => depth = depth.saturating_sub(1),
            b',' if depth == 0 => {
                out.push(s[start..i].trim().to_string());
                start = i + 1;
            }
            _ => {}
        }
    }
    out.push(s[start..].trim().to_string());
    out
}

// -- 级联 --
//
// 层级由低到高：继承 < 样式表 < 内联 prop < 样式表 !important < 内联 !important。
// 同一层内按特异性、再按源码顺序。
pub(crate) fn cascade_rank(important: bool, inline: bool) -> u8 {
    match (important, inline) {
        (false, false) => 1,
        (false, true) => 2,
        (true, false) => 3,
        (true, true) => 4,
    }
}

pub(crate) enum PendingValue {
    Css(String),
    Json(Value),
}

pub(crate) struct PendingDecl {
    pub(crate) rank: u8,
    pub(crate) specificity: Specificity,
    pub(crate) order: usize,
    pub(crate) key: String,
    /// 选择器原文，只用于错误信息。
    pub(crate) source: String,
    pub(crate) value: PendingValue,
}

// -- 值 --

pub(crate) const CSS_UNITS: &[&str] = &[
    "px", "em", "rem", "ex", "ch", "vh", "vw", "vmin", "vmax", "pt", "pc", "in", "cm", "mm", "q",
    "deg", "rad", "grad", "turn", "s", "ms", "hz", "khz", "dpi", "dpcm", "dppx", "fr",
];

pub(crate) fn css_number(raw: &str) -> Result<f64, String> {
    let v = raw.trim();
    for f in ["calc(", "min(", "max(", "clamp("] {
        if v.starts_with(f) {
            return Err(format!(
                "`{v}` is not supported: material values must be plain numbers"
            ));
        }
    }
    if let Some(p) = v.strip_suffix('%') {
        let n = p
            .trim()
            .parse::<f64>()
            .map_err(|_| format!("`{v}` is not a percentage"))?;
        return Ok(n / 100.0);
    }
    for u in CSS_UNITS {
        if let Some(p) = v.strip_suffix(u) {
            if p.chars()
                .last()
                .map(|c| c.is_ascii_digit() || c == '.')
                .unwrap_or(false)
            {
                return Err(format!(
                    "`{v}` has a unit: material values are unitless (write a plain number)"
                ));
            }
        }
    }
    v.parse::<f64>()
        .map_err(|_| format!("`{v}` is not a number"))
}

pub(crate) fn css_bool(raw: &str) -> Result<bool, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "true" | "yes" | "on" | "1" => Ok(true),
        "false" | "no" | "off" | "0" => Ok(false),
        v => Err(format!("`{v}` is not a boolean")),
    }
}

/// 颜色 → `(0xRRGGBB, alpha ∈ [0,1])`。named / hex / rgb / hsl / hwb 全都由
/// lightningcss 解析，所以判据是 CSS 本身而不是我们自己的正则。
pub(crate) fn css_color(raw: &str) -> Result<(f64, f64), String> {
    use lightningcss::traits::Parse;
    let text = raw.trim();
    let color = lightningcss::values::color::CssColor::parse_string(text)
        .map_err(|_| format!("`{text}` is not a valid color"))?;
    match color {
        lightningcss::values::color::CssColor::RGBA(rgba) => {
            let hex =
                (u32::from(rgba.red) << 16) | (u32::from(rgba.green) << 8) | u32::from(rgba.blue);
            Ok((f64::from(hex), f64::from(rgba.alpha) / 255.0))
        }
        lightningcss::values::color::CssColor::CurrentColor => {
            Err("`currentcolor` is not supported here".to_string())
        }
        _ => Err(format!("unsupported color space in `{text}`")),
    }
}

pub(crate) fn css_value(name: &str, raw: &str) -> Result<ArgValue, String> {
    match name {
        "color" | "emissive" => {
            let (hex, _) = css_color(raw)?;
            Ok(ArgValue::Num(hex))
        }
        "map" => {
            let v = raw.trim();
            let p = v
                .strip_prefix("url(")
                .and_then(|s| s.strip_suffix(')'))
                .unwrap_or(v);
            Ok(ArgValue::Str(
                p.trim().trim_matches('"').trim_matches('\'').to_string(),
            ))
        }
        "unlit" => Ok(ArgValue::Bool(css_bool(raw)?)),
        _ => css_number(raw).map(ArgValue::Num),
    }
}

/// `var(--x)` / `var(--x, fallback)` 文本替换。未定义且没有 fallback → 报错。
pub(crate) fn var_substitute(
    raw: &str,
    customs: &HashMap<String, String>,
) -> Result<String, String> {
    if !raw.contains("var(") {
        return Ok(raw.to_string());
    }
    let mut out = String::new();
    let mut rest = raw;
    while let Some(p) = rest.find("var(") {
        out.push_str(&rest[..p]);
        let after = &rest[p + 4..];
        let bytes = after.as_bytes();
        let mut depth = 1usize;
        let mut j = 0usize;
        while j < bytes.len() {
            match bytes[j] {
                b'(' => depth += 1,
                b')' => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                _ => {}
            }
            j += 1;
        }
        if j >= bytes.len() {
            return Err("unterminated var()".to_string());
        }
        let inner = &after[..j];
        // fallback = 顶层第一个逗号之后的内容
        let mut k = 0usize;
        let mut d2 = 0usize;
        let ib = inner.as_bytes();
        while k < ib.len() {
            match ib[k] {
                b'(' => d2 += 1,
                b')' => d2 = d2.saturating_sub(1),
                b',' if d2 == 0 => break,
                _ => {}
            }
            k += 1;
        }
        let (name, fallback) = if k < ib.len() {
            (&inner[..k], Some(&inner[k + 1..]))
        } else {
            (inner, None)
        };
        let name = name.trim();
        if !name.starts_with("--") {
            return Err(format!("var({name}) must be a custom property (--name)"));
        }
        let value = match customs.get(name) {
            Some(v) => v.clone(),
            None => match fallback {
                Some(f) => var_substitute(f.trim(), customs)?,
                None => return Err(format!("var({name}) is not defined")),
            },
        };
        out.push_str(&value);
        rest = &after[j + 1..];
    }
    out.push_str(rest);
    Ok(out)
}
