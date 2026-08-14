//! CGS 数值参数提取: 从 .cgs 文本中找出可拖拽调参的数值字面量。
//!
//! 扫描规则 (与 `cga/scene_lang/lexer.py` 对齐): 命名参数 `name=<数>` 与
//! 按位参数 `<数>`、向量元素 `[x,y,z]`、range `[初:步:止]` 都会被标注为
//! `<调用名>.<参数名>[下标]` 之类的标签; 十六进制色值不纳入拖拽。

use std::collections::HashMap;
use std::ops::Range;

/// 一个可拖拽调参的数值字面量 (纯数据, 不含 UI 实体)。
pub struct RawParam {
    /// 语义标签, 例如 `sphere.r` / `translate.t[1]` / `for.i.start`。
    pub label: String,
    /// 同名标签中的序号 (用于回写时精确定位)。
    pub ordinal: usize,
    /// 数值字面量在原文中的 byte 区间。
    pub range: Range<usize>,
    pub value: f64,
    pub min: f32,
    pub max: f32,
    pub step: f32,
}

#[derive(PartialEq)]
enum K {
    Ident(String),
    Num(f64),
    Hex,
    Sym(char),
}

struct T {
    k: K,
    s: usize,
    e: usize,
}

fn tokenize(text: &str) -> Vec<T> {
    let b = text.as_bytes();
    let n = b.len();
    let mut toks = Vec::new();
    let mut i = 0;
    while i < n {
        let c = b[i];
        if c == b'/' && i + 1 < n && b[i + 1] == b'/' {
            while i < n && b[i] != b'\n' {
                i += 1;
            }
            continue;
        }
        if c.is_ascii_whitespace() {
            i += 1;
            continue;
        }
        if c == b'0' && i + 1 < n && (b[i + 1] == b'x' || b[i + 1] == b'X') {
            let s = i;
            i += 2;
            while i < n && b[i].is_ascii_hexdigit() {
                i += 1;
            }
            toks.push(T { k: K::Hex, s, e: i });
            continue;
        }
        if c.is_ascii_digit() {
            let s = i;
            while i < n && (b[i].is_ascii_digit() || b[i] == b'.') {
                i += 1;
            }
            if i < n && (b[i] == b'e' || b[i] == b'E') {
                let mut j = i + 1;
                if j < n && (b[j] == b'+' || b[j] == b'-') {
                    j += 1;
                }
                let mut k = j;
                while k < n && b[k].is_ascii_digit() {
                    k += 1;
                }
                if k > j {
                    i = k;
                }
            }
            let value = text[s..i].parse::<f64>().unwrap_or(0.0);
            toks.push(T { k: K::Num(value), s, e: i });
            continue;
        }
        if c.is_ascii_alphabetic() || c == b'_' {
            let s = i;
            while i < n && (b[i].is_ascii_alphanumeric() || b[i] == b'_') {
                i += 1;
            }
            toks.push(T {
                k: K::Ident(text[s..i].to_string()),
                s,
                e: i,
            });
            continue;
        }
        if matches!(
            c,
            b'(' | b')' | b'[' | b']' | b',' | b'=' | b';' | b':'
        ) {
            toks.push(T {
                k: K::Sym(c as char),
                s: i,
                e: i + 1,
            });
            i += 1;
            continue;
        }
        // 运算符等与标签无关的符号: 跳过
        i += 1;
    }
    toks
}

struct Frame {
    name: String,
    positional: usize,
    outer_arg: Option<String>,
}

struct List {
    index: usize,
    is_range: bool,
    colons: usize,
    ordinal: usize,
}

/// 探测 `[` 是否为 range, 并返回顶层 `:` 数量。
fn list_is_range(toks: &[T], open: usize) -> (bool, usize) {
    let mut depth = 0usize;
    let mut colons = 0usize;
    for j in open + 1..toks.len() {
        match toks[j].k {
            K::Sym('[') => depth += 1,
            K::Sym(']') => {
                if depth == 0 {
                    break;
                }
                depth -= 1;
            }
            K::Sym(':') if depth == 0 => colons += 1,
            _ => {}
        }
    }
    (colons > 0, colons)
}

/// 按位参数 → 参数名 (与 Python 端 SIGNATURES 一致)。
fn positional_name(call: &str, index: usize) -> String {
    let sig: &[&str] = match call {
        "sphere" => &["r"],
        "plane" => &["n", "d"],
        "cylinder" => &["r", "h"],
        "box" => &["s"],
        "circle" => &["r"],
        "translate" => &["t"],
        "rotate" => &["axis", "angle"],
        "directional_light" => &["direction", "intensity", "color"],
        "point_light" => &["position", "intensity", "color"],
        "ambient_light" => &["intensity", "color"],
        "background" => &["color"],
        _ => &[],
    };
    sig.get(index)
        .map(|s| s.to_string())
        .unwrap_or_else(|| format!("_{index}"))
}

/// 依据标签语义给出滑块范围 (保证当前值附近可拖)。
fn range_for(label: &str, value: f64) -> (f32, f32, f32) {
    let leaf = label.rsplit('.').next().unwrap_or(label);
    let leaf = leaf.split('[').next().unwrap_or(leaf);
    match leaf {
        "angle" => (0.0, 6.2832, 0.01),
        "fov" => (1.0, 120.0, 1.0),
        "roughness" | "metalness" | "opacity" | "emissive" => (0.0, 1.0, 0.01),
        "ior" => (0.5, 3.0, 0.01),
        "absorption" => (0.0, 2.0, 0.01),
        "intensity" => (0.0, 2.0, 0.01),
        "aspect" => (0.5, 2.5, 0.01),
        // 坐标 / 尺寸类: 围绕当前值 ±5
        _ => {
            let v = value as f32;
            (v - 5.0, v + 5.0, 0.1)
        }
    }
}

/// 从 CGS 文本提取所有可拖拽数值参数 (按出现顺序)。
pub fn extract_params(text: &str) -> Vec<RawParam> {
    let toks = tokenize(text);
    let mut frames: Vec<Option<Frame>> = Vec::new();
    let mut arg: Option<String> = None;
    let mut lists: Vec<List> = Vec::new();
    let mut ordinals: HashMap<String, usize> = HashMap::new();
    let mut out: Vec<RawParam> = Vec::new();

    let mut i = 0;
    while i < toks.len() {
        let t = &toks[i];
        match &t.k {
            K::Sym('(') => {
                let is_call = i > 0 && matches!(&toks[i - 1].k, K::Ident(_));
                if is_call {
                    if let K::Ident(name) = &toks[i - 1].k {
                        let nm = name.clone();
                        let first = positional_name(&nm, 0);
                        frames.push(Some(Frame {
                            name: nm.clone(),
                            positional: 0,
                            outer_arg: arg.clone(),
                        }));
                        arg = Some(format!("{nm}.{first}"));
                    }
                } else {
                    frames.push(None);
                }
            }
            K::Sym(')') => {
                if let Some(top) = frames.pop() {
                    if let Some(f) = top {
                        arg = f.outer_arg;
                    }
                }
            }
            K::Sym('=') => {
                if i > 0 {
                    if let K::Ident(name) = &toks[i - 1].k {
                        arg = Some(match frames.last() {
                            Some(Some(f)) => format!("{}.{}", f.name, name),
                            _ => name.clone(),
                        });
                    }
                }
            }
            K::Sym('[') => {
                let (is_range, colons) = list_is_range(&toks, i);
                lists.push(List {
                    index: 0,
                    is_range,
                    colons,
                    ordinal: 0,
                });
            }
            K::Sym(']') => {
                lists.pop();
            }
            K::Sym(',') => {
                if let Some(l) = lists.last_mut() {
                    if !l.is_range {
                        l.index += 1;
                    }
                } else if let Some(Some(f)) = frames.last_mut() {
                    f.positional += 1;
                    arg = Some(format!(
                        "{}.{}",
                        f.name,
                        positional_name(&f.name, f.positional)
                    ));
                }
            }
            K::Sym(':') => {
                // range 已在 list_is_range 预探测, 这里无需处理
            }
            K::Sym(';') => {
                arg = None;
            }
            K::Num(value) => {
                if let Some(base) = arg.clone() {
                    let label = if let Some(l) = lists.last() {
                        if l.is_range {
                            let name = match (l.ordinal, l.colons) {
                                (0, _) => "start",
                                (1, 1) => "stop",
                                (1, _) => "step",
                                _ => "elem",
                            };
                            format!("{base}.{name}")
                        } else {
                            format!("{base}[{}]", l.index)
                        }
                    } else {
                        base
                    };
                    let ordinal = *ordinals.get(&label).unwrap_or(&0);
                    ordinals.insert(label.clone(), ordinal + 1);
                    let (min, max, step) = range_for(&label, *value);
                    out.push(RawParam {
                        label,
                        ordinal,
                        range: t.s..t.e,
                        value: *value,
                        min,
                        max,
                        step,
                    });
                    if let Some(l) = lists.last_mut() {
                        if l.is_range {
                            l.ordinal += 1;
                        }
                    }
                }
            }
            K::Hex => {
                // 色值不纳入拖拽
            }
            K::Ident(_) => {
                // 通过 '(' / '=' 的前视处理
            }
            _ => {}
        }
        i += 1;
    }
    out
}

/// 把数值格式化为简洁字符串 (去掉多余的 0 / 小数点)。
pub fn format_number(v: f64) -> String {
    let s = format!("{v:.4}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() || s == "-0" || s == "-" {
        "0".to_string()
    } else {
        s.to_string()
    }
}

/// 用 `repl` 替换 `text[range]` (range 需为字符边界, 由 tokenizer 保证)。
pub fn replace_range(text: &str, range: Range<usize>, repl: &str) -> String {
    let mut s = String::with_capacity(text.len() - range.len() + repl.len());
    s.push_str(&text[..range.start]);
    s.push_str(repl);
    s.push_str(&text[range.end..]);
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn named_scalar() {
        let ps = extract_params("sphere(r=1);");
        assert_eq!(ps.len(), 1);
        assert_eq!(ps[0].label, "sphere.r");
        assert_eq!(ps[0].value, 1.0);
        assert_eq!(
            replace_range("sphere(r=1);", ps[0].range.clone(), &format_number(2.5)),
            "sphere(r=2.5);"
        );
    }

    #[test]
    fn vector_elements() {
        let ps = extract_params("translate([0, 1, 0]) sphere(r=0.5);");
        let labels: Vec<_> = ps.iter().map(|p| p.label.as_str()).collect();
        assert_eq!(
            labels,
            vec!["translate.t[0]", "translate.t[1]", "translate.t[2]", "sphere.r"]
        );
    }

    #[test]
    fn skips_hex_colors() {
        let ps = extract_params("background(color=0x87CEEB);");
        assert!(ps.is_empty());
    }

    #[test]
    fn range_loop_bounds() {
        let ps = extract_params("for (i = [0:2]) sphere(r=1);");
        let labels: Vec<_> = ps.iter().map(|p| p.label.as_str()).collect();
        assert_eq!(labels, vec!["for.i.start", "for.i.stop", "sphere.r"]);
    }

    #[test]
    fn material_named_args() {
        let ps = extract_params("material(color=0xB0B0B0, roughness=0.7, metalness=0.25) sphere(r=1);");
        let labels: Vec<_> = ps.iter().map(|p| p.label.as_str()).collect();
        assert_eq!(
            labels,
            vec!["material.roughness", "material.metalness", "sphere.r"]
        );
    }

    #[test]
    fn format_number_trims() {
        assert_eq!(format_number(1.0), "1");
        assert_eq!(format_number(0.7), "0.7");
        assert_eq!(format_number(6.1999998), "6.2");
        assert_eq!(format_number(-0.0), "0");
    }
}

