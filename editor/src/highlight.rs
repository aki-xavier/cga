//! CGS 语法高亮: 手写词法分析器 + 语义着色主题。
//!
//! 实现 gpui-component 的 `InputHighlighter` / `HighlightStyleResolver` 两个
//! seam。CGS 文件都很小, 每次编辑整体重扫即可, 无需增量解析。

use std::ops::Range;

use gpui::{Context, HighlightStyle, SharedString, Window, rgb};
use gpui_component::input::{
    FoldRange, HighlightStyleResolver, InputBaseState, InputEdit, InputHighlighter, Rope,
};

/// CGS 语义高亮器。
pub struct CgsHighlighter {
    language: SharedString,
    highlights: Vec<(Range<usize>, &'static str)>,
    folds: Vec<FoldRange>,
}

impl CgsHighlighter {
    pub fn new() -> Self {
        Self {
            language: "cgs".into(),
            highlights: Vec::new(),
            folds: Vec::new(),
        }
    }
}

impl InputHighlighter for CgsHighlighter {
    fn language(&self) -> SharedString {
        self.language.clone()
    }

    fn update(
        &mut self,
        _edit: Option<InputEdit>,
        text: &Rope,
        folding: bool,
        _window: &mut Window,
        _cx: &mut Context<InputBaseState>,
    ) {
        let text = text.to_string();
        self.highlights = tokenize(&text);
        self.folds = if folding {
            brace_fold_ranges(&text)
        } else {
            Vec::new()
        };
    }

    fn styles(
        &self,
        range: &Range<usize>,
        resolver: &dyn HighlightStyleResolver,
    ) -> Vec<(Range<usize>, HighlightStyle)> {
        resolve_styles(&self.highlights, range, resolver)
    }

    fn fold_ranges(&self, _text: &Rope) -> Vec<FoldRange> {
        self.folds.clone()
    }

    fn fold_ranges_for_edit(&self, _range: Range<usize>, _text: &Rope) -> Vec<FoldRange> {
        self.folds.clone()
    }
}

/// 词法扫描, 产出有序、不重叠的语义区间 (byte 偏移)。
fn tokenize(text: &str) -> Vec<(Range<usize>, &'static str)> {
    let b = text.as_bytes();
    let n = b.len();
    let mut out = Vec::new();
    let mut i = 0;

    while i < n {
        let c = b[i];

        // 行注释
        if c == b'/' && i + 1 < n && b[i + 1] == b'/' {
            let s = i;
            while i < n && b[i] != b'\n' {
                i += 1;
            }
            out.push((s..i, "comment"));
            continue;
        }
        if c.is_ascii_whitespace() {
            i += 1;
            continue;
        }
        // 0x 十六进制色值
        if c == b'0' && i + 1 < n && (b[i + 1] == b'x' || b[i + 1] == b'X') {
            let s = i;
            i += 2;
            while i < n && b[i].is_ascii_hexdigit() {
                i += 1;
            }
            out.push((s..i, "number"));
            continue;
        }
        // 数字 (含浮点 / 指数)
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
            out.push((s..i, "number"));
            continue;
        }
        // 标识符 / 关键字
        if c.is_ascii_alphabetic() || c == b'_' {
            let s = i;
            while i < n && (b[i].is_ascii_alphanumeric() || b[i] == b'_') {
                i += 1;
            }
            if let Some(name) = classify_word(&text[s..i]) {
                out.push((s..i, name));
            }
            continue;
        }
        // 两字符运算符
        if i + 1 < n {
            let two = &text[i..i + 2];
            if two == "==" || two == "!=" || two == "<=" || two == ">=" || two == "&&" || two == "||"
            {
                out.push((i..i + 2, "operator"));
                i += 2;
                continue;
            }
        }
        // 单字符运算符
        if matches!(
            c,
            b'+' | b'-' | b'*' | b'/' | b'%' | b'<' | b'>' | b'!' | b':'
        ) {
            out.push((i..i + 1, "operator"));
            i += 1;
            continue;
        }
        // 标点
        if matches!(
            c,
            b'[' | b']' | b'{' | b'}' | b'(' | b')' | b',' | b';' | b'='
        ) {
            out.push((i..i + 1, "punctuation"));
            i += 1;
            continue;
        }
        // 未知字节 (如注释外的非 ASCII) 直接跳过
        i += 1;
    }
    out
}

/// 标识符 → 语义名。变量不单独着色 (保持默认前景色)。
fn classify_word(word: &str) -> Option<&'static str> {
    Some(match word {
        "module" | "for" | "if" | "else" | "echo" | "union" => "keyword",
        "true" | "false" | "pi" => "constant",
        "sphere" | "plane" | "cylinder" | "box" | "circle" | "translate" | "rotate"
        | "material" | "directional_light" | "point_light" | "ambient_light" | "background"
        | "camera" => "type",
        "abs" | "sign" | "sin" | "cos" | "tan" | "asin" | "acos" | "atan" | "atan2" | "sqrt"
        | "exp" | "ln" | "log" | "floor" | "ceil" | "round" | "pow" | "min" | "max" | "len"
        | "norm" | "cross" => "function",
        _ => return None,
    })
}

/// 把覆盖 `range` 的语义区间展开为无缝的样式 run (未着色区间用默认样式)。
fn resolve_styles(
    highlights: &[(Range<usize>, &'static str)],
    range: &Range<usize>,
    resolver: &dyn HighlightStyleResolver,
) -> Vec<(Range<usize>, HighlightStyle)> {
    let first = highlights.partition_point(|(h, _)| h.end <= range.start);
    let mut runs = Vec::new();
    let mut cursor = range.start;

    for (highlight_range, name) in &highlights[first..] {
        if highlight_range.start >= range.end {
            break;
        }
        let start = highlight_range.start.max(range.start);
        let end = highlight_range.end.min(range.end);
        if start >= end || end <= cursor {
            continue;
        }
        if cursor < start {
            runs.push((cursor..start, HighlightStyle::default()));
        }
        runs.push((start..end, resolver.style(name).unwrap_or_default()));
        cursor = end;
    }

    if cursor < range.end {
        runs.push((cursor..range.end, HighlightStyle::default()));
    }
    runs
}

/// 大括号折叠 (跳过 `//` 注释)。
fn brace_fold_ranges(text: &str) -> Vec<FoldRange> {
    let mut starts = Vec::new();
    let mut ranges = Vec::new();

    for (line_number, line) in text.lines().enumerate() {
        let b = line.as_bytes();
        let mut i = 0;
        while i < b.len() {
            if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'/' {
                break;
            }
            if b[i] == b'{' {
                starts.push(line_number);
            } else if b[i] == b'}' {
                if let Some(start_line) = starts.pop() {
                    if start_line < line_number {
                        ranges.push(FoldRange::new(start_line, line_number));
                    }
                }
            }
            i += 1;
        }
    }
    ranges
}

/// CGS 主题: 语义名 → 前景色。
pub struct CgsHighlightStyles;

impl HighlightStyleResolver for CgsHighlightStyles {
    fn style(&self, name: &str) -> Option<HighlightStyle> {
        let color = match name.split('.').next()? {
            "comment" => 0x6a737d,
            "keyword" => 0xc678dd,
            "type" => 0x61afef,
            "function" => 0xe5c07b,
            "constant" => 0xd19a66,
            "number" => 0xd19a66,
            "operator" => 0x56b6c2,
            "punctuation" => 0x9da5b4,
            _ => return None,
        };
        Some(HighlightStyle {
            color: Some(rgb(color).into()),
            ..Default::default()
        })
    }
}
