#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HighlightClass {
    Comment,
    Keyword,
    Typ,
    Function,
    Constant,
    Number,
    Operator,
    Punctuation,
    Plain,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HighlightSpan {
    pub start: usize,
    pub end: usize,
    pub class: HighlightClass,
}

pub fn highlight_tokenize(text: &str) -> Vec<HighlightSpan> {
    let b = text.as_bytes();
    let n = b.len();
    let mut out: Vec<HighlightSpan> = Vec::new();
    let mut i = 0;
    while i < n {
        let c = b[i];

        if c == b'/' && i + 1 < n && b[i + 1] == b'/' {
            let s = i;
            while i < n && b[i] != b'\n' {
                i += 1;
            }
            out.push(HighlightSpan {
                start: s,
                end: i,
                class: HighlightClass::Comment,
            });
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
            out.push(HighlightSpan {
                start: s,
                end: i,
                class: HighlightClass::Number,
            });
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
            out.push(HighlightSpan {
                start: s,
                end: i,
                class: HighlightClass::Number,
            });
            continue;
        }

        if c.is_ascii_alphabetic() || c == b'_' {
            let s = i;
            while i < n && (b[i].is_ascii_alphanumeric() || b[i] == b'_') {
                i += 1;
            }
            if let Some(cls) = classify_word(&text[s..i]) {
                out.push(HighlightSpan {
                    start: s,
                    end: i,
                    class: cls,
                });
            }
            continue;
        }

        if i + 1 < n {
            let two = (b[i], b[i + 1]);
            if matches!(
                two,
                (b'=', b'=')
                    | (b'!', b'=')
                    | (b'<', b'=')
                    | (b'>', b'=')
                    | (b'&', b'&')
                    | (b'|', b'|')
            ) {
                out.push(HighlightSpan {
                    start: i,
                    end: i + 2,
                    class: HighlightClass::Operator,
                });
                i += 2;
                continue;
            }
        }

        if matches!(
            c,
            b'+' | b'-' | b'*' | b'/' | b'%' | b'<' | b'>' | b'!' | b':'
        ) {
            out.push(HighlightSpan {
                start: i,
                end: i + 1,
                class: HighlightClass::Operator,
            });
            i += 1;
            continue;
        }

        if matches!(
            c,
            b'[' | b']' | b'{' | b'}' | b'(' | b')' | b',' | b';' | b'='
        ) {
            out.push(HighlightSpan {
                start: i,
                end: i + 1,
                class: HighlightClass::Punctuation,
            });
            i += 1;
            continue;
        }
        i += 1;
    }
    out
}

pub fn classify_word(word: &str) -> Option<HighlightClass> {
    match word {
        "module" | "for" | "if" | "else" | "echo" | "union" => Some(HighlightClass::Keyword),
        "true" | "false" | "pi" => Some(HighlightClass::Constant),
        "sphere" | "plane" | "cylinder" | "box" | "circle" | "translate" | "rotate"
        | "material" | "directional_light" | "point_light" | "ambient_light" | "background"
        | "camera" => Some(HighlightClass::Typ),
        "abs" | "sign" | "sin" | "cos" | "tan" | "asin" | "acos" | "atan" | "atan2" | "sqrt"
        | "exp" | "ln" | "log" | "floor" | "ceil" | "round" | "pow" | "min" | "max" | "len"
        | "norm" | "cross" => Some(HighlightClass::Function),
        _ => None,
    }
}

pub fn highlight_color(class: HighlightClass) -> i32 {
    match class {
        HighlightClass::Comment => 0x6a737d,
        HighlightClass::Keyword => 0xc678dd,
        HighlightClass::Typ => 0x61afef,
        HighlightClass::Function => 0xe5c07b,
        HighlightClass::Constant => 0xd19a66,
        HighlightClass::Number => 0xd19a66,
        HighlightClass::Operator => 0x56b6c2,
        HighlightClass::Punctuation => 0x9da5b4,
        HighlightClass::Plain => 0xdfe2ea,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FoldRange {
    pub start: usize,
    pub end: usize,
}

pub fn brace_fold_ranges(text: &str) -> Vec<FoldRange> {
    let mut starts: Vec<usize> = Vec::new();
    let mut ranges: Vec<FoldRange> = Vec::new();
    for (line_number, line) in text.lines().enumerate() {
        let b = line.as_bytes();
        let mut i = 0;
        while i < b.len() {
            if b[i] == b'/' && i + 1 < b.len() && b[i + 1] == b'/' {
                break;
            }
            if b[i] == b'{' {
                starts.push(line_number);
            } else if b[i] == b'}' && !starts.is_empty() {
                let start_line = starts.pop().unwrap();
                if start_line < line_number {
                    ranges.push(FoldRange {
                        start: start_line,
                        end: line_number,
                    });
                }
            }
            i += 1;
        }
    }
    ranges
}

#[cfg(test)]
mod tests {
    use super::*;

    fn span_classes(text: &str) -> Vec<HighlightClass> {
        let mut out: Vec<HighlightClass> = Vec::new();
        for s in highlight_tokenize(text) {
            out.push(s.class);
        }
        out
    }

    #[test]
    fn test_highlight_keywords_and_types() {
        let spans = highlight_tokenize("for (i = [0:2]) sphere(r=1);");
        let classes = span_classes("for (i = [0:2]) sphere(r=1);");
        assert!(classes.contains(&HighlightClass::Keyword));
        assert!(classes.contains(&HighlightClass::Typ));
        assert!(classes.contains(&HighlightClass::Number));
        assert!(classes.contains(&HighlightClass::Punctuation));
        let _ = spans;
    }

    #[test]
    fn test_highlight_comment() {
        let spans = highlight_tokenize("// hello\nsphere(r=1);");
        assert!(!spans.is_empty());
        assert!(spans[0].class == HighlightClass::Comment);
        assert!(spans[0].start == 0);
        assert!(spans[0].end == 8);
    }

    #[test]
    fn test_highlight_hex_is_number() {
        let spans = highlight_tokenize("background(color=0x87CEEB);");
        let mut has_hex = false;
        for s in spans {
            if s.class == HighlightClass::Number && s.end - s.start == 8 {
                has_hex = true;
            }
        }
        assert!(has_hex);
    }

    #[test]
    fn test_highlight_functions_and_constants() {
        let spans = highlight_tokenize("translate([sin(pi), 0, 0]) sphere(r=1);");
        let classes = span_classes("translate([sin(pi), 0, 0]) sphere(r=1);");
        assert!(classes.contains(&HighlightClass::Function));
        assert!(classes.contains(&HighlightClass::Constant));
        let _ = spans;
    }

    #[test]
    fn test_highlight_classify_word() {
        assert!(
            classify_word("module").unwrap_or(HighlightClass::Plain) == HighlightClass::Keyword
        );
        assert!(classify_word("sphere").unwrap_or(HighlightClass::Plain) == HighlightClass::Typ);
        assert!(classify_word("sqrt").unwrap_or(HighlightClass::Plain) == HighlightClass::Function);
        assert!(classify_word("pi").unwrap_or(HighlightClass::Plain) == HighlightClass::Constant);
        assert!(classify_word("myvar").is_none());
    }

    #[test]
    fn test_highlight_braces() {
        let folds = brace_fold_ranges("module m() {\n  sphere(r=1);\n}\n");
        assert!(folds.len() == 1);
        assert!(folds[0].start == 0);
        assert!(folds[0].end == 2);
    }

    #[test]
    fn test_highlight_color_map() {
        assert!(highlight_color(HighlightClass::Keyword) == 0xc678dd);
        assert!(highlight_color(HighlightClass::Plain) == 0xdfe2ea);
    }
}
