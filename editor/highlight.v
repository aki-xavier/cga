module main

// highlight.v — CGS syntax highlighting (port of editor/src/highlight.rs).
//
// A small hand-written lexer that classifies each span into a semantic class.
// CGS documents are tiny, so we re-scan the whole text on every edit.

// HighlightClass is one semantic syntax class.
pub enum HighlightClass {
	comment
	keyword
	typ
	function
	constant
	number
	operator
	punctuation
	plain
}

// HighlightSpan is one classified byte span.
pub struct HighlightSpan {
pub:
	start int
	end   int
	class HighlightClass
}

// highlight_tokenize scans `text` into ordered, non-overlapping spans.
pub fn highlight_tokenize(text string) []HighlightSpan {
	b := text.bytes()
	n := b.len
	mut out := []HighlightSpan{}
	mut i := 0
	for i < n {
		c := b[i]
		// line comment
		if c == `/` && i + 1 < n && b[i + 1] == `/` {
			s := i
			for i < n && b[i] != `\n` {
				i++
			}
			out << HighlightSpan{
				start: s
				end:   i
				class: .comment
			}
			continue
		}
		if c.is_space() {
			i++
			continue
		}
		// 0x hex colour
		if c == `0` && i + 1 < n && (b[i + 1] == `x` || b[i + 1] == `X`) {
			s := i
			i += 2
			for i < n && b[i].is_hex_digit() {
				i++
			}
			out << HighlightSpan{
				start: s
				end:   i
				class: .number
			}
			continue
		}
		// number (float / exponent)
		if c.is_digit() {
			s := i
			for i < n && (b[i].is_digit() || b[i] == `.`) {
				i++
			}
			if i < n && (b[i] == `e` || b[i] == `E`) {
				mut j := i + 1
				if j < n && (b[j] == `+` || b[j] == `-`) {
					j++
				}
				mut k := j
				for k < n && b[k].is_digit() {
					k++
				}
				if k > j {
					i = k
				}
			}
			out << HighlightSpan{
				start: s
				end:   i
				class: .number
			}
			continue
		}
		// identifier / keyword
		if c.is_letter() || c == `_` {
			s := i
			for i < n && (b[i].is_alnum() || b[i] == `_`) {
				i++
			}
			if cls := classify_word(text[s..i]) {
				out << HighlightSpan{
					start: s
					end:   i
					class: cls
				}
			}
			continue
		}
		// two-character operators
		if i + 1 < n {
			two := text[i..i + 2]
			if two in ['==', '!=', '<=', '>=', '&&', '||'] {
				out << HighlightSpan{
					start: i
					end:   i + 2
					class: .operator
				}
				i += 2
				continue
			}
		}
		// single-character operators
		if c in [`+`, `-`, `*`, `/`, `%`, `<`, `>`, `!`, `:`] {
			out << HighlightSpan{
				start: i
				end:   i + 1
				class: .operator
			}
			i++
			continue
		}
		// punctuation
		if c in [`[`, `]`, `{`, `}`, `(`, `)`, `,`, `;`, `=`] {
			out << HighlightSpan{
				start: i
				end:   i + 1
				class: .punctuation
			}
			i++
			continue
		}
		i++
	}
	return out
}

// classify_word maps an identifier to a semantic class (none = plain variable).
pub fn classify_word(word string) ?HighlightClass {
	match word {
		'module', 'for', 'if', 'else', 'echo', 'union' {
			return .keyword
		}
		'true', 'false', 'pi' {
			return .constant
		}
		'sphere', 'plane', 'cylinder', 'box', 'circle', 'translate', 'rotate', 'material',
		'splats', 'directional_light', 'point_light', 'ambient_light', 'background', 'camera' {
			return .typ
		}
		'abs', 'sign', 'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2', 'sqrt', 'exp', 'ln',
		'log', 'floor', 'ceil', 'round', 'pow', 'min', 'max', 'len', 'norm', 'cross' {
			return .function
		}
		else {
			return none
		}
	}
}

// highlight_color maps a semantic class to a 0xRRGGBB colour.
pub fn highlight_color(class HighlightClass) int {
	return match class {
		.comment { 0x6a737d }
		.keyword { 0xc678dd }
		.typ { 0x61afef }
		.function { 0xe5c07b }
		.constant { 0xd19a66 }
		.number { 0xd19a66 }
		.operator { 0x56b6c2 }
		.punctuation { 0x9da5b4 }
		.plain { 0xdfe2ea }
	}
}

// FoldRange is a brace-fold region (line numbers).
pub struct FoldRange {
pub:
	start int
	end   int
}

// brace_fold_ranges finds `{ … }` fold regions (comments skipped).
pub fn brace_fold_ranges(text string) []FoldRange {
	mut starts := []int{}
	mut ranges := []FoldRange{}
	for line_number, line in text.split_into_lines() {
		b := line.bytes()
		mut i := 0
		for i < b.len {
			if b[i] == `/` && i + 1 < b.len && b[i + 1] == `/` {
				break
			}
			if b[i] == `{` {
				starts << line_number
			} else if b[i] == `}` {
				if starts.len > 0 {
					start_line := starts.pop()
					if start_line < line_number {
						ranges << FoldRange{
							start: start_line
							end:   line_number
						}
					}
				}
			}
			i++
		}
	}
	return ranges
}
