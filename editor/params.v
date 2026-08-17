module main

// params.v — CGS numeric-parameter extraction (port of editor/src/params.rs).
//
// Scans a .cgs document for draggable numeric literals: named args `name=<n>`,
// positional args `<n>`, vector elements `[x,y,z]` and range bounds `[a:b:c]`,
// labelling each as `<call>.<arg>[i]` / `<call>.<arg>.start|step|stop`.
// Hex colour literals (0x…) are skipped.

// RawParam is one draggable numeric literal (pure data, no UI).
pub struct RawParam {
pub:
	label   string // e.g. `sphere.r` / `translate.t[1]` / `for.i.start`
	ordinal int    // index among same-labelled params (for precise write-back)
	start   int    // byte offset of the literal in the source
	end     int    // byte offset just past the literal
	value   f64
	min     f32
	max     f32
	step    f32
}

enum Kind {
	ident
	num
	hex
	sym
}

struct Tok {
	k    Kind
	text string
	s    int
	e    int
}

struct Frame {
mut:
	name       string
	positional int
	outer_arg  string
}

struct List {
mut:
	index    int
	is_range bool
	colons   int
	ordinal  int
}

// tokenize splits `text` into tokens (comments and irrelevant symbols skipped).
fn tokenize(text string) []Tok {
	b := text.bytes()
	n := b.len
	mut toks := []Tok{}
	mut i := 0
	for i < n {
		c := b[i]
		if c == `/` && i + 1 < n && b[i + 1] == `/` {
			for i < n && b[i] != `\n` {
				i++
			}
			continue
		}
		if c.is_space() {
			i++
			continue
		}
		if c == `0` && i + 1 < n && (b[i + 1] == `x` || b[i + 1] == `X`) {
			s := i
			i += 2
			for i < n && b[i].is_hex_digit() {
				i++
			}
			toks << Tok{
				k: .hex
				text: text[s..i]
				s: s
				e: i
			}
			continue
		}
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
			toks << Tok{
				k: .num
				text: text[s..i]
				s: s
				e: i
			}
			continue
		}
		if c.is_letter() || c == `_` {
			s := i
			for i < n && (b[i].is_alnum() || b[i] == `_`) {
				i++
			}
			toks << Tok{
				k: .ident
				text: text[s..i]
				s: s
				e: i
			}
			continue
		}
		if c in [`(`, `)`, `[`, `]`, `,`, `=`, `;`, `:`] {
			toks << Tok{
				k: .sym
				text: text[i..i + 1]
				s: i
				e: i + 1
			}
			i++
			continue
		}
		i++
	}
	return toks
}

// list_is_range reports whether a `[` starts a range and the top-level `:` count.
fn list_is_range(toks []Tok, open int) (bool, int) {
	mut depth := 0
	mut colons := 0
	for j in open + 1 .. toks.len {
		if toks[j].k != .sym {
			continue
		}
		match toks[j].text {
			'[' { depth++ }
			']' {
				if depth == 0 {
					return colons > 0, colons
				}
				depth--
			}
			':' {
				if depth == 0 {
					colons++
				}
			}
			else {}
		}
	}
	return colons > 0, colons
}

// positional_name maps a positional argument index to the CGS signature name.
fn positional_name(call string, index int) string {
	sig := match call {
		'sphere' { ['r'] }
		'plane' { ['n', 'd'] }
		'cylinder' { ['r', 'h'] }
		'box' { ['s'] }
		'circle' { ['r'] }
		'translate' { ['t'] }
		'rotate' { ['axis', 'angle'] }
		'directional_light' { ['direction', 'intensity', 'color'] }
		'point_light' { ['position', 'intensity', 'color'] }
		'ambient_light' { ['intensity', 'color'] }
		'background' { ['color'] }
		else { [] }
	}
	if index < sig.len {
		return sig[index]
	}
	return '_${index}'
}

// range_for gives a sensible slider range for a labelled parameter.
fn range_for(label string, value f64) (f32, f32, f32) {
	leaf := label.all_after_last('.').all_before('[')
	match leaf {
		'angle' { return 0.0, 6.2832, 0.01 }
		'fov' { return 1.0, 120.0, 1.0 }
		'roughness', 'metalness', 'opacity', 'emissive' { return 0.0, 1.0, 0.01 }
		'ior' { return 0.5, 3.0, 0.01 }
		'absorption' { return 0.0, 2.0, 0.01 }
		'intensity' { return 0.0, 2.0, 0.01 }
		'aspect' { return 0.5, 2.5, 0.01 }
		else {
			v := f32(value)
			return v - 5.0, v + 5.0, 0.1
		}
	}
}

// extract_params returns every draggable numeric parameter in `text`, in order.
pub fn extract_params(text string) []RawParam {
	toks := tokenize(text)
	mut frames := []?Frame{}
	mut arg := ''
	mut lists := []List{}
	mut ordinals := map[string]int{}
	mut out := []RawParam{}
	mut i := 0
	for i < toks.len {
		t := toks[i]
		match t.k {
			.sym {
				match t.text {
					'(' {
						is_call := i > 0 && toks[i - 1].k == .ident
						if is_call {
							name := toks[i - 1].text
							first := positional_name(name, 0)
							frames << Frame{
								name: name
								positional: 0
								outer_arg: arg
							}
							arg = '${name}.${first}'
						} else {
							frames << none
						}
					}
					')' {
						if frames.len > 0 {
							if f := frames.pop() {
								arg = f.outer_arg
							}
						}
					}
					'=' {
						if i > 0 && toks[i - 1].k == .ident {
							name := toks[i - 1].text
							arg = if frames.len > 0 {
								if f := frames[frames.len - 1] {
									'${f.name}.${name}'
								} else {
									name
								}
							} else {
								name
							}
						}
					}
					'[' {
						is_range, colons := list_is_range(toks, i)
						lists << List{
							index: 0
							is_range: is_range
							colons: colons
							ordinal: 0
						}
					}
					']' {
						if lists.len > 0 {
							lists.pop()
						}
					}
					',' {
						if lists.len > 0 {
							if !lists[lists.len - 1].is_range {
								mut l := lists[lists.len - 1]
								l.index++
								lists[lists.len - 1] = l
							}
						} else if frames.len > 0 {
							if mut f := frames[frames.len - 1] {
								f.positional++
								frames[frames.len - 1] = f
								arg = '${f.name}.${positional_name(f.name, f.positional)}'
							}
						}
					}
					';' { arg = '' }
					else {}
				}
			}
			.num {
				if arg != '' {
					value := t.text.f64()
					mut label := arg
					if lists.len > 0 {
						l := lists[lists.len - 1]
						if l.is_range {
							name := if l.ordinal == 0 {
								'start'
							} else if l.ordinal == 1 && l.colons == 1 {
								'stop'
							} else if l.ordinal == 1 {
								'step'
							} else {
								'elem'
							}
							label = '${arg}.${name}'
						} else {
							label = '${arg}[${l.index}]'
						}
					}
					ordinal := ordinals[label] or { 0 }
					ordinals[label] = ordinal + 1
					mn, mx, st := range_for(label, value)
					out << RawParam{
						label: label
						ordinal: ordinal
						start: t.s
						end: t.e
						value: value
						min: mn
						max: mx
						step: st
					}
					if lists.len > 0 && lists[lists.len - 1].is_range {
						mut l := lists[lists.len - 1]
						l.ordinal++
						lists[lists.len - 1] = l
					}
				}
			}
			.hex {}
			.ident {}
		}
		i++
	}
	return out
}

// format_number formats a value compactly (trailing zeros / dot trimmed).
pub fn format_number(v f64) string {
	mut s := '${v:.4}'
	s = s.trim_right('0').trim_right('.')
	if s == '' || s == '-0' || s == '-' {
		return '0'
	}
	return s
}

// replace_range replaces text[start..end] with `repl`.
pub fn replace_range(text string, start int, end int, repl string) string {
	return text[..start] + repl + text[end..]
}
