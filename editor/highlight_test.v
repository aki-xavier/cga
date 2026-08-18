module main

// highlight_test.v — tests for CGS syntax highlighting.

fn span_classes(text string) []HighlightClass {
	mut out := []HighlightClass{}
	for s in highlight_tokenize(text) {
		out << s.class
	}
	return out
}

fn test_highlight_keywords_and_types() {
	spans := highlight_tokenize('for (i = [0:2]) sphere(r=1);')
	classes := span_classes('for (i = [0:2]) sphere(r=1);')
	assert classes.contains(.keyword) // for
	assert classes.contains(.typ) // sphere
	assert classes.contains(.number) // 0, 2, 1
	assert classes.contains(.punctuation) // ( ) [ ] = ; etc
	_ = spans
}

fn test_highlight_comment() {
	spans := highlight_tokenize('// hello\nsphere(r=1);')
	assert spans.len > 0
	assert spans[0].class == .comment
	assert spans[0].start == 0
	assert spans[0].end == 8 // '// hello'
}

fn test_highlight_hex_is_number() {
	spans := highlight_tokenize('background(color=0x87CEEB);')
	mut has_hex := false
	for s in spans {
		if s.class == .number && s.end - s.start == 8 {
			has_hex = true
		}
	}
	assert has_hex
}

fn test_highlight_functions_and_constants() {
	spans := highlight_tokenize('translate([sin(pi), 0, 0]) sphere(r=1);')
	classes := span_classes('translate([sin(pi), 0, 0]) sphere(r=1);')
	assert classes.contains(.function) // sin
	assert classes.contains(.constant) // pi
	_ = spans
}

fn test_highlight_classify_word() {
	assert classify_word('module') or { HighlightClass.plain } == .keyword
	assert classify_word('sphere') or { HighlightClass.plain } == .typ
	assert classify_word('sqrt') or { HighlightClass.plain } == .function
	assert classify_word('pi') or { HighlightClass.plain } == .constant
	assert classify_word('myvar') == none
}

fn test_highlight_braces() {
	folds := brace_fold_ranges('module m() {\n  sphere(r=1);\n}\n')
	assert folds.len == 1
	assert folds[0].start == 0
	assert folds[0].end == 2
}

fn test_highlight_color_map() {
	assert highlight_color(.keyword) == 0xc678dd
	assert highlight_color(.plain) == 0xdfe2ea
}
