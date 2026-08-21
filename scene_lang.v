module cga

// CGS (CGA Scene) language: lexer + single-pass parser/evaluator producing an
// engine Scene + PerspectiveCamera.  Port of cga_py/scene_lang.
//
// Supports translate/rotate/scale/mirror modifiers, for/if/echo/module,
// variables/expressions/math functions, all primitives, lights, camera,
// background, CSG (difference/intersection/union), and
// extrude/loft/mesh(.obj/.glb/.gltf).
import math
import strconv

// TokenKind classifies a lexed token.
enum TokenKind {
	ident
	number
	op
	str
	eof
	lparen
	rparen
	lbracket
	rbracket
	lbrace
	rbrace
	comma
	semi
	assign
}

// punct_kind maps a single-character punctuation byte to its token kind.
fn punct_kind(ch u8) TokenKind {
	return match ch {
		`(` { .lparen }
		`)` { .rparen }
		`[` { .lbracket }
		`]` { .rbracket }
		`{` { .lbrace }
		`}` { .rbrace }
		`,` { .comma }
		`;` { .semi }
		`=` { .assign }
		else { .eof }
	}
}

// CgsToken is one lexed token.
pub struct CgsToken {
pub:
	kind TokenKind
	text string
	num  f64
	line int
}

// lex tokenises CGS source text.
pub fn cgs_lex(text string) ![]CgsToken {
	mut toks := []CgsToken{}
	mut i := 0
	mut ln := 1
	n := text.len
	for i < n {
		ch := text[i]
		if ch == `\n` {
			ln++
			i++
		} else if ch == ` ` || ch == `\t` || ch == `\r` {
			i++
		} else if text[i..].starts_with('//') {
			for i < n && text[i] != `\n` {
				i++
			}
		} else if i + 1 < n && text[i..i + 2] in ['==', '!=', '<=', '>=', '&&', '||'] {
			toks << CgsToken{
				kind: .op
				text: text[i..i + 2]
				line: ln
			}
			i += 2
		} else if ch in [`+`, `-`, `*`, `/`, `%`, `<`, `>`, `!`, `:`] {
			toks << CgsToken{
				kind: .op
				text: ch.ascii_str()
				line: ln
			}
			i++
		} else if ch in [`[`, `]`, `{`, `}`, `,`, `;`, `=`, `(`, `)`] {
			toks << CgsToken{
				kind: punct_kind(ch)
				text: ch.ascii_str()
				line: ln
			}
			i++
		} else if ch == `"` || ch == `'` {
			quote := ch
			mut j := i + 1
			for j < n && text[j] != quote {
				if text[j] == `\\` {
					j++
				}
				j++
			}
			if j >= n {
				return error('CGS line ${ln}: unclosed string')
			}
			toks << CgsToken{
				kind: .str
				text: text[i + 1..j]
				line: ln
			}
			i = j + 1
		} else if ch.is_letter() || ch == `_` {
			mut j := i + 1
			for j < n && (text[j].is_alnum() || text[j] == `_`) {
				j++
			}
			toks << CgsToken{
				kind: .ident
				text: text[i..j]
				line: ln
			}
			i = j
		} else if ch >= `0` && ch <= `9` {
			if text[i..].starts_with('0x') || text[i..].starts_with('0X') {
				mut j := i + 2
				for j < n && text[j].is_hex_digit() {
					j++
				}
				if j == i + 2 {
					return error('CGS line ${ln}: illegal hex colour')
				}
				toks << CgsToken{
					kind: .number
					num:  f64(strconv.parse_uint(text[i + 2..j], 16, 32) or { 0 })
					line: ln
				}
				i = j
			} else {
				mut j := i
				for j < n && text[j] >= `0` && text[j] <= `9` {
					j++
				}
				if j < n && text[j] == `.` {
					j++
					for j < n && text[j] >= `0` && text[j] <= `9` {
						j++
					}
				}
				if j < n && (text[j] == `e` || text[j] == `E`) {
					j++
					if j < n && (text[j] == `+` || text[j] == `-`) {
						j++
					}
					for j < n && text[j] >= `0` && text[j] <= `9` {
						j++
					}
				}
				toks << CgsToken{
					kind: .number
					num:  text[i..j].f64()
					line: ln
				}
				i = j
			}
		} else {
			return error('CGS line ${ln}: illegal character ${ch.ascii_str()}')
		}
	}
	return toks
}

// CgsVec3 is a 3-vector value in the CGS language (a `[x,y,z]` literal).
struct CgsVec3 {
	x f64
	y f64
	z f64
}

// CgsValue is the dynamic value of the language (number / bool / string / list /
// 3-vector).  A `[x,y,z]` literal with three numeric elements is a CgsVec3;
// ranges and other lists stay `[]CgsValue`.
type CgsValue = f64 | bool | string | []CgsValue | CgsVec3

fn cgs_num(v CgsValue, line int, what string) !f64 {
	match v {
		f64 { return v }
		else { return error('CGS line ${line}: ${what} needs a number, got ${v}') }
	}
}

fn cgs_vec3(v CgsValue, line int, what string) ![3]f64 {
	match v {
		CgsVec3 {
			return [v.x, v.y, v.z]!
		}
		[]CgsValue {
			if v.len != 3 {
				return error('CGS line ${line}: ${what} needs [x,y,z], got ${v}')
			}
			return [cgs_num(v[0], line, what)!, cgs_num(v[1], line, what)!,
				cgs_num(v[2], line, what)!]!
		}
		else {
			return error('CGS line ${line}: ${what} needs [x,y,z], got ${v}')
		}
	}
}

fn cgs_opt_num(v CgsValue, def f64) f64 {
	match v {
		f64 {
			if v < 0.0 {
				return def
			}
			return v
		}
		else {
			return def
		}
	}
}

fn translate4(t [3]f64) [16]f64 {
	return [1.0, 0.0, 0.0, t[0], 0.0, 1.0, 0.0, t[1], 0.0, 0.0, 1.0, t[2], 0.0, 0.0, 0.0, 1.0]!
}

fn scale4(s [3]f64) [16]f64 {
	return [s[0], 0.0, 0.0, 0.0, 0.0, s[1], 0.0, 0.0, 0.0, 0.0, s[2], 0.0, 0.0, 0.0, 0.0, 1.0]!
}

fn mirror4(ax [3]f64) ![16]f64 {
	n := math.sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2])
	if n < 1e-12 {
		return error('mirror.axis must not be zero')
	}
	ux := ax[0] / n
	uy := ax[1] / n
	uz := ax[2] / n
	return [1.0 - 2.0 * ux * ux, -2.0 * ux * uy, -2.0 * ux * uz, 0.0, -2.0 * uy * ux,
		1.0 - 2.0 * uy * uy, -2.0 * uy * uz, 0.0, -2.0 * uz * ux, -2.0 * uz * uy, 1.0 - 2.0 * uz * uz,
		0.0, 0.0, 0.0, 0.0, 1.0]!
}

fn cgs_truthy(v CgsValue) bool {
	match v {
		[]CgsValue { return v.len > 0 }
		CgsVec3 { return true }
		bool { return v }
		f64 { return v != 0.0 }
		string { return v != '' }
	}
}

fn cgs_neg(v CgsValue, line int) !CgsValue {
	match v {
		f64 {
			x := -v
			return x
		}
		CgsVec3 {
			return CgsVec3{-v.x, -v.y, -v.z}
		}
		[]CgsValue {
			mut out := []CgsValue{}
			for x in v {
				out << cgs_neg(x, line)!
			}
			return out
		}
		else {
			return error('CGS line ${line}: unary minus needs number/vector')
		}
	}
}

// cgs_as_list normalises a 3-vector to a 3-element numeric list, so vector and
// list arithmetic share one elementwise path.
fn cgs_as_list(v CgsValue) CgsValue {
	return match v {
		CgsVec3 { [CgsValue(v.x), CgsValue(v.y), CgsValue(v.z)] }
		else { v }
	}
}

// BinOp is a binary operator in a CGS expression.
enum BinOp {
	or_
	and_
	eq
	ne
	lt
	le
	gt
	ge
	add
	sub
	mul
	div
	mod_
}

// cgs_binop_from_text maps an operator token's text to a BinOp.
fn cgs_binop_from_text(text string) ?BinOp {
	return match text {
		'||' { .or_ }
		'&&' { .and_ }
		'==' { .eq }
		'!=' { .ne }
		'<' { .lt }
		'<=' { .le }
		'>' { .gt }
		'>=' { .ge }
		'+' { .add }
		'-' { .sub }
		'*' { .mul }
		'/' { .div }
		'%' { .mod_ }
		else { none }
	}
}

fn cgs_scalar_arith(op BinOp, a f64, b f64) f64 {
	return match op {
		.add { a + b }
		.sub { a - b }
		.mul { a * b }
		.div { a / b }
		.mod_ { math.fmod(a, b) }
		else { panic('bad op ${op}') }
	}
}

fn cgs_binop(op BinOp, a CgsValue, b CgsValue) !CgsValue {
	if op == .eq {
		return a == b
	}
	if op == .ne {
		return a != b
	}
	if op == .and_ {
		return cgs_truthy(a) && cgs_truthy(b)
	}
	if op == .or_ {
		return cgs_truthy(a) || cgs_truthy(b)
	}
	if op in [.lt, .le, .gt, .ge] {
		aa := cgs_num(a, 0, 'cmp')!
		bb := cgs_num(b, 0, 'cmp')!
		return match op {
			.lt { aa < bb }
			.le { aa <= bb }
			.gt { aa > bb }
			else { aa >= bb }
		}
	}
	// arithmetic: scalar or elementwise vector (scalar broadcast)
	al := cgs_as_list(a)
	bl := cgs_as_list(b)
	match al {
		[]CgsValue {
			match bl {
				[]CgsValue {
					if al.len != bl.len {
						return error('CGS: vector length mismatch (${al.len} vs ${bl.len})')
					}
					mut out := []CgsValue{}
					for i in 0 .. al.len {
						out << cgs_binop(op, al[i], bl[i])!
					}
					return out
				}
				else {
					mut out := []CgsValue{}
					for x in al {
						out << cgs_binop(op, x, bl)!
					}
					return out
				}
			}
		}
		else {
			match bl {
				[]CgsValue {
					mut out := []CgsValue{}
					for x in bl {
						out << cgs_binop(op, al, x)!
					}
					return out
				}
				else {
					return cgs_scalar_arith(op, cgs_num(al, 0, 'arith')!, cgs_num(bl, 0, 'arith')!)
				}
			}
		}
	}
}

fn cgs_call_fn(name string, args []CgsValue, line int) !CgsValue {
	if name == 'len' {
		a0 := args[0]
		match a0 {
			[]CgsValue { return f64(a0.len) }
			CgsVec3 { return f64(3) }
			else { return error('CGS line ${line}: len needs a list') }
		}
	}
	if name == 'norm' {
		a0 := args[0]
		match a0 {
			CgsVec3 {
				return math.sqrt(a0.x * a0.x + a0.y * a0.y + a0.z * a0.z)
			}
			[]CgsValue {
				mut s := 0.0
				for x in a0 {
					s += cgs_num(x, line, 'norm')! * cgs_num(x, line, 'norm')!
				}
				return math.sqrt(s)
			}
			else {
				return error('CGS line ${line}: norm needs a vector')
			}
		}
	}
	if name == 'cross' {
		a := cgs_vec3(args[0], line, 'cross.a')!
		b := cgs_vec3(args[1], line, 'cross.b')!
		return [CgsValue(a[1] * b[2] - a[2] * b[1]), a[2] * b[0] - a[0] * b[2],
			a[0] * b[1] - a[1] * b[0]]
	}
	// unary math functions
	if args.len == 1 {
		x := cgs_num(args[0], line, name)!
		v := match name {
			'abs' {
				math.abs(x)
			}
			'sign' {
				if x > 0 {
					1.0
				} else if x < 0 {
					-1.0
				} else {
					0.0
				}
			}
			'sin' {
				math.sin(x)
			}
			'cos' {
				math.cos(x)
			}
			'tan' {
				math.tan(x)
			}
			'asin' {
				math.asin(x)
			}
			'acos' {
				math.acos(x)
			}
			'atan' {
				math.atan(x)
			}
			'sqrt' {
				math.sqrt(x)
			}
			'exp' {
				math.exp(x)
			}
			'ln' {
				math.log(x)
			}
			'log' {
				math.log10(x)
			}
			'floor' {
				math.floor(x)
			}
			'ceil' {
				math.ceil(x)
			}
			'round' {
				math.round(x)
			}
			else {
				return error('CGS line ${line}: unknown function ${name}')
			}
		}
		return v
	}
	if args.len == 2 {
		a := cgs_num(args[0], line, name)!
		b := cgs_num(args[1], line, name)!
		return match name {
			'atan2' { math.atan2(a, b) }
			'pow' { math.pow(a, b) }
			'min' { math.min(a, b) }
			'max' { math.max(a, b) }
			else { return error('CGS line ${line}: unknown function ${name}') }
		}
	}
	return error('CGS line ${line}: ${name} wrong arity')
}

// SceneLoader parses CGS text.
pub struct SceneLoader {
mut:
	toks       []CgsToken
	pos        int
	asset_root string
	scene      Scene
	camera     ?PerspectiveCamera
	modules    map[string][]CgsToken
	params     map[string][]CgsToken // module formal parameters (name -> body tokens)
	collect    []CollectedGeom
	collecting bool
}

struct CollectedGeom {
	geo Geometry
	m4  [16]f64
}

// cgs_load parses CGS text into (Scene, PerspectiveCamera), panicking on error.
pub fn cgs_load(text string, asset_root string) (Scene, PerspectiveCamera) {
	return cgs_load_result(text, asset_root) or { panic(err) }
}

// cgs_load_result parses CGS text, returning the first error instead of
// panicking (so callers such as the render server can report it cleanly).
pub fn cgs_load_result(text string, asset_root string) !(Scene, PerspectiveCamera) {
	toks := cgs_lex(text)!
	mut l := SceneLoader{
		toks:       toks
		asset_root: asset_root
		scene:      scene(none)
		modules:    map[string][]CgsToken{}
		params:     map[string][]CgsToken{}
	}
	mut root_scope := map[string]CgsValue{}
	root_scope['pi'] = f64(math.pi)
	l.run_tokens(l.toks.clone(), mat4_identity(), map[string]CgsValue{}, mut root_scope)!
	mut cam := if c := l.camera {
		c
	} else {
		mut c2 := perspective_camera(50.0, 16.0 / 9.0, 0.1, 100.0, [0.0, 0.0, 5.0]!, [
			0.0,
			0.0,
			0.0,
		]!, [0.0, 1.0, 0.0]!)
		c2.look_at([0.0, 0.0, 0.0]!, none)
		c2
	}
	return l.scene, cam
}

fn (mut l SceneLoader) peek() CgsToken {
	if l.pos >= l.toks.len {
		return CgsToken{
			kind: .eof
			line: 1
		}
	}
	return l.toks[l.pos]
}

fn (mut l SceneLoader) peek1() CgsToken {
	if l.pos + 1 >= l.toks.len {
		return CgsToken{
			kind: .eof
			line: 1
		}
	}
	return l.toks[l.pos + 1]
}

fn (mut l SceneLoader) take() CgsToken {
	t := l.peek()
	l.pos++
	return t
}

fn (mut l SceneLoader) expect(sym TokenKind) ! {
	t := l.take()
	if t.kind != sym {
		return error('CGS line ${t.line}: expected ${sym}, got ${t.kind}')
	}
}

fn (mut l SceneLoader) run_tokens(toks []CgsToken, ctx [16]f64, mat map[string]CgsValue, mut scope map[string]CgsValue) ! {
	saved := l.toks
	saved_pos := l.pos
	l.toks = toks
	l.pos = 0
	for l.pos < l.toks.len {
		l.statement(ctx, mat, mut scope)!
	}
	l.toks = saved
	l.pos = saved_pos
}

// --- expression parser (precedence climbing) --------------------------------

fn (mut l SceneLoader) expr(scope map[string]CgsValue, min_prec int) !CgsValue {
	mut lhs := l.unary(scope)!
	for {
		t := l.peek()
		if t.kind != .op {
			return lhs
		}
		op := cgs_binop_from_text(t.text) or { return lhs }
		prec := cgs_precedence(op)
		if prec < min_prec {
			return lhs
		}
		l.take()
		rhs := l.expr(scope, prec + 1)!
		lhs = cgs_binop(op, lhs, rhs) or { return error(err.msg()) }
	}
	return lhs
}

fn cgs_precedence(op BinOp) int {
	return match op {
		.or_ { 1 }
		.and_ { 2 }
		.eq, .ne { 3 }
		.lt, .le, .gt, .ge { 4 }
		.add, .sub { 5 }
		.mul, .div, .mod_ { 6 }
	}
}

fn (mut l SceneLoader) unary(scope map[string]CgsValue) !CgsValue {
	t := l.peek()
	if t.kind == .op && t.text == '-' {
		l.take()
		return cgs_neg(l.expr(scope, 7)!, t.line) or { return error(err.msg()) }
	}
	if t.kind == .op && t.text == '!' {
		l.take()
		return !cgs_truthy(l.expr(scope, 7)!)
	}
	return l.primary(scope)!
}

fn (mut l SceneLoader) primary(scope map[string]CgsValue) !CgsValue {
	t := l.take()
	if t.kind == .number {
		return t.num
	}
	if t.kind == .lparen {
		v := l.expr(scope, 1)!
		l.expect(.rparen)!
		return v
	}
	if t.kind == .lbracket {
		return l.list_literal(scope, t.line)!
	}
	if t.kind == .str {
		return t.text
	}
	if t.kind == .ident {
		if l.peek().kind == .lparen {
			l.take()
			mut args := []CgsValue{}
			if l.peek().kind != .rparen {
				for {
					args << l.expr(scope, 1)!
					if l.peek().kind == .comma {
						l.take()
					} else {
						break
					}
				}
			}
			l.expect(.rparen)!
			return cgs_call_fn(t.text, args, t.line) or { return error(err.msg()) }
		}
		if t.text == 'true' {
			return true
		}
		if t.text == 'false' {
			return false
		}
		if v := scope[t.text] {
			return v
		}
		return error('CGS line ${t.line}: undefined variable ${t.text}')
	}
	return error('CGS line ${t.line}: bad expression start ${t.kind}')
}

fn (mut l SceneLoader) list_literal(scope map[string]CgsValue, line int) !CgsValue {
	first := l.expr(scope, 1)!
	if l.peek().kind == .op && l.peek().text == ':' {
		l.take()
		second := l.expr(scope, 1)!
		mut step := 1.0
		mut start_v := cgs_num(first, line, 'range') or { return error(err.msg()) }
		mut stop_v := cgs_num(second, line, 'range') or { return error(err.msg()) }
		if l.peek().kind == .op && l.peek().text == ':' {
			l.take()
			third := l.expr(scope, 1)!
			start_v = cgs_num(first, line, 'range') or { return error(err.msg()) }
			step = cgs_num(second, line, 'range') or { return error(err.msg()) }
			stop_v = cgs_num(third, line, 'range') or { return error(err.msg()) }
		}
		l.expect(.rbracket)!
		if step == 0.0 {
			return error('CGS line ${line}: range step must not be 0')
		}
		mut out := []CgsValue{}
		mut v := start_v
		if step > 0 {
			for v <= stop_v + 1e-12 {
				out << f64(v)
				v += step
			}
		} else {
			for v >= stop_v - 1e-12 {
				out << f64(v)
				v += step
			}
		}
		return out
	}
	mut items := [first]
	for l.peek().kind == .comma {
		l.take()
		items << l.expr(scope, 1)!
	}
	l.expect(.rbracket)!
	// A 3-element numeric literal is a vector (CgsVec3); anything else is a list.
	if items.len == 3 {
		x := items[0]
		y := items[1]
		z := items[2]
		if x is f64 && y is f64 && z is f64 {
			return CgsVec3{x, y, z}
		}
	}
	return items
}

// --- statements -------------------------------------------------------------

fn (mut l SceneLoader) statement(ctx [16]f64, mat map[string]CgsValue, mut scope map[string]CgsValue) ! {
	t := l.peek()
	if t.kind == .lbrace {
		l.expect(.lbrace)!
		for l.peek().kind != .rbrace {
			l.statement(ctx, mat, mut scope)!
		}
		l.expect(.rbrace)!
		return
	}
	if t.kind == .ident {
		name := t.text
		if name == 'module' {
			l.module_def()!
			return
		}
		if name == 'for' {
			l.for_loop(ctx, mat, mut scope)!
			return
		}
		if name == 'if' {
			l.if_stmt(ctx, mat, mut scope)!
			return
		}
		if name == 'echo' {
			l.echo_stmt(scope)!
			return
		}
		if name == 'union' {
			l.take()
			l.expect(.lparen)!
			l.expect(.rparen)!
			l.body(ctx, mat, mut scope, t.line)!
			return
		}
		if name in ['difference', 'intersection'] {
			l.take()
			l.expect(.lparen)!
			l.expect(.rparen)!
			op := if name == 'difference' { CsgOp.difference } else { CsgOp.intersection }
			l.csg_block(op, ctx, mat, mut scope, t.line)!
			return
		}
		if l.peek1().kind == .assign {
			l.take()
			l.take()
			scope[name] = l.expr(scope, 1)!
			l.expect(.semi)!
			return
		}
	}
	l.take()
	if t.kind != .ident {
		return error('CGS line ${t.line}: expected statement name, got ${t.kind}')
	}
	pos, kw := l.call_args(scope)!
	name := t.text
	if body := l.modules[name] {
		_ = body
		l.module_call(name, pos, kw, ctx, mat, t.line)!
		return
	}
	args := l.resolve(name, pos, kw, t.line)!
	if name == 'translate' {
		v := cgs_vec3(args['t'] or { f64(0.0) }, t.line, 'translate.t') or {
			return error(err.msg())
		}
		m2 := mat4_mul(ctx, translate4(v))
		l.body(m2, mat, mut scope, t.line)!
		return
	}
	if name == 'rotate' {
		ax := cgs_vec3(args['axis'] or { f64(0.0) }, t.line, 'rotate.axis') or {
			return error(err.msg())
		}
		ang := cgs_num(args['angle'] or { f64(0.0) }, t.line, 'rotate.angle') or {
			return error(err.msg())
		}
		m2 := mat4_mul(ctx, motor_rotor(ax, ang).to_matrix())
		l.body(m2, mat, mut scope, t.line)!
		return
	}
	if name == 'scale' {
		sv := args['s'] or { f64(1.0) }
		mut s4 := [16]f64{}
		match sv {
			CgsVec3 {
				s4 = scale4([sv.x, sv.y, sv.z]!)
			}
			[]CgsValue {
				v := cgs_vec3(sv, t.line, 'scale.s') or { return error(err.msg()) }
				s4 = scale4(v)
			}
			else {
				x := cgs_num(sv, t.line, 'scale.s') or { return error(err.msg()) }
				s4 = scale4([x, x, x]!)
			}
		}
		m2 := mat4_mul(ctx, s4)
		l.body(m2, mat, mut scope, t.line)!
		return
	}
	if name == 'mirror' {
		ax := cgs_vec3(args['axis'] or { f64(0.0) }, t.line, 'mirror.axis') or {
			return error(err.msg())
		}
		m2 := mat4_mul(ctx, mirror4(ax) or { return error(err.msg()) })
		l.body(m2, mat, mut scope, t.line)!
		return
	}
	if name == 'material' {
		mut merged := map[string]CgsValue{}
		for k, v in mat {
			merged[k] = v
		}
		for k, v in args {
			merged[k] = v
		}
		l.body(ctx, merged, mut scope, t.line)!
		return
	}
	if name == 'background' {
		l.expect(.semi)!
		c := cgs_num(args['color'] or { f64(0.0) }, t.line, 'color') or { return error(err.msg()) }
		l.scene.background = color_hex(int(c))
		return
	}
	if name == 'camera' {
		l.expect(.semi)!
		fov := cgs_num(args['fov'] or { f64(0.0) }, t.line, 'fov') or { return error(err.msg()) }
		aspect := cgs_num(args['aspect'] or { f64(0.0) }, t.line, 'aspect') or {
			return error(err.msg())
		}
		campos := cgs_vec3(args['position'] or { f64(0.0) }, t.line, 'camera.position') or {
			return error(err.msg())
		}
		tgt := cgs_vec3(args['target'] or { f64(0.0) }, t.line, 'camera.target') or {
			return error(err.msg())
		}
		mut cam := perspective_camera(fov, aspect, 0.1, 100.0, campos, tgt, [0.0, 1.0, 0.0]!)
		cam.look_at(tgt, none)
		l.camera = cam
		return
	}
	if name.ends_with('_light') {
		l.expect(.semi)!
		l.add_light(name, args, t.line)!
		return
	}
	l.expect(.semi)!
	geo := l.build_geometry(name, args, t.line)!
	l.add_geometry(geo, ctx, mat)!
}

fn (mut l SceneLoader) body(ctx [16]f64, mat map[string]CgsValue, mut scope map[string]CgsValue, line int) ! {
	if l.peek().kind == .lbrace {
		l.expect(.lbrace)!
		for l.peek().kind != .rbrace {
			l.statement(ctx, mat, mut scope)!
		}
		l.expect(.rbrace)!
	} else if l.peek().kind == .semi {
		return error('CGS line ${line}: modifier missing target statement')
	} else {
		l.statement(ctx, mat, mut scope)!
	}
}

fn (mut l SceneLoader) for_loop(ctx [16]f64, mat map[string]CgsValue, mut scope map[string]CgsValue) ! {
	l.take() // for
	l.expect(.lparen)!
	vt := l.take()
	if vt.kind != .ident || l.peek().kind != .assign {
		return error('CGS line ${vt.line}: for needs (var = list)')
	}
	l.take()
	values := l.expr(scope, 1)!
	l.expect(.rparen)!
	match values {
		CgsVec3 {
			body := l.capture_statement()
			for v in [values.x, values.y, values.z] {
				scope[vt.text] = v
				l.run_tokens(body, ctx, mat, mut scope)!
			}
		}
		[]CgsValue {
			body := l.capture_statement()
			for v in values {
				scope[vt.text] = v
				l.run_tokens(body, ctx, mat, mut scope)!
			}
		}
		else {
			return error('CGS line ${vt.line}: for needs a list')
		}
	}
}

fn (mut l SceneLoader) if_stmt(ctx [16]f64, mat map[string]CgsValue, mut scope map[string]CgsValue) ! {
	l.take() // if
	l.expect(.lparen)!
	cond := cgs_truthy(l.expr(scope, 1)!)
	l.expect(.rparen)!
	if cond {
		l.statement(ctx, mat, mut scope)!
		if l.peek().kind == .ident && l.peek().text == 'else' {
			l.take()
			l.skip_statement()
		}
	} else {
		l.skip_statement()
		if l.peek().kind == .ident && l.peek().text == 'else' {
			l.take()
			l.statement(ctx, mat, mut scope)!
		}
	}
}

fn (mut l SceneLoader) echo_stmt(scope map[string]CgsValue) ! {
	l.take() // echo
	l.expect(.lparen)!
	mut vals := []CgsValue{}
	if l.peek().kind != .rparen {
		for {
			vals << l.expr(scope, 1)!
			if l.peek().kind == .comma {
				l.take()
			} else {
				break
			}
		}
	}
	l.expect(.rparen)!
	l.expect(.semi)!
	print('ECHO:')
	for v in vals {
		print(' ${v}')
	}
	println('')
}

fn (mut l SceneLoader) module_def() ! {
	l.take() // module
	nt := l.take()
	if nt.kind != .ident {
		return error('CGS line ${nt.line}: module missing name')
	}
	l.expect(.lparen)!
	if l.peek().kind != .rparen {
		for {
			pt := l.take()
			if pt.kind != .ident {
				return error('CGS line ${pt.line}: module parameter must be a name')
			}
			if l.peek().kind == .assign {
				l.take()
				l.params[nt.text + ':' + pt.text] = l.capture_expr()!
			} else {
				l.params[nt.text + ':' + pt.text] = []CgsToken{}
			}
			if l.peek().kind == .comma {
				l.take()
			} else {
				break
			}
		}
	}
	l.expect(.rparen)!
	l.expect(.lbrace)!
	start := l.pos
	mut depth := 1
	for depth > 0 {
		k := l.take().kind
		if k == .lbrace {
			depth++
		} else if k == .rbrace {
			depth--
		}
	}
	l.modules[nt.text] = l.toks[start..l.pos - 1]
}

fn (mut l SceneLoader) module_call(name string, pos []CgsValue, kw map[string]CgsValue, ctx [16]f64, mat map[string]CgsValue, line int) ! {
	l.expect(.semi)!
	// formal parameter names are tracked separately; simplified: only positional + keyword by name
	body := l.modules[name]
	mut scope := map[string]CgsValue{}
	scope['pi'] = f64(math.pi)
	// find formal parameter names (stored as keys "name:param")
	mut names := []string{}
	for k, _ in l.params {
		if k.starts_with(name + ':') {
			names << k.all_after(':')
		}
	}
	for i, pname in names {
		if i < pos.len {
			scope[pname] = pos[i]
		}
	}
	for k, v in kw {
		scope[k] = v
	}
	for pname in names {
		if pname !in scope {
			def := l.params[name + ':' + pname] or { []CgsToken{} }
			if def.len == 0 {
				return error('CGS line ${line}: module ${name} missing parameter ${pname}')
			}
			scope[pname] = l.eval_tokens(def, scope)!
		}
	}
	l.run_tokens(body, ctx, mat, mut scope)!
}

fn (mut l SceneLoader) capture_expr() ![]CgsToken {
	start := l.pos
	mut depth := 0
	for {
		t := l.peek()
		if t.kind == .eof {
			return error('CGS line ${t.line}: unclosed expression')
		}
		if depth == 0 && t.kind in [.comma, .rparen, .semi] {
			break
		}
		if t.kind in [.lparen, .lbracket] {
			depth++
		} else if t.kind in [.rparen, .rbracket] {
			depth--
		}
		l.take()
	}
	return l.toks[start..l.pos]
}

fn (mut l SceneLoader) eval_tokens(toks []CgsToken, scope map[string]CgsValue) !CgsValue {
	saved := l.toks
	saved_pos := l.pos
	l.toks = toks
	l.pos = 0
	v := l.expr(scope, 1)!
	l.toks = saved
	l.pos = saved_pos
	return v
}

fn (mut l SceneLoader) capture_statement() []CgsToken {
	start := l.pos
	l.skip_statement()
	return l.toks[start..l.pos]
}

fn (mut l SceneLoader) skip_statement() {
	t := l.peek()
	if t.kind == .lbrace {
		l.take()
		mut depth := 1
		for depth > 0 {
			k := l.take().kind
			if k == .lbrace {
				depth++
			} else if k == .rbrace {
				depth--
			}
		}
		return
	}
	if t.kind == .ident && t.text == 'if' {
		l.take()
		l.skip_parens()
		l.skip_statement()
		if l.peek().kind == .ident && l.peek().text == 'else' {
			l.take()
			l.skip_statement()
		}
		return
	}
	if t.kind == .ident && t.text in ['for', 'union', 'echo'] {
		l.take()
		l.skip_parens()
		if t.text == 'echo' {
			l.expect(.semi) or { return }
		} else {
			l.skip_statement()
		}
		return
	}
	if t.kind == .ident && t.text == 'module' {
		l.take()
		l.take()
		l.skip_parens()
		l.skip_statement()
		return
	}
	l.take()
	if l.peek().kind == .assign {
		for l.take().kind != .semi {
		}
		return
	}
	if l.peek().kind == .lparen {
		l.skip_parens()
		nxt := l.peek().kind
		if nxt == .lbrace {
			l.skip_statement()
		} else if nxt == .semi {
			l.take()
		} else {
			l.skip_statement()
		}
		return
	}
	for l.take().kind != .semi {
	}
}

fn (mut l SceneLoader) skip_parens() {
	l.expect(.lparen) or { return }
	mut depth := 1
	for depth > 0 {
		k := l.take().kind
		if k == .lparen {
			depth++
		} else if k == .rparen {
			depth--
		}
	}
}

// --- construction -----------------------------------------------------------

fn (mut l SceneLoader) call_args(scope map[string]CgsValue) !([]CgsValue, map[string]CgsValue) {
	l.expect(.lparen)!
	mut pos := []CgsValue{}
	mut kw := map[string]CgsValue{}
	if l.peek().kind != .rparen {
		for {
			t := l.peek()
			if t.kind == .ident && l.peek1().kind == .assign {
				l.take()
				l.take()
				kw[t.text] = l.expr(scope, 1)!
			} else {
				pos << l.expr(scope, 1)!
			}
			if l.peek().kind == .comma {
				l.take()
			} else {
				break
			}
		}
	}
	l.expect(.rparen)!
	return pos, kw
}

fn (mut l SceneLoader) resolve(name string, pos []CgsValue, kw map[string]CgsValue, line int) !map[string]CgsValue {
	names := cgs_sig_names(name)
	defaults := cgs_sig_defaults(name)
	mut merged := map[string]CgsValue{}
	for k, v in defaults {
		merged[k] = v
	}
	if pos.len > names.len {
		return error('CGS line ${line}: ${name} too many positional args')
	}
	for i, pname in names {
		if i < pos.len {
			merged[pname] = pos[i]
		}
	}
	for k, v in kw {
		if k !in names && k !in defaults {
			return error('CGS line ${line}: ${name} has no parameter ${k}')
		}
		merged[k] = v
	}
	for pname in names {
		if v := merged[pname] {
			match v {
				f64 {
					if math.is_nan(v) && pname == 'cylinder.h' {
						// h may be None -> use -1 sentinel
						merged[pname] = f64(-1.0)
					}
				}
				else {}
			}
		}
	}
	return merged
}

fn cgs_sig_names(name string) []string {
	return match name {
		'sphere' { ['r'] }
		'plane' { ['n'] }
		'cylinder' { ['r'] }
		'box' { ['s'] }
		'circle' { ['r'] }
		'cone' { ['r', 'h'] }
		'torus' { ['R', 'r'] }
		'cyclide' { ['a', 'b', 'd'] }
		'ellipsoid' { ['radii'] }
		'extrude' { ['profile', 'h'] }
		'loft' { ['profiles', 'zs'] }
		'mesh' { ['file'] }
		'translate' { ['t'] }
		'rotate' { ['axis', 'angle'] }
		'scale' { ['s'] }
		'mirror' { ['axis'] }
		'difference' { [] }
		'intersection' { [] }
		'directional_light' { ['direction'] }
		'point_light' { ['position'] }
		'ambient_light' { [] }
		'background' { ['color'] }
		'camera' { [] }
		'material' { [] }
		else { [] }
	}
}

fn cgs_sig_defaults(name string) map[string]CgsValue {
	mut m := map[string]CgsValue{}
	match name {
		'plane' {
			m['d'] = f64(0.0)
		}
		'cylinder' {
			m['h'] = f64(-1.0)
		}
		'directional_light', 'point_light' {
			m['intensity'] = f64(1.0)
			m['color'] = f64(0xFFFFFF)
		}
		'ambient_light' {
			m['intensity'] = f64(0.3)
			m['color'] = f64(0xFFFFFF)
		}
		'camera' {
			m['fov'] = f64(50.0)
			m['aspect'] = f64(16.0 / 9.0)
			m['position'] = CgsVec3{0.0, 0.0, 0.0}
			m['target'] = CgsVec3{0.0, 0.0, 0.0}
		}
		'material' {
			m['color'] = f64(0xFFFFFF)
			m['roughness'] = f64(-1.0)
			m['metalness'] = f64(-1.0)
			m['emissive'] = f64(-1.0)
			m['opacity'] = f64(-1.0)
			m['ior'] = f64(-1.0)
			m['absorption'] = f64(-1.0)
			m['unlit'] = false
			m['map'] = string('')
		}
		else {}
	}
	return m
}

fn (mut l SceneLoader) add_geometry(geo Geometry, ctx [16]f64, mat map[string]CgsValue) ! {
	if l.collecting {
		l.collect << CollectedGeom{
			geo: geo
			m4:  ctx
		}
		return
	}
	motor, lin := decompose_rigid(ctx)
	g2 := if is_identity3(lin) { geo } else { affine_geometry(geo, lin) }
	l.scene.add_mesh(mesh(MeshParams{
		geometry:       g2
		material:       l.build_material(mat)!
		position:       [
			0.0,
			0.0,
			0.0,
		]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          motor
	}))
}

fn (mut l SceneLoader) csg_block(op CsgOp, ctx [16]f64, mat map[string]CgsValue, mut scope map[string]CgsValue, line int) ! {
	saved := l.collect
	was_collecting := l.collecting
	l.collect = []CollectedGeom{}
	l.collecting = true
	l.body(ctx, mat, mut scope, line)!
	children := l.collect
	l.collect = saved
	l.collecting = was_collecting
	if children.len < 2 {
		return error('CGS line ${line}: ${op} needs >= 2 geometry children')
	}
	mut kids := []Geometry{}
	for c in children {
		cm, cl := decompose_rigid(c.m4)
		kids << transformed_geometry(c.geo, cm, cl)
	}
	l.scene.add_mesh(mesh(MeshParams{
		geometry:       csg_geometry(op, kids)
		material:       l.build_material(mat)!
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [
			0.0,
			0.0,
			1.0,
		]!
		rotation_angle: 0.0
		motor:          motor_identity()
	}))
}

fn (mut l SceneLoader) build_geometry(name string, args map[string]CgsValue, line int) !Geometry {
	match name {
		'sphere' {
			r := cgs_num(args['r'] or { f64(0.0) }, line, 'sphere.r') or { return error(err.msg()) }
			return sphere_geometry(r)
		}
		'plane' {
			n := cgs_vec3(args['n'] or { f64(0.0) }, line, 'plane.n') or { return error(err.msg()) }
			d := cgs_num(args['d'] or { f64(0.0) }, line, 'plane.d') or { return error(err.msg()) }
			return plane_geometry(n, d)
		}
		'cylinder' {
			h := cgs_num(args['h'] or { f64(0.0) }, line, 'cylinder.h') or {
				return error(err.msg())
			}
			r := cgs_num(args['r'] or { f64(0.0) }, line, 'cylinder.r') or {
				return error(err.msg())
			}
			return cylinder_geometry(r, if h < 0.0 { -1.0 } else { h })
		}
		'box' {
			s := cgs_vec3(args['s'] or { f64(0.0) }, line, 'box.s') or { return error(err.msg()) }
			return box_geometry(s[0], s[1], s[2])
		}
		'circle' {
			r := cgs_num(args['r'] or { f64(0.0) }, line, 'circle.r') or { return error(err.msg()) }
			return circle_geometry(r)
		}
		'cone' {
			r := cgs_num(args['r'] or { f64(0.0) }, line, 'cone.r') or { return error(err.msg()) }
			h := cgs_num(args['h'] or { f64(0.0) }, line, 'cone.h') or { return error(err.msg()) }
			return cone_geometry(r, h)
		}
		'torus' {
			r1 := cgs_num(args['R'] or { f64(0.0) }, line, 'torus.R') or { return error(err.msg()) }
			r2 := cgs_num(args['r'] or { f64(0.0) }, line, 'torus.r') or { return error(err.msg()) }
			return torus_geometry(r1, r2)
		}
		'cyclide' {
			a := cgs_num(args['a'] or { f64(0.0) }, line, 'cyclide.a') or {
				return error(err.msg())
			}
			b := cgs_num(args['b'] or { f64(0.0) }, line, 'cyclide.b') or {
				return error(err.msg())
			}
			d := cgs_num(args['d'] or { f64(0.0) }, line, 'cyclide.d') or {
				return error(err.msg())
			}
			return cyclide_geometry(a, b, d, [0.0, 0.0, 0.0]!)
		}
		'ellipsoid' {
			r := cgs_vec3(args['radii'] or { f64(0.0) }, line, 'ellipsoid.radii') or {
				return error(err.msg())
			}
			return ellipsoid_geometry(r[0], r[1], r[2])
		}
		'extrude' {
			prof := l.profile2d(args['profile'] or { f64(0.0) }, line, 'extrude.profile')!
			h := cgs_num(args['h'] or { f64(0.0) }, line, 'extrude.h') or {
				return error(err.msg())
			}
			v, f := extrude(prof, h)
			return trimesh_geometry(v, f)
		}
		'loft' {
			raw := args['profiles'] or { f64(0.0) }
			match raw {
				[]CgsValue {
					if raw.len < 2 {
						return error('CGS line ${line}: loft.profiles needs >= 2 sections')
					}
					mut profiles := [][][2]f64{}
					for p in raw {
						profiles << l.profile2d(p, line, 'loft.profiles[i]')!
					}
					zsraw := args['zs'] or { f64(0.0) }
					match zsraw {
						CgsVec3 {
							v, f := loft(profiles, [zsraw.x, zsraw.y, zsraw.z])
							return trimesh_geometry(v, f)
						}
						[]CgsValue {
							mut zs := []f64{}
							for z in zsraw {
								zv := cgs_num(z, line, 'loft.zs[i]') or { return error(err.msg()) }
								zs << zv
							}
							v, f := loft(profiles, zs)
							return trimesh_geometry(v, f)
						}
						else {
							return error('CGS line ${line}: loft.zs needs a list')
						}
					}
				}
				else {
					return error('CGS line ${line}: loft.profiles needs a list')
				}
			}
		}
		'mesh' {
			p := args['file'] or { f64(0.0) }
			match p {
				string {
					if l.asset_root == '' {
						return error('CGS line ${line}: mesh needs an explicit asset_root')
					}
					full := l.asset_root + '/' + p
					if full.ends_with('.obj') {
						v, f := load_obj(full) or { return error(err.msg()) }
						return trimesh_geometry(v, f)
					}
					if full.ends_with('.glb') || full.ends_with('.gltf') {
						loaded := load_gltf(full) or { return error(err.msg()) }
						return gltf_to_geometry(loaded)
					}
					return error('CGS line ${line}: unsupported mesh file "${p}" (use .obj/.glb/.gltf)')
				}
				else {
					return error('CGS line ${line}: mesh.file needs a string path')
				}
			}
		}
		else {
			return error('CGS line ${line}: unknown primitive ${name}')
		}
	}
}

fn (mut l SceneLoader) profile2d(v CgsValue, line int, what string) ![][2]f64 {
	match v {
		[]CgsValue {
			if v.len < 3 {
				return error('CGS line ${line}: ${what} needs >= 3 [x,y] points')
			}
			mut pts := [][2]f64{}
			for p in v {
				match p {
					[]CgsValue {
						if p.len != 2 {
							return error('CGS line ${line}: ${what} items must be [x,y]')
						}
						x := cgs_num(p[0], line, what) or { return error(err.msg()) }
						y := cgs_num(p[1], line, what) or { return error(err.msg()) }
						pts << [x, y]!
					}
					else {
						return error('CGS line ${line}: ${what} items must be [x,y]')
					}
				}
			}
			return pts
		}
		else {
			return error('CGS line ${line}: ${what} needs a list')
		}
	}
}

fn (mut l SceneLoader) build_material(mat map[string]CgsValue) !Material {
	color := if v := mat['color'] {
		c := cgs_num(v, 0, 'color') or { return error(err.msg()) }
		color_hex(int(c))
	} else {
		color_hex(0xFFFFFF)
	}
	mut tex := ?Texture(none)
	if v := mat['map'] {
		match v {
			string {
				if v != '' {
					if l.asset_root == '' {
						return error('CGS material.map needs an explicit asset_root')
					}
					tex = texture_load(l.asset_root + '/' + v) or { return error(err.msg()) }
				}
			}
			else {
				return error('CGS material.map needs a string path')
			}
		}
	}
	if u := mat['unlit'] {
		if cgs_truthy(u) {
			op := if o := mat['opacity'] {
				cgs_num(o, 0, 'opacity') or { return error(err.msg()) }
			} else {
				1.0
			}
			return basic_material(color, clamp01(op))
		}
	}
	roughness := cgs_opt_num(mat['roughness'] or { f64(-1.0) }, 0.5)
	metalness := cgs_opt_num(mat['metalness'] or { f64(-1.0) }, 0.0)
	emissive := if v := mat['emissive'] {
		ev := cgs_num(v, 0, 'emissive') or { return error(err.msg()) }
		if ev < 0.0 {
			color_hex(0x000000)
		} else {
			color_hex(int(ev))
		}
	} else {
		color_hex(0x000000)
	}
	opacity := cgs_opt_num(mat['opacity'] or { f64(-1.0) }, 1.0)
	ior := cgs_opt_num(mat['ior'] or { f64(-1.0) }, 1.5)
	absorption := cgs_opt_num(mat['absorption'] or { f64(-1.0) }, 0.0)
	mut m := standard_material(MaterialParams{
		color:      color
		roughness:  roughness
		metalness:  metalness
		emissive:   emissive
		opacity:    opacity
		ior:        ior
		absorption: absorption
	})
	if t := tex {
		m.map = t
	}
	return m
}

fn (mut l SceneLoader) add_light(name string, args map[string]CgsValue, line int) ! {
	c := cgs_num(args['color'] or { f64(0.0) }, line, 'color') or { return error(err.msg()) }
	color := color_hex(int(c))
	intensity := cgs_num(args['intensity'] or { f64(0.0) }, line, 'intensity') or {
		return error(err.msg())
	}
	if name == 'directional_light' {
		d := cgs_vec3(args['direction'] or { f64(0.0) }, line, 'direction') or {
			return error(err.msg())
		}
		l.scene.add_light(directional_light(color, intensity, d))
	} else if name == 'point_light' {
		p := cgs_vec3(args['position'] or { f64(0.0) }, line, 'position') or {
			return error(err.msg())
		}
		l.scene.add_light(point_light(color, intensity, p))
	} else {
		l.scene.add_light(ambient_light(color, intensity))
	}
}
