// CGS (CGA Scene) language: lexer + single-pass parser/evaluator producing an
// engine Scene + PerspectiveCamera.  Port of cga_py/scene_lang.
//
// Supports translate/rotate/scale/mirror modifiers, for/if/echo/module,
// variables/expressions/math functions, all primitives, lights, camera,
// background, CSG (difference/intersection/union), and
// extrude/loft/mesh(.obj/.glb/.gltf).

use std::collections::HashMap;
use std::fmt;

use cga_core::{
    affine_geometry, box_geometry, circle_geometry, clamp01, cone_geometry, csg_geometry,
    cyclide_geometry, cylinder_geometry, decompose_rigid, ellipsoid_geometry, extrude, load_obj,
    loft, mat4_identity, mat4_mul, motor_identity, motor_rotor, plane_geometry, sphere_geometry,
    torus_geometry, transformed_geometry, trimesh_geometry, validate_profile, CsgOp, Geometry,
};

use crate::mesh_io_gltf::{gltf_to_geometry, load_gltf};
use crate::scene::{mesh, perspective_camera, scene, MeshParams, PerspectiveCamera, Scene};
use crate::scene_graph::{color_hex, is_identity3, vec3_dot};
use crate::shading::{
    ambient_light, basic_material, directional_light, point_light, standard_material, Material,
    MaterialParams,
};
use crate::texture::texture_load;

// TokenKind classifies a lexed token.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TokenKind {
    Ident,
    Number,
    Op,
    Str,
    Eof,
    Lparen,
    Rparen,
    Lbracket,
    Rbracket,
    Lbrace,
    Rbrace,
    Comma,
    Semi,
    Assign,
}

impl fmt::Display for TokenKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            TokenKind::Ident => "ident",
            TokenKind::Number => "number",
            TokenKind::Op => "op",
            TokenKind::Str => "str",
            TokenKind::Eof => "eof",
            TokenKind::Lparen => "lparen",
            TokenKind::Rparen => "rparen",
            TokenKind::Lbracket => "lbracket",
            TokenKind::Rbracket => "rbracket",
            TokenKind::Lbrace => "lbrace",
            TokenKind::Rbrace => "rbrace",
            TokenKind::Comma => "comma",
            TokenKind::Semi => "semi",
            TokenKind::Assign => "assign",
        };
        f.write_str(s)
    }
}

// punct_kind maps a single-character punctuation byte to its token kind.
fn punct_kind(ch: u8) -> TokenKind {
    match ch {
        b'(' => TokenKind::Lparen,
        b')' => TokenKind::Rparen,
        b'[' => TokenKind::Lbracket,
        b']' => TokenKind::Rbracket,
        b'{' => TokenKind::Lbrace,
        b'}' => TokenKind::Rbrace,
        b',' => TokenKind::Comma,
        b';' => TokenKind::Semi,
        b'=' => TokenKind::Assign,
        _ => TokenKind::Eof,
    }
}

// CgsToken is one lexed token.
#[derive(Clone, Debug)]
pub struct CgsToken {
    pub kind: TokenKind,
    pub text: String,
    pub num: f64,
    pub line: i32,
}

fn is_letter(ch: u8) -> bool {
    ch.is_ascii_lowercase() || ch.is_ascii_uppercase()
}

fn is_alnum(ch: u8) -> bool {
    is_letter(ch) || ch.is_ascii_digit()
}

fn is_hex_digit(ch: u8) -> bool {
    ch.is_ascii_digit() || (b'a'..=b'f').contains(&ch) || (b'A'..=b'F').contains(&ch)
}

// lex tokenises CGS source text.
pub fn cgs_lex(text: &str) -> Result<Vec<CgsToken>, String> {
    let b = text.as_bytes();
    let mut toks: Vec<CgsToken> = Vec::new();
    let mut i = 0usize;
    let mut ln = 1i32;
    let n = b.len();
    while i < n {
        let ch = b[i];
        if ch == b'\n' {
            ln += 1;
            i += 1;
        } else if ch == b' ' || ch == b'\t' || ch == b'\r' {
            i += 1;
        } else if b[i..].starts_with(b"//") {
            while i < n && b[i] != b'\n' {
                i += 1;
            }
        } else if i + 1 < n && matches!(&b[i..i + 2], b"==" | b"!=" | b"<=" | b">=" | b"&&" | b"||")
        {
            toks.push(CgsToken {
                kind: TokenKind::Op,
                text: String::from_utf8_lossy(&b[i..i + 2]).into_owned(),
                num: 0.0,
                line: ln,
            });
            i += 2;
        } else if matches!(
            ch,
            b'+' | b'-' | b'*' | b'/' | b'%' | b'<' | b'>' | b'!' | b':'
        ) {
            toks.push(CgsToken {
                kind: TokenKind::Op,
                text: (ch as char).to_string(),
                num: 0.0,
                line: ln,
            });
            i += 1;
        } else if matches!(
            ch,
            b'[' | b']' | b'{' | b'}' | b',' | b';' | b'=' | b'(' | b')'
        ) {
            toks.push(CgsToken {
                kind: punct_kind(ch),
                text: (ch as char).to_string(),
                num: 0.0,
                line: ln,
            });
            i += 1;
        } else if ch == b'"' || ch == b'\'' {
            let quote = ch;
            let mut j = i + 1;
            while j < n && b[j] != quote {
                if b[j] == b'\\' {
                    j += 1;
                }
                j += 1;
            }
            if j >= n {
                return Err(format!("CGS line {ln}: unclosed string"));
            }
            toks.push(CgsToken {
                kind: TokenKind::Str,
                text: String::from_utf8_lossy(&b[i + 1..j]).into_owned(),
                num: 0.0,
                line: ln,
            });
            i = j + 1;
        } else if is_letter(ch) || ch == b'_' {
            let mut j = i + 1;
            while j < n && (is_alnum(b[j]) || b[j] == b'_') {
                j += 1;
            }
            toks.push(CgsToken {
                kind: TokenKind::Ident,
                text: String::from_utf8_lossy(&b[i..j]).into_owned(),
                num: 0.0,
                line: ln,
            });
            i = j;
        } else if ch.is_ascii_digit() {
            if b[i..].starts_with(b"0x") || b[i..].starts_with(b"0X") {
                let mut j = i + 2;
                while j < n && is_hex_digit(b[j]) {
                    j += 1;
                }
                if j == i + 2 {
                    return Err(format!("CGS line {ln}: illegal hex colour"));
                }
                let num = u32::from_str_radix(&text[i + 2..j], 16)
                    .map(|v| v as f64)
                    .unwrap_or(0.0);
                toks.push(CgsToken {
                    kind: TokenKind::Number,
                    text: String::new(),
                    num,
                    line: ln,
                });
                i = j;
            } else {
                let mut j = i;
                while j < n && b[j].is_ascii_digit() {
                    j += 1;
                }
                if j < n && b[j] == b'.' {
                    j += 1;
                    while j < n && b[j].is_ascii_digit() {
                        j += 1;
                    }
                }
                if j < n && (b[j] == b'e' || b[j] == b'E') {
                    j += 1;
                    if j < n && (b[j] == b'+' || b[j] == b'-') {
                        j += 1;
                    }
                    while j < n && b[j].is_ascii_digit() {
                        j += 1;
                    }
                }
                toks.push(CgsToken {
                    kind: TokenKind::Number,
                    text: String::new(),
                    num: text[i..j].parse::<f64>().unwrap_or(0.0),
                    line: ln,
                });
                i = j;
            }
        } else {
            return Err(format!("CGS line {ln}: illegal character {}", ch as char));
        }
    }
    Ok(toks)
}

// CgsVec3 is a 3-vector value in the CGS language (a `[x,y,z]` literal).
#[derive(Clone, Copy, Debug)]
pub struct CgsVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

// CgsValue is the dynamic value of the language (number / bool / string / list /
// 3-vector).  A `[x,y,z]` literal with three numeric elements is a CgsVec3;
// ranges and other lists stay `Vec<CgsValue>`.
#[derive(Clone, Debug)]
pub enum CgsValue {
    Num(f64),
    Bool(bool),
    Str(String),
    List(Vec<CgsValue>),
    Vec3(CgsVec3),
}

impl PartialEq for CgsValue {
    fn eq(&self, other: &CgsValue) -> bool {
        match (self, other) {
            (CgsValue::Num(a), CgsValue::Num(b)) => a == b,
            (CgsValue::Bool(a), CgsValue::Bool(b)) => a == b,
            (CgsValue::Str(a), CgsValue::Str(b)) => a == b,
            (CgsValue::List(a), CgsValue::List(b)) => a == b,
            (CgsValue::Vec3(a), CgsValue::Vec3(b)) => a.x == b.x && a.y == b.y && a.z == b.z,
            _ => false,
        }
    }
}

// fmt_f64 mirrors V's `${f64}` interpolation style (integral values print with
// a trailing ".0").
fn fmt_f64(x: f64) -> String {
    if x.is_finite() && x.fract() == 0.0 && x.abs() < 1e16 {
        format!("{x:.1}")
    } else {
        format!("{x}")
    }
}

impl fmt::Display for CgsValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CgsValue::Num(x) => f.write_str(&fmt_f64(*x)),
            CgsValue::Bool(v) => f.write_str(if *v { "true" } else { "false" }),
            CgsValue::Str(s) => f.write_str(s),
            CgsValue::List(items) => {
                let parts: Vec<String> = items.iter().map(|v| format!("{v}")).collect();
                write!(f, "[{}]", parts.join(", "))
            }
            CgsValue::Vec3(v) => {
                write!(f, "[{}, {}, {}]", fmt_f64(v.x), fmt_f64(v.y), fmt_f64(v.z))
            }
        }
    }
}

fn cgs_num(v: &CgsValue, line: i32, what: &str) -> Result<f64, String> {
    match v {
        CgsValue::Num(x) => Ok(*x),
        _ => Err(format!("CGS line {line}: {what} needs a number, got {v}")),
    }
}

fn cgs_vec3(v: &CgsValue, line: i32, what: &str) -> Result<[f64; 3], String> {
    match v {
        CgsValue::Vec3(v3) => Ok([v3.x, v3.y, v3.z]),
        CgsValue::List(items) => {
            if items.len() != 3 {
                return Err(format!("CGS line {line}: {what} needs [x,y,z], got {v}"));
            }
            Ok([
                cgs_num(&items[0], line, what)?,
                cgs_num(&items[1], line, what)?,
                cgs_num(&items[2], line, what)?,
            ])
        }
        _ => Err(format!("CGS line {line}: {what} needs [x,y,z], got {v}")),
    }
}

fn cgs_opt_num(v: &CgsValue, def: f64) -> f64 {
    match v {
        CgsValue::Num(x) => {
            if *x < 0.0 {
                def
            } else {
                *x
            }
        }
        _ => def,
    }
}

fn translate4(t: [f64; 3]) -> [f64; 16] {
    [
        1.0, 0.0, 0.0, t[0], 0.0, 1.0, 0.0, t[1], 0.0, 0.0, 1.0, t[2], 0.0, 0.0, 0.0, 1.0,
    ]
}

fn scale4(s: [f64; 3]) -> [f64; 16] {
    [
        s[0], 0.0, 0.0, 0.0, 0.0, s[1], 0.0, 0.0, 0.0, 0.0, s[2], 0.0, 0.0, 0.0, 0.0, 1.0,
    ]
}

fn mirror4(ax: [f64; 3]) -> Result<[f64; 16], String> {
    let n = (ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]).sqrt();
    if n < 1e-12 {
        return Err("mirror.axis must not be zero".to_string());
    }
    let ux = ax[0] / n;
    let uy = ax[1] / n;
    let uz = ax[2] / n;
    Ok([
        1.0 - 2.0 * ux * ux,
        -2.0 * ux * uy,
        -2.0 * ux * uz,
        0.0,
        -2.0 * uy * ux,
        1.0 - 2.0 * uy * uy,
        -2.0 * uy * uz,
        0.0,
        -2.0 * uz * ux,
        -2.0 * uz * uy,
        1.0 - 2.0 * uz * uz,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ])
}

fn cgs_truthy(v: &CgsValue) -> bool {
    match v {
        CgsValue::List(items) => !items.is_empty(),
        CgsValue::Vec3(_) => true,
        CgsValue::Bool(b) => *b,
        CgsValue::Num(x) => *x != 0.0,
        CgsValue::Str(s) => !s.is_empty(),
    }
}

fn cgs_neg(v: &CgsValue, line: i32) -> Result<CgsValue, String> {
    match v {
        CgsValue::Num(x) => Ok(CgsValue::Num(-x)),
        CgsValue::Vec3(v3) => Ok(CgsValue::Vec3(CgsVec3 {
            x: -v3.x,
            y: -v3.y,
            z: -v3.z,
        })),
        CgsValue::List(items) => {
            let mut out: Vec<CgsValue> = Vec::new();
            for x in items {
                out.push(cgs_neg(x, line)?);
            }
            Ok(CgsValue::List(out))
        }
        _ => Err(format!("CGS line {line}: unary minus needs number/vector")),
    }
}

// cgs_as_list normalises a 3-vector to a 3-element numeric list, so vector and
// list arithmetic share one elementwise path.
fn cgs_as_list(v: CgsValue) -> CgsValue {
    match v {
        CgsValue::Vec3(v3) => CgsValue::List(vec![
            CgsValue::Num(v3.x),
            CgsValue::Num(v3.y),
            CgsValue::Num(v3.z),
        ]),
        _ => v,
    }
}

// BinOp is a binary operator in a CGS expression.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum BinOp {
    Or,
    And,
    Eq,
    Ne,
    Lt,
    Le,
    Gt,
    Ge,
    Add,
    Sub,
    Mul,
    Div,
    Mod,
}

// cgs_binop_from_text maps an operator token's text to a BinOp.
fn cgs_binop_from_text(text: &str) -> Option<BinOp> {
    match text {
        "||" => Some(BinOp::Or),
        "&&" => Some(BinOp::And),
        "==" => Some(BinOp::Eq),
        "!=" => Some(BinOp::Ne),
        "<" => Some(BinOp::Lt),
        "<=" => Some(BinOp::Le),
        ">" => Some(BinOp::Gt),
        ">=" => Some(BinOp::Ge),
        "+" => Some(BinOp::Add),
        "-" => Some(BinOp::Sub),
        "*" => Some(BinOp::Mul),
        "/" => Some(BinOp::Div),
        "%" => Some(BinOp::Mod),
        _ => None,
    }
}

fn cgs_precedence(op: BinOp) -> i32 {
    match op {
        BinOp::Or => 1,
        BinOp::And => 2,
        BinOp::Eq | BinOp::Ne => 3,
        BinOp::Lt | BinOp::Le | BinOp::Gt | BinOp::Ge => 4,
        BinOp::Add | BinOp::Sub => 5,
        BinOp::Mul | BinOp::Div | BinOp::Mod => 6,
    }
}

fn cgs_scalar_arith(op: BinOp, a: f64, b: f64) -> f64 {
    match op {
        BinOp::Add => a + b,
        BinOp::Sub => a - b,
        BinOp::Mul => a * b,
        BinOp::Div => a / b,
        BinOp::Mod => a % b,
        _ => panic!("bad op {op:?}"),
    }
}

fn cgs_binop(op: BinOp, a: CgsValue, b: CgsValue) -> Result<CgsValue, String> {
    if op == BinOp::Eq {
        return Ok(CgsValue::Bool(a == b));
    }
    if op == BinOp::Ne {
        return Ok(CgsValue::Bool(a != b));
    }
    if op == BinOp::And {
        return Ok(CgsValue::Bool(cgs_truthy(&a) && cgs_truthy(&b)));
    }
    if op == BinOp::Or {
        return Ok(CgsValue::Bool(cgs_truthy(&a) || cgs_truthy(&b)));
    }
    if matches!(op, BinOp::Lt | BinOp::Le | BinOp::Gt | BinOp::Ge) {
        let aa = cgs_num(&a, 0, "cmp")?;
        let bb = cgs_num(&b, 0, "cmp")?;
        return Ok(CgsValue::Bool(match op {
            BinOp::Lt => aa < bb,
            BinOp::Le => aa <= bb,
            BinOp::Gt => aa > bb,
            _ => aa >= bb,
        }));
    }
    // arithmetic: scalar or elementwise vector (scalar broadcast)
    let al = cgs_as_list(a);
    let bl = cgs_as_list(b);
    match al {
        CgsValue::List(alist) => match bl {
            CgsValue::List(blist) => {
                if alist.len() != blist.len() {
                    return Err(format!(
                        "CGS: vector length mismatch ({} vs {})",
                        alist.len(),
                        blist.len()
                    ));
                }
                let mut out: Vec<CgsValue> = Vec::new();
                for i in 0..alist.len() {
                    out.push(cgs_binop(op, alist[i].clone(), blist[i].clone())?);
                }
                Ok(CgsValue::List(out))
            }
            _ => {
                let mut out: Vec<CgsValue> = Vec::new();
                for x in alist {
                    out.push(cgs_binop(op, x, bl.clone())?);
                }
                Ok(CgsValue::List(out))
            }
        },
        _ => match bl {
            CgsValue::List(blist) => {
                let mut out: Vec<CgsValue> = Vec::new();
                for x in blist {
                    out.push(cgs_binop(op, al.clone(), x)?);
                }
                Ok(CgsValue::List(out))
            }
            _ => Ok(CgsValue::Num(cgs_scalar_arith(
                op,
                cgs_num(&al, 0, "arith")?,
                cgs_num(&bl, 0, "arith")?,
            ))),
        },
    }
}

fn cgs_call_fn(name: &str, args: &[CgsValue], line: i32) -> Result<CgsValue, String> {
    if name == "len" {
        let a0 = &args[0];
        return match a0 {
            CgsValue::List(items) => Ok(CgsValue::Num(items.len() as f64)),
            CgsValue::Vec3(_) => Ok(CgsValue::Num(3.0)),
            _ => Err(format!("CGS line {line}: len needs a list")),
        };
    }
    if name == "norm" {
        let a0 = &args[0];
        return match a0 {
            CgsValue::Vec3(v3) => Ok(CgsValue::Num(
                (v3.x * v3.x + v3.y * v3.y + v3.z * v3.z).sqrt(),
            )),
            CgsValue::List(items) => {
                let mut s = 0.0;
                for x in items {
                    s += cgs_num(x, line, "norm")? * cgs_num(x, line, "norm")?;
                }
                Ok(CgsValue::Num(s.sqrt()))
            }
            _ => Err(format!("CGS line {line}: norm needs a vector")),
        };
    }
    if name == "cross" {
        let a = cgs_vec3(&args[0], line, "cross.a")?;
        let b = cgs_vec3(&args[1], line, "cross.b")?;
        return Ok(CgsValue::List(vec![
            CgsValue::Num(a[1] * b[2] - a[2] * b[1]),
            CgsValue::Num(a[2] * b[0] - a[0] * b[2]),
            CgsValue::Num(a[0] * b[1] - a[1] * b[0]),
        ]));
    }
    // unary math functions
    if args.len() == 1 {
        let x = cgs_num(&args[0], line, name)?;
        let v = match name {
            "abs" => x.abs(),
            "sign" => {
                if x > 0.0 {
                    1.0
                } else if x < 0.0 {
                    -1.0
                } else {
                    0.0
                }
            }
            "sin" => x.sin(),
            "cos" => x.cos(),
            "tan" => x.tan(),
            "asin" => x.asin(),
            "acos" => x.acos(),
            "atan" => x.atan(),
            "sqrt" => x.sqrt(),
            "exp" => x.exp(),
            "ln" => x.ln(),
            "log" => x.log10(),
            "floor" => x.floor(),
            "ceil" => x.ceil(),
            "round" => x.round(),
            _ => return Err(format!("CGS line {line}: unknown function {name}")),
        };
        return Ok(CgsValue::Num(v));
    }
    if args.len() == 2 {
        let a = cgs_num(&args[0], line, name)?;
        let b = cgs_num(&args[1], line, name)?;
        return Ok(CgsValue::Num(match name {
            "atan2" => a.atan2(b),
            "pow" => a.powf(b),
            "min" => a.min(b),
            "max" => a.max(b),
            _ => return Err(format!("CGS line {line}: unknown function {name}")),
        }));
    }
    Err(format!("CGS line {line}: {name} wrong arity"))
}

// SceneLoader parses CGS text.
pub struct SceneLoader {
    toks: Vec<CgsToken>,
    pos: usize,
    asset_root: String,
    scene: Scene,
    camera: Option<PerspectiveCamera>,
    modules: HashMap<String, Vec<CgsToken>>,
    params: HashMap<String, Vec<CgsToken>>, // module formal parameters (name -> body tokens)
    param_order: Vec<String>,               // params keys in insertion order (V maps are ordered)
    collect: Vec<CollectedGeom>,
    collecting: bool,
}

struct CollectedGeom {
    geo: Geometry,
    m4: [f64; 16],
}

// cgs_load parses CGS text into (Scene, PerspectiveCamera), panicking on error.
pub fn cgs_load(text: &str, asset_root: &str) -> (Scene, PerspectiveCamera) {
    match cgs_load_result(text, asset_root) {
        Ok(r) => r,
        Err(e) => panic!("{e}"),
    }
}

// cgs_load_result parses CGS text, returning the first error instead of
// panicking (so callers such as the render server can report it cleanly).
pub fn cgs_load_result(text: &str, asset_root: &str) -> Result<(Scene, PerspectiveCamera), String> {
    let toks = cgs_lex(text)?;
    let mut l = SceneLoader {
        toks,
        pos: 0,
        asset_root: asset_root.to_string(),
        scene: scene(None),
        camera: None,
        modules: HashMap::new(),
        params: HashMap::new(),
        param_order: Vec::new(),
        collect: Vec::new(),
        collecting: false,
    };
    let mut root_scope: HashMap<String, CgsValue> = HashMap::new();
    root_scope.insert("pi".to_string(), CgsValue::Num(std::f64::consts::PI));
    let toks = l.toks.clone();
    l.run_tokens(toks, mat4_identity(), &HashMap::new(), &mut root_scope)?;
    let cam = match l.camera {
        Some(c) => c,
        None => {
            let mut c2 = perspective_camera(
                50.0,
                16.0 / 9.0,
                0.1,
                100.0,
                [0.0, 0.0, 5.0],
                [0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            );
            c2.look_at([0.0, 0.0, 0.0], None);
            c2
        }
    };
    Ok((l.scene, cam))
}

impl SceneLoader {
    fn peek(&self) -> CgsToken {
        if self.pos >= self.toks.len() {
            return CgsToken {
                kind: TokenKind::Eof,
                text: String::new(),
                num: 0.0,
                line: 1,
            };
        }
        self.toks[self.pos].clone()
    }

    fn peek1(&self) -> CgsToken {
        if self.pos + 1 >= self.toks.len() {
            return CgsToken {
                kind: TokenKind::Eof,
                text: String::new(),
                num: 0.0,
                line: 1,
            };
        }
        self.toks[self.pos + 1].clone()
    }

    fn take(&mut self) -> CgsToken {
        let t = self.peek();
        self.pos += 1;
        t
    }

    fn expect(&mut self, sym: TokenKind) -> Result<(), String> {
        let t = self.take();
        if t.kind != sym {
            return Err(format!(
                "CGS line {}: expected {}, got {}",
                t.line, sym, t.kind
            ));
        }
        Ok(())
    }

    fn run_tokens(
        &mut self,
        toks: Vec<CgsToken>,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        let saved = std::mem::replace(&mut self.toks, toks);
        let saved_pos = self.pos;
        self.pos = 0;
        while self.pos < self.toks.len() {
            self.statement(ctx, mat, scope)?;
        }
        self.toks = saved;
        self.pos = saved_pos;
        Ok(())
    }

    // --- expression parser (precedence climbing) --------------------------------

    fn expr(
        &mut self,
        scope: &HashMap<String, CgsValue>,
        min_prec: i32,
    ) -> Result<CgsValue, String> {
        let mut lhs = self.unary(scope)?;
        loop {
            let t = self.peek();
            if t.kind != TokenKind::Op {
                return Ok(lhs);
            }
            let op = match cgs_binop_from_text(&t.text) {
                Some(op) => op,
                None => return Ok(lhs),
            };
            let prec = cgs_precedence(op);
            if prec < min_prec {
                return Ok(lhs);
            }
            self.take();
            let rhs = self.expr(scope, prec + 1)?;
            lhs = cgs_binop(op, lhs, rhs)?;
        }
    }

    fn unary(&mut self, scope: &HashMap<String, CgsValue>) -> Result<CgsValue, String> {
        let t = self.peek();
        if t.kind == TokenKind::Op && t.text == "-" {
            self.take();
            let v = self.expr(scope, 7)?;
            return cgs_neg(&v, t.line);
        }
        if t.kind == TokenKind::Op && t.text == "!" {
            self.take();
            let v = self.expr(scope, 7)?;
            return Ok(CgsValue::Bool(!cgs_truthy(&v)));
        }
        self.primary(scope)
    }

    fn primary(&mut self, scope: &HashMap<String, CgsValue>) -> Result<CgsValue, String> {
        let t = self.take();
        if t.kind == TokenKind::Number {
            return Ok(CgsValue::Num(t.num));
        }
        if t.kind == TokenKind::Lparen {
            let v = self.expr(scope, 1)?;
            self.expect(TokenKind::Rparen)?;
            return Ok(v);
        }
        if t.kind == TokenKind::Lbracket {
            return self.list_literal(scope, t.line);
        }
        if t.kind == TokenKind::Str {
            return Ok(CgsValue::Str(t.text));
        }
        if t.kind == TokenKind::Ident {
            if self.peek().kind == TokenKind::Lparen {
                self.take();
                let mut args: Vec<CgsValue> = Vec::new();
                if self.peek().kind != TokenKind::Rparen {
                    loop {
                        args.push(self.expr(scope, 1)?);
                        if self.peek().kind == TokenKind::Comma {
                            self.take();
                        } else {
                            break;
                        }
                    }
                }
                self.expect(TokenKind::Rparen)?;
                return cgs_call_fn(&t.text, &args, t.line);
            }
            if t.text == "true" {
                return Ok(CgsValue::Bool(true));
            }
            if t.text == "false" {
                return Ok(CgsValue::Bool(false));
            }
            if let Some(v) = scope.get(&t.text) {
                return Ok(v.clone());
            }
            return Err(format!(
                "CGS line {}: undefined variable {}",
                t.line, t.text
            ));
        }
        Err(format!(
            "CGS line {}: bad expression start {}",
            t.line, t.kind
        ))
    }

    fn list_literal(
        &mut self,
        scope: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<CgsValue, String> {
        let first = self.expr(scope, 1)?;
        if self.peek().kind == TokenKind::Op && self.peek().text == ":" {
            self.take();
            let second = self.expr(scope, 1)?;
            let mut step = 1.0;
            let mut start_v = cgs_num(&first, line, "range")?;
            let mut stop_v = cgs_num(&second, line, "range")?;
            if self.peek().kind == TokenKind::Op && self.peek().text == ":" {
                self.take();
                let third = self.expr(scope, 1)?;
                start_v = cgs_num(&first, line, "range")?;
                step = cgs_num(&second, line, "range")?;
                stop_v = cgs_num(&third, line, "range")?;
            }
            self.expect(TokenKind::Rbracket)?;
            if step == 0.0 {
                return Err(format!("CGS line {line}: range step must not be 0"));
            }
            let mut out: Vec<CgsValue> = Vec::new();
            let mut v = start_v;
            if step > 0.0 {
                while v <= stop_v + 1e-12 {
                    out.push(CgsValue::Num(v));
                    v += step;
                }
            } else {
                while v >= stop_v - 1e-12 {
                    out.push(CgsValue::Num(v));
                    v += step;
                }
            }
            return Ok(CgsValue::List(out));
        }
        let mut items = vec![first];
        while self.peek().kind == TokenKind::Comma {
            self.take();
            items.push(self.expr(scope, 1)?);
        }
        self.expect(TokenKind::Rbracket)?;
        // A 3-element numeric literal is a vector (CgsVec3); anything else is a list.
        if items.len() == 3 {
            if let (CgsValue::Num(x), CgsValue::Num(y), CgsValue::Num(z)) =
                (&items[0], &items[1], &items[2])
            {
                return Ok(CgsValue::Vec3(CgsVec3 {
                    x: *x,
                    y: *y,
                    z: *z,
                }));
            }
        }
        Ok(CgsValue::List(items))
    }

    // --- statements -------------------------------------------------------------

    fn statement(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        let t = self.peek();
        if t.kind == TokenKind::Lbrace {
            self.expect(TokenKind::Lbrace)?;
            while self.peek().kind != TokenKind::Rbrace {
                self.statement(ctx, mat, scope)?;
            }
            self.expect(TokenKind::Rbrace)?;
            return Ok(());
        }
        if t.kind == TokenKind::Ident {
            let name = t.text.as_str();
            if name == "module" {
                self.module_def()?;
                return Ok(());
            }
            if name == "for" {
                self.for_loop(ctx, mat, scope)?;
                return Ok(());
            }
            if name == "if" {
                self.if_stmt(ctx, mat, scope)?;
                return Ok(());
            }
            if name == "echo" {
                self.echo_stmt(scope)?;
                return Ok(());
            }
            if name == "union" {
                self.take();
                self.expect(TokenKind::Lparen)?;
                self.expect(TokenKind::Rparen)?;
                self.body(ctx, mat, scope, t.line)?;
                return Ok(());
            }
            if name == "difference" || name == "intersection" {
                self.take();
                self.expect(TokenKind::Lparen)?;
                self.expect(TokenKind::Rparen)?;
                let op = if name == "difference" {
                    CsgOp::Difference
                } else {
                    CsgOp::Intersection
                };
                self.csg_block(op, ctx, mat, scope, t.line)?;
                return Ok(());
            }
            if self.peek1().kind == TokenKind::Assign {
                self.take();
                self.take();
                let v = self.expr(scope, 1)?;
                scope.insert(name.to_string(), v);
                self.expect(TokenKind::Semi)?;
                return Ok(());
            }
        }
        let t = self.take();
        if t.kind != TokenKind::Ident {
            return Err(format!(
                "CGS line {}: expected statement name, got {}",
                t.line, t.kind
            ));
        }
        let (pos, kw) = self.call_args(scope)?;
        let name = t.text.clone();
        if self.modules.contains_key(&name) {
            self.module_call(&name, pos, kw, ctx, mat, t.line)?;
            return Ok(());
        }
        let args = self.resolve(&name, pos, kw, t.line)?;
        if name == "translate" {
            let v = cgs_vec3(
                &args.get("t").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "translate.t",
            )?;
            let m2 = mat4_mul(ctx, translate4(v));
            self.body(m2, mat, scope, t.line)?;
            return Ok(());
        }
        if name == "rotate" {
            let ax = cgs_vec3(
                &args.get("axis").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "rotate.axis",
            )?;
            let ang = cgs_num(
                &args.get("angle").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "rotate.angle",
            )?;
            let m2 = mat4_mul(ctx, motor_rotor(ax, ang).to_matrix());
            self.body(m2, mat, scope, t.line)?;
            return Ok(());
        }
        if name == "scale" {
            let sv = args.get("s").cloned().unwrap_or(CgsValue::Num(1.0));
            let s4 = match sv {
                CgsValue::Vec3(v3) => scale4([v3.x, v3.y, v3.z]),
                CgsValue::List(_) => {
                    let v = cgs_vec3(&sv, t.line, "scale.s")?;
                    scale4(v)
                }
                _ => {
                    let x = cgs_num(&sv, t.line, "scale.s")?;
                    scale4([x, x, x])
                }
            };
            let m2 = mat4_mul(ctx, s4);
            self.body(m2, mat, scope, t.line)?;
            return Ok(());
        }
        if name == "mirror" {
            let ax = cgs_vec3(
                &args.get("axis").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "mirror.axis",
            )?;
            let m2 = mat4_mul(ctx, mirror4(ax)?);
            self.body(m2, mat, scope, t.line)?;
            return Ok(());
        }
        if name == "material" {
            let mut merged: HashMap<String, CgsValue> = HashMap::new();
            for (k, v) in mat {
                merged.insert(k.clone(), v.clone());
            }
            for (k, v) in &args {
                merged.insert(k.clone(), v.clone());
            }
            self.body(ctx, &merged, scope, t.line)?;
            return Ok(());
        }
        if name == "background" {
            self.expect(TokenKind::Semi)?;
            let c = cgs_num(
                &args.get("color").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "color",
            )?;
            self.scene.background = color_hex(c as i32);
            return Ok(());
        }
        if name == "camera" {
            self.expect(TokenKind::Semi)?;
            let fov = cgs_num(
                &args.get("fov").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "fov",
            )?;
            let aspect = cgs_num(
                &args.get("aspect").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "aspect",
            )?;
            let campos = cgs_vec3(
                &args.get("position").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "camera.position",
            )?;
            let tgt = cgs_vec3(
                &args.get("target").cloned().unwrap_or(CgsValue::Num(0.0)),
                t.line,
                "camera.target",
            )?;
            // perspective_camera panics on bad fov; the editor server has no panic
            // recovery, so validate here
            if fov <= 0.0 || fov >= 180.0 {
                return Err(format!(
                    "CGS line {}: camera.fov must be in (0, 180), got {}",
                    t.line, fov
                ));
            }
            if aspect <= 0.0 {
                return Err(format!(
                    "CGS line {}: camera.aspect must be > 0, got {}",
                    t.line, aspect
                ));
            }
            let mut cam = perspective_camera(fov, aspect, 0.1, 100.0, campos, tgt, [0.0, 1.0, 0.0]);
            cam.look_at(tgt, None);
            self.camera = Some(cam);
            return Ok(());
        }
        if name.ends_with("_light") {
            self.expect(TokenKind::Semi)?;
            self.add_light(&name, &args, t.line)?;
            return Ok(());
        }
        self.expect(TokenKind::Semi)?;
        let geo = self.build_geometry(&name, &args, t.line)?;
        self.add_geometry(geo, ctx, mat)
    }

    fn body(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        if self.peek().kind == TokenKind::Lbrace {
            self.expect(TokenKind::Lbrace)?;
            while self.peek().kind != TokenKind::Rbrace {
                self.statement(ctx, mat, scope)?;
            }
            self.expect(TokenKind::Rbrace)?;
        } else if self.peek().kind == TokenKind::Semi {
            return Err(format!(
                "CGS line {line}: modifier missing target statement"
            ));
        } else {
            self.statement(ctx, mat, scope)?;
        }
        Ok(())
    }

    fn for_loop(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        self.take(); // for
        self.expect(TokenKind::Lparen)?;
        let vt = self.take();
        if vt.kind != TokenKind::Ident || self.peek().kind != TokenKind::Assign {
            return Err(format!("CGS line {}: for needs (var = list)", vt.line));
        }
        self.take();
        let values = self.expr(scope, 1)?;
        self.expect(TokenKind::Rparen)?;
        match values {
            CgsValue::Vec3(v3) => {
                let body = self.capture_statement();
                for v in [v3.x, v3.y, v3.z] {
                    scope.insert(vt.text.clone(), CgsValue::Num(v));
                    self.run_tokens(body.clone(), ctx, mat, scope)?;
                }
            }
            CgsValue::List(list) => {
                let body = self.capture_statement();
                for v in list {
                    scope.insert(vt.text.clone(), v);
                    self.run_tokens(body.clone(), ctx, mat, scope)?;
                }
            }
            _ => return Err(format!("CGS line {}: for needs a list", vt.line)),
        }
        Ok(())
    }

    fn if_stmt(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        self.take(); // if
        self.expect(TokenKind::Lparen)?;
        let cond = cgs_truthy(&self.expr(scope, 1)?);
        self.expect(TokenKind::Rparen)?;
        if cond {
            self.statement(ctx, mat, scope)?;
            if self.peek().kind == TokenKind::Ident && self.peek().text == "else" {
                self.take();
                self.skip_statement();
            }
        } else {
            self.skip_statement();
            if self.peek().kind == TokenKind::Ident && self.peek().text == "else" {
                self.take();
                self.statement(ctx, mat, scope)?;
            }
        }
        Ok(())
    }

    fn echo_stmt(&mut self, scope: &HashMap<String, CgsValue>) -> Result<(), String> {
        self.take(); // echo
        self.expect(TokenKind::Lparen)?;
        let mut vals: Vec<CgsValue> = Vec::new();
        if self.peek().kind != TokenKind::Rparen {
            loop {
                vals.push(self.expr(scope, 1)?);
                if self.peek().kind == TokenKind::Comma {
                    self.take();
                } else {
                    break;
                }
            }
        }
        self.expect(TokenKind::Rparen)?;
        self.expect(TokenKind::Semi)?;
        print!("ECHO:");
        for v in &vals {
            print!(" {v}");
        }
        println!();
        Ok(())
    }

    fn module_def(&mut self) -> Result<(), String> {
        self.take(); // module
        let nt = self.take();
        if nt.kind != TokenKind::Ident {
            return Err(format!("CGS line {}: module missing name", nt.line));
        }
        self.expect(TokenKind::Lparen)?;
        if self.peek().kind != TokenKind::Rparen {
            loop {
                let pt = self.take();
                if pt.kind != TokenKind::Ident {
                    return Err(format!(
                        "CGS line {}: module parameter must be a name",
                        pt.line
                    ));
                }
                let key = format!("{}:{}", nt.text, pt.text);
                if self.peek().kind == TokenKind::Assign {
                    self.take();
                    let def = self.capture_expr()?;
                    self.set_param(key, def);
                } else {
                    self.set_param(key, Vec::new());
                }
                if self.peek().kind == TokenKind::Comma {
                    self.take();
                } else {
                    break;
                }
            }
        }
        self.expect(TokenKind::Rparen)?;
        self.expect(TokenKind::Lbrace)?;
        let start = self.pos;
        let mut depth = 1;
        while depth > 0 {
            let k = self.take().kind;
            if k == TokenKind::Lbrace {
                depth += 1;
            } else if k == TokenKind::Rbrace {
                depth -= 1;
            }
        }
        self.modules
            .insert(nt.text.clone(), self.toks[start..self.pos - 1].to_vec());
        Ok(())
    }

    fn set_param(&mut self, key: String, val: Vec<CgsToken>) {
        if !self.params.contains_key(&key) {
            self.param_order.push(key.clone());
        }
        self.params.insert(key, val);
    }

    fn module_call(
        &mut self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.expect(TokenKind::Semi)?;
        // formal parameter names are tracked separately; simplified: only positional + keyword by name
        let body = match self.modules.get(name) {
            Some(b) => b.clone(),
            None => Vec::new(),
        };
        let mut scope: HashMap<String, CgsValue> = HashMap::new();
        scope.insert("pi".to_string(), CgsValue::Num(std::f64::consts::PI));
        // find formal parameter names (stored as keys "name:param")
        let mut names: Vec<String> = Vec::new();
        let prefix = format!("{name}:");
        for k in &self.param_order {
            if k.starts_with(&prefix) {
                names.push(k[prefix.len()..].to_string());
            }
        }
        for (i, pname) in names.iter().enumerate() {
            if i < pos.len() {
                scope.insert(pname.clone(), pos[i].clone());
            }
        }
        for (k, v) in kw {
            scope.insert(k, v);
        }
        for pname in &names {
            if !scope.contains_key(pname) {
                let def = self
                    .params
                    .get(&format!("{name}:{pname}"))
                    .cloned()
                    .unwrap_or_default();
                if def.is_empty() {
                    return Err(format!(
                        "CGS line {line}: module {name} missing parameter {pname}"
                    ));
                }
                let v = self.eval_tokens(def, &scope)?;
                scope.insert(pname.clone(), v);
            }
        }
        self.run_tokens(body, ctx, mat, &mut scope)
    }

    fn capture_expr(&mut self) -> Result<Vec<CgsToken>, String> {
        let start = self.pos;
        let mut depth = 0;
        loop {
            let t = self.peek();
            if t.kind == TokenKind::Eof {
                return Err(format!("CGS line {}: unclosed expression", t.line));
            }
            if depth == 0
                && matches!(
                    t.kind,
                    TokenKind::Comma | TokenKind::Rparen | TokenKind::Semi
                )
            {
                break;
            }
            if matches!(t.kind, TokenKind::Lparen | TokenKind::Lbracket) {
                depth += 1;
            } else if matches!(t.kind, TokenKind::Rparen | TokenKind::Rbracket) {
                depth -= 1;
            }
            self.take();
        }
        Ok(self.toks[start..self.pos].to_vec())
    }

    fn eval_tokens(
        &mut self,
        toks: Vec<CgsToken>,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<CgsValue, String> {
        let saved = std::mem::replace(&mut self.toks, toks);
        let saved_pos = self.pos;
        self.pos = 0;
        let v = self.expr(scope, 1)?;
        self.toks = saved;
        self.pos = saved_pos;
        Ok(v)
    }

    fn capture_statement(&mut self) -> Vec<CgsToken> {
        let start = self.pos;
        self.skip_statement();
        self.toks[start..self.pos].to_vec()
    }

    fn skip_statement(&mut self) {
        let t = self.peek();
        if t.kind == TokenKind::Lbrace {
            self.take();
            let mut depth = 1;
            while depth > 0 {
                let k = self.take().kind;
                if k == TokenKind::Lbrace {
                    depth += 1;
                } else if k == TokenKind::Rbrace {
                    depth -= 1;
                }
            }
            return;
        }
        if t.kind == TokenKind::Ident && t.text == "if" {
            self.take();
            self.skip_parens();
            self.skip_statement();
            if self.peek().kind == TokenKind::Ident && self.peek().text == "else" {
                self.take();
                self.skip_statement();
            }
            return;
        }
        if t.kind == TokenKind::Ident && (t.text == "for" || t.text == "union" || t.text == "echo")
        {
            self.take();
            self.skip_parens();
            if t.text == "echo" {
                let _ = self.expect(TokenKind::Semi);
            } else {
                self.skip_statement();
            }
            return;
        }
        if t.kind == TokenKind::Ident && t.text == "module" {
            self.take();
            self.take();
            self.skip_parens();
            self.skip_statement();
            return;
        }
        self.take();
        if self.peek().kind == TokenKind::Assign {
            while self.take().kind != TokenKind::Semi {}
            return;
        }
        if self.peek().kind == TokenKind::Lparen {
            self.skip_parens();
            let nxt = self.peek().kind;
            if nxt == TokenKind::Lbrace {
                self.skip_statement();
            } else if nxt == TokenKind::Semi {
                self.take();
            } else {
                self.skip_statement();
            }
            return;
        }
        while self.take().kind != TokenKind::Semi {}
    }

    fn skip_parens(&mut self) {
        if self.expect(TokenKind::Lparen).is_err() {
            return;
        }
        let mut depth = 1;
        while depth > 0 {
            let k = self.take().kind;
            if k == TokenKind::Lparen {
                depth += 1;
            } else if k == TokenKind::Rparen {
                depth -= 1;
            }
        }
    }

    // --- construction -----------------------------------------------------------

    fn call_args(
        &mut self,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<(Vec<CgsValue>, HashMap<String, CgsValue>), String> {
        self.expect(TokenKind::Lparen)?;
        let mut pos: Vec<CgsValue> = Vec::new();
        let mut kw: HashMap<String, CgsValue> = HashMap::new();
        if self.peek().kind != TokenKind::Rparen {
            loop {
                let t = self.peek();
                if t.kind == TokenKind::Ident && self.peek1().kind == TokenKind::Assign {
                    self.take();
                    self.take();
                    let v = self.expr(scope, 1)?;
                    kw.insert(t.text.clone(), v);
                } else {
                    pos.push(self.expr(scope, 1)?);
                }
                if self.peek().kind == TokenKind::Comma {
                    self.take();
                } else {
                    break;
                }
            }
        }
        self.expect(TokenKind::Rparen)?;
        Ok((pos, kw))
    }

    fn resolve(
        &self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<HashMap<String, CgsValue>, String> {
        let names = cgs_sig_names(name);
        let defaults = cgs_sig_defaults(name);
        let mut merged: HashMap<String, CgsValue> = HashMap::new();
        for (k, v) in &defaults {
            merged.insert(k.clone(), v.clone());
        }
        if pos.len() > names.len() {
            return Err(format!("CGS line {line}: {name} too many positional args"));
        }
        for (i, pname) in names.iter().enumerate() {
            if i < pos.len() {
                merged.insert(pname.to_string(), pos[i].clone());
            }
        }
        for (k, v) in kw {
            if !names.contains(&k.as_str()) && !defaults.contains_key(&k) {
                return Err(format!("CGS line {line}: {name} has no parameter {k}"));
            }
            merged.insert(k, v);
        }
        for pname in &names {
            if let Some(CgsValue::Num(x)) = merged.get(*pname) {
                if x.is_nan() && *pname == "cylinder.h" {
                    // h may be None -> use -1 sentinel
                    merged.insert(pname.to_string(), CgsValue::Num(-1.0));
                }
            }
        }
        Ok(merged)
    }

    fn add_geometry(
        &mut self,
        geo: Geometry,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
    ) -> Result<(), String> {
        if self.collecting {
            self.collect.push(CollectedGeom { geo, m4: ctx });
            return Ok(());
        }
        let (motor, lin) = decompose_rigid(ctx);
        let g2 = if is_identity3(lin) {
            geo
        } else {
            Geometry::AffineGeometry(affine_geometry(geo, lin))
        };
        self.scene.add_mesh(mesh(MeshParams {
            geometry: g2,
            material: self.build_material(mat)?,
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: Some(motor),
        }));
        Ok(())
    }

    fn csg_block(
        &mut self,
        op: CsgOp,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        let saved = std::mem::take(&mut self.collect);
        let was_collecting = self.collecting;
        self.collecting = true;
        self.body(ctx, mat, scope, line)?;
        let children = std::mem::replace(&mut self.collect, saved);
        self.collecting = was_collecting;
        if children.len() < 2 {
            return Err(format!(
                "CGS line {line}: {} needs >= 2 geometry children",
                csg_op_name(op)
            ));
        }
        let mut kids: Vec<Geometry> = Vec::new();
        for c in children {
            if matches!(c.geo, Geometry::CircleGeometry(_)) {
                return Err(format!(
                    "CGS line {line}: {} children must be solids (circle is not)",
                    csg_op_name(op)
                ));
            }
            let (cm, cl) = decompose_rigid(c.m4);
            kids.push(Geometry::AffineGeometry(transformed_geometry(
                c.geo, cm, cl,
            )));
        }
        self.scene.add_mesh(mesh(MeshParams {
            geometry: Geometry::CsgGeometry(csg_geometry(op, kids)),
            material: self.build_material(mat)?,
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: Some(motor_identity()),
        }));
        Ok(())
    }

    fn build_geometry(
        &mut self,
        name: &str,
        args: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<Geometry, String> {
        validate_geometry_params(name, args, line)?;
        match name {
            "sphere" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "sphere.r",
                )?;
                Ok(Geometry::SphereGeometry(sphere_geometry(r)))
            }
            "plane" => {
                let n = cgs_vec3(
                    &args.get("n").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "plane.n",
                )?;
                let d = cgs_num(
                    &args.get("d").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "plane.d",
                )?;
                Ok(Geometry::PlaneGeometry(plane_geometry(n, d)))
            }
            "cylinder" => {
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cylinder.h",
                )?;
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cylinder.r",
                )?;
                Ok(Geometry::CylinderGeometry(cylinder_geometry(
                    r,
                    if h < 0.0 { -1.0 } else { h },
                )))
            }
            "box" => {
                let s = cgs_vec3(
                    &args.get("s").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "box.s",
                )?;
                Ok(Geometry::BoxGeometry(box_geometry(s[0], s[1], s[2])))
            }
            "circle" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "circle.r",
                )?;
                Ok(Geometry::CircleGeometry(circle_geometry(r)))
            }
            "cone" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cone.r",
                )?;
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cone.h",
                )?;
                Ok(Geometry::ConeGeometry(cone_geometry(r, h)))
            }
            "torus" => {
                let r1 = cgs_num(
                    &args.get("R").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "torus.R",
                )?;
                let r2 = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "torus.r",
                )?;
                Ok(Geometry::TorusGeometry(torus_geometry(r1, r2)))
            }
            "cyclide" => {
                let a = cgs_num(
                    &args.get("a").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cyclide.a",
                )?;
                let b = cgs_num(
                    &args.get("b").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cyclide.b",
                )?;
                let d = cgs_num(
                    &args.get("d").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cyclide.d",
                )?;
                Ok(Geometry::CyclideGeometry(cyclide_geometry(
                    a,
                    b,
                    d,
                    [0.0, 0.0, 0.0],
                )))
            }
            "ellipsoid" => {
                let r = cgs_vec3(
                    &args.get("radii").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "ellipsoid.radii",
                )?;
                Ok(Geometry::EllipsoidGeometry(ellipsoid_geometry(
                    r[0], r[1], r[2],
                )))
            }
            "extrude" => {
                let prof = profile2d(
                    &args.get("profile").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "extrude.profile",
                )?;
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "extrude.h",
                )?;
                let (v, f) = extrude(&prof, h);
                Ok(Geometry::TrimeshGeometry(trimesh_geometry(&v, &f)))
            }
            "loft" => {
                let raw = args.get("profiles").cloned().unwrap_or(CgsValue::Num(0.0));
                match raw {
                    CgsValue::List(list) => {
                        if list.len() < 2 {
                            return Err(format!(
                                "CGS line {line}: loft.profiles needs >= 2 sections"
                            ));
                        }
                        let mut profiles: Vec<Vec<[f64; 2]>> = Vec::new();
                        for p in &list {
                            profiles.push(profile2d(p, line, "loft.profiles[i]")?);
                        }
                        let zsraw = args.get("zs").cloned().unwrap_or(CgsValue::Num(0.0));
                        match zsraw {
                            CgsValue::Vec3(v3) => {
                                let (v, f) = loft(&profiles, &[v3.x, v3.y, v3.z]);
                                Ok(Geometry::TrimeshGeometry(trimesh_geometry(&v, &f)))
                            }
                            CgsValue::List(zlist) => {
                                let mut zs: Vec<f64> = Vec::new();
                                for z in &zlist {
                                    zs.push(cgs_num(z, line, "loft.zs[i]")?);
                                }
                                let (v, f) = loft(&profiles, &zs);
                                Ok(Geometry::TrimeshGeometry(trimesh_geometry(&v, &f)))
                            }
                            _ => Err(format!("CGS line {line}: loft.zs needs a list")),
                        }
                    }
                    _ => Err(format!("CGS line {line}: loft.profiles needs a list")),
                }
            }
            "mesh" => {
                let p = args.get("file").cloned().unwrap_or(CgsValue::Num(0.0));
                match p {
                    CgsValue::Str(path) => {
                        if self.asset_root.is_empty() {
                            return Err(format!(
                                "CGS line {line}: mesh needs an explicit asset_root"
                            ));
                        }
                        let full = format!("{}/{}", self.asset_root, path);
                        if full.ends_with(".obj") {
                            let (v, f) = load_obj(&full)?;
                            return Ok(Geometry::TrimeshGeometry(trimesh_geometry(&v, &f)));
                        }
                        if full.ends_with(".glb") || full.ends_with(".gltf") {
                            let loaded = load_gltf(&full)?;
                            return Ok(gltf_to_geometry(&loaded));
                        }
                        Err(format!(
                            "CGS line {line}: unsupported mesh file \"{path}\" (use .obj/.glb/.gltf)"
                        ))
                    }
                    _ => Err(format!("CGS line {line}: mesh.file needs a string path")),
                }
            }
            _ => Err(format!("CGS line {line}: unknown primitive {name}")),
        }
    }

    fn build_material(&self, mat: &HashMap<String, CgsValue>) -> Result<Material, String> {
        let color = match mat.get("color") {
            Some(v) => {
                let c = cgs_num(v, 0, "color")?;
                color_hex(c as i32)
            }
            None => color_hex(0xFFFFFF),
        };
        let mut tex = None;
        if let Some(v) = mat.get("map") {
            match v {
                CgsValue::Str(path) => {
                    if !path.is_empty() {
                        if self.asset_root.is_empty() {
                            return Err("CGS material.map needs an explicit asset_root".to_string());
                        }
                        tex = Some(texture_load(&format!("{}/{}", self.asset_root, path))?);
                    }
                }
                _ => return Err("CGS material.map needs a string path".to_string()),
            }
        }
        if let Some(u) = mat.get("unlit") {
            if cgs_truthy(u) {
                let op = match mat.get("opacity") {
                    Some(o) => cgs_num(o, 0, "opacity")?,
                    None => 1.0,
                };
                return Ok(basic_material(color, clamp01(op)));
            }
        }
        let roughness = cgs_opt_num(
            &mat.get("roughness").cloned().unwrap_or(CgsValue::Num(-1.0)),
            0.5,
        );
        let metalness = cgs_opt_num(
            &mat.get("metalness").cloned().unwrap_or(CgsValue::Num(-1.0)),
            0.0,
        );
        let emissive = match mat.get("emissive") {
            Some(v) => {
                let ev = cgs_num(v, 0, "emissive")?;
                if ev < 0.0 {
                    color_hex(0x000000)
                } else {
                    color_hex(ev as i32)
                }
            }
            None => color_hex(0x000000),
        };
        let opacity = cgs_opt_num(
            &mat.get("opacity").cloned().unwrap_or(CgsValue::Num(-1.0)),
            1.0,
        );
        let ior = cgs_opt_num(&mat.get("ior").cloned().unwrap_or(CgsValue::Num(-1.0)), 1.5);
        let absorption = cgs_opt_num(
            &mat.get("absorption")
                .cloned()
                .unwrap_or(CgsValue::Num(-1.0)),
            0.0,
        );
        let mut m = standard_material(MaterialParams {
            color,
            roughness,
            metalness,
            emissive,
            opacity,
            ior,
            absorption,
        });
        if let Some(t) = tex {
            m.map = Some(t);
        }
        Ok(m)
    }

    fn add_light(
        &mut self,
        name: &str,
        args: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        let c = cgs_num(
            &args.get("color").cloned().unwrap_or(CgsValue::Num(0.0)),
            line,
            "color",
        )?;
        let color = color_hex(c as i32);
        let intensity = cgs_num(
            &args.get("intensity").cloned().unwrap_or(CgsValue::Num(0.0)),
            line,
            "intensity",
        )?;
        if name == "directional_light" {
            let d = cgs_vec3(
                &args.get("direction").cloned().unwrap_or(CgsValue::Num(0.0)),
                line,
                "direction",
            )?;
            self.scene.add_light(directional_light(color, intensity, d));
        } else if name == "point_light" {
            let p = cgs_vec3(
                &args.get("position").cloned().unwrap_or(CgsValue::Num(0.0)),
                line,
                "position",
            )?;
            self.scene.add_light(point_light(color, intensity, p));
        } else {
            self.scene.add_light(ambient_light(color, intensity));
        }
        Ok(())
    }
}

// csg_op_name mirrors V's `${CsgOp}` interpolation (the enum variant name).
fn csg_op_name(op: CsgOp) -> &'static str {
    match op {
        CsgOp::Union => "union",
        CsgOp::Difference => "difference",
        CsgOp::Intersection => "intersection",
    }
}

// cgs_arg_num reads a numeric arg with a 0.0 fallback (validation only;
// build_geometry re-reads with proper errors).
fn cgs_arg_num(args: &HashMap<String, CgsValue>, key: &str) -> f64 {
    cgs_num(
        &args.get(key).cloned().unwrap_or(CgsValue::Num(0.0)),
        0,
        key,
    )
    .unwrap_or(0.0)
}

// validate_geometry_params turns constructor panics into clean CGS errors:
// the geometry constructors panic on bad parameters (sphere r <= 0, plane
// zero normal, loft unsorted zs, ...) and the editor server has no panic
// recovery, so common user mistakes must error out here.
fn validate_geometry_params(
    name: &str,
    args: &HashMap<String, CgsValue>,
    line: i32,
) -> Result<(), String> {
    let pos_keys: &[&str] = match name {
        "sphere" | "circle" | "cylinder" => &["r"],
        "cone" => &["r", "h"],
        "torus" => &["R", "r"],
        "extrude" => &["h"],
        _ => &[],
    };
    for k in pos_keys {
        if cgs_arg_num(args, k) <= 0.0 {
            return Err(format!("CGS line {line}: {name}.{k} must be > 0"));
        }
    }
    match name {
        "box" | "ellipsoid" => {
            let key = if name == "box" { "s" } else { "radii" };
            let v = cgs_vec3(
                &args.get(key).cloned().unwrap_or(CgsValue::Num(0.0)),
                line,
                &format!("{name}.{key}"),
            )?;
            if v[0].min(v[1].min(v[2])) <= 0.0 {
                return Err(format!(
                    "CGS line {line}: {name}.{key} components must be > 0"
                ));
            }
        }
        "plane" => {
            let n = cgs_vec3(
                &args.get("n").cloned().unwrap_or(CgsValue::Num(0.0)),
                line,
                "plane.n",
            )?;
            if vec3_dot(n, n) < 1e-24 {
                return Err(format!("CGS line {line}: plane.n must not be zero"));
            }
        }
        "cyclide" => {
            let a = cgs_arg_num(args, "a");
            let b = cgs_arg_num(args, "b");
            let d = cgs_arg_num(args, "d");
            if !(a > b && b > 0.0) {
                return Err(format!("CGS line {line}: cyclide needs a > b > 0"));
            }
            if d <= 0.0 {
                return Err(format!("CGS line {line}: cyclide.d must be > 0"));
            }
        }
        "loft" => {
            // loft() panics on mismatched profile sizes or unsorted zs
            if let Some(CgsValue::List(raw)) = args.get("profiles") {
                let mut m: i64 = -1;
                for p in raw {
                    if let CgsValue::List(pl) = p {
                        if m >= 0 && pl.len() as i64 != m {
                            return Err(format!(
                                "CGS line {line}: loft profiles must share the same vertex count"
                            ));
                        }
                        m = pl.len() as i64;
                    }
                }
            }
            let mut zs: Vec<f64> = Vec::new();
            let zsraw = args.get("zs").cloned().unwrap_or(CgsValue::Num(0.0));
            match zsraw {
                CgsValue::Vec3(v3) => {
                    zs = vec![v3.x, v3.y, v3.z];
                }
                CgsValue::List(zlist) => {
                    for z in &zlist {
                        zs.push(cgs_num(z, line, "loft.zs[i]")?);
                    }
                }
                _ => {}
            }
            for i in 0..zs.len().saturating_sub(1) {
                if zs[i] >= zs[i + 1] {
                    return Err(format!(
                        "CGS line {line}: loft.zs must be strictly increasing"
                    ));
                }
            }
        }
        _ => {}
    }
    Ok(())
}

fn profile2d(v: &CgsValue, line: i32, what: &str) -> Result<Vec<[f64; 2]>, String> {
    match v {
        CgsValue::List(items) => {
            if items.len() < 3 {
                return Err(format!("CGS line {line}: {what} needs >= 3 [x,y] points"));
            }
            let mut pts: Vec<[f64; 2]> = Vec::new();
            for p in items {
                match p {
                    CgsValue::List(pair) => {
                        if pair.len() != 2 {
                            return Err(format!("CGS line {line}: {what} items must be [x,y]"));
                        }
                        let x = cgs_num(&pair[0], line, what)?;
                        let y = cgs_num(&pair[1], line, what)?;
                        pts.push([x, y]);
                    }
                    _ => {
                        return Err(format!("CGS line {line}: {what} items must be [x,y]"));
                    }
                }
            }
            // triangulate/extrude panic on bad profiles; the server has no
            // panic recovery, so validate here
            validate_profile(&pts).map_err(|e| format!("CGS line {line}: {e}"))?;
            Ok(pts)
        }
        _ => Err(format!("CGS line {line}: {what} needs a list")),
    }
}

fn cgs_sig_names(name: &str) -> Vec<&'static str> {
    match name {
        "sphere" => vec!["r"],
        "plane" => vec!["n"],
        "cylinder" => vec!["r"],
        "box" => vec!["s"],
        "circle" => vec!["r"],
        "cone" => vec!["r", "h"],
        "torus" => vec!["R", "r"],
        "cyclide" => vec!["a", "b", "d"],
        "ellipsoid" => vec!["radii"],
        "extrude" => vec!["profile", "h"],
        "loft" => vec!["profiles", "zs"],
        "mesh" => vec!["file"],
        "translate" => vec!["t"],
        "rotate" => vec!["axis", "angle"],
        "scale" => vec!["s"],
        "mirror" => vec!["axis"],
        "difference" => vec![],
        "intersection" => vec![],
        "directional_light" => vec!["direction"],
        "point_light" => vec!["position"],
        "ambient_light" => vec![],
        "background" => vec!["color"],
        "camera" => vec![],
        "material" => vec![],
        _ => vec![],
    }
}

fn cgs_sig_defaults(name: &str) -> HashMap<String, CgsValue> {
    let mut m: HashMap<String, CgsValue> = HashMap::new();
    match name {
        "plane" => {
            m.insert("d".to_string(), CgsValue::Num(0.0));
        }
        "cylinder" => {
            m.insert("h".to_string(), CgsValue::Num(-1.0));
        }
        "directional_light" | "point_light" => {
            m.insert("intensity".to_string(), CgsValue::Num(1.0));
            m.insert("color".to_string(), CgsValue::Num(0xFFFFFF as f64));
        }
        "ambient_light" => {
            m.insert("intensity".to_string(), CgsValue::Num(0.3));
            m.insert("color".to_string(), CgsValue::Num(0xFFFFFF as f64));
        }
        "camera" => {
            m.insert("fov".to_string(), CgsValue::Num(50.0));
            m.insert("aspect".to_string(), CgsValue::Num(16.0 / 9.0));
            m.insert(
                "position".to_string(),
                CgsValue::Vec3(CgsVec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                }),
            );
            m.insert(
                "target".to_string(),
                CgsValue::Vec3(CgsVec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                }),
            );
        }
        "material" => {
            m.insert("color".to_string(), CgsValue::Num(0xFFFFFF as f64));
            m.insert("roughness".to_string(), CgsValue::Num(-1.0));
            m.insert("metalness".to_string(), CgsValue::Num(-1.0));
            m.insert("emissive".to_string(), CgsValue::Num(-1.0));
            m.insert("opacity".to_string(), CgsValue::Num(-1.0));
            m.insert("ior".to_string(), CgsValue::Num(-1.0));
            m.insert("absorption".to_string(), CgsValue::Num(-1.0));
            m.insert("unlit".to_string(), CgsValue::Bool(false));
            m.insert("map".to_string(), CgsValue::Str(String::new()));
        }
        _ => {}
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    // port of gpu/scene_lang_test.v ------------------------------------------------

    #[test]
    fn test_cgs_orbit() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/cgs/orbit.cgs"
        ))
        .expect("read orbit.cgs");
        let (sc, _) = cgs_load(
            &text,
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples"),
        );
        assert_eq!(sc.objects.len(), 7);
        assert_eq!(sc.lights.len(), 4);
    }

    #[test]
    fn test_cgs_grid_module() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/cgs/grid.cgs"
        ))
        .expect("read grid.cgs");
        let (sc, _) = cgs_load(
            &text,
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples"),
        );
        assert_eq!(sc.objects.len(), 10);
        assert_eq!(sc.lights.len(), 2);
    }

    #[test]
    fn test_cgs_csg_and_scale() {
        let text = "scale([2,1,1]) sphere(r=1);";
        let (sc, _) = cgs_load(text, "");
        assert_eq!(sc.objects.len(), 1);
        let text2 = "difference() { sphere(r=1); box(s=[1,1,1]); }";
        let (sc2, _) = cgs_load(text2, "");
        assert_eq!(sc2.objects.len(), 1);
    }

    #[test]
    fn test_cgs_render() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/cgs/orbit.cgs"
        ))
        .expect("read orbit.cgs");
        let (sc, cam) = cgs_load(
            &text,
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples"),
        );
        let mut r = crate::renderer(120, 90, 1, 3);
        let img = r.render(sc, cam);
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../artifacts/tests");
        std::fs::create_dir_all(dir).ok();
        crate::save_frame_png(&format!("{dir}/cgs_orbit.png"), &img);
        img.eval().unwrap();
        let data = img.as_slice::<f32>();
        let idx = 45 * 120 * 4 + 60 * 4;
        // centre is not the sky-blue background
        assert!(data[idx + 2] < 200.0);
    }

    // port of gpu/scene_lang_v2_test.v ---------------------------------------------
    // CGS v2: variables, expressions, math functions, for+range, module, if-else,
    // echo, union grouping (OpenSCAD-aligned semantics).

    fn sl_sphere_radius(g: &Geometry) -> f64 {
        match g {
            Geometry::SphereGeometry(g) => g.radius,
            _ => panic!("expected sphere geometry"),
        }
    }

    #[test]
    fn test_cgs_variables_and_expressions() {
        let (sc, _) = cgs_load(
            "r = 0.4 + 0.1;\n\
             x = 2 * (r + 0.5);\n\
             translate([x, 0, 0]) sphere(r=r);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!(sl_sphere_radius(&sc.objects[0].geometry) == 0.5);
        assert!((sc.objects[0].position[0] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_unary_minus_and_vector_arith() {
        let (sc, _) = cgs_load(
            "base = [1, 2, 3];\ntranslate([-1, 0, 0] + base * 2) sphere(r=0.5);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        let p = sc.objects[0].position;
        assert!((p[0] - 1.0).abs() < 1e-9);
        assert!((p[1] - 4.0).abs() < 1e-9);
        assert!((p[2] - 6.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_math_functions() {
        let (sc, _) = cgs_load(
            "translate([sin(pi / 2), sqrt(16), max(3, 7)]) sphere(r=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        let p = sc.objects[0].position;
        assert!((p[0] - 1.0).abs() < 1e-6);
        assert!((p[1] - 4.0).abs() < 1e-6);
        assert!((p[2] - 7.0).abs() < 1e-6);
    }

    #[test]
    fn test_cgs_for_range() {
        let (sc, _) = cgs_load(
            "for (i = [0:2]) translate([i, 0, 0]) sphere(r=0.1);\
             for (j = [0:0.5:1]) translate([0, j, 0]) sphere(r=0.1);",
            "",
        );
        assert_eq!(sc.objects.len(), 6); // [0:2] -> 3 + [0:0.5:1] -> 3
        assert!(sc.objects[0].position[0].abs() < 1e-9);
        assert!((sc.objects[1].position[0] - 1.0).abs() < 1e-9);
        assert!((sc.objects[2].position[0] - 2.0).abs() < 1e-9);
        assert!(sc.objects[3].position[1].abs() < 1e-9);
        assert!((sc.objects[4].position[1] - 0.5).abs() < 1e-9);
        assert!((sc.objects[5].position[1] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_module_with_defaults() {
        let (sc, _) = cgs_load(
            "module bead(r, gap=1.0) { translate([r * gap, 0, 0]) sphere(r=r); }\n\
             bead(0.5);\n\
             bead(0.5, gap=4.0);\n\
             bead(r=0.25);",
            "",
        );
        assert_eq!(sc.objects.len(), 3);
        assert!((sc.objects[0].position[0] - 0.5).abs() < 1e-9);
        assert!((sc.objects[1].position[0] - 2.0).abs() < 1e-9);
        assert!((sc.objects[2].position[0] - 0.25).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_module_inherits_transform() {
        let (sc, _) = cgs_load(
            "module ball() { sphere(r=1); }\ntranslate([3, 0, 0]) ball();",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!((sc.objects[0].position[0] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_if_else() {
        let (sc, _) = cgs_load(
            "which = 2;\n\
             if (which == 1) { sphere(r=1); } else { sphere(r=2); }\n\
             if (which > 1) sphere(r=3);",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
        assert!((sl_sphere_radius(&sc.objects[0].geometry) - 2.0).abs() < 1e-9);
        assert!((sl_sphere_radius(&sc.objects[1].geometry) - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_echo() {
        // echo runs without error and emits "ECHO: 3.0 [2.0, 4.0]"
        let (sc, _) = cgs_load("x = 1 + 2;\necho(x, [1, 2] * 2);", "");
        assert_eq!(sc.objects.len(), 0);
    }

    #[test]
    fn test_cgs_union_is_grouping() {
        let (sc, _) = cgs_load(
            "union() { sphere(r=1); translate([2, 0, 0]) sphere(r=1); }",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
    }

    #[test]
    fn test_cgs_load_result_errors() {
        // cgs_load_result returns an error instead of panicking.
        let mut saw_err = false;
        if let Err(e) = cgs_load_result("sphere(r=nope);", "") {
            assert!(e.contains("undefined variable"));
            saw_err = true;
        }
        assert!(saw_err);

        let mut saw_err2 = false;
        if let Err(e) = cgs_load_result("blah();", "") {
            assert!(e.contains("unknown primitive"));
            saw_err2 = true;
        }
        assert!(saw_err2);

        let mut saw_err3 = false;
        if let Err(e) = cgs_load_result("for (i = [0:0:1]) sphere(r=1);", "") {
            assert!(e.contains("range step must not be 0"));
            saw_err3 = true;
        }
        assert!(saw_err3);

        // constructor-panic inputs must come back as clean errors (no panic)
        let bad = [
            "sphere(r=-1);|sphere.r must be > 0",
            "sphere();|sphere.r must be > 0",
            "box(s=[1, 0, 1]);|box.s components must be > 0",
            "plane(n=[0, 0, 0]);|plane.n must not be zero",
            "cyclide(a=1, b=2, d=0.3);|cyclide needs a > b > 0",
            "camera(fov=0);|camera.fov must be in (0, 180)",
            "loft(profiles=[[[0, 0], [1, 0], [1, 1]], [[0, 0], [1, 0]]], zs=[0, 1]);|same vertex count",
            "loft(profiles=[[[0, 0], [1, 0], [1, 1]], [[0, 0], [1, 0], [1, 1]]], zs=[1, 0]);|strictly increasing",
            "difference() { circle(r=1); sphere(r=1); }|must be solids",
            "extrude(profile=[[0, 0], [0, 0], [0, 0]], h=1);|duplicate consecutive points",
            "extrude(profile=[[0, 0], [1, 0], [0.5, 0]], h=1);|degenerate",
            "extrude(profile=[[0, 0], [2, 0], [2, 2], [1, -1], [0, 2]], h=1);|self-intersects",
        ];
        for entry in bad {
            let parts: Vec<&str> = entry.split('|').collect();
            let mut saw = false;
            if let Err(e) = cgs_load_result(parts[0], "") {
                assert!(e.contains(parts[1]), "error {e:?} lacks {:?}", parts[1]);
                saw = true;
            }
            assert!(saw, "expected error for {:?}", parts[0]);
        }

        // valid input still parses
        let (sc, _) = cgs_load_result("sphere(r=1);", "").expect("unexpected error");
        assert_eq!(sc.objects.len(), 1);
    }
}
