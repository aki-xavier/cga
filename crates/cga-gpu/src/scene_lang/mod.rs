use std::collections::{BTreeMap, HashMap};
use std::fmt;

use cga_core::{
    clamp01, decompose_rigid, extrude, load_obj, loft, mat4_identity, mat4_mul, transform_point,
    validate_profile, AffineGeometry, BezierPatchGeometry, BoxGeometry, CircleGeometry,
    ConeGeometry, CsgGeometry, CsgOp, CyclideGeometry, CylinderGeometry, EllipsoidGeometry,
    Geometry, Multivector, PlaneGeometry, SphereGeometry, TorusGeometry, TrimeshGeometry,
};

use crate::mesh_io_gltf::{gltf_to_geometry, load_gltf};
use crate::scene::{Mesh, MeshParams, PerspectiveCamera, Scene};
use crate::scene_graph::{is_identity3, vec3_dot, Color};
use crate::shading::{Light, Material, MaterialParams};
use crate::texture::texture_load;

pub mod token_kind;
pub use self::token_kind::*;
pub mod cgs_token;
pub use self::cgs_token::*;
pub mod cgs_vec3;
pub use self::cgs_vec3::*;
pub mod geom_val;
pub use self::geom_val::*;
pub mod cgs_value;
pub use self::cgs_value::*;
pub mod drill_end;
pub(crate) use self::drill_end::*;
pub mod constrain_rel;
pub(crate) use self::constrain_rel::*;
pub mod bin_op;
pub(crate) use self::bin_op::*;
pub mod scene_loader;
pub use self::scene_loader::*;
pub mod inst;
pub(crate) use self::inst::*;
pub mod collected_geom;
pub(crate) use self::collected_geom::*;
pub mod tag_instance;
pub use self::tag_instance::*;
pub mod cgs_run;
pub use self::cgs_run::*;
pub mod kinematics;
pub use self::kinematics::*;

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
        b'.' => TokenKind::Dot,
        _ => TokenKind::Eof,
    }
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
            b'[' | b']' | b'{' | b'}' | b',' | b';' | b'=' | b'(' | b')' | b'.'
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

fn fmt_f64(x: f64) -> String {
    if x.is_finite() && x.fract() == 0.0 && x.abs() < 1e16 {
        format!("{x:.1}")
    } else {
        format!("{x}")
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

fn mat4_inv(m: [f64; 16]) -> [f64; 16] {
    let mut a = [[0.0f64; 8]; 4];
    for i in 0..4 {
        for j in 0..4 {
            a[i][j] = m[i * 4 + j];
            a[i][j + 4] = if i == j { 1.0 } else { 0.0 };
        }
    }
    for c in 0..4 {
        let mut p = c;
        for r in c + 1..4 {
            if a[r][c].abs() > a[p][c].abs() {
                p = r;
            }
        }
        if a[p][c].abs() < 1e-14 {
            return mat4_identity();
        }
        a.swap(c, p);
        let d = a[c][c];
        for j in 0..8 {
            a[c][j] /= d;
        }
        for r in 0..4 {
            if r == c {
                continue;
            }
            let f = a[r][c];
            if f != 0.0 {
                for j in 0..8 {
                    a[r][j] -= f * a[c][j];
                }
            }
        }
    }
    let mut out = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            out[i * 4 + j] = a[i][j + 4];
        }
    }
    out
}

pub(crate) fn transform_bbox(b: [[f64; 3]; 2], m: [f64; 16]) -> [[f64; 3]; 2] {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for i in 0..8 {
        let p = [
            if i & 1 == 0 { b[0][0] } else { b[1][0] },
            if i & 2 == 0 { b[0][1] } else { b[1][1] },
            if i & 4 == 0 { b[0][2] } else { b[1][2] },
        ];
        let q = transform_point(m, p);
        for k in 0..3 {
            lo[k] = lo[k].min(q[k]);
            hi[k] = hi[k].max(q[k]);
        }
    }
    [lo, hi]
}

pub(crate) fn parse_face_key(key: &str, line: i32, what: &str) -> Result<(usize, f64), String> {
    let ok =
        key.len() == 2 && matches!(&key[..1], "+" | "-") && matches!(&key[1..], "x" | "y" | "z");
    if !ok {
        return Err(format!(
            "CGS line {line}: {what}: unknown face key \"{key}\" (expected \"+x\"/\"-x\"/\"+y\"/\"-y\"/\"+z\"/\"-z\")"
        ));
    }
    let axis = match &key[1..] {
        "x" => 0,
        "y" => 1,
        _ => 2,
    };
    let sign = if &key[..1] == "+" { 1.0 } else { -1.0 };
    Ok((axis, sign))
}

fn face_key_str(axis: usize, sign: f64) -> String {
    format!(
        "{}{}",
        if sign > 0.0 { '+' } else { '-' },
        ["x", "y", "z"][axis]
    )
}

pub(crate) fn face_local(geo: &Geometry, axis: usize, sign: f64) -> Option<([f64; 3], [f64; 3])> {
    let mut k = [0.0, 0.0, 0.0];
    k[axis] = sign;
    match geo {
        Geometry::BoxGeometry(b) => {
            let mut p = [0.0; 3];
            p[axis] = sign * b.half[axis];
            Some((p, k))
        }
        Geometry::CylinderGeometry(c) => {
            if axis == 2 {
                let mut p = [0.0; 3];
                p[2] = sign * c.half;
                Some((p, k))
            } else {
                let mut p = [0.0; 3];
                p[axis] = sign * c.radius;
                Some((p, k))
            }
        }
        Geometry::ConeGeometry(c) => {
            if axis == 2 {
                let mut p = [0.0; 3];
                p[2] = if sign > 0.0 {
                    c.height / 2.0
                } else {
                    -c.height / 2.0
                };
                Some((p, k))
            } else {
                let mut p = [0.0; 3];
                p[axis] = sign * c.radius / 2.0;
                let mut n = [0.0; 3];
                n[axis] = sign * 2.0 * c.height;
                n[2] = c.radius;
                let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                Some((p, [n[0] / l, n[1] / l, n[2] / l]))
            }
        }
        Geometry::SphereGeometry(s) => {
            let mut p = [0.0; 3];
            p[axis] = sign * s.radius;
            Some((p, k))
        }
        Geometry::EllipsoidGeometry(e) => {
            let mut p = [0.0; 3];
            p[axis] = sign * e.radii[axis];
            Some((p, k))
        }
        _ => {
            let b = crate::geometry_ops::geom_bounds(&geo.identity_params())?;
            let mut p = [0.0; 3];
            for i in 0..3 {
                p[i] = (b[0][i] + b[1][i]) / 2.0;
            }
            p[axis] = if sign > 0.0 { b[1][axis] } else { b[0][axis] };
            Some((p, k))
        }
    }
}

pub(crate) fn transform_normal(m: [f64; 16], n: [f64; 3]) -> [f64; 3] {
    let inv = mat4_inv(m);
    let mut w = [0.0; 3];
    for r in 0..3 {
        w[r] = inv[r] * n[0] + inv[4 + r] * n[1] + inv[8 + r] * n[2];
    }
    let l = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
    if l < 1e-12 {
        return n;
    }
    [w[0] / l, w[1] / l, w[2] / l]
}

fn drill_end_ref(v: Option<&CgsValue>, line: i32, which: &str) -> Result<Option<DrillEnd>, String> {
    match v {
        None => Ok(None),
        Some(CgsValue::Num(n)) => Ok(Some(DrillEnd::Num(*n))),
        Some(CgsValue::Str(s)) => {
            let (name, key) = s.split_once(':').ok_or_else(|| {
                format!(
                    "CGS line {line}: drill.{which} face reference must be \"name:key\", got \"{s}\""
                )
            })?;
            if name.is_empty() {
                return Err(format!(
                    "CGS line {line}: drill.{which} face reference needs an instance name, got \"{s}\""
                ));
            }
            let (axis, sign) = parse_face_key(key, line, &format!("drill.{which}"))?;
            Ok(Some(DrillEnd::Face {
                name: name.to_string(),
                axis,
                sign,
            }))
        }
        Some(other) => Err(format!(
            "CGS line {line}: drill.{which} needs a number or \"name:key\" face reference, got {other}"
        )),
    }
}

fn flatten_val(v: &CgsValue, line: i32) -> Result<Vec<f64>, String> {
    match v {
        CgsValue::Num(n) => Ok(vec![*n]),
        CgsValue::Vec3(v3) => Ok(vec![v3.x, v3.y, v3.z]),
        CgsValue::List(items) => {
            let mut out = Vec::new();
            for i in items {
                out.extend(flatten_val(i, line)?);
            }
            Ok(out)
        }
        _ => Err(format!(
            "CGS line {line}: constraint values must be numeric or vectors, got {v}"
        )),
    }
}

fn maxabs(v: &[f64]) -> f64 {
    v.iter().fold(0.0f64, |m, x| m.max(x.abs()))
}

fn gauss_solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for c in 0..n {
        let mut p = c;
        for r in c + 1..n {
            if a[r][c].abs() > a[p][c].abs() {
                p = r;
            }
        }
        if a[p][c].abs() < 1e-14 {
            return None;
        }
        a.swap(c, p);
        b.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            if f != 0.0 {
                for k in c..n {
                    a[r][k] -= f * a[c][k];
                }
                b[r] -= f * b[c];
            }
        }
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for k in i + 1..n {
            s -= a[i][k] * x[k];
        }
        let d = a[i][i];
        if d.abs() < 1e-300 {
            return None;
        }
        x[i] = s / d;
    }
    Some(x)
}

fn geom_val(v: &CgsValue, line: i32, what: &str) -> Result<GeomVal, String> {
    match v {
        CgsValue::Geom(g) => Ok(g.clone()),
        _ => Err(format!(
            "CGS line {line}: {what} needs a geometry value, got {v}"
        )),
    }
}

const GEOM_EXPR_NAMES: &[&str] = &[
    "sphere",
    "plane",
    "cylinder",
    "box",
    "circle",
    "cone",
    "torus",
    "cyclide",
    "ellipsoid",
    "extrude",
    "loft",
    "mesh",
    "bezier",
];

const QUERY_FNS: &[&str] = &[
    "center",
    "lo",
    "hi",
    "size",
    "xdir",
    "ydir",
    "zdir",
    "dist",
    "face",
    "fnrm",
    "instances",
];

fn is_statement_only_fn(name: &str) -> bool {
    matches!(
        name,
        "show"
            | "tag"
            | "drill"
            | "var"
            | "constrain"
            | "joint"
            | "gear"
            | "cam"
            | "module"
            | "for"
            | "if"
            | "echo"
            | "translate"
            | "rotate"
            | "scale"
            | "mirror"
            | "material"
            | "background"
            | "camera"
    ) || name.ends_with("_light")
}

fn is_expr_only_fn(name: &str) -> bool {
    matches!(
        name,
        "polar"
            | "comp"
            | "len"
            | "norm"
            | "cross"
            | "abs"
            | "sign"
            | "sin"
            | "cos"
            | "tan"
            | "asin"
            | "acos"
            | "atan"
            | "sqrt"
            | "exp"
            | "ln"
            | "log"
            | "floor"
            | "ceil"
            | "round"
            | "atan2"
            | "pow"
            | "min"
            | "max"
    ) || QUERY_FNS.contains(&name)
        || matches!(name, "at" | "rot" | "scaled")
}

fn cgs_truthy(v: &CgsValue) -> bool {
    match v {
        CgsValue::List(items) => !items.is_empty(),
        CgsValue::Vec3(_) => true,
        CgsValue::Geom(_) => true,
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
    if op == BinOp::Eq || op == BinOp::Ne {
        if matches!(a, CgsValue::Geom(_)) || matches!(b, CgsValue::Geom(_)) {
            return Err("CGS: geometry values are not comparable".to_string());
        }
    }
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
    if name == "polar" {
        if args.len() != 2 {
            return Err(format!("CGS line {line}: polar needs (r, angle)"));
        }
        let r = cgs_num(&args[0], line, "polar.r")?;
        let a = cgs_num(&args[1], line, "polar.a")?;
        return Ok(CgsValue::Vec3(CgsVec3 {
            x: r * a.cos(),
            y: r * a.sin(),
            z: 0.0,
        }));
    }
    if name == "comp" {
        if args.len() != 2 {
            return Err(format!("CGS line {line}: comp needs (vector, index)"));
        }
        let v = cgs_vec3(&args[0], line, "comp.v")?;
        let i = cgs_num(&args[1], line, "comp.i")?;
        if i != 0.0 && i != 1.0 && i != 2.0 {
            return Err(format!("CGS line {line}: comp index must be 0, 1 or 2"));
        }
        return Ok(CgsValue::Num(v[i as usize]));
    }
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

pub type TagRegistry = BTreeMap<String, Vec<TagInstance>>;

pub fn cgs_load(text: &str, asset_root: &str) -> (Scene, PerspectiveCamera) {
    match cgs_load_result(text, asset_root) {
        Ok(r) => r,
        Err(e) => panic!("{e}"),
    }
}

pub fn cgs_load_result(text: &str, asset_root: &str) -> Result<(Scene, PerspectiveCamera), String> {
    cgs_run_result(text, asset_root).map(|r| (r.scene, r.camera))
}

pub fn cgs_run(text: &str, asset_root: &str) -> CgsRun {
    match cgs_run_result(text, asset_root) {
        Ok(r) => r,
        Err(e) => panic!("{e}"),
    }
}

/// Evaluate a CGS scene with pose overrides (docs/cgs-articulation.md P4).
/// An override takes effect at the assignment point for a scope variable, or
/// at the `joint` statement for a 1-DOF joint whose `q` is omitted.
pub fn cgs_pose(
    text: &str,
    asset_root: &str,
    overrides: &[(String, f64)],
) -> Result<CgsRun, String> {
    cgs_run_result_pose(text, asset_root, overrides)
}

pub(crate) fn csg_op_name(op: CsgOp) -> &'static str {
    match op {
        CsgOp::Union => "union",
        CsgOp::Difference => "difference",
        CsgOp::Intersection => "intersection",
    }
}

fn cgs_arg_num(args: &HashMap<String, CgsValue>, key: &str) -> f64 {
    cgs_num(
        &args.get(key).cloned().unwrap_or(CgsValue::Num(0.0)),
        0,
        key,
    )
    .unwrap_or(0.0)
}

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
        "torus" => {
            let arc = cgs_arg_num(args, "arc");
            if !(arc > 0.0) || arc > std::f64::consts::TAU {
                return Err(format!("CGS line {line}: torus.arc must be in (0, 2*pi]"));
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
        "bezier" => {
            if cgs_arg_num(args, "thickness") < 0.0 {
                return Err(format!("CGS line {line}: bezier.thickness must be >= 0"));
            }
            let div = cgs_arg_num(args, "div");
            if div.fract() != 0.0 || div < 1.0 || div > 32.0 {
                return Err(format!(
                    "CGS line {line}: bezier.div must be an integer in 1..=32"
                ));
            }
            match args.get("points") {
                Some(CgsValue::List(items)) if items.len() == 16 => {}
                Some(CgsValue::List(items)) => {
                    return Err(format!(
                        "CGS line {line}: bezier.points needs 16 [x,y,z] control points, got {}",
                        items.len()
                    ));
                }
                _ => {
                    return Err(format!(
                        "CGS line {line}: bezier.points needs 16 [x,y,z] control points, got none"
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

            validate_profile(&pts).map_err(|e| format!("CGS line {line}: {e}"))?;
            Ok(pts)
        }
        _ => Err(format!("CGS line {line}: {what} needs a list")),
    }
}

fn points16(v: &CgsValue, line: i32, what: &str) -> Result<Vec<[f64; 3]>, String> {
    match v {
        CgsValue::List(items) => {
            if items.len() != 16 {
                return Err(format!(
                    "CGS line {line}: {what} needs 16 [x,y,z] control points, got {}",
                    items.len()
                ));
            }
            let mut pts: Vec<[f64; 3]> = Vec::with_capacity(16);
            for p in items {
                pts.push(cgs_vec3(p, line, what)?);
            }
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
        "bezier" => vec!["points"],
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
        "torus" => {
            m.insert("arc".to_string(), CgsValue::Num(std::f64::consts::TAU));
        }
        "bezier" => {
            m.insert("thickness".to_string(), CgsValue::Num(0.0));
            m.insert("div".to_string(), CgsValue::Num(8.0));
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
        let mut r = crate::Renderer::new(120, 90, 1, 3);
        let img = r.render(sc, cam);
        let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../artifacts/tests");
        std::fs::create_dir_all(dir).ok();
        crate::save_frame_png(&format!("{dir}/cgs_orbit.png"), &img);
        img.eval().unwrap();
        let data = img.as_slice::<f32>();
        let idx = 45 * 120 * 4 + 60 * 4;

        assert!(data[idx + 2] < 200.0);
    }

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
        assert_eq!(sc.objects.len(), 6);
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

        let (sc, _) = cgs_load_result("sphere(r=1);", "").expect("unexpected error");
        assert_eq!(sc.objects.len(), 1);
    }

    fn v2_err(src: &str) -> String {
        match cgs_load_result(src, "") {
            Err(e) => e,
            Ok(_) => panic!("expected error for {src:?}"),
        }
    }

    #[test]
    fn test_cgs_p4_chain_equivalence() {
        let (sa, ca) = cgs_load("box(s=[2,2,2]).at([1,0,0]).rot([0,1,0], 45);", "");
        let (sb, cb) = cgs_load("show(rot(at(box(s=[2,2,2]), [1,0,0]), [0,1,0], 45));", "");
        assert_eq!(sa.objects.len(), 1);
        assert_eq!(sb.objects.len(), 1);
        assert_eq!(
            sa.report(&ca, &TagRegistry::new(), &Kinematics::default()),
            sb.report(&cb, &TagRegistry::new(), &Kinematics::default())
        );
    }

    #[test]
    fn test_cgs_p4_chain_errors_inherited() {
        let e = v2_err("g = box(s=[1,1,1]);\ng.show();");
        assert!(
            e.contains("show is a statement and cannot be used in an expression"),
            "{e}"
        );

        let e_chain = v2_err("g = box(s=[1,1,1]);\ng.frobnicate();");
        let e_plain = v2_err("g = box(s=[1,1,1]);\nx = frobnicate(g);");
        assert_eq!(e_chain, e_plain);
        assert!(
            e_chain.contains("frobnicate needs a number, got <geom>"),
            "{e_chain}"
        );

        let e = v2_err("x = (1).frobnicate();");
        assert!(e.contains("unknown function frobnicate"), "{e}");
        let e = v2_err("g = box(s=[1,1,1]);\ng.at();");
        assert!(e.contains("at needs (geometry, offset)"), "{e}");
        let e = v2_err("g = box(s=[1,1,1]);\ng.5;");
        assert!(e.contains("expected ident, got number"), "{e}");
        let e = v2_err("g = box(s=[1,1,1]);\ng.center;");
        assert!(e.contains("expected lparen, got semi"), "{e}");
    }

    #[test]
    fn test_cgs_p4_chain_statement_render() {
        let (sc, _) = cgs_load("box(s=[1,1,1]).at([1,0,0]);", "");
        assert_eq!(sc.objects.len(), 1);
        let e = v2_err("s = sphere(r=1);\ns.size();");
        assert!(
            e.contains("expression statement needs a geometry value, got"),
            "{e}"
        );
    }

    #[test]
    fn test_cgs_p4_chain_var_head() {
        let (sa, ca) = cgs_load(
            "g = box(s=[1,1,1]);\ng.at([1,0,0]);\ng.rot([0,1,0], 45);",
            "",
        );
        let (sb, cb) = cgs_load(
            "g = box(s=[1,1,1]);\nshow(at(g, [1,0,0]));\nshow(rot(g, [0,1,0], 45));",
            "",
        );
        assert_eq!(sa.objects.len(), 2);
        assert_eq!(sb.objects.len(), 2);
        assert_eq!(
            sa.report(&ca, &TagRegistry::new(), &Kinematics::default()),
            sb.report(&cb, &TagRegistry::new(), &Kinematics::default())
        );
    }

    #[test]
    fn test_cgs_p4_chain_str_head() {
        let (sa, ca) = cgs_load(
            "tag(\"plate\") box(s=[1,1,2]);\ntranslate(\"plate\".face(\"+z\")) sphere(r=0.1);",
            "",
        );
        let (sb, cb) = cgs_load(
            "tag(\"plate\") box(s=[1,1,2]);\ntranslate(face(\"plate\", \"+z\")) sphere(r=0.1);",
            "",
        );
        assert_eq!(sa.objects.len(), 2);
        assert_eq!(
            sa.report(&ca, &TagRegistry::new(), &Kinematics::default()),
            sb.report(&cb, &TagRegistry::new(), &Kinematics::default())
        );
    }

    #[test]
    fn test_cgs_p4_lex_dot() {
        let (sc, _) = cgs_load("x = 1.5.abs();", "");
        assert_eq!(sc.objects.len(), 0);
        let e = v2_err("x = 1.abs();");
        assert!(e.contains("expected semi, got ident"), "{e}");
        let e = v2_err("x = .5;");
        assert!(e.contains("bad expression start dot"), "{e}");
    }

    #[test]
    fn test_cgs_p5_instances_order() {
        let reg = "for (i = [0:2]) tag(\"hole\") translate([i, 0, 0]) cylinder(r=0.3, h=5);\n";
        let (sa, ca) = cgs_load(&format!("{reg}for (h = instances(\"hole\")) show(h);"), "");
        let (sb, cb) = cgs_load(
            &format!(
                "{reg}show(at(cylinder(r=0.3, h=5), [0, 0, 0]));\n\
                 show(at(cylinder(r=0.3, h=5), [1, 0, 0]));\n\
                 show(at(cylinder(r=0.3, h=5), [2, 0, 0]));"
            ),
            "",
        );
        assert_eq!(sa.objects.len(), 6);
        assert_eq!(sb.objects.len(), 6);
        assert_eq!(
            sa.report(&ca, &TagRegistry::new(), &Kinematics::default()),
            sb.report(&cb, &TagRegistry::new(), &Kinematics::default())
        );
    }

    #[test]
    fn test_cgs_p5_instances_count_if() {
        let reg = "for (i = [0:2]) tag(\"hole\") translate([i, 0, 0]) cylinder(r=0.3, h=5);\n";
        let (sc, _) = cgs_load(
            &format!(
                "{reg}n = len(instances(\"hole\"));\nif (n > 2) translate([0, 0, 5]) sphere(r=0.5);"
            ),
            "",
        );
        assert_eq!(sc.objects.len(), 4);
        let (sc, _) = cgs_load(
            &format!(
                "{reg}n = len(instances(\"hole\"));\nif (n > 5) translate([0, 0, 5]) sphere(r=0.5);"
            ),
            "",
        );
        assert_eq!(sc.objects.len(), 3);
    }

    #[test]
    fn test_cgs_p5_instances_errors() {
        let e = v2_err("v = instances(\"nope\");");
        assert!(e.contains("unknown reference \"nope\""), "{e}");
        let e = v2_err("v = instances(5);");
        assert!(e.contains("instances needs a reference name, got 5"), "{e}");
        let e = v2_err("instances(\"x\");");
        assert!(
            e.contains("instances is an expression function and cannot be used as a statement"),
            "{e}"
        );
        let e = v2_err("v = instances();");
        assert!(e.contains("instances needs 1 argument(s)"), "{e}");
    }

    #[test]
    fn test_cgs_p5_instances_aggregate_element() {
        let reg = "for (i = [0:2]) tag(\"hole\") translate([i, 0, 0]) cylinder(r=0.3, h=5);\n";
        let (sa, ca) = cgs_load(
            &format!("{reg}translate(center(\"hole\")) sphere(r=0.05);"),
            "",
        );
        let (sb, cb) = cgs_load(
            &format!(
                "{reg}k = 0;\nm = [0, 0, 0];\n\
                 for (h = instances(\"hole\")) {{ if (k == 1) {{ m = center(h); }} k = k + 1; }}\n\
                 translate(m) sphere(r=0.05);"
            ),
            "",
        );
        assert_eq!(sa.objects.len(), 4);
        assert_eq!(sb.objects.len(), 4);
        assert_eq!(
            sa.report(&ca, &TagRegistry::new(), &Kinematics::default()),
            sb.report(&cb, &TagRegistry::new(), &Kinematics::default())
        );
        let (sc, _) = cgs_load(
            &format!(
                "{reg}hs = instances(\"hole\");\n\
                 difference() {{ box(s=[6, 6, 1]); for (h = hs) drill(r=0.2, through=h, axis=2); }}"
            ),
            "",
        );
        assert_eq!(sc.objects.len(), 4);
    }

    #[test]
    fn test_cgs_p5_face_single_string() {
        let reg = "tag(\"plate\") box(s=[1, 1, 2]);\n";
        let (sa, ca) = cgs_load(
            &format!("{reg}translate(face(\"plate:+z\")) sphere(r=0.1);"),
            "",
        );
        let (sb, cb) = cgs_load(
            &format!("{reg}translate(face(\"plate\", \"+z\")) sphere(r=0.1);"),
            "",
        );
        assert_eq!(sa.objects.len(), 2);
        assert_eq!(
            sa.report(&ca, &TagRegistry::new(), &Kinematics::default()),
            sb.report(&cb, &TagRegistry::new(), &Kinematics::default())
        );
        let e = v2_err("tag(\"plate\") box(s=[1,1,2]);\nv = face(\"plate\");");
        assert!(e.contains("face needs 2 argument(s)"), "{e}");
    }

    #[test]
    fn test_cgs_geom_value_bind_show() {
        let (sc, _) = cgs_load("g = cylinder(r=1, h=2);", "");
        assert_eq!(sc.objects.len(), 0);
        let (sc, _) = cgs_load("g = cylinder(r=1, h=2);\nshow(g);", "");
        assert_eq!(sc.objects.len(), 1);
        match &sc.objects[0].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.radius - 1.0).abs() < 1e-12);
                assert!((c.half - 1.0).abs() < 1e-9);
            }
            _ => panic!("expected cylinder geometry"),
        }
    }

    #[test]
    fn test_cgs_frameless_binding() {
        let (sc, _) = cgs_load(
            "translate([3, 0, 0]) { g = box(s=[1, 1, 1]); show(g); }",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!((sc.objects[0].position[0] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_frame_helpers() {
        let (sc, _) = cgs_load("g = sphere(r=1);\nshow(at(g, [1, 0, 0]));", "");
        assert_eq!(sc.objects.len(), 1);
        assert!((sc.objects[0].position[0] - 1.0).abs() < 1e-9);

        let (sc, _) = cgs_load(
            "g = sphere(r=1);\ntranslate([10, 0, 0]) show(at(g, [1, 0, 0]));",
            "",
        );
        assert!((sc.objects[0].position[0] - 11.0).abs() < 1e-9);

        let (sc, _) = cgs_load("g = sphere(r=1);\nshow(scaled(g, 2));", "");
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(
            sc.objects[0].geometry,
            Geometry::AffineGeometry(_)
        ));

        let (sc, _) = cgs_load(
            "g = box(s=[1, 1, 1]);\nshow(rot(g, [0, 0, 1], pi / 2));",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
    }

    #[test]
    fn test_cgs_expr_csg_forms() {
        let (sc, _) = cgs_load("difference(sphere(r=1), box(s=[1, 1, 1]));", "");
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(sc.objects[0].geometry, Geometry::CsgGeometry(_)));

        let (sc, _) = cgs_load(
            "x = difference(sphere(r=1), box(s=[1, 1, 1]));\nshow(x);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(sc.objects[0].geometry, Geometry::CsgGeometry(_)));

        let (sc, _) = cgs_load(
            "difference() { box(s=[2, 2, 2]); difference() { sphere(r=3); box(s=[4, 4, 4]); } }",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(sc.objects[0].geometry, Geometry::CsgGeometry(_)));

        let (sc, _) = cgs_load(
            "intersection() { difference() { sphere(r=2); box(s=[1, 1, 1]); } difference() { box(s=[3, 3, 3]); sphere(r=0.5); } }",
            "",
        );
        assert_eq!(sc.objects.len(), 1);

        let (sc, _) = cgs_load(
            "difference() { box(s=[6, 6, 6]); difference(sphere(r=4), box(s=[8, 8, 8])); }",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
    }

    #[test]
    fn test_cgs_tag_reference_queries() {
        let (sc, _) = cgs_load(
            "tag(\"p\") translate([5, 0, 0]) sphere(r=1);\ntranslate(center(\"p\")) sphere(r=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
        assert!((sc.objects[1].position[0] - 5.0).abs() < 1e-9);

        let (sc, _) = cgs_load(
            "tag(\"a\") box(s=[2, 4, 6]);\ntranslate(size(\"a\")) sphere(r=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
        let p = sc.objects[1].position;
        assert!(
            (p[0] - 2.0).abs() < 1e-9 && (p[1] - 4.0).abs() < 1e-9 && (p[2] - 6.0).abs() < 1e-9
        );

        let (sc, _) = cgs_load(
            "tag(\"a\") box(s=[2, 4, 6]);\ntranslate(hi(\"a\")) sphere(r=1);",
            "",
        );
        let p = sc.objects[1].position;
        assert!(
            (p[0] - 1.0).abs() < 1e-9 && (p[1] - 2.0).abs() < 1e-9 && (p[2] - 3.0).abs() < 1e-9
        );

        let (sc, _) = cgs_load(
            "tag(\"pair\") { translate([2, 0, 0]) sphere(r=1); translate([-2, 0, 0]) sphere(r=1); }\ntranslate(center(\"pair\")) sphere(r=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 3);
        assert!(sc.objects[2].position[0].abs() < 1e-9);
    }

    #[test]
    fn test_cgs_axis_and_dist_queries() {
        let (sc, _) = cgs_load(
            "tag(\"r\") rotate(axis=[0, 0, 1], angle=pi / 2) box(s=[2, 2, 2]);\ntranslate(xdir(\"r\")) sphere(r=1);",
            "",
        );
        let p = sc.objects[1].position;
        assert!(p[1].abs() > 0.99, "xdir must map onto ±y: {p:?}");
        assert!(p[0].abs() < 1e-9 && p[2].abs() < 1e-9);

        let (sc, _) = cgs_load(
            "tag(\"a\") translate([3, 0, 0]) sphere(r=1);\nd = dist(\"a\", [0, 0, 0]);\ntranslate([d, 0, 0]) sphere(r=1);",
            "",
        );
        assert!((sc.objects[1].position[0] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_show_in_csg_and_tag_group() {
        let (sc, _) = cgs_load(
            "g = sphere(r=1);\ndifference() { show(g); box(s=[1, 1, 1]); }",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(sc.objects[0].geometry, Geometry::CsgGeometry(_)));
    }

    #[test]
    fn test_cgs_v2_query_errors() {
        let e = v2_err("translate(center(\"missing\")) sphere(r=1);");
        assert!(e.contains("unknown reference"), "{e}");
        let e = v2_err("translate(center(plane(n=[0, 1, 0]))) sphere(r=1);");
        assert!(e.contains("no finite bounds"), "{e}");
        let e = v2_err("show(5);");
        assert!(e.contains("geometry value"), "{e}");
        let e = v2_err("x = sphere(r=1);\ny = (x == x);");
        assert!(e.contains("not comparable"), "{e}");
        let e = v2_err("y = sqrt(x=4);");
        assert!(e.contains("no named arguments"), "{e}");
        let e = v2_err("g = sphere(r=1);\nshow(at(g));");
        assert!(e.contains("at needs (geometry, offset)"), "{e}");
    }

    #[test]
    fn test_mat4_inv_roundtrip() {
        let m = mat4_mul(
            translate4([1.0, 2.0, 3.0]),
            Multivector::rotor([0.0, 0.0, 1.0], 0.7).to_matrix(),
        );
        let p = mat4_mul(m, mat4_inv(m));
        let eye = mat4_identity();
        for i in 0..16 {
            assert!(
                (p[i] - eye[i]).abs() < 1e-9,
                "idx {i}: {} vs {}",
                p[i],
                eye[i]
            );
        }
    }

    #[test]
    fn test_cgs_drill_tracks_target() {
        let (sc, _) = cgs_load(
            "p = box(s=[4, 0.3, 4]);\ntag(\"plate\") show(p);\ndrill(r=0.2, through=\"plate\", axis=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
        match &sc.objects[1].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.radius - 0.2).abs() < 1e-12);
                assert!((c.half - 0.15).abs() < 1e-9, "half = {}", c.half);
            }
            _ => panic!("expected cylinder cutter"),
        }

        let (sc, _) = cgs_load(
            "p = box(s=[4, 0.5, 4]);\ntag(\"plate\") show(p);\ndrill(r=0.2, through=\"plate\", axis=1);",
            "",
        );
        match &sc.objects[1].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.half - 0.25).abs() < 1e-9, "half = {}", c.half);
            }
            _ => panic!("expected cylinder cutter"),
        }

        let (sc, _) = cgs_load(
            "p = box(s=[4, 0.3, 4]);\ntag(\"plate\") translate([0, 2, 0]) show(p);\ndrill(r=0.2, through=\"plate\", axis=1);",
            "",
        );
        assert!(
            (sc.objects[1].position[1] - 2.0).abs() < 1e-9,
            "pos = {:?}",
            sc.objects[1].position
        );

        let (sc, _) = cgs_load(
            "p = box(s=[4, 0.5, 4]);\ndrill(r=0.2, through=p, axis=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        match &sc.objects[0].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.half - 0.25).abs() < 1e-9, "half = {}", c.half);
            }
            _ => panic!("expected cylinder cutter"),
        }
    }

    #[test]
    fn test_cgs_drill_from_to_and_csg() {
        let (sc, _) = cgs_load(
            "p = box(s=[4, 0.3, 4]);\ntag(\"plate\") show(p);\ndrill(r=0.2, through=\"plate\", axis=1, from=-1, to=1);",
            "",
        );
        match &sc.objects[1].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.half - 1.0).abs() < 1e-9, "half = {}", c.half);
            }
            _ => panic!("expected cylinder cutter"),
        }

        let (sc, _) = cgs_load(
            "p = box(s=[4, 0.3, 4]);\ntag(\"plate\") show(p);\ndifference() { show(p); drill(r=0.2, through=\"plate\", axis=1); }",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
        assert!(matches!(sc.objects[1].geometry, Geometry::CsgGeometry(_)));
    }

    #[test]
    fn test_cgs_polar_and_comp() {
        let (sc, _) = cgs_load(
            "tag(\"c\") translate([1, 0, 0]) box(s=[2, 2, 2]);\n\
             for (i = [0:1]) {\n  a = i * pi;\n  translate(center(\"c\") + polar(1.0, a)) sphere(r=0.1);\n}",
            "",
        );
        assert_eq!(sc.objects.len(), 3);
        assert!(
            (sc.objects[1].position[0] - 2.0).abs() < 1e-9,
            "{:?}",
            sc.objects[1].position
        );
        assert!(
            sc.objects[2].position[0].abs() < 1e-9,
            "{:?}",
            sc.objects[2].position
        );

        let (sc, _) = cgs_load(
            "tag(\"c\") box(s=[2, 4, 6]);\ntranslate([comp(size(\"c\"), 0), 0, 0]) sphere(r=1);",
            "",
        );
        assert!((sc.objects[1].position[0] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_drill_errors() {
        let e = v2_err("drill(r=0.2, axis=1);");
        assert!(e.contains("through"), "{e}");
        let e = v2_err("drill(r=0.2, through=\"a\", axis=1);");
        assert!(e.contains("unknown reference"), "{e}");
        let e = v2_err("p = box(s=[1, 1, 1]);\ndrill(r=0.2, through=p, axis=5);");
        assert!(e.contains("axis must be 0, 1 or 2"), "{e}");
        let e = v2_err("p = box(s=[1, 1, 1]);\ndrill(through=p, axis=1);");
        assert!(e.contains("needs r="), "{e}");
        let e = v2_err(
            "p = box(s=[1, 1, 1]);\ntag(\"t\") show(p);\ndrill(r=0.2, through=\"t\", axis=1, from=1, to=0);",
        );
        assert!(e.contains("non-empty"), "{e}");
        let e = v2_err("p = plane(n=[0, 1, 0]);\ndrill(r=0.2, through=p, axis=1);");
        assert!(e.contains("no finite bounds"), "{e}");
        let e = v2_err("v = comp([1, 2, 3], 5);");
        assert!(e.contains("index must be 0, 1 or 2"), "{e}");
    }

    #[test]
    fn test_cgs_constrain_single_equation() {
        let (sc, _) = cgs_load(
            "var d = 1.0;\n\
             tag(\"a\") translate([-1, 0, 0]) sphere(r=1);\n\
             b = at(sphere(r=1), [d + 3, 0, 0]);\n\
             show(b);\n\
             constrain(d) { dist(center(\"a\"), [d + 3, 0, 0]) == 4; } solve;\n\
             translate([d + 3, 0, 0]) sphere(r=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 3);

        assert!(
            (sc.objects[1].position[0] - 4.0).abs() < 1e-6,
            "{:?}",
            sc.objects[1].position
        );

        assert!(
            (sc.objects[2].position[0] - 3.0).abs() < 1e-6,
            "{:?}",
            sc.objects[2].position
        );
    }

    #[test]
    fn test_cgs_constrain_vector_equation() {
        let (sc, _) = cgs_load(
            "var t = 0.0;\n\
             constrain(t) { [t, t + 1, 0] == [4, 5, 0]; } solve;\n\
             translate([t, 0, 0]) sphere(r=1);",
            "",
        );
        assert!(
            (sc.objects[0].position[0] - 4.0).abs() < 1e-6,
            "{:?}",
            sc.objects[0].position
        );
    }

    #[test]
    fn test_cgs_constrain_init_independent() {
        for init in ["0.0", "3.5"] {
            let text = format!(
                "var t = {init};\nconstrain(t) {{ t == 4; }} solve;\ntranslate([t, 0, 0]) sphere(r=1);"
            );
            let (sc, _) = cgs_load(&text, "");
            assert!(
                (sc.objects[0].position[0] - 4.0).abs() < 1e-9,
                "init {init} → {:?}",
                sc.objects[0].position
            );
        }

        let (sc, _) = cgs_load(
            "var t = 3.5;\nconstrain(t) { t * t == 9; } solve;\ntranslate([t, 0, 0]) sphere(r=1);",
            "",
        );
        assert!(
            (sc.objects[0].position[0] - 3.0).abs() < 1e-6,
            "{:?}",
            sc.objects[0].position
        );
    }

    #[test]
    fn test_cgs_constrain_contradiction() {
        let e = v2_err("var x = 0.0;\nconstrain(x) { x == 3; x == 5; } solve;");
        assert!(e.contains("did not converge"), "{e}");
        assert!(e.contains("var=x"), "{e}");
    }

    #[test]
    fn test_cgs_constrain_underdetermined() {
        let (sc, _) = cgs_load(
            "var p = 1.0;\nvar q = 2.0;\n\
             constrain(p, q) { p + q == 5; } solve;\n\
             translate([p, q, 0]) sphere(r=1);",
            "",
        );
        let pos = sc.objects[0].position;
        assert!((pos[0] + pos[1] - 5.0).abs() < 1e-6, "{pos:?}");
    }

    #[test]
    fn test_cgs_constrain_in_for_loop() {
        let (sc, _) = cgs_load(
            "for (i = [0:2]) {\n  var x = 0.0;\n  constrain(x) { x == i; } solve;\n  translate([x, i, 0]) sphere(r=0.1);\n}",
            "",
        );
        assert_eq!(sc.objects.len(), 3);
        for (k, i) in [0.0f64, 1.0, 2.0].iter().enumerate() {
            assert!((sc.objects[k].position[0] - i).abs() < 1e-9, "k={k}");
            assert!((sc.objects[k].position[1] - i).abs() < 1e-9, "k={k}");
        }
    }

    #[test]
    fn test_cgs_constrain_errors() {
        let e = v2_err("constrain(nope) { nope == 1; } solve;");
        assert!(e.contains("not defined"), "{e}");

        let (sc, _) = cgs_load(
            "var x = 0.0;\nconstrain(x) { x < 1; } solve;\ntranslate([x, 0, 0]) sphere(r=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!(sc.objects[0].position[0].abs() < 1e-9);
        let e = v2_err("var x = 0.0;\nconstrain(x) { x; } solve;");
        assert!(e.contains("must be `lhs == rhs;`"), "{e}");
        let e = v2_err("var x = 0.0;\nconstrain(x) { x == 1 }");
        assert!(e.contains("`solve;`"), "{e}");
        let e = v2_err("var x = 0.0;\nconstrain(x) { } solve;");
        assert!(e.contains("block is empty"), "{e}");
        let e = v2_err("s = \"a\";\nconstrain(s) { s == \"a\"; } solve;");
        assert!(e.contains("must be a number"), "{e}");
        let e = v2_err("var x = \"a\";");
        assert!(e.contains("var x must be a number"), "{e}");
    }

    #[test]
    fn test_cgs_p6_inequality_satisfied_no_pull() {
        let (sc, _) = cgs_load(
            "var x = 0.0;\nconstrain(x) { x <= 5; } solve;\ntranslate([x, 0, 0]) sphere(r=1);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!(
            sc.objects[0].position[0].abs() < 1e-9,
            "{:?}",
            sc.objects[0].position
        );
    }

    #[test]
    fn test_cgs_p6_inequality_violated_to_boundary() {
        let (sc, _) = cgs_load(
            "var x = 5.0;\nconstrain(x) { x <= 1; } solve;\ntranslate([x, 0, 0]) sphere(r=1);",
            "",
        );
        assert!(
            (sc.objects[0].position[0] - 1.0).abs() < 1e-6,
            "{:?}",
            sc.objects[0].position
        );
    }

    #[test]
    fn test_cgs_p6_mixed_eq_inequality() {
        let (sc, _) = cgs_load(
            "var x = 0.0;\nvar y = 0.0;\nconstrain(x, y) { x + y == 10; x >= 0; y >= 0; } solve;\ntranslate([x, y, 0]) sphere(r=1);",
            "",
        );
        let p = sc.objects[0].position;
        assert!(
            (p[0] - 5.0).abs() < 1e-6 && (p[1] - 5.0).abs() < 1e-6,
            "{p:?}"
        );
    }

    #[test]
    fn test_cgs_p6_infeasible_explicit() {
        let e = v2_err("var x = 0.0;\nconstrain(x) { x == 6; x <= 5; } solve;");
        assert!(e.contains("did not converge"), "{e}");
    }

    #[test]
    fn test_cgs_p6_relation_errors() {
        let e = v2_err("var x = 0.0;\nconstrain(x) { x != 1; } solve;");
        assert!(
            e.contains("constrain does not support != — use ==, <= or >="),
            "{e}"
        );
        let e = v2_err("var x = 0.0;\nvar y = 0.0;\nconstrain(x, y) { x <= y == 0; } solve;");
        assert!(e.contains("one relation per constrain equation"), "{e}");
        let e = v2_err("var x = 0.0;\nvar y = 0.0;\nconstrain(x, y) { x <= y <= 0; } solve;");
        assert!(e.contains("one relation per constrain equation"), "{e}");
    }

    #[test]
    fn test_cgs_p6_lt_closed_vector_components() {
        let (sc, _) = cgs_load(
            "var x = 5.0;\nconstrain(x) { [x, 0, 0] < [1, 1, 1]; } solve;\ntranslate([x, 0, 0]) sphere(r=1);",
            "",
        );
        assert!(
            (sc.objects[0].position[0] - 1.0).abs() < 1e-6,
            "{:?}",
            sc.objects[0].position
        );
        let (sc, _) = cgs_load(
            "var x = 0.0;\nconstrain(x) { [x, 0, 0] < [1, 1, 1]; } solve;\ntranslate([x, 0, 0]) sphere(r=1);",
            "",
        );
        assert!(
            sc.objects[0].position[0].abs() < 1e-9,
            "{:?}",
            sc.objects[0].position
        );
    }

    #[test]
    fn test_cgs_statement_expression_boundary() {
        let (sc, _) = cgs_load("background = 0x2B3138;", "");
        assert!((sc.background.r - 0x2Bu32 as f64 / 255.0).abs() < 1e-12);
        assert!((sc.background.g - 0x31u32 as f64 / 255.0).abs() < 1e-12);
        assert!((sc.background.b - 0x38u32 as f64 / 255.0).abs() < 1e-12);
        let (sc, _) = cgs_load("background(color=0x87CEEB);", "");
        assert!((sc.background.r - 0x87u32 as f64 / 255.0).abs() < 1e-12);

        let (sc, _) = cgs_load("echo = 5;\nfor = 6;\necho(echo + for);", "");
        assert_eq!(sc.objects.len(), 0);

        let e = v2_err("x = 1; y = x = 2;");
        assert!(
            e.contains("assignment is a statement and cannot be used in an expression"),
            "{e}"
        );

        for src in [
            "g = show(box(s=[1, 1, 1]));",
            "g = tag(\"t\");",
            "g = drill(r=1);",
            "g = var(x);",
            "g = constrain(x);",
            "g = for (i = [0:1]) sphere(r=1);",
            "g = if (true) sphere(r=1);",
            "g = echo(\"x\");",
            "g = translate([1, 0, 0]);",
            "g = rotate(axis=[0, 0, 1], angle=1);",
            "g = scale(s=2);",
            "g = mirror(axis=[1, 0, 0]);",
            "g = material(color=1);",
            "g = camera(fov=60, aspect=1.5);",
            "g = background(color=1);",
            "g = point_light(position=[0, 0, 0], intensity=1);",
        ] {
            let e = v2_err(src);
            assert!(
                e.contains("is a statement and cannot be used in an expression"),
                "[{src}] -> {e}"
            );
        }

        for src in [
            "len([1, 2]);",
            "dist([1, 0, 0], [0, 0, 0]);",
            "center(\"x\");",
            "at(sphere(r=1), [1, 0, 0]);",
            "polar(1, 2);",
        ] {
            let e = v2_err(src);
            assert!(
                e.contains("is an expression function and cannot be used as a statement"),
                "[{src}] -> {e}"
            );
        }

        for src in ["a = [1, 2, 3]; b = a[0];", "a = [1, 2, 3]; a[0];"] {
            let e = v2_err(src);
            assert!(e.contains("indexing is not supported"), "[{src}] -> {e}");
        }

        let e = v2_err("blah();");
        assert!(e.contains("unknown primitive"), "{e}");
        let (sc, _) = cgs_load("x = 1; x = x + 1; echo(x);", "");
        assert_eq!(sc.objects.len(), 0);
        let (sc, _) = cgs_load(
            "g = difference(sphere(r=1), box(s=[1, 1, 1])); show(g);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        let (sc, _) = cgs_load("size = 4; box(s=[size, size, size]);", "");
        assert_eq!(sc.objects.len(), 1);
    }

    #[test]
    fn test_cgs_face_fnrm_queries() {
        let (sc, _) = cgs_load(
            "tag(\"p\") rotate(axis=[0, 0, 1], angle=pi / 2) translate([5, 0, 0]) box(s=[2, 4, 6]);\n\
             translate(face(\"p\", \"+x\")) sphere(r=0.1);\n\
             translate(fnrm(\"p\", \"+x\")) sphere(r=0.1);",
            "",
        );
        assert_eq!(sc.objects.len(), 3);
        let fp = sc.objects[1].position;
        assert!(
            (fp[0] - 0.0).abs() < 1e-9 && (fp[1] - 6.0).abs() < 1e-9 && fp[2].abs() < 1e-9,
            "face +x = {fp:?}"
        );
        let np = sc.objects[2].position;
        assert!(
            np[0].abs() < 1e-9 && (np[1] - 1.0).abs() < 1e-9 && np[2].abs() < 1e-9,
            "fnrm +x = {np:?}"
        );

        let (sc, _) = cgs_load(
            "c = cylinder(r=2, h=4);\n\
             translate(face(c, \"+z\")) sphere(r=0.1);\n\
             translate(face(c, \"-z\")) sphere(r=0.1);\n\
             translate(face(c, \"+x\")) sphere(r=0.1);",
            "",
        );
        assert_eq!(sc.objects.len(), 3);
        assert!((sc.objects[0].position[2] - 2.0).abs() < 1e-9);
        assert!((sc.objects[1].position[2] + 2.0).abs() < 1e-9);
        assert!((sc.objects[2].position[0] - 2.0).abs() < 1e-9);

        let (sc, _) = cgs_load(
            "s = sphere(r=3);\n\
             e = ellipsoid(radii=[4, 5, 6]);\n\
             translate(face(s, \"-y\")) sphere(r=0.1);\n\
             translate(face(e, \"+x\")) sphere(r=0.1);",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
        assert!((sc.objects[0].position[1] + 3.0).abs() < 1e-9);
        assert!((sc.objects[1].position[0] - 4.0).abs() < 1e-9);

        let (sc, _) = cgs_load(
            "k = cone(r=3, h=6);\n\
             translate(face(k, \"+z\")) sphere(r=0.1);\n\
             translate(face(k, \"-z\")) sphere(r=0.1);\n\
             translate(face(k, \"+x\")) sphere(r=0.1);\n\
             translate(fnrm(k, \"+x\")) sphere(r=0.1);",
            "",
        );
        assert_eq!(sc.objects.len(), 4);
        assert!((sc.objects[0].position[2] - 3.0).abs() < 1e-9);
        assert!((sc.objects[1].position[2] + 3.0).abs() < 1e-9);
        assert!((sc.objects[2].position[0] - 1.5).abs() < 1e-9);
        let n = (153.0f64).sqrt();
        let np = sc.objects[3].position;
        assert!((np[0] - 12.0 / n).abs() < 1e-9, "cone side normal = {np:?}");
        assert!((np[2] - 3.0 / n).abs() < 1e-9, "cone side normal = {np:?}");

        let (sc, _) = cgs_load(
            "tm = torus(R=3, r=1);\ntranslate(face(tm, \"+x\")) sphere(r=0.1);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!((sc.objects[0].position[0] - 4.0).abs() < 1e-9);
    }

    #[test]
    fn test_cgs_face_query_errors() {
        let e = v2_err("p = box(s=[1, 1, 1]);\nx = face(\"p\", \"q\");");
        assert!(e.contains("unknown face key"), "{e}");
        let e = v2_err("p = box(s=[1, 1, 1]);\nx = face(\"p\", 1);");
        assert!(e.contains("key must be a string"), "{e}");
        let e = v2_err("x = face(\"missing\", \"+x\");");
        assert!(e.contains("unknown reference"), "{e}");
        let e = v2_err("pl = plane(n=[0, 1, 0]);\nx = face(pl, \"+y\");");
        assert!(e.contains("no finite bounds"), "{e}");
        let e = v2_err("p = box(s=[1, 1, 1]);\nx = fnrm(\"p\");");
        assert!(e.contains("needs 2 argument"), "{e}");
    }

    #[test]
    fn test_cgs_drill_face_refs() {
        let (sc, _) = cgs_load(
            "tag(\"plate\") box(s=[4, 4, 0.3]);\n\
             drill(r=0.2, through=\"plate\", from=\"plate:-z\", to=\"plate:+z\");",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
        match &sc.objects[1].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.radius - 0.2).abs() < 1e-12);
                assert!((c.half - 0.15).abs() < 1e-9, "half = {}", c.half);
            }
            _ => panic!("expected cylinder cutter"),
        }

        let (sc, _) = cgs_load(
            "tag(\"plate\") box(s=[4, 4, 0.3]);\n\
             drill(r=0.2, from=\"plate:-z\", to=\"plate:+z\");",
            "",
        );
        assert_eq!(sc.objects.len(), 2);
        match &sc.objects[1].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.half - 0.15).abs() < 1e-9, "half = {}", c.half);
            }
            _ => panic!("expected cylinder cutter"),
        }

        let (sc, _) = cgs_load(
            "tag(\"base\") box(s=[4, 4, 1]);\n\
             tag(\"top\") translate([0, 0, 2.5]) box(s=[4, 4, 1]);\n\
             drill(r=0.2, from=\"base:+z\", to=\"top:-z\");",
            "",
        );
        assert_eq!(sc.objects.len(), 3);
        match &sc.objects[2].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.half - 0.75).abs() < 1e-9, "half = {}", c.half);
            }
            _ => panic!("expected cylinder cutter"),
        }
        assert!(
            (sc.objects[2].position[2] - 1.25).abs() < 1e-9,
            "pos = {:?}",
            sc.objects[2].position
        );

        let (sc, _) = cgs_load(
            "tag(\"plate\") box(s=[4, 4, 0.3]);\n\
             drill(r=0.2, through=\"plate\", axis=2, from=\"plate:-z\", to=\"plate:+z\");",
            "",
        );
        match &sc.objects[1].geometry {
            Geometry::CylinderGeometry(c) => {
                assert!((c.half - 0.15).abs() < 1e-9, "half = {}", c.half);
            }
            _ => panic!("expected cylinder cutter"),
        }
    }

    #[test]
    fn test_cgs_drill_face_ref_errors() {
        let e = v2_err(
            "tag(\"p\") box(s=[1, 1, 1]);\ndrill(r=0.2, axis=1, from=\"p:-z\", to=\"p:+z\");",
        );
        assert!(e.contains("does not match axis"), "{e}");
        let e = v2_err("tag(\"p\") box(s=[1, 1, 1]);\ndrill(r=0.2, from=\"p:-z\", to=\"p:+x\");");
        assert!(e.contains("different axes"), "{e}");
        let e = v2_err("tag(\"p\") box(s=[1, 1, 1]);\ndrill(r=0.2, from=\"p\");");
        assert!(e.contains("name:key"), "{e}");
        let e = v2_err("tag(\"p\") box(s=[1, 1, 1]);\ndrill(r=0.2, from=\"p:q\", to=\"p:+z\");");
        assert!(e.contains("unknown face key"), "{e}");
        let e = v2_err("drill(r=0.2, from=\"missing:+z\", to=\"other:-z\");");
        assert!(e.contains("unknown reference"), "{e}");
        let e = v2_err("tag(\"p\") box(s=[1, 1, 1]);\ndrill(r=0.2, axis=2, from=\"p:-z\");");
        assert!(e.contains("needs through"), "{e}");
        let e = v2_err(
            "pl = plane(n=[0, 1, 0]);\ntag(\"pl\") show(pl);\ndrill(r=0.2, from=\"pl:+y\", to=\"pl:-y\");",
        );
        assert!(e.contains("no finite bounds"), "{e}");
    }

    fn bez_pts_flat() -> String {
        let mut s = String::from("[");
        for i in 0..4 {
            for j in 0..4 {
                if i + j > 0 {
                    s.push_str(", ");
                }
                s.push_str(&format!(
                    "[{}, {}, 0]",
                    -1.0 + i as f64 * (2.0 / 3.0),
                    -1.0 + j as f64 * (2.0 / 3.0)
                ));
            }
        }
        s.push(']');
        s
    }

    #[test]
    fn test_cgs_bezier_build_and_query() {
        let src = format!(
            "b = bezier(points={}, thickness=0.2, div=4);\nshow(b);",
            bez_pts_flat()
        );
        let (sc, _) = cgs_load(&src, "");
        assert_eq!(sc.objects.len(), 1);
        let src2 = format!(
            "b = bezier(points={}, thickness=0.2, div=4);\ntag(\"bb\") show(b);\ntranslate(center(\"bb\")) sphere(r=0.1);",
            bez_pts_flat()
        );
        let (sc2, _) = cgs_load(&src2, "");
        assert_eq!(sc2.objects.len(), 2);
        let p = sc2.objects[1].position;
        assert!(
            p[0].abs() < 1e-9 && p[1].abs() < 1e-9 && p[2].abs() < 1e-9,
            "{p:?}"
        );
    }

    #[test]
    fn test_cgs_bezier_errors() {
        let e = v2_err("bezier(points=[[0,0,0],[1,0,0],[0,1,0]], thickness=0.2);");
        assert!(e.contains("needs 16 [x,y,z] control points"), "{e}");
        let e = v2_err(&format!(
            "bezier(points={}, thickness=-1.0);",
            bez_pts_flat()
        ));
        assert!(e.contains("bezier.thickness must be >= 0"), "{e}");
        let e = v2_err(&format!("bezier(points={}, div=0);", bez_pts_flat()));
        assert!(e.contains("bezier.div must be an integer in 1..=32"), "{e}");
        let e = v2_err(&format!(
            "difference() {{ bezier(points={}); box(s=[2,2,2]); }}",
            bez_pts_flat()
        ));
        assert!(
            e.contains("must be solids (bezier surface with thickness=0 is not)"),
            "{e}"
        );
        let e = v2_err(&format!(
            "s = bezier(points={});\ndifference(s, box(s=[2,2,2]));",
            bez_pts_flat()
        ));
        assert!(
            e.contains("must be solids (bezier surface with thickness=0 is not)"),
            "{e}"
        );
    }

    #[test]
    fn test_cgs_freeform_example() {
        let text = std::fs::read_to_string(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../examples/cgs/freeform.cgs"
        ))
        .expect("read freeform.cgs");
        let (sc, _) = cgs_load(
            &text,
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples"),
        );
        assert_eq!(sc.objects.len(), 3);
    }

    #[test]
    fn test_cgs_torus_arc() {
        // 省略 arc（或恰为 2π）→ 整环，行为与既有 torus 逐位一致
        for src in [
            "torus(R=1, r=0.3);",
            "torus(R=1, r=0.3, arc=6.283185307179586);",
        ] {
            let (sc, _) = cgs_load(src, "");
            let Geometry::TorusGeometry(t) = &sc.objects[0].geometry else {
                panic!("expected torus")
            };
            assert_eq!(t.arc, std::f64::consts::TAU, "{src}");
        }
        // arc < 2π → 部分弧环管（tube）
        for src in [
            "torus(R=1, r=0.3, arc=4.2);",
            "t = torus(R=1, r=0.3, arc=4.2); show(t);",
            "torus(R=1, r=0.3, arc=4.2).at([0, 1, 0]);",
        ] {
            let (sc, _) = cgs_load(src, "");
            let Geometry::TorusGeometry(t) = &sc.objects[0].geometry else {
                panic!("expected torus")
            };
            assert_eq!(t.arc, 4.2, "{src}");
        }
        // 错误组
        for src in [
            "torus(R=1, r=0.3, arc=0);",
            "torus(R=1, r=0.3, arc=-1);",
            "torus(R=1, r=0.3, arc=7);",
        ] {
            let e = v2_err(src);
            assert_eq!(e, "CGS line 1: torus.arc must be in (0, 2*pi]", "{src}");
        }
        let e = v2_err("torus(1, 0.3, 4.2);");
        assert_eq!(e, "CGS line 1: torus too many positional args");
    }

    // ---- joint / gear / cam (docs/cgs-joint.md P1–P3) ----

    fn kin_of(src: &str) -> Kinematics {
        cgs_run_result(src, "")
            .expect("unexpected error")
            .kinematics
    }

    #[test]
    fn test_p1_joint_six_kinds_pose() {
        use cga_core::transform_point;
        // revolute: q=pi/2 about z at [1,0,0]; child frame origin maps to at.
        let k = kin_of(
            "joint(\"j\", type=\"revolute\", axis=[0,0,1], at=[1,0,0], q=pi/2) sphere(r=0.1);",
        );
        let w = k.joints[0].world;
        assert!(
            (w[3] - 1.0).abs() < 1e-9 && w[7].abs() < 1e-9,
            "锚点: {w:?}"
        );
        let p = transform_point(w, [1.0, 0.0, 0.0]);
        assert!(
            (p[0] - 1.0).abs() < 1e-9 && (p[1] - 1.0).abs() < 1e-9,
            "旋转: {p:?}"
        );

        // prismatic: axis z, q=2.
        let k = kin_of("joint(\"j\", type=\"prismatic\", axis=[0,0,1], q=2) sphere(r=0.1);");
        assert!(
            (kin_of("joint(\"j\", type=\"prismatic\", axis=[0,0,1], q=2) sphere(r=0.1);").joints
                [0]
            .world[11]
                - 2.0)
                .abs()
                < 1e-9
        );
        let _ = k;

        // helical: pitch=0.1, q=pi → 转 pi + 移 0.1pi。
        let k =
            kin_of("joint(\"j\", type=\"helical\", axis=[0,0,1], pitch=0.1, q=pi) sphere(r=0.1);");
        let p = transform_point(k.joints[0].world, [1.0, 0.0, 0.0]);
        assert!((p[0] + 1.0).abs() < 1e-9, "螺旋转: {p:?}");
        assert!(
            (p[2] - 0.1 * std::f64::consts::PI).abs() < 1e-9,
            "螺旋转移: {p:?}"
        );

        // cylindrical: q=[qr, qp] = [0, 0.2] → 纯移 0.2。
        let k =
            kin_of("joint(\"j\", type=\"cylindrical\", axis=[0,0,1], q=[0, 0.2]) sphere(r=0.1);");
        assert!((k.joints[0].world[11] - 0.2).abs() < 1e-9);

        // spherical: rx=pi/2 → [0,1,0] 映到 [0,0,1]。
        let k = kin_of("joint(\"j\", type=\"spherical\", q=[pi/2, 0, 0]) sphere(r=0.1);");
        let p = transform_point(k.joints[0].world, [0.0, 1.0, 0.0]);
        assert!((p[2] - 1.0).abs() < 1e-9, "球面副: {p:?}");

        // planar: axis z, q=[1,2,pi/2]。基约定：e1 = axis × x̂（axis≈x̂ 时取 ŷ），e2 = axis × e1。
        // e1=[0,1,0]，e2=[-1,0,0] → 平移 1·e1+2·e2 = [-2,1,0]；[1,0,0] 转 pi/2 得 [0,1,0]。
        let k = kin_of("joint(\"j\", type=\"planar\", axis=[0,0,1], q=[1,2,pi/2]) sphere(r=0.1);");
        let p = transform_point(k.joints[0].world, [1.0, 0.0, 0.0]);
        assert!(
            (p[0] + 2.0).abs() < 1e-9 && (p[1] - 2.0).abs() < 1e-9,
            "平面副: {p:?}"
        );

        // fixed: 恒等。
        let k = kin_of("joint(\"j\", type=\"fixed\", at=[1,0,0]) sphere(r=0.1);");
        let p = transform_point(k.joints[0].world, [1.0, 0.0, 0.0]);
        assert!((p[0] - 2.0).abs() < 1e-9, "固定副: {p:?}");
    }

    #[test]
    fn test_p1_joint_nesting_and_report() {
        let run = cgs_run_result(
            "joint(\"root\", type=\"revolute\", axis=[0,0,1], q=0.5, limit=[-1.57, 1.57]) \
             joint(\"elbow\", type=\"prismatic\", axis=[1,0,0], at=[0.3,0,0], q=0.1) \
             sphere(r=0.05);",
            "",
        )
        .expect("unexpected error");
        let k = &run.kinematics;
        assert_eq!(k.joints.len(), 2);
        assert_eq!(k.joints[1].parent.as_deref(), Some("root"));
        // 肘的子系 = 根旋转 0.5 ∘ (锚 [0.3,0,0] + 移动 0.1·x)：平移 = R(0.5)·[0.4,0,0]。
        let w = k.joints[1].world;
        assert!((w[3] - 0.4 * 0.5f64.cos()).abs() < 1e-9, "肘 x: {}", w[3]);
        assert!((w[7] - 0.4 * 0.5f64.sin()).abs() < 1e-9, "肘 y: {}", w[7]);

        let rep = crate::cgs_report(
            "joint(\"root\", type=\"revolute\", axis=[0,0,1], q=0.5, limit=[-1.57, 1.57]) \
             joint(\"elbow\", type=\"prismatic\", axis=[1,0,0], at=[0.3,0,0], q=0.1) \
             sphere(r=0.05);",
            "",
        )
        .expect("unexpected error");
        assert!(
            rep.contains(
                "joint 0 \"root\" type=revolute parent=none axis=[0,0,1] at=[0,0,0] q=0.5 limit=[-1.57,1.57]"
            ),
            "{rep}"
        );
        assert!(
            rep.contains(
                "joint 1 \"elbow\" type=prismatic parent=\"root\" axis=[1,0,0] at=[0.3,0,0] q=0.1"
            ),
            "{rep}"
        );
        assert!(rep.contains("joints=2 gears=0 cams=0"), "{rep}");
    }

    #[test]
    fn test_p1_joint_errors() {
        assert_eq!(
            v2_err("joint(\"j\", type=\"blah\") sphere(r=1);"),
            "CGS line 1: joint.type must be \"revolute\", \"continuous\", \"prismatic\", \"helical\", \"cylindrical\", \"spherical\", \"planar\" or \"fixed\""
        );
        assert_eq!(
            v2_err("joint(\"j\", type=\"fixed\") sphere(r=1);\njoint(\"j\", type=\"fixed\") box(s=[1,1,1]);"),
            "CGS line 2: duplicate joint name j"
        );
        assert_eq!(
            v2_err("joint(\"j\", type=\"revolute\", axis=[0,0,0]) sphere(r=1);"),
            "CGS line 1: joint.axis must be nonzero"
        );
        assert_eq!(
            v2_err("joint(\"j\", type=\"revolute\", q=2, limit=[-1, 1]) sphere(r=1);"),
            "CGS line 1: joint j q=2.0 outside limit [-1.0, 1.0]"
        );
        assert_eq!(
            v2_err("joint(\"j\", type=\"helical\", q=1) sphere(r=1);"),
            "CGS line 1: helical joint needs pitch="
        );
        assert_eq!(
            v2_err("joint(\"j\", type=\"fixed\", q=1) sphere(r=1);"),
            "CGS line 1: fixed joint takes no q"
        );
        assert_eq!(
            v2_err("joint(\"j\", type=\"cylindrical\", q=1) sphere(r=1);"),
            "CGS line 1: cylindrical joint q must be [qr, qp]"
        );
        assert_eq!(
            v2_err("joint(\"j\", type=\"continuous\", limit=[0, 1]) sphere(r=1);"),
            "CGS line 1: continuous joint takes no limit"
        );
        assert_eq!(
            v2_err("x = joint(\"j\", type=\"fixed\");"),
            "CGS line 1: joint is a statement and cannot be used in an expression"
        );
    }

    #[test]
    fn test_p2_gear_drives_driven() {
        let k = kin_of(
            "joint(\"a\", type=\"revolute\", q=0.4) sphere(r=0.1);\n\
             gear(\"a\", \"b\", ratio=-0.5);\n\
             joint(\"b\", type=\"prismatic\", axis=[0,0,1]) box(s=[0.1,0.1,0.1]);",
        );
        assert_eq!(k.joints.len(), 2);
        assert!(
            (k.joints[1].q[0] + 0.2).abs() < 1e-12,
            "q_b = -0.5 * 0.4 = -0.2: {}",
            k.joints[1].q[0]
        );
        assert!((k.joints[1].world[11] + 0.2).abs() < 1e-9);
        assert_eq!(k.gears.len(), 1);

        let rep = crate::cgs_report(
            "joint(\"a\", type=\"revolute\", q=0.4) sphere(r=0.1);\n\
             gear(\"a\", \"b\", ratio=-0.5);\n\
             joint(\"b\", type=\"prismatic\", axis=[0,0,1]) box(s=[0.1,0.1,0.1]);",
            "",
        )
        .expect("unexpected error");
        assert!(
            rep.contains("gear 0 driver=\"a\" driven=\"b\" ratio=-0.5 offset=0"),
            "{rep}"
        );
        assert!(rep.contains("joints=2 gears=1 cams=0"), "{rep}");
    }

    #[test]
    fn test_p2_gear_errors() {
        assert_eq!(
            v2_err("gear(\"nope\", \"b\", ratio=1);"),
            "CGS line 1: unknown joint nope"
        );
        assert_eq!(
            v2_err("joint(\"b\", type=\"fixed\") sphere(r=1);\ngear(\"b\", \"c\", ratio=1);"),
            "CGS line 2: gear needs 1-DOF joints, got fixed"
        );
        assert_eq!(
            v2_err(
                "joint(\"a\", type=\"revolute\") sphere(r=1);\n\
                 joint(\"b\", type=\"prismatic\") sphere(r=1);\n\
                 gear(\"a\", \"b\", ratio=1);"
            ),
            "CGS line 3: gear must precede the driven joint b"
        );
        assert_eq!(
            v2_err(
                "joint(\"a\", type=\"revolute\") sphere(r=1);\n\
                 gear(\"a\", \"b\", ratio=1);\n\
                 joint(\"b\", type=\"prismatic\", q=1) sphere(r=1);"
            ),
            "CGS line 3: joint b is driven by gear, q must be omitted"
        );
        assert_eq!(
            v2_err(
                "joint(\"a\", type=\"revolute\") sphere(r=1);\n\
                 gear(\"a\", \"b\", ratio=1);\n\
                 gear(\"a\", \"b\", ratio=2);"
            ),
            "CGS line 3: joint b is already driven"
        );
    }

    #[test]
    fn test_p3_cam_circle_circle_roller() {
        // 偏心圆凸轮：圆心 [0.05,0,0]、r=0.1，绕 z 转，q=0。
        // 滚子从动件：r=0.02，锚 [0,0.15,0]，沿 y 移动副。
        // 接触：sqrt(0.05² + (0.15+q)²) = 0.12 → q = sqrt(0.0119) − 0.15。
        let k = kin_of(
            "joint(\"c\", type=\"revolute\", axis=[0,0,1], q=0) sphere(r=0.02);\n\
             cam(\"c\", \"f\", driver_profile=at(circle(r=0.1), [0.05,0,0]), driven_profile=circle(r=0.02));\n\
             joint(\"f\", type=\"prismatic\", axis=[0,1,0], at=[0,0.15,0], limit=[-0.05, 0.05]) sphere(r=0.02);",
        );
        assert_eq!(k.cams.len(), 1);
        let want = 0.0119f64.sqrt() - 0.15;
        assert!(
            (k.cams[0].q - want).abs() < 1e-9,
            "凸轮解应为 {want}，实际 {}",
            k.cams[0].q
        );
        // 从动件世界锚点 = at + q·axis。
        let w = k.joints[1].world;
        assert!((w[7] - (0.15 + want)).abs() < 1e-9, "从动件 y: {}", w[7]);
    }

    #[test]
    fn test_p3_cam_flat_follower() {
        // 平底从动件：平面 n=[0,1,0] 过从动件原点；平面压到 cy + r 时接触。
        // d_plane = 0.15 + q = 0.1 → q = -0.05。
        let k = kin_of(
            "joint(\"c\", type=\"revolute\", axis=[0,0,1], q=0) sphere(r=0.02);\n\
             cam(\"c\", \"f\", driver_profile=at(circle(r=0.1), [0.05,0,0]), driven_profile=plane(n=[0,1,0], d=0));\n\
             joint(\"f\", type=\"prismatic\", axis=[0,1,0], at=[0,0.15,0], limit=[-0.2, 0.2]) sphere(r=0.02);",
        );
        assert!(
            (k.cams[0].q + 0.05).abs() < 1e-9,
            "平底凸轮解应为 -0.05，实际 {}",
            k.cams[0].q
        );
    }

    #[test]
    fn test_p3_cam_errors() {
        // 无解：接触距离超出 limit。
        assert_eq!(
            v2_err(
                "joint(\"c\", type=\"revolute\", axis=[0,0,1], q=0) sphere(r=0.02);\n\
                 cam(\"c\", \"f\", driver_profile=at(circle(r=0.1), [0.05,0,0]), driven_profile=circle(r=0.02));\n\
                 joint(\"f\", type=\"prismatic\", axis=[0,1,0], at=[0,0.15,0], limit=[0.02, 0.05]) sphere(r=0.02);"
            ),
            "CGS line 3: cam found no contact in limit [0.02, 0.05]"
        );
        // 轮廓类型越界。
        assert_eq!(
            v2_err(
                "joint(\"c\", type=\"revolute\", axis=[0,0,1], q=0) sphere(r=0.02);\n\
                 cam(\"c\", \"f\", driver_profile=sphere(r=0.1), driven_profile=circle(r=0.02));"
            ),
            "CGS line 2: cam profiles must be circle(...) or plane(...)"
        );
        // 非平面机构：凸轮轮廓法向不平行于关节轴。
        assert_eq!(
            v2_err(
                "joint(\"c\", type=\"revolute\", axis=[0,0,1], q=0) sphere(r=0.02);\n\
                 cam(\"c\", \"f\", driver_profile=rot(circle(r=0.1), [1,0,0], 0.5), driven_profile=circle(r=0.02));\n\
                 joint(\"f\", type=\"prismatic\", axis=[0,1,0], at=[0,0.15,0], limit=[-0.05, 0.05]) sphere(r=0.02);"
            ),
            "CGS line 3: cam profile normal must be parallel to its joint axis"
        );
        // driven 自带 q。
        assert_eq!(
            v2_err(
                "joint(\"c\", type=\"revolute\", axis=[0,0,1], q=0) sphere(r=0.02);\n\
                 cam(\"c\", \"f\", driver_profile=circle(r=0.1), driven_profile=circle(r=0.02));\n\
                 joint(\"f\", type=\"prismatic\", q=0.1, limit=[-1, 1]) sphere(r=0.02);"
            ),
            "CGS line 3: joint f is driven by cam, q must be omitted"
        );
        // 缺 limit。
        assert_eq!(
            v2_err(
                "joint(\"c\", type=\"revolute\", axis=[0,0,1], q=0) sphere(r=0.02);\n\
                 cam(\"c\", \"f\", driver_profile=circle(r=0.1), driven_profile=circle(r=0.02));\n\
                 joint(\"f\", type=\"prismatic\") sphere(r=0.02);"
            ),
            "CGS line 3: cam driven joint needs limit="
        );
    }

    #[test]
    fn test_p1_1_joint_rpy() {
        use cga_core::transform_point;
        // rpy=[0,0,pi/2] at [1,0,0]：关节系先平移再偏航 pi/2；点 [1,0,0] → [1,1,0]。
        let k =
            kin_of("joint(\"j\", type=\"revolute\", at=[1,0,0], rpy=[0,0,pi/2]) sphere(r=0.1);");
        let p = transform_point(k.joints[0].world, [1.0, 0.0, 0.0]);
        assert!(
            (p[0] - 1.0).abs() < 1e-9 && (p[1] - 1.0).abs() < 1e-9,
            "rpy: {p:?}"
        );
        assert_eq!(k.joints[0].rpy, [0.0, 0.0, std::f64::consts::FRAC_PI_2]);

        // rpy 全零 ≡ 缺省（逐字节同报告）。
        let ra =
            crate::cgs_report("joint(\"j\", type=\"revolute\", q=0.3) sphere(r=0.1);", "").unwrap();
        let rb = crate::cgs_report(
            "joint(\"j\", type=\"revolute\", q=0.3, rpy=[0,0,0]) sphere(r=0.1);",
            "",
        )
        .unwrap();
        assert_eq!(ra, rb, "rpy=0 必须与缺省逐字节一致");

        // rpy 非零进报告。
        assert!(
            rb.contains("q=0.3") && !rb.contains("rpy="),
            "零 rpy 不进报告: {rb}"
        );
        let rc = crate::cgs_report(
            "joint(\"j\", type=\"revolute\", q=0.3, rpy=[0,0,1.570796]) sphere(r=0.1);",
            "",
        )
        .unwrap();
        assert!(rc.contains("rpy=[0,0,1.570796]"), "{rc}");

        // 元数错误。
        assert_eq!(
            v2_err("joint(\"j\", type=\"fixed\", rpy=[0,0]) sphere(r=1);"),
            "CGS line 1: joint.rpy must be [roll, pitch, yaw]"
        );

        // URDF 约定核验：rpy=[roll,0,0] 是绕 x 的 roll（固定轴）。
        let k = kin_of("joint(\"j\", type=\"revolute\", rpy=[pi/2,0,0]) sphere(r=0.1);");
        let p = transform_point(k.joints[0].world, [0.0, 1.0, 0.0]);
        assert!((p[2] - 1.0).abs() < 1e-9, "roll 应把 y 转到 z: {p:?}");
    }

    #[test]
    fn test_p4_pose_variable_override() {
        let src = "theta = 0.5;\njoint(\"j\", type=\"revolute\", q=theta) sphere(r=0.1);";
        let k = cgs_pose(src, "", &[("theta".to_string(), 1.0)])
            .unwrap()
            .kinematics;
        assert!(
            (k.joints[0].q[0] - 1.0).abs() < 1e-12,
            "覆盖后 q=1: {}",
            k.joints[0].q[0]
        );
        let rep = crate::cgs_report_pose(src, "", &[("theta".to_string(), 1.0)]).unwrap();
        assert!(rep.contains("pose theta=1"), "{rep}");
        assert!(rep.contains("q=1"), "{rep}");
        // 无覆盖时报告无 pose 行。
        let r0 = crate::cgs_report(src, "").unwrap();
        assert!(!r0.contains("pose "), "{r0}");
    }

    #[test]
    fn test_p4_pose_joint_override_and_gear_chain() {
        // 关节级覆盖：q 省略时由 pose 提供；gear 链随动。
        let src = "joint(\"a\", type=\"revolute\") sphere(r=0.1);\n\
                   gear(\"a\", \"b\", ratio=-0.5);\n\
                   joint(\"b\", type=\"prismatic\", axis=[0,0,1]) box(s=[0.1,0.1,0.1]);";
        let k = cgs_pose(src, "", &[("a".to_string(), 0.8)])
            .unwrap()
            .kinematics;
        assert!(
            (k.joints[0].q[0] - 0.8).abs() < 1e-12,
            "覆盖 a: {}",
            k.joints[0].q[0]
        );
        assert!(
            (k.joints[1].q[0] + 0.4).abs() < 1e-12,
            "gear 随动：q_b = -0.5*0.8 = -0.4: {}",
            k.joints[1].q[0]
        );
        // driven 关节禁止覆盖。
        assert_eq!(
            cgs_pose(src, "", &[("b".to_string(), 0.1)]).unwrap_err(),
            "CGS line 3: joint b is driven by gear, q must be omitted"
        );
        // 覆盖 + 字面 q 并存报错。
        assert_eq!(
            cgs_pose(
                "joint(\"j\", type=\"revolute\", q=0.3) sphere(r=0.1);",
                "",
                &[("j".to_string(), 0.5)]
            )
            .unwrap_err(),
            "CGS line 1: joint j has a pose override, q must be omitted"
        );
        // 多 DOF 关节不可覆盖。
        assert_eq!(
            cgs_pose(
                "joint(\"j\", type=\"spherical\") sphere(r=0.1);",
                "",
                &[("j".to_string(), 0.5)]
            )
            .unwrap_err(),
            "CGS line 1: pose override needs a 1-DOF joint, got spherical"
        );
        // 覆盖过 limit。
        assert_eq!(
            cgs_pose(
                "joint(\"j\", type=\"revolute\", limit=[-1,1]) sphere(r=0.1);",
                "",
                &[("j".to_string(), 2.0)]
            )
            .unwrap_err(),
            "CGS line 1: joint j q=2.0 outside limit [-1.0, 1.0]"
        );
    }
}
