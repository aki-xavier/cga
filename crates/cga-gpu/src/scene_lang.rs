//! CGS scene language: lexer, single-pass statement dispatcher, evaluator
//! (v2 additions specified in `docs/cgs-v2.md`; v3 P4 suffix chain in
//! `docs/cgs-v3.md`).
//!
//! # Statement forms (dispatch order in `SceneLoader::statement`)
//! 1. `{ … }` block
//! 2. assignment `name = expr;` — checked FIRST, so any keyword can be
//!    shadowed by a variable (`echo = 5;`, `for = 6;`). `background = 0x…;`
//!    doubles as the property form of the background statement (equivalent to
//!    `background(color=0x…);`); `name[…]` is rejected with the indexing error.
//! 3. P4 method chain `name.…` or `name(…).…` (balanced parens, next token
//!    `.`) → expression statement `expr → geom_val → ; → add_geometry` —
//!    pure desugaring `e.f(a)` ≡ `f(e, a)`, sharing the plain call's
//!    dispatch and error texts (`docs/cgs-v3.md` §3).
//! 4. `module` / `for` / `if` / `echo` / `show` / `tag` / `drill` / `var` /
//!    `constrain(…) { … } solve;`
//! 5. `union` / `difference` / `intersection` (block or expression form)
//! 6. modifiers `translate|rotate|scale|mirror|material` (missing target →
//!    `modifier missing target statement`), property statements
//!    `background|camera|*_light`, then primitive statements
//!    (sphere/box/cylinder/… /mesh via `build_geometry`).
//!
//! # Statement/expression boundary error contract
//! Canonical per-group texts (README "语句/表达式边界与错误契约"); each is
//! deterministic so an LLM verifier can assert on it:
//! - statement keyword / modifier / property called in an expression →
//!   `CGS line N: {name} is a statement and cannot be used in an expression`
//!   (the same text a chain method spelling like `g.show()` produces)
//! - assignment appearing inside an expression →
//!   `CGS line N: assignment is a statement and cannot be used in an expression`
//! - expression function (math/query/at|rot|scaled) used as a statement →
//!   `CGS line N: {name} is an expression function and cannot be used as a statement`
//! - list indexing `a[…]` in either position →
//!   `CGS line N: indexing is not supported — use comp(vector, index)`
//! - unknown statement name → `CGS line N: unknown primitive {name}`

use std::collections::{BTreeMap, HashMap};
use std::fmt;

use cga_core::{
    affine_geometry, box_geometry, circle_geometry, clamp01, cone_geometry, csg_geometry,
    cyclide_geometry, cylinder_geometry, decompose_rigid, ellipsoid_geometry, extrude, load_obj,
    loft, mat4_identity, mat4_mul, motor_identity, motor_rotor, plane_geometry, sphere_geometry,
    torus_geometry, transform_point, transformed_geometry, trimesh_geometry, validate_profile,
    CsgOp, Geometry,
};

use crate::mesh_io_gltf::{gltf_to_geometry, load_gltf};
use crate::scene::{mesh, perspective_camera, scene, MeshParams, PerspectiveCamera, Scene};
use crate::scene_graph::{color_hex, is_identity3, vec3_dot};
use crate::shading::{
    ambient_light, basic_material, directional_light, point_light, standard_material, Material,
    MaterialParams,
};
use crate::texture::texture_load;

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
    Dot,
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
            TokenKind::Dot => "dot",
        };
        f.write_str(s)
    }
}

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

#[derive(Clone, Copy, Debug)]
pub struct CgsVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A geometry value in expression position: a shape plus its own local
/// frame (identity unless [`at`/`rot`/`scaled`] set one). Bindings are
/// frameless by design — placement comes from where the value is `show`n.
/// See `docs/cgs-v2.md` §4.2 for the frame rules.
#[derive(Clone, Debug)]
pub struct GeomVal {
    pub geo: Geometry,
    pub m4: [f64; 16],
}

#[derive(Clone, Debug)]
pub enum CgsValue {
    Num(f64),
    Bool(bool),
    Str(String),
    List(Vec<CgsValue>),
    Vec3(CgsVec3),
    Geom(GeomVal),
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
            CgsValue::Geom(_) => f.write_str("<geom>"),
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

/// 4x4 inverse (Gauss-Jordan). Row-major row-vector convention: p' = p·M,
/// `mat4_mul(a, b)` applies `a` first, then `b`. Singular input falls back to
/// identity (only reachable from degenerate scale/mirror contexts).
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

/// World AABB of a local AABB under `m` (8 corners transformed).
fn transform_bbox(b: [[f64; 3]; 2], m: [f64; 16]) -> [[f64; 3]; 2] {
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

/// Face key `"+x"`/`"-y"`/… → `(axis, sign)`. Shared by the `face`/`fnrm`
/// queries and drill's `"name:key"` extent references (docs/cgs-v2.md §7).
fn parse_face_key(key: &str, line: i32, what: &str) -> Result<(usize, f64), String> {
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

/// Local face point and outward normal for a face key (docs/cgs-v2.md §7):
/// box/cylinder/cone/sphere/ellipsoid exact per the primitive's parameters
/// (cylinder & cone caps exact, sides on the surface at the axial midpoint);
/// everything else (csg/mesh/torus/…) falls back to the AABB face center.
/// `None` = no finite bounds (plane).
fn face_local(geo: &Geometry, axis: usize, sign: f64) -> Option<([f64; 3], [f64; 3])> {
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
                // cap: center of the end disc (exact)
                let mut p = [0.0; 3];
                p[2] = sign * c.half;
                Some((p, k))
            } else {
                // side: tangent point at the axial midpoint (on the surface)
                let mut p = [0.0; 3];
                p[axis] = sign * c.radius;
                Some((p, k))
            }
        }
        Geometry::ConeGeometry(c) => {
            // canonical cone: apex at z=+h/2, base disc (radius r) at z=-h/2
            if axis == 2 {
                let mut p = [0.0; 3];
                p[2] = if sign > 0.0 {
                    c.height / 2.0
                } else {
                    -c.height / 2.0
                };
                Some((p, k))
            } else {
                // side: on the surface at the axial midpoint (radius r/2);
                // normal ⟂ generator, outward and tilted toward the apex
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
            // AABB fallback for csg / mesh / torus / cyclide / circle / …
            let b = crate::geometry_ops::geom_bounds(&cga_core::bake::identity_params(geo))?;
            let mut p = [0.0; 3];
            for i in 0..3 {
                p[i] = (b[0][i] + b[1][i]) / 2.0;
            }
            p[axis] = if sign > 0.0 { b[1][axis] } else { b[0][axis] };
            Some((p, k))
        }
    }
}

/// Transform a normal by the inverse-transpose 3×3 (correct under non-uniform
/// scale / shear, same as a plain rotation otherwise), then normalize.
fn transform_normal(m: [f64; 16], n: [f64; 3]) -> [f64; 3] {
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

/// A drill axial endpoint: a coordinate in the cutter frame, or a face
/// reference `"instance:key"` resolved at statement execution time.
enum DrillEnd {
    Num(f64),
    Face {
        name: String,
        axis: usize,
        sign: f64,
    },
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

/// Flatten a constraint value into residuals: Num → 1, Vec3 → 3, List → parts.
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

/// Solve `a · x = b` by Gaussian elimination with partial pivoting.
/// Returns None on a (near-)singular pivot; the caller raises λ and retries.
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

/// Geometry constructors usable in expression position (frameless values).
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
];

/// Reference-query functions (Str name / Geom value dispatch).
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

/// Statement forms with no meaning inside an expression. Calling one with `(`
/// in expression position yields the canonical boundary error instead of a
/// confusing arity/type complaint (README "语句/表达式边界与错误契约").
fn is_statement_only_fn(name: &str) -> bool {
    matches!(
        name,
        "show"
            | "tag"
            | "drill"
            | "var"
            | "constrain"
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

/// Pure expression constructs (math functions of `cgs_call_fn`, reference
/// queries, expression geometry helpers). Using one as a statement yields the
/// canonical boundary error instead of `unknown primitive`.
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

pub struct SceneLoader {
    toks: Vec<CgsToken>,
    pos: usize,
    asset_root: String,
    scene: Scene,
    camera: Option<PerspectiveCamera>,
    modules: HashMap<String, Vec<CgsToken>>,
    params: HashMap<String, Vec<CgsToken>>,
    param_order: Vec<String>,
    collect: Vec<CollectedGeom>,
    collecting: bool,
    /// Instances registered by active `tag` statements: world frame for
    /// reference queries, rel frame (relative to the tag statement's entry
    /// context) for `drill` reuse in another context.
    named: HashMap<String, Vec<Inst>>,
    /// Stack of active (tag name, entry context) frames.
    pending: Vec<(String, [f64; 16])>,
}

#[derive(Clone, Debug)]
struct Inst {
    geo: Geometry,
    world: [f64; 16],
    rel: [f64; 16],
}

struct CollectedGeom {
    geo: Geometry,
    m4: [f64; 16],
}

/// One geometry instance registered by an active `tag` statement. `world` is
/// the emission world frame (the element frame `instances()` reads), `rel` the
/// frame relative to the tag statement's entry context (`drill` reuse); `geo`
/// is the geometry as registered (pre-transform). Exposed by `cgs_run_result`
/// for `scene_report` (`docs/scene-report.md` §8).
#[derive(Clone, Debug)]
pub struct TagInstance {
    pub geo: Geometry,
    pub world: [f64; 16],
    pub rel: [f64; 16],
}

/// All tag instances of one load, keyed by tag name. `BTreeMap` (not the
/// loader's `HashMap`) so report output is deterministic: names iterate in
/// lexicographic order; instance order inside each `Vec` is emission order.
pub type TagRegistry = BTreeMap<String, Vec<TagInstance>>;

/// Full load result: scene + camera + the tag registry that `cgs_load`
/// historically dropped at its exit.
#[derive(Clone, Debug)]
pub struct CgsRun {
    pub scene: Scene,
    pub camera: PerspectiveCamera,
    pub tags: TagRegistry,
}

pub fn cgs_load(text: &str, asset_root: &str) -> (Scene, PerspectiveCamera) {
    match cgs_load_result(text, asset_root) {
        Ok(r) => r,
        Err(e) => panic!("{e}"),
    }
}

pub fn cgs_load_result(text: &str, asset_root: &str) -> Result<(Scene, PerspectiveCamera), String> {
    cgs_run_result(text, asset_root).map(|r| (r.scene, r.camera))
}

/// `cgs_load` with the tag registry kept — panics on error like `cgs_load`.
pub fn cgs_run(text: &str, asset_root: &str) -> CgsRun {
    match cgs_run_result(text, asset_root) {
        Ok(r) => r,
        Err(e) => panic!("{e}"),
    }
}

/// Error path is identical to the historical `cgs_load_result`: same code,
/// same texts, same line numbers (`docs/scene-report.md` §9, zero new errors).
pub fn cgs_run_result(text: &str, asset_root: &str) -> Result<CgsRun, String> {
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
        named: HashMap::new(),
        pending: Vec::new(),
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
    let tags = l
        .named
        .into_iter()
        .map(|(name, insts)| {
            let v = insts
                .into_iter()
                .map(|i| TagInstance {
                    geo: i.geo,
                    world: i.world,
                    rel: i.rel,
                })
                .collect();
            (name, v)
        })
        .collect();
    Ok(CgsRun {
        scene: l.scene,
        camera: cam,
        tags,
    })
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
        let v = self.atom(scope)?;
        self.suffix_chain(v, scope)
    }

    /// Atom: one primary token — literal, group, list, call or variable —
    /// the `primary` body before P4 layered the postfix chain on top
    /// (`docs/cgs-v3.md` §3). Recursion through groups re-enters `primary`,
    /// so `(…).f()` chains too, and unary binds looser than the chain:
    /// `-x.abs()` ≡ `-(abs(x))`.
    fn atom(&mut self, scope: &HashMap<String, CgsValue>) -> Result<CgsValue, String> {
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
            // Canonical statement/expression boundary errors (README
            // "语句/表达式边界与错误契约"): assignment and indexing have no
            // meaning inside an expression.
            if self.peek().kind == TokenKind::Assign {
                return Err(format!(
                    "CGS line {}: assignment is a statement and cannot be used in an expression",
                    t.line
                ));
            }
            if self.peek().kind == TokenKind::Lbracket {
                return Err(format!(
                    "CGS line {}: indexing is not supported — use comp(vector, index)",
                    t.line
                ));
            }
            if self.peek().kind == TokenKind::Lparen {
                // Boundary check before the args are parsed, as always;
                // `call_dispatch` re-runs it for the chain path, which only
                // reaches the method name after them.
                if is_statement_only_fn(&t.text) {
                    return Err(format!(
                        "CGS line {}: {} is a statement and cannot be used in an expression",
                        t.line, t.text
                    ));
                }
                self.take();
                let (pos, kw) = self.paren_args(scope)?;
                return self.call_dispatch(&t.text, t.line, pos, kw);
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

    /// Expression-position call dispatch — one code path shared verbatim by
    /// plain calls `f(a, b)` and P4's suffix chain `e.f(a, b)` (receiver
    /// spliced in as the first positional argument): statement/expression
    /// boundary → frame helpers / CSG / reference queries / geometry
    /// constructors → named-argument check → plain function table. The chain
    /// therefore inherits every canonical error text unchanged
    /// (`docs/cgs-v3.md` §3.2: 糖不是新语义).
    fn call_dispatch(
        &mut self,
        n: &str,
        line: i32,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
    ) -> Result<CgsValue, String> {
        if is_statement_only_fn(n) {
            return Err(format!(
                "CGS line {line}: {n} is a statement and cannot be used in an expression"
            ));
        }
        let is_geom = GEOM_EXPR_NAMES.contains(&n)
            || QUERY_FNS.contains(&n)
            || matches!(
                n,
                "at" | "rot" | "scaled" | "difference" | "intersection" | "union"
            );
        if is_geom {
            if let Some(v) = self.geom_expr_call(n, pos.clone(), kw.clone(), line)? {
                return Ok(v);
            }
        }
        if !kw.is_empty() {
            return Err(format!(
                "CGS line {line}: function {n} takes no named arguments"
            ));
        }
        cgs_call_fn(n, &pos, line)
    }

    /// P4 suffix chain: `e.f(a, b)` ≡ `f(e, a, b)` — the tightest-binding
    /// postfix operator, applied to any atom before binary operators run.
    /// After the method name the syntax *is* a plain call — `expect(lparen)`
    /// → args → [`Self::call_dispatch`] — so arity, type, boundary and
    /// unknown-function errors are the existing texts, byte for byte
    /// (`docs/cgs-v3.md` §3.3).
    fn suffix_chain(
        &mut self,
        mut v: CgsValue,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<CgsValue, String> {
        while self.peek().kind == TokenKind::Dot {
            self.take(); // `.`
            let m = self.take();
            if m.kind != TokenKind::Ident {
                return Err(format!(
                    "CGS line {}: expected {}, got {}",
                    m.line,
                    TokenKind::Ident,
                    m.kind
                ));
            }
            self.expect(TokenKind::Lparen)?;
            let (mut pos, kw) = self.paren_args(scope)?;
            pos.insert(0, v);
            v = self.call_dispatch(&m.text, m.line, pos, kw)?;
        }
        Ok(v)
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

    /// Expression-position call dispatch: frame helpers (`at`/`rot`/`scaled`),
    /// CSG composition, reference queries and geometry constructors. Returns
    /// `Ok(None)` when `name` is an ordinary numeric function (the caller then
    /// falls through to `cgs_call_fn`).
    fn geom_expr_call(
        &mut self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<Option<CgsValue>, String> {
        match name {
            "at" | "rot" | "scaled" => {
                if !kw.is_empty() {
                    return Err(format!(
                        "CGS line {line}: {name} takes positional arguments only"
                    ));
                }
                let base = geom_val(pos.first().unwrap_or(&CgsValue::Num(0.0)), line, name)?;
                // Column-vector chain: the new transform wraps (is outer to)
                // the value's own frame, exactly like a statement modifier
                // would (`translate(v) g` ≡ `at(g, v)`).
                let m = match name {
                    "at" => {
                        if pos.len() != 2 {
                            return Err(format!("CGS line {line}: at needs (geometry, offset)"));
                        }
                        let off = cgs_vec3(&pos[1], line, "at.off")?;
                        mat4_mul(translate4(off), base.m4)
                    }
                    "rot" => {
                        if pos.len() != 3 {
                            return Err(format!(
                                "CGS line {line}: rot needs (geometry, axis, angle)"
                            ));
                        }
                        let ax = cgs_vec3(&pos[1], line, "rot.axis")?;
                        let ang = cgs_num(&pos[2], line, "rot.angle")?;
                        mat4_mul(motor_rotor(ax, ang).to_matrix(), base.m4)
                    }
                    _ => {
                        if pos.len() != 2 {
                            return Err(format!(
                                "CGS line {line}: scaled needs (geometry, factor)"
                            ));
                        }
                        let s4 = match &pos[1] {
                            CgsValue::Vec3(v3) => scale4([v3.x, v3.y, v3.z]),
                            CgsValue::List(_) => scale4(cgs_vec3(&pos[1], line, "scaled.s")?),
                            _ => {
                                let x = cgs_num(&pos[1], line, "scaled.s")?;
                                scale4([x, x, x])
                            }
                        };
                        mat4_mul(s4, base.m4)
                    }
                };
                Ok(Some(CgsValue::Geom(GeomVal {
                    geo: base.geo,
                    m4: m,
                })))
            }
            "difference" | "intersection" | "union" => {
                if !kw.is_empty() {
                    return Err(format!("CGS line {line}: {name} takes no named arguments"));
                }
                if pos.len() < 2 {
                    return Err(format!(
                        "CGS line {line}: {name} needs >= 2 geometry arguments"
                    ));
                }
                let op = match name {
                    "difference" => CsgOp::Difference,
                    "intersection" => CsgOp::Intersection,
                    _ => CsgOp::Union,
                };
                let mut kids: Vec<Geometry> = Vec::new();
                for v in &pos {
                    let g = geom_val(v, line, name)?;
                    if matches!(g.geo, Geometry::CircleGeometry(_)) {
                        return Err(format!(
                            "CGS line {line}: {name} children must be solids (circle is not)"
                        ));
                    }
                    let (cm, cl) = decompose_rigid(g.m4);
                    kids.push(Geometry::AffineGeometry(transformed_geometry(
                        g.geo, cm, cl,
                    )));
                }
                Ok(Some(CgsValue::Geom(GeomVal {
                    geo: Geometry::CsgGeometry(csg_geometry(op, kids)),
                    m4: mat4_identity(),
                })))
            }
            _ if QUERY_FNS.contains(&name) => Ok(Some(self.eval_query(name, pos, kw, line)?)),
            _ if GEOM_EXPR_NAMES.contains(&name) => {
                let args = self.resolve(name, pos, kw, line)?;
                let geo = self.build_geometry(name, &args, line)?;
                Ok(Some(CgsValue::Geom(GeomVal {
                    geo,
                    m4: mat4_identity(),
                })))
            }
            _ => Ok(None),
        }
    }

    /// Reference query: `Str` = tagged instance (world frame), `Geom` = value
    /// (own frame). Returns the frame plus the world AABB (None = unbounded).
    /// `(frame, geometry)` for `face`/`fnrm` and drill face references.
    /// Instances resolve to their FIRST instance (a face of a multi-instance
    /// tag is ambiguous; bbox queries keep unioning instead).
    fn face_target<'a>(
        &'a self,
        v: &'a CgsValue,
        line: i32,
        what: &str,
    ) -> Result<([f64; 16], &'a Geometry), String> {
        match v {
            CgsValue::Str(s) => {
                let insts = match self.named.get(s) {
                    Some(list) if !list.is_empty() => list,
                    _ => {
                        return Err(format!("CGS line {line}: unknown reference \"{s}\""));
                    }
                };
                Ok((insts[0].world, &insts[0].geo))
            }
            CgsValue::Geom(g) => Ok((g.m4, &g.geo)),
            _ => Err(format!(
                "CGS line {line}: {what} needs a reference name or geometry, got {v}"
            )),
        }
    }

    /// Axial coordinate of a drill endpoint in the cutter frame `f`.
    fn drill_end_coord(
        &self,
        e: &DrillEnd,
        f: [f64; 16],
        ax: usize,
        line: i32,
    ) -> Result<f64, String> {
        match e {
            DrillEnd::Num(n) => Ok(*n),
            DrillEnd::Face { name, axis, sign } => {
                let insts = match self.named.get(name) {
                    Some(list) if !list.is_empty() => list,
                    _ => {
                        return Err(format!("CGS line {line}: unknown reference \"{name}\""));
                    }
                };
                let inst = &insts[0];
                let (p_local, _) = face_local(&inst.geo, *axis, *sign).ok_or_else(|| {
                    format!("CGS line {line}: drill face reference has no finite bounds")
                })?;
                let p_world = transform_point(inst.world, p_local);
                Ok(transform_point(mat4_inv(f), p_world)[ax])
            }
        }
    }

    fn query_target(
        &self,
        v: &CgsValue,
        line: i32,
        what: &str,
    ) -> Result<([f64; 16], Option<[[f64; 3]; 2]>), String> {
        match v {
            CgsValue::Str(s) => {
                let insts = match self.named.get(s) {
                    Some(list) if !list.is_empty() => list,
                    _ => {
                        return Err(format!("CGS line {line}: unknown reference \"{s}\""));
                    }
                };
                let mut acc: Option<[[f64; 3]; 2]> = None;
                for i in insts {
                    if let Some(b) = self
                        .local_bounds(&i.geo)
                        .map(|b| transform_bbox(b, i.world))
                    {
                        acc = Some(match acc {
                            None => b,
                            Some(a) => [
                                [
                                    a[0][0].min(b[0][0]),
                                    a[0][1].min(b[0][1]),
                                    a[0][2].min(b[0][2]),
                                ],
                                [
                                    a[1][0].max(b[1][0]),
                                    a[1][1].max(b[1][1]),
                                    a[1][2].max(b[1][2]),
                                ],
                            ],
                        });
                    }
                }
                Ok((insts[0].world, acc))
            }
            CgsValue::Geom(g) => Ok((
                g.m4,
                self.local_bounds(&g.geo).map(|b| transform_bbox(b, g.m4)),
            )),
            _ => Err(format!(
                "CGS line {line}: {what} needs a reference name or geometry, got {v}"
            )),
        }
    }

    fn local_bounds(&self, geo: &Geometry) -> Option<[[f64; 3]; 2]> {
        crate::geometry_ops::geom_bounds(&cga_core::bake::identity_params(geo))
    }

    fn eval_query(
        &self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<CgsValue, String> {
        if !kw.is_empty() {
            return Err(format!("CGS line {line}: {name} takes no named arguments"));
        }
        let mut pos = pos;
        // P5 optional sub-item (docs/cgs-v3.md §4.4): `face/fnrm` accept the
        // drill-style single string "name:key" — split it into the two-
        // parameter form and fall through the existing path unchanged (bad
        // key → `parse_face_key`, unknown name → `unknown reference`, a
        // string without ':' → the normal arity error). Zero new error texts.
        if matches!(name, "face" | "fnrm") && pos.len() == 1 {
            if let CgsValue::Str(s) = &pos[0] {
                if let Some((nm, key)) = s.split_once(':') {
                    pos = vec![
                        CgsValue::Str(nm.to_string()),
                        CgsValue::Str(key.to_string()),
                    ];
                }
            }
        }
        let need = if matches!(name, "dist" | "face" | "fnrm") {
            2
        } else {
            1
        };
        if pos.len() != need {
            return Err(format!("CGS line {line}: {name} needs {need} argument(s)"));
        }
        if name == "instances" {
            // P5 (docs/cgs-v3.md §4.1): the tag's instance set as a
            // first-class value — element frames are the stored world
            // matrices (same frame `center("name")` unions over) and order is
            // the registry's push order = emission order, so `for` walks the
            // set exactly as it was built. The one new error text of P5.
            let s = match &pos[0] {
                CgsValue::Str(s) => s,
                v => {
                    return Err(format!(
                        "CGS line {line}: instances needs a reference name, got {v}"
                    ));
                }
            };
            let insts = match self.named.get(s) {
                Some(list) if !list.is_empty() => list,
                _ => {
                    return Err(format!("CGS line {line}: unknown reference \"{s}\""));
                }
            };
            return Ok(CgsValue::List(
                insts
                    .iter()
                    .map(|i| {
                        CgsValue::Geom(GeomVal {
                            geo: i.geo.clone(),
                            m4: i.world,
                        })
                    })
                    .collect(),
            ));
        }
        if matches!(name, "face" | "fnrm") {
            // key first: a malformed key is reported even when the target
            // reference itself does not exist
            let key = match &pos[1] {
                CgsValue::Str(s) => s.clone(),
                v => {
                    return Err(format!(
                        "CGS line {line}: {name} key must be a string like \"+x\", got {v}"
                    ));
                }
            };
            let (axis, sign) = parse_face_key(&key, line, name)?;
            let (m, geo) = self.face_target(&pos[0], line, name)?;
            let (p, n) = face_local(geo, axis, sign).ok_or_else(|| {
                format!("CGS line {line}: {name}: reference has no finite bounds")
            })?;
            let w = if name == "face" {
                transform_point(m, p)
            } else {
                transform_normal(m, n)
            };
            return Ok(CgsValue::Vec3(CgsVec3 {
                x: w[0],
                y: w[1],
                z: w[2],
            }));
        }
        if name == "dist" {
            let a = self.query_point(&pos[0], line, name)?;
            let b = self.query_point(&pos[1], line, name)?;
            let d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
            return Ok(CgsValue::Num(
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt(),
            ));
        }
        if matches!(name, "xdir" | "ydir" | "zdir") {
            let (m, _) = self.query_target(&pos[0], line, name)?;
            // Column-vector convention: local axes map to matrix columns.
            let col = match name {
                "xdir" => [m[0], m[4], m[8]],
                "ydir" => [m[1], m[5], m[9]],
                _ => [m[2], m[6], m[10]],
            };
            let n = (col[0] * col[0] + col[1] * col[1] + col[2] * col[2]).sqrt();
            if n < 1e-12 {
                return Err(format!("CGS line {line}: {name} is degenerate"));
            }
            return Ok(CgsValue::Vec3(CgsVec3 {
                x: col[0] / n,
                y: col[1] / n,
                z: col[2] / n,
            }));
        }
        let (_, b) = self.query_target(&pos[0], line, name)?;
        let b = match b {
            Some(b) => b,
            None => {
                return Err(format!(
                    "CGS line {line}: {name}: reference has no finite bounds"
                ));
            }
        };
        let v = match name {
            "center" => [
                (b[0][0] + b[1][0]) / 2.0,
                (b[0][1] + b[1][1]) / 2.0,
                (b[0][2] + b[1][2]) / 2.0,
            ],
            "lo" => b[0],
            "hi" => b[1],
            _ => [b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]],
        };
        Ok(CgsValue::Vec3(CgsVec3 {
            x: v[0],
            y: v[1],
            z: v[2],
        }))
    }

    /// Center point for `dist`: Vec3 passes through, references resolve to
    /// their AABB center.
    fn query_point(&self, v: &CgsValue, line: i32, what: &str) -> Result<[f64; 3], String> {
        if let CgsValue::Vec3(v3) = v {
            return Ok([v3.x, v3.y, v3.z]);
        }
        let (_, b) = self.query_target(v, line, what)?;
        let b = match b {
            Some(b) => b,
            None => {
                return Err(format!(
                    "CGS line {line}: {what}: reference has no finite bounds"
                ));
            }
        };
        Ok([
            (b[0][0] + b[1][0]) / 2.0,
            (b[0][1] + b[1][1]) / 2.0,
            (b[0][2] + b[1][2]) / 2.0,
        ])
    }

    /// Register a rendered instance under every active `tag`.
    fn register(&mut self, geo: &Geometry, world: [f64; 16], emit: [f64; 16]) {
        if self.pending.is_empty() {
            return;
        }
        for i in 0..self.pending.len() {
            let (name, entry) = self.pending[i].clone();
            let rel = mat4_mul(mat4_inv(entry), emit);
            self.named.entry(name).or_default().push(Inst {
                geo: geo.clone(),
                world,
                rel,
            });
        }
    }

    /// `show(g);` — render a geometry value at `ctx ∘ g.m4`.
    fn show_stmt(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.take();
        let (pos, kw) = self.call_args(scope)?;
        if !kw.is_empty() || pos.len() != 1 {
            return Err(format!("CGS line {line}: show takes one geometry argument"));
        }
        let g = geom_val(&pos[0], line, "show")?;
        self.expect(TokenKind::Semi)?;
        self.add_geometry(g.geo, mat4_mul(ctx, g.m4), mat)
    }

    /// `tag("name") <statement>` — register the statement's emitted
    /// instances under `name` for reference queries.
    fn tag_stmt(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.take();
        self.expect(TokenKind::Lparen)?;
        let nt = self.take();
        if nt.kind != TokenKind::Str {
            return Err(format!("CGS line {}: tag needs a name string", nt.line));
        }
        self.expect(TokenKind::Rparen)?;
        self.pending.push((nt.text, ctx));
        let r = self.body(ctx, mat, scope, line);
        self.pending.pop();
        r
    }

    /// `drill(r=…, through=…, axis=…[, from=…, to=…]);` — a through-cutter
    /// whose axial extent tracks the target's bounding box (derived-feature
    /// following, docs/cgs-v2.md §5.1). The cutter's frame is `ctx ∘ rel`,
    /// so it replays the target's placement inside the current context
    /// instead of double-applying the emit context.
    fn drill_stmt(
        &mut self,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.take();
        let (pos, kw) = self.call_args(scope)?;
        if !pos.is_empty() {
            return Err(format!(
                "CGS line {line}: drill takes named arguments: r, through, axis[, from, to]"
            ));
        }
        let r = match kw.get("r") {
            Some(v) => cgs_num(v, line, "drill.r")?,
            None => return Err(format!("CGS line {line}: drill needs r=")),
        };
        if !(r > 0.0) {
            return Err(format!("CGS line {line}: drill.r must be > 0"));
        }
        // from/to endpoints: number = coordinate in the cutter frame,
        // "name:key" = face reference (world face point → frame coords).
        let from_ref = drill_end_ref(kw.get("from"), line, "from")?;
        let to_ref = drill_end_ref(kw.get("to"), line, "to")?;
        let face_axes: Vec<(usize, f64)> = [&from_ref, &to_ref]
            .iter()
            .filter_map(|e| match e {
                Some(DrillEnd::Face { axis, sign, .. }) => Some((*axis, *sign)),
                _ => None,
            })
            .collect();

        // axis: explicit, else inferred from the face-reference keys — the
        // "name:key" form replaces axis + bounding box (docs/cgs-v2.md §7).
        let ax = match kw.get("axis") {
            Some(v) => {
                let a = cgs_num(v, line, "drill.axis")?;
                if a != 0.0 && a != 1.0 && a != 2.0 {
                    return Err(format!(
                        "CGS line {line}: drill.axis must be 0, 1 or 2 (X/Y/Z)"
                    ));
                }
                let ax = a as usize;
                if let Some(&(ka, ks)) = face_axes.first() {
                    if ka != ax {
                        return Err(format!(
                            "CGS line {line}: drill face key \"{}\" does not match axis={ax}",
                            face_key_str(ka, ks)
                        ));
                    }
                }
                ax
            }
            None => match face_axes.first() {
                None => {
                    return Err(format!("CGS line {line}: drill needs axis= (0/1/2)"));
                }
                Some(&(ka, ks)) => {
                    for &(kb, kb_sign) in &face_axes {
                        if kb != ka {
                            return Err(format!(
                                "CGS line {line}: from/to face references use different axes (\"{}\" vs \"{}\")",
                                face_key_str(ka, ks),
                                face_key_str(kb, kb_sign)
                            ));
                        }
                    }
                    ka
                }
            },
        };

        // Cutter frame F = ctx ∘ rel; target box (default extent) in F coords.
        // through is optional when from/to fully specify the extent.
        let (rel, fbox): ([f64; 16], Option<[[f64; 3]; 2]>) = match kw.get("through") {
            Some(CgsValue::Str(s)) => {
                let insts = match self.named.get(s) {
                    Some(list) if !list.is_empty() => list,
                    _ => return Err(format!("CGS line {line}: unknown reference \"{s}\"")),
                };
                let rel0 = insts[0].rel;
                let f = mat4_mul(ctx, rel0);
                let finv = mat4_inv(f);
                let mut acc: Option<[[f64; 3]; 2]> = None;
                for i in insts {
                    // World box of each instance (Csg instances carry an
                    // identity slot with the frame baked into the geometry).
                    if let Some(b) = self
                        .local_bounds(&i.geo)
                        .map(|b| transform_bbox(b, i.world))
                    {
                        acc = Some(match acc {
                            None => b,
                            Some(a) => [
                                [
                                    a[0][0].min(b[0][0]),
                                    a[0][1].min(b[0][1]),
                                    a[0][2].min(b[0][2]),
                                ],
                                [
                                    a[1][0].max(b[1][0]),
                                    a[1][1].max(b[1][1]),
                                    a[1][2].max(b[1][2]),
                                ],
                            ],
                        });
                    }
                }
                let b = acc
                    .ok_or_else(|| format!("CGS line {line}: drill target has no finite bounds"))?;
                (rel0, Some(transform_bbox(b, finv)))
            }
            Some(CgsValue::Geom(g)) => {
                // F contains g.m4, so the box is the geometry's local one.
                let b = self
                    .local_bounds(&g.geo)
                    .ok_or_else(|| format!("CGS line {line}: drill target has no finite bounds"))?;
                (g.m4, Some(b))
            }
            Some(other) => {
                return Err(format!(
                    "CGS line {line}: drill.through needs a reference name or geometry, got {other}"
                ));
            }
            None => (mat4_identity(), None),
        };
        // A missing endpoint falls back to the target bounding box (through=).
        if (from_ref.is_none() || to_ref.is_none()) && fbox.is_none() {
            return Err(format!(
                "CGS line {line}: drill needs through=<name or geometry>"
            ));
        }

        let f = mat4_mul(ctx, rel);
        let a0 = if let Some(e) = &from_ref {
            self.drill_end_coord(e, f, ax, line)?
        } else {
            fbox.as_ref()
                .map(|b| b[0][ax])
                .ok_or_else(|| format!("CGS line {line}: drill needs through=<name or geometry>"))?
        };
        let a1 = if let Some(e) = &to_ref {
            self.drill_end_coord(e, f, ax, line)?
        } else {
            fbox.as_ref()
                .map(|b| b[1][ax])
                .ok_or_else(|| format!("CGS line {line}: drill needs through=<name or geometry>"))?
        };
        if !(a1 > a0) {
            return Err(format!(
                "CGS line {line}: drill extent must be non-empty ({a0} .. {a1})"
            ));
        }
        let center = (a0 + a1) / 2.0;
        // Chain (innermost first): orient the +Z cylinder onto `axis`,
        // offset along the axis, then apply the cutter frame.
        let rmat = match ax {
            0 => motor_rotor([0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2).to_matrix(),
            1 => motor_rotor([1.0, 0.0, 0.0], -std::f64::consts::FRAC_PI_2).to_matrix(),
            _ => mat4_identity(),
        };
        let mut off = [0.0, 0.0, 0.0];
        off[ax] = center;
        let w = mat4_mul(mat4_mul(f, translate4(off)), rmat);
        self.add_geometry(
            Geometry::CylinderGeometry(cylinder_geometry(r, a1 - a0)),
            w,
            mat,
        )?;
        self.expect(TokenKind::Semi)?;
        Ok(())
    }

    /// `var name = expr;` — declares a numeric solve unknown (docs/cgs-v2.md
    /// §6.1). Executes as a plain assignment; the `var` form documents that
    /// the value is meant to appear in a later `constrain(…) solve;`.
    fn var_stmt(&mut self, scope: &mut HashMap<String, CgsValue>, line: i32) -> Result<(), String> {
        self.take();
        let nt = self.take();
        if nt.kind != TokenKind::Ident {
            return Err(format!("CGS line {line}: var needs a name"));
        }
        if self.peek().kind != TokenKind::Assign {
            return Err(format!(
                "CGS line {}: var {} needs = <number>",
                nt.line, nt.text
            ));
        }
        self.take();
        let v = self.expr(scope, 1)?;
        let n = match v {
            CgsValue::Num(n) => n,
            _ => {
                return Err(format!(
                    "CGS line {}: var {} must be a number (solve unknowns are numeric)",
                    line, nt.text
                ));
            }
        };
        self.expect(TokenKind::Semi)?;
        scope.insert(nt.text, CgsValue::Num(n));
        Ok(())
    }

    /// `constrain(v1, …) { lhs == rhs; … } solve;` — compile-time
    /// Levenberg-damped Gauss-Newton solve (docs/cgs-v2.md §6.2). The block
    /// is captured as tokens and re-evaluated with trial values overlaid on
    /// the scope; on success the solution is written back, so every
    /// statement after `solve` reads baked constants (text stays truth).
    fn constrain_stmt(
        &mut self,
        scope: &mut HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<(), String> {
        self.take();
        self.expect(TokenKind::Lparen)?;
        let mut unknowns: Vec<String> = Vec::new();
        if self.peek().kind == TokenKind::Rparen {
            return Err(format!(
                "CGS line {line}: constrain needs at least one unknown"
            ));
        }
        loop {
            let t = self.take();
            if t.kind != TokenKind::Ident {
                return Err(format!(
                    "CGS line {}: constrain unknown must be a name",
                    t.line
                ));
            }
            unknowns.push(t.text);
            if self.peek().kind == TokenKind::Comma {
                self.take();
            } else {
                break;
            }
        }
        self.expect(TokenKind::Rparen)?;
        self.expect(TokenKind::Lbrace)?;
        // capture the constraint body up to its matching '}'
        let start = self.pos;
        let mut depth = 1i32;
        while depth > 0 {
            if self.pos >= self.toks.len() {
                return Err(format!("CGS line {line}: unterminated constrain block"));
            }
            let k = self.take().kind;
            if k == TokenKind::Lbrace {
                depth += 1;
            } else if k == TokenKind::Rbrace {
                depth -= 1;
            }
        }
        let body = self.toks[start..self.pos - 1].to_vec();
        let st = self.take();
        if !(st.kind == TokenKind::Ident && st.text == "solve") {
            return Err(format!(
                "CGS line {}: constrain block must be closed with `solve;`",
                st.line
            ));
        }
        self.expect(TokenKind::Semi)?;

        // split the body into `lhs == rhs;` equations
        let mut segs: Vec<Vec<CgsToken>> = Vec::new();
        let mut cur: Vec<CgsToken> = Vec::new();
        for t in body {
            if t.kind == TokenKind::Semi {
                if !cur.is_empty() {
                    segs.push(std::mem::take(&mut cur));
                }
            } else {
                cur.push(t);
            }
        }
        if !cur.is_empty() {
            segs.push(cur);
        }
        if segs.is_empty() {
            return Err(format!("CGS line {line}: constrain block is empty"));
        }
        let mut eqs: Vec<(Vec<CgsToken>, Vec<CgsToken>, i32)> = Vec::new();
        for seg in segs {
            let eline = seg[0].line;
            let mut depth = 0i32;
            let mut eq_at: Option<usize> = None;
            let mut ineq_at: Option<usize> = None;
            for (i, t) in seg.iter().enumerate() {
                if matches!(t.kind, TokenKind::Lparen | TokenKind::Lbracket) {
                    depth += 1;
                } else if matches!(t.kind, TokenKind::Rparen | TokenKind::Rbracket) {
                    depth -= 1;
                } else if depth == 0 && t.kind == TokenKind::Op {
                    if t.text == "==" {
                        if eq_at.is_some() {
                            return Err(format!(
                                "CGS line {}: one == per constrain equation",
                                t.line
                            ));
                        }
                        eq_at = Some(i);
                    } else if matches!(t.text.as_str(), "<" | ">" | "<=" | ">=" | "!=") {
                        ineq_at = Some(i);
                    }
                }
            }
            if let Some(i) = eq_at {
                eqs.push((seg[..i].to_vec(), seg[i + 1..].to_vec(), eline));
            } else if ineq_at.is_some() {
                return Err(format!(
                    "CGS line {eline}: constrain supports == equations only (inequality solving is planned)"
                ));
            } else {
                return Err(format!(
                    "CGS line {eline}: constrain equations must be `lhs == rhs;`"
                ));
            }
        }

        // validate unknowns against the current scope
        let mut x: Vec<f64> = Vec::new();
        for u in &unknowns {
            match scope.get(u) {
                Some(CgsValue::Num(n)) => x.push(*n),
                Some(_) => {
                    return Err(format!(
                        "CGS line {line}: constrain unknown {u} must be a number"
                    ));
                }
                None => {
                    return Err(format!(
                        "CGS line {line}: constrain unknown {u} is not defined"
                    ));
                }
            }
        }

        let base = scope.clone();
        let solved = self.constrain_solve(line, &unknowns, x, &eqs, &base)?;
        for (u, v) in unknowns.iter().zip(solved) {
            scope.insert(u.clone(), CgsValue::Num(v));
        }
        Ok(())
    }

    /// Evaluate one constraint-side token slice as an expression against an
    /// overlay scope (parser position/tokens saved and restored).
    fn eval_token_expr(
        &mut self,
        toks: &[CgsToken],
        scope: &HashMap<String, CgsValue>,
    ) -> Result<CgsValue, String> {
        let saved_toks = std::mem::replace(&mut self.toks, toks.to_vec());
        let saved_pos = self.pos;
        self.pos = 0;
        let res = self.expr(scope, 1);
        let consumed = self.pos >= self.toks.len();
        self.toks = saved_toks;
        self.pos = saved_pos;
        let v = res?;
        if !consumed {
            return Err(format!(
                "CGS line {}: trailing tokens in constraint expression",
                toks[0].line
            ));
        }
        Ok(v)
    }

    fn constrain_residual(
        &mut self,
        eqs: &[(Vec<CgsToken>, Vec<CgsToken>, i32)],
        base_scope: &HashMap<String, CgsValue>,
        unknowns: &[String],
        x: &[f64],
    ) -> Result<Vec<f64>, String> {
        let mut scope = base_scope.clone();
        for (u, v) in unknowns.iter().zip(x.iter()) {
            scope.insert(u.clone(), CgsValue::Num(*v));
        }
        let mut out: Vec<f64> = Vec::new();
        for (lhs, rhs, line) in eqs {
            let a = self.eval_token_expr(lhs, &scope)?;
            let b = self.eval_token_expr(rhs, &scope)?;
            let fa = flatten_val(&a, *line)?;
            let fb = flatten_val(&b, *line)?;
            if fa.len() != fb.len() {
                return Err(format!(
                    "CGS line {line}: constraint sides must have matching shapes ({} vs {})",
                    fa.len(),
                    fb.len()
                ));
            }
            for (u, v) in fa.iter().zip(fb.iter()) {
                out.push(u - v);
            }
        }
        Ok(out)
    }

    /// Levenberg-damped Gauss-Newton over the constraint residuals.
    fn constrain_solve(
        &mut self,
        line: i32,
        unknowns: &[String],
        x0: Vec<f64>,
        eqs: &[(Vec<CgsToken>, Vec<CgsToken>, i32)],
        base_scope: &HashMap<String, CgsValue>,
    ) -> Result<Vec<f64>, String> {
        let n = unknowns.len();
        let mut x = x0;
        let mut lambda = 1e-3f64;
        let mut r = self.constrain_residual(eqs, base_scope, unknowns, &x)?;
        let mut converged = maxabs(&r) < 1e-10;
        let mut iter = 0;
        let mut stalled = false;
        while !converged && iter < 50 && !stalled {
            iter += 1;
            let m = r.len();
            // forward-difference Jacobian
            let mut jac = vec![vec![0.0f64; n]; m];
            for k in 0..n {
                let h = 1e-6 * x[k].abs().max(1.0);
                let mut xp = x.clone();
                xp[k] += h;
                let rp = self.constrain_residual(eqs, base_scope, unknowns, &xp)?;
                if rp.len() != m {
                    return Err(format!(
                        "CGS line {line}: constraint shape changed during solve"
                    ));
                }
                for i in 0..m {
                    jac[i][k] = (rp[i] - r[i]) / h;
                }
            }
            // damped normal-equation step until |r|∞ decreases
            let mut stepped = false;
            while !stepped {
                let mut a = vec![vec![0.0f64; n]; n];
                let mut b = vec![0.0f64; n];
                for p in 0..n {
                    for q in 0..n {
                        let mut s = 0.0;
                        for i in 0..m {
                            s += jac[i][p] * jac[i][q];
                        }
                        a[p][q] = s;
                    }
                    a[p][p] += lambda;
                    let mut s = 0.0;
                    for i in 0..m {
                        s += jac[i][p] * r[i];
                    }
                    b[p] = -s;
                }
                match gauss_solve(a, b) {
                    Some(dx) => {
                        let mut xn = x.clone();
                        for k in 0..n {
                            xn[k] += dx[k];
                        }
                        let rn = self.constrain_residual(eqs, base_scope, unknowns, &xn)?;
                        if maxabs(&rn) < maxabs(&r) {
                            x = xn;
                            r = rn;
                            lambda = (lambda / 10.0).max(1e-12);
                            converged = maxabs(&r) < 1e-10;
                            stepped = true;
                        } else {
                            lambda *= 10.0;
                            if lambda > 1e14 {
                                stalled = true;
                                stepped = true;
                            }
                        }
                    }
                    None => {
                        lambda *= 10.0;
                        if lambda > 1e14 {
                            stalled = true;
                            stepped = true;
                        }
                    }
                }
            }
        }
        if !converged {
            return Err(format!(
                "CGS line {line}: constrain did not converge (|r|={:.3e}, iter={iter}, var={})",
                maxabs(&r),
                unknowns.join(",")
            ));
        }
        Ok(x)
    }

    /// P4 statement-position chain lookahead (`docs/cgs-v3.md` §3.4):
    /// `ident.…` or `ident(…).…`. The head call's parens are balanced across
    /// the token stream, so keyword statements — `for (…)`, `echo(…);`,
    /// `difference(a, b);`, `translate(…) box(…);`, `module m(…)` — never
    /// match: only a head directly followed by `.`, or by a call whose
    /// closing paren is, does. Assignment and index checks run first, so
    /// `g = …;` and `g[…];` keep their existing forms.
    fn peek_chain(&self) -> bool {
        if self.toks.get(self.pos).map(|t| t.kind) != Some(TokenKind::Ident) {
            return false;
        }
        let mut i = self.pos + 1;
        match self.toks.get(i).map(|t| t.kind) {
            Some(TokenKind::Dot) => return true,
            Some(TokenKind::Lparen) => {}
            _ => return false,
        }
        let mut depth = 0i32;
        while let Some(t) = self.toks.get(i) {
            match t.kind {
                TokenKind::Lparen => depth += 1,
                TokenKind::Rparen => {
                    depth -= 1;
                    if depth == 0 {
                        return self.toks.get(i + 1).map(|t| t.kind) == Some(TokenKind::Dot);
                    }
                }
                TokenKind::Eof => return false,
                _ => {}
            }
            i += 1;
        }
        false
    }

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
            // Assignment is its own statement form, checked before the
            // keyword dispatch so any keyword can be shadowed by a variable
            // (`echo = 5;`, `for = …`). `background = 0x…;` doubles as the
            // property form of the background statement.
            if self.peek1().kind == TokenKind::Assign {
                self.take();
                self.take();
                let v = self.expr(scope, 1)?;
                self.expect(TokenKind::Semi)?;
                if name == "background" {
                    let c = cgs_num(&v, t.line, "background")?;
                    self.scene.background = color_hex(c as i32);
                }
                scope.insert(name.to_string(), v);
                return Ok(());
            }
            if self.peek1().kind == TokenKind::Lbracket {
                return Err(format!(
                    "CGS line {}: indexing is not supported — use comp(vector, index)",
                    t.line
                ));
            }
            // P4 statement-position chain (`docs/cgs-v3.md` §3.4): a head
            // ident followed by `.`, or by its own balanced call ending in
            // `.`, makes the statement an expression statement — evaluated
            // with `expr`, then rendered through the exact path the CSG
            // expression form uses below (`geom_val` → `;` → `add_geometry`).
            if self.peek_chain() {
                let v = self.expr(scope, 1)?;
                let g = geom_val(&v, t.line, "expression statement")?;
                self.expect(TokenKind::Semi)?;
                return self.add_geometry(g.geo, mat4_mul(ctx, g.m4), mat);
            }
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
            if name == "show" {
                self.show_stmt(ctx, mat, scope, t.line)?;
                return Ok(());
            }
            if name == "tag" {
                self.tag_stmt(ctx, mat, scope, t.line)?;
                return Ok(());
            }
            if name == "drill" {
                self.drill_stmt(ctx, mat, scope, t.line)?;
                return Ok(());
            }
            if name == "var" {
                self.var_stmt(scope, t.line)?;
                return Ok(());
            }
            if name == "constrain" {
                self.constrain_stmt(scope, t.line)?;
                return Ok(());
            }
            if name == "union" || name == "difference" || name == "intersection" {
                self.take();
                self.expect(TokenKind::Lparen)?;
                if self.peek().kind == TokenKind::Rparen {
                    // block form: difference() { ... }
                    self.take();
                    if name == "union" {
                        self.body(ctx, mat, scope, t.line)?;
                        return Ok(());
                    }
                    let op = if name == "difference" {
                        CsgOp::Difference
                    } else {
                        CsgOp::Intersection
                    };
                    self.csg_block(op, ctx, mat, scope, t.line)?;
                    return Ok(());
                }
                // expression form: difference(a, b); — build a value, render it
                let (pos, kw) = self.paren_args(scope)?;
                let v = self
                    .geom_expr_call(name, pos, kw, t.line)?
                    .ok_or_else(|| format!("CGS line {}: {name} is not callable here", t.line))?;
                let g = geom_val(&v, t.line, name)?;
                self.expect(TokenKind::Semi)?;
                return self.add_geometry(g.geo, mat4_mul(ctx, g.m4), mat);
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
        if is_expr_only_fn(&name) {
            return Err(format!(
                "CGS line {}: {} is an expression function and cannot be used as a statement",
                t.line, name
            ));
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
        self.take();
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
        self.take();
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
        self.take();
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
        self.take();
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

        let body = match self.modules.get(name) {
            Some(b) => b.clone(),
            None => Vec::new(),
        };
        let mut scope: HashMap<String, CgsValue> = HashMap::new();
        scope.insert("pi".to_string(), CgsValue::Num(std::f64::consts::PI));

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

    fn call_args(
        &mut self,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<(Vec<CgsValue>, HashMap<String, CgsValue>), String> {
        self.expect(TokenKind::Lparen)?;
        self.paren_args(scope)
    }

    /// Parse `(pos, kw)` args; the opening `(` must already be consumed.
    fn paren_args(
        &mut self,
        scope: &HashMap<String, CgsValue>,
    ) -> Result<(Vec<CgsValue>, HashMap<String, CgsValue>), String> {
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
        self.register(&geo, ctx, ctx);
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
        // The result absorbs every child frame: its geometry is world-space
        // (the emit context is baked in), so the instance slot is identity.
        let result = Geometry::CsgGeometry(csg_geometry(op, kids));
        self.register(&result, mat4_identity(), ctx);
        if was_collecting {
            // Nested CSG: flow the result back into the enclosing collect so
            // the parent difference/intersection sees it as a child.
            self.collect.push(CollectedGeom {
                geo: result,
                m4: mat4_identity(),
            });
            return Ok(());
        }
        self.scene.add_mesh(mesh(MeshParams {
            geometry: result,
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
    use crate::scene_report::scene_report;

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

    // ── P4 后缀方法链（docs/cgs-v3.md §3）───────────────────────────────────

    /// 等价金样：语句位链式与去糖函数式（`show` 包裹）渲染同一场景，
    /// 确定性报告逐位相同（链只是糖，不产生新语义）。
    #[test]
    fn test_cgs_p4_chain_equivalence() {
        let (sa, ca) = cgs_load("box(s=[2,2,2]).at([1,0,0]).rot([0,1,0], 45);", "");
        let (sb, cb) = cgs_load("show(rot(at(box(s=[2,2,2]), [1,0,0]), [0,1,0], 45));", "");
        assert_eq!(sa.objects.len(), 1);
        assert_eq!(sb.objects.len(), 1);
        assert_eq!(
            scene_report(&sa, &ca, &TagRegistry::new()),
            scene_report(&sb, &cb, &TagRegistry::new())
        );
    }

    /// 错误继承组：链式拼法逐字复用既有错误文本——边界、未知函数、
    /// 变元数、方法名、左括号，全是既有格式串（§3.2 错误文本几乎零新增）。
    #[test]
    fn test_cgs_p4_chain_errors_inherited() {
        let e = v2_err("g = box(s=[1,1,1]);\ng.show();");
        assert!(
            e.contains("show is a statement and cannot be used in an expression"),
            "{e}"
        );
        // 链式与函数式的报错逐字相同（同一分发、同一行号）。
        let e_chain = v2_err("g = box(s=[1,1,1]);\ng.frobnicate();");
        let e_plain = v2_err("g = box(s=[1,1,1]);\nx = frobnicate(g);");
        assert_eq!(e_chain, e_plain);
        assert!(
            e_chain.contains("frobnicate needs a number, got <geom>"),
            "{e_chain}"
        );
        // Number 目标越过 1 元类型分派后，落回既有 unknown-function 文案。
        let e = v2_err("x = (1).frobnicate();");
        assert!(e.contains("unknown function frobnicate"), "{e}");
        let e = v2_err("g = box(s=[1,1,1]);\ng.at();");
        assert!(e.contains("at needs (geometry, offset)"), "{e}");
        let e = v2_err("g = box(s=[1,1,1]);\ng.5;");
        assert!(e.contains("expected ident, got number"), "{e}");
        let e = v2_err("g = box(s=[1,1,1]);\ng.center;");
        assert!(e.contains("expected lparen, got semi"), "{e}");
    }

    /// 语句位链直接渲染；链先按表达式求值，查询结果（Vec3）再撞语句位
    /// geom_val 的规范文案（§3.4）。
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

    /// 变量头链：两条链语句与等价的 `show(f(…))` 两条语句渲染同一场景。
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
            scene_report(&sa, &ca, &TagRegistry::new()),
            scene_report(&sb, &cb, &TagRegistry::new())
        );
    }

    /// Str 头链：`"plate".face("+z")` 去糖为 `face("plate", "+z")`，
    /// 同一场景（引用查询接受 Str 目标，链不分新语义）。
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
            scene_report(&sa, &ca, &TagRegistry::new()),
            scene_report(&sb, &cb, &TagRegistry::new())
        );
    }

    /// 词法：数字内的点不断词（`1.5` 完整、链在 `.abs()` 上接续）；
    /// 数字紧贴标识符并置成两个 token；表达式以点开头是确定的非法起始。
    #[test]
    fn test_cgs_p4_lex_dot() {
        let (sc, _) = cgs_load("x = 1.5.abs();", "");
        assert_eq!(sc.objects.len(), 0);
        let e = v2_err("x = 1.abs();");
        assert!(e.contains("expected semi, got ident"), "{e}");
        let e = v2_err("x = .5;");
        assert!(e.contains("bad expression start dot"), "{e}");
    }

    // ── P5 集合选择 instances()（docs/cgs-v3.md §4）─────────────────────────

    /// 顺序金样：注册序 = 发射序 = `for` 遍历序；元素倒序会立刻被报告
    /// 差异抓住（对照侧把三次 show 按 x=0,1,2 显式写出，tag 段两边相同）。
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
            scene_report(&sa, &ca, &TagRegistry::new()),
            scene_report(&sb, &cb, &TagRegistry::new())
        );
    }

    /// 计数与条件：`len(instances(...))` + `if` = SQL 的 COUNT/WHERE 直觉。
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

    /// 错误组：未知名复用 `unknown reference`；非 Str 实参是 P5 唯一新增
    /// 文本；语句位命中 expression-only 契约；缺参走既有 arity 检查。
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

    /// 聚合与元素互通：并集 `center("hole")` 与 `for` 逐元素取中心实例
    /// 几何一致（对称布局 = 中间实例，两份报告逐位相同）；集合元素直接作
    /// `drill through=` 实参（difference 块内逐元素贯穿切割）。
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
            scene_report(&sa, &ca, &TagRegistry::new()),
            scene_report(&sb, &cb, &TagRegistry::new())
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

    /// §4.4 可选子项：`face("plate:+z")` 单串面字面量 ≡ `face("plate", "+z")`
    /// （零新增错误文本；不含 `:` 落回既有 arity 检查）。
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
            scene_report(&sa, &ca, &TagRegistry::new()),
            scene_report(&sb, &cb, &TagRegistry::new())
        );
        let e = v2_err("tag(\"plate\") box(s=[1,1,2]);\nv = face(\"plate\");");
        assert!(e.contains("face needs 2 argument(s)"), "{e}");
    }

    #[test]
    fn test_cgs_geom_value_bind_show() {
        // A binding alone renders nothing; show renders it. (docs/cgs-v2.md P0)
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
        // Bindings do not capture the statement context: translate applies
        // once at show time, not squared (docs/cgs-v2.md §4.2 rule 1/2).
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
        // at() offset stays in the enclosing frame even under translation
        let (sc, _) = cgs_load(
            "g = sphere(r=1);\ntranslate([10, 0, 0]) show(at(g, [1, 0, 0]));",
            "",
        );
        assert!((sc.objects[0].position[0] - 11.0).abs() < 1e-9);
        // scaled() folds into an affine instance
        let (sc, _) = cgs_load("g = sphere(r=1);\nshow(scaled(g, 2));", "");
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(
            sc.objects[0].geometry,
            Geometry::AffineGeometry(_)
        ));
        // rot() composes a rotation frame
        let (sc, _) = cgs_load(
            "g = box(s=[1, 1, 1]);\nshow(rot(g, [0, 0, 1], pi / 2));",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
    }

    #[test]
    fn test_cgs_expr_csg_forms() {
        // expression form renders (docs/cgs-v2.md §4.3)
        let (sc, _) = cgs_load("difference(sphere(r=1), box(s=[1, 1, 1]));", "");
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(sc.objects[0].geometry, Geometry::CsgGeometry(_)));
        // bound and shown later
        let (sc, _) = cgs_load(
            "x = difference(sphere(r=1), box(s=[1, 1, 1]));\nshow(x);",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(sc.objects[0].geometry, Geometry::CsgGeometry(_)));
        // statement nesting — previously "difference needs >= 2 geometry children"
        let (sc, _) = cgs_load(
            "difference() { box(s=[2, 2, 2]); difference() { sphere(r=3); box(s=[4, 4, 4]); } }",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        assert!(matches!(sc.objects[0].geometry, Geometry::CsgGeometry(_)));
        // three levels
        let (sc, _) = cgs_load(
            "intersection() { difference() { sphere(r=2); box(s=[1, 1, 1]); } difference() { box(s=[3, 3, 3]); sphere(r=0.5); } }",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
        // nested inside a collecting parent, expression form too
        let (sc, _) = cgs_load(
            "difference() { box(s=[6, 6, 6]); difference(sphere(r=4), box(s=[8, 8, 8])); }",
            "",
        );
        assert_eq!(sc.objects.len(), 1);
    }

    #[test]
    fn test_cgs_tag_reference_queries() {
        // golden pattern: a query result drives a placement
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

        // tag around a block registers a group; union bbox center = origin
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
        // show() inside a difference collect path
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
            motor_rotor([0.0, 0.0, 1.0], 0.7).to_matrix(),
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
        // The drill spans exactly the plate's thickness (G3: derived extent
        // follows the target — change the plate, the hole follows).
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

        // thicken the plate → the cutter follows automatically
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

        // displaced target: extent center follows the instance's rel frame
        let (sc, _) = cgs_load(
            "p = box(s=[4, 0.3, 4]);\ntag(\"plate\") translate([0, 2, 0]) show(p);\ndrill(r=0.2, through=\"plate\", axis=1);",
            "",
        );
        assert!(
            (sc.objects[1].position[1] - 2.0).abs() < 1e-9,
            "pos = {:?}",
            sc.objects[1].position
        );

        // through = geometry value (frameless binding)
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
        // from/to override the axial extent
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

        // drill as a difference child → exactly one CSG mesh
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
        // Solve in place: the instance rendered BEFORE solve keeps the
        // initial value; statements AFTER solve read the baked solution.
        // (Equations must reference the unknown directly: bound values and
        // tagged instances are baked at bind/render time.)
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
        // b snapshot at d=1 → |d+4| = 5 ≠ 4, renders at x=4
        assert!(
            (sc.objects[1].position[0] - 4.0).abs() < 1e-6,
            "{:?}",
            sc.objects[1].position
        );
        // statement after solve: d = 0 → x = 3
        assert!(
            (sc.objects[2].position[0] - 3.0).abs() < 1e-6,
            "{:?}",
            sc.objects[2].position
        );
    }

    #[test]
    fn test_cgs_constrain_vector_equation() {
        // Vec3/list residual: three components constrain one unknown.
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
        // nonlinear: Newton from 3.5 converges to +3
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
        // 2 unknowns, 1 equation: any solution on p + q = 5 is accepted.
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
        // `var` re-declares each iteration, so the solve restarts from 0.
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
        let e = v2_err("var x = 0.0;\nconstrain(x) { x < 1; } solve;");
        assert!(e.contains("inequality solving is planned"), "{e}");
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
    fn test_cgs_statement_expression_boundary() {
        // background: property assignment and statement call are equivalent.
        let (sc, _) = cgs_load("background = 0x2B3138;", "");
        assert!((sc.background.r - 0x2Bu32 as f64 / 255.0).abs() < 1e-12);
        assert!((sc.background.g - 0x31u32 as f64 / 255.0).abs() < 1e-12);
        assert!((sc.background.b - 0x38u32 as f64 / 255.0).abs() < 1e-12);
        let (sc, _) = cgs_load("background(color=0x87CEEB);", "");
        assert!((sc.background.r - 0x87u32 as f64 / 255.0).abs() < 1e-12);

        // assignment dispatches first → keywords are shadowable as variables
        let (sc, _) = cgs_load("echo = 5;\nfor = 6;\necho(echo + for);", "");
        assert_eq!(sc.objects.len(), 0);

        // nested assignment in expression position
        let e = v2_err("x = 1; y = x = 2;");
        assert!(
            e.contains("assignment is a statement and cannot be used in an expression"),
            "{e}"
        );

        // statement keywords / modifiers / property statements in expressions
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

        // expression functions used as statements
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

        // indexing rejected in both positions
        for src in ["a = [1, 2, 3]; b = a[0];", "a = [1, 2, 3]; a[0];"] {
            let e = v2_err(src);
            assert!(e.contains("indexing is not supported"), "[{src}] -> {e}");
        }

        // unchanged fallbacks and still-working forms
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
        // box: face center and outward normal through a rotate+translate
        // instance (world frame; normal via inverse-transpose).
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

        // cylinder: caps exact, side tangent point at the axial midpoint
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

        // sphere / ellipsoid: exact axis intersections
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

        // cone (apex at +h/2): tip, base center, side point on the surface,
        // side normal ⟂ generator tilted toward the apex
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

        // AABB fallback (torus: bounds = major + minor)
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
        // same target end faces: axis inferred from the keys, bbox replaced
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

        // through omitted: F = ctx, the two faces fully specify the extent
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

        // end-to-end BETWEEN two instances: base top face → top bottom face
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

        // axis kw accepted when it matches the face keys
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
}
