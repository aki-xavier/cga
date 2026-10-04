//! Scene Report: deterministic, line-assertable text rendering of an executed
//! CGS scene (`docs/scene-report.md` §3–§5).
//!
//! Total function: no validation, no failure branches — it contributes **zero**
//! new diagnostic texts to the language error contract (§9). Report skeleton
//! and line order are fixed by §4: `scene` → `background` → `camera` → lights →
//! (`object` + `bounds`) pairs → tag section (names in lexicographic order,
//! instances in emission order) → `summary`.
//!
//! Numbers follow §5.1 (`{:.6}`, trailing zeros stripped, any zero → `0`,
//! non-finite → `nan`/`inf`/`-inf`, never scientific notation). Transforms go
//! through `to_matrix → matrix_to_quaternion → 2·atan2(‖xyz‖, w)` with the
//! angle canonicalized to `[0, π]` (§5.2); rigid statement slots print as a
//! `translate(…) rotate(…)` prefix, expression slots as a `frame(t=…, axis=…,
//! angle=…, lin=…, inner)` token whose identity keys are omitted.

use cga_core::{bounds_of, decompose_rigid, matrix_to_quaternion, Geometry};
use std::f64::consts::PI;

use crate::geom_kernels::geom_to_camera;
use crate::scene::{Mesh, PerspectiveCamera, Scene};
use crate::scene_graph::{vec3_unit, Color};
use crate::scene_lang::{cgs_run_result, csg_op_name, TagInstance, TagRegistry};
use crate::shading::{Light, LightKind, Material, MaterialKind};

/// §5.1 six-place canonicalization: `{:.6}` → strip trailing zeros → strip a
/// trailing `.` → any value that parses back to 0 (incl. `-0.000000`) prints
/// `0`. Non-finite values print `nan`/`inf`/`-inf` (diagnostic only, §7).
fn fmt_num(v: f64) -> String {
    if v.is_nan() {
        return "nan".to_string();
    }
    if v.is_infinite() {
        return if v > 0.0 { "inf" } else { "-inf" }.to_string();
    }
    let s = format!("{v:.6}");
    let t = s.trim_end_matches('0');
    let t = t.strip_suffix('.').unwrap_or(t);
    if t.is_empty() || t == "-" {
        return "0".to_string();
    }
    match t.parse::<f64>() {
        Ok(x) if x == 0.0 => "0".to_string(),
        _ => t.to_string(),
    }
}

/// Vectors print without spaces: `[1.2,0,0]` (§4).
fn fmt_vec3(v: [f64; 3]) -> String {
    format!("[{},{},{}]", fmt_num(v[0]), fmt_num(v[1]), fmt_num(v[2]))
}

/// Row-major 3×3 as nested lists: `[[2,0,0],[0,1,0],[0,0,1]]` (§4).
fn fmt_mat3(m: [[f64; 3]; 3]) -> String {
    format!("[{},{},{}]", fmt_vec3(m[0]), fmt_vec3(m[1]), fmt_vec3(m[2]))
}

fn is_identity3_fmt(m: [[f64; 3]; 3]) -> bool {
    fmt_mat3(m) == "[[1,0,0],[0,1,0],[0,0,1]]"
}

/// Colors print as `0xRRGGBB` uppercase, recovered from the stored sRGB value
/// by rounding 8 bits per channel (§4 — the inverse of `color_hex`).
fn fmt_color(c: &Color) -> String {
    let ch = |x: f64| (x * 255.0).round().clamp(0.0, 255.0) as i64;
    format!("0x{:06X}", (ch(c.r) << 16) | (ch(c.g) << 8) | ch(c.b))
}

/// §5.2 extraction: translation from `[m[3],m[7],m[11]]`, rotation through
/// `matrix_to_quaternion`, `angle = 2·atan2(‖xyz‖, w)` flipped to `[0, π]`
/// (axis negated when `angle > π`). A degenerate (near-)identity rotation
/// returns angle 0 with a placeholder unit axis.
fn frame_of(m4: &[f64; 16]) -> ([f64; 3], [f64; 3], f64) {
    let t = [m4[3], m4[7], m4[11]];
    let r = [
        [m4[0], m4[1], m4[2]],
        [m4[4], m4[5], m4[6]],
        [m4[8], m4[9], m4[10]],
    ];
    let q = matrix_to_quaternion(r);
    let n = (q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    if n < 1e-15 {
        return (t, [0.0, 0.0, 1.0], 0.0);
    }
    let mut angle = 2.0 * n.atan2(q.w);
    let mut axis = [q.x / n, q.y / n, q.z / n];
    if angle > PI {
        angle = 2.0 * PI - angle;
        axis = [-axis[0], -axis[1], -axis[2]];
    }
    (t, axis, angle)
}

/// Statement slot (object / tag-instance prefix): the CGS chain shape
/// `translate([…]) rotate(axis=[…], angle=…)` , translate always first (§5.2).
/// Frame keys are structurally omitted when they canonicalize to identity.
fn fmt_prefix(t: [f64; 3], axis: [f64; 3], angle: f64) -> String {
    let mut s = String::new();
    if fmt_vec3(t) != "[0,0,0]" {
        s.push_str(&format!("translate({}) ", fmt_vec3(t)));
    }
    if fmt_num(angle) != "0" {
        s.push_str(&format!(
            "rotate(axis={}, angle={}) ",
            fmt_vec3(axis),
            fmt_num(angle)
        ));
    }
    s
}

/// Expression slot (CSG children, `AffineGeometry`, `mesh.linear ≠ I`):
/// `frame(t=…, axis=…, angle=…, lin=…, inner)` with key order t, axis, angle,
/// lin; identity keys dropped; all keys empty ⇒ the bare inner (§5.2).
#[allow(clippy::too_many_arguments)]
fn fmt_frame(
    t: [f64; 3],
    axis: [f64; 3],
    angle: f64,
    lin: Option<[[f64; 3]; 3]>,
    inner: &str,
) -> String {
    let mut keys: Vec<String> = Vec::new();
    if fmt_vec3(t) != "[0,0,0]" {
        keys.push(format!("t={}", fmt_vec3(t)));
    }
    if fmt_num(angle) != "0" {
        keys.push(format!("axis={}", fmt_vec3(axis)));
        keys.push(format!("angle={}", fmt_num(angle)));
    }
    if let Some(l) = lin {
        if !is_identity3_fmt(l) {
            keys.push(format!("lin={}", fmt_mat3(l)));
        }
    }
    if keys.is_empty() {
        inner.to_string()
    } else {
        format!("frame({}, {})", keys.join(", "), inner)
    }
}

/// §5.4: the 12 stored variants → CGS-shaped tokens, parameter names from
/// `cgs_sig_names`. `s = 2·half`, `h = -1` unbounded cylinder; semantic
/// parameters (torus `arc`, cyclide `shift`) always print.
fn fmt_geom(g: &Geometry) -> String {
    match g {
        Geometry::SphereGeometry(s) => format!("sphere(r={})", fmt_num(s.radius)),
        Geometry::PlaneGeometry(p) => {
            let n = vec3_unit(p.blade.euclidean_vector());
            format!(
                "plane(n={}, d={})",
                fmt_vec3(n),
                fmt_num(p.blade.einf_coeff())
            )
        }
        Geometry::CylinderGeometry(c) => {
            let h = if c.half < 0.0 { -1.0 } else { 2.0 * c.half };
            format!("cylinder(r={}, h={})", fmt_num(c.radius), fmt_num(h))
        }
        Geometry::BoxGeometry(b) => format!(
            "box(s={})",
            fmt_vec3([b.half[0] * 2.0, b.half[1] * 2.0, b.half[2] * 2.0])
        ),
        Geometry::CircleGeometry(c) => format!("circle(r={})", fmt_num(c.radius)),
        Geometry::ConeGeometry(c) => {
            format!("cone(r={}, h={})", fmt_num(c.radius), fmt_num(c.height))
        }
        Geometry::TorusGeometry(t) => format!(
            "torus(R={}, r={}, arc={})",
            fmt_num(t.major),
            fmt_num(t.minor),
            fmt_num(t.arc)
        ),
        Geometry::EllipsoidGeometry(e) => format!("ellipsoid(radii={})", fmt_vec3(e.radii)),
        Geometry::CyclideGeometry(c) => format!(
            "cyclide(a={}, b={}, d={}, shift={})",
            fmt_num(c.a),
            fmt_num(c.b),
            fmt_num(c.d),
            fmt_vec3(c.shift)
        ),
        Geometry::TrimeshGeometry(t) => format!(
            "trimesh(faces={}, lo={}, hi={})",
            t.n_faces,
            fmt_vec3(t.lo),
            fmt_vec3(t.hi)
        ),
        Geometry::CsgGeometry(c) => {
            let kids: Vec<String> = c.children.iter().map(fmt_geom).collect();
            format!("{}({})", csg_op_name(c.op), kids.join(", "))
        }
        Geometry::AffineGeometry(a) => {
            // The renderer reads exactly `inner[0]`; an empty inner cannot be
            // produced by the constructors, but the report stays total.
            let inner = a.inner.first().map(fmt_geom).unwrap_or_default();
            let (t, axis, angle) = frame_of(&a.motor.to_matrix());
            fmt_frame(t, axis, angle, Some(a.linear), &inner)
        }
    }
}

/// §5.5: every field printed (defaults are values, not omissions); optional
/// `, map=WxH` for a texture and `, unlit=true` for `MaterialKind::Basic`.
fn fmt_material(m: &Material) -> String {
    let mut s = format!(
        "material(color={}, roughness={}, metalness={}, emissive={}, opacity={}, ior={}, absorption={}",
        fmt_color(&m.color),
        fmt_num(m.roughness),
        fmt_num(m.metalness),
        fmt_color(&m.emissive),
        fmt_num(m.opacity),
        fmt_num(m.ior),
        fmt_num(m.absorption)
    );
    if let Some(tex) = &m.map {
        s.push_str(&format!(", map={}x{}", tex.width, tex.height));
    }
    if let MaterialKind::Basic = m.kind {
        s.push_str(", unlit=true");
    }
    s.push(')');
    s
}

/// §5.3: keyword order is the CGS call's order; `kind` is carried by the
/// function name, not printed as its own key.
fn fmt_light(l: &Light) -> String {
    match l.kind {
        LightKind::Ambient => format!(
            "ambient_light(color={}, intensity={});",
            fmt_color(&l.color),
            fmt_num(l.intensity)
        ),
        LightKind::Directional => format!(
            "directional_light(direction={}, color={}, intensity={});",
            fmt_vec3(l.direction),
            fmt_color(&l.color),
            fmt_num(l.intensity)
        ),
        LightKind::Point => format!(
            "point_light(position={}, color={}, intensity={});",
            fmt_vec3(l.position),
            fmt_color(&l.color),
            fmt_num(l.intensity)
        ),
    }
}

/// §5.3: all seven keys; the CGS `camera` statement reads four of them and
/// fixes near/far/up (§7 回读差距 — the report prints the stored truth).
fn fmt_camera(c: &PerspectiveCamera) -> String {
    format!(
        "camera(fov={}, aspect={}, position={}, target={}, up={}, near={}, far={});",
        fmt_num(c.fov),
        fmt_num(c.aspect),
        fmt_vec3(c.position),
        fmt_vec3(c.target),
        fmt_vec3(c.up),
        fmt_num(c.near),
        fmt_num(c.far)
    )
}

/// `object <i> <rigid prefix> material(…) <geometry>;` — the prefix comes
/// from `mesh.motor()`, a `frame(lin=…)` wrap appears in the geometry slot
/// only for hand-built scenes with `mesh.linear ≠ I` (§5.2).
fn fmt_object(i: usize, m: &Mesh) -> String {
    let (t, axis, angle) = frame_of(&m.motor().to_matrix());
    let prefix = fmt_prefix(t, axis, angle);
    let geom = fmt_frame(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.0,
        Some(m.linear),
        &fmt_geom(&m.geometry),
    );
    format!("object {i} {prefix}{} {geom};", fmt_material(&m.material))
}

/// Tag names quote-escaped for the report's string slots. The CGS lexer does
/// not decode escapes (`scene_lang.rs` string branch), so ordinary names pass
/// through byte-identical; only `"`/`\` get the standard backslash form (§5.6).
fn escape_name(name: &str) -> String {
    name.replace('\\', "\\\\").replace('"', "\\\"")
}

/// §5.6: `tag "<名>" <j> <frame prefix> <geometry>;` — frame = `Inst.world`
/// decomposed as `motor ∘ linear` (a scaled context shows up as `lin=…`);
/// no material (`rel` never printed). A CSG instance's world is identity, so
/// its line carries no prefix and the geometry is world-space.
fn fmt_instance(name: &str, j: usize, inst: &TagInstance) -> String {
    let (motor, lin) = decompose_rigid(inst.world);
    let (t, axis, angle) = frame_of(&motor.to_matrix());
    let prefix = fmt_prefix(t, axis, angle);
    let geom = fmt_frame(
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        0.0,
        Some(lin),
        &fmt_geom(&inst.geo),
    );
    format!("tag \"{}\" {j} {prefix}{geom};", escape_name(name))
}

/// Format a loaded scene as the Scene Report (§3–§5). Deterministic: two runs
/// over equal scenes are byte-identical — tag names iterate a `BTreeMap`.
/// `tags` comes from `cgs_run_result`; hand-built scenes pass
/// `&TagRegistry::new()`.
pub fn scene_report(scene: &Scene, camera: &PerspectiveCamera, tags: &TagRegistry) -> String {
    let mut out = String::new();
    out.push_str("scene version=1\n");
    out.push_str(&format!(
        "background(color={});\n",
        fmt_color(&scene.background)
    ));
    out.push_str(&fmt_camera(camera));
    out.push('\n');
    for l in &scene.lights {
        out.push_str(&fmt_light(l));
        out.push('\n');
    }
    // Per-object lines share one pass with the §5.7 summary aggregation.
    let mut no_bounds = 0usize;
    let mut bbox_lo = [f64::INFINITY; 3];
    let mut bbox_hi = [f64::NEG_INFINITY; 3];
    let mut any_bounds = false;
    for (i, m) in scene.objects.iter().enumerate() {
        out.push_str(&fmt_object(i, m));
        out.push('\n');
        let params = geom_to_camera(&m.geometry, &m.motor());
        match bounds_of(&params) {
            Some(b) => {
                out.push_str(&format!(
                    "bounds {i} lo={} hi={}\n",
                    fmt_vec3(b[0]),
                    fmt_vec3(b[1])
                ));
                for k in 0..3 {
                    bbox_lo[k] = bbox_lo[k].min(b[0][k]);
                    bbox_hi[k] = bbox_hi[k].max(b[1][k]);
                }
                any_bounds = true;
            }
            None => {
                no_bounds += 1;
                out.push_str(&format!("bounds {i} none\n"));
            }
        }
    }
    for (name, insts) in tags {
        out.push_str(&format!(
            "tag \"{}\" count={}\n",
            escape_name(name),
            insts.len()
        ));
        for (j, inst) in insts.iter().enumerate() {
            out.push_str(&fmt_instance(name, j, inst));
            out.push('\n');
        }
    }
    let mut summary = format!(
        "summary objects={} lights={} no_bounds={}",
        scene.objects.len(),
        scene.lights.len(),
        no_bounds
    );
    if any_bounds {
        summary.push_str(&format!(
            " bbox_lo={} bbox_hi={}",
            fmt_vec3(bbox_lo),
            fmt_vec3(bbox_hi)
        ));
    }
    out.push_str(&summary);
    out.push('\n');
    out
}

/// Load a CGS script and report it in one step. The error path is
/// `cgs_run_result`'s — identical texts and line numbers to the historical
/// `cgs_load_result` (§9, zero new errors).
pub fn cgs_report(text: &str, asset_root: &str) -> Result<String, String> {
    let run = cgs_run_result(text, asset_root)?;
    Ok(scene_report(&run.scene, &run.camera, &run.tags))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{mesh, perspective_camera, scene, MeshParams};
    use crate::scene_graph::color_hex;
    use crate::shading::MaterialKind;
    use crate::texture::texture_from_rgba;

    /// The default material a material-less CGS statement builds
    /// (`build_material`: white / roughness 0.5 / …).
    const DEF_MAT: &str = "material(color=0xFFFFFF, roughness=0.5, metalness=0, \
emissive=0x000000, opacity=1, ior=1.5, absorption=0)";

    fn cam() -> PerspectiveCamera {
        perspective_camera(
            50.0,
            16.0 / 9.0,
            0.1,
            100.0,
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        )
    }

    fn assert_has(rep: &str, line: &str) {
        assert!(
            rep.contains(line),
            "expected line {line:?} in report:\n{rep}"
        );
    }

    /// T1 — byte-exact whole report: default background, synthesized default
    /// camera, one translated object + bounds pair, summary with bbox.
    #[test]
    fn test_report_minimal_golden() {
        let rep = cgs_report(
            "translate([2,0,0]) material(color=0xFF0000) sphere(r=0.5);",
            "",
        )
        .expect("report");
        let want = [
            "scene version=1",
            "background(color=0x87CEEB);",
            "camera(fov=50, aspect=1.777778, position=[0,0,5], target=[0,0,0], \
up=[0,1,0], near=0.1, far=100);",
            "object 0 translate([2,0,0]) material(color=0xFF0000, roughness=0.5, \
metalness=0, emissive=0x000000, opacity=1, ior=1.5, absorption=0) sphere(r=0.5);",
            "bounds 0 lo=[1.5,-0.5,-0.5] hi=[2.5,0.5,0.5]",
            "summary objects=1 lights=0 no_bounds=0 bbox_lo=[1.5,-0.5,-0.5] \
bbox_hi=[2.5,0.5,0.5]",
            "",
        ]
        .join("\n");
        assert_eq!(rep, want);
    }

    /// T2 — §5.2 rotation canonicalization: `pi/2` keeps the source axis,
    /// `-pi/2` flips it and stays positive, identity / 1e-9 translation /
    /// `2*pi` all drop their prefix, `-0.0` prints `0`.
    #[test]
    fn test_report_rotation_canonical() {
        let rep = cgs_report(
            "rotate(axis=[0,0,1], angle=pi/2) sphere(r=1);\n\
             rotate(axis=[0,0,1], angle=-pi/2) sphere(r=1);\n\
             sphere(r=1);\n\
             translate([1e-9,0,0]) sphere(r=1);\n\
             rotate(axis=[0,0,1], angle=2*pi) sphere(r=1);\n\
             plane(n=[0,0,1], d=-0.0);",
            "",
        )
        .expect("report");
        assert_has(
            &rep,
            "object 0 rotate(axis=[0,0,1], angle=1.570796) material",
        );
        assert_has(
            &rep,
            "object 1 rotate(axis=[0,0,-1], angle=1.570796) material",
        );
        // identity, sub-6dp translation and a full turn all omit the prefix
        assert_has(&rep, "object 2 material");
        assert_has(&rep, "object 3 material");
        assert_has(&rep, "object 4 material");
        // angle canonicalized to [0, π] — never printed negative
        assert!(!rep.contains("angle=-"), "negative angle leaked:\n{rep}");
        // plane(n,d) construction/extraction are inverse; -0.0 prints 0
        assert_has(
            &rep,
            "object 5 material(color=0xFFFFFF, roughness=0.5, \
metalness=0, emissive=0x000000, opacity=1, ior=1.5, absorption=0) \
plane(n=[0,0,1], d=0);",
        );
        assert_has(&rep, "bounds 5 none");
        assert_has(
            &rep,
            "summary objects=6 lights=0 no_bounds=1 bbox_lo=[-1,-1,-1] bbox_hi=[1,1,1]",
        );
    }

    /// T3 — CSG expression slots: no object prefix, `frame(t=…)` for a
    /// translated child, `frame(lin=[[2,0,0],…])` for a scaled one, identity
    /// frames dropped, nested `difference(…)` recursive and flat.
    #[test]
    fn test_report_csg_nested_frames() {
        let rep = cgs_report(
            "difference() {\n  translate([0.5,0,0]) box(s=[2,2,2]);\n  \
             scale([2,1,1]) sphere(r=1);\n}\n\
             difference() {\n  difference() { box(s=[4,4,4]); sphere(r=0.5); }\n  \
             translate([1,0,0]) sphere(r=0.5);\n}",
            "",
        )
        .expect("report");
        assert_has(
            &rep,
            &format!(
                "object 0 {DEF_MAT} difference(frame(t=[0.5,0,0], box(s=[2,2,2])), \
frame(lin=[[2,0,0],[0,1,0],[0,0,1]], sphere(r=1)));"
            ),
        );
        // Difference → first child's bounds (world-space translated box)
        assert_has(&rep, "bounds 0 lo=[-0.5,-1,-1] hi=[1.5,1,1]");
        assert_has(
            &rep,
            &format!(
                "object 1 {DEF_MAT} difference(difference(box(s=[4,4,4]), \
sphere(r=0.5)), frame(t=[1,0,0], sphere(r=0.5)));"
            ),
        );
        assert_has(&rep, "bounds 1 lo=[-2,-2,-2] hi=[2,2,2]");
        assert_has(
            &rep,
            "summary objects=2 lights=0 no_bounds=0 bbox_lo=[-2,-2,-2] bbox_hi=[2,2,2]",
        );
    }

    /// T4 — unbounded geometry: `h=-1` cylinder and plane report
    /// `bounds <i> none`, both counted in `no_bounds`, summary bbox built
    /// from the bounded objects only (incl. the `Affine → a_fwd` path).
    #[test]
    fn test_report_unbounded_bounds() {
        let rep = cgs_report(
            "cylinder(r=1);\n\
             plane(n=[0,0,1], d=0);\n\
             translate([0,1,0]) box(s=[2,1,2]);\n\
             scale([2,1,1]) box(s=[1,1,1]);",
            "",
        )
        .expect("report");
        assert_has(&rep, &format!("object 0 {DEF_MAT} cylinder(r=1, h=-1);"));
        assert_has(&rep, "bounds 0 none");
        assert_has(&rep, &format!("object 1 {DEF_MAT} plane(n=[0,0,1], d=0);"));
        assert_has(&rep, "bounds 1 none");
        assert_has(&rep, "bounds 2 lo=[-1,0.5,-1] hi=[1,1.5,1]");
        assert_has(
            &rep,
            &format!("object 3 {DEF_MAT} frame(lin=[[2,0,0],[0,1,0],[0,0,1]], box(s=[1,1,1]));"),
        );
        assert_has(&rep, "bounds 3 lo=[-1,-0.5,-0.5] hi=[1,0.5,0.5]");
        assert_has(
            &rep,
            "summary objects=4 lights=0 no_bounds=2 bbox_lo=[-1,-0.5,-1] \
bbox_hi=[1,1.5,1]",
        );
    }

    /// T5 — tag registry: names in lexicographic order (BTreeMap, not the
    /// loader's HashMap order), `count=`, instances in emission order, CSG
    /// result registered with an identity world (world-space geometry, no
    /// prefix), no material on instances; two runs byte-identical.
    #[test]
    fn test_report_tags() {
        let src = "tag(\"zeta\") sphere(r=1);\n\
                   tag(\"alpha\") translate([1,0,0]) sphere(r=0.5);\n\
                   tag(\"alpha\") translate([2,0,0]) sphere(r=0.5);\n\
                   tag(\"alpha\") difference() { box(s=[2,2,2]); sphere(r=0.5); }\n\
                   tag(\"alpha\") scale([2,1,1]) box(s=[1,1,1]);\n\
                   tag(\"zeta\") translate([3,0,0]) sphere(r=0.5);";
        let a = cgs_report(src, "").expect("report a");
        let b = cgs_report(src, "").expect("report b");
        assert_eq!(a, b, "two runs must be byte-identical");
        // `add_geometry` registers every geometry statement while a tag is
        // pending, so the CSG block contributes its two children AND the
        // result (scene_lang `register`); the report dumps `named` as-is.
        assert_has(&a, "tag \"alpha\" count=6\n");
        assert_has(&a, "tag \"alpha\" 0 translate([1,0,0]) sphere(r=0.5);\n");
        assert_has(&a, "tag \"alpha\" 1 translate([2,0,0]) sphere(r=0.5);\n");
        assert_has(&a, "tag \"alpha\" 2 box(s=[2,2,2]);\n");
        assert_has(&a, "tag \"alpha\" 3 sphere(r=0.5);\n");
        assert_has(
            &a,
            "tag \"alpha\" 4 difference(box(s=[2,2,2]), sphere(r=0.5));\n",
        );
        assert_has(
            &a,
            "tag \"alpha\" 5 frame(lin=[[2,0,0],[0,1,0],[0,0,1]], box(s=[1,1,1]));\n",
        );
        assert_has(&a, "tag \"zeta\" count=2\n");
        assert_has(&a, "tag \"zeta\" 0 sphere(r=1);\n");
        assert_has(&a, "tag \"zeta\" 1 translate([3,0,0]) sphere(r=0.5);\n");
        // lexicographic: alpha block ends before zeta begins
        let ia = a.find("tag \"alpha\" count").expect("alpha section");
        let iz = a.find("tag \"zeta\" count").expect("zeta section");
        assert!(ia < iz, "tag names not in lexicographic order:\n{a}");
        // no material() on any tag instance line
        assert!(
            !a.lines()
                .any(|l| l.starts_with("tag \"") && l.contains("material(")),
            "tag instances must not print material:\n{a}"
        );
    }

    /// T6 — `extrude(profile, h)` lands as a TrimeshGeometry: local-frame
    /// `trimesh(faces, lo, hi)` (12 tris for a 4-point profile = 8 sides +
    /// 4 cap tris, z from 0 to h).
    #[test]
    fn test_report_trimesh() {
        let rep =
            cgs_report("extrude(profile=[[0,0],[2,0],[2,2],[0,2]], h=3);", "").expect("report");
        assert_has(
            &rep,
            &format!("object 0 {DEF_MAT} trimesh(faces=12, lo=[0,0,0], hi=[2,2,3]);"),
        );
        assert_has(&rep, "bounds 0 lo=[0,0,0] hi=[2,2,3]");
        assert_has(
            &rep,
            "summary objects=1 lights=0 no_bounds=0 bbox_lo=[0,0,0] bbox_hi=[2,2,3]",
        );
    }

    /// T7 — material variants: `unlit=true` uses `basic_material` storage
    /// (roughness/metalness 0, ior 1.5, absorption 0); full standard fields;
    /// hand-built `Texture` prints `map=WxH`.
    #[test]
    fn test_report_material_variants() {
        let rep = cgs_report(
            "material(unlit=true, color=0x123456, opacity=1) box(s=[1,1,1]);\n\
             material(color=0xFF0000, emissive=0x00FF00, roughness=0.9, \
             metalness=0.5, opacity=0.5, ior=1.8, absorption=0.1) sphere(r=1);",
            "",
        )
        .expect("report");
        assert_has(
            &rep,
            "object 0 material(color=0x123456, roughness=0, metalness=0, \
emissive=0x000000, opacity=1, ior=1.5, absorption=0, unlit=true) box(s=[1,1,1]);",
        );
        assert_has(
            &rep,
            "object 1 material(color=0xFF0000, roughness=0.9, metalness=0.5, \
emissive=0x00FF00, opacity=0.5, ior=1.8, absorption=0.1) sphere(r=1);",
        );

        // hand-built scene: map branch (CGS `material.map` needs a file; the
        // report only shows the stored size)
        let mut sc = scene(None);
        let tex = texture_from_rgba(&vec![vec![vec![0.5; 4]; 3]; 2]); // h=2, w=3
        sc.objects.push(mesh(MeshParams {
            geometry: Geometry::SphereGeometry(cga_core::sphere_geometry(1.0)),
            material: Material {
                kind: MaterialKind::Standard,
                color: color_hex(0xFFFFFF),
                roughness: 0.5,
                metalness: 0.0,
                emissive: color_hex(0x000000),
                opacity: 1.0,
                ior: 1.5,
                absorption: 0.0,
                map: Some(tex),
            },
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let rep = scene_report(&sc, &cam(), &TagRegistry::new());
        assert_has(
            &rep,
            &format!(
                "object 0 {} sphere(r=1);",
                DEF_MAT.replace("absorption=0)", "absorption=0, map=3x2)")
            ),
        );
    }

    /// T8 — scene-level payload lines: background, the seven-key camera
    /// (`16/9 → 1.777778`), all three light shapes with fixed keyword order.
    #[test]
    fn test_report_lights_camera() {
        let rep = cgs_report(
            "background(color=0x20313A);\n\
             camera(fov=60, aspect=16/9, position=[1,2,3], target=[0,1,0]);\n\
             ambient_light(color=0x112233, intensity=0.4);\n\
             directional_light(direction=[0,-1,0], color=0x445566, intensity=0.7);\n\
             point_light(position=[1,2,3], color=0x778899, intensity=1.5);\n\
             sphere(r=1);",
            "",
        )
        .expect("report");
        assert_has(&rep, "background(color=0x20313A);\n");
        assert_has(
            &rep,
            "camera(fov=60, aspect=1.777778, position=[1,2,3], target=[0,1,0], \
up=[0,1,0], near=0.1, far=100);\n",
        );
        assert_has(&rep, "ambient_light(color=0x112233, intensity=0.4);\n");
        assert_has(
            &rep,
            "directional_light(direction=[0,-1,0], color=0x445566, intensity=0.7);\n",
        );
        assert_has(
            &rep,
            "point_light(position=[1,2,3], color=0x778899, intensity=1.5);\n",
        );
        assert_has(
            &rep,
            "summary objects=1 lights=3 no_bounds=0 bbox_lo=[-1,-1,-1] bbox_hi=[1,1,1]",
        );
    }
}
