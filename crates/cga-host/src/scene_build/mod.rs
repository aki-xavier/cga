//! Scene-build residue of the retired CGS parser (R4): value types, builders,
//! kinematics, face/bounds helpers. Text parsing lives in the JSX host
//! (crate::jsx); CGS text syntax is frozen and no longer parsed.

use std::collections::{BTreeMap, HashMap};
use std::fmt;

use cga_core::{
    clamp01, extrude, load_obj, loft, mat4_identity, mat4_mul, transform_point, validate_profile,
    BezierPatchGeometry, BoxGeometry, CircleGeometry, ConeGeometry, CsgOp, CyclideGeometry,
    CylinderGeometry, EllipsoidGeometry, Geometry, Multivector, PlaneGeometry, SphereGeometry,
    TorusGeometry, TrimeshGeometry,
};

use cga_gpu::mesh_io_gltf::{gltf_to_geometry, load_gltf};
use cga_gpu::scene::Scene;
use cga_gpu::scene_graph::{vec3_dot, Color};
use cga_gpu::shading::{Material, MaterialParams};
use cga_gpu::texture::texture_load;

pub mod arg_value;
pub use self::arg_value::*;
pub mod arg_vec3;
pub use self::arg_vec3::*;
pub mod scene_run;
pub use self::scene_run::*;
pub mod kinematics;
pub use self::kinematics::*;
pub mod tag_instance;
pub use self::tag_instance::*;
pub mod builders;
pub use self::builders::*;

pub type TagRegistry = BTreeMap<String, Vec<TagInstance>>;

pub(crate) fn arg_truthy(v: &ArgValue) -> bool {
    match v {
        ArgValue::Bool(b) => *b,
        ArgValue::Num(x) => *x != 0.0,
        ArgValue::Str(s) => !s.is_empty(),
        ArgValue::List(l) => !l.is_empty(),
        ArgValue::Vec3(_) => true,
    }
}
fn fmt_f64(x: f64) -> String {
    if x.is_finite() && x.fract() == 0.0 && x.abs() < 1e16 {
        format!("{x:.1}")
    } else {
        format!("{x}")
    }
}

fn cgs_num(v: &ArgValue, _line: i32, what: &str) -> Result<f64, String> {
    match v {
        ArgValue::Num(x) => Ok(*x),
        _ => Err(format!("build: {what} needs a number, got {v}")),
    }
}

fn cgs_vec3(v: &ArgValue, line: i32, what: &str) -> Result<[f64; 3], String> {
    match v {
        ArgValue::Vec3(v3) => Ok([v3.x, v3.y, v3.z]),
        ArgValue::List(items) => {
            if items.len() != 3 {
                return Err(format!("build: {what} needs [x,y,z], got {v}"));
            }
            Ok([
                cgs_num(&items[0], line, what)?,
                cgs_num(&items[1], line, what)?,
                cgs_num(&items[2], line, what)?,
            ])
        }
        _ => Err(format!("build: {what} needs [x,y,z], got {v}")),
    }
}

fn cgs_opt_num(v: &ArgValue, def: f64) -> f64 {
    match v {
        ArgValue::Num(x) => {
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

pub(crate) fn parse_face_key(key: &str, _line: i32, what: &str) -> Result<(usize, f64), String> {
    let ok =
        key.len() == 2 && matches!(&key[..1], "+" | "-") && matches!(&key[1..], "x" | "y" | "z");
    if !ok {
        return Err(format!(
            "build: {what}: unknown face key \"{key}\" (expected \"+x\"/\"-x\"/\"+y\"/\"-y\"/\"+z\"/\"-z\")"
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
            let b = cga_gpu::geometry_ops::geom_bounds(&geo.identity_params())?;
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

pub(crate) fn csg_op_name(op: CsgOp) -> &'static str {
    match op {
        CsgOp::Union => "union",
        CsgOp::Difference => "difference",
        CsgOp::Intersection => "intersection",
    }
}

fn arg_num(args: &HashMap<String, ArgValue>, key: &str) -> f64 {
    cgs_num(
        &args.get(key).cloned().unwrap_or(ArgValue::Num(0.0)),
        0,
        key,
    )
    .unwrap_or(0.0)
}

fn validate_geometry_params(
    name: &str,
    args: &HashMap<String, ArgValue>,
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
        if arg_num(args, k) <= 0.0 {
            return Err(format!("build: {name}.{k} must be > 0"));
        }
    }
    match name {
        "box" | "ellipsoid" => {
            let key = if name == "box" { "s" } else { "radii" };
            let v = cgs_vec3(
                &args.get(key).cloned().unwrap_or(ArgValue::Num(0.0)),
                line,
                &format!("{name}.{key}"),
            )?;
            if v[0].min(v[1].min(v[2])) <= 0.0 {
                return Err(format!("build: {name}.{key} components must be > 0"));
            }
        }
        "plane" => {
            let n = cgs_vec3(
                &args.get("n").cloned().unwrap_or(ArgValue::Num(0.0)),
                line,
                "plane.n",
            )?;
            if vec3_dot(n, n) < 1e-24 {
                return Err(format!("build: plane.n must not be zero"));
            }
        }
        "torus" => {
            let arc = arg_num(args, "arc");
            if !(arc > 0.0) || arc > std::f64::consts::TAU {
                return Err(format!("build: torus.arc must be in (0, 2*pi]"));
            }
        }
        "cyclide" => {
            let a = arg_num(args, "a");
            let b = arg_num(args, "b");
            let d = arg_num(args, "d");
            if !(a > b && b > 0.0) {
                return Err(format!("build: cyclide needs a > b > 0"));
            }
            if d <= 0.0 {
                return Err(format!("build: cyclide.d must be > 0"));
            }
        }
        "loft" => {
            if let Some(ArgValue::List(raw)) = args.get("profiles") {
                let mut m: i64 = -1;
                for p in raw {
                    if let ArgValue::List(pl) = p {
                        if m >= 0 && pl.len() as i64 != m {
                            return Err(format!(
                                "build: loft profiles must share the same vertex count"
                            ));
                        }
                        m = pl.len() as i64;
                    }
                }
            }
            let mut zs: Vec<f64> = Vec::new();
            let zsraw = args.get("zs").cloned().unwrap_or(ArgValue::Num(0.0));
            match zsraw {
                ArgValue::Vec3(v3) => {
                    zs = vec![v3.x, v3.y, v3.z];
                }
                ArgValue::List(zlist) => {
                    for z in &zlist {
                        zs.push(cgs_num(z, line, "loft.zs[i]")?);
                    }
                }
                _ => {}
            }
            for i in 0..zs.len().saturating_sub(1) {
                if zs[i] >= zs[i + 1] {
                    return Err(format!("build: loft.zs must be strictly increasing"));
                }
            }
        }
        "bezier" => {
            if arg_num(args, "thickness") < 0.0 {
                return Err(format!("build: bezier.thickness must be >= 0"));
            }
            let div = arg_num(args, "div");
            if div.fract() != 0.0 || div < 1.0 || div > 32.0 {
                return Err(format!("build: bezier.div must be an integer in 1..=32"));
            }
            match args.get("points") {
                Some(ArgValue::List(items)) if items.len() == 16 => {}
                Some(ArgValue::List(items)) => {
                    return Err(format!(
                        "build: bezier.points needs 16 [x,y,z] control points, got {}",
                        items.len()
                    ));
                }
                _ => {
                    return Err(format!(
                        "build: bezier.points needs 16 [x,y,z] control points, got none"
                    ));
                }
            }
        }
        _ => {}
    }
    Ok(())
}

fn profile2d(v: &ArgValue, line: i32, what: &str) -> Result<Vec<[f64; 2]>, String> {
    match v {
        ArgValue::List(items) => {
            if items.len() < 3 {
                return Err(format!("build: {what} needs >= 3 [x,y] points"));
            }
            let mut pts: Vec<[f64; 2]> = Vec::new();
            for p in items {
                match p {
                    ArgValue::List(pair) => {
                        if pair.len() != 2 {
                            return Err(format!("build: {what} items must be [x,y]"));
                        }
                        let x = cgs_num(&pair[0], line, what)?;
                        let y = cgs_num(&pair[1], line, what)?;
                        pts.push([x, y]);
                    }
                    _ => {
                        return Err(format!("build: {what} items must be [x,y]"));
                    }
                }
            }

            validate_profile(&pts).map_err(|e| format!("build: {e}"))?;
            Ok(pts)
        }
        _ => Err(format!("build: {what} needs a list")),
    }
}

fn points16(v: &ArgValue, line: i32, what: &str) -> Result<Vec<[f64; 3]>, String> {
    match v {
        ArgValue::List(items) => {
            if items.len() != 16 {
                return Err(format!(
                    "build: {what} needs 16 [x,y,z] control points, got {}",
                    items.len()
                ));
            }
            let mut pts: Vec<[f64; 3]> = Vec::with_capacity(16);
            for p in items {
                pts.push(cgs_vec3(p, line, what)?);
            }
            Ok(pts)
        }
        _ => Err(format!("build: {what} needs a list")),
    }
}

fn sig_names(name: &str) -> Vec<&'static str> {
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

fn sig_defaults(name: &str) -> HashMap<String, ArgValue> {
    let mut m: HashMap<String, ArgValue> = HashMap::new();
    match name {
        "plane" => {
            m.insert("d".to_string(), ArgValue::Num(0.0));
        }
        "cylinder" => {
            m.insert("h".to_string(), ArgValue::Num(-1.0));
        }
        "torus" => {
            m.insert("arc".to_string(), ArgValue::Num(std::f64::consts::TAU));
        }
        "bezier" => {
            m.insert("thickness".to_string(), ArgValue::Num(0.0));
            m.insert("div".to_string(), ArgValue::Num(8.0));
        }
        "directional_light" | "point_light" => {
            m.insert("intensity".to_string(), ArgValue::Num(1.0));
            m.insert("color".to_string(), ArgValue::Num(0xFFFFFF as f64));
        }
        "ambient_light" => {
            m.insert("intensity".to_string(), ArgValue::Num(0.3));
            m.insert("color".to_string(), ArgValue::Num(0xFFFFFF as f64));
        }
        "camera" => {
            m.insert("fov".to_string(), ArgValue::Num(50.0));
            m.insert("aspect".to_string(), ArgValue::Num(16.0 / 9.0));
            m.insert(
                "position".to_string(),
                ArgValue::Vec3(ArgVec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                }),
            );
            m.insert(
                "target".to_string(),
                ArgValue::Vec3(ArgVec3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                }),
            );
        }
        "material" => {
            m.insert("color".to_string(), ArgValue::Num(0xFFFFFF as f64));
            m.insert("roughness".to_string(), ArgValue::Num(-1.0));
            m.insert("metalness".to_string(), ArgValue::Num(-1.0));
            m.insert("emissive".to_string(), ArgValue::Num(-1.0));
            m.insert("opacity".to_string(), ArgValue::Num(-1.0));
            m.insert("ior".to_string(), ArgValue::Num(-1.0));
            m.insert("absorption".to_string(), ArgValue::Num(-1.0));
            m.insert("unlit".to_string(), ArgValue::Bool(false));
            m.insert("map".to_string(), ArgValue::Str(String::new()));
        }
        _ => {}
    }
    m
}

#[cfg(test)]
mod tests {
    use super::*;

    fn geom_kind(g: &Geometry) -> &'static str {
        match g {
            Geometry::CsgGeometry(_) => "csg",
            Geometry::ConeGeometry(_) => "cone",
            Geometry::TorusGeometry(_) => "torus",
            Geometry::EllipsoidGeometry(_) => "ellipsoid",
            Geometry::TrimeshGeometry(_) => "mesh",
            Geometry::SphereGeometry(_) => "sphere",
            Geometry::BoxGeometry(_) => "box",
            Geometry::CylinderGeometry(_) => "cylinder",
            Geometry::PlaneGeometry(_) => "plane",
            _ => "other",
        }
    }

    #[test]
    fn test_jsx_modifier_ordering() {
        let sc = crate::jsx::run_jsx(
            "export default <translate t={[10,0,0]}><scale s={2}><sphere r={1} /></scale></translate>;",
            None,
            "",
        )
        .unwrap()
        .scene;
        assert_eq!(sc.objects.len(), 1);
        let p = sc.objects[0].position;
        assert!((p[0] - 10.0).abs() < 1e-6);
        assert!(p[1].abs() < 1e-6);
        assert!(p[2].abs() < 1e-6);
        let sc2 = crate::jsx::run_jsx(
            "export default <mirror axis={[1,0,0]}><translate t={[2.5,0,0]}><sphere r={1} /></translate></mirror>;",
            None,
            "",
        )
        .unwrap()
        .scene;
        let p2 = sc2.objects[0].position;
        assert!((p2[0] + 2.5).abs() < 1e-6);
        assert!(p2[1].abs() < 1e-6);
        assert!(p2[2].abs() < 1e-6);
    }

    #[test]
    fn test_jsx_csg_block_and_new_primitives() {
        let text = "export default (\n  <scene>\n    \
            <difference><box s={[2,2,2]} /><cylinder r={0.5} h={4} /></difference>\n    \
            <cone r={1} h={2} />\n    \
            <torus R={1} r={0.3} />\n    \
            <ellipsoid radii={[1,2,3]} />\n    \
            <extrude profile={[[0,0],[1,0],[1,1],[0,1]]} h={0.5} />\n    \
            <loft profiles={[[[0,0],[1,0],[1,1],[0,1]], [[0.2,0.2],[0.8,0.2],[0.8,0.8],[0.2,0.8]]]} zs={[0, 0.5]} />\n  \
            </scene>\n);";
        let sc = crate::jsx::run_jsx(text, None, "").unwrap().scene;
        assert_eq!(sc.objects.len(), 6);
        assert_eq!(geom_kind(&sc.objects[0].geometry), "csg");
        assert_eq!(geom_kind(&sc.objects[1].geometry), "cone");
        assert_eq!(geom_kind(&sc.objects[2].geometry), "torus");
        assert_eq!(geom_kind(&sc.objects[3].geometry), "ellipsoid");
        assert_eq!(geom_kind(&sc.objects[4].geometry), "mesh");
        assert_eq!(geom_kind(&sc.objects[5].geometry), "mesh");
    }

    #[test]
    fn test_jsx_gltf_mesh() {
        let (verts, faces) = extrude(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]], 1.0);
        cga_gpu::save_glb(
            "/tmp/cga_host.glb",
            &[cga_gpu::GltfMeshIn {
                vertices: verts,
                faces,
                transform: None,
                color: None,
            }],
        );
        let sc = crate::jsx::run_jsx(
            "export default <mesh file=\"cga_host.glb\" />;",
            None,
            "/tmp",
        )
        .unwrap()
        .scene;
        assert_eq!(sc.objects.len(), 1);
        assert_eq!(geom_kind(&sc.objects[0].geometry), "mesh");
        let _ = std::fs::remove_file("/tmp/cga_host.glb");
    }
}
