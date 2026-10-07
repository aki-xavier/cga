use cga_core::{decompose_rigid, Geometry, Quaternion};
use std::f64::consts::PI;

use crate::scene_build::{csg_op_name, JointDef, Kinematics, TagInstance, TagRegistry};
use cga_gpu::geom_kernels::geom_to_camera;
use cga_gpu::scene::{Object, PerspectiveCamera, Scene};
use cga_gpu::scene_graph::{vec3_unit, Color};
use cga_gpu::shading::{Light, LightKind, Material, MaterialKind};

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

fn fmt_vec3(v: [f64; 3]) -> String {
    format!("[{},{},{}]", fmt_num(v[0]), fmt_num(v[1]), fmt_num(v[2]))
}

fn fmt_mat3(m: [[f64; 3]; 3]) -> String {
    format!("[{},{},{}]", fmt_vec3(m[0]), fmt_vec3(m[1]), fmt_vec3(m[2]))
}

fn is_identity3_fmt(m: [[f64; 3]; 3]) -> bool {
    fmt_mat3(m) == "[[1,0,0],[0,1,0],[0,0,1]]"
}

fn fmt_color(c: &Color) -> String {
    let ch = |x: f64| (x * 255.0).round().clamp(0.0, 255.0) as i64;
    format!("0x{:06X}", (ch(c.r) << 16) | (ch(c.g) << 8) | ch(c.b))
}

fn frame_of(m4: &[f64; 16]) -> ([f64; 3], [f64; 3], f64) {
    let t = [m4[3], m4[7], m4[11]];
    let r = [
        [m4[0], m4[1], m4[2]],
        [m4[4], m4[5], m4[6]],
        [m4[8], m4[9], m4[10]],
    ];
    let q = Quaternion::from_matrix(r);
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
        Geometry::CsgGeometry(c) => {
            let kids: Vec<String> = c.children.iter().map(fmt_geom).collect();
            format!("{}({})", csg_op_name(c.op), kids.join(", "))
        }
        Geometry::AffineGeometry(a) => {
            let inner = a.inner.first().map(fmt_geom).unwrap_or_default();
            let (t, axis, angle) = frame_of(&a.motor.to_matrix());
            fmt_frame(t, axis, angle, Some(a.linear), &inner)
        }
    }
}

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

fn fmt_object(i: usize, m: &Object) -> String {
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

fn escape_name(name: &str) -> String {
    name.replace('\\', "\\\\").replace('"', "\\\"")
}

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

fn fmt_joint(i: usize, j: &JointDef) -> String {
    let parent = match &j.parent {
        Some(p) => format!("\"{}\"", escape_name(p)),
        None => "none".to_string(),
    };
    let q = if j.q.len() == 1 {
        fmt_num(j.q[0])
    } else {
        let parts: Vec<String> = j.q.iter().map(|x| fmt_num(*x)).collect();
        format!("[{}]", parts.join(","))
    };
    let mut s = format!(
        "joint {i} \"{}\" type={} parent={} axis={} at={} q={q}",
        escape_name(&j.name),
        j.kind.name(),
        parent,
        fmt_vec3(j.axis),
        fmt_vec3(j.at)
    );
    if j.rpy != [0.0, 0.0, 0.0] {
        s.push_str(&format!(" rpy={}", fmt_vec3(j.rpy)));
    }
    if let Some(p) = j.pitch {
        s.push_str(&format!(" pitch={}", fmt_num(p)));
    }
    if let Some([lo, hi]) = j.limit {
        s.push_str(&format!(" limit=[{},{}]", fmt_num(lo), fmt_num(hi)));
    }
    s
}

pub fn scene_report(
    scene: &Scene,
    camera: &PerspectiveCamera,
    tags: &TagRegistry,
    kin: &Kinematics,
) -> String {
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

    let mut no_bounds = 0usize;
    let mut bbox_lo = [f64::INFINITY; 3];
    let mut bbox_hi = [f64::NEG_INFINITY; 3];
    let mut any_bounds = false;
    for (i, m) in scene.objects.iter().enumerate() {
        out.push_str(&fmt_object(i, m));
        out.push('\n');
        let params = geom_to_camera(&m.geometry, &m.motor());
        match cga_gpu::geometry_ops::geom_bounds(&params) {
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
    for (i, j) in kin.joints.iter().enumerate() {
        out.push_str(&fmt_joint(i, j));
        out.push('\n');
    }
    for (i, g) in kin.gears.iter().enumerate() {
        out.push_str(&format!(
            "gear {i} driver=\"{}\" driven=\"{}\" ratio={} offset={}\n",
            escape_name(&g.driver),
            escape_name(&g.driven),
            fmt_num(g.ratio),
            fmt_num(g.offset)
        ));
    }
    for (i, c) in kin.cams.iter().enumerate() {
        out.push_str(&format!(
            "cam {i} driver=\"{}\" driven=\"{}\" q={}\n",
            escape_name(&c.driver),
            escape_name(&c.driven),
            fmt_num(c.q)
        ));
    }
    for (name, v) in &kin.pose {
        out.push_str(&format!("pose {}={}\n", escape_name(name), fmt_num(*v)));
    }
    // 碰撞接触表（C1）：只列非 No 的对象对（Yes / Unknown），没有则整节省略。
    let hits = crate::collision::CollisionScan::new().scan(scene);
    for h in hits
        .iter()
        .filter(|h| h.hit != cga_core::collision::Hit::No)
    {
        let kind = match h.hit {
            cga_core::collision::Hit::Yes => "yes",
            cga_core::collision::Hit::Unknown => "unknown",
            cga_core::collision::Hit::No => unreachable!(),
        };
        let sep = h
            .separation
            .map(fmt_num)
            .unwrap_or_else(|| "unknown".to_string());
        out.push_str(&format!("collide {} {} {} sep={}\n", h.a, h.b, kind, sep));
    }
    let mut summary = format!(
        "summary objects={} lights={} no_bounds={}",
        scene.objects.len(),
        scene.lights.len(),
        no_bounds
    );
    if !kin.joints.is_empty() || !kin.gears.is_empty() || !kin.cams.is_empty() {
        summary.push_str(&format!(
            " joints={} gears={} cams={}",
            kin.joints.len(),
            kin.gears.len(),
            kin.cams.len()
        ));
    }
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

#[cfg(test)]
mod tests {
    use super::*;
    use cga_gpu::scene::{Object, ObjectParams};
    use cga_gpu::scene_graph::Color;
    use cga_gpu::shading::MaterialKind;
    use cga_gpu::texture::Texture;

    const DEF_MAT: &str = "material(color=0xFFFFFF, roughness=0.5, metalness=0, \
emissive=0x000000, opacity=1, ior=1.5, absorption=0)";

    fn cam() -> PerspectiveCamera {
        PerspectiveCamera::new(
            50.0,
            16.0 / 9.0,
            0.1,
            100.0,
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        )
    }

    fn rep(jsx: &str) -> String {
        let run = crate::jsx::run_jsx(jsx, None, "").expect("run");
        scene_report(&run.scene, &run.camera, &run.tags, &run.kinematics)
    }

    fn assert_has(rep: &str, line: &str) {
        assert!(
            rep.contains(line),
            "expected line {line:?} in report:\n{rep}"
        );
    }

    #[test]
    fn test_report_contacts() {
        // 两个重叠的球（sep = 1.5 − 2 = −0.5）→ 接触表一行；全分离则无此节。
        let report = rep(
            "export default <scene><sphere r={1} /><translate t={[1.5,0,0]}><sphere r={1} /></translate></scene>;",
        );
        assert!(report.contains("collide 0 1 yes sep=-0.5"), "{report}");
        let quiet = rep("export default <scene><sphere r={1} /><translate t={[5,0,0]}><sphere r={1} /></translate></scene>;");
        assert!(!quiet.contains("collide"), "{quiet}");
    }

    #[test]
    fn test_report_minimal_golden() {
        let rep = rep("export default <translate t={[2,0,0]}><sphere r={0.5} color={0xFF0000} /></translate>;");
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

    #[test]
    fn test_report_rotation_canonical() {
        let rep = rep("export default <scene>\
<rotate axis={[0,0,1]} angle={Math.PI/2}><sphere r={1} /></rotate>\
<rotate axis={[0,0,1]} angle={-Math.PI/2}><sphere r={1} /></rotate>\
<sphere r={1} />\
<translate t={[1e-9,0,0]}><sphere r={1} /></translate>\
<rotate axis={[0,0,1]} angle={2*Math.PI}><sphere r={1} /></rotate>\
<plane n={[0,0,1]} d={-0.0} />\
</scene>;");
        assert_has(
            &rep,
            "object 0 rotate(axis=[0,0,1], angle=1.570796) material",
        );
        assert_has(
            &rep,
            "object 1 rotate(axis=[0,0,-1], angle=1.570796) material",
        );

        assert_has(&rep, "object 2 material");
        assert_has(&rep, "object 3 material");
        assert_has(&rep, "object 4 material");

        assert!(!rep.contains("angle=-"), "negative angle leaked:\n{rep}");

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

    #[test]
    fn test_report_csg_nested_frames() {
        let rep = rep(
            "export default <scene>\
<difference><translate t={[0.5,0,0]}><box s={[2,2,2]} /></translate><scale s={[2,1,1]}><sphere r={1} /></scale></difference>\
<difference><difference><box s={[4,4,4]} /><sphere r={0.5} /></difference><translate t={[1,0,0]}><sphere r={0.5} /></translate></difference>\
</scene>;",
        );
        assert_has(
            &rep,
            &format!(
                "object 0 {DEF_MAT} difference(frame(t=[0.5,0,0], box(s=[2,2,2])), \
frame(lin=[[2,0,0],[0,1,0],[0,0,1]], sphere(r=1)));"
            ),
        );

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

    #[test]
    fn test_report_unbounded_bounds() {
        let rep = rep("export default <scene>\
<cylinder r={1} />\
<plane n={[0,0,1]} d={0} />\
<translate t={[0,1,0]}><box s={[2,1,2]} /></translate>\
<scale s={[2,1,1]}><box s={[1,1,1]} /></scale>\
</scene>;");
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

    #[test]
    fn test_report_tags() {
        let src = "export default <scene>\
<tag name=\"zeta\"><sphere r={1} /></tag>\
<tag name=\"alpha\"><translate t={[1,0,0]}><sphere r={0.5} /></translate></tag>\
<tag name=\"alpha\"><translate t={[2,0,0]}><sphere r={0.5} /></translate></tag>\
<tag name=\"alpha\"><difference><box s={[2,2,2]} /><sphere r={0.5} /></difference></tag>\
<tag name=\"alpha\"><scale s={[2,1,1]}><box s={[1,1,1]} /></scale></tag>\
<tag name=\"zeta\"><translate t={[3,0,0]}><sphere r={0.5} /></translate></tag>\
</scene>;";
        let a = rep(src);
        let b = rep(src);
        assert_eq!(a, b, "two runs must be byte-identical");

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

        let ia = a.find("tag \"alpha\" count").expect("alpha section");
        let iz = a.find("tag \"zeta\" count").expect("zeta section");
        assert!(ia < iz, "tag names not in lexicographic order:\n{a}");

        assert!(
            !a.lines()
                .any(|l| l.starts_with("tag \"") && l.contains("material(")),
            "tag instances must not print material:\n{a}"
        );
    }
    #[test]
    fn test_report_material_variants() {
        let rep = rep(
            "export default <scene>\
<box s={[1,1,1]} unlit={true} color={0x123456} opacity={1} />\
<sphere r={1} color={0xFF0000} emissive={0x00FF00} roughness={0.9} metalness={0.5} opacity={0.5} ior={1.8} absorption={0.1} />\
</scene>;",
        );
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

        let mut sc = Scene::new(None);
        let tex = Texture::from_rgba(&vec![vec![vec![0.5; 4]; 3]; 2]);
        sc.objects.push(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(cga_core::SphereGeometry::new(1.0)),
            material: Material {
                kind: MaterialKind::Standard,
                color: Color::from_hex(0xFFFFFF),
                roughness: 0.5,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
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
        let rep = scene_report(&sc, &cam(), &TagRegistry::new(), &Kinematics::default());
        assert_has(
            &rep,
            &format!(
                "object 0 {} sphere(r=1);",
                DEF_MAT.replace("absorption=0)", "absorption=0, map=3x2)")
            ),
        );
    }

    #[test]
    fn test_report_lights_camera() {
        let rep = rep("export default <scene>\
<background color={0x20313A} />\
<camera fov={60} aspect={16/9} position={[1,2,3]} target={[0,1,0]} />\
<ambient_light color={0x112233} intensity={0.4} />\
<directional_light direction={[0,-1,0]} color={0x445566} intensity={0.7} />\
<point_light position={[1,2,3]} color={0x778899} intensity={1.5} />\
<sphere r={1} />\
</scene>;");
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
