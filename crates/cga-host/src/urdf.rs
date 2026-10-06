//! URDF interop (关节化 P5/P6（调研文档已随完成退役）).
//!
//! Export: the joint tree maps 1:1 onto URDF (origin xyz/rpy ↔ at/rpy,
//! axis in the joint frame, limit, gear → mimic). helical/cylindrical/
//! spherical decompose into 1-DOF series joints (documented downgrade).
//! Link geometry: a single native primitive (sphere/box/cylinder) exports as
//! URDF geometry; anything else bakes to a watertight OBJ.
//!
//! Import: urdf-rs parses, JSX text is generated `jsx_gen`-style (flat,
//! deterministic, parseable by construction).

use cga_core::GeometryParams;

use crate::jsx::run_jsx;
use crate::scene_build::{JointDef, JointKind, Kinematics, SceneRun};
use cga_gpu::geom_to_camera;

fn uf(v: f64) -> String {
    let s = format!("{v:.6}");
    let t = s.trim_end_matches('0');
    let t = t.strip_suffix('.').unwrap_or(t);
    if t.is_empty() || t == "-" || t == "-0" {
        "0".to_string()
    } else {
        t.to_string()
    }
}

fn uv(v: [f64; 3]) -> String {
    format!("{} {} {}", uf(v[0]), uf(v[1]), uf(v[2]))
}

/// Rigid inverse of a row-major [R|t] 4×4.
fn rigid_inv(m: [f64; 16]) -> [f64; 16] {
    let mut r = [0.0; 16];
    for i in 0..3 {
        for j in 0..3 {
            r[i * 4 + j] = m[j * 4 + i];
        }
    }
    let t = [m[3], m[7], m[11]];
    for i in 0..3 {
        r[i * 4 + 3] = -(r[i * 4] * t[0] + r[i * 4 + 1] * t[1] + r[i * 4 + 2] * t[2]);
    }
    r[15] = 1.0;
    r
}

fn xform_point(m: [f64; 16], p: [f64; 3]) -> [f64; 3] {
    [
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
    ]
}

fn xform_dir(m: [f64; 16], v: [f64; 3]) -> [f64; 3] {
    [
        m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
        m[4] * v[0] + m[5] * v[1] + m[6] * v[2],
        m[8] * v[0] + m[9] * v[1] + m[10] * v[2],
    ]
}

/// Fixed-axis rpy from a rotation 3×3: R = Rz(yaw)·Ry(pitch)·Rx(roll).
fn mat_to_rpy(m: [f64; 16]) -> [f64; 3] {
    let pitch = (-m[8]).atan2((m[0] * m[0] + m[4] * m[4]).sqrt());
    let roll = m[9].atan2(m[10]);
    let yaw = m[4].atan2(m[0]);
    [roll, pitch, yaw]
}

#[derive(Debug)]
pub struct UrdfExport {
    pub xml: String,
    /// (相对文件名, STL 字节) —— 非图元链接几何的三角化产物。
    pub meshes: Vec<(String, Vec<u8>)>,
}

/// 一个对象的 visual：URDF 原生图元，或（非图元）三角化后的 STL 引用。
struct Visual {
    origin_xyz: [f64; 3],
    origin_rpy: [f64; 3],
    geom: String,
    mesh_file: Option<String>,
    mesh_data: Option<Vec<u8>>,
}

fn visual_of(
    run: &SceneRun,
    mesh_idx: usize,
    link_world: [f64; 16],
    link: &str,
    slot: usize,
    step: f64,
) -> Result<Visual, String> {
    let m = &run.scene.objects[mesh_idx];
    let world = m.motor().to_matrix();
    let inv = rigid_inv(link_world);
    let p = geom_to_camera(&m.geometry, &m.motor());
    let to_link_p = |q: [f64; 3]| xform_point(inv, q);
    let to_link_d = |q: [f64; 3]| xform_dir(inv, q);
    let (xyz, rpy, geom): ([f64; 3], [f64; 3], Option<String>) = match &p {
        GeometryParams::SphereParams(s) => (
            to_link_p(s.c),
            [0.0; 3],
            Some(format!("<sphere radius=\"{}\"/>", uf(s.r))),
        ),
        GeometryParams::BoxParams(b) => {
            let c = to_link_p(b.c);
            // Rotation rows: R(row i) maps local axes into link frame.
            let mut rm = [0.0; 16];
            for i in 0..3 {
                let a = to_link_d(b.axes[i]);
                rm[i] = a[0];
                rm[4 + i] = a[1];
                rm[8 + i] = a[2];
            }
            rm[15] = 1.0;
            (
                c,
                mat_to_rpy(rm),
                Some(format!(
                    "<box size=\"{} {} {}\"/>",
                    uf(2.0 * b.half[0]),
                    uf(2.0 * b.half[1]),
                    uf(2.0 * b.half[2])
                )),
            )
        }
        GeometryParams::CylinderParams(cy) if cy.h > 0.0 => {
            let q = to_link_p(cy.q);
            let u = to_link_d(cy.u);
            // Rotate local +z onto u: rpy of that alignment.
            let z = [0.0, 0.0, 1.0];
            let v = [
                z[1] * u[2] - z[2] * u[1],
                z[2] * u[0] - z[0] * u[2],
                z[0] * u[1] - z[1] * u[0],
            ];
            let c = z[0] * u[0] + z[1] * u[1] + z[2] * u[2];
            let vn = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
            let rpy = if vn < 1e-12 {
                if c > 0.0 {
                    [0.0; 3]
                } else {
                    [std::f64::consts::PI, 0.0, 0.0]
                }
            } else {
                let ang = vn.atan2(c);
                let ax = [v[0] / vn, v[1] / vn, v[2] / vn];
                let (ca, sa) = (ang.cos(), ang.sin());
                // axis-angle → 3×3 (row-major), then → rpy
                let r = [
                    ca + ax[0] * ax[0] * (1.0 - ca),
                    ax[0] * ax[1] * (1.0 - ca) - ax[2] * sa,
                    ax[0] * ax[2] * (1.0 - ca) + ax[1] * sa,
                    0.0,
                    ax[1] * ax[0] * (1.0 - ca) + ax[2] * sa,
                    ca + ax[1] * ax[1] * (1.0 - ca),
                    ax[1] * ax[2] * (1.0 - ca) - ax[0] * sa,
                    0.0,
                    ax[2] * ax[0] * (1.0 - ca) - ax[1] * sa,
                    ax[2] * ax[1] * (1.0 - ca) + ax[0] * sa,
                    ca + ax[2] * ax[2] * (1.0 - ca),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ];
                mat_to_rpy(r)
            };
            (
                q,
                rpy,
                Some(format!(
                    "<cylinder radius=\"{}\" length=\"{}\"/>",
                    uf(cy.r),
                    uf(2.0 * cy.h)
                )),
            )
        }
        _ => (world_origin(world), [0.0; 3], None),
    };
    if let Some(g) = geom {
        return Ok(Visual {
            origin_xyz: xyz,
            origin_rpy: rpy,
            geom: g,
            mesh_file: None,
            mesh_data: None,
        });
    }
    // 非图元几何（解析 CSG 等）：导出方向三角化成 STL，放链接坐标系（origin 归零）。
    let baked = p
        .bake(step)
        .map_err(|e| format!("urdf: bake failed for link {link}: {e}"))?;
    let verts: Vec<[f64; 3]> = baked
        .vertices
        .iter()
        .map(|v| xform_point(inv, *v))
        .collect();
    let data = cga_core::stl_binary(&verts, &baked.faces);
    let file = format!("meshes/{link}_{slot}.stl");
    Ok(Visual {
        origin_xyz: [0.0; 3],
        origin_rpy: [0.0; 3],
        geom: format!("<mesh filename=\"{file}\"/>"),
        mesh_file: Some(file),
        mesh_data: Some(data),
    })
}

fn world_origin(world: [f64; 16]) -> [f64; 3] {
    [world[3], world[7], world[11]]
}

fn urdf_kind(k: &JointKind) -> Option<&'static str> {
    match k {
        JointKind::Revolute => Some("revolute"),
        JointKind::Continuous => Some("continuous"),
        JointKind::Prismatic => Some("prismatic"),
        JointKind::Fixed => Some("fixed"),
        JointKind::Planar => Some("planar"),
        _ => None,
    }
}

fn limit_xml(j: &JointDef, ty: &str) -> Result<String, String> {
    match ty {
        "revolute" | "prismatic" => {
            let [lo, hi] = j
                .limit
                .ok_or_else(|| format!("urdf: joint {} needs limit= for URDF export", j.name))?;
            Ok(format!(
                "<limit lower=\"{}\" upper=\"{}\" effort=\"0\" velocity=\"0\"/>",
                uf(lo),
                uf(hi)
            ))
        }
        _ => Ok(String::new()),
    }
}

fn mimic_xml(kin: &Kinematics, name: &str) -> String {
    match kin.gears.iter().find(|g| g.driven == name) {
        Some(g) => format!(
            "<mimic joint=\"{}\" multiplier=\"{}\" offset=\"{}\"/>",
            g.driver,
            uf(g.ratio),
            uf(g.offset)
        ),
        None => String::new(),
    }
}

fn joint_origin(j: &JointDef) -> String {
    format!("<origin xyz=\"{}\" rpy=\"{}\"/>", uv(j.at), uv(j.rpy))
}

/// Export a JSX scene's joint tree as URDF. `step` 是非图元链接几何三角化的网格步长
/// （导出方向；导入不提供）。
pub fn jsx_to_urdf(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
    robot: &str,
    step: f64,
) -> Result<UrdfExport, String> {
    let run = run_jsx(jsx_src, css_src, asset_root)?;
    let kin = &run.kinematics;
    let mut xml = String::new();
    xml.push_str(&format!("<robot name=\"{robot}\">\n"));
    let mut meshes: Vec<(String, Vec<u8>)> = Vec::new();
    let emit_link = |name: &str,
                     object_ids: &[usize],
                     link_world: [f64; 16],
                     meshes: &mut Vec<(String, Vec<u8>)>,
                     xml: &mut String|
     -> Result<(), String> {
        xml.push_str(&format!("  <link name=\"{name}\">\n"));
        for (slot, &mi) in object_ids.iter().enumerate() {
            let v = visual_of(&run, mi, link_world, name, slot, step)?;
            xml.push_str("    <visual>\n");
            xml.push_str(&format!(
                "      <origin xyz=\"{}\" rpy=\"{}\"/>\n",
                uv(v.origin_xyz),
                uv(v.origin_rpy)
            ));
            xml.push_str(&format!("      <geometry>{}\n", v.geom));
            xml.push_str("      </geometry>\n    </visual>\n");
            if let (Some(f), Some(d)) = (v.mesh_file, v.mesh_data) {
                meshes.push((f, d));
            }
        }
        xml.push_str("  </link>\n");
        Ok(())
    };

    // base_link carries every mesh outside any joint body.
    let owned: std::collections::BTreeSet<usize> = kin
        .joints
        .iter()
        .flat_map(|j| j.meshes.iter().copied())
        .collect();
    let free: Vec<usize> = (0..run.scene.objects.len())
        .filter(|i| !owned.contains(i))
        .collect();
    emit_link(
        "base_link",
        &free,
        cga_core::mat4_identity(),
        &mut meshes,
        &mut xml,
    )?;

    for j in &kin.joints {
        emit_link(
            &format!("{}_link", j.name),
            &j.meshes,
            j.world,
            &mut meshes,
            &mut xml,
        )?;
    }

    let parent_link = |j: &JointDef| -> String {
        match &j.parent {
            Some(p) => format!("{p}_link"),
            None => "base_link".to_string(),
        }
    };
    let joint_xml = |xml: &mut String,
                     name: &str,
                     ty: &str,
                     parent: &str,
                     child: &str,
                     origin: &str,
                     axis: Option<[f64; 3]>,
                     extra: &str| {
        xml.push_str(&format!("  <joint name=\"{name}\" type=\"{ty}\">\n"));
        xml.push_str(&format!("    <parent link=\"{parent}\"/>\n"));
        xml.push_str(&format!("    <child link=\"{child}\"/>\n"));
        xml.push_str(&format!("    {origin}\n"));
        if let Some(a) = axis {
            xml.push_str(&format!("    <axis xyz=\"{}\"/>\n", uv(a)));
        }
        if !extra.is_empty() {
            xml.push_str(&format!("    {extra}\n"));
        }
        xml.push_str("  </joint>\n");
    };

    for j in &kin.joints {
        let origin = joint_origin(j);
        let mimic = mimic_xml(kin, &j.name);
        match j.kind {
            JointKind::Revolute
            | JointKind::Continuous
            | JointKind::Prismatic
            | JointKind::Fixed
            | JointKind::Planar => {
                let ty = urdf_kind(&j.kind).unwrap();
                let limit = limit_xml(j, ty)?;
                let extra = format!("{limit}{mimic}");
                joint_xml(
                    &mut xml,
                    &j.name,
                    ty,
                    &parent_link(j),
                    &format!("{}_link", j.name),
                    &origin,
                    if j.kind == JointKind::Fixed {
                        None
                    } else {
                        Some(j.axis)
                    },
                    extra.trim(),
                );
            }
            JointKind::Helical => {
                // revolute + mimic'd prismatic: q_slide = pitch · q_rot.
                let limit = limit_xml(j, "revolute")?;
                joint_xml(
                    &mut xml,
                    &j.name,
                    "revolute",
                    &parent_link(j),
                    &format!("{}_screw", j.name),
                    &origin,
                    Some(j.axis),
                    limit.trim(),
                );
                xml.push_str(&format!("  <link name=\"{}_screw\"/>\n", j.name));
                let pitch = j.pitch.unwrap_or(0.0);
                joint_xml(
                    &mut xml,
                    &format!("{}_slide", j.name),
                    "prismatic",
                    &format!("{}_screw", j.name),
                    &format!("{}_link", j.name),
                    "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>",
                    Some(j.axis),
                    &format!(
                        "<limit lower=\"-inf\" upper=\"inf\" effort=\"0\" velocity=\"0\"/><mimic joint=\"{}\" multiplier=\"{}\" offset=\"0\"/>",
                        j.name,
                        uf(pitch)
                    ),
                );
            }
            JointKind::Cylindrical => {
                let limit = limit_xml(j, "revolute")?;
                joint_xml(
                    &mut xml,
                    &format!("{}_rot", j.name),
                    "revolute",
                    &parent_link(j),
                    &format!("{}_cyl", j.name),
                    &origin,
                    Some(j.axis),
                    limit.trim(),
                );
                xml.push_str(&format!("  <link name=\"{}_cyl\"/>\n", j.name));
                joint_xml(
                    &mut xml,
                    &format!("{}_slide", j.name),
                    "prismatic",
                    &format!("{}_cyl", j.name),
                    &format!("{}_link", j.name),
                    "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>",
                    Some(j.axis),
                    "<limit lower=\"-inf\" upper=\"inf\" effort=\"0\" velocity=\"0\"/>",
                );
            }
            JointKind::Spherical => {
                // Series Rx → Ry → Rz, matching the spherical motion.
                let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                let names = ["r", "p", "y"];
                let links = [
                    parent_link(j),
                    format!("{}_sx", j.name),
                    format!("{}_sy", j.name),
                ];
                for i in 0..3 {
                    let child = if i == 2 {
                        format!("{}_link", j.name)
                    } else {
                        format!("{}_s{}", j.name, ["x", "y"][i])
                    };
                    let o = if i == 0 {
                        origin.clone()
                    } else {
                        "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>".to_string()
                    };
                    joint_xml(
                        &mut xml,
                        &format!("{}_{}", j.name, names[i]),
                        "revolute",
                        &links[i],
                        &child,
                        &o,
                        Some(axes[i]),
                        "<limit lower=\"-3.141593\" upper=\"3.141593\" effort=\"0\" velocity=\"0\"/>",
                    );
                    if i < 2 {
                        xml.push_str(&format!("  <link name=\"{}\"/>\n", links[i + 1]));
                    }
                }
            }
        }
    }
    xml.push_str("</robot>\n");
    Ok(UrdfExport { xml, meshes })
}

#[cfg(test)]
mod tests {
    use super::*;
    use urdf_rs::JointType;

    const ARM: &str = r#"export default (
  <scene>
    <joint name="shoulder" type="revolute" axis={[0,0,1]} q={0.3} limit={[-1.57, 1.57]}>
      <cylinder r={0.05} h={0.4} />
      <joint name="elbow" type="prismatic" axis={[1,0,0]} at={[0.3,0,0]} limit={[0, 0.2]}>
        <box s={[0.2, 0.2, 0.2]} />
        <gear driver="elbow" driven="wrist" ratio={2} offset={0.1} />
        <joint name="wrist" type="revolute" axis={[0,1,0]} at={[0.5,0,0]} limit={[-1, 1]}>
          <sphere r={0.08} />
        </joint>
      </joint>
    </joint>
  </scene>
);"#;

    #[test]
    fn test_p5_urdf_export_arm() {
        let e = jsx_to_urdf(ARM, None, "", "arm", 0.08).unwrap();
        let x = &e.xml;
        assert!(x.contains("<robot name=\"arm\">"), "{x}");
        assert!(
            x.contains("<joint name=\"shoulder\" type=\"revolute\">"),
            "{x}"
        );
        assert!(x.contains("<axis xyz=\"0 0 1\"/>"), "{x}");
        assert!(
            x.contains("<limit lower=\"-1.57\" upper=\"1.57\" effort=\"0\" velocity=\"0\"/>"),
            "{x}"
        );
        assert!(
            x.contains("<cylinder radius=\"0.05\" length=\"0.4\"/>"),
            "{x}"
        );
        assert!(x.contains("<box size=\"0.2 0.2 0.2\"/>"), "{x}");
        assert!(
            x.contains("<mimic joint=\"elbow\" multiplier=\"2\" offset=\"0.1\"/>"),
            "{x}"
        );
        assert!(x.contains("<sphere radius=\"0.08\"/>"), "{x}");
        assert!(x.contains("<parent link=\"shoulder_link\"/>"), "{x}");
        assert!(x.contains("<parent link=\"base_link\"/>"), "{x}");
    }

    #[test]
    fn test_p5_urdf_export_roundtrip_urdf_rs() {
        let e = jsx_to_urdf(ARM, None, "", "arm", 0.08).unwrap();
        let robot = urdf_rs::read_from_string(&e.xml).expect("urdf-rs 读回应成功");
        assert_eq!(robot.joints.len(), 3);
        let sh = robot.joints.iter().find(|j| j.name == "shoulder").unwrap();
        assert!(matches!(sh.joint_type, JointType::Revolute));
        assert!((sh.limit.lower + 1.57).abs() < 1e-6);
        let wr = robot.joints.iter().find(|j| j.name == "wrist").unwrap();
        let m = wr.mimic.as_ref().expect("wrist 应带 mimic");
        assert_eq!(m.joint, "elbow");
        assert!((m.multiplier.unwrap_or(0.0) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_p5_urdf_export_helical_degrades() {
        let e = jsx_to_urdf(
            r#"export default <joint name="screw" type="helical" axis={[0,0,1]} pitch={0.05} q={1} limit={[-2, 2]}><sphere r={0.1} /></joint>;"#,
            None,
            "",
            "bot",
                0.08,
        )
        .unwrap();
        assert!(
            e.xml.contains("<joint name=\"screw\" type=\"revolute\">"),
            "{}",
            e.xml
        );
        assert!(
            e.xml
                .contains("<joint name=\"screw_slide\" type=\"prismatic\">"),
            "{}",
            e.xml
        );
        assert!(
            e.xml
                .contains("<mimic joint=\"screw\" multiplier=\"0.05\" offset=\"0\"/>"),
            "{}",
            e.xml
        );
    }

    #[test]
    fn test_p5_urdf_export_csg_bakes_to_stl() {
        // 非图元（解析 CSG）链接几何：导出方向三角化成 STL，文件写进 UrdfExport.meshes
        let e = jsx_to_urdf(
            r#"export default <joint name="j" type="fixed"><difference><box s={[0.8,0.8,0.8]} /><sphere r={0.3} /></difference></joint>;"#,
            None,
            "",
            "bot",
            0.08,
        )
        .unwrap();
        assert!(
            e.xml.contains("<mesh filename=\"meshes/j_link_0.stl\"/>"),
            "{}",
            e.xml
        );
        assert_eq!(e.meshes.len(), 1);
        let (name, data) = &e.meshes[0];
        assert_eq!(name, "meshes/j_link_0.stl");
        assert!(data.len() > 84, "二进制 STL 至少有头 + 面数");
        let n = u32::from_le_bytes([data[80], data[81], data[82], data[83]]) as usize;
        assert!(n > 10, "应有多面：{n}");
        assert_eq!(data.len(), 84 + n * 50, "二进制 STL 尺寸 = 84 + 50×面数");
    }

    #[test]
    fn test_p5_urdf_export_needs_limit() {
        let e = jsx_to_urdf(
            r#"export default <joint name="j" type="revolute"><sphere r={0.1} /></joint>;"#,
            None,
            "",
            "bot",
            0.08,
        )
        .unwrap_err();
        assert_eq!(e, "urdf: joint j needs limit= for URDF export");
    }
}
