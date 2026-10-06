//! URDF interop (关节化 P5/P6（调研文档已随完成退役）).
//!
//! Export: the CGS joint tree maps 1:1 onto URDF (origin xyz/rpy ↔ at/rpy,
//! axis in the joint frame, limit, gear → mimic). helical/cylindrical/
//! spherical decompose into 1-DOF series joints (documented downgrade).
//! Link geometry: a single native primitive (sphere/box/cylinder) exports as
//! URDF geometry; anything else bakes to a watertight OBJ.
//!
//! Import: urdf-rs parses, JSX text is generated `jsx_gen`-style (flat,
//! deterministic, parseable by construction).

use cga_core::GeometryParams;
use urdf_rs::{Geometry as UGeometry, JointType, Robot};

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
    /// (relative filename, OBJ text) for baked links.
    pub meshes: Vec<(String, String)>,
}

/// One mesh's visual: native URDF geometry or a baked OBJ reference.
struct Visual {
    origin_xyz: [f64; 3],
    origin_rpy: [f64; 3],
    geom: String,
    mesh_file: Option<String>,
    mesh_obj: Option<String>,
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
            mesh_obj: None,
        });
    }
    // Bake path: watertight OBJ in the link frame.
    let baked = p
        .bake(step)
        .map_err(|e| format!("urdf: bake failed for link {link}: {e}"))?;
    let mut obj = String::new();
    for v in &baked.vertices {
        let lv = xform_point(inv, *v);
        obj.push_str(&format!("v {} {} {}\n", uf(lv[0]), uf(lv[1]), uf(lv[2])));
    }
    for f in &baked.faces {
        obj.push_str(&format!("f {} {} {}\n", f[0] + 1, f[1] + 1, f[2] + 1));
    }
    let file = format!("meshes/{link}_{slot}.obj");
    Ok(Visual {
        origin_xyz: [0.0; 3],
        origin_rpy: [0.0; 3],
        geom: format!("<mesh filename=\"{file}\"/>"),
        mesh_file: Some(file),
        mesh_obj: Some(obj),
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

/// Export a JSX scene's joint tree as URDF. `step` is the bake step for
/// non-primitive link geometry.
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
    let mut meshes: Vec<(String, String)> = Vec::new();

    let emit_link = |name: &str,
                     mesh_ids: &[usize],
                     link_world: [f64; 16],
                     meshes: &mut Vec<(String, String)>,
                     xml: &mut String|
     -> Result<(), String> {
        xml.push_str(&format!("  <link name=\"{name}\">\n"));
        for (slot, &mi) in mesh_ids.iter().enumerate() {
            let v = visual_of(&run, mi, link_world, name, slot, step)?;
            xml.push_str("    <visual>\n");
            xml.push_str(&format!(
                "      <origin xyz=\"{}\" rpy=\"{}\"/>\n",
                uv(v.origin_xyz),
                uv(v.origin_rpy)
            ));
            xml.push_str(&format!("      <geometry>{}\n", v.geom));
            xml.push_str("      </geometry>\n    </visual>\n");
            if let (Some(f), Some(o)) = (v.mesh_file, v.mesh_obj) {
                meshes.push((f, o));
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
                // Series Rx → Ry → Rz, matching the CGS spherical motion.
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

/// Import URDF as JSX scene text (flat, deterministic, `jsx_gen` style).
/// Mesh references keep their basename under `mesh_prefix`.
pub fn urdf_to_jsx(xml: &str, mesh_prefix: &str) -> Result<String, String> {
    let robot: Robot =
        urdf_rs::read_from_string(xml).map_err(|e| format!("urdf: parse failed: {e}"))?;
    let mut out = String::new();
    out.push_str(&format!("// from URDF robot \"{}\"\n", robot.name));
    out.push_str("export default (\n  <scene>\n");

    // Topological order: joints whose parent link is already emitted.
    let mut emitted_links: Vec<String> = Vec::new();
    let child_links: std::collections::BTreeSet<String> =
        robot.joints.iter().map(|j| j.child.link.clone()).collect();
    let mut queue: Vec<String> = robot
        .links
        .iter()
        .map(|l| l.name.clone())
        .filter(|n| !child_links.contains(n))
        .collect();
    let mut remaining: Vec<&urdf_rs::Joint> = robot.joints.iter().collect();
    let mut body = String::new();
    while !remaining.is_empty() {
        let mut progress = false;
        let mut next: Vec<&urdf_rs::Joint> = Vec::new();
        for j in remaining {
            if !queue.contains(&j.parent.link) {
                next.push(j);
                continue;
            }
            progress = true;
            write_joint(&mut body, &robot, j)?;
            emitted_links.push(j.child.link.clone());
            queue.push(j.child.link.clone());
        }
        if !progress {
            return Err(format!(
                "urdf: joint tree has a cycle or a mimic-order problem at {}",
                next[0].name
            ));
        }
        remaining = next;
    }
    // Root-link visuals at top level.
    for l in &robot.links {
        if !child_links.contains(&l.name) {
            write_visuals(&mut body, l, mesh_prefix)?;
        }
    }
    out.push_str(&body);
    out.push_str("  </scene>\n);\n");
    Ok(out)
}

fn write_visuals(out: &mut String, link: &urdf_rs::Link, mesh_prefix: &str) -> Result<(), String> {
    for v in &link.visual {
        let xyz = [v.origin.xyz[0], v.origin.xyz[1], v.origin.xyz[2]];
        let rpy = [v.origin.rpy[0], v.origin.rpy[1], v.origin.rpy[2]];
        let mut prefix = String::new();
        let mut suffix = String::new();
        if xyz != [0.0; 3] {
            prefix.push_str(&format!(
                "<translate t={{[{}]}}>",
                uv(xyz).replace(' ', ", ")
            ));
            suffix.push_str("</translate>");
        }
        if rpy != [0.0; 3] {
            let (ax, ang) = rpy_axis_angle(rpy);
            if ang.abs() > 1e-12 {
                prefix.push_str(&format!(
                    "<rotate axis={{[{}]}} angle={{ {} }}>",
                    uv(ax).replace(' ', ", "),
                    uf(ang)
                ));
                suffix = format!("</rotate>{suffix}");
            }
        }
        let stmt = match &v.geometry {
            UGeometry::Box { size } => format!(
                "<box s={{[{}, {}, {}]}} />",
                uf(size[0]),
                uf(size[1]),
                uf(size[2])
            ),
            UGeometry::Cylinder { radius, length } => {
                format!(
                    "<cylinder r={{ {} }} h={{ {} }} />",
                    uf(*radius),
                    uf(*length)
                )
            }
            UGeometry::Sphere { radius } => format!("<sphere r={{ {} }} />", uf(*radius)),
            UGeometry::Mesh { filename, .. } => {
                let base = filename.rsplit('/').next().unwrap_or(filename);
                format!("<mesh file=\"{mesh_prefix}{base}\" />")
            }
            _ => return Err("urdf: unsupported visual geometry".to_string()),
        };
        out.push_str(&format!("    {prefix}{stmt}{suffix}\n"));
    }
    Ok(())
}

fn rpy_axis_angle(rpy: [f64; 3]) -> ([f64; 3], f64) {
    let (cr, sr) = (rpy[0].cos(), rpy[0].sin());
    let (cp, sp) = (rpy[1].cos(), rpy[1].sin());
    let (cy, sy) = (rpy[2].cos(), rpy[2].sin());
    // R = Rz(y)·Ry(p)·Rx(r), row-major 3×3.
    let r = [
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        -sp,
        cp * sr,
        cp * cr,
    ];
    let tr = r[0] + r[4] + r[8];
    let ang = ((tr - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
    if ang.abs() < 1e-12 {
        return ([0.0, 0.0, 1.0], 0.0);
    }
    let s = 2.0 * ang.sin();
    (
        [(r[7] - r[5]) / s, (r[2] - r[6]) / s, (r[3] - r[1]) / s],
        ang,
    )
}

fn write_joint(out: &mut String, robot: &Robot, j: &urdf_rs::Joint) -> Result<(), String> {
    let ty = match j.joint_type {
        JointType::Revolute => "revolute",
        JointType::Continuous => "continuous",
        JointType::Prismatic => "prismatic",
        JointType::Fixed => "fixed",
        JointType::Planar => "planar",
        _ => return Err(format!("urdf: floating joint {} is not supported", j.name)),
    };
    let at = [j.origin.xyz[0], j.origin.xyz[1], j.origin.xyz[2]];
    let rpy = [j.origin.rpy[0], j.origin.rpy[1], j.origin.rpy[2]];
    let axis = [j.axis.xyz[0], j.axis.xyz[1], j.axis.xyz[2]];
    // mimic → gear：紧随被驱动关节之前（JSX 走文档序，driver 已先于它发射）。
    if let Some(m) = &j.mimic {
        out.push_str(&format!(
            "    <gear driver=\"{}\" driven=\"{}\" ratio={{ {} }} offset={{ {} }} />\n",
            m.joint,
            j.name,
            uf(m.multiplier.unwrap_or(1.0)),
            uf(m.offset.unwrap_or(0.0))
        ));
    }
    let mut args = format!(
        "name=\"{}\" type=\"{}\" at={{[{}]}} rpy={{[{}]}}",
        j.name,
        ty,
        uv(at).replace(' ', ", "),
        uv(rpy).replace(' ', ", ")
    );
    if ty != "fixed" {
        args.push_str(&format!(" axis={{[{}]}}", uv(axis).replace(' ', ", ")));
    }
    if ty == "revolute" || ty == "prismatic" {
        args.push_str(&format!(
            " limit={{[{}, {}]}}",
            uf(j.limit.lower),
            uf(j.limit.upper)
        ));
    }
    out.push_str(&format!("    <joint {args}>\n"));
    let link = robot
        .links
        .iter()
        .find(|l| l.name == j.child.link)
        .ok_or_else(|| format!("urdf: unknown link {}", j.child.link))?;
    write_visuals(out, link, "")?;
    out.push_str("    </joint>\n");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let e = jsx_to_urdf(ARM, None, "", "arm", 0.05).unwrap();
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
        assert!(e.meshes.is_empty(), "全原生几何，无烘焙");
    }

    #[test]
    fn test_p5_urdf_export_roundtrip_urdf_rs() {
        let e = jsx_to_urdf(ARM, None, "", "arm", 0.05).unwrap();
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
            0.05,
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
    fn test_p5_urdf_export_csg_bakes() {
        let e = jsx_to_urdf(
            r#"export default <joint name="j" type="fixed"><difference><box s={[0.8,0.8,0.8]} /><sphere r={0.3} /></difference></joint>;"#,
            None,
            "",
            "bot",
            0.1,
        )
        .unwrap();
        assert!(
            e.xml.contains("<mesh filename=\"meshes/j_link_0.obj\"/>"),
            "{}",
            e.xml
        );
        assert_eq!(e.meshes.len(), 1);
        let (name, obj) = &e.meshes[0];
        assert_eq!(name, "meshes/j_link_0.obj");
        assert!(obj.starts_with("v "), "OBJ 应有顶点");
        assert!(obj.contains("\nf "), "OBJ 应有面");
    }

    #[test]
    fn test_p5_urdf_export_needs_limit() {
        let e = jsx_to_urdf(
            r#"export default <joint name="j" type="revolute"><sphere r={0.1} /></joint>;"#,
            None,
            "",
            "bot",
            0.05,
        )
        .unwrap_err();
        assert_eq!(e, "urdf: joint j needs limit= for URDF export");
    }

    #[test]
    fn test_p6_urdf_import_roundtrip() {
        let xml = r#"<robot name="two_link">
  <link name="base"/>
  <link name="upper">
    <visual><origin xyz="0 0 0.2" rpy="0 0 0"/><geometry><cylinder radius="0.05" length="0.4"/></geometry></visual>
  </link>
  <joint name="hip" type="revolute">
    <parent link="base"/><child link="upper"/>
    <origin xyz="0.1 0 0.3" rpy="0 0 1.570796"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="1"/>
  </joint>
</robot>"#;
        let jsx = urdf_to_jsx(xml, "").unwrap();
        assert!(
            jsx.contains("<joint name=\"hip\" type=\"revolute\""),
            "{jsx}"
        );
        assert!(jsx.contains("at={[0.1, 0, 0.3]}"), "{jsx}");
        assert!(jsx.contains("limit={[-1.5, 1.5]}"), "{jsx}");
        assert!(jsx.contains("<cylinder r={ 0.05 } h={ 0.4 } />"), "{jsx}");
        // 往返：生成的 JSX 可运行，关节树一致。
        let run = run_jsx(&jsx, None, "").unwrap();
        assert_eq!(run.kinematics.joints.len(), 1);
        let j = &run.kinematics.joints[0];
        assert_eq!(j.name, "hip");
        assert_eq!(j.kind, JointKind::Revolute);
        assert!((j.at[0] - 0.1).abs() < 1e-9 && (j.at[2] - 0.3).abs() < 1e-9);
        assert!((j.rpy[2] - 1.570796).abs() < 1e-6);
        assert_eq!(j.limit, Some([-1.5, 1.5]));
        assert_eq!(run.scene.objects.len(), 1, "圆柱体应进场景");
    }

    #[test]
    fn test_p6_urdf_import_floating_rejected() {
        let xml = r#"<robot name="x">
  <link name="a"/><link name="b"/>
  <joint name="j" type="floating"><parent link="a"/><child link="b"/></joint>
</robot>"#;
        assert_eq!(
            urdf_to_jsx(xml, "").unwrap_err(),
            "urdf: floating joint j is not supported"
        );
    }
}
