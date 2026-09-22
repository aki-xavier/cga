pub fn mat4_identity() -> [f64; 16] {
    [
        1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    ]
}

pub fn mat4_mul(a: [f64; 16], b: [f64; 16]) -> [f64; 16] {
    let mut r = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a[i * 4 + k] * b[k * 4 + j];
            }
            r[i * 4 + j] = s;
        }
    }
    r
}

pub fn transform_point(m: [f64; 16], p: [f64; 3]) -> [f64; 3] {
    [
        m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3],
        m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7],
        m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11],
    ]
}

pub fn from_trs(translation: [f64; 3], rotation: [f64; 4], scale: [f64; 3]) -> [f64; 16] {
    let tx = translation[0];
    let ty = translation[1];
    let tz = translation[2];
    let mut qx = rotation[0];
    let mut qy = rotation[1];
    let mut qz = rotation[2];
    let mut qw = rotation[3];
    let sx = scale[0];
    let sy = scale[1];
    let sz = scale[2];
    let n = (qx * qx + qy * qy + qz * qz + qw * qw).sqrt();
    if n < 1e-12 {
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
        qw = 1.0;
    } else {
        qx /= n;
        qy /= n;
        qz /= n;
        qw /= n;
    }
    let r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    let r01 = 2.0 * (qx * qy - qz * qw);
    let r02 = 2.0 * (qx * qz + qy * qw);
    let r10 = 2.0 * (qx * qy + qz * qw);
    let r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    let r12 = 2.0 * (qy * qz - qx * qw);
    let r20 = 2.0 * (qx * qz - qy * qw);
    let r21 = 2.0 * (qy * qz + qx * qw);
    let r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
    [
        r00 * sx,
        r01 * sy,
        r02 * sz,
        tx,
        r10 * sx,
        r11 * sy,
        r12 * sz,
        ty,
        r20 * sx,
        r21 * sy,
        r22 * sz,
        tz,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

pub fn to_column_major(m: [f64; 16]) -> [f64; 16] {
    let mut r = [0.0; 16];
    for j in 0..4 {
        for i in 0..4 {
            r[j * 4 + i] = m[i * 4 + j];
        }
    }
    r
}

pub fn from_column_major(vals: [f64; 16]) -> [f64; 16] {
    let mut r = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            r[i * 4 + j] = vals[j * 4 + i];
        }
    }
    r
}

#[derive(Clone, Default, Debug)]
pub struct ObjMesh {
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<[i32; 3]>,
    pub has_transform: bool,
    pub transform: [f64; 16],
}

fn resolve_obj_index(ref_: &str, vertex_count: i32, lineno: i32) -> Result<i32, String> {
    if ref_.is_empty() {
        return Err(format!("line {lineno}: empty face vertex reference"));
    }
    let raw: i32 = ref_
        .parse()
        .map_err(|_| format!("line {lineno}: illegal vertex index \"{ref_}\""))?;
    if raw == 0 {
        return Err(format!(
            "line {lineno}: OBJ indices are 1-based, 0 not allowed"
        ));
    }
    let idx = if raw < 0 { vertex_count + raw } else { raw - 1 };
    if idx < 0 || idx >= vertex_count {
        return Err(format!(
            "line {lineno}: vertex index {raw} out of range ({vertex_count} vertices)"
        ));
    }
    Ok(idx)
}

pub type ObjData = (Vec<[f64; 3]>, Vec<[i32; 3]>);

pub fn load_obj(path: &str) -> Result<ObjData, String> {
    let text = std::fs::read_to_string(path).map_err(|_| format!("cannot read {path}"))?;
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut faces: Vec<[i32; 3]> = Vec::new();
    for (lineno, raw_line) in text.lines().enumerate() {
        let trimmed = raw_line.split('#').next().unwrap().trim();
        if trimmed.is_empty() {
            continue;
        }
        let parts: Vec<&str> = trimmed.split(' ').collect();
        if parts.is_empty() {
            continue;
        }
        let keyword = parts[0];
        let fields = &parts[1..];
        if keyword == "v" {
            if fields.len() < 3 {
                return Err(format!(
                    "line {}: v needs at least 3 coordinates",
                    lineno + 1
                ));
            }
            let x: f64 = fields[0]
                .parse()
                .map_err(|_| "bad coordinate".to_string())?;
            let y: f64 = fields[1]
                .parse()
                .map_err(|_| "bad coordinate".to_string())?;
            let z: f64 = fields[2]
                .parse()
                .map_err(|_| "bad coordinate".to_string())?;
            vertices.push([x, y, z]);
        } else if keyword == "f" {
            let mut indices: Vec<i32> = Vec::new();
            for tok in fields {
                let ref_ = tok.split('/').next().unwrap();
                indices.push(resolve_obj_index(
                    ref_,
                    vertices.len() as i32,
                    lineno as i32 + 1,
                )?);
            }
            if indices.len() < 3 {
                return Err(format!("line {}: face with < 3 vertices", lineno + 1));
            }
            for i in 1..indices.len() - 1 {
                faces.push([indices[0], indices[i], indices[i + 1]]);
            }
        }
    }
    Ok((vertices, faces))
}

fn mesh_vertex_world(m: &ObjMesh, p: [f64; 3]) -> [f64; 3] {
    if m.has_transform {
        return transform_point(m.transform, p);
    }
    p
}

fn format_g9(v: f64) -> String {
    const P: i32 = 9;
    if v == 0.0 {
        return "0".to_string();
    }
    let sci = format!("{:.*e}", (P - 1) as usize, v);
    let epos = sci.find('e').unwrap();
    let exp: i32 = sci[epos + 1..].parse().unwrap();
    if !(-4..P).contains(&exp) {
        let mantissa = sci[..epos].trim_end_matches('0').trim_end_matches('.');
        format!(
            "{}e{}{:02}",
            mantissa,
            if exp < 0 { "-" } else { "+" },
            exp.abs()
        )
    } else {
        let decimals = (P - 1 - exp).max(0) as usize;
        let s = format!("{:.*}", decimals, v);
        let s = if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        } else {
            s
        };
        if s == "-0" {
            "0".to_string()
        } else {
            s
        }
    }
}

pub fn save_obj(path: &str, meshes: &[ObjMesh]) {
    let mut lines = vec!["# generated by cga.mesh_io".to_string()];
    let mut offset: i32 = 0;
    for (mi, m) in meshes.iter().enumerate() {
        lines.push(format!("o mesh_{mi}"));
        for p in &m.vertices {
            let q = mesh_vertex_world(m, *p);
            lines.push(format!(
                "v {} {} {}",
                format_g9(q[0]),
                format_g9(q[1]),
                format_g9(q[2])
            ));
        }
        for f in &m.faces {
            lines.push(format!(
                "f {} {} {}",
                f[0] + 1 + offset,
                f[1] + 1 + offset,
                f[2] + 1 + offset
            ));
        }
        offset += m.vertices.len() as i32;
    }
    std::fs::write(path, lines.join("\n") + "\n").unwrap_or_else(|_| panic!("cannot write {path}"));
}
