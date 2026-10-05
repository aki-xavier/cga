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

pub type ObjData = (Vec<[f64; 3]>, Vec<[i32; 3]>);

pub fn load_obj(path: &str) -> Result<ObjData, String> {
    let mut f = std::fs::File::open(path).map_err(|_| format!("cannot read {path}"))?;
    let data = obj::ObjData::load_buf(&mut f).map_err(|e| format!("{path}: {e}"))?;
    let vertices: Vec<[f64; 3]> = data
        .position
        .iter()
        .map(|p| [f64::from(p[0]), f64::from(p[1]), f64::from(p[2])])
        .collect();
    let mut faces: Vec<[i32; 3]> = Vec::new();
    for object in &data.objects {
        for group in &object.groups {
            for poly in &group.polys {
                let idx: Vec<i32> = poly.0.iter().map(|t| t.0 as i32).collect();
                if idx.len() < 3 {
                    return Err(format!("{path}: face with < 3 vertices"));
                }
                for i in 1..idx.len() - 1 {
                    faces.push([idx[0], idx[i], idx[i + 1]]);
                }
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

pub fn save_obj(path: &str, meshes: &[ObjMesh]) {
    let mut data = obj::ObjData::default();
    let mut offset: i32 = 0;
    for (mi, m) in meshes.iter().enumerate() {
        let mut polys: Vec<obj::SimplePolygon> = Vec::new();
        for f in &m.faces {
            polys.push(obj::SimplePolygon(
                [f[0], f[1], f[2]]
                    .iter()
                    .map(|&i| obj::IndexTuple((i + offset) as usize, None, None))
                    .collect(),
            ));
        }
        for p in &m.vertices {
            let q = mesh_vertex_world(m, *p);
            data.position.push([q[0] as f32, q[1] as f32, q[2] as f32]);
        }
        let name = format!("mesh_{mi}");
        data.objects.push(obj::Object {
            name: name.clone(),
            groups: vec![obj::Group {
                name,
                index: 0,
                material: None,
                polys,
            }],
        });
        offset += m.vertices.len() as i32;
    }
    data.save(path)
        .unwrap_or_else(|_| panic!("cannot write {path}"));
}
