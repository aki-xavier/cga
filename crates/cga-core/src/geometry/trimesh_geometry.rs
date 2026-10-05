use super::*;

#[derive(Clone, Debug)]
pub struct TrimeshGeometry {
    pub n_faces: i32,
    pub v0: Vec<[f64; 3]>,
    pub e1: Vec<[f64; 3]>,
    pub e2: Vec<[f64; 3]>,
    pub nrm: Vec<[f64; 3]>,
    pub uv: Vec<TriUvs>,
    pub lo: [f64; 3],
    pub hi: [f64; 3],
}
impl TrimeshGeometry {
    pub fn new(vertices: &[[f64; 3]], faces: &[[i32; 3]]) -> TrimeshGeometry {
        if faces.is_empty() {
            panic!("trimesh needs >= 1 face");
        }
        let nv = vertices.len() as i32;
        let mut v0: Vec<[f64; 3]> = Vec::new();
        let mut edge1: Vec<[f64; 3]> = Vec::new();
        let mut edge2: Vec<[f64; 3]> = Vec::new();
        let mut nrm: Vec<[f64; 3]> = Vec::new();
        let mut bmin = [1e30, 1e30, 1e30];
        let mut bmax = [-1e30, -1e30, -1e30];
        for v in vertices {
            for k in 0..3 {
                if v[k] < bmin[k] {
                    bmin[k] = v[k];
                }
                if v[k] > bmax[k] {
                    bmax[k] = v[k];
                }
            }
        }
        for f in faces {
            if f[0] < 0 || f[1] < 0 || f[2] < 0 || f[0] >= nv || f[1] >= nv || f[2] >= nv {
                panic!("bad face {:?} (vertices={})", f, nv);
            }
            let a = vertices[f[0] as usize];
            let b = vertices[f[1] as usize];
            let c = vertices[f[2] as usize];
            let ee1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let ee2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let cr = vec3_cross(ee1, ee2);
            let cl = (cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]).sqrt();
            if cl < 1e-12 {
                panic!("trimesh has degenerate (zero-area) faces");
            }
            v0.push(a);
            edge1.push(ee1);
            edge2.push(ee2);
            nrm.push([cr[0] / cl, cr[1] / cl, cr[2] / cl]);
        }
        TrimeshGeometry {
            n_faces: faces.len() as i32,
            v0,
            e1: edge1,
            e2: edge2,
            nrm,
            uv: Vec::new(),
            lo: bmin,
            hi: bmax,
        }
    }
}
impl TrimeshGeometry {
    pub fn with_uv(vertices: &[[f64; 3]], faces: &[[i32; 3]], uvs: &[[f64; 2]]) -> TrimeshGeometry {
        let g = Self::new(vertices, faces);
        if uvs.is_empty() {
            return g;
        }
        if uvs.len() != vertices.len() {
            panic!("uv count {} != vertex count {}", uvs.len(), vertices.len());
        }
        let mut tri = vec![
            TriUvs {
                u0x: 0.0,
                u0y: 0.0,
                u1x: 0.0,
                u1y: 0.0,
                u2x: 0.0,
                u2y: 0.0,
            };
            faces.len()
        ];
        for (i, f) in faces.iter().enumerate() {
            let a = uvs[f[0] as usize];
            let b = uvs[f[1] as usize];
            let c = uvs[f[2] as usize];
            tri[i] = TriUvs {
                u0x: a[0],
                u0y: a[1],
                u1x: b[0],
                u1y: b[1],
                u2x: c[0],
                u2y: c[1],
            };
        }
        TrimeshGeometry {
            n_faces: g.n_faces,
            v0: g.v0,
            e1: g.e1,
            e2: g.e2,
            nrm: g.nrm,
            uv: tri,
            lo: g.lo,
            hi: g.hi,
        }
    }
}
