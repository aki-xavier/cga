use super::*;
use crate::{BezierPatchGeometry, TrimeshGeometry, TrimeshParams};

#[derive(Clone, Debug)]
pub struct BezierParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub v0: Vec<[f64; 3]>,
    pub e1: Vec<[f64; 3]>,
    pub e2: Vec<[f64; 3]>,
    pub nrm: Vec<[f64; 3]>,
    pub lo: [f64; 3],
    pub hi: [f64; 3],
    pub thickness: f64,
}
impl BezierParams {
    pub fn tessellated(g: &BezierPatchGeometry) -> BezierParams {
        let eye: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let (verts, faces) = g.tessellate();
        let tri = TrimeshGeometry::new(&verts, &faces);
        let b = g.control_bounds();
        BezierParams {
            a_inv3: eye,
            t_inv: [0.0; 3],
            a_fwd: [
                1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            ],
            v0: tri.v0,
            e1: tri.e1,
            e2: tri.e2,
            nrm: tri.nrm,
            lo: b[0],
            hi: b[1],
            thickness: g.thickness,
        }
    }
    pub fn to_trimesh_params(&self) -> TrimeshParams {
        TrimeshParams {
            a_inv3: self.a_inv3,
            t_inv: self.t_inv,
            a_fwd: self.a_fwd,
            v0: self.v0.clone(),
            e1: self.e1.clone(),
            e2: self.e2.clone(),
            nrm: self.nrm.clone(),
            lo: self.lo,
            hi: self.hi,
        }
    }
}
