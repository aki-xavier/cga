// Geometry types: CGA-blade primitives (sphere / plane / cylinder / box /
// circle) plus their per-frame camera-space parameters.  `geom_to_camera`
// computes the parameters (CPU-side versor conjugation); the per-pixel
// intersection kernels live in geometry_ops.v.

use crate::affine_geom::{AffineGeometry, AffineParams};
use crate::csg_node::{CsgGeometry, CsgParams};
use crate::primitives::{circle, cylinder, plane, sphere, Cylinder};
use crate::{Mat3, Multivector};

// --- per-geometry camera-space parameters ------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct SphereParams {
    pub c: [f64; 3],
    pub r: f64,
    pub axes: [[f64; 3]; 3],
}

#[derive(Clone, Copy, Debug)]
pub struct PlaneParams {
    pub n: [f64; 3],
    pub d: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct CylinderParams {
    pub q: [f64; 3],
    pub u: [f64; 3],
    pub r: f64,
    pub h: f64, // half-length; -1.0 means infinite cylinder
}

#[derive(Clone, Copy, Debug)]
pub struct BoxParams {
    pub c: [f64; 3],
    pub axes: [[f64; 3]; 3],
    pub half: [f64; 3],
}

#[derive(Clone, Copy, Debug)]
pub struct CircleParams {
    pub c: [f64; 3],
    pub n: [f64; 3],
    pub r: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct ConeParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub r: f64,
    pub h: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct TorusParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub major: f64,
    pub minor: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct EllipsoidParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
}

#[derive(Clone, Copy, Debug)]
pub struct CyclideParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub a: f64,
    pub b: f64,
    pub d: f64,
    pub c: f64,
    pub shift: [f64; 3],
}

#[derive(Clone, Debug)]
pub struct TrimeshParams {
    pub a_inv3: Mat3,
    pub t_inv: [f64; 3],
    pub a_fwd: [f64; 16],
    pub v0: Vec<[f64; 3]>,
    pub e1: Vec<[f64; 3]>,
    pub e2: Vec<[f64; 3]>,
    pub nrm: Vec<[f64; 3]>,
    pub lo: [f64; 3],
    pub hi: [f64; 3],
}

// GeometryParams is the sum of all camera-space parameter variants.
#[derive(Clone, Debug)]
pub enum GeometryParams {
    AffineParams(AffineParams),
    CsgParams(CsgParams),
    TrimeshParams(TrimeshParams),
    CircleParams(CircleParams),
    ConeParams(ConeParams),
    CyclideParams(CyclideParams),
    EllipsoidParams(EllipsoidParams),
    TorusParams(TorusParams),
    BoxParams(BoxParams),
    CylinderParams(CylinderParams),
    PlaneParams(PlaneParams),
    SphereParams(SphereParams),
}

// --- geometry structs --------------------------------------------------------

#[derive(Clone, Copy, Debug)]
pub struct SphereGeometry {
    pub radius: f64,
    pub blade: Multivector,
}

pub fn sphere_geometry(radius: f64) -> SphereGeometry {
    if radius <= 0.0 {
        panic!("sphere radius must be > 0, got {}", radius);
    }
    SphereGeometry {
        radius,
        blade: sphere([0.0, 0.0, 0.0], radius),
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PlaneGeometry {
    pub blade: Multivector,
}

pub fn plane_geometry(normal: [f64; 3], distance: f64) -> PlaneGeometry {
    PlaneGeometry {
        blade: plane(normal, distance),
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CylinderGeometry {
    pub radius: f64,
    pub half: f64, // -1.0 = infinite
    pub blade: Cylinder,
}

pub fn cylinder_geometry(radius: f64, length: f64) -> CylinderGeometry {
    if radius <= 0.0 {
        panic!("cylinder radius must be > 0, got {}", radius);
    }
    if length > 0.0 {
        return CylinderGeometry {
            radius,
            half: length / 2.0,
            blade: cylinder([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], radius),
        };
    }
    CylinderGeometry {
        radius,
        half: -1.0,
        blade: cylinder([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], radius),
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BoxGeometry {
    pub half: [f64; 3],
}

pub fn box_geometry(width: f64, height: f64, depth: f64) -> BoxGeometry {
    if width.min(height.min(depth)) <= 0.0 {
        panic!(
            "box dimensions must be > 0, got ({}, {}, {})",
            width, height, depth
        );
    }
    BoxGeometry {
        half: [width / 2.0, height / 2.0, depth / 2.0],
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CircleGeometry {
    pub radius: f64,
    pub blade: Multivector,
}

pub fn circle_geometry(radius: f64) -> CircleGeometry {
    if radius <= 0.0 {
        panic!("circle radius must be > 0, got {}", radius);
    }
    CircleGeometry {
        radius,
        blade: circle([0.0, 0.0, 0.0], radius, [0.0, 0.0, 1.0]),
    }
}

// Geometry is the sum type of all renderable geometries.
#[derive(Clone, Debug)]
pub enum Geometry {
    AffineGeometry(AffineGeometry),
    CsgGeometry(CsgGeometry),
    TrimeshGeometry(TrimeshGeometry),
    ConeGeometry(ConeGeometry),
    CyclideGeometry(CyclideGeometry),
    EllipsoidGeometry(EllipsoidGeometry),
    TorusGeometry(TorusGeometry),
    SphereGeometry(SphereGeometry),
    PlaneGeometry(PlaneGeometry),
    CylinderGeometry(CylinderGeometry),
    BoxGeometry(BoxGeometry),
    CircleGeometry(CircleGeometry),
}

// TriUvs is one triangle's three UV pairs (u0,u1,u2), as scalars (fixed-array
// struct fields trip a V 0.5.2 codegen bug).
#[derive(Clone, Copy, Debug)]
pub struct TriUvs {
    pub u0x: f64,
    pub u0y: f64,
    pub u1x: f64,
    pub u1y: f64,
    pub u2x: f64,
    pub u2y: f64,
}

#[derive(Clone, Debug)]
pub struct TrimeshGeometry {
    pub n_faces: i32,
    pub v0: Vec<[f64; 3]>,
    pub e1: Vec<[f64; 3]>,
    pub e2: Vec<[f64; 3]>,
    pub nrm: Vec<[f64; 3]>,
    pub uv: Vec<TriUvs>, // per-face; empty when the mesh has no UVs
    pub lo: [f64; 3],
    pub hi: [f64; 3],
}

// trimesh_geometry builds a triangle mesh (flat shading, no BVH).
pub fn trimesh_geometry(vertices: &[[f64; 3]], faces: &[[i32; 3]]) -> TrimeshGeometry {
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

// trimesh_geometry_uv is trimesh_geometry but also carries per-vertex UVs
// (parallel to `vertices`) as per-face UV triples for texture sampling.
pub fn trimesh_geometry_uv(
    vertices: &[[f64; 3]],
    faces: &[[i32; 3]],
    uvs: &[[f64; 2]],
) -> TrimeshGeometry {
    let g = trimesh_geometry(vertices, faces);
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

#[derive(Clone, Copy, Debug)]
pub struct ConeGeometry {
    pub radius: f64,
    pub height: f64,
}

pub fn cone_geometry(radius: f64, height: f64) -> ConeGeometry {
    if radius <= 0.0 || height <= 0.0 {
        panic!(
            "cone radius/height must be > 0, got ({}, {})",
            radius, height
        );
    }
    ConeGeometry { radius, height }
}

#[derive(Clone, Copy, Debug)]
pub struct TorusGeometry {
    pub major: f64,
    pub minor: f64,
}

pub fn torus_geometry(major: f64, minor: f64) -> TorusGeometry {
    if major <= 0.0 || minor <= 0.0 {
        panic!("torus radii must be > 0, got ({}, {})", major, minor);
    }
    TorusGeometry { major, minor }
}

#[derive(Clone, Copy, Debug)]
pub struct EllipsoidGeometry {
    pub radii: [f64; 3],
}

pub fn ellipsoid_geometry(rx: f64, ry: f64, rz: f64) -> EllipsoidGeometry {
    if rx.min(ry.min(rz)) <= 0.0 {
        panic!("ellipsoid radii must be > 0, got ({}, {}, {})", rx, ry, rz);
    }
    EllipsoidGeometry {
        radii: [rx, ry, rz],
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CyclideGeometry {
    pub a: f64,
    pub b: f64,
    pub d: f64,
    pub shift: [f64; 3],
}

pub fn cyclide_geometry(a: f64, b: f64, d: f64, shift: [f64; 3]) -> CyclideGeometry {
    if !(a > b && b > 0.0) {
        panic!("cyclide needs a > b > 0, got ({}, {})", a, b);
    }
    if d <= 0.0 {
        panic!("cyclide needs d > 0, got {}", d);
    }
    CyclideGeometry { a, b, d, shift }
}

// geom_to_camera conjugates a geometry's blade into camera space and returns
// the camera-space parameters.

pub fn vec3_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
