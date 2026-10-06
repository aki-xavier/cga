use crate::{
    AffineParams, BoxParams, ConeParams, CsgOp, CyclideParams, CylinderParams, EllipsoidParams,
    GeometryParams, PlaneParams, SphereParams, TorusParams,
};

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn to_local(a_inv3: [[f64; 3]; 3], t_inv: [f64; 3], p: [f64; 3]) -> [f64; 3] {
    [
        a_inv3[0][0] * p[0] + a_inv3[0][1] * p[1] + a_inv3[0][2] * p[2] + t_inv[0],
        a_inv3[1][0] * p[0] + a_inv3[1][1] * p[1] + a_inv3[1][2] * p[2] + t_inv[1],
        a_inv3[2][0] * p[0] + a_inv3[2][1] * p[1] + a_inv3[2][2] * p[2] + t_inv[2],
    ]
}

fn sphere_field(p: SphereParams, x: [f64; 3]) -> f64 {
    dot(sub(x, p.c), sub(x, p.c)) - p.r * p.r
}

fn plane_field(p: PlaneParams, x: [f64; 3]) -> f64 {
    dot(p.n, x) - p.d
}

fn cylinder_field(p: CylinderParams, x: [f64; 3]) -> f64 {
    let rel = sub(x, p.q);
    let s = dot(rel, p.u);
    let rad = sub(rel, [p.u[0] * s, p.u[1] * s, p.u[2] * s]);
    let f = dot(rad, rad) - p.r * p.r;
    if p.h < 0.0 {
        return f;
    }
    f.max(s.abs() - p.h)
}

fn box_field(p: BoxParams, x: [f64; 3]) -> f64 {
    let q = sub(x, p.c);
    let mut f = (dot(q, p.axes[0]).abs() - p.half[0]).max(dot(q, p.axes[1]).abs() - p.half[1]);
    f = f.max(dot(q, p.axes[2]).abs() - p.half[2]);
    f
}

fn cone_field(p: ConeParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    let s = l[2] - p.h / 2.0;
    let k2 = 1.0 + (p.r / p.h) * (p.r / p.h);
    let f = l[0] * l[0] + l[1] * l[1] + s * s - k2 * s * s;
    f.max(s).max(-(s + p.h))
}

fn ellipsoid_field(p: EllipsoidParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    dot(l, l) - 1.0
}

fn torus_field(p: TorusParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    let r2 = p.major * p.major;
    let f = (dot(l, l) + r2 - p.minor * p.minor).powi(2) - 4.0 * r2 * (l[0] * l[0] + l[1] * l[1]);
    if p.arc >= std::f64::consts::TAU {
        return f;
    }
    let mut theta = l[1].atan2(l[0]);
    if theta < 0.0 {
        theta += std::f64::consts::TAU;
    }
    f.max(theta - p.arc)
}

fn cyclide_field(p: CyclideParams, x: [f64; 3]) -> f64 {
    let l = to_local(p.a_inv3, p.t_inv, x);
    let x0 = l[0] - p.shift[0];
    let y0 = l[1] - p.shift[1];
    let z0 = l[2] - p.shift[2];
    let bb = p.b * p.b - p.d * p.d;
    let rho = x0 * x0 + y0 * y0 + z0 * z0;
    (rho + bb).powi(2) - 4.0 * (x0 * p.a - p.c * p.d).powi(2) - 4.0 * p.b * p.b * y0 * y0
}

fn affine_field(p: &AffineParams, x: [f64; 3]) -> f64 {
    p.inner.field(to_local(p.a_inv3, p.t_inv, x))
}

fn csg_field(op: CsgOp, children: &[GeometryParams], x: [f64; 3]) -> f64 {
    match op {
        CsgOp::Union => {
            let mut f = f64::INFINITY;
            for c in children {
                let v = c.field(x);
                if v < f {
                    f = v;
                }
            }
            f
        }
        CsgOp::Intersection => {
            let mut f = f64::NEG_INFINITY;
            for c in children {
                let v = c.field(x);
                if v > f {
                    f = v;
                }
            }
            f
        }
        CsgOp::Difference => {
            let mut f = children[0].field(x);
            for c in &children[1..] {
                let v = -c.field(x);
                if v > f {
                    f = v;
                }
            }
            f
        }
    }
}

impl GeometryParams {
    pub fn field(&self, x: [f64; 3]) -> f64 {
        match self {
            GeometryParams::SphereParams(p) => sphere_field(*p, x),
            GeometryParams::PlaneParams(p) => plane_field(*p, x),
            GeometryParams::CylinderParams(p) => cylinder_field(*p, x),
            GeometryParams::BoxParams(p) => box_field(*p, x),
            GeometryParams::ConeParams(p) => cone_field(*p, x),
            GeometryParams::EllipsoidParams(p) => ellipsoid_field(*p, x),
            GeometryParams::TorusParams(p) => torus_field(*p, x),
            GeometryParams::CyclideParams(p) => cyclide_field(*p, x),
            GeometryParams::AffineParams(p) => affine_field(p, x),
            GeometryParams::CsgParams(p) => csg_field(p.op, &p.children, x),
            GeometryParams::CircleParams(_) => panic!("circle is not a solid (no field)"),
        }
    }
}

fn apply_fwd(a_fwd: &[f64; 16], p: [f64; 3]) -> [f64; 3] {
    [
        a_fwd[0] * p[0] + a_fwd[1] * p[1] + a_fwd[2] * p[2] + a_fwd[3],
        a_fwd[4] * p[0] + a_fwd[5] * p[1] + a_fwd[6] * p[2] + a_fwd[7],
        a_fwd[8] * p[0] + a_fwd[9] * p[1] + a_fwd[10] * p[2] + a_fwd[11],
    ]
}

fn corners_bounds(lo: [f64; 3], hi: [f64; 3], a_fwd: &[f64; 16]) -> [[f64; 3]; 2] {
    let mut mn = [f64::INFINITY; 3];
    let mut mx = [f64::NEG_INFINITY; 3];
    for &i in &[lo[0], hi[0]] {
        for &j in &[lo[1], hi[1]] {
            for &k in &[lo[2], hi[2]] {
                let q = apply_fwd(a_fwd, [i, j, k]);
                for a in 0..3 {
                    if q[a] < mn[a] {
                        mn[a] = q[a];
                    }
                    if q[a] > mx[a] {
                        mx[a] = q[a];
                    }
                }
            }
        }
    }
    [mn, mx]
}

fn union_bounds(list: &[Option<[[f64; 3]; 2]>]) -> Option<[[f64; 3]; 2]> {
    let mut out: Option<[[f64; 3]; 2]> = None;
    for b in list.iter().flatten() {
        out = Some(match out {
            None => *b,
            Some([mn, mx]) => {
                let mut lo = mn;
                let mut hi = mx;
                for a in 0..3 {
                    if b[0][a] < lo[a] {
                        lo[a] = b[0][a];
                    }
                    if b[1][a] > hi[a] {
                        hi[a] = b[1][a];
                    }
                }
                [lo, hi]
            }
        });
    }
    out
}

fn intersect_bounds(list: &[Option<[[f64; 3]; 2]>]) -> Option<[[f64; 3]; 2]> {
    let mut out: Option<[[f64; 3]; 2]> = None;
    for b in list.iter().flatten() {
        out = Some(match out {
            None => *b,
            Some([mn, mx]) => {
                let mut lo = mn;
                let mut hi = mx;
                for a in 0..3 {
                    if b[0][a] > lo[a] {
                        lo[a] = b[0][a];
                    }
                    if b[1][a] < hi[a] {
                        hi[a] = b[1][a];
                    }
                }
                [lo, hi]
            }
        });
    }
    out
}

impl GeometryParams {
    pub fn bounds(&self) -> Option<[[f64; 3]; 2]> {
        match self {
            GeometryParams::SphereParams(p) => Some([
                [p.c[0] - p.r, p.c[1] - p.r, p.c[2] - p.r],
                [p.c[0] + p.r, p.c[1] + p.r, p.c[2] + p.r],
            ]),
            GeometryParams::PlaneParams(_) => None,
            GeometryParams::CylinderParams(p) => {
                if p.h < 0.0 {
                    return None;
                }
                let mut lo = [0.0; 3];
                let mut hi = [0.0; 3];
                for i in 0..3 {
                    let e = p.u[i].abs() * p.h + p.r;
                    lo[i] = p.q[i] - e;
                    hi[i] = p.q[i] + e;
                }
                Some([lo, hi])
            }
            GeometryParams::BoxParams(p) => {
                let mut lo = [0.0; 3];
                let mut hi = [0.0; 3];
                for i in 0..3 {
                    let mut e = 0.0;
                    for j in 0..3 {
                        e += p.axes[j][i].abs() * p.half[j];
                    }
                    lo[i] = p.c[i] - e;
                    hi[i] = p.c[i] + e;
                }
                Some([lo, hi])
            }
            GeometryParams::ConeParams(p) => Some(corners_bounds(
                [-p.r, -p.r, -p.h / 2.0],
                [p.r, p.r, p.h / 2.0],
                &p.a_fwd,
            )),
            GeometryParams::TorusParams(p) => {
                let e = p.major + p.minor;
                Some(corners_bounds(
                    [-e, -e, -p.minor],
                    [e, e, p.minor],
                    &p.a_fwd,
                ))
            }
            GeometryParams::EllipsoidParams(p) => {
                Some(corners_bounds([-1.0; 3], [1.0; 3], &p.a_fwd))
            }
            GeometryParams::CyclideParams(p) => {
                let r = p.d + p.c;
                Some(corners_bounds(
                    [p.shift[0] - p.a - r, p.shift[1] - p.b - r, p.shift[2] - r],
                    [p.shift[0] + p.a + r, p.shift[1] + p.b + r, p.shift[2] + r],
                    &p.a_fwd,
                ))
            }
            GeometryParams::CsgParams(p) => {
                let bs: Vec<_> = p.children.iter().map(|c| c.bounds()).collect();
                if p.op == CsgOp::Difference {
                    return bs.into_iter().next().unwrap_or(None);
                }
                if p.op == CsgOp::Union {
                    union_bounds(&bs)
                } else {
                    intersect_bounds(&bs)
                }
            }
            GeometryParams::AffineParams(p) => {
                let b = p.inner.bounds()?;
                Some(corners_bounds(b[0], b[1], &p.a_fwd))
            }
            GeometryParams::CircleParams(_) => None,
        }
    }
}

const CORN: [[usize; 3]; 8] = [
    [0, 0, 0],
    [1, 0, 0],
    [1, 1, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 1],
    [1, 1, 1],
    [0, 1, 1],
];

const TETS: [[usize; 4]; 6] = [
    [0, 1, 2, 6],
    [0, 1, 5, 6],
    [0, 3, 2, 6],
    [0, 3, 7, 6],
    [0, 4, 5, 6],
    [0, 4, 7, 6],
];

const TET_EDGES: [[usize; 2]; 6] = [[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]];

fn lerp_pt(a: [f64; 3], va: f64, b: [f64; 3], vb: f64) -> [f64; 3] {
    // Exact endpoint cases: a zero-valued grid node is a crossing shared by
    // every incident cell; returning the node bit-exactly (instead of
    // `a + 1.0 * (b - a)`, which is not bit-exact in f64) keeps the weld and
    // the edge-pairing topology check exact.
    if va == 0.0 {
        return a;
    }
    if vb == 0.0 {
        return b;
    }
    let t = va / (va - vb);
    [
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
    ]
}

fn grad(params: &GeometryParams, x: [f64; 3]) -> [f64; 3] {
    let e = 1e-6;
    [
        (params.field([x[0] + e, x[1], x[2]]) - params.field([x[0] - e, x[1], x[2]])) / (2.0 * e),
        (params.field([x[0], x[1] + e, x[2]]) - params.field([x[0], x[1] - e, x[2]])) / (2.0 * e),
        (params.field([x[0], x[1], x[2] + e]) - params.field([x[0], x[1], x[2] - e])) / (2.0 * e),
    ]
}

fn emit_tri(
    verts: &mut Vec<[f64; 3]>,
    faces: &mut Vec<[i32; 3]>,
    a: [f64; 3],
    b: [f64; 3],
    c: [f64; 3],
) {
    // Zero-area faces (the iso-surface passes exactly through a grid node and
    // several crossings collapse onto it) carry no area and no orientation;
    // drop them. The region they degenerate from is covered by the adjacent
    // cells' faces.
    if a == b || b == c || a == c {
        return;
    }
    let base = verts.len() as i32;
    verts.extend_from_slice(&[a, b, c]);
    faces.push([base, base + 1, base + 2]);
}

fn polygonize_tet(
    verts: &mut Vec<[f64; 3]>,
    faces: &mut Vec<[i32; 3]>,
    p: [[f64; 3]; 4],
    v: [f64; 4],
) {
    let inside = [v[0] < 0.0, v[1] < 0.0, v[2] < 0.0, v[3] < 0.0];
    let n_in: usize = inside.iter().map(|&b| b as usize).sum();
    if n_in == 0 || n_in == 4 {
        return;
    }
    let mut cross: [Option<[f64; 3]>; 6] = [None; 6];
    for (e, &[i, j]) in TET_EDGES.iter().enumerate() {
        if inside[i] != inside[j] {
            cross[e] = Some(lerp_pt(p[i], v[i], p[j], v[j]));
        }
    }
    let x = |e: usize| cross[e].unwrap();
    if n_in == 1 {
        let k = inside.iter().position(|&b| b).unwrap();

        let mut es = Vec::new();
        for (e, &[i, j]) in TET_EDGES.iter().enumerate() {
            if i == k || j == k {
                es.push(e);
            }
        }
        emit_tri(verts, faces, x(es[0]), x(es[1]), x(es[2]));
    } else if n_in == 3 {
        let k = inside.iter().position(|&b| !b).unwrap();
        let mut es = Vec::new();
        for (e, &[i, j]) in TET_EDGES.iter().enumerate() {
            if i == k || j == k {
                es.push(e);
            }
        }
        emit_tri(verts, faces, x(es[0]), x(es[1]), x(es[2]));
    } else {
        let mut ins = Vec::new();
        let mut outs = Vec::new();
        for (k, &b) in inside.iter().enumerate() {
            if b {
                ins.push(k);
            } else {
                outs.push(k);
            }
        }
        let edge_of = |a: usize, b: usize| {
            TET_EDGES
                .iter()
                .position(|&[i, j]| (i == a && j == b) || (i == b && j == a))
                .unwrap()
        };
        let q0 = x(edge_of(ins[0], outs[0]));
        let q1 = x(edge_of(ins[0], outs[1]));
        let q2 = x(edge_of(ins[1], outs[1]));
        let q3 = x(edge_of(ins[1], outs[0]));
        emit_tri(verts, faces, q0, q1, q2);
        emit_tri(verts, faces, q0, q2, q3);
    }
}

#[derive(Clone, Debug, Default)]
pub struct BakedMesh {
    pub vertices: Vec<[f64; 3]>,
    pub faces: Vec<[i32; 3]>,
}

/// Topology audit of a baked mesh. Marching tetrahedra over a fully decided
/// sign grid (no undecidable corners) yields a closed surface by construction;
/// this report turns that guarantee into an assertion.
///
/// Degenerate (zero-area) faces arise when the iso-surface passes exactly
/// through a grid node; they carry no area and pair their non-self edges
/// internally, so the audit ignores them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TopologyReport {
    /// Vertices referenced by non-degenerate faces.
    pub vertices: usize,
    /// Non-degenerate faces.
    pub faces: usize,
    /// Zero-area faces (repeated vertex index), excluded from all counts.
    pub degenerate_faces: usize,
    /// Edges with exactly one incident non-degenerate face (holes).
    pub boundary_edges: usize,
    /// Edges with more than two incident non-degenerate faces.
    pub nonmanifold_edges: usize,
    /// Closed edges whose two incident faces wind in the same direction.
    pub inconsistent_edges: usize,
    /// V − E + F over the non-degenerate part (2 per spherical shell, 0 per
    /// torus shell, additive over disjoint shells).
    pub euler: i64,
}

impl TopologyReport {
    pub fn is_watertight(&self) -> bool {
        self.faces > 0 && self.boundary_edges == 0 && self.nonmanifold_edges == 0
    }

    pub fn is_consistently_oriented(&self) -> bool {
        self.inconsistent_edges == 0
    }
}

impl BakedMesh {
    pub fn is_empty(&self) -> bool {
        self.faces.is_empty()
    }

    pub fn triangle_count(&self) -> usize {
        self.faces.len()
    }

    /// Merge bit-identical vertices (grid nodes and lerp crossings are shared
    /// bit-exactly across cells) so topology can be audited per edge.
    pub fn weld(&mut self) {
        use std::collections::HashMap;
        let mut map: HashMap<(u64, u64, u64), i32> = HashMap::with_capacity(self.vertices.len());
        let mut verts: Vec<[f64; 3]> = Vec::with_capacity(self.vertices.len());
        let mut remap = vec![0i32; self.vertices.len()];
        for (i, v) in self.vertices.iter().enumerate() {
            let key = (v[0].to_bits(), v[1].to_bits(), v[2].to_bits());
            let id = *map.entry(key).or_insert_with(|| {
                verts.push(*v);
                (verts.len() - 1) as i32
            });
            remap[i] = id;
        }
        for f in &mut self.faces {
            *f = [
                remap[f[0] as usize],
                remap[f[1] as usize],
                remap[f[2] as usize],
            ];
        }
        self.vertices = verts;
    }

    pub fn topology_report(&self) -> TopologyReport {
        use std::collections::HashMap;
        let mut half: HashMap<(i32, i32), usize> = HashMap::new();
        let mut used: Vec<bool> = vec![false; self.vertices.len()];
        let mut degenerate_faces = 0;
        let mut faces = 0;
        for f in &self.faces {
            if f[0] == f[1] || f[1] == f[2] || f[2] == f[0] {
                degenerate_faces += 1;
                continue;
            }
            faces += 1;
            for &i in f {
                used[i as usize] = true;
            }
            for e in [[f[0], f[1]], [f[1], f[2]], [f[2], f[0]]] {
                *half.entry((e[0], e[1])).or_insert(0) += 1;
            }
        }
        let mut boundary_edges = 0;
        let mut nonmanifold_edges = 0;
        let mut inconsistent_edges = 0;
        let mut edges = 0usize;
        let mut seen: HashMap<(i32, i32), (usize, usize)> = HashMap::new();
        for (&(a, b), &n) in &half {
            let key = (a.min(b), a.max(b));
            let e = seen.entry(key).or_default();
            if a < b {
                e.0 += n;
            } else {
                e.1 += n;
            }
        }
        for &(fwd, rev) in seen.values() {
            edges += 1;
            let total = fwd + rev;
            if total == 1 {
                boundary_edges += 1;
            } else if total > 2 {
                nonmanifold_edges += 1;
            } else if fwd != 1 || rev != 1 {
                inconsistent_edges += 1;
            }
        }
        let vertices = used.iter().filter(|&&u| u).count();
        TopologyReport {
            vertices,
            faces,
            degenerate_faces,
            boundary_edges,
            nonmanifold_edges,
            inconsistent_edges,
            euler: vertices as i64 - edges as i64 + faces as i64,
        }
    }
}

impl BakedMesh {
    pub fn volume_of(verts: &[[f64; 3]], faces: &[[i32; 3]]) -> f64 {
        let mut v = 0.0;
        for f in faces {
            let a = verts[f[0] as usize];
            let b = verts[f[1] as usize];
            let c = verts[f[2] as usize];
            v += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]);
        }
        v / 6.0
    }

    pub fn volume(&self) -> f64 {
        Self::volume_of(&self.vertices, &self.faces)
    }
}

/// Re-orient every connected shell consistently (BFS over shared edges),
/// seeded by the field gradient at the most confident face of each shell.
/// Per-triangle gradient flips are noisy at field creases (cap rims, CSG
/// seams); propagation makes orientation consistent by construction.
fn orient_outward(mesh: &mut BakedMesh, params: &GeometryParams) {
    use std::collections::HashMap;
    let mut edge_faces: HashMap<(i32, i32), Vec<(usize, bool)>> = HashMap::new();
    for (fi, f) in mesh.faces.iter().enumerate() {
        for e in [[f[0], f[1]], [f[1], f[2]], [f[2], f[0]]] {
            edge_faces
                .entry((e[0].min(e[1]), e[0].max(e[1])))
                .or_default()
                .push((fi, e[0] < e[1]));
        }
    }
    let n = mesh.faces.len();
    let mut flip = vec![false; n];
    let mut visited = vec![false; n];
    let face_geom = |fi: usize| -> ([f64; 3], [f64; 3]) {
        let f = mesh.faces[fi];
        let (a, b, c) = (
            mesh.vertices[f[0] as usize],
            mesh.vertices[f[1] as usize],
            mesh.vertices[f[2] as usize],
        );
        let ab = sub(b, a);
        let ac = sub(c, a);
        let nrm = [
            ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0],
        ];
        let cen = [
            (a[0] + b[0] + c[0]) / 3.0,
            (a[1] + b[1] + c[1]) / 3.0,
            (a[2] + b[2] + c[2]) / 3.0,
        ];
        (nrm, cen)
    };
    for seed in 0..n {
        if visited[seed] {
            continue;
        }
        // Collect the connected component first, then seed orientation at the
        // face whose normal best agrees with the field gradient.
        let mut comp = Vec::new();
        let mut stack = vec![seed];
        visited[seed] = true;
        while let Some(fi) = stack.pop() {
            comp.push(fi);
            let f = mesh.faces[fi];
            for e in [[f[0], f[1]], [f[1], f[2]], [f[2], f[0]]] {
                if let Some(list) = edge_faces.get(&(e[0].min(e[1]), e[0].max(e[1]))) {
                    for &(gi, _) in list {
                        if !visited[gi] {
                            visited[gi] = true;
                            stack.push(gi);
                        }
                    }
                }
            }
        }
        let mut best = seed;
        let mut best_conf = -1.0;
        for &fi in &comp {
            let (nrm, cen) = face_geom(fi);
            let conf = dot(nrm, grad(params, cen)).abs();
            if conf > best_conf {
                best_conf = conf;
                best = fi;
            }
        }
        let (nrm, cen) = face_geom(best);
        flip[best] = dot(nrm, grad(params, cen)) < 0.0;
        let mut stack = vec![best];
        let mut done = vec![false; n];
        done[best] = true;
        while let Some(fi) = stack.pop() {
            let f = mesh.faces[fi];
            for e in [[f[0], f[1]], [f[1], f[2]], [f[2], f[0]]] {
                let dir = (e[0] < e[1]) != flip[fi]; // traversal after flip
                if let Some(list) = edge_faces.get(&(e[0].min(e[1]), e[0].max(e[1]))) {
                    for &(gi, gdir) in list {
                        if gi != fi && !done[gi] {
                            // neighbor must traverse the shared edge oppositely
                            flip[gi] = gdir == dir;
                            done[gi] = true;
                            stack.push(gi);
                        }
                    }
                }
            }
        }
    }
    for (fi, f) in mesh.faces.iter_mut().enumerate() {
        if flip[fi] {
            f.swap(1, 2);
        }
    }
}

pub const MAX_BAKE_NODES: usize = 6_000_000;

impl GeometryParams {
    pub fn bake(&self, step: f64) -> Result<BakedMesh, String> {
        if !(step > 0.0) || !step.is_finite() {
            return Err(format!("bake: bad step {step}"));
        }
        match self {
            GeometryParams::CircleParams(_) => return Err("bake: circle is not a solid".into()),
            _ => {}
        }
        let [lo, hi] = self
            .bounds()
            .ok_or("bake: unbounded geometry (plane/infinite)")?;
        // Pad by half a cell on every side: with tight bounds the surface
        // would otherwise sit exactly on the outer grid nodes (v == 0), the
        // systematically degenerate configuration for cell classification.
        let span = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        let mut nx = (span[0] / step).ceil().max(1.0) as usize;
        let mut ny = (span[1] / step).ceil().max(1.0) as usize;
        let mut nz = (span[2] / step).ceil().max(1.0) as usize;
        let lo = [
            lo[0] - 0.5 * span[0] / nx as f64,
            lo[1] - 0.5 * span[1] / ny as f64,
            lo[2] - 0.5 * span[2] / nz as f64,
        ];
        let hi = [
            hi[0] + 0.5 * span[0] / nx as f64,
            hi[1] + 0.5 * span[1] / ny as f64,
            hi[2] + 0.5 * span[2] / nz as f64,
        ];
        let span = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
        nx = (span[0] / step).ceil().max(1.0) as usize;
        ny = (span[1] / step).ceil().max(1.0) as usize;
        nz = (span[2] / step).ceil().max(1.0) as usize;
        let nodes = (nx + 1).saturating_mul(ny + 1).saturating_mul(nz + 1);
        if nodes > MAX_BAKE_NODES {
            return Err(format!(
            "bake: grid {nx}x{ny}x{nz} ({nodes} nodes) exceeds limit {MAX_BAKE_NODES}; raise step"
        ));
        }
        let dx = (hi[0] - lo[0]) / nx as f64;
        let dy = (hi[1] - lo[1]) / ny as f64;
        let dz = (hi[2] - lo[2]) / nz as f64;
        let at = |i: usize, j: usize, k: usize| {
            [
                lo[0] + dx * i as f64,
                lo[1] + dy * j as f64,
                lo[2] + dz * k as f64,
            ]
        };
        let mut vals = vec![0.0f64; nodes];
        for k in 0..=nz {
            for j in 0..=ny {
                for i in 0..=nx {
                    vals[k * (ny + 1) * (nx + 1) + j * (nx + 1) + i] = self.field(at(i, j, k));
                }
            }
        }
        // Export may not abstain (§3.1 of the robustness plan): every grid
        // node must have a decided sign before any cell is emitted.
        if let Some(bad) = vals.iter().position(|v| !v.is_finite()) {
            return Err(format!(
                "bake: non-finite field value at grid node {bad} (undecidable corner)"
            ));
        }
        let val = |i: usize, j: usize, k: usize| vals[k * (ny + 1) * (nx + 1) + j * (nx + 1) + i];
        let mut mesh = BakedMesh::default();
        for k in 0..nz {
            for j in 0..ny {
                for i in 0..nx {
                    let mut cp = [[0.0; 3]; 8];
                    let mut cv = [0.0; 8];
                    for (c, &[ox, oy, oz]) in CORN.iter().enumerate() {
                        let (ii, jj, kk) = (i + ox, j + oy, k + oz);
                        cp[c] = at(ii, jj, kk);
                        cv[c] = val(ii, jj, kk);
                    }
                    for tet in &TETS {
                        let p = [cp[tet[0]], cp[tet[1]], cp[tet[2]], cp[tet[3]]];
                        let v = [cv[tet[0]], cv[tet[1]], cv[tet[2]], cv[tet[3]]];
                        polygonize_tet(&mut mesh.vertices, &mut mesh.faces, p, v);
                    }
                }
            }
        }
        mesh.weld();
        orient_outward(&mut mesh, self);
        Ok(mesh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        BoxGeometry, ConeGeometry, CsgGeometry, CsgOp, CylinderGeometry, EllipsoidGeometry,
        Geometry, SphereGeometry, TorusGeometry,
    };

    fn world(g: &Geometry) -> GeometryParams {
        g.identity_params()
    }

    #[test]
    fn sphere_volume() {
        let p = world(&Geometry::SphereGeometry(SphereGeometry::new(1.0)));
        let m = p.bake(0.08).unwrap();
        assert!(m.triangle_count() > 1000);
        let v = m.volume().abs();
        assert!((v - 4.18879).abs() < 4.18879 * 0.08, "V={v}");
    }

    #[test]
    fn box_volume() {
        let p = world(&Geometry::BoxGeometry(BoxGeometry::new(2.0, 2.0, 2.0)));
        let m = p.bake(0.1).unwrap();
        let v = m.volume().abs();
        assert!((v - 8.0).abs() < 8.0 * 0.05, "V={v}");
    }

    #[test]
    fn difference_removes_volume() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Difference,
            vec![
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0)),
            ],
        ));
        let p = world(&g);
        let m = p.bake(0.08).unwrap();
        let v = m.volume().abs();

        assert!((v - 3.18879).abs() < 0.35, "V={v}");
        assert!(v < 4.18879);
    }

    #[test]
    fn torus_and_cone_bake() {
        let t = world(&Geometry::TorusGeometry(TorusGeometry::new(1.0, 0.3)));
        let mt = t.bake(0.08).unwrap();
        assert!(mt.triangle_count() > 500);
        let c = world(&Geometry::ConeGeometry(ConeGeometry::new(0.5, 1.0)));
        let mc = c.bake(0.06).unwrap();
        let v = mc.volume().abs();

        assert!((v - 0.261799).abs() < 0.05, "V={v}");
    }

    #[test]
    fn cylinder_and_ellipsoid_bake() {
        let cy = world(&Geometry::CylinderGeometry(CylinderGeometry::new(0.5, 2.0)));
        let m = cy.bake(0.06).unwrap();
        let v = m.volume().abs();
        assert!(
            (v - std::f64::consts::PI * 0.25 * 2.0).abs() < 0.15,
            "V={v}"
        );
        let el = world(&Geometry::EllipsoidGeometry(EllipsoidGeometry::new(
            1.0, 0.5, 0.5,
        )));
        let me = el.bake(0.06).unwrap();
        let ve = me.volume().abs();
        assert!((ve - 4.18879 * 0.25).abs() < 0.2, "V={ve}");
    }

    fn assert_shells(m: &BakedMesh, want_euler: i64) {
        let r = m.topology_report();
        assert!(r.is_watertight(), "not watertight: {r:?}");
        assert!(r.is_consistently_oriented(), "inconsistent winding: {r:?}");
        assert_eq!(r.euler, want_euler, "euler mismatch: {r:?}");
    }

    #[test]
    fn watertight_sphere_and_torus() {
        let s = world(&Geometry::SphereGeometry(SphereGeometry::new(1.0)));
        assert_shells(&s.bake(0.08).unwrap(), 2);
        let t = world(&Geometry::TorusGeometry(TorusGeometry::new(1.0, 0.3)));
        assert_shells(&t.bake(0.08).unwrap(), 0);
    }

    #[test]
    fn watertight_box() {
        // Tight bounds: without the half-cell pad the faces would sit exactly
        // on grid nodes (v == 0) — the systematically degenerate case.
        let b = world(&Geometry::BoxGeometry(BoxGeometry::new(2.0, 2.0, 2.0)));
        assert_shells(&b.bake(0.1).unwrap(), 2);
    }

    #[test]
    fn watertight_cylinder_cone_ellipsoid() {
        let cy = world(&Geometry::CylinderGeometry(CylinderGeometry::new(0.5, 2.0)));
        assert_shells(&cy.bake(0.08).unwrap(), 2);
        let co = world(&Geometry::ConeGeometry(ConeGeometry::new(0.5, 1.0)));
        assert_shells(&co.bake(0.06).unwrap(), 2);
        let el = world(&Geometry::EllipsoidGeometry(EllipsoidGeometry::new(
            1.0, 0.5, 0.5,
        )));
        assert_shells(&el.bake(0.08).unwrap(), 2);
    }

    #[test]
    fn watertight_cavity_euler_additive() {
        // Sphere with a fully contained box cavity: two disjoint shells,
        // chi = 2 + 2 = 4.
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Difference,
            vec![
                Geometry::SphereGeometry(SphereGeometry::new(1.0)),
                Geometry::BoxGeometry(BoxGeometry::new(1.0, 1.0, 1.0)),
            ],
        ));
        let p = world(&g);
        assert_shells(&p.bake(0.08).unwrap(), 4);
    }

    #[test]
    fn bake_rejects_nonfinite_field() {
        // 字段不可判定的图元（NaN 中心）：导出不得静默放弃
        let eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let p = GeometryParams::SphereParams(crate::SphereParams {
            c: [f64::NAN, 0.0, 0.0],
            r: 1.0,
            axes: eye,
        });
        let e = p.bake(0.1).unwrap_err();
        assert!(
            e.contains("non-finite"),
            "export may not abstain silently: {e}"
        );
    }

    #[test]
    fn unbounded_and_nonsolid_error() {
        let pl = world(&Geometry::PlaneGeometry(crate::PlaneGeometry::new(
            [0.0, 1.0, 0.0],
            0.0,
        )));
        assert!(pl.bounds().is_none());
        assert!(pl.bake(0.1).is_err());
        let ci = world(&Geometry::CircleGeometry(crate::CircleGeometry::new(1.0)));
        assert!(ci.bake(0.1).is_err());
    }
}
