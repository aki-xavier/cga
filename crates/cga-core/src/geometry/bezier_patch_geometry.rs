#[derive(Clone, Debug)]
pub struct BezierPatchGeometry {
    pub points: [[f64; 3]; 16],
    pub weights: [f64; 16],
    pub thickness: f64,
    pub div: usize,
}
impl BezierPatchGeometry {
    pub fn new(points: &[[f64; 3]], thickness: f64, div: usize) -> BezierPatchGeometry {
        Self::with_weights(points, None, thickness, div)
    }
    pub fn with_weights(
        points: &[[f64; 3]],
        weights: Option<&[f64]>,
        thickness: f64,
        div: usize,
    ) -> BezierPatchGeometry {
        if points.len() != 16 {
            panic!(
                "bezier patch needs 16 control points (4x4), got {}",
                points.len()
            );
        }
        if thickness < 0.0 {
            panic!("bezier thickness must be >= 0, got {thickness}");
        }
        if div < 1 || div > 32 {
            panic!("bezier div must be in 1..=32, got {div}");
        }
        let mut pts = [[0.0; 3]; 16];
        pts.copy_from_slice(&points[..16]);
        let mut w = [1.0; 16];
        if let Some(ws) = weights {
            if ws.len() != 16 {
                panic!("bezier patch needs 16 weights, got {}", ws.len());
            }
            for (i, v) in ws.iter().enumerate() {
                if *v <= 0.0 {
                    panic!("bezier weights must be > 0, got {v} at {i}");
                }
                w[i] = *v;
            }
        }
        BezierPatchGeometry {
            points: pts,
            weights: w,
            thickness,
            div,
        }
    }
    pub fn is_solid(&self) -> bool {
        self.thickness > 0.0
    }
    fn basis(t: f64) -> [f64; 4] {
        let s = 1.0 - t;
        [s * s * s, 3.0 * s * s * t, 3.0 * s * t * t, t * t * t]
    }
    fn dbasis(t: f64) -> [f64; 4] {
        let s = 1.0 - t;
        [
            -3.0 * s * s,
            3.0 * s * s - 6.0 * s * t,
            6.0 * s * t - 3.0 * t * t,
            3.0 * t * t,
        ]
    }
    pub fn eval(&self, u: f64, v: f64) -> [f64; 3] {
        let bu = Self::basis(u);
        let bv = Self::basis(v);
        let mut num = [0.0; 3];
        let mut den = 0.0;
        for i in 0..4 {
            for j in 0..4 {
                let k = i * 4 + j;
                let w = bu[i] * bv[j] * self.weights[k];
                num[0] += w * self.points[k][0];
                num[1] += w * self.points[k][1];
                num[2] += w * self.points[k][2];
                den += w;
            }
        }
        [num[0] / den, num[1] / den, num[2] / den]
    }
    fn deriv(&self, u: f64, v: f64) -> ([f64; 3], [f64; 3]) {
        let bu = Self::basis(u);
        let bv = Self::basis(v);
        let du = Self::dbasis(u);
        let dv = Self::dbasis(v);
        let mut n = [0.0; 3];
        let mut nu = [0.0; 3];
        let mut nv = [0.0; 3];
        let mut den = 0.0;
        let mut denu = 0.0;
        let mut denv = 0.0;
        for i in 0..4 {
            for j in 0..4 {
                let k = i * 4 + j;
                let w = self.weights[k];
                let p = self.points[k];
                let b = bu[i] * bv[j];
                den += b * w;
                denu += du[i] * bv[j] * w;
                denv += bu[i] * dv[j] * w;
                for a in 0..3 {
                    n[a] += b * w * p[a];
                    nu[a] += du[i] * bv[j] * w * p[a];
                    nv[a] += bu[i] * dv[j] * w * p[a];
                }
            }
        }
        let mut pu = [0.0; 3];
        let mut pv = [0.0; 3];
        for a in 0..3 {
            pu[a] = (nu[a] * den - n[a] * denu) / (den * den);
            pv[a] = (nv[a] * den - n[a] * denv) / (den * den);
        }
        (pu, pv)
    }
    pub fn normal(&self, u: f64, v: f64) -> [f64; 3] {
        let (pu, pv) = self.deriv(u, v);
        let c = super::vec3_cross(pu, pv);
        let l = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        if l < 1e-12 {
            return [0.0, 0.0, 1.0];
        }
        [c[0] / l, c[1] / l, c[2] / l]
    }
    pub fn control_bounds(&self) -> [[f64; 3]; 2] {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in &self.points {
            for a in 0..3 {
                if p[a] < lo[a] {
                    lo[a] = p[a];
                }
                if p[a] > hi[a] {
                    hi[a] = p[a];
                }
            }
        }
        let e = self.thickness / 2.0;
        for a in 0..3 {
            lo[a] -= e;
            hi[a] += e;
        }
        [lo, hi]
    }
    pub fn chord_error(&self) -> f64 {
        let n = self.div;
        let mut worst = 0.0;
        for i in 0..n {
            for j in 0..n {
                let u0 = i as f64 / n as f64;
                let u1 = (i + 1) as f64 / n as f64;
                let v0 = j as f64 / n as f64;
                let v1 = (j + 1) as f64 / n as f64;
                let c00 = self.eval(u0, v0);
                let c10 = self.eval(u1, v0);
                let c01 = self.eval(u0, v1);
                let c11 = self.eval(u1, v1);
                let mid = self.eval((u0 + u1) / 2.0, (v0 + v1) / 2.0);
                for a in 0..3 {
                    let lin = (c00[a] + c10[a] + c01[a] + c11[a]) / 4.0;
                    let e = (mid[a] - lin).abs();
                    if e > worst {
                        worst = e;
                    }
                }
            }
        }
        worst
    }
    fn grid_point(&self, i: usize, j: usize) -> ([f64; 3], [f64; 3]) {
        let n = self.div;
        let u = i as f64 / n as f64;
        let v = j as f64 / n as f64;
        (self.eval(u, v), self.normal(u, v))
    }
    fn oriented(a: [f64; 3], b: [f64; 3], c: [f64; 3], n: [f64; 3]) -> [i32; 3] {
        let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let cr = super::vec3_cross(ab, ac);
        let d = cr[0] * n[0] + cr[1] * n[1] + cr[2] * n[2];
        if d >= 0.0 {
            [0, 1, 2]
        } else {
            [0, 2, 1]
        }
    }
    pub fn tessellate(&self) -> (Vec<[f64; 3]>, Vec<[i32; 3]>) {
        let n = self.div;
        let w = n + 1;
        let mut pts: Vec<[f64; 3]> = Vec::with_capacity(w * w);
        let mut nrms: Vec<[f64; 3]> = Vec::with_capacity(w * w);
        for i in 0..=n {
            for j in 0..=n {
                let (p, q) = self.grid_point(i, j);
                pts.push(p);
                nrms.push(q);
            }
        }
        let idx = |i: usize, j: usize| (i * w + j) as i32;
        let mut top: Vec<[i32; 3]> = Vec::with_capacity(2 * n * n);
        for i in 0..n {
            for j in 0..n {
                let a = idx(i, j);
                let b = idx(i + 1, j);
                let c = idx(i, j + 1);
                let d = idx(i + 1, j + 1);
                let nm = self.normal((i as f64 + 0.5) / n as f64, (j as f64 + 0.5) / n as f64);
                let o1 = Self::oriented(pts[a as usize], pts[b as usize], pts[d as usize], nm);
                let o2 = Self::oriented(pts[a as usize], pts[d as usize], pts[c as usize], nm);
                let vs1 = [a, b, d];
                let vs2 = [a, d, c];
                top.push([
                    vs1[o1[0] as usize],
                    vs1[o1[1] as usize],
                    vs1[o1[2] as usize],
                ]);
                top.push([
                    vs2[o2[0] as usize],
                    vs2[o2[1] as usize],
                    vs2[o2[2] as usize],
                ]);
            }
        }
        if self.thickness <= 0.0 {
            return (pts, top);
        }
        let h = self.thickness / 2.0;
        let mut verts: Vec<[f64; 3]> = Vec::with_capacity(2 * w * w);
        for (k, p) in pts.iter().enumerate() {
            let q = nrms[k];
            verts.push([p[0] + q[0] * h, p[1] + q[1] * h, p[2] + q[2] * h]);
        }
        let base = verts.len() as i32;
        for (k, p) in pts.iter().enumerate() {
            let q = nrms[k];
            verts.push([p[0] - q[0] * h, p[1] - q[1] * h, p[2] - q[2] * h]);
        }
        let mut faces: Vec<[i32; 3]> = Vec::with_capacity(top.len() * 2 + 8 * n);
        faces.extend(top.iter().cloned());
        for f in &top {
            faces.push([base + f[0], base + f[2], base + f[1]]);
        }
        use std::collections::HashMap;
        let mut occ: HashMap<(i32, i32), i32> = HashMap::new();
        for f in &top {
            for e in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
                let key = if e.0 < e.1 { (e.0, e.1) } else { (e.1, e.0) };
                *occ.entry(key).or_insert(0) += 1;
            }
        }
        for f in &top {
            for e in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
                let key = if e.0 < e.1 { (e.0, e.1) } else { (e.1, e.0) };
                if occ[&key] == 1 {
                    let (a, b) = e;
                    faces.push([a, base + b, b]);
                    faces.push([a, base + a, base + b]);
                }
            }
        }
        (verts, faces)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn flat_points() -> Vec<[f64; 3]> {
        let mut pts = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                pts.push([
                    -1.0 + i as f64 * (2.0 / 3.0),
                    -1.0 + j as f64 * (2.0 / 3.0),
                    0.0,
                ]);
            }
        }
        pts
    }

    fn bump_points() -> Vec<[f64; 3]> {
        let mut pts = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                let x = -1.0 + i as f64 * (2.0 / 3.0);
                let y = -1.0 + j as f64 * (2.0 / 3.0);
                pts.push([x, y, 0.3 * (1.0 - (x * x + y * y) / 2.0).max(0.0)]);
            }
        }
        pts
    }

    #[test]
    fn test_bezier_flat_eval() {
        let g = BezierPatchGeometry::new(&flat_points(), 0.0, 4);
        assert_eq!(g.eval(0.0, 0.0), [-1.0, -1.0, 0.0]);
        assert_eq!(g.eval(1.0, 1.0), [1.0, 1.0, 0.0]);
        let c = g.eval(0.5, 0.5);
        assert!((c[0]).abs() < 1e-12);
        assert!((c[1]).abs() < 1e-12);
        assert!((c[2]).abs() < 1e-12);
    }

    #[test]
    fn test_bezier_flat_normal_and_chord() {
        let g = BezierPatchGeometry::new(&flat_points(), 0.0, 4);
        let n = g.normal(0.3, 0.7);
        assert!((n[0]).abs() < 1e-12);
        assert!((n[1]).abs() < 1e-12);
        assert!((n[2] - 1.0).abs() < 1e-12);
        assert!(g.chord_error() < 1e-12);
    }

    #[test]
    fn test_bezier_chord_shrinks_with_div() {
        let e1 = BezierPatchGeometry::new(&bump_points(), 0.0, 1).chord_error();
        let e8 = BezierPatchGeometry::new(&bump_points(), 0.0, 8).chord_error();
        assert!(e1 > 1e-3);
        assert!(e8 < e1 / 10.0);
        assert!(e8 < 5e-3);
    }

    #[test]
    fn test_bezier_rational_corner_interp() {
        let mut w = [1.0; 16];
        w[0] = 5.0;
        w[15] = 0.2;
        let g = BezierPatchGeometry::with_weights(&flat_points(), Some(&w), 0.0, 4);
        assert_eq!(g.eval(0.0, 0.0), [-1.0, -1.0, 0.0]);
        assert_eq!(g.eval(1.0, 1.0), [1.0, 1.0, 0.0]);
        let b = g.control_bounds();
        let c = g.eval(0.37, 0.61);
        for a in 0..3 {
            assert!(c[a] >= b[0][a] - 1e-12 && c[a] <= b[1][a] + 1e-12);
        }
    }

    #[test]
    fn test_bezier_open_counts() {
        let g = BezierPatchGeometry::new(&flat_points(), 0.0, 2);
        let (v, f) = g.tessellate();
        assert_eq!(v.len(), 9);
        assert_eq!(f.len(), 8);
    }

    #[test]
    fn test_bezier_shell_manifold() {
        let g = BezierPatchGeometry::new(&bump_points(), 0.2, 3);
        assert!(g.is_solid());
        let (v, f) = g.tessellate();
        assert_eq!(v.len(), 2 * 16);
        assert_eq!(f.len(), 2 * 18 + 2 * 12);
        let mut occ: HashMap<(i32, i32), i32> = HashMap::new();
        for t in &f {
            for e in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                let key = if e.0 < e.1 { (e.0, e.1) } else { (e.1, e.0) };
                *occ.entry(key).or_insert(0) += 1;
            }
        }
        assert!(!occ.is_empty());
        for (k, c) in &occ {
            assert_eq!(*c, 2, "edge {k:?} shared by {c} faces");
        }
    }

    #[test]
    fn test_bezier_shell_field_sign() {
        let g = BezierPatchGeometry::new(&flat_points(), 0.2, 4);
        let p = crate::BezierParams::tessellated(&g);
        let inside = crate::GeometryParams::BezierParams(p.clone()).field([0.0, 0.0, 0.0]);
        assert!(inside < 0.0);
        let above = crate::GeometryParams::BezierParams(p.clone()).field([0.0, 0.0, 0.5]);
        assert!(above > 0.0);
        let side = crate::GeometryParams::BezierParams(p).field([3.0, 0.0, 0.0]);
        assert!(side > 0.0);
    }

    #[test]
    #[should_panic(expected = "16 control points")]
    fn test_bezier_rejects_short_points() {
        BezierPatchGeometry::new(&flat_points()[..15].to_vec(), 0.0, 4);
    }

    #[test]
    #[should_panic(expected = "thickness must be >= 0")]
    fn test_bezier_rejects_negative_thickness() {
        BezierPatchGeometry::new(&flat_points(), -0.1, 4);
    }

    #[test]
    #[should_panic(expected = "div must be in 1..=32")]
    fn test_bezier_rejects_bad_div() {
        BezierPatchGeometry::new(&flat_points(), 0.0, 0);
    }
}
