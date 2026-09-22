use crate::tables::*;
use std::fmt;

#[derive(Clone, Copy, Debug)]
pub struct Multivector {
    pub values: [f64; 32],
}

pub fn mv_zero() -> Multivector {
    Multivector {
        values: [0.0; num_components],
    }
}

pub fn mv_scalar(s: f64) -> Multivector {
    let mut m = mv_zero();
    m.values[0] = s;
    m
}

pub fn mv_vector(v1: f64, v2: f64, v3: f64, v0: f64, ve: f64) -> Multivector {
    let mut m = mv_zero();
    m.values[1] = v1;
    m.values[2] = v2;
    m.values[3] = v3;
    m.values[4] = v0;
    m.values[5] = ve;
    m
}

pub fn mv_bivector(components: [f64; 10]) -> Multivector {
    let mut m = mv_zero();
    for i in 0..10 {
        m.values[grade_indices[grade_start[2] + i]] = components[i];
    }
    m
}

pub fn e1() -> Multivector {
    mv_vector(1.0, 0.0, 0.0, 0.0, 0.0)
}

pub fn e2() -> Multivector {
    mv_vector(0.0, 1.0, 0.0, 0.0, 0.0)
}

pub fn e3() -> Multivector {
    mv_vector(0.0, 0.0, 1.0, 0.0, 0.0)
}

pub fn e0() -> Multivector {
    mv_vector(0.0, 0.0, 0.0, 1.0, 0.0)
}

pub fn einf() -> Multivector {
    mv_vector(0.0, 0.0, 0.0, 0.0, 1.0)
}

impl Multivector {
    pub fn grade(&self, g: usize) -> Multivector {
        let mut res = mv_zero();
        for &idx in &grade_indices[grade_start[g]..grade_start[g + 1]] {
            res.values[idx] = self.values[idx];
        }
        res
    }

    pub fn scalar_part(&self) -> f64 {
        self.values[0]
    }

    pub fn vector_part(&self) -> [f64; 5] {
        [
            self.values[1],
            self.values[2],
            self.values[3],
            self.values[4],
            self.values[5],
        ]
    }

    pub fn euclidean_vector(&self) -> [f64; 3] {
        [self.values[1], self.values[2], self.values[3]]
    }

    pub fn e0_coeff(&self) -> f64 {
        self.values[4]
    }

    pub fn einf_coeff(&self) -> f64 {
        self.values[5]
    }

    pub fn bivector_part(&self) -> [f64; 10] {
        let mut out = [0.0; 10];
        for i in 0..10 {
            out[i] = self.values[grade_indices[grade_start[2] + i]];
        }
        out
    }

    pub fn coords(&self) -> [f64; 3] {
        let w = self.values[4];
        if w.abs() < 1e-12 {
            panic!("multivector has no e0 component; not a finite point");
        }
        [self.values[1] / w, self.values[2] / w, self.values[3] / w]
    }

    // 为什么: 这里判断的是系数近似全零，不是 CGA null 性质——conformal point 本身就是 null 向量但分量不全为零。
    pub fn is_zero(&self) -> bool {
        for v in self.values {
            if v.abs() > 1e-10 {
                return false;
            }
        }
        true
    }

    pub fn vmax(&self) -> f64 {
        let mut mx = 0.0;
        for v in self.values {
            let a = v.abs();
            if a > mx {
                mx = a;
            }
        }
        mx
    }

    pub fn add(&self, o: &Multivector) -> Multivector {
        let mut res = mv_zero();
        for i in 0..num_components {
            res.values[i] = self.values[i] + o.values[i];
        }
        res
    }

    pub fn sub(&self, o: &Multivector) -> Multivector {
        let mut res = mv_zero();
        for i in 0..num_components {
            res.values[i] = self.values[i] - o.values[i];
        }
        res
    }

    pub fn mul_scalar(&self, s: f64) -> Multivector {
        let mut res = mv_zero();
        for i in 0..num_components {
            res.values[i] = self.values[i] * s;
        }
        res
    }

    pub fn div_scalar(&self, s: f64) -> Multivector {
        let mut res = mv_zero();
        for i in 0..num_components {
            res.values[i] = self.values[i] / s;
        }
        res
    }

    pub fn neg(&self) -> Multivector {
        let mut res = mv_zero();
        for i in 0..num_components {
            res.values[i] = -self.values[i];
        }
        res
    }

    // 为什么: 近似相等不传递，所以 Multivector 故意不实现 PartialEq；需要比较时显式调用，atol=1e-6。
    pub fn approx_eq(&self, o: &Multivector) -> bool {
        self.values
            .iter()
            .zip(&o.values)
            .all(|(&a, &b)| (a - b).abs() <= 1e-6)
    }

    pub fn copy(&self) -> Multivector {
        *self
    }

    pub fn gp(&self, o: &Multivector) -> Multivector {
        let mut res = mv_zero();
        for i in 0..num_components {
            let a = self.values[i];
            if a == 0.0 {
                continue;
            }
            for k in gp_row_start[i]..gp_row_start[i + 1] {
                let b = o.values[gp_j[k]];
                if b == 0.0 {
                    continue;
                }
                res.values[gp_dst[k]] += gp_sign[k] * a * b;
            }
        }
        res
    }

    // 为什么: Hestenes fat-dot 内积（匹配 clifford 的 `|`），只对 r,s≥1 求和以避免与左/右收缩混淆。
    pub fn ip(&self, o: &Multivector) -> Multivector {
        let mut res = mv_zero();
        for ga in 1..num_grades {
            let a_g = self.grade(ga);
            if a_g.is_zero() {
                continue;
            }
            for gb in 1..num_grades {
                let b_g = o.grade(gb);
                if b_g.is_zero() {
                    continue;
                }
                let prod = a_g.gp(&b_g);
                res = res.add(&prod.grade((gb as i64 - ga as i64).unsigned_abs() as usize));
            }
        }
        res
    }

    // 为什么: 外积 = 各阶 wedge 之和；grade 自然相加，便于按 grade 拆分 blade。
    pub fn op(&self, o: &Multivector) -> Multivector {
        let mut res = mv_zero();
        for ga in 0..num_grades {
            let a_g = self.grade(ga);
            if a_g.is_zero() {
                continue;
            }
            for gb in 0..num_grades - ga {
                let b_g = o.grade(gb);
                if b_g.is_zero() {
                    continue;
                }
                let prod = a_g.gp(&b_g);
                res = res.add(&prod.grade(ga + gb));
            }
        }
        res
    }

    pub fn reverse(&self) -> Multivector {
        let mut res = mv_zero();
        for (r, (&v, &m)) in res
            .values
            .iter_mut()
            .zip(self.values.iter().zip(&reverse_mask))
        {
            *r = v * m;
        }
        res
    }

    pub fn grade_involution(&self) -> Multivector {
        let mut res = mv_zero();
        for (r, (&v, &m)) in res
            .values
            .iter_mut()
            .zip(self.values.iter().zip(&involution_mask))
        {
            *r = v * m;
        }
        res
    }

    pub fn conjugate(&self) -> Multivector {
        self.reverse().grade_involution()
    }

    // 为什么: 左乘 I⁻¹ 即 Hodge 对偶；在 null 基底下 I²=-1，所以 I⁻¹=-I，无需显式求逆。
    pub fn dual(&self) -> Multivector {
        let mut i_inv = mv_zero();
        i_inv.values[31] = -1.0;
        self.gp(&i_inv)
    }

    // 为什么: dual∘dual = -id，所以 undual 就是再取一次 dual（即 -dual）。
    pub fn undual(&self) -> Multivector {
        self.dual().neg()
    }

    // 为什么: 几何求交用对偶的外积（regressive product）；当双对偶 wedge 为零（包含/平行 blade）时退化为子空间线性交点，对直接形式 blade（点/线/面）精确；非 blade 输入 panic。
    pub fn meet(&self, o: &Multivector) -> Multivector {
        if self.is_zero() || o.is_zero() {
            return mv_zero();
        }
        let ga = self.blade_grade();
        let gb = o.blade_grade();
        if ga < 0 || gb < 0 {
            panic!("meet: arguments must be blades");
        }
        if ga == 0 || gb == 0 {
            return mv_zero();
        }
        let w = self.dual().op(&o.dual());
        if !w.is_zero() {
            return w.dual();
        }

        let na = nullspace(&rows_of_mask(blade_mask(self)));
        let nb = nullspace(&rows_of_mask(blade_mask(o)));
        let mut merged = na;
        merged.extend(nb);
        let c = nullspace(&merged);
        if c.is_empty() {
            return mv_zero();
        }
        wedge_rows(&c)
    }

    // 为什么: 求张成 self 与 o 的最小 blade；scalar/零 输入退化为另一 blade，非 blade 输入 panic。
    pub fn join(&self, o: &Multivector) -> Multivector {
        if self.is_zero() {
            return *o;
        }
        if o.is_zero() {
            return *self;
        }
        let ga = self.blade_grade();
        let gb = o.blade_grade();
        if ga < 0 || gb < 0 {
            panic!("join: arguments must be blades");
        }
        if ga == 0 {
            return *o;
        }
        if gb == 0 {
            return *self;
        }
        let mut rows = rows_of_mask(blade_mask(self));
        rows.extend(rows_of_mask(blade_mask(o)));
        wedge_rows(&row_basis(&rows))
    }

    pub fn norm(&self) -> f64 {
        let s = self.gp(&self.reverse()).values[0];
        s.abs().sqrt()
    }

    pub fn normalized(&self) -> Multivector {
        let n = self.norm();
        if n < 1e-12 {
            return mv_zero();
        }
        self.div_scalar(n)
    }

    pub fn bulk(&self) -> Multivector {
        let mut res = mv_zero();
        for idx in [0, 1, 2, 3, 6, 7, 10, 16] {
            res.values[idx] = self.values[idx];
        }
        res
    }

    pub fn weight(&self) -> Multivector {
        self.sub(&self.bulk())
    }

    fn blade_grade(&self) -> i32 {
        let mut g = -1;
        for (&v, &slot) in self.values.iter().zip(&slot_mask) {
            if v != 0.0 {
                let gi = bit_count(slot);
                if g != -1 && gi != g {
                    return -1;
                }
                g = gi;
            }
        }
        g
    }

    // 为什么: 公式 A⁻¹ = rev(A) / (A·rev(A))₀；只对 invertible blade/versor 成立——A·rev(A) 非 scalar 时 A 不可逆，会 panic。
    pub fn inverse(&self) -> Multivector {
        let prod = self.gp(&self.reverse());
        let s = prod.values[0];
        if s.abs() < 1e-12 || !prod.grade(0).approx_eq(&prod) {
            panic!("inverse: only blades and versors with nonzero norm are supported");
        }
        self.reverse().div_scalar(s)
    }

    pub fn lc(&self, o: &Multivector) -> Multivector {
        let mut res = mv_zero();
        for ga in 1..num_grades {
            let a_g = self.grade(ga);
            if a_g.is_zero() {
                continue;
            }
            for gb in ga..num_grades {
                let b_g = o.grade(gb);
                if b_g.is_zero() {
                    continue;
                }
                res = res.add(&a_g.gp(&b_g).grade(gb - ga));
            }
        }
        res
    }

    pub fn rc(&self, o: &Multivector) -> Multivector {
        let mut res = mv_zero();
        for ga in 1..num_grades {
            let a_g = self.grade(ga);
            if a_g.is_zero() {
                continue;
            }
            for gb in 1..ga + 1 {
                let b_g = o.grade(gb);
                if b_g.is_zero() {
                    continue;
                }
                res = res.add(&a_g.gp(&b_g).grade(ga - gb));
            }
        }
        res
    }

    pub fn commutator(&self, o: &Multivector) -> Multivector {
        self.gp(o).sub(&o.gp(self)).mul_scalar(0.5)
    }

    pub fn anticommutator(&self, o: &Multivector) -> Multivector {
        self.gp(o).add(&o.gp(self)).mul_scalar(0.5)
    }

    pub fn proj(&self, o: &Multivector) -> Multivector {
        self.ip(o).gp(&o.inverse())
    }

    pub fn rej(&self, o: &Multivector) -> Multivector {
        self.sub(&self.proj(o))
    }

    // 为什么: 用 versor v' = -n·v·n 做镜像；n 取归一法向，对向量和 blade 均成立（versor 共轭保持 grade）。
    pub fn reflect(&self, normal: [f64; 3]) -> Multivector {
        let len2 = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
        if len2 < 1e-18 {
            panic!("reflect: zero normal");
        }
        let n = mv_vector(normal[0], normal[1], normal[2], 0.0, 0.0).div_scalar(len2.sqrt());
        n.gp(self).gp(&n).neg()
    }
}

impl fmt::Display for Multivector {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut parts: Vec<String> = Vec::new();
        for g in 0..num_grades {
            for &idx in &grade_indices[grade_start[g]..grade_start[g + 1]] {
                let v = self.values[idx];
                if v.abs() <= 1e-10 {
                    continue;
                }
                let name = basis_names[idx];
                if name == "1" {
                    parts.push(format!("{v:.4}"));
                } else {
                    parts.push(format!("{v:+.4}*{name}"));
                }
            }
        }
        if parts.is_empty() {
            return write!(f, "Multivector(0)");
        }
        write!(f, "Multivector({})", parts.join(" "))
    }
}

#[allow(non_upper_case_globals)]
const slot_mask: [i32; 32] = [
    0, 1, 2, 4, 8, 16, 3, 5, 9, 17, 6, 10, 18, 12, 20, 24, 7, 11, 19, 13, 21, 25, 14, 22, 26, 28,
    15, 23, 27, 29, 30, 31,
];

fn bit_count(x: i32) -> i32 {
    let mut n = 0;
    let mut v = x;
    while v > 0 {
        n += v & 1;
        v >>= 1;
    }
    n
}

fn blade_mask(m: &Multivector) -> i32 {
    let mut mask = 0;
    for (&v, &slot) in m.values.iter().zip(&slot_mask) {
        if v != 0.0 {
            mask |= slot;
        }
    }
    mask
}

fn rows_of_mask(mask: i32) -> Vec<Vec<f64>> {
    let mut rows: Vec<Vec<f64>> = Vec::new();
    for i in 0..5 {
        if mask & (1 << i) != 0 {
            let mut row = vec![0.0; 5];
            row[i] = 1.0;
            rows.push(row);
        }
    }
    rows
}

fn row_basis(rows: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let mut mat: Vec<Vec<f64>> = rows.to_vec();
    let mut pivots = 0;
    for col in 0..5 {
        let pivot = mat[pivots..]
            .iter()
            .position(|row| row[col].abs() > 1e-12)
            .map(|p| p + pivots);
        let Some(pivot) = pivot else {
            continue;
        };
        mat.swap(pivot, pivots);
        let scale = mat[pivots][col];
        for v in &mut mat[pivots][col..5] {
            *v /= scale;
        }
        let pivot_row = mat[pivots].clone();
        for (i, row) in mat.iter_mut().enumerate() {
            if i == pivots {
                continue;
            }
            let f = row[col];
            if f.abs() < 1e-12 {
                continue;
            }
            for (v, &pv) in row[col..5].iter_mut().zip(&pivot_row[col..5]) {
                *v -= f * pv;
            }
        }
        pivots += 1;
    }
    mat[..pivots].to_vec()
}

fn nullspace(rows: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let rref = row_basis(rows);
    let mut pivot_cols: Vec<usize> = Vec::new();
    for row in &rref {
        if let Some(c) = row.iter().position(|&v| v.abs() > 1e-12) {
            pivot_cols.push(c);
        }
    }
    let mut basis: Vec<Vec<f64>> = Vec::new();
    for fc in 0..5 {
        if pivot_cols.contains(&fc) {
            continue;
        }
        let mut v = vec![0.0; 5];
        v[fc] = 1.0;
        for (i, &pc) in pivot_cols.iter().enumerate() {
            v[pc] = -rref[i][fc];
        }
        basis.push(v);
    }
    basis
}

fn wedge_rows(rows: &[Vec<f64>]) -> Multivector {
    let mut res = mv_scalar(1.0);
    for row in rows {
        res = res.op(&mv_vector(row[0], row[1], row[2], row[3], row[4]));
    }
    res
}

#[cfg(test)]
mod tests {
    use crate::*;

    #[allow(dead_code)]
    fn close(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_gp_basis() {
        let e1v = e1();
        let e2v = e2();
        let e3v = e3();
        let e0v = e0();
        let eiv = einf();
        assert!(e1v.gp(&e1v).approx_eq(&mv_scalar(1.0)));
        assert!(e2v.gp(&e2v).approx_eq(&mv_scalar(1.0)));
        assert!(e3v.gp(&e3v).approx_eq(&mv_scalar(1.0)));
        assert!(e0v.gp(&e0v).is_zero());
        assert!(eiv.gp(&eiv).is_zero());
        assert!(e0v.gp(&eiv).scalar_part() == -1.0);
        assert!(eiv.gp(&e0v).scalar_part() == -1.0);
    }

    #[test]
    fn test_null_point() {
        let p = mv_vector(1.0, 2.0, 3.0, 1.0, 7.0);
        assert!(p.gp(&p).scalar_part() == 0.0);
        assert_eq!(p.coords(), [1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_dual_involution() {
        let p = mv_vector(1.0, 2.0, 3.0, 1.0, 7.0);
        assert!(p.dual().dual().approx_eq(&p.neg()));
        assert!(p.undual().approx_eq(&p.dual().neg()));
    }

    #[test]
    fn test_reverse() {
        let b = e1().op(&e2());
        assert!(b.reverse().approx_eq(&b.neg()));

        assert!(mv_scalar(3.0).reverse().approx_eq(&mv_scalar(3.0)));
    }

    fn tb12() -> Multivector {
        e1().op(&e2())
    }

    fn tb13() -> Multivector {
        e1().op(&e3())
    }

    fn tb23() -> Multivector {
        e2().op(&e3())
    }

    #[test]
    fn test_blade_join() {
        assert!(e1().join(&e2()).approx_eq(&tb12()));

        assert!(e1().join(&tb12()).approx_eq(&tb12()));

        assert!(tb12()
            .join(&e0().op(&einf()))
            .approx_eq(&e1().op(&e2()).op(&e0()).op(&einf())));

        assert!(tb12().join(&tb23()).approx_eq(&e1().op(&e2()).op(&e3())));

        assert!(tb12()
            .join(&e1().op(&e3()).op(&e0()))
            .approx_eq(&e1().op(&e2()).op(&e3()).op(&e0())));

        assert!(e2().join(&e2().mul_scalar(-3.0)).approx_eq(&e2()));

        assert!(mv_scalar(2.0).join(&e1()).approx_eq(&e1()));
        assert!(mv_zero().join(&e1()).approx_eq(&e1()));
    }

    #[test]
    fn test_blade_meet() {
        assert!(e1().meet(&tb12()).approx_eq(&e1()));
        assert!(tb12().meet(&tb12()).approx_eq(&tb12()));
        assert!(e1().meet(&mv_scalar(3.0)).approx_eq(&mv_zero()));

        assert!(e1().meet(&e2()).approx_eq(&mv_zero()));
        assert!(e1().meet(&e0().op(&einf())).approx_eq(&mv_zero()));

        assert!(tb12().meet(&tb13()).approx_eq(&e1()));
        assert!(tb12().meet(&tb23()).approx_eq(&e2()));

        assert!(tb12().meet(&e1().op(&e3()).op(&e0())).approx_eq(&e1()));

        assert!(tb12().meet(&tb13().mul_scalar(-2.0)).approx_eq(&e1()));

        let mut whole = mv_zero();
        whole.values[31] = 1.0;
        assert!(whole.meet(&tb12()).approx_eq(&tb12().neg()));
    }

    #[test]
    fn test_dual_hodge() {
        let expected = e2().op(&e3()).op(&e0()).op(&einf()).neg();
        assert!(e1().dual().approx_eq(&expected));

        let p = point(1.0, 2.0, 3.0);
        assert!(p.dual().dual().approx_eq(&p.neg()));
    }

    #[test]
    fn test_blade_inverse() {
        assert!(tb12().inverse().approx_eq(&tb12().neg()));
        let v = mv_vector(1.0, 2.0, 3.0, 0.0, 0.0);
        assert!(v.gp(&v.inverse()).approx_eq(&mv_scalar(1.0)));
        let r = motor_rotor([0.0, 0.0, 1.0], 0.7);
        assert!(r.gp(&r.inverse()).approx_eq(&mv_scalar(1.0)));
        assert!(r.inverse().approx_eq(&r.reverse()));
    }

    #[test]
    fn test_blade_proj_rej() {
        let plane = tb12();
        let v = mv_vector(1.0, 2.0, 3.0, 0.0, 0.0);
        assert!(v
            .proj(&plane)
            .approx_eq(&mv_vector(1.0, 2.0, 0.0, 0.0, 0.0)));
        assert!(v.rej(&plane).approx_eq(&mv_vector(0.0, 0.0, 3.0, 0.0, 0.0)));
        assert!(v.proj(&plane).add(&v.rej(&plane)).approx_eq(&v));
        assert!(mv_vector(1.0, 2.0, 3.0, 0.0, 0.0)
            .proj(&e1())
            .approx_eq(&mv_vector(1.0, 0.0, 0.0, 0.0, 0.0)));
    }

    #[test]
    fn test_blade_reflect() {
        let v = mv_vector(1.0, 2.0, 3.0, 0.0, 0.0);
        assert!(v
            .reflect([0.0, 0.0, 1.0])
            .approx_eq(&mv_vector(1.0, 2.0, -3.0, 0.0, 0.0)));

        assert!(v
            .reflect([0.0, 0.0, 2.0])
            .approx_eq(&mv_vector(1.0, 2.0, -3.0, 0.0, 0.0)));
        assert!(e3().reflect([0.0, 0.0, 1.0]).approx_eq(&e3().neg()));
    }

    #[test]
    fn test_blade_contractions() {
        assert!(e1().lc(&tb12()).approx_eq(&e2()));
        assert!(tb12().lc(&e1()).approx_eq(&mv_zero()));
        assert!(tb12().rc(&e1()).approx_eq(&e2().neg()));
        assert!(e1().rc(&tb12()).approx_eq(&mv_zero()));
        assert!(e1().commutator(&e2()).approx_eq(&tb12()));
        assert!(e1().commutator(&e1()).approx_eq(&mv_zero()));
        assert!(e1().anticommutator(&e2()).approx_eq(&mv_zero()));
    }

    #[test]
    fn test_point_null_and_dist() {
        let p1 = point(0.0, 0.0, 0.0);
        assert!(close(p1.gp(&p1).values[0], 0.0, 1e-4));
        assert!(close(point_dist(&p1, &point(3.0, 4.0, 0.0)), 5.0, 1e-4));
    }

    #[test]
    fn test_line_incidence() {
        let l = line(&point(0.0, 0.0, 0.0), &point(1.0, 0.0, 0.0));
        assert!(close(point(2.0, 0.0, 0.0).op(&l).vmax(), 0.0, 1e-4));
        assert!(point(0.0, 1.0, 0.0).op(&l).vmax() > 1e-3);
    }

    #[test]
    fn test_plane_incidence() {
        let pi = plane([0.0, 0.0, 1.0], 2.0);
        assert!(close(point(0.3, -0.7, 2.0).ip(&pi).vmax(), 0.0, 1e-4));
        assert!(point(0.0, 0.0, 0.0).ip(&pi).vmax() > 1e-3);
    }

    #[test]
    fn test_sphere_incidence() {
        let s = sphere([1.0, 2.0, 3.0], 2.0);
        assert!(close(point(3.0, 2.0, 3.0).ip(&s).vmax(), 0.0, 1e-4));
        assert!(point(0.0, 0.0, 0.0).ip(&s).vmax() > 1e-3);
    }

    #[test]
    fn test_circle_incidence() {
        let c = circle([0.0, 0.0, 0.0], 1.0, [0.0, 0.0, 1.0]);
        assert!(close(point(0.0, 1.0, 0.0).ip(&c).vmax(), 0.0, 1e-4));
        assert!(point(0.0, 0.0, 1.0).ip(&c).vmax() > 1e-3);
        let cnu = circle([1.0, 2.0, 3.0], 2.0, [0.0, 0.0, 2.0]);
        assert!(close(point(3.0, 2.0, 3.0).ip(&cnu).vmax(), 0.0, 1e-4));
    }

    #[test]
    fn test_distances() {
        let pi = plane([0.0, 0.0, 1.0], 2.0);
        let s = sphere([1.0, 2.0, 3.0], 2.0);
        assert!(close(plane_dist(&pi, &point(0.0, 0.0, 5.0)), 3.0, 1e-4));
        assert!(close(sphere_dist(&s, &point(3.0, 2.0, 3.0)), 0.0, 1e-4));
        assert!(sphere_dist(&s, &point(5.0, 2.0, 3.0)) > 0.0);
        assert!(sphere_dist(&s, &point(1.0, 2.0, 3.0)) < 0.0);
    }

    #[test]
    fn test_meet_plane_plane() {
        let pi = plane([0.0, 0.0, 1.0], 2.0);
        let pi2 = plane([0.0, 1.0, 0.0], 1.0);
        let lm = pi.dual().meet(&pi2.dual());
        assert!(close(point(0.0, 1.0, 2.0).op(&lm).vmax(), 0.0, 1e-4));
        assert!(close(point(5.0, 1.0, 2.0).op(&lm).vmax(), 0.0, 1e-4));
    }

    #[test]
    fn test_meet_line_sphere() {
        let lz = line(&point(0.0, 0.0, -2.0), &point(0.0, 0.0, 2.0));
        let ppm = lz.meet(&sphere([0.0, 0.0, 0.0], 1.0).dual());
        assert!(close(point(0.0, 0.0, 1.0).op(&ppm).vmax(), 0.0, 1e-4));
        assert!(close(point(0.0, 0.0, -1.0).op(&ppm).vmax(), 0.0, 1e-4));
    }

    #[test]
    fn test_far_from_origin_dist() {
        assert!(close(
            point_dist(&point(1000.0, 0.0, 0.0), &point(1001.0, 0.0, 0.0)),
            1.0,
            1e-2
        ));
    }

    #[test]
    fn test_cylinder_distances() {
        let cy = cylinder([0.0, 0.0, 2.0], [0.0, 1.0, 0.0], 1.0);
        assert!(close(cylinder_dist(&cy, &point(1.0, 5.0, 2.0)), 0.0, 1e-4));
        assert!(close(cylinder_dist(&cy, &point(0.2, 0.0, 2.0)), -0.8, 1e-4));
        assert!(close(cylinder_dist(&cy, &point(3.0, -2.0, 2.0)), 2.0, 1e-4));
        assert!(close(cylinder_dist(&cy, &point(-1.0, 5.0, 2.0)), 0.0, 1e-4));
    }

    #[test]
    fn test_from_dual_after_motor() {
        let s_cam = translator([1.0, 2.0, 3.0]).apply(&sphere([0.0, 0.0, 0.0], 0.5));
        let (c, rho) = sphere_from_dual(&s_cam);
        assert!(close(c[0], 1.0, 1e-4) && close(c[1], 2.0, 1e-4) && close(c[2], 3.0, 1e-4));
        assert!(close(rho, 0.5, 1e-4));
        let pi_cam = translator([1.0, 2.0, 3.0]).apply(&plane([0.0, 1.0, 0.0], 0.0));
        assert!(close(pi_cam.einf_coeff(), 2.0, 1e-4));
    }
}
