use inari::Interval;

pub mod quad_roots;
pub use self::quad_roots::*;
pub mod quartic;
pub use self::quartic::*;
pub mod quartic_roots;
pub use self::quartic_roots::*;
pub mod probe;
pub(crate) use self::probe::*;
pub mod sweep;
pub(crate) use self::sweep::*;

#[inline]
fn iv(a: f64, b: f64) -> Interval {
    Interval::try_from((a, b)).unwrap_or(Interval::EMPTY)
}

fn two_prod(a: f64, b: f64) -> Option<(f64, f64)> {
    if a == 0.0 || b == 0.0 {
        return Some((0.0, 0.0));
    }
    let hi = a * b;
    if !hi.is_normal() {
        return None;
    }
    let lo = f64::mul_add(a, b, -hi);
    if !lo.is_finite() {
        return None;
    }
    Some((hi, lo))
}

fn discriminant_sign(a: f64, b: f64, c: f64) -> Option<i8> {
    use std::cmp::Ordering;
    let (bh, bl) = two_prod(b, b)?;
    let (ph, pl) = two_prod(a, c)?;
    let qh = 4.0 * ph;
    let ql = 4.0 * pl;
    if !qh.is_finite() || !ql.is_finite() || qh * 0.25 != ph || ql * 0.25 != pl {
        return None;
    }
    match bh.partial_cmp(&qh) {
        Some(Ordering::Greater) => Some(1),
        Some(Ordering::Less) => Some(-1),
        _ => match bl.partial_cmp(&ql) {
            Some(Ordering::Greater) => Some(1),
            Some(Ordering::Less) => Some(-1),
            Some(Ordering::Equal) => Some(0),
            None => None,
        },
    }
}

type Cx = (f64, f64);

#[inline]
fn c_add(a: Cx, b: Cx) -> Cx {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: Cx, b: Cx) -> Cx {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: Cx, b: Cx) -> Cx {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_div(a: Cx, b: Cx) -> Cx {
    let m = b.0 * b.0 + b.1 * b.1;
    ((a.0 * b.0 + a.1 * b.1) / m, (a.1 * b.0 - a.0 * b.1) / m)
}

fn c_eval(q: &Quartic, z: Cx) -> Cx {
    let mut h = c_add(z, (q.c3, 0.0));
    h = c_add(c_mul(h, z), (q.c2, 0.0));
    h = c_add(c_mul(h, z), (q.c1, 0.0));
    c_add(c_mul(h, z), (q.c0, 0.0))
}

fn dk_seeds(rad: f64) -> [Cx; 4] {
    const S: [Cx; 4] = [(0.4, 0.9), (-0.65, 0.72), (-0.74, -0.67), (0.73, -0.68)];
    S.map(|(re, im)| (re * rad, im * rad))
}

fn dk_iterate(q: &Quartic, z: &mut [Cx; 4]) {
    for _ in 0..50 {
        let mut next = *z;
        for (i, zi) in z.iter().enumerate() {
            let p = c_eval(q, *zi);
            let mut den: Cx = (1.0, 0.0);
            for (j, zj) in z.iter().enumerate() {
                if j != i {
                    den = c_mul(den, c_sub(*zi, *zj));
                }
            }
            next[i] = c_sub(*zi, c_div(p, den));
        }
        *z = next;
    }
}

fn ulp64(x: f64) -> f64 {
    let a = x.abs();
    if a.is_normal() {
        a.next_up() - a
    } else if a.is_finite() {
        f64::MIN_POSITIVE
    } else {
        f64::NAN
    }
}

fn push_root(certs: &mut Vec<(f64, f64)>, r: f64, t: f64) {
    for c in certs.iter_mut() {
        let scale = c.1.max(t).max(ulp64(c.0)).max(ulp64(r));
        if scale.is_finite() && (c.0 - r).abs() <= 8.0 * scale {
            c.1 = c.1.max(t);
            return;
        }
    }
    certs.push((r, t));
}

fn point_sign(q: &Quartic, t: f64) -> Option<i8> {
    let p = q.eval_iv(iv(t, t));
    if p.is_empty() {
        None
    } else if p.inf() > 0.0 {
        Some(1)
    } else if p.sup() < 0.0 {
        Some(-1)
    } else {
        None
    }
}

fn ambiguity_lemma(q: &Quartic, m: f64) -> Option<(f64, f64)> {
    let pe = q.eval_iv(iv(m, m));
    if pe.is_empty() {
        return None;
    }
    let e = pe.inf().abs().max(pe.sup().abs());
    let mut w = ulp64(m);
    if !w.is_finite() || w <= 0.0 {
        w = f64::MIN_POSITIVE;
    }
    for _ in 0..64 {
        if !w.is_finite() {
            return None;
        }
        let fpi = q.deriv_iv(iv(m - w, m + w));
        if !fpi.is_empty() && !fpi.contains(0.0) {
            let g = if fpi.inf() > 0.0 {
                fpi.inf()
            } else {
                -fpi.sup()
            };

            let d = (e / g) * (1.0 + 8.0 * f64::EPSILON);
            if d <= w {
                return Some((m, d));
            }
        }
        w *= 4.0;
    }
    None
}

fn probe_box(q: &Quartic, x: Interval) -> Probe {
    if x.is_empty() {
        return Probe::NoRoot;
    }
    let p = q.eval_iv(x);
    if p.is_empty() || p.inf() > 0.0 || p.sup() < 0.0 {
        return Probe::NoRoot;
    }
    let fp = q.deriv_iv(x);
    if fp.is_empty() {
        return Probe::Unknown;
    }
    if fp.contains(0.0) {
        return Probe::Unknown;
    }
    let (mut lo, mut hi) = (x.inf(), x.sup());

    let a = match point_sign(q, lo) {
        Some(s) => s,
        None => {
            return match ambiguity_lemma(q, lo) {
                Some((r, t)) => Probe::Root(r, t),
                None => Probe::Unknown,
            };
        }
    };
    let b = match point_sign(q, hi) {
        Some(s) => s,
        None => {
            return match ambiguity_lemma(q, hi) {
                Some((r, t)) => Probe::Root(r, t),
                None => Probe::Unknown,
            };
        }
    };
    if a == b {
        return Probe::NoRoot;
    }

    for _ in 0..128 {
        let w = hi - lo;
        let mid = lo + w * 0.5;
        if w <= 8.0 * ulp64(mid) {
            return Probe::Root(mid, 0.5 * w);
        }
        match point_sign(q, mid) {
            None => {
                return match ambiguity_lemma(q, mid) {
                    Some((r, t)) => Probe::Root(r, t),
                    None => Probe::Unknown,
                };
            }
            Some(c) => {
                if c == a {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
        }
    }
    Probe::Unknown
}

const SWEEP_BUDGET: u32 = 16_384;

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::DupinCyclide;

    #[test]
    fn quadratic_exact_double_root() {
        assert_eq!(QuadRoots::solve(1.0, -4.0, 4.0), QuadRoots::Double(2.0));
        assert_eq!(QuadRoots::solve(1.0, -2.0, 1.0), QuadRoots::Double(1.0));
    }

    #[test]
    fn quadratic_two_roots_and_proven_none() {
        assert_eq!(QuadRoots::solve(1.0, -3.0, 2.0), QuadRoots::Two(1.0, 2.0));
        assert_eq!(QuadRoots::solve(1.0, 0.0, 1.0), QuadRoots::None);
        assert_eq!(QuadRoots::solve(2.0, 4.0, 100.0), QuadRoots::None);
    }

    #[test]
    fn grazing_pair_below_old_guard_threshold() {
        let (a, b, c) = (1.0, -2.0, 1.0 - 1e-13);
        let naive = b * b - 4.0 * (a * c);
        assert!(
            naive > 0.0 && naive < 1e-12,
            "前置条件：常规 disc 在旧守卫零域"
        );
        match QuadRoots::solve(a, b, c) {
            QuadRoots::Two(t1, t2) => {
                assert!(
                    t1 < 1.0 && t2 > 1.0,
                    "两根应夹住切点 1.0，实际 ({t1}, {t2})"
                );
                assert!(
                    t2 - t1 < 1e-6,
                    "根间距应 ≈ √disc = 6.3e-7，实际 {}",
                    t2 - t1
                );
            }
            other => panic!("期望 Two，实际 {other:?}"),
        }
    }

    #[test]
    fn sub_ulp_positive_discriminant_exact_roots() {
        let a = 1.0f64.next_up();
        let c = 1.0f64.next_down().next_down();
        let b = -2.0;

        assert_eq!(b * b - 4.0 * (a * c), 0.0, "前置条件：常规计算落入零域");
        match QuadRoots::solve(a, b, c) {
            QuadRoots::Two(t1, t2) => {
                assert_eq!(t2, 1.0, "较大根应精确为 1");
                let gap = t2 - t1;
                assert!(
                    gap > 1e-17 && gap < 1e-15,
                    "根间距应 ≈ 2⁻⁵¹ = 4.4e-16，实际 {gap}"
                );
            }
            other => panic!("期望 Two，实际 {other:?}"),
        }
    }

    #[test]
    fn quadratic_degenerate_inputs_are_explicit() {
        assert_eq!(QuadRoots::solve(0.0, 2.0, -4.0), QuadRoots::One(2.0));
        assert_eq!(QuadRoots::solve(0.0, 0.0, 1.0), QuadRoots::None);
        assert_eq!(QuadRoots::solve(0.0, 0.0, 0.0), QuadRoots::Indeterminate);
        assert_eq!(
            QuadRoots::solve(f64::NAN, 1.0, 1.0),
            QuadRoots::Indeterminate
        );
        assert_eq!(
            QuadRoots::solve(1.0, f64::INFINITY, 1.0),
            QuadRoots::Indeterminate
        );

        assert_eq!(
            QuadRoots::solve(1e308, 1.0, 1e308),
            QuadRoots::Indeterminate
        );

        assert_eq!(
            QuadRoots::solve(f64::from_bits(1), 1.0, 1.0),
            QuadRoots::Indeterminate
        );
    }

    #[test]
    fn discriminant_sign_matches_exact_integer_reference() {
        let mut st: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut rnd = move || {
            st = st
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (st >> 33) as u32
        };
        for case in 0..600 {
            let e = (rnd() % 61) as i32 - 30;
            let m: i64 = (1 << 20) + (rnd() as i64 % (1 << 21));
            let n: i64 = (1 << 20) + (rnd() as i64 % (1 << 21));

            let (ma, mc, mb): (i64, i64, i64) = match rnd() % 4 {
                0 => (m * m, n * n, 2 * m * n),
                1 => (m * m, n * n + 1, 2 * m * n),
                2 => (m * m, n * n - 1, 2 * m * n),
                _ => (m * m, n * n, 2 * m * n + 1),
            };
            let (sa, sc): (i64, i64) = (
                if rnd() & 1 == 1 { -1 } else { 1 },
                if rnd() & 1 == 1 { -1 } else { 1 },
            );

            let d_int = (mb as i128) * (mb as i128)
                - 4 * (sa as i128) * (sc as i128) * (ma as i128) * (mc as i128);
            let want = d_int.signum() as i8;
            let pow = (2.0f64).powi(e);
            let a = (sa * ma) as f64 * pow;
            let b = (mb as f64) * pow * if rnd() & 1 == 1 { -1.0 } else { 1.0 };
            let c = (sc * mc) as f64 * pow;

            assert_eq!(
                discriminant_sign(a, b, c),
                Some(want),
                "case {case}: D_int = {d_int}"
            );
            match (want, QuadRoots::solve(a, b, c)) {
                (1, QuadRoots::Two(t1, t2)) => assert!(t1 < t2 && t1.is_finite() && t2.is_finite()),
                (0, QuadRoots::Double(t)) => assert!(t.is_finite()),
                (-1, QuadRoots::None) => {}
                (_, other) => panic!("case {case}: sign {want} 与分类 {other:?} 不一致"),
            }
        }
    }

    #[test]
    fn quartic_four_simple_roots_certified() {
        let q = Quartic {
            c3: -10.0,
            c2: 35.0,
            c1: -50.0,
            c0: 24.0,
        };
        match q.roots() {
            QuarticRoots::Certified(rs) => {
                assert_eq!(rs.len(), 4, "四根都应被证明：{rs:?}");
                for (got, want) in rs.iter().zip([1.0, 2.0, 3.0, 4.0]) {
                    assert!((got - want).abs() < 1e-9, "根 {got} ≠ {want}");
                }
            }
            other => panic!("期望 Certified([1,2,3,4])，实际 {other:?}"),
        }
    }

    #[test]
    fn quartic_no_real_roots_certified_empty() {
        let q = Quartic {
            c3: 0.0,
            c2: 0.0,
            c1: 0.0,
            c0: 1.0,
        };
        assert_eq!(q.roots(), QuarticRoots::Certified(Vec::new()));
    }

    #[test]
    fn quartic_double_root_reports_unknown() {
        let q = Quartic {
            c3: -7.0,
            c2: 17.0,
            c1: -17.0,
            c0: 6.0,
        };
        let r = q.roots();
        assert!(r.is_unknown(), "重根必须显式 Unknown，实际 {r:?}");
        let rs = r.roots();
        assert_eq!(rs.len(), 2, "单根 2、3 应已证：{rs:?}");
        assert!((rs[0] - 2.0).abs() < 1e-9 && (rs[1] - 3.0).abs() < 1e-9);
    }

    #[test]
    fn quartic_tangent_torus_ray_reports_unknown() {
        let q = Quartic::torus(2.0, 1.0, [3.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert_eq!(
            (q.c3, q.c2, q.c1, q.c0),
            (-20.0, 174.0, -740.0, 1225.0),
            "与 geometry_extra 推导同式"
        );
        let r = q.roots();
        assert!(r.is_unknown(), "相切必须 Unknown，实际 {r:?}");
        assert!(r.roots().is_empty(), "唯一实根是重根，无可证根：{r:?}");
    }

    #[test]
    fn quartic_transversal_torus_ray_certified() {
        let q = Quartic::torus(2.0, 1.0, [2.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert_eq!((q.c3, q.c2, q.c1, q.c0), (-20.0, 164.0, -640.0, 960.0));
        match q.roots() {
            QuarticRoots::Certified(rs) => {
                assert_eq!(rs.len(), 2, "实根恰为 4、6：{rs:?}");
                assert!((rs[0] - 4.0).abs() < 1e-9 && (rs[1] - 6.0).abs() < 1e-9);
            }
            other => panic!("期望 Certified([4,6])，实际 {other:?}"),
        }
    }

    #[test]
    fn quartic_cyclide_ray_roots_satisfy_implicit() {
        let cy = DupinCyclide::new(1.0, 0.6, 1.5, [0.0, 0.0, 0.0]);
        let s = cy.surface(0.7, 1.1);
        assert!(
            cy.implicit(s[0], s[1], s[2]).abs() < 1e-10,
            "表面点前置条件"
        );
        let o = [s[0], s[1], s[2] + 4.0];
        let d = [0.0, 0.0, -1.0];
        let q = Quartic::cyclide(cy.a, cy.b, cy.d, cy.c(), cy.shift, o, d);
        let r = q.roots();
        assert!(!r.is_unknown(), "横截 cyclide 射线应可完全证明，实际 {r:?}");
        let rs = r.roots();
        assert!(
            rs.iter().any(|&t| (t - 4.0).abs() < 1e-9),
            "应证出构造根 t=4：{rs:?}"
        );
        for &t in rs {
            let (x, y, z) = (o[0], o[1], o[2] + d[2] * t);
            let f = cy.implicit(x, y, z);

            let (bb, sx, sy) = (cy.b * cy.b - cy.d * cy.d, x - cy.shift[0], y - cy.shift[1]);
            let rho = sx * sx + sy * sy + (z - cy.shift[2]) * (z - cy.shift[2]);
            let lin = cy.a * sx - cy.c() * cy.d;
            let scale = (rho + bb) * (rho + bb) + 4.0 * lin * lin + 4.0 * cy.b * cy.b * sy * sy;
            assert!(
                f.abs() <= 1e-9 * scale + 1e-9,
                "根 t={t} 回代残差 {f} 超出相对量级 {scale}"
            );
        }
    }

    #[test]
    fn quartic_nonfinite_coeffs_unknown() {
        let q = Quartic {
            c3: f64::NAN,
            c2: 1.0,
            c1: 1.0,
            c0: 1.0,
        };
        assert_eq!(
            q.roots(),
            QuarticRoots::Unknown {
                certified: Vec::new()
            }
        );
    }
}
