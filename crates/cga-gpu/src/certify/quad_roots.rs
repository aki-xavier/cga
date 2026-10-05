use super::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QuadRoots {
    Two(f64, f64),

    Double(f64),

    None,

    One(f64),

    Indeterminate,
}
impl QuadRoots {
    pub fn solve(a: f64, b: f64, c: f64) -> QuadRoots {
        if !(a.is_finite() && b.is_finite() && c.is_finite()) {
            return QuadRoots::Indeterminate;
        }
        if a == 0.0 {
            return if b == 0.0 {
                if c == 0.0 {
                    QuadRoots::Indeterminate
                } else {
                    QuadRoots::None
                }
            } else {
                let t = -c / b;
                if t.is_finite() {
                    QuadRoots::One(t)
                } else {
                    QuadRoots::Indeterminate
                }
            };
        }
        match discriminant_sign(a, b, c) {
            None => QuadRoots::Indeterminate,
            Some(1) => {
                let (bh, bl) = match two_prod(b, b) {
                    Some(p) => p,
                    None => return QuadRoots::Indeterminate,
                };
                let (ph, pl) = match two_prod(a, c) {
                    Some(p) => p,
                    None => return QuadRoots::Indeterminate,
                };
                let scale = bh.abs() + 4.0 * ph.abs();
                if !scale.is_finite() {
                    return QuadRoots::Indeterminate;
                }
                let mut disc = (bh - 4.0 * ph) + (bl - 4.0 * pl);
                if !disc.is_finite() || disc <= 0.0 {
                    disc = scale.max(f64::MIN_POSITIVE) * f64::EPSILON;
                }
                let sq = disc.sqrt();
                if b == 0.0 {
                    let t = (-c / a).max(0.0).sqrt();
                    return if t.is_finite() {
                        QuadRoots::Two(-t, t)
                    } else {
                        QuadRoots::Indeterminate
                    };
                }
                let q = -0.5 * (b + b.signum() * sq);
                let (t1, t2) = (q / a, c / q);

                if t1.is_finite() && t2.is_finite() {
                    QuadRoots::Two(t1.min(t2), t1.max(t2))
                } else {
                    QuadRoots::Indeterminate
                }
            }
            Some(0) => {
                let t = -b / (2.0 * a);
                if t.is_finite() {
                    QuadRoots::Double(t)
                } else {
                    QuadRoots::Indeterminate
                }
            }
            _ => QuadRoots::None,
        }
    }
}
