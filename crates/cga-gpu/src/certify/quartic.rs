use super::*;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quartic {
    pub c3: f64,
    pub c2: f64,
    pub c1: f64,
    pub c0: f64,
}
impl Quartic {
    pub(crate) fn eval_iv(&self, x: Interval) -> Interval {
        (((x + iv(self.c3, self.c3)) * x + iv(self.c2, self.c2)) * x + iv(self.c1, self.c1)) * x
            + iv(self.c0, self.c0)
    }

    pub(crate) fn deriv_iv(&self, x: Interval) -> Interval {
        (((x * iv(4.0, 4.0) + iv(3.0 * self.c3, 3.0 * self.c3)) * x
            + iv(2.0 * self.c2, 2.0 * self.c2))
            * x)
            + iv(self.c1, self.c1)
    }
}
impl Quartic {
    pub fn torus(major: f64, minor: f64, o: [f64; 3], d: [f64; 3]) -> Quartic {
        let r2 = major * major;
        let oo = o[0] * o[0] + o[1] * o[1] + o[2] * o[2];
        let od = o[0] * d[0] + o[1] * d[1] + o[2] * d[2];
        let g = oo + r2 - minor * minor;
        Quartic {
            c3: 4.0 * od,
            c2: 2.0 * g + 4.0 * od * od - 4.0 * r2 * (d[0] * d[0] + d[1] * d[1]),
            c1: 4.0 * od * g - 8.0 * r2 * (o[0] * d[0] + o[1] * d[1]),
            c0: g * g - 4.0 * r2 * (o[0] * o[0] + o[1] * o[1]),
        }
    }
}
impl Quartic {
    #[allow(clippy::too_many_arguments)]
    pub fn cyclide(
        a: f64,
        b: f64,
        dd: f64,
        c: f64,
        shift: [f64; 3],
        o: [f64; 3],
        d: [f64; 3],
    ) -> Quartic {
        let ox = o[0] - shift[0];
        let oy = o[1] - shift[1];
        let oz = o[2] - shift[2];
        let (dx, dy) = (d[0], d[1]);
        let big_a = ox * ox + oy * oy + oz * oz;
        let b1 = ox * dx + oy * dy + oz * d[2];
        let bb = b * b - dd * dd;
        let g = big_a + bb;
        let p0 = ox * a - c * dd;
        let p1 = dx * a;
        Quartic {
            c3: 4.0 * b1,
            c2: 2.0 * g + 4.0 * b1 * b1 - 4.0 * p1 * p1 - 4.0 * b * b * dy * dy,
            c1: 4.0 * b1 * g - 8.0 * p0 * p1 - 8.0 * b * b * oy * dy,
            c0: g * g - 4.0 * p0 * p0 - 4.0 * b * b * oy * oy,
        }
    }
}
impl Quartic {
    pub fn roots(&self) -> QuarticRoots {
        if !(self.c3.is_finite()
            && self.c2.is_finite()
            && self.c1.is_finite()
            && self.c0.is_finite())
        {
            return QuarticRoots::Unknown {
                certified: Vec::new(),
            };
        }
        let rad = 1.0
            + self
                .c3
                .abs()
                .max(self.c2.abs())
                .max(self.c1.abs())
                .max(self.c0.abs());
        let mut certs: Vec<(f64, f64)> = Vec::new();

        let mut z = dk_seeds(rad);
        dk_iterate(self, &mut z);
        for (zr, _) in z {
            if !zr.is_finite() {
                continue;
            }
            let r0 = (rad * 1e-6).max(8.0 * ulp64(zr));
            if !r0.is_finite() || !iv(zr - r0, zr + r0).is_common_interval() {
                continue;
            }
            if let Probe::Root(r, t) = probe_box(self, iv(zr - r0, zr + r0)) {
                push_root(&mut certs, r, t);
            }
        }

        let mut sweep = Sweep {
            q: self,
            certs: &mut certs,
            budget: SWEEP_BUDGET,
            unknown: false,
        };
        sweep.visit(iv(-rad, rad), 0);
        let unknown = sweep.unknown;

        certs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let certs: Vec<f64> = certs.into_iter().map(|(v, _)| v).collect();
        if unknown {
            QuarticRoots::Unknown { certified: certs }
        } else {
            QuarticRoots::Certified(certs)
        }
    }
}
