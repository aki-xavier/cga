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
    /// 从降幂系数 `[c3, c2, c1, c0]` 构造（`t⁴ + c3 t³ + c2 t² + c1 t + c0`）。
    /// 系数由 `cga_core` 的 `ray` 模块给出（唯一几何定义），本模块只负责求根与认证。
    pub fn from_coeffs(c: [f64; 4]) -> Quartic {
        Quartic {
            c3: c[0],
            c2: c[1],
            c1: c[2],
            c0: c[3],
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
