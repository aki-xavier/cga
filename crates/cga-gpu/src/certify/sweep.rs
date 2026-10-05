use super::*;

pub(crate) struct Sweep<'a> {
    pub(crate) q: &'a Quartic,
    pub(crate) certs: &'a mut Vec<(f64, f64)>,
    pub(crate) budget: u32,
    pub(crate) unknown: bool,
}
impl Sweep<'_> {
    pub(crate) fn visit(&mut self, x: Interval, depth: u32) {
        if self.unknown {
            return;
        }
        let p = self.q.eval_iv(x);
        if p.is_empty() || p.inf() > 0.0 || p.sup() < 0.0 {
            return;
        }

        let fp = self.q.deriv_iv(x);
        if !fp.is_empty() && !fp.contains(0.0) {
            if let (Some(a), Some(b)) = (point_sign(self.q, x.inf()), point_sign(self.q, x.sup())) {
                if a == b {
                    return;
                }
            }
        }
        if self.budget == 0 {
            self.unknown = true;
            return;
        }
        self.budget -= 1;
        let mid = x.mid();
        let w = x.sup() - x.inf();
        let u = ulp64(mid).max(ulp64(x.inf())).max(ulp64(x.sup()));
        if depth >= 128 || w <= 8.0 * u {
            match probe_box(self.q, x) {
                Probe::Root(r, t) => push_root(self.certs, r, t),
                Probe::NoRoot => {}
                Probe::Unknown => {
                    let explained = self.certs.iter().any(|&(c, t)| {
                        let slack = 8.0 * u + t;
                        slack.is_finite() && c - slack <= x.sup() && c + slack >= x.inf()
                    });
                    if !explained {
                        self.unknown = true;
                    }
                }
            }
            return;
        }
        let lo = iv(x.inf(), mid);
        let hi = iv(mid, x.sup());
        self.visit(lo, depth + 1);
        self.visit(hi, depth + 1);
    }
}
