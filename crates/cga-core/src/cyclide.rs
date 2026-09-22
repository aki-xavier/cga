use crate::primitives::{circle, point, sphere};
use crate::Multivector;

#[derive(Clone, Copy, Debug)]
pub struct DupinCyclide {
    pub a: f64,
    pub b: f64,
    pub d: f64,
    pub shift: [f64; 3],
}

pub fn dupin_cyclide(a: f64, b: f64, d: f64, shift: [f64; 3]) -> DupinCyclide {
    if !(a > b && b > 0.0) {
        panic!("need a > b > 0, got a={}, b={}", a, b);
    }
    if d <= 0.0 {
        panic!("need d > 0, got d={}", d);
    }
    DupinCyclide { a, b, d, shift }
}

impl DupinCyclide {
    pub fn c(&self) -> f64 {
        (self.a * self.a - self.b * self.b).sqrt()
    }

    pub fn kind(&self) -> &'static str {
        let c = self.c();
        if c < self.d && self.d < self.a {
            return "ring";
        }
        if self.d > self.a {
            return "spindle";
        }
        "horn"
    }

    pub fn spine(&self, u: f64) -> [f64; 3] {
        [self.a * u.cos(), self.b * u.sin(), 0.0]
    }

    pub fn radius(&self, u: f64) -> f64 {
        self.d - self.c() * u.cos()
    }

    pub fn generator_sphere(&self, u: f64) -> Multivector {
        let sp = self.spine(u);
        sphere(sp, self.radius(u))
    }

    pub fn focal_spheres(&self) -> (Multivector, Multivector) {
        let c = self.c();
        (
            sphere([c, 0.0, 0.0], self.a - self.d),
            sphere([-c, 0.0, 0.0], self.a + self.d),
        )
    }

    pub fn tangency_residual(&self, u: f64) -> [f64; 2] {
        let sp = self.spine(u);
        let r = self.radius(u);
        let c = self.c();
        let d1 = (sp[0] - c).hypot(sp[1]);
        let d2 = (sp[0] + c).hypot(sp[1]);
        [d1 - r - (self.a - self.d), d2 + r - (self.a + self.d)]
    }

    pub fn characteristic_circle(&self, u: f64) -> Multivector {
        let c = self.c();
        let cu = u.cos();
        let su = u.sin();
        let e = [self.a * cu, self.b * su, 0.0];
        let ep = [-self.a * su, self.b * cu, 0.0];
        let r = self.d - c * cu;
        let rp = c * su;
        let ep2 = self.a * self.a * su * su + self.b * self.b * cu * cu;
        if ep2 < 1e-18 {
            panic!("characteristic circle degenerates (zero tangent)");
        }
        let lam = (r * rp) / ep2;
        let center = [e[0] - lam * ep[0], e[1] - lam * ep[1], e[2] - lam * ep[2]];
        let rho2 = r * r - lam * lam * ep2;
        if rho2 <= 0.0 {
            panic!("characteristic circle radius non-positive (cusp/degenerate)");
        }
        circle(center, rho2.sqrt(), ep)
    }

    pub fn surface(&self, u: f64, v: f64) -> [f64; 3] {
        let a = self.a;
        let b = self.b;
        let c = self.c();
        let d = self.d;
        let cu = u.cos();
        let cv = v.cos();
        let su = u.sin();
        let sv = v.sin();
        let den = a - c * cu * cv;
        let x = (d * (c - a * cu * cv) + b * b * cu) / den;
        let y = (b * su * (a - d * cv)) / den;
        let z = (b * sv * (c * cu - d)) / den;
        [x + self.shift[0], y + self.shift[1], z + self.shift[2]]
    }

    pub fn implicit(&self, x: f64, y: f64, z: f64) -> f64 {
        let a = self.a;
        let b = self.b;
        let c = self.c();
        let d = self.d;
        let sx = x - self.shift[0];
        let sy = y - self.shift[1];
        let sz = z - self.shift[2];
        let bb = b * b - d * d;
        let rho = sx * sx + sy * sy + sz * sz;
        (rho + bb) * (rho + bb) - 4.0 * (a * sx - c * d) * (a * sx - c * d) - 4.0 * b * b * sy * sy
    }

    pub fn gradient(&self, x: f64, y: f64, z: f64) -> [f64; 3] {
        let a = self.a;
        let b = self.b;
        let c = self.c();
        let d = self.d;
        let sx = x - self.shift[0];
        let sy = y - self.shift[1];
        let sz = z - self.shift[2];
        let bb = b * b - d * d;
        let rho = sx * sx + sy * sy + sz * sz;
        let g = rho + bb;
        [
            4.0 * sx * g - 8.0 * a * (a * sx - c * d),
            4.0 * sy * g - 8.0 * b * b * sy,
            4.0 * sz * g,
        ]
    }

    pub fn normal(&self, x: f64, y: f64, z: f64) -> [f64; 3] {
        let g = self.gradient(x, y, z);
        let n = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
        if n < 1e-12 {
            return [0.0, 0.0, 1.0];
        }
        [g[0] / n, g[1] / n, g[2] / n]
    }

    pub fn contains(&self, x: f64, y: f64, z: f64) -> bool {
        self.implicit(x, y, z) < 0.0
    }

    pub fn uv(&self, x: f64, y: f64, z: f64) -> [f64; 2] {
        let a = self.a;
        let b = self.b;
        let c = self.c();
        let d = self.d;
        let sx = x - self.shift[0];
        let sy = y - self.shift[1];
        let sz = z - self.shift[2];
        let rho = sx * sx + sy * sy + sz * sz;
        let u = (2.0 * b * sy).atan2(2.0 * (a * sx - c * d));
        let v = (2.0 * b * sz).atan2(d * d + b * b - rho);
        [u, v]
    }

    pub fn inversion_versor(&self) -> Multivector {
        sphere([0.0, 0.0, 0.0], 1.0)
    }

    pub fn invert_point(&self, p: &Multivector) -> Multivector {
        let s = self.inversion_versor();
        let out = s.gp(p).gp(&s);
        let c = out.coords();
        point(c[0], c[1], c[2])
    }
}

pub fn from_torus_inversion(major: f64, minor: f64, shift_x: f64) -> DupinCyclide {
    let r = major;
    let rr = minor;
    if !(r > rr && rr > 0.0) {
        panic!(
            "need major > minor > 0, got major={}, minor={}",
            major, minor
        );
    }
    let s = shift_x;
    let xs = [s + r + rr, s + r - rr, s - r + rr, s - r - rr];
    for x in xs {
        if x.abs() < 1e-12 {
            panic!("torus passes through inversion centre; result not a ring");
        }
    }
    let mut ys = [1.0 / xs[0], 1.0 / xs[1], 1.0 / xs[2], 1.0 / xs[3]];
    ys.sort_by(|a: &f64, b: &f64| a.total_cmp(b));
    let y1 = ys[3];
    let y2 = ys[2];
    let y3 = ys[1];
    let y4 = ys[0];
    let a = 0.25 * (y1 + y2 - y3 - y4);
    let d = 0.25 * (y1 - y2 + y3 - y4);
    let c = 0.25 * (-y1 + y2 + y3 - y4);
    let m0 = 0.25 * (y1 + y2 + y3 + y4);
    if c.abs() < 1e-12 * 1.0_f64.max(a.abs()) {
        panic!("torus centred at inversion centre stays a torus (c=0); need shift_x != 0");
    }
    let b = (a * a - c * c).sqrt();
    dupin_cyclide(a, b, d, [m0, 0.0, 0.0])
}
