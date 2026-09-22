use crate::{einf, mv_vector, Multivector};

pub fn point(x: f64, y: f64, z: f64) -> Multivector {
    let r2 = x * x + y * y + z * z;
    mv_vector(x, y, z, 1.0, 0.5 * r2)
}

pub fn point_pair(p1: &Multivector, p2: &Multivector) -> Multivector {
    p1.op(p2)
}

pub fn line(p1: &Multivector, p2: &Multivector) -> Multivector {
    p1.op(p2).op(&einf())
}

pub fn plane(normal: [f64; 3], distance: f64) -> Multivector {
    let mut nx = normal[0];
    let mut ny = normal[1];
    let mut nz = normal[2];
    let nl = (nx * nx + ny * ny + nz * nz).sqrt();
    if nl <= 1e-12 {
        panic!("plane normal vector is zero or degenerate");
    }
    nx /= nl;
    ny /= nl;
    nz /= nl;
    mv_vector(nx, ny, nz, 0.0, distance)
}

pub fn sphere(center: [f64; 3], radius: f64) -> Multivector {
    let half = 0.5 * radius * radius;
    point(center[0], center[1], center[2]).sub(&mv_vector(0.0, 0.0, 0.0, 0.0, half))
}

pub fn sphere_from_dual(s: &Multivector) -> ([f64; 3], f64) {
    let w = s.e0_coeff();
    if w.abs() < 1e-12 {
        panic!("sphere multivector has no e0 component");
    }
    let v = s.euclidean_vector();
    let f = s.einf_coeff();
    let cx = v[0] / w;
    let cy = v[1] / w;
    let cz = v[2] / w;
    let mut rho_sq = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / (w * w) - 2.0 * f / w;
    if rho_sq < 0.0 {
        rho_sq = 0.0;
    }
    ([cx, cy, cz], rho_sq.sqrt())
}

pub fn circle(center: [f64; 3], radius: f64, normal: [f64; 3]) -> Multivector {
    let s = sphere(center, radius);
    let nl = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    let mut d = 0.0;
    if nl > 1e-12 {
        d = (center[0] * normal[0] + center[1] * normal[1] + center[2] * normal[2]) / nl;
    }
    let p = plane(normal, d);
    s.op(&p)
}

// 为什么: 圆柱没有单一 blade 代数形式——它由 axis Line blade 与 radius/axis_dir/axis_point 元数据组合而成，是一个 rebuilt primitive 而非纯 blade。
#[derive(Clone, Copy, Debug)]
pub struct Cylinder {
    pub blade: Multivector,
    pub radius: f64,
    pub axis_dir: [f64; 3],
    pub axis_point: [f64; 3],
}

pub fn cylinder(axis_point: [f64; 3], axis_dir: [f64; 3], radius: f64) -> Cylinder {
    let ax = axis_dir[0];
    let ay = axis_dir[1];
    let az = axis_dir[2];
    let al = (ax * ax + ay * ay + az * az).sqrt();
    if al <= 1e-12 {
        panic!("cylinder axis is degenerate");
    }
    let ux = ax / al;
    let uy = ay / al;
    let uz = az / al;
    let q = point(axis_point[0], axis_point[1], axis_point[2]);
    let q2 = point(axis_point[0] + ux, axis_point[1] + uy, axis_point[2] + uz);
    Cylinder {
        blade: line(&q, &q2),
        radius,
        axis_dir: [ux, uy, uz],
        axis_point,
    }
}

pub fn point_dist(a: &Multivector, b: &Multivector) -> f64 {
    let c1 = a.coords();
    let c2 = b.coords();
    let dx = c1[0] - c2[0];
    let dy = c1[1] - c2[1];
    let dz = c1[2] - c2[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

pub fn plane_dist(pi: &Multivector, p: &Multivector) -> f64 {
    let c = p.coords();
    let v = pi.euclidean_vector();
    let d = pi.einf_coeff();
    let nl = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if nl < 1e-12 {
        return 1e300;
    }
    (v[0] * c[0] + v[1] * c[1] + v[2] * c[2] - d) / nl
}

pub fn sphere_dist(s: &Multivector, p: &Multivector) -> f64 {
    let (c, r) = sphere_from_dual(s);
    let pc = p.coords();
    let dx = pc[0] - c[0];
    let dy = pc[1] - c[1];
    let dz = pc[2] - c[2];
    (dx * dx + dy * dy + dz * dz).sqrt() - r
}

pub fn cylinder_dist(cy: &Cylinder, p: &Multivector) -> f64 {
    let c = p.coords();
    let dx = c[0] - cy.axis_point[0];
    let dy = c[1] - cy.axis_point[1];
    let dz = c[2] - cy.axis_point[2];
    let ux = cy.axis_dir[0];
    let uy = cy.axis_dir[1];
    let uz = cy.axis_dir[2];
    let dot = dx * ux + dy * uy + dz * uz;
    let ex = dx - dot * ux;
    let ey = dy - dot * uy;
    let ez = dz - dot * uz;
    let d = (ex * ex + ey * ey + ez * ez).sqrt();
    d - cy.radius
}
