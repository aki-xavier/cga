use super::*;

#[derive(Clone, Copy, Debug)]
pub struct PerspectiveCamera {
    pub fov: f64,
    pub aspect: f64,
    pub near: f64,
    pub far: f64,
    pub position: [f64; 3],
    pub target: [f64; 3],
    pub up: [f64; 3],
    pub motor: Multivector,
}
impl PerspectiveCamera {
    pub fn new(
        fov: f64,
        aspect: f64,
        near: f64,
        far: f64,
        position: [f64; 3],
        target: [f64; 3],
        up: [f64; 3],
    ) -> PerspectiveCamera {
        if fov <= 0.0 || fov >= 180.0 {
            panic!("fov must be in (0, 180), got {}", fov);
        }
        PerspectiveCamera {
            fov,
            aspect,
            near,
            far,
            position,
            target,
            up: vec3_unit(up),
            motor: Multivector::identity(),
        }
    }

    pub fn look_at(&mut self, target: [f64; 3], up: Option<[f64; 3]>) {
        self.target = target;
        if let Some(u) = up {
            self.up = vec3_unit(u);
        }
        let f = vec3_unit([
            target[0] - self.position[0],
            target[1] - self.position[1],
            target[2] - self.position[2],
        ]);
        let r = vec3_unit(vec3_cross(f, self.up));
        let u = vec3_cross(r, f);
        let mut mat = identity3();
        mat[0] = r;
        mat[1] = [-u[0], -u[1], -u[2]];
        mat[2] = f;
        let t = [
            -vec3_dot(mat[0], self.position),
            -vec3_dot(mat[1], self.position),
            -vec3_dot(mat[2], self.position),
        ];
        self.motor = Multivector::from_matrix(mat, t);
    }
}
