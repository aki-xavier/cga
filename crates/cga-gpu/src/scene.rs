// Mesh, Scene, PerspectiveCamera and OrbitControls (three.js-style surface).

use crate::scene_graph::{color_hex, identity3, object3d, vec3_dot, vec3_unit, Color, Object3D};
use crate::shading::{Light, Material};
use cga_core::{motor_from_matrix, motor_identity, vec3_cross, Geometry, Multivector};

// Mesh binds a geometry, a material and a local pose (Object3D).
#[derive(Clone, Debug)]
pub struct Mesh {
    pub base: Object3D,
    pub geometry: Geometry,
    pub material: Material,
}

// MeshParams configures a Mesh (position/rotation_axis/rotation_angle define
// the pose; motor overrides them if given).
pub struct MeshParams {
    pub geometry: Geometry,
    pub material: Material,
    pub position: [f64; 3],
    pub rotation_axis: [f64; 3],
    pub rotation_angle: f64,
    pub motor: Option<Multivector>,
}

// mesh builds a Mesh from params.
pub fn mesh(p: MeshParams) -> Mesh {
    Mesh {
        base: object3d(
            p.position,
            p.rotation_axis,
            p.rotation_angle,
            p.motor,
            identity3(),
        ),
        geometry: p.geometry,
        material: p.material,
    }
}

// V struct embedding: Mesh.Object3D is flattened, so obj.position /
// obj.motor() resolve through to the embedded node.
impl std::ops::Deref for Mesh {
    type Target = Object3D;

    fn deref(&self) -> &Object3D {
        &self.base
    }
}

impl std::ops::DerefMut for Mesh {
    fn deref_mut(&mut self) -> &mut Object3D {
        &mut self.base
    }
}

// Scene holds the object list, the light list and the background colour.
#[derive(Clone, Debug)]
pub struct Scene {
    pub objects: Vec<Mesh>,
    pub lights: Vec<Light>,
    pub background: Color,
}

// scene builds a Scene (default sky-blue background).
pub fn scene(background: Option<Color>) -> Scene {
    Scene {
        objects: Vec::new(),
        lights: Vec::new(),
        background: match background {
            Some(b) => b,
            None => color_hex(0x87CEEB),
        },
    }
}

impl Scene {
    pub fn add_mesh(&mut self, m: Mesh) {
        self.objects.push(m);
    }

    pub fn add_light(&mut self, l: Light) {
        self.lights.push(l);
    }
}

// PerspectiveCamera is a pinhole camera (world->camera motor from look_at).
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

// perspective_camera builds a PerspectiveCamera.
pub fn perspective_camera(
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
        motor: motor_identity(),
    }
}

impl PerspectiveCamera {
    // look_at builds the world->camera motor (camera basis = {right, -up, forward}).
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
        self.motor = motor_from_matrix(mat, t);
    }
}

// OrbitControls is the static spherical orbit helper (update() repositions the
// camera).
#[derive(Clone, Copy, Debug)]
pub struct OrbitControls {
    pub target: [f64; 3],
    pub azimuth: f64,
    pub elevation: f64,
    pub radius: f64,
}

pub fn orbit_controls(
    target: [f64; 3],
    azimuth: f64,
    elevation: f64,
    radius: f64,
) -> OrbitControls {
    OrbitControls {
        target,
        azimuth,
        elevation,
        radius,
    }
}

impl OrbitControls {
    // update repositions the camera (spherical orbit around `target`).
    pub fn update(&self, camera: &mut PerspectiveCamera) {
        let ce = self.elevation.cos();
        let x = self.radius * ce * self.azimuth.sin();
        let y = self.radius * self.elevation.sin();
        let z = self.radius * ce * self.azimuth.cos();
        camera.position = [self.target[0] + x, self.target[1] + y, self.target[2] + z];
        camera.look_at(self.target, None);
    }
}
