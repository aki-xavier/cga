use super::*;

#[derive(Clone, Copy, Debug)]
pub struct OrbitControls {
    pub target: [f64; 3],
    pub azimuth: f64,
    pub elevation: f64,
    pub radius: f64,
}
impl OrbitControls {
    pub fn new(target: [f64; 3], azimuth: f64, elevation: f64, radius: f64) -> OrbitControls {
        OrbitControls {
            target,
            azimuth,
            elevation,
            radius,
        }
    }

    pub fn update(&self, camera: &mut PerspectiveCamera) {
        let ce = self.elevation.cos();
        let x = self.radius * ce * self.azimuth.sin();
        let y = self.radius * self.elevation.sin();
        let z = self.radius * ce * self.azimuth.cos();
        camera.position = [self.target[0] + x, self.target[1] + y, self.target[2] + z];
        camera.look_at(self.target, None);
    }
}
