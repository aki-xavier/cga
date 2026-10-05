use super::*;

#[derive(Clone, Debug)]
pub struct Scene {
    pub objects: Vec<Mesh>,
    pub lights: Vec<Light>,
    pub background: Color,
}
impl Scene {
    pub fn new(background: Option<Color>) -> Scene {
        Scene {
            objects: Vec::new(),
            lights: Vec::new(),
            background: match background {
                Some(b) => b,
                None => Color::from_hex(0x87CEEB),
            },
        }
    }

    pub fn add_mesh(&mut self, m: Mesh) {
        self.objects.push(m);
    }

    pub fn add_light(&mut self, l: Light) {
        self.lights.push(l);
    }
}
