#[derive(Clone, Copy, Debug)]
pub struct BoxGeometry {
    pub half: [f64; 3],
}
impl BoxGeometry {
    pub fn new(width: f64, height: f64, depth: f64) -> BoxGeometry {
        if width.min(height.min(depth)) <= 0.0 {
            panic!(
                "box dimensions must be > 0, got ({}, {}, {})",
                width, height, depth
            );
        }
        BoxGeometry {
            half: [width / 2.0, height / 2.0, depth / 2.0],
        }
    }
}
