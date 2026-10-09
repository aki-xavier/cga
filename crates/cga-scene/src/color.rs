use super::*;

#[derive(Clone, Copy, Debug)]
pub struct Color {
    pub r: f64,
    pub g: f64,
    pub b: f64,
}
impl Color {
    pub fn from_hex(c: i32) -> Color {
        Color {
            r: f64::from((c >> 16) & 0xFF) / 255.0,
            g: f64::from((c >> 8) & 0xFF) / 255.0,
            b: f64::from(c & 0xFF) / 255.0,
        }
    }

    pub fn from_rgb(r: f64, g: f64, b: f64) -> Color {
        Color { r, g, b }
    }

    pub fn rgb(&self) -> [f64; 3] {
        [
            srgb_to_linear(self.r),
            srgb_to_linear(self.g),
            srgb_to_linear(self.b),
        ]
    }
}
