use super::*;

#[derive(Clone, Copy, Debug)]
pub struct GearSpec {
    pub y: f64,
    pub r_hub: f64,
    pub r_teeth: f64,
    pub n_teeth: u32,
    pub thick: f64,
    pub color: u32,
}
impl Default for GearSpec {
    fn default() -> Self {
        Self {
            y: 2.6,
            r_hub: 1.15,
            r_teeth: 1.75,
            n_teeth: 16,
            thick: 0.45,
            color: 0xC8A24A,
        }
    }
}
impl GearSpec {
    pub fn generate(&self) -> String {
        let mut out = format!(
            "translate([0.0, {}, 0.0]) material(color=0x{:06X}, roughness=0.3, metalness=0.8) {}\n",
            fmt_num(self.y),
            self.color,
            gen_post(self.r_hub, self.thick)
        );
        for i in 0..self.n_teeth {
            let a = i as f64 * 2.0 * std::f64::consts::PI / self.n_teeth as f64;
            out.push_str(&format!(
            "rotate(axis=[0.0, 1.0, 0.0], angle={}) translate([{}, {}, 0.0]) material(color=0x{:06X}, roughness=0.3, metalness=0.8) box(s=[{}, {}, 0.22]);\n",
            fmt_num(a),
            fmt_num(self.r_hub + (self.r_teeth - self.r_hub) / 2.0),
            fmt_num(self.y),
            self.color,
            fmt_num(self.r_teeth - self.r_hub + 0.12),
            fmt_num(self.thick)
        ));
        }
        out
    }
}
