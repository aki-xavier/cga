use super::*;

#[derive(Clone, Copy, Debug)]
pub struct BoltCircleSpec {
    pub n: u32,
    pub radius: f64,
    pub y: f64,
    pub bolt_r: f64,
    pub bolt_h: f64,
    pub color: u32,
}
impl Default for BoltCircleSpec {
    fn default() -> Self {
        Self {
            n: 8,
            radius: 2.0,
            y: 0.55,
            bolt_r: 0.13,
            bolt_h: 0.7,
            color: 0x6E747A,
        }
    }
}
impl BoltCircleSpec {
    pub fn generate(&self) -> String {
        let mut out = String::new();
        for i in 0..self.n {
            let a = i as f64 * 2.0 * std::f64::consts::PI / self.n as f64;
            let hw = self.bolt_r * 3.2;
            out.push_str(&format!(
            "rotate(axis=[0.0, 1.0, 0.0], angle={}) translate([{}, {}, 0.0]) {{\n  material(color=0x{:06X}, roughness=0.4, metalness=0.7) {}\n  translate([0.0, {}, 0.0]) material(color=0x{:06X}, roughness=0.4, metalness=0.7) box(s=[{}, 0.18, {}]);\n}}\n",
            fmt_num(a),
            fmt_num(self.radius),
            fmt_num(self.y),
            self.color,
            gen_post(self.bolt_r, self.bolt_h),
            fmt_num(self.bolt_h / 2.0 + 0.09),
            self.color,
            fmt_num(hw),
            fmt_num(hw)
        ));
        }
        out
    }
}
