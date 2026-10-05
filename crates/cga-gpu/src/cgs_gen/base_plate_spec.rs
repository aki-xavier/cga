use super::*;

#[derive(Clone, Copy, Debug)]
pub struct BasePlateSpec {
    pub size: f64,
    pub thick: f64,
    pub hole_offset: f64,
    pub sink_r: f64,
    pub sink_h: f64,
    pub color: u32,
}
impl Default for BasePlateSpec {
    fn default() -> Self {
        Self {
            size: 7.0,
            thick: 0.3,
            hole_offset: 2.9,
            sink_r: 0.32,
            sink_h: 0.35,
            color: 0x4A4F54,
        }
    }
}
impl BasePlateSpec {
    pub fn generate(&self) -> String {
        let mut out = format!(
        "material(color=0x{:06X}, roughness=0.5, metalness=0.6)\ndifference() {{\n  box(s=[{}, {}, {}]);\n",
        self.color,
        fmt_num(self.size),
        fmt_num(self.thick),
        fmt_num(self.size)
    );
        for i in 0..4 {
            let a = i as f64 * std::f64::consts::PI / 2.0 + std::f64::consts::PI / 4.0;
            out.push_str(&format!(
            "  rotate(axis=[0.0, 1.0, 0.0], angle={}) translate([{}, 0.0, 0.0]) rotate(axis=[1.0, 0.0, 0.0], angle={}) cone(r={}, h={});\n",
            fmt_num(a),
            fmt_num(self.hole_offset),
            fmt_num(std::f64::consts::PI / 2.0),
            fmt_num(self.sink_r),
            fmt_num(self.sink_h)
        ));
        }
        out.push_str("}\n");
        out
    }
}
