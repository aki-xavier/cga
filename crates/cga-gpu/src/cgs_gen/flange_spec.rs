use super::*;

#[derive(Clone, Copy, Debug)]
pub struct FlangeSpec {
    pub r_disc: f64,
    pub h_disc: f64,
    pub n_holes: u32,
    pub hole_circle_r: f64,
    pub hole_r: f64,
    pub hole_h: f64,
    pub center_hole_r: f64,
    pub color: u32,
}
impl Default for FlangeSpec {
    fn default() -> Self {
        Self {
            r_disc: 2.6,
            h_disc: 0.5,
            n_holes: 8,
            hole_circle_r: 2.0,
            hole_r: 0.16,
            hole_h: 2.0,
            center_hole_r: 0.58,
            color: 0x9BA1A6,
        }
    }
}
impl FlangeSpec {
    pub fn generate(&self) -> String {
        let mut out = format!(
            "material(color=0x{:06X}, roughness=0.35, metalness=0.75)\ndifference() {{\n  {}\n",
            self.color,
            gen_post(self.r_disc, self.h_disc)
        );
        for i in 0..self.n_holes {
            let a = i as f64 * 2.0 * std::f64::consts::PI / self.n_holes as f64;
            out.push_str(&format!(
                "  rotate(axis=[0.0, 1.0, 0.0], angle={}) translate([{}, 0.0, 0.0]) {}\n",
                fmt_num(a),
                fmt_num(self.hole_circle_r),
                gen_post(self.hole_r, self.hole_h)
            ));
        }
        out.push_str(&format!(
            "  {}\n}}\n",
            gen_post(self.center_hole_r, self.hole_h)
        ));
        out
    }
}
