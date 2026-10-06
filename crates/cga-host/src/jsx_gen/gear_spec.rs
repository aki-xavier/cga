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
            "    <Translate t={{[0.0, {}, 0.0]}}>{}</Translate>\n",
            fmt_num(self.y),
            gen_post_mat(self.r_hub, self.thick, self.color, 0.3, 0.8)
        );
        for i in 0..self.n_teeth {
            let a = i as f64 * 2.0 * std::f64::consts::PI / self.n_teeth as f64;
            out.push_str(&format!(
                "    <Rotate axis={{[0.0, 1.0, 0.0]}} angle={{ {} }}><Translate t={{[{}, {}, 0.0]}}><Box s={{[{}, {}, 0.22]}} color={{0x{:06X}}} roughness={{0.3}} metalness={{0.8}} /></Translate></Rotate>\n",
                fmt_num(a),
                fmt_num(self.r_hub + (self.r_teeth - self.r_hub) / 2.0),
                fmt_num(self.y),
                fmt_num(self.r_teeth - self.r_hub + 0.12),
                fmt_num(self.thick),
                self.color
            ));
        }
        out
    }
}
