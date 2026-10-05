use super::*;

#[derive(Clone, Copy, Debug)]
pub struct Light {
    pub kind: LightKind,
    pub color: Color,
    pub intensity: f64,
    pub direction: [f64; 3],
    pub position: [f64; 3],
}
impl Light {
    pub fn ambient(color: Color, intensity: f64) -> Light {
        Light {
            kind: LightKind::Ambient,
            color,
            intensity,
            direction: [0.0; 3],
            position: [0.0; 3],
        }
    }

    pub fn directional(color: Color, intensity: f64, direction: [f64; 3]) -> Light {
        Light {
            kind: LightKind::Directional,
            color,
            intensity,
            direction: vec3_unit(direction),
            position: [0.0; 3],
        }
    }

    pub fn point(color: Color, intensity: f64, position: [f64; 3]) -> Light {
        Light {
            kind: LightKind::Point,
            color,
            intensity,
            direction: [0.0; 3],
            position,
        }
    }

    pub fn to_camera(&self, m: Multivector) -> Light {
        match self.kind {
            LightKind::Directional => {
                let d = m.apply(&Multivector::vector(
                    self.direction[0],
                    self.direction[1],
                    self.direction[2],
                    0.0,
                    0.0,
                ));
                Self::directional(self.color, self.intensity, d.dir3())
            }
            LightKind::Point => {
                let c = m
                    .apply(&Multivector::point(
                        self.position[0],
                        self.position[1],
                        self.position[2],
                    ))
                    .coords();
                Self::point(self.color, self.intensity, c)
            }
            LightKind::Ambient => *self,
        }
    }

    pub fn direction_at(&self, p: &Array) -> (Array, Array) {
        match self.kind {
            LightKind::Directional => {
                let ld = ck(ops::broadcast_to(arr3v(self.direction), p.shape()));
                (ld, fs(self.intensity))
            }
            LightKind::Point => {
                let lv = ck(ops::broadcast_to(arr3v(self.position), p.shape())).subtract(p);
                let lv = ck(lv);
                let dist2 = ck(ck(lv.multiply(&lv)).sum_axes(&[-1], true));
                let ld = ck(lv.divide(ck(dist2.sqrt())));
                let atten = s_rdiv(&s_add(&s_div(&dist2, 8.0), 1.0), self.intensity);
                (ld, atten)
            }
            LightKind::Ambient => {
                panic!("ambient light is not part of the per-light loop")
            }
        }
    }

    pub fn far(&self, p: &Array) -> Array {
        if self.kind == LightKind::Point {
            let lv = ck(ops::broadcast_to(arr3v(self.position), p.shape())).subtract(p);
            let lv = ck(lv);
            return ck(ck(ck(lv.multiply(&lv)).sum_axes(&[-1], false)).sqrt());
        }
        fs(f64::INFINITY)
    }
}
