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
}
// 注：批量光照方法 direction_at/far（MLX Array）留在 cga-gpu 的
// shading::{light_direction_at, light_far}——那是渲染器的活，不是场景模型的。
