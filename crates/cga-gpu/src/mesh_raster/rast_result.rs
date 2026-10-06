use super::*;

#[derive(Clone, Debug)]
pub struct RastResult {
    pub depth: Array,
    pub color: Array,
    /// 每像素不透明度（命中材质的 opacity；miss = 0），供合成端 alpha 混合。
    pub opacity: Array,
    pub hit: Array,
}

impl RastResult {
    pub fn empty(w: i32, h: i32) -> RastResult {
        RastResult {
            depth: ck(ops::full::<f32>(&[h, w], &fs(f64::INFINITY))),
            color: ck(ops::zeros::<f32>(&[h, w, 3])),
            opacity: ck(ops::zeros::<f32>(&[h, w])),
            hit: ck(ops::zeros::<bool>(&[h, w])),
        }
    }
}
