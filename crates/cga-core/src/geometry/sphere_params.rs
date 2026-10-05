#[derive(Clone, Copy, Debug)]
pub struct SphereParams {
    pub c: [f64; 3],
    pub r: f64,
    pub axes: [[f64; 3]; 3],
}
