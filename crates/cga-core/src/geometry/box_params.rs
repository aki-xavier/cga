#[derive(Clone, Copy, Debug)]
pub struct BoxParams {
    pub c: [f64; 3],
    pub axes: [[f64; 3]; 3],
    pub half: [f64; 3],
}
