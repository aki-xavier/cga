pub(crate) enum DrillEnd {
    Num(f64),
    Face {
        name: String,
        axis: usize,
        sign: f64,
    },
}
