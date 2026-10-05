#[derive(Debug)]
pub(crate) enum Probe {
    Root(f64, f64),

    NoRoot,

    Unknown,
}
