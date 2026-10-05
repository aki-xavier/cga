#[derive(Debug, Clone, PartialEq)]
pub enum QuarticRoots {
    Certified(Vec<f64>),

    Unknown { certified: Vec<f64> },
}
impl QuarticRoots {
    pub fn roots(&self) -> &[f64] {
        match self {
            QuarticRoots::Certified(v) | QuarticRoots::Unknown { certified: v } => v,
        }
    }

    pub fn is_unknown(&self) -> bool {
        matches!(self, QuarticRoots::Unknown { .. })
    }
}
