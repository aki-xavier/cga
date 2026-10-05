use super::*;

#[derive(Clone, Debug)]
pub struct Truth {
    pub hit: Array,

    pub t: Array,

    pub normal: Array,

    pub index: Array,

    pub vis: Vec<Array>,
}
