use super::*;

#[derive(Clone, Debug)]
pub struct CgsToken {
    pub kind: TokenKind,
    pub text: String,
    pub num: f64,
    pub line: i32,
}
