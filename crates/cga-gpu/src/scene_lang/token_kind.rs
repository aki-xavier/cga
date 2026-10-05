use super::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TokenKind {
    Ident,
    Number,
    Op,
    Str,
    Eof,
    Lparen,
    Rparen,
    Lbracket,
    Rbracket,
    Lbrace,
    Rbrace,
    Comma,
    Semi,
    Assign,
    Dot,
}
impl fmt::Display for TokenKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            TokenKind::Ident => "ident",
            TokenKind::Number => "number",
            TokenKind::Op => "op",
            TokenKind::Str => "str",
            TokenKind::Eof => "eof",
            TokenKind::Lparen => "lparen",
            TokenKind::Rparen => "rparen",
            TokenKind::Lbracket => "lbracket",
            TokenKind::Rbracket => "rbracket",
            TokenKind::Lbrace => "lbrace",
            TokenKind::Rbrace => "rbrace",
            TokenKind::Comma => "comma",
            TokenKind::Semi => "semi",
            TokenKind::Assign => "assign",
            TokenKind::Dot => "dot",
        };
        f.write_str(s)
    }
}
