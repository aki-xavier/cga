use super::*;

#[derive(Clone, Debug)]
pub enum ArgValue {
    Num(f64),
    Bool(bool),
    Str(String),
    List(Vec<ArgValue>),
    Vec3(ArgVec3),
}
impl PartialEq for ArgValue {
    fn eq(&self, other: &ArgValue) -> bool {
        match (self, other) {
            (ArgValue::Num(a), ArgValue::Num(b)) => a == b,
            (ArgValue::Bool(a), ArgValue::Bool(b)) => a == b,
            (ArgValue::Str(a), ArgValue::Str(b)) => a == b,
            (ArgValue::List(a), ArgValue::List(b)) => a == b,
            (ArgValue::Vec3(a), ArgValue::Vec3(b)) => a.x == b.x && a.y == b.y && a.z == b.z,
            _ => false,
        }
    }
}
impl fmt::Display for ArgValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ArgValue::Num(x) => f.write_str(&fmt_f64(*x)),
            ArgValue::Bool(v) => f.write_str(if *v { "true" } else { "false" }),
            ArgValue::Str(s) => f.write_str(s),
            ArgValue::List(items) => {
                let parts: Vec<String> = items.iter().map(|v| format!("{v}")).collect();
                write!(f, "[{}]", parts.join(", "))
            }
            ArgValue::Vec3(v) => {
                write!(f, "[{}, {}, {}]", fmt_f64(v.x), fmt_f64(v.y), fmt_f64(v.z))
            }
        }
    }
}
