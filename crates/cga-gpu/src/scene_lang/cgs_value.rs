use super::*;

#[derive(Clone, Debug)]
pub enum CgsValue {
    Num(f64),
    Bool(bool),
    Str(String),
    List(Vec<CgsValue>),
    Vec3(CgsVec3),
    Geom(GeomVal),
}
impl PartialEq for CgsValue {
    fn eq(&self, other: &CgsValue) -> bool {
        match (self, other) {
            (CgsValue::Num(a), CgsValue::Num(b)) => a == b,
            (CgsValue::Bool(a), CgsValue::Bool(b)) => a == b,
            (CgsValue::Str(a), CgsValue::Str(b)) => a == b,
            (CgsValue::List(a), CgsValue::List(b)) => a == b,
            (CgsValue::Vec3(a), CgsValue::Vec3(b)) => a.x == b.x && a.y == b.y && a.z == b.z,
            _ => false,
        }
    }
}
impl fmt::Display for CgsValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CgsValue::Num(x) => f.write_str(&fmt_f64(*x)),
            CgsValue::Bool(v) => f.write_str(if *v { "true" } else { "false" }),
            CgsValue::Str(s) => f.write_str(s),
            CgsValue::List(items) => {
                let parts: Vec<String> = items.iter().map(|v| format!("{v}")).collect();
                write!(f, "[{}]", parts.join(", "))
            }
            CgsValue::Vec3(v) => {
                write!(f, "[{}, {}, {}]", fmt_f64(v.x), fmt_f64(v.y), fmt_f64(v.z))
            }
            CgsValue::Geom(_) => f.write_str("<geom>"),
        }
    }
}
