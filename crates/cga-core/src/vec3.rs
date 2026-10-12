//! `[f64; 3]` 向量基本运算——全工作区唯一的定义处。
//!
//! 这三个函数曾在 mesh/collision/gpu/host/scene 里各有一份逐字相同的拷贝
//! （`dot`/`v3_dot`/`vdot`/`vec3_dot`、`scale`/`v3_scale`/`vscale`/`v3scale`、
//! `cross`/`v3cross`/`vec3_cross`）。统一到这里，调用点只留一个名字。

pub fn vec3_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn vec3_scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn vec3_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn vec3_add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn vec3_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn vec3_len(a: [f64; 3]) -> f64 {
    vec3_dot(a, a).sqrt()
}
