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

/// 单位化。近零退化时返回 `[0, 0, 1]`（"没有方向 ⇒ +z"）——**注意**：工作区里另有一类
/// 本地 `*_unit` 的退化语义是 `[0, 0, 0]`（除法得不到方向就清零），两者不可互换；
/// 法线/轴向这类"必须有个方向"的场合用本函数，`cga-collision`/`kinematics` 的向量
/// 兜底用各自的本地版本（见那些文件的注释）。
pub fn vec3_unit(a: [f64; 3]) -> [f64; 3] {
    let n = vec3_len(a);
    if n < 1e-12 {
        return [0.0, 0.0, 1.0];
    }
    [a[0] / n, a[1] / n, a[2] / n]
}
