use mlx_rs::ops;
use mlx_rs::Array;

pub fn ck(r: Result<Array, mlx_rs::error::Exception>) -> Array {
    r.unwrap()
}

#[inline]
pub fn fs(v: f64) -> Array {
    Array::from_f32(v as f32)
}

pub fn arr3(a: f64, b: f64, c: f64) -> Array {
    Array::from_slice(&[a as f32, b as f32, c as f32], &[3])
}

pub fn arr3v(v: [f64; 3]) -> Array {
    arr3(v[0], v[1], v[2])
}

#[inline]
pub fn s_add(a: &Array, v: f64) -> Array {
    ck(a.add(fs(v)))
}

#[inline]
pub fn s_sub(a: &Array, v: f64) -> Array {
    ck(a.subtract(fs(v)))
}

#[inline]
pub fn s_mul(a: &Array, v: f64) -> Array {
    ck(a.multiply(fs(v)))
}

#[inline]
pub fn s_div(a: &Array, v: f64) -> Array {
    ck(a.divide(fs(v)))
}

#[inline]
pub fn s_pow(a: &Array, v: f64) -> Array {
    ck(a.power(fs(v)))
}

#[inline]
pub fn s_rsub(a: &Array, v: f64) -> Array {
    ck(fs(v).subtract(a))
}

#[inline]
pub fn s_rdiv(a: &Array, v: f64) -> Array {
    ck(fs(v).divide(a))
}

#[inline]
pub fn s_lt(a: &Array, v: f64) -> Array {
    ck(a.lt(fs(v)))
}

#[inline]
pub fn s_le(a: &Array, v: f64) -> Array {
    ck(a.le(fs(v)))
}

#[inline]
pub fn s_gt(a: &Array, v: f64) -> Array {
    ck(a.gt(fs(v)))
}

#[inline]
pub fn s_ge(a: &Array, v: f64) -> Array {
    ck(a.ge(fs(v)))
}

#[inline]
pub fn s_eq(a: &Array, v: f64) -> Array {
    ck(a.eq(fs(v)))
}

#[inline]
pub fn s_max(a: &Array, v: f64) -> Array {
    ck(ops::maximum(a, fs(v)))
}

#[inline]
pub fn s_min(a: &Array, v: f64) -> Array {
    ck(ops::minimum(a, fs(v)))
}

#[inline]
pub fn s_clip(a: &Array, lo: f64, hi: f64) -> Array {
    ck(ops::clip(a, (&fs(lo), &fs(hi))))
}
