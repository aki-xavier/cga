module cga

// Thin scalar-broadcast helpers over mlx-v's `Array`.  mlx-v has no
// `Array`-scalar operator overloads (V forbids mixed-type operator methods),
// and this module cannot extend `mlx.Array` (methods on foreign types are not
// allowed), so these are free functions.  All operate in float32, matching the
// render-kernel dtype.
import mlx

// fs returns a 0-d float32 scalar array (broadcasts in elementwise ops).
@[inline]
pub fn fs(v f64) mlx.Array {
	return mlx.f32_scalar(f32(v))
}

// arr3 builds a (3,) float32 array from three f64 values.
pub fn arr3(a f64, b f64, c f64) mlx.Array {
	return mlx.array_f32([f32(a), f32(b), f32(c)], [3])
}

// arr3v builds a (3,) float32 array from a [3]f64.
pub fn arr3v(v [3]f64) mlx.Array {
	return mlx.array_f32([f32(v[0]), f32(v[1]), f32(v[2])], [3])
}

@[inline]
pub fn s_add(a mlx.Array, v f64) mlx.Array {
	return a.add(fs(v))
}

@[inline]
pub fn s_sub(a mlx.Array, v f64) mlx.Array {
	return a.subtract(fs(v))
}

@[inline]
pub fn s_mul(a mlx.Array, v f64) mlx.Array {
	return a.multiply(fs(v))
}

@[inline]
pub fn s_div(a mlx.Array, v f64) mlx.Array {
	return a.divide(fs(v))
}

@[inline]
pub fn s_pow(a mlx.Array, v f64) mlx.Array {
	return a.power(fs(v))
}

@[inline]
pub fn s_rsub(a mlx.Array, v f64) mlx.Array {
	return fs(v).subtract(a)
}

@[inline]
pub fn s_rdiv(a mlx.Array, v f64) mlx.Array {
	return fs(v).divide(a)
}

@[inline]
pub fn s_lt(a mlx.Array, v f64) mlx.Array {
	return a.less(fs(v))
}

@[inline]
pub fn s_le(a mlx.Array, v f64) mlx.Array {
	return a.less_equal(fs(v))
}

@[inline]
pub fn s_gt(a mlx.Array, v f64) mlx.Array {
	return a.greater(fs(v))
}

@[inline]
pub fn s_ge(a mlx.Array, v f64) mlx.Array {
	return a.greater_equal(fs(v))
}

@[inline]
pub fn s_eq(a mlx.Array, v f64) mlx.Array {
	return a.equal(fs(v))
}

@[inline]
pub fn s_max(a mlx.Array, v f64) mlx.Array {
	return a.maximum(fs(v))
}

@[inline]
pub fn s_min(a mlx.Array, v f64) mlx.Array {
	return a.minimum(fs(v))
}

@[inline]
pub fn s_clip(a mlx.Array, lo f64, hi f64) mlx.Array {
	return a.clip(fs(lo), fs(hi))
}
