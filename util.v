module cga

// Small shared numeric helpers.

pub fn clamp01(v f64) f64 {
	if v < 0.0 {
		return 0.0
	}
	if v > 1.0 {
		return 1.0
	}
	return v
}
