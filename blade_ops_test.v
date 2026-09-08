module cga

fn tb12() Multivector {
	return e1().op(e2())
}

fn tb13() Multivector {
	return e1().op(e3())
}

fn tb23() Multivector {
	return e2().op(e3())
}

fn test_blade_join() {
	assert e1().join(e2()).eq(tb12())
	// containment joins to the larger blade
	assert e1().join(tb12()).eq(tb12())
	// independent euclidean and conjugate blocks
	assert tb12().join(e0().op(einf())).eq(e1().op(e2()).op(e0()).op(einf()))
	// overlapping euclidean planes join to the 3-space they span
	assert tb12().join(tb23()).eq(e1().op(e2()).op(e3()))
	// 5D cross-grade overlap: e12 and e1e3e0 span 4 dimensions
	assert tb12().join(e1().op(e3()).op(e0())).eq(e1().op(e2()).op(e3()).op(e0()))
	// parallel blades share the span
	assert e2().join(e2().mul_scalar(-3.0)).eq(e2())
	// scalar / zero join as neutral
	assert mv_scalar(2.0).join(e1()).eq(e1())
	assert mv_zero().join(e1()).eq(e1())
}

fn test_blade_meet() {
	// containment cases the pure dual formula misses
	assert e1().meet(tb12()).eq(e1())
	assert tb12().meet(tb12()).eq(tb12())
	assert e1().meet(mv_scalar(3.0)).eq(mv_zero())
	// disjoint blades
	assert e1().meet(e2()).eq(mv_zero())
	assert e1().meet(e0().op(einf())).eq(mv_zero())
	// overlapping euclidean planes meet in their common line
	assert tb12().meet(tb13()).eq(e1())
	assert tb12().meet(tb23()).eq(e2())
	// 5D cross-grade partial overlap: e12 ∩ e1e3e0 = e1
	assert tb12().meet(e1().op(e3()).op(e0())).eq(e1())
	// input scale does not matter
	assert tb12().meet(tb13().mul_scalar(-2.0)).eq(e1())
	// meet with the whole space: dual(dual(x)) = -x
	mut whole := Multivector{}
	whole.values[31] = 1.0
	assert whole.meet(tb12()).eq(tb12().neg())
}

fn test_dual_hodge() {
	// dual is now the true Hodge dual: dual(e1) = -(e2^e3^e0^einf)
	expected := e2().op(e3()).op(e0()).op(einf()).neg()
	assert e1().dual().eq(expected)
	// double dual inverts sign (restated for the fix)
	p := point(1.0, 2.0, 3.0)
	assert p.dual().dual().eq(p.neg())
}

fn test_blade_inverse() {
	assert tb12().inverse().eq(tb12().neg())
	v := mv_vector(1.0, 2.0, 3.0, 0.0, 0.0)
	assert v.gp(v.inverse()).eq(mv_scalar(1.0))
	r := motor_rotor([0.0, 0.0, 1.0]!, 0.7)
	assert r.gp(r.inverse()).eq(mv_scalar(1.0))
	assert r.inverse().eq(r.reverse())
}

fn test_blade_proj_rej() {
	plane := tb12()
	v := mv_vector(1.0, 2.0, 3.0, 0.0, 0.0)
	assert v.proj(plane).eq(mv_vector(1.0, 2.0, 0.0, 0.0, 0.0))
	assert v.rej(plane).eq(mv_vector(0.0, 0.0, 3.0, 0.0, 0.0))
	assert v.proj(plane).add(v.rej(plane)).eq(v)
	assert mv_vector(1.0, 2.0, 3.0, 0.0, 0.0).proj(e1()).eq(mv_vector(1.0, 0.0, 0.0, 0.0, 0.0))
}

fn test_blade_reflect() {
	v := mv_vector(1.0, 2.0, 3.0, 0.0, 0.0)
	assert v.reflect([0.0, 0.0, 1.0]!).eq(mv_vector(1.0, 2.0, -3.0, 0.0, 0.0))
	// the normal is normalized internally
	assert v.reflect([0.0, 0.0, 2.0]!).eq(mv_vector(1.0, 2.0, -3.0, 0.0, 0.0))
	assert e3().reflect([0.0, 0.0, 1.0]!).eq(e3().neg())
}

fn test_blade_contractions() {
	assert e1().lc(tb12()).eq(e2())
	assert tb12().lc(e1()).eq(mv_zero())
	assert tb12().rc(e1()).eq(e2().neg())
	assert e1().rc(tb12()).eq(mv_zero())
	assert e1().commutator(e2()).eq(tb12())
	assert e1().commutator(e1()).eq(mv_zero())
	assert e1().anticommutator(e2()).eq(mv_zero())
}
