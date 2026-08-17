module cga

fn test_gp_basis() {
	e1v := e1()
	e2v := e2()
	e3v := e3()
	e0v := e0()
	eiv := einf()
	assert e1v.gp(e1v).eq(mv_scalar(1.0))
	assert e2v.gp(e2v).eq(mv_scalar(1.0))
	assert e3v.gp(e3v).eq(mv_scalar(1.0))
	assert e0v.gp(e0v).is_zero()
	assert eiv.gp(eiv).is_zero()
	assert e0v.gp(eiv).scalar_part() == -1.0
	assert eiv.gp(e0v).scalar_part() == -1.0
}

fn test_null_point() {
	// conformal point p = e0 + x e1 + y e2 + z e3 + 0.5 r^2 einf is null
	p := mv_vector(1.0, 2.0, 3.0, 1.0, 7.0)
	assert p.gp(p).scalar_part() == 0.0
	assert p.coords() == [1.0, 2.0, 3.0]!
}

fn test_dual_involution() {
	p := mv_vector(1.0, 2.0, 3.0, 1.0, 7.0)
	assert p.dual().dual().eq(p.neg())
	assert p.undual().eq(p.dual().neg())
}

fn test_reverse() {
	// reversal of a bivector flips sign
	b := e1().op(e2()) // e12
	assert b.reverse().eq(b.neg())
	// reversal of scalar is identity
	assert mv_scalar(3.0).reverse().eq(mv_scalar(3.0))
}
