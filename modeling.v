module cga

// Modeling builders: ear-clipping triangulation + extrude / loft.
// Pure data transforms producing watertight triangle meshes (vertices, faces).

import math

// --- ear clipping -----------------------------------------------------------

// signed_area returns the signed area of a 2D profile (CCW positive).
pub fn signed_area(profile [][2]f64) f64 {
	n := profile.len
	mut acc := 0.0
	for i in 0 .. n {
		j := (i + 1) % n
		acc += profile[i][0] * profile[j][1] - profile[j][0] * profile[i][1]
	}
	return 0.5 * acc
}

fn cross2(o [2]f64, a [2]f64, b [2]f64) f64 {
	return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

fn in_tri(p [2]f64, a [2]f64, b [2]f64, c [2]f64) bool {
	return cross2(a, b, p) >= -1e-12 && cross2(b, c, p) >= -1e-12 && cross2(c, a,
		p) >= -1e-12
}

// triangulate ear-clips a simple polygon (no self-intersection, no holes) into
// CCW triangle indices.
pub fn triangulate(profile [][2]f64) [][3]int {
	n := profile.len
	if n < 3 {
		panic('profile needs >= 3 points, got ${n}')
	}
	mut pts := profile.clone()
	mut idx := []int{len: n}
	for i in 0 .. n {
		idx[i] = i
	}
	if signed_area(pts) < 0.0 {
		// reverse to CCW
		mut rp := []f64{len: n * 2}
		for i in 0 .. n {
			rp[2 * i] = pts[n - 1 - i][0]
			rp[2 * i + 1] = pts[n - 1 - i][1]
		}
		for i in 0 .. n {
			pts[i] = [rp[2 * i], rp[2 * i + 1]]!
		}
		for i in 0 .. n {
			idx[i] = n - 1 - i
		}
	}
	if math.abs(signed_area(pts)) < 1e-12 {
		panic('profile is degenerate (zero area / collinear)')
	}
	mut tris := [][3]int{}
	for idx.len > 3 {
		m := idx.len
		mut clipped := false
		for i in 0 .. m {
			i0 := idx[(i + m - 1) % m]
			i1 := idx[i]
			i2 := idx[(i + 1) % m]
			a := pts[i0]
			b := pts[i1]
			c := pts[i2]
			if cross2(a, b, c) <= 1e-12 {
				continue // concave or collinear, not an ear
			}
			mut inside := false
			for j in idx {
				if j == i0 || j == i1 || j == i2 {
					continue
				}
				if in_tri(pts[j], a, b, c) {
					inside = true
					break
				}
			}
			if inside {
				continue
			}
			tris << [i0, i1, i2]!
			idx.delete(i)
			clipped = true
			break
		}
		if !clipped {
			panic('ear clipping failed: profile likely self-intersects (${idx.len} vertices left)')
		}
	}
	tris << [idx[0], idx[1], idx[2]]!
	return tris
}

// --- extrude / loft ---------------------------------------------------------

fn ccw(profile [][2]f64) [][2]f64 {
	mut pts := profile.clone()
	if signed_area(pts) < 0.0 {
		mut rev := [][2]f64{len: pts.len}
		for i in 0 .. pts.len {
			rev[i] = pts[pts.len - 1 - i]
		}
		return rev
	}
	return pts
}

// extrude extrudes a 2D profile along +Z from 0 to height.
pub fn extrude(profile [][2]f64, height f64) ([][3]f64, [][3]int) {
	if height <= 0.0 {
		panic('extrude height must be > 0, got ${height}')
	}
	pts := ccw(profile)
	n := pts.len
	mut verts := [][3]f64{}
	for p in pts {
		verts << [p[0], p[1], 0.0]!
	}
	for p in pts {
		verts << [p[0], p[1], height]!
	}
	mut faces := [][3]int{}
	for i in 0 .. n {
		j := (i + 1) % n
		faces << [i, j, n + j]!
		faces << [i, n + j, n + i]!
	}
	for tri in triangulate(pts) {
		faces << [n + tri[0], n + tri[1], n + tri[2]]!
		faces << [tri[2], tri[1], tri[0]]!
	}
	return verts, faces
}

// loft lofts equal-vertex-count profiles at strictly increasing zs.
pub fn loft(profiles [][][2]f64, zs []f64) ([][3]f64, [][3]int) {
	if profiles.len != zs.len || profiles.len < 2 {
		panic('loft needs >= 2 profiles with matching zs, got ${profiles.len} profiles / ${zs.len} zs')
	}
	for i in 0 .. zs.len - 1 {
		if zs[i] >= zs[i + 1] {
			panic('loft zs must be strictly increasing, got ${zs}')
		}
	}
	m := profiles[0].len
	if m < 3 {
		panic('loft profiles must have >= 3 vertices')
	}
	for p in profiles {
		if p.len != m {
			panic('loft profiles must share the same vertex count (>= 3)')
		}
	}
	mut verts := [][3]f64{}
	for k, prof in profiles {
		for p in prof {
			verts << [p[0], p[1], zs[k]]!
		}
	}
	mut faces := [][3]int{}
	for k in 0 .. profiles.len - 1 {
		base := k * m
		for i in 0 .. m {
			j := (i + 1) % m
			faces << [base + i, base + j, base + m + j]!
			faces << [base + i, base + m + j, base + m + i]!
		}
	}
	top_off := (profiles.len - 1) * m
	for tri in triangulate(ccw(profiles[profiles.len - 1])) {
		faces << [top_off + tri[0], top_off + tri[1], top_off + tri[2]]!
	}
	for tri in triangulate(ccw(profiles[0])) {
		faces << [tri[2], tri[1], tri[0]]!
	}
	return verts, faces
}
