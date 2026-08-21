module cga

// Surface baking: "primitive + residual detail" represented as a MESH
// tessellated on the primitive's uv grid (exportable as glTF via save_glb).
// Replaces the old ray-marched DisplacedGeometry renderer: for each grid node
// the base surface point p0 and normal n are evaluated exactly, a ray is cast
// along +/-n against the target geometry, and the mesh vertex is placed at
// p0 + residual * n.
//
// SEMANTIC NOTE: the baked mesh IS the visible surface (base + detail
// combined); the base primitive remains the algebraic/semantic handle.  Do
// NOT render the base and the baked mesh together — they coincide wherever
// the residual is 0 and would z-fight.
//
// Supported bases (local frames; pose the mesh via the Object3D motor):
//   sphere    u = atan2(y,x)/(2pi)+0.5, node u_i = i/res_u (u WRAPS, faces
//             share the seam column); v_j = j/(res_v-1) in [0,1], poles at
//             the end rows (degenerate pole triangles are skipped, giving
//             triangle fans)
//   cylinder  INFINITE, axis +z: u as sphere (wrap); v_j covers the axial
//             band [0, 2r]
//   plane     bounded patch: node (i, j) at (i, j) world units from the plane
//             origin along the in-plane axes b1 = e1 projected on the plane
//             (e2 if nearly parallel), b2 = b1 x n
//   cyclide   canonical (a,b,d,shift); u_i = 2pi*i/res_u, v_j = 2pi*j/res_v,
//             BOTH wrap (seam-stitched, no duplicate rows)
// Other bases (ellipsoid/torus/cone/box/trimesh/csg/finite cylinder) panic.
//
// Boundary loops (the cylinder band edges, the plane patch border) are closed
// automatically: each open loop is capped with a planar fill (Newell normal +
// dominant-axis projection + ear clipping, modeling.v), wound so the cap
// normal points AWAY from the mesh interior (derived from the boundary edges'
// direction in the surface triangles, not assumed).  Non-planar loops still
// get a cap (the projection flattens them); the cap is only exact for
// near-planar loops.
//
// Bake conventions (same as the old residual baker): ray p0 + eps*n + t*n
// hits the target => residual = +(eps + t) (outward bump); else the reverse
// ray hits at t => residual = eps - t (dimple); both miss => residual = 0
// (vertex stays on the base surface).  Two guards keep knife-edge geometry
// honest: hits are accepted only when the hit normal faces the base
// (dot(n_hit, n_base) > 0.3 — rejects in-plane grazes and far-wall
// through-hits; assumes the target is wound outward) and only when
// |displacement| <= the base characteristic length (rejects far hits reported
// when a feature-plane-aligned ray makes the kernel skip the true near hit).
import math
import mlx

// BakedSurface is the baked mesh as raw vertices + faces (for save_glb).
pub struct BakedSurface {
pub:
	vertices [][3]f64
	faces    [][3]int
}

// check_bake_base validates the base kind (constructors panic elsewhere).
fn check_bake_base(base Geometry, res_u int, res_v int) {
	match base {
		SphereGeometry, PlaneGeometry, CyclideGeometry {}
		CylinderGeometry {
			if base.half > 0.0 {
				panic('bake base cylinder must be infinite (cylinder length <= 0)')
			}
		}
		else {
			panic('bake base must be sphere/plane/cylinder/cyclide, got ${base.type_name()}')
		}
	}
	if res_u < 3 || res_v < 2 {
		panic('bake grid must be >= 3x2, got ${res_u}x${res_v}')
	}
}

// base_surface_node evaluates the base surface point and outward normal at
// grid node (i, j) in LOCAL space (see the header for conventions).
fn base_surface_node(base Geometry, i int, j int, res_u int, res_v int) ([3]f64, [3]f64) {
	u := f64(i) / f64(res_u)
	phi := 2.0 * math.pi * (u - 0.5) // atan2 argument at node u
	match base {
		SphereGeometry {
			theta := math.pi * f64(j) / f64(res_v - 1)
			st := math.sin(theta)
			dir := [st * math.cos(phi), st * math.sin(phi), math.cos(theta)]!
			r := base.radius
			return [r * dir[0], r * dir[1], r * dir[2]]!, dir
		}
		CylinderGeometry {
			r := base.radius
			v := f64(j) / f64(res_v - 1)
			return [r * math.cos(phi), r * math.sin(phi), 2.0 * r * v]!, [
				math.cos(phi),
				math.sin(phi),
				0.0,
			]!
		}
		PlaneGeometry {
			n, d, b1, b2 := plane_frame(base.blade)
			// one node per world unit: node (i, j) at (i, j)
			return [n[0] * d + b1[0] * f64(i) + b2[0] * f64(j),
				n[1] * d + b1[1] * f64(i) + b2[1] * f64(j),
				n[2] * d + b1[2] * f64(i) +
					b2[2] * f64(j)]!, n
		}
		CyclideGeometry {
			cy := dupin_cyclide(base.a, base.b, base.d, base.shift)
			p := cy.surface(2.0 * math.pi * u, 2.0 * math.pi * f64(j) / f64(res_v))
			return p, cy.normal(p[0], p[1], p[2])
		}
		else {
			panic('bake base must be sphere/plane/cylinder/cyclide')
		}
	}
}

// plane_frame extracts (unit normal, distance, in-plane axes b1/b2) from a
// plane blade.  b1 = e1 projected onto the plane (e2 when e1 is nearly
// parallel to n), b2 = b1 x n.
fn plane_frame(blade Multivector) ([3]f64, f64, [3]f64, [3]f64) {
	n := vec3_unit(blade.euclidean_vector())
	d := blade.einf_coeff()
	mut seed := [1.0, 0.0, 0.0]!
	if math.abs(n[0]) >= 0.9 {
		seed = [0.0, 1.0, 0.0]!
	}
	sn := vec3_dot(seed, n)
	b1 := vec3_unit([seed[0] - sn * n[0], seed[1] - sn * n[1], seed[2] - sn * n[2]]!)
	b2 := vec3_cross(b1, n)
	return n, d, b1, b2
}

// bake_char_len returns a characteristic length of the base (ray epsilon).
fn bake_char_len(base Geometry) f64 {
	return match base {
		SphereGeometry { base.radius }
		CylinderGeometry { base.radius }
		CyclideGeometry { base.a }
		PlaneGeometry { 1.0 }
		else { 1.0 }
	}
}

// bake_surface bakes `target` onto `base`'s uv grid as a triangle mesh, in
// LOCAL space (both geometries unposed).  Rays along the node normals are
// cast through the standard MLX geom_intersect machinery (identity motor).
pub fn bake_surface(base Geometry, target Geometry, res_u int, res_v int) BakedSurface {
	check_bake_base(base, res_u, res_v)
	n_nodes := res_u * res_v
	eps := 1e-3 * bake_char_len(base)
	mut o_flat := []f32{len: 3 * n_nodes}
	mut n_flat := []f32{len: 3 * n_nodes}
	for j in 0 .. res_v {
		for i in 0 .. res_u {
			p0, n := base_surface_node(base, i, j, res_u, res_v)
			k := 3 * (j * res_u + i)
			o_flat[k] = f32(p0[0] + eps * n[0])
			o_flat[k + 1] = f32(p0[1] + eps * n[1])
			o_flat[k + 2] = f32(p0[2] + eps * n[2])
			n_flat[k] = f32(n[0])
			n_flat[k + 1] = f32(n[1])
			n_flat[k + 2] = f32(n[2])
		}
	}
	o := mlx.array_f32(o_flat, [n_nodes, 3])
	nd := mlx.array_f32(n_flat, [n_nodes, 3])
	tp := geom_to_camera(target, motor_identity())
	t_out, n_out, m_out := geom_intersect(tp, o, nd)
	t_in, n_in, m_in := geom_intersect(tp, o, nd.negative())
	// Hit acceptance: |dot(n_hit, n_base)| > 0.3 (NOTE: the intersection
	// kernels flip normals toward the ray origin, so the sign carries no
	// near/far information — the magnitude test rejects in-plane grazes, e.g.
	// a ray running in a flat cap's plane).  Far-wall through-hits are caught
	// by the distance bound below.
	fo := n_out.multiply(nd).sum_axis(-1, false).abs()
	fi := n_in.multiply(nd).sum_axis(-1, false).abs()
	ms_o := m_out.data_bool()
	ms_i := m_in.data_bool()
	fo_v := fo.data_f32()
	fi_v := fi.data_f32()
	ts_o := t_out.data_f32()
	ts_i := t_in.data_f32()
	// Distance bound: knife-edge alignment between a ray and a target feature
	// plane (e.g. a band-edge node level with a flat cap) can make the batched
	// trimesh kernel skip the true near hit and report a far one — a hit
	// farther than the base's characteristic length is never a meaningful
	// displacement, so treat it as a miss.
	char_len := bake_char_len(base)
	mut verts := [][3]f64{len: n_nodes}
	for k in 0 .. n_nodes {
		mut r := f64(0.0)
		if ms_o[k] && fo_v[k] > 0.3 && eps + f64(ts_o[k]) <= char_len {
			r = eps + f64(ts_o[k])
		} else if ms_i[k] && fi_v[k] > 0.3 && f64(ts_i[k]) - eps <= char_len {
			r = eps - f64(ts_i[k])
		}
		p0, n := base_surface_node(base, k % res_u, k / res_u, res_u, res_v)
		verts[k] = [p0[0] + r * n[0], p0[1] + r * n[1], p0[2] + r * n[2]]!
	}
	o.free()
	nd.free()
	// faces: quads (two tris each), stitched across the u seam (and the v seam
	// for cyclide); degenerate triangles (sphere poles) are skipped
	u_wrap := base is SphereGeometry || base is CylinderGeometry || base is CyclideGeometry
	v_wrap := base is CyclideGeometry
	mut faces := [][3]int{}
	v_steps := if v_wrap { res_v } else { res_v - 1 }
	for j in 0 .. v_steps {
		j1 := (j + 1) % res_v
		i_steps := if u_wrap { res_u } else { res_u - 1 }
		for i in 0 .. i_steps {
			i1 := (i + 1) % res_u
			a := j * res_u + i
			b := j * res_u + i1
			c := j1 * res_u + i1
			d := j1 * res_u + i
			push_tri(verts, mut faces, a, b, c)
			push_tri(verts, mut faces, a, c, d)
		}
	}
	cap_boundary_loops(verts, mut faces)
	return BakedSurface{
		vertices: verts
		faces:    faces
	}
}

// push_tri appends a triangle unless it is degenerate (zero area, e.g. the
// sphere pole rows where two vertices coincide).
fn push_tri(verts [][3]f64, mut faces [][3]int, a int, b int, c int) {
	pa := verts[a]
	pb := verts[b]
	pc := verts[c]
	ea := [pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]]!
	eb := [pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]]!
	cr := vec3_cross(ea, eb)
	if vec3_dot(cr, cr) < 1e-20 {
		return
	}
	faces << [a, b, c]!
}

// --- boundary-loop capping ----------------------------------------------------

// newell3 returns the Newell normal of a 3D polygon (robust for non-planar
// and non-convex loops).
fn newell3(poly [][3]f64) [3]f64 {
	mut n := [0.0, 0.0, 0.0]!
	for i in 0 .. poly.len {
		p := poly[i]
		q := poly[(i + 1) % poly.len]
		n[0] += (p[1] - q[1]) * (p[2] + q[2])
		n[1] += (p[2] - q[2]) * (p[0] + q[0])
		n[2] += (p[0] - q[0]) * (p[1] + q[1])
	}
	return n
}

// cap_boundary_loops finds every boundary loop (edges used by exactly one
// triangle) and closes it with a planar-fill cap.  Boundary edges are chained
// DIRECTED (as they appear in the consistently wound surface triangles), and
// the cap polygon is the reversed chain: that winding makes the cap's normal
// point away from the mesh interior.  Non-planar loops are flattened by the
// projection (near-planar loops are exact).
fn cap_boundary_loops(verts [][3]f64, mut faces [][3]int) {
	// edge -> (count, directed edge as found in its triangle)
	mut edge_count := map[u64]int{}
	mut edge_dir := map[u64][2]int{}
	for f in faces {
		for k in 0 .. 3 {
			a := f[k]
			b := f[(k + 1) % 3]
			key := u64(math.min(a, b)) << 32 | u64(math.max(a, b))
			edge_count[key] = edge_count[key] + 1
			edge_dir[key] = [a, b]!
		}
	}
	// outgoing map of directed boundary edges: in a consistently oriented
	// manifold each boundary vertex has exactly one outgoing boundary edge
	mut out_count := map[int]int{}
	for key, cnt in edge_count {
		if cnt == 1 {
			d := edge_dir[key]
			out_count[d[0]] = out_count[d[0]] + 1
		}
	}
	mut next := map[int]int{}
	for key, cnt in edge_count {
		if cnt == 1 {
			d := edge_dir[key]
			next[d[0]] = d[1]
		}
	}
	if next.len == 0 {
		return
	}
	mut used := map[int]bool{}
	for start, _ in next {
		if start in used {
			continue
		}
		mut loop_ := []int{}
		mut cur := start
		for {
			used[cur] = true
			loop_ << cur
			nxt := next[cur] or { break }
			if nxt == start || nxt in used {
				break
			}
			cur = nxt
		}
		if loop_.len >= 3 {
			cap_loop(verts, mut faces, loop_)
		}
	}
}

// cap_loop caps one directed boundary loop.
fn cap_loop(verts [][3]f64, mut faces [][3]int, loop_ []int) {
	// cap polygon = reversed loop (opposite winding to the boundary edges in
	// the surface triangles => outward-pointing cap normal)
	mut poly := [][3]f64{cap: loop_.len}
	for i := loop_.len - 1; i >= 0; i-- {
		p := verts[loop_[i]]
		poly << p
	}
	n := newell3(poly)
	if vec3_dot(n, n) < 1e-24 {
		return
	}
	// project to 2D dropping the dominant axis of n, keeping a cyclic
	// (orientation-preserving) coordinate order so 2D CCW <=> normal along +n
	mut p2 := [][2]f64{cap: poly.len}
	ax := if math.abs(n[0]) >= math.abs(n[1]) && math.abs(n[0]) >= math.abs(n[2]) {
		0
	} else if math.abs(n[1]) >= math.abs(n[2]) {
		1
	} else {
		2
	}
	for p in poly {
		pt := match ax {
			0 { [p[1], p[2]]! }
			1 { [p[2], p[0]]! }
			else { [p[0], p[1]]! }
		}
		p2 << pt
	}
	// ear clipping always returns CCW-in-2D triangles, which is the intended
	// winding only when n's dominant component is positive; verify against
	// the Newell normal in 3D and flip if the triangulation came out reversed
	tris := triangulate(p2)
	mut flip := false
	if tris.len > 0 {
		t0 := tris[0]
		pa := poly[t0[0]]
		pb := poly[t0[1]]
		pc := poly[t0[2]]
		cr := vec3_cross([pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]]!, [pc[0] - pa[0], pc[1] - pa[1],
			pc[2] - pa[2]]!)
		flip = vec3_dot(cr, n) < 0.0
	}
	for t in tris {
		// map back through the reversed polygon indices
		i0 := loop_[loop_.len - 1 - t[0]]
		i1 := loop_[loop_.len - 1 - t[1]]
		i2 := loop_[loop_.len - 1 - t[2]]
		if flip {
			push_tri(verts, mut faces, i0, i2, i1)
		} else {
			push_tri(verts, mut faces, i0, i1, i2)
		}
	}
}

// bake_surface_mesh bakes and returns a renderable TrimeshGeometry.
pub fn bake_surface_mesh(base Geometry, target Geometry, res_u int, res_v int) TrimeshGeometry {
	b := bake_surface(base, target, res_u, res_v)
	return trimesh_geometry(b.vertices, b.faces)
}
