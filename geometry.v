module cga

// Geometry types: CGA-blade primitives (sphere / plane / cylinder / box /
// circle) plus their per-frame camera-space parameters.  `geom_to_camera`
// computes the parameters (CPU-side versor conjugation); the per-pixel
// intersection kernels live in geometry_ops.v.
import math
import mlx

// --- per-geometry camera-space parameters ------------------------------------

pub struct SphereParams {
pub:
	c    [3]f64
	r    f64
	axes [3][3]f64
}

pub struct PlaneParams {
pub:
	n [3]f64
	d f64
}

pub struct CylinderParams {
pub:
	q [3]f64
	u [3]f64
	r f64
	h f64 // half-length; -1.0 means infinite cylinder
}

pub struct BoxParams {
pub:
	c    [3]f64
	axes [3][3]f64
	half [3]f64
}

pub struct CircleParams {
pub:
	c [3]f64
	n [3]f64
	r f64
}

pub struct ConeParams {
pub:
	a_inv3 Mat3
	t_inv  [3]f64
	a_fwd  [16]f64
	r      f64
	h      f64
}

pub struct TorusParams {
pub:
	a_inv3 Mat3
	t_inv  [3]f64
	a_fwd  [16]f64
	major  f64
	minor  f64
}

pub struct EllipsoidParams {
pub:
	a_inv3 Mat3
	t_inv  [3]f64
	a_fwd  [16]f64
}

pub struct CyclideParams {
pub:
	a_inv3 Mat3
	t_inv  [3]f64
	a_fwd  [16]f64
	a      f64
	b      f64
	d      f64
	c      f64
	shift  [3]f64
}

pub struct TrimeshParams {
pub:
	a_inv3 Mat3
	t_inv  [3]f64
	a_fwd  [16]f64
	v0     mlx.Array
	e1     mlx.Array
	e2     mlx.Array
	nrm    mlx.Array
	lo     [3]f64
	hi     [3]f64
}

// GeometryParams is the sum of all camera-space parameter variants.
pub type GeometryParams = AffineParams
	| CsgParams
	| TrimeshParams
	| CircleParams
	| ConeParams
	| CyclideParams
	| EllipsoidParams
	| TorusParams
	| BoxParams
	| CylinderParams
	| PlaneParams
	| SphereParams
	| SplatsParams

// SplatsParams is the inert camera-space placeholder for SplatsGeometry: a
// splat cloud has no surface to intersect, so every ray kernel is a no-op.
// The splat data lives on SplatsGeometry and is rendered by splat_render.v.
pub struct SplatsParams {
}

// --- geometry structs --------------------------------------------------------

pub struct SphereGeometry {
pub:
	radius f64
	blade  Multivector
}

pub fn sphere_geometry(radius f64) SphereGeometry {
	if radius <= 0.0 {
		panic('sphere radius must be > 0, got ${radius}')
	}
	return SphereGeometry{
		radius: radius
		blade:  sphere([0.0, 0.0, 0.0]!, radius)
	}
}

pub struct PlaneGeometry {
pub:
	blade Multivector
}

pub fn plane_geometry(normal [3]f64, distance f64) PlaneGeometry {
	return PlaneGeometry{
		blade: plane(normal, distance)
	}
}

pub struct CylinderGeometry {
pub:
	radius f64
	half   f64 // -1.0 = infinite
	blade  Cylinder
}

pub fn cylinder_geometry(radius f64, length f64) CylinderGeometry {
	if radius <= 0.0 {
		panic('cylinder radius must be > 0, got ${radius}')
	}
	if length > 0.0 {
		return CylinderGeometry{
			radius: radius
			half:   length / 2.0
			blade:  cylinder([0.0, 0.0, 0.0]!, [0.0, 0.0, 1.0]!, radius)
		}
	}
	return CylinderGeometry{
		radius: radius
		half:   -1.0
		blade:  cylinder([0.0, 0.0, 0.0]!, [0.0, 0.0, 1.0]!, radius)
	}
}

pub struct BoxGeometry {
pub:
	half [3]f64
}

pub fn box_geometry(width f64, height f64, depth f64) BoxGeometry {
	if math.min(width, math.min(height, depth)) <= 0.0 {
		panic('box dimensions must be > 0, got (${width}, ${height}, ${depth})')
	}
	return BoxGeometry{
		half: [width / 2.0, height / 2.0, depth / 2.0]!
	}
}

pub struct CircleGeometry {
pub:
	radius f64
	blade  Multivector
}

pub fn circle_geometry(radius f64) CircleGeometry {
	if radius <= 0.0 {
		panic('circle radius must be > 0, got ${radius}')
	}
	return CircleGeometry{
		radius: radius
		blade:  circle([0.0, 0.0, 0.0]!, radius, [0.0, 0.0, 1.0]!)
	}
}

// Geometry is the sum type of all renderable geometries.
pub type Geometry = AffineGeometry
	| CsgGeometry
	| TrimeshGeometry
	| ConeGeometry
	| CyclideGeometry
	| EllipsoidGeometry
	| TorusGeometry
	| SphereGeometry
	| PlaneGeometry
	| CylinderGeometry
	| BoxGeometry
	| CircleGeometry
	| SplatsGeometry

// SplatsGeometry is a Gaussian-splat set (LOCAL space) used as scene geometry.
// It is invisible to the ray-tracing kernels (no surface); render scenes
// containing splat meshes with render_scene_with_splats (splat_render.v).
pub struct SplatsGeometry {
pub:
	gaussians Gaussians
}

// splats_geometry wraps a Gaussians set as scene geometry (local space).
pub fn splats_geometry(g Gaussians) SplatsGeometry {
	return SplatsGeometry{
		gaussians: g
	}
}

pub struct TrimeshGeometry {
pub:
	n_faces int
	v0      [][3]f64
	e1      [][3]f64
	e2      [][3]f64
	nrm     [][3]f64
	lo      [3]f64
	hi      [3]f64
}

// trimesh_geometry builds a triangle mesh (flat shading, no BVH).
pub fn trimesh_geometry(vertices [][3]f64, faces [][3]int) TrimeshGeometry {
	if faces.len < 1 {
		panic('trimesh needs >= 1 face')
	}
	nv := vertices.len
	mut v0 := [][3]f64{}
	mut edge1 := [][3]f64{}
	mut edge2 := [][3]f64{}
	mut nrm := [][3]f64{}
	mut bmin := [1e30, 1e30, 1e30]!
	mut bmax := [-1e30, -1e30, -1e30]!
	for v in vertices {
		for k in 0 .. 3 {
			if v[k] < bmin[k] {
				bmin[k] = v[k]
			}
			if v[k] > bmax[k] {
				bmax[k] = v[k]
			}
		}
	}
	for f in faces {
		if f[0] < 0 || f[1] < 0 || f[2] < 0 || f[0] >= nv || f[1] >= nv || f[2] >= nv {
			panic('bad face ${f} (vertices=${nv})')
		}
		a := vertices[f[0]]
		b := vertices[f[1]]
		c := vertices[f[2]]
		ee1 := [b[0] - a[0], b[1] - a[1], b[2] - a[2]]!
		ee2 := [c[0] - a[0], c[1] - a[1], c[2] - a[2]]!
		cr := vec3_cross(ee1, ee2)
		cl := math.sqrt(cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2])
		if cl < 1e-12 {
			panic('trimesh has degenerate (zero-area) faces')
		}
		v0 << a
		edge1 << ee1
		edge2 << ee2
		nrm << [cr[0] / cl, cr[1] / cl, cr[2] / cl]!
	}
	return TrimeshGeometry{
		n_faces: faces.len
		v0:      v0
		e1:      edge1
		e2:      edge2
		nrm:     nrm
		lo:      bmin
		hi:      bmax
	}
}

pub struct ConeGeometry {
pub:
	radius f64
	height f64
}

pub fn cone_geometry(radius f64, height f64) ConeGeometry {
	if radius <= 0.0 || height <= 0.0 {
		panic('cone radius/height must be > 0, got (${radius}, ${height})')
	}
	return ConeGeometry{
		radius: radius
		height: height
	}
}

pub struct TorusGeometry {
pub:
	major f64
	minor f64
}

pub fn torus_geometry(major f64, minor f64) TorusGeometry {
	if major <= 0.0 || minor <= 0.0 {
		panic('torus radii must be > 0, got (${major}, ${minor})')
	}
	return TorusGeometry{
		major: major
		minor: minor
	}
}

pub struct EllipsoidGeometry {
pub:
	radii [3]f64
}

pub fn ellipsoid_geometry(rx f64, ry f64, rz f64) EllipsoidGeometry {
	if math.min(rx, math.min(ry, rz)) <= 0.0 {
		panic('ellipsoid radii must be > 0, got (${rx}, ${ry}, ${rz})')
	}
	return EllipsoidGeometry{
		radii: [rx, ry, rz]!
	}
}

pub struct CyclideGeometry {
pub:
	a     f64
	b     f64
	d     f64
	shift [3]f64
}

pub fn cyclide_geometry(a f64, b f64, d f64, shift [3]f64) CyclideGeometry {
	if !(a > b && b > 0.0) {
		panic('cyclide needs a > b > 0, got (${a}, ${b})')
	}
	if d <= 0.0 {
		panic('cyclide needs d > 0, got ${d}')
	}
	return CyclideGeometry{
		a:     a
		b:     b
		d:     d
		shift: shift
	}
}

// geom_to_camera conjugates a geometry's blade into camera space and returns
// the camera-space parameters.
pub fn geom_to_camera(g Geometry, m Multivector) GeometryParams {
	return match g {
		SphereGeometry {
			s := m.apply(g.blade)
			c, r := sphere_from_dual(s)
			SphereParams{
				c:    c
				r:    r
				axes: [vec3_unit(dir3(m.apply(e1()))), vec3_unit(dir3(m.apply(e2()))),
					vec3_unit(dir3(m.apply(e3())))]!
			}
		}
		PlaneGeometry {
			pi := m.apply(g.blade)
			PlaneParams{
				n: vec3_unit(dir3(pi))
				d: pi.einf_coeff()
			}
		}
		CylinderGeometry {
			CylinderParams{
				q: m.apply(point(0.0, 0.0, 0.0)).coords()
				u: vec3_unit(dir3(m.apply(e3())))
				r: g.radius
				h: g.half
			}
		}
		BoxGeometry {
			hf := g.half
			BoxParams{
				c:    m.apply(point(0.0, 0.0, 0.0)).coords()
				axes: [vec3_unit(dir3(m.apply(e1()))), vec3_unit(dir3(m.apply(e2()))),
					vec3_unit(dir3(m.apply(e3())))]!
				half: hf
			}
		}
		CircleGeometry {
			CircleParams{
				c: m.apply(point(0.0, 0.0, 0.0)).coords()
				n: vec3_unit(dir3(m.apply(e3())))
				r: g.radius
			}
		}
		ConeGeometry {
			ai, ti, af := affine_from_motor(m, identity3())
			ConeParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
				r:      g.radius
				h:      g.height
			}
		}
		TorusGeometry {
			ai, ti, af := affine_from_motor(m, identity3())
			TorusParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
				major:  g.major
				minor:  g.minor
			}
		}
		EllipsoidGeometry {
			rr := g.radii
			diag := mat3_new([rr[0], 0.0, 0.0]!, [0.0, rr[1], 0.0]!, [0.0, 0.0, rr[2]]!)
			ai, ti, af := affine_from_motor(m, diag)
			EllipsoidParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
			}
		}
		CyclideGeometry {
			ai, ti, af := affine_from_motor(m, identity3())
			sh := g.shift
			CyclideParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
				a:      g.a
				b:      g.b
				d:      g.d
				c:      math.sqrt(g.a * g.a - g.b * g.b)
				shift:  sh
			}
		}
		TrimeshGeometry {
			ai, ti, af := affine_from_motor(m, identity3())
			glo := g.lo
			ghi := g.hi
			TrimeshParams{
				a_inv3: ai
				t_inv:  ti
				a_fwd:  af
				v0:     mlx.array_f32(to_f32_3(g.v0), [g.v0.len, 3])
				e1:     mlx.array_f32(to_f32_3(g.e1), [g.e1.len, 3])
				e2:     mlx.array_f32(to_f32_3(g.e2), [g.e2.len, 3])
				nrm:    mlx.array_f32(to_f32_3(g.nrm), [g.nrm.len, 3])
				lo:     glo
				hi:     ghi
			}
		}
		CsgGeometry {
			mut ch := []GeometryParams{}
			for c in g.children {
				ch << geom_to_camera(c, m)
			}
			CsgParams{
				op:       g.op
				children: ch
			}
		}
		AffineGeometry {
			affine_to_camera(g, m)
		}
		SplatsGeometry {
			// inert: no ray-tracing parameters (rendered by splat_render.v)
			SplatsParams{}
		}
	}
}

fn to_f32_3(v [][3]f64) []f32 {
	mut out := []f32{len: v.len * 3}
	for i, p in v {
		out[i * 3] = f32(p[0])
		out[i * 3 + 1] = f32(p[1])
		out[i * 3 + 2] = f32(p[2])
	}
	return out
}
