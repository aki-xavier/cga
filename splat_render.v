module cga

// True Gaussian splatting renderer (Route B): EWA projection + depth-sorted
// front-to-back alpha compositing on MLX, replacing the ray-traced-ellipsoid
// approximation (Route A, splat.v) for dense splat counts.
//
// Conventions:
//   - Route A frame: world covariance Sigma = R * diag(scale)^2 * R^T with R
//     recovered from Gaussian.quat (columns t1, t2, n; thin axis local e3).
//   - Pinhole: pixel = (fx * X/Z + cx, fy * Y/Z + cy) with fx, fy, cx, cy
//     exactly as in Renderer.build_rays; camera space +z is forward.
//   - Compositing is in LINEAR space (0..1): splat colours are decoded from
//     the sRGB Color at projection time, the ray-traced base frame comes from
//     Renderer.render_linear, and the sRGB encode (srgb_encode_frame, shared
//     with Renderer.render) happens once at the very end.
//   - Depth compositing treats the scene as ONE opaque surface per pixel (the
//     primary hit): splats behind it are fully occluded.  Refraction and
//     multi-layer transparency of the base scene are ignored.  Each splat's
//     whole footprint is occlusion-tested with its CENTRE depth.  To keep
//     splats attached exactly ON the ray-traced surface from flickering
//     (t == dep coin flip), the centre depth is biased nearer by half the
//     normal extent: t -= 0.5 * sigma_normal (a splat half-buried in the
//     surface shows its front half — the intended attachment semantics).
import mlx
import math

// splat_chunk bounds the (chunk, H, W) temporaries; combined with the per-chunk
// eval + gc_collect below, measured peak at 2048 splats / 320x240 is ~0.7 GB.
const splat_chunk = 32

// Splat2D is one Gaussian after camera transform + EWA projection.
struct Splat2D {
	px    f32 // pixel-space mean (column, row)
	py    f32
	t     f32 // camera distance (depth sort + occlusion test)
	ai    f32 // inverse 2D covariance entries
	bi    f32
	ci    f32
	alpha f32
	col   [3]f32 // linear-space rgb (0..1)
	x0    int    // 3-sigma bounding box, clamped to the image
	x1    int
	y0    int
	y1    int
}

// project_splats transforms every Gaussian to camera space, applies the EWA
// Jacobian approximation for the 2D covariance (with a 0.3px variance
// regulariser), culls invisible splats and returns survivors sorted
// front-to-back.
fn project_splats(g Gaussians, cam PerspectiveCamera, width int, height int) []Splat2D {
	fy := f64(height) / (2.0 * math.tan(math.radians(cam.fov) / 2.0))
	fx := fy * cam.aspect
	cx := f64(width - 1) / 2.0
	cy := f64(height - 1) / 2.0
	m4 := cam.motor.to_matrix() // world -> camera, row-major
	rc := mat3_new([m4[0], m4[1], m4[2]]!, [m4[4], m4[5], m4[6]]!, [m4[8], m4[9], m4[10]]!)
	z_near := math.max(cam.near, 1e-3)
	mut out := []Splat2D{cap: g.splats.len}
	for s in g.splats {
		// camera-space mean
		xc := m4[0] * s.mean[0] + m4[1] * s.mean[1] + m4[2] * s.mean[2] + m4[3]
		yc := m4[4] * s.mean[0] + m4[5] * s.mean[1] + m4[6] * s.mean[2] + m4[7]
		zc := m4[8] * s.mean[0] + m4[9] * s.mean[1] + m4[10] * s.mean[2] + m4[11]
		if zc < z_near {
			continue // behind / too close to the camera plane
		}
		// world rotation from the stored quaternion (Route A convention),
		// then M = R_w2c * R maps the local frame into camera space
		rm := rotor_from_quaternion(s.quat).to_matrix()
		rw := mat3_new([rm[0], rm[1], rm[2]]!, [rm[4], rm[5], rm[6]]!, [rm[8], rm[9], rm[10]]!)
		m := mat3_mul(rc, rw)
		// Sigma_cam = M * diag(scale^2) * M^T (symmetric)
		s0 := s.scale[0] * s.scale[0]
		s1 := s.scale[1] * s.scale[1]
		s2 := s.scale[2] * s.scale[2]
		s11 := m[0][0] * m[0][0] * s0 + m[0][1] * m[0][1] * s1 + m[0][2] * m[0][2] * s2
		s12 := m[0][0] * m[1][0] * s0 + m[0][1] * m[1][1] * s1 + m[0][2] * m[1][2] * s2
		s13 := m[0][0] * m[2][0] * s0 + m[0][1] * m[2][1] * s1 + m[0][2] * m[2][2] * s2
		s22 := m[1][0] * m[1][0] * s0 + m[1][1] * m[1][1] * s1 + m[1][2] * m[1][2] * s2
		s23 := m[1][0] * m[2][0] * s0 + m[1][1] * m[2][1] * s1 + m[1][2] * m[2][2] * s2
		s33 := m[2][0] * m[2][0] * s0 + m[2][1] * m[2][1] * s1 + m[2][2] * m[2][2] * s2
		// EWA: Sigma2 = J * Sigma_cam * J^T, J = d(pixel)/d(X,Y,Z)
		rz := 1.0 / zc
		x := xc * rz
		y := yc * rz
		a := fx * fx * rz * rz * (s11 - 2.0 * x * s13 + x * x * s33) + 0.3
		b := fx * fy * rz * rz * (s12 - y * s13 - x * s23 + x * y * s33)
		c := fy * fy * rz * rz * (s22 - 2.0 * y * s23 + y * y * s33) + 0.3
		det := a * c - b * b
		if det < 1e-9 {
			continue // degenerate 2D covariance
		}
		// 3-sigma screen bounding box (axis-aligned), capped for speed
		hx := math.min(3.0 * math.sqrt(a), f64(width))
		hy := math.min(3.0 * math.sqrt(c), f64(height))
		px := fx * x + cx
		py := fy * y + cy
		x0 := math.max(int(math.floor(px - hx)), 0)
		x1 := math.min(int(math.ceil(px + hx)), width - 1)
		y0 := math.max(int(math.floor(py - hy)), 0)
		y1 := math.min(int(math.ceil(py + hy)), height - 1)
		if x0 > x1 || y0 > y1 {
			continue // fully off-screen
		}
		lin := s.color.rgb() // sRGB Color -> linear 0..1
		out << Splat2D{
			px: f32(px)
			py: f32(py)
			// centre depth, biased nearer by half the normal extent so splats
			// attached exactly on the opaque surface stay visible (see header)
			t:     f32(math.sqrt(xc * xc + yc * yc + zc * zc) - 0.5 * s.scale[2])
			ai:    f32(c / det)
			bi:    f32(-b / det)
			ci:    f32(a / det)
			alpha: f32(s.opacity)
			col:   [f32(lin[0]), f32(lin[1]), f32(lin[2])]!
			x0:    x0
			x1:    x1
			y0:    y0
			y1:    y1
		}
	}
	out.sort(a.t < b.t) // front-to-back
	return out
}

// composite_splats blends depth-sorted splats over a LINEAR base frame
// (H, W, 3, 0..1) with a per-pixel opaque scene depth (H, W, inf = miss),
// returning the LINEAR (H, W, 3) 0..1 result (callers sRGB-encode once at the
// end via srgb_encode_frame).  The gather-based front-to-back loop uses
// sorted-order accumulation (no scatter in the mlx wrapper).
fn composite_splats(splats []Splat2D, base mlx.Array, depth mlx.Array, width int, height int) mlx.Array {
	mut acc := mlx.zeros([height, width, 3], .float32)
	mut trans := mlx.ones([height, width], .float32)
	if splats.len > 0 {
		xx := mlx.arange(0, width, 1, .float32).expand_dims(0).expand_dims(0).broadcast_to([
			1,
			height,
			width,
		])
		yy := mlx.arange(0, height, 1, .float32).expand_dims(1).expand_dims(0).broadcast_to([
			1,
			height,
			width,
		])
		dep := depth.expand_dims(0)
		for start := 0; start < splats.len; start += splat_chunk {
			k := math.min(splat_chunk, splats.len - start)
			chunk := splats[start..start + k]
			mut vpx := []f32{cap: k}
			mut vpy := []f32{cap: k}
			mut vt := []f32{cap: k}
			mut vai := []f32{cap: k}
			mut vbi := []f32{cap: k}
			mut vci := []f32{cap: k}
			mut val := []f32{cap: k}
			mut vlx := []f32{cap: k}
			mut vhx := []f32{cap: k}
			mut vly := []f32{cap: k}
			mut vhy := []f32{cap: k}
			mut vcol := []f32{cap: 3 * k}
			for sp in chunk {
				vpx << sp.px
				vpy << sp.py
				vt << sp.t
				vai << sp.ai
				vbi << sp.bi
				vci << sp.ci
				val << sp.alpha
				vlx << f32(sp.x0) - sp.px
				vhx << f32(sp.x1) - sp.px
				vly << f32(sp.y0) - sp.py
				vhy << f32(sp.y1) - sp.py
				vcol << sp.col[0]
				vcol << sp.col[1]
				vcol << sp.col[2]
			}
			// per-splat params as (k, 1, 1) columns
			px := mlx.array_f32(vpx, [k, 1, 1])
			py := mlx.array_f32(vpy, [k, 1, 1])
			dx := xx.subtract(px) // (k, H, W)
			dy := yy.subtract(py)
			inside := dx.greater_equal(mlx.array_f32(vlx, [k, 1, 1])).logical_and(dx.less_equal(mlx.array_f32(vhx, [
				k,
				1,
				1,
			]))).logical_and(dy.greater_equal(mlx.array_f32(vly, [k, 1, 1]))).logical_and(dy.less_equal(mlx.array_f32(vhy, [
				k,
				1,
				1,
			])))
			// 2D Gaussian weight w = exp(-1/2 d^T Sigma2^-1 d)
			p2 := mlx.array_f32(vai, [k, 1, 1]).multiply(dx.multiply(dx)).add(mlx.s_mul(mlx.array_f32(vbi, [
				k,
				1,
				1,
			]).multiply(dx.multiply(dy)), 2.0)).add(mlx.array_f32(vci, [k, 1, 1]).multiply(dy.multiply(dy)))
			wgt := mlx.s_mul(p2, -0.5).exp()
			mut alpha := mlx.array_f32(val, [k, 1, 1]).multiply(wgt)
			alpha = mlx.where(inside, alpha, mlx.zeros_like(alpha))
			// clamp alpha below 1 (3DGS practice): keeps the transmittance
			// T > 0 after any number of layers, so deeper splats always keep
			// their (small) contribution and compositing stays smooth
			alpha = mlx.s_clip(alpha, 0.0, 0.99)
			// occlusion by the opaque scene surface
			vis := mlx.array_f32(vt, [k, 1, 1]).less(dep)
			alpha = mlx.where(vis, alpha, mlx.zeros_like(alpha))
			cols := mlx.array_f32(vcol, [k, 3])
			// front-to-back alpha compositing with transmittance (gather-based;
			// the mlx wrapper has no scatter)
			for i in 0 .. k {
				a_i := alpha.take_axis(mlx.int_scalar(i), 0)
				c_i := cols.take_axis(mlx.int_scalar(i), 0)
				acc = acc.add(trans.multiply(a_i).expand_dims(-1).multiply(c_i))
				trans = trans.multiply(mlx.fs(1.0).subtract(a_i))
			}
			// materialise per chunk, then release the big temporaries
			acc.eval()
			trans.eval()
			dx.free()
			dy.free()
			inside.free()
			p2.free()
			wgt.free()
			alpha.free()
			vis.free()
			// the wrapper's Boehm finalizers release MLX handles lazily; force
			// a cycle per chunk so peak memory stays bounded by the chunk size
			mlx.gc_collect()
		}
		xx.free()
		yy.free()
	}
	mut out := acc.add(trans.expand_dims(-1).multiply(base))
	out = mlx.s_clip(out, 0.0, 1.0)
	return out
}

// project_point maps a world point to pixel coordinates (column, row) with
// the same pinhole convention as Renderer.build_rays / project_splats.
fn project_point(cam PerspectiveCamera, p [3]f64, width int, height int) (f64, f64) {
	fy := f64(height) / (2.0 * math.tan(math.radians(cam.fov) / 2.0))
	fx := fy * cam.aspect
	m4 := cam.motor.to_matrix()
	xc := m4[0] * p[0] + m4[1] * p[1] + m4[2] * p[2] + m4[3]
	yc := m4[4] * p[0] + m4[5] * p[1] + m4[6] * p[2] + m4[7]
	zc := m4[8] * p[0] + m4[9] * p[1] + m4[10] * p[2] + m4[11]
	return fx * xc / zc + f64(width - 1) / 2.0, fy * yc / zc + f64(height - 1) / 2.0
}

// bright_bbox returns (min_col, max_col, min_row, max_row) of frame pixels
// whose brightness exceeds thresh (used by the splat tests, which compile
// per-file and therefore cannot share test-local helpers).
fn bright_bbox(data []f32, w int, h int, thresh f32) (int, int, int, int) {
	mut c0 := w
	mut c1 := -1
	mut r0 := h
	mut r1 := -1
	for row in 0 .. h {
		for col in 0 .. w {
			idx := (row * w + col) * 4
			if data[idx] + data[idx + 1] + data[idx + 2] > thresh {
				if col < c0 {
					c0 = col
				}
				if col > c1 {
					c1 = col
				}
				if row < r0 {
					r0 = row
				}
				if row > r1 {
					r1 = row
				}
			}
		}
	}
	return c0, c1, r0, r1
}

// render_splats renders the splat set alone over a flat background, returning
// an (H, W, 4) float32 0..255 frame (same convention as Renderer.render).
pub fn render_splats(g Gaussians, cam PerspectiveCamera, width int, height int, background Color) mlx.Array {
	splats := project_splats(g, cam, width, height)
	bg := mlx.arr3v(background.rgb()).expand_dims(0).expand_dims(0).broadcast_to([
		height,
		width,
		3,
	])
	depth := mlx.full_value([height, width], f32(math.inf(1)), .float32)
	lin := composite_splats(splats, bg, depth, width, height)
	return srgb_encode_frame(lin.reshape([height * width, 3]), width, height)
}

// render_splats_over ray-traces the scene with r (LINEAR base colour via
// r.render_linear + primary-hit depth via r.depth_map()) and composites the
// splats in linear space with correct occlusion by the opaque scene surface.
// Returns (H, W, 4) float32 0..255 (sRGB-encoded).
pub fn render_splats_over(g Gaussians, scene Scene, mut r Renderer, cam PerspectiveCamera) mlx.Array {
	base := r.render_linear(scene, cam) // (H*W, 3) linear 0..1
	depth := r.depth_map()
	splats := project_splats(g, cam, r.width, r.height)
	rgb := base.reshape([r.height, r.width, 3])
	lin := composite_splats(splats, rgb, depth, r.width, r.height)
	return srgb_encode_frame(lin.reshape([r.height * r.width, 3]), r.width, r.height)
}

// transform_gaussians applies a RIGID motor (rotation + translation) to a
// splat set: means are transformed as points, each splat's world rotation is
// pre-multiplied by the motor's rotation block (scale is unchanged, so the
// covariance Sigma = R diag(s)^2 R^T transforms correctly).  Non-rigid Object3D
// .linear blocks are NOT supported (the public mesh() constructor always uses
// the identity linear block anyway).
pub fn transform_gaussians(g Gaussians, m Multivector) Gaussians {
	m4 := m.to_matrix() // row-major rigid 4x4
	ro := mat3_new([m4[0], m4[1], m4[2]]!, [m4[4], m4[5], m4[6]]!, [m4[8], m4[9], m4[10]]!)
	mut out := Gaussians{
		material: g.material
		splats:   []Gaussian{cap: g.splats.len}
	}
	for s in g.splats {
		mean := [
			m4[0] * s.mean[0] + m4[1] * s.mean[1] + m4[2] * s.mean[2] + m4[3],
			m4[4] * s.mean[0] + m4[5] * s.mean[1] + m4[6] * s.mean[2] + m4[7],
			m4[8] * s.mean[0] + m4[9] * s.mean[1] + m4[10] * s.mean[2] + m4[11],
		]!
		rm := rotor_from_quaternion(s.quat).to_matrix()
		rw := mat3_new([rm[0], rm[1], rm[2]]!, [rm[4], rm[5], rm[6]]!, [rm[8], rm[9], rm[10]]!)
		out.splats << Gaussian{
			mean:    mean
			quat:    matrix_to_quaternion(mat3_mul(ro, rw))
			scale:   s.scale
			opacity: s.opacity
			color:   s.color
		}
	}
	return out
}

// render_scene_with_splats renders a mixed scene: opaque meshes are
// ray-traced with r, every Mesh whose geometry is SplatsGeometry is splatted
// (its local Gaussians transformed by the mesh's rigid pose motor), and all
// splat sets are composited TOGETHER (globally depth-sorted) with correct
// occlusion against the opaque surface.  Returns (H, W, 4) float32 0..255.
//
// Note: the plain Renderer.render path leaves splat meshes invisible (the
// SplatsParams ray kernels are no-ops) — use this entry point for them.
pub fn render_scene_with_splats(scene Scene, mut r Renderer, cam PerspectiveCamera) mlx.Array {
	mut opaque := Scene{
		objects:    []Mesh{cap: scene.objects.len}
		lights:     scene.lights
		background: scene.background
	}
	mut world := Gaussians{
		splats: []Gaussian{}
	}
	for obj in scene.objects {
		if obj.geometry is SplatsGeometry {
			sg := obj.geometry as SplatsGeometry
			world.splats << transform_gaussians(sg.gaussians, obj.motor()).splats
		} else {
			opaque.objects << obj
		}
	}
	base := r.render_linear(opaque, cam) // (H*W, 3) linear 0..1
	depth := r.depth_map()
	splats := project_splats(world, cam, r.width, r.height)
	rgb := base.reshape([r.height, r.width, 3])
	lin := composite_splats(splats, rgb, depth, r.width, r.height)
	return srgb_encode_frame(lin.reshape([r.height * r.width, 3]), r.width, r.height)
}
