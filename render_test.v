module cga

import mlx
import math

fn depth_at(depth mlx.Array, row int, col int, w int) f32 {
	return depth.data_f32()[row * w + col]
}

fn test_render_masked_sphere_plane() {
	fx := 100.0
	fy := 100.0
	cx := 64.0
	cy := 48.0
	h := 96
	w := 128
	pl := plane([0.0, 0.0, 1.0]!, 2.0)
	sp := sphere([0.0, 0.0, 3.0]!, 0.5)
	prims := [render_primitive('plane', pl, 1, 1.0), render_primitive('sphere', sp, 2, 1.0)]
	yy := mlx.arange(0, h, 1, .float32).expand_dims(1).broadcast_to([h, w])
	xx := mlx.arange(0, w, 1, .float32).expand_dims(0).broadcast_to([h, w])
	disc := s_le(s_sub(xx, 64.0).square().add(s_sub(yy, 48.0).square()), 40.0 * 40.0)
	regions := mlx.where(disc, mlx.int_scalar(2), mlx.int_scalar(1))
	out := render_scene(prims, fx, fy, cx, cy, h, w, regions, none, 0.1, 1e4)
	assert math.abs(depth_at(out.depth, 48, 64, w) - 2.5) < 1e-3
	assert math.abs(depth_at(out.depth, 10, 10, w) - 2.0) < 1e-3
}

fn test_render_full_zbuffer() {
	h := 96
	w := 128
	pf := plane([0.0, 0.0, 1.0]!, 4.0)
	sn := sphere([0.0, 0.0, 2.0]!, 0.5)
	prims := [render_primitive('sphere', sn, 2, 1.0), render_primitive('plane', pf, 1, 1.0)]
	out := render_scene(prims, 100.0, 100.0, 64.0, 48.0, h, w, none, none, 0.1, 1e4)
	assert math.abs(depth_at(out.depth, 48, 64, w) - 1.5) < 1e-3
	assert math.abs(depth_at(out.depth, 5, 5, w) - 4.0) < 1e-3
}

fn test_render_motor() {
	h := 96
	w := 128
	pf2 := plane([0.0, 0.0, 1.0]!, 4.0)
	m := translator([0.0, 0.0, -1.0]!)
	out := render_scene([render_primitive('plane', pf2, 1, 1.0)], 100.0, 100.0, 64.0, 48.0, h, w,
		none, m, 0.1, 1e4)
	assert math.abs(depth_at(out.depth, 48, 64, w) - 3.0) < 1e-3
}

fn test_render_cylinder() {
	h := 96
	w := 128
	cy := cylinder([0.0, 0.0, 3.0]!, [0.0, 1.0, 0.0]!, 0.4)
	wall := plane([0.0, 0.0, 1.0]!, 5.0)
	mut cylp := render_primitive('cylinder', cy.blade, 1, 1.0)
	cylp.cylinder_data = cy
	prims := [cylp, render_primitive('plane', wall, 2, 1.0)]
	out := render_scene(prims, 100.0, 100.0, 64.0, 48.0, h, w, none, none, 0.1, 1e4)
	assert math.abs(depth_at(out.depth, 48, 64, w) - 2.6) < 1e-3
	assert math.abs(depth_at(out.depth, 5, 5, w) - 5.0) < 1e-3
}
