module main

// Kinematics demo: gears + crank-slider + spiral trajectory driven by Motor.
// Saves PNG frames and an animated GIF (assembled in V).
import cga
import os
import math

struct KinScene {
mut:
	scene  cga.Scene
	big    []int
	small  []int
	rod    int
	slider int
	ball   int
	m0     cga.Multivector
	twist  cga.Multivector
}

fn gear_meshes(mut ks KinScene, r_hub f64, r_tooth f64, n_teeth int, thick f64, color cga.Color) []int {
	mat := cga.standard_material(cga.MaterialParams{
		color:      color
		roughness:  0.35
		metalness:  0.7
		emissive:   cga.color_hex(0x000000)
		opacity:    1.0
		ior:        1.5
		absorption: 0.0
	})
	mut idx := []int{}
	ks.scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.cylinder_geometry(r_hub, thick)
		material:       mat
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [1.0, 0.0, 0.0]!
		rotation_angle: -math.pi / 2.0
		motor:          none
	}))
	idx << ks.scene.objects.len - 1
	r_mid := r_hub + (r_tooth - r_hub) / 2.0
	for i in 0 .. n_teeth {
		a := f64(i) * 2.0 * math.pi / f64(n_teeth)
		m := cga.motor_rotor([0.0, 1.0, 0.0]!, a).gp(cga.translator([r_mid, 0.0, 0.0]!))
		ks.scene.add_mesh(cga.mesh(cga.MeshParams{
			geometry:       cga.box_geometry(r_tooth - r_hub + 0.06, thick, 0.16)
			material:       mat
			position:       [0.0, 0.0, 0.0]!
			rotation_axis:  [0.0, 0.0, 1.0]!
			rotation_angle: 0.0
			motor:          m
		}))
		idx << ks.scene.objects.len - 1
	}
	return idx
}

fn frame_motor(axis [3]f64, angle f64, point [3]f64) cga.Multivector {
	return cga.translator(point).gp(cga.motor_rotor(axis, angle)).gp(cga.translator([
		-point[0],
		-point[1],
		-point[2],
	]!))
}

fn rod_motor(p [3]f64, q [3]f64) (cga.Multivector, f64) {
	d := [q[0] - p[0], q[1] - p[1], q[2] - p[2]]!
	length := math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
	u := [d[0] / length, d[1] / length, d[2] / length]!
	ax := [-u[1], u[0]]
	an := math.sqrt(ax[0] * ax[0] + ax[1] * ax[1])
	rot := if an < 1e-12 {
		cga.motor_rotor([1.0, 0.0, 0.0]!, if u[2] > 0.0 { 0.0 } else { math.pi })
	} else {
		angle := math.acos(math.max(-1.0, math.min(1.0, u[2])))
		cga.motor_rotor([ax[0] / an, ax[1] / an, 0.0]!, angle)
	}
	mid := [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0, (p[2] + q[2]) / 2.0]!
	return cga.translator(mid).gp(rot), length
}

fn build_scene() KinScene {
	mut ks := KinScene{}
	ks.scene = cga.scene(none)
	ks.scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.plane_geometry([0.0, 1.0, 0.0]!, -0.05)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x3A4046)
			roughness:  0.9
			metalness:  0.0
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	ks.scene.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.5, [0.4, 1.0, 0.5]!))
	ks.scene.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.5, [-4.0, 6.0, 4.0]!))
	ks.scene.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.4))
	// gears: big (16 teeth) at origin, small (8 teeth) at x=2.4
	ks.big = gear_meshes(mut ks, 1.28, 1.6, 16, 0.4, cga.color_hex(0xC8A24A))
	ks.small = gear_meshes(mut ks, 0.64, 0.8, 8, 0.4, cga.color_hex(0x9BA1A6))
	for idx in ks.small {
		ks.scene.objects[idx].motor_override =
			cga.translator([2.4, 0.0, 0.0]!).gp(ks.scene.objects[idx].motor())
	}
	// crank-slider (rear z=2.6 plane)
	crank_c := [-1.6, 0.6, 2.6]!
	ks.scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.cylinder_geometry(0.55, 0.25)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x4A4F54)
			roughness:  0.5
			metalness:  0.6
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       crank_c
		rotation_axis:  [1.0, 0.0, 0.0]!
		rotation_angle: -math.pi / 2.0
		motor:          none
	}))
	ks.scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.cylinder_geometry(0.09, 1.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xC8A24A)
			roughness:  0.3
			metalness:  0.8
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	ks.rod = ks.scene.objects.len - 1
	ks.scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.box_geometry(0.5, 0.4, 0.35)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x9BA1A6)
			roughness:  0.35
			metalness:  0.75
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	ks.slider = ks.scene.objects.len - 1
	ks.scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.box_geometry(2.6, 0.08, 0.5)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x4A4F54)
			roughness:  0.5
			metalness:  0.6
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.4, 2.6]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	// spiral ball
	ks.scene.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.sphere_geometry(0.18)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.0
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	ks.ball = ks.scene.objects.len - 1
	m0 := cga.translator([-3.2, 0.5, 2.2]!)
	m1 := cga.translator([3.6, 1.8, 3.4]!).gp(cga.motor_rotor([0.0, 1.0, 0.0]!, 1.5 * math.pi))
	ks.m0 = m0
	ks.twist = m0.reverse().gp(m1).log()
	for k in 0 .. 9 {
		ks.scene.add_mesh(cga.mesh(cga.MeshParams{
			geometry:       cga.sphere_geometry(0.05)
			material:       cga.standard_material(cga.MaterialParams{
				color:      cga.color_hex(0x7F8C8D)
				roughness:  0.6
				metalness:  0.0
				emissive:   cga.color_hex(0x000000)
				opacity:    1.0
				ior:        1.5
				absorption: 0.0
			})
			position:       [0.0, 0.0, 0.0]!
			rotation_axis:  [0.0, 0.0, 1.0]!
			rotation_angle: 0.0
			motor:          m0.gp(cga.motor_exp(ks.twist, f64(k) / 8.0))
		}))
	}
	return ks
}

fn main() {
	mut ks := build_scene()
	n_frames := 24
	mut big_local := []cga.Multivector{}
	for idx in ks.big {
		big_local << ks.scene.objects[idx].motor()
	}
	mut small_local := []cga.Multivector{}
	for idx in ks.small {
		small_local << ks.scene.objects[idx].motor()
	}
	mut cam := cga.perspective_camera(48.0, 4.0 / 3.0, 0.1, 100.0, [0.4, 5.6, 8.8]!, [
		0.4,
		0.2,
		1.2,
	]!, [0.0, 1.0, 0.0]!)
	cam.look_at([0.4, 0.2, 1.2]!, none)
	mut renderer := cga.renderer(360, 270, 2, 3)
	mut gif_frames := [][]u8{}
	for f in 0 .. n_frames {
		s := f64(f) / f64(n_frames)
		g1 := frame_motor([0.0, 1.0, 0.0]!, 2.0 * math.pi * s, [0.0, 0.0, 0.0]!)
		for i, idx in ks.big {
			ks.scene.objects[idx].motor_override = g1.gp(big_local[i])
		}
		g2 := frame_motor([0.0, 1.0, 0.0]!, -4.0 * math.pi * s + math.pi / 8.0, [2.4, 0.0, 0.0]!)
		for i, idx in ks.small {
			ks.scene.objects[idx].motor_override = g2.gp(small_local[i])
		}
		th := 2.0 * math.pi * s
		p := [-1.6 + 0.35 * math.cos(th), 0.6, 2.6 + 0.35 * math.sin(th)]!
		xs := p[0] + math.sqrt(1.35 * 1.35 - (2.6 - p[2]) * (2.6 - p[2]))
		q := [xs, 0.6, 2.6]!
		rm, rl := rod_motor(p, q)
		ks.scene.objects[ks.rod].motor_override = rm
		ks.scene.objects[ks.rod].geometry = cga.cylinder_geometry(0.09, rl)
		ks.scene.objects[ks.slider].position = q
		ks.scene.objects[ks.slider].motor_override = none
		ks.scene.objects[ks.ball].motor_override = ks.m0.gp(cga.motor_exp(ks.twist, s))
		img := renderer.render(ks.scene, cam)
		gif_frames << cga.f32_rgba_to_u8(img.data_f32())
		img.free()
	}
	out := os.dir(@FILE)
	cga.save_gif('${out}/kinematics.gif', gif_frames, 360, 270, 5)
}
