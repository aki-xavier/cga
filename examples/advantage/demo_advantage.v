module main

// Advantage demo: three panels (no-polygons / infinite geometry / transform
// isomorphism).  Saves advantage_{a,b,c}.png.
import cga
import os
import math

fn render_panel(sc cga.Scene, cam_pos [3]f64, target [3]f64, name string) {
	mut cam := cga.perspective_camera(50.0, 400.0 / 300.0, 0.1, 100.0, cam_pos, target, [
		0.0,
		1.0,
		0.0,
	]!)
	cam.look_at(target, none)
	mut r := cga.renderer(400, 300, 2, 3)
	img := r.render(sc, cam)
	out_dir := os.dir(@FILE)
	os.mkdir_all(out_dir) or {}
	cga.save_frame_png('${out_dir}/${name}.png', img)
}

fn panel_a() cga.Scene {
	mut sc := cga.scene(cga.color_hex(0x101418))
	sc.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.sphere_geometry(1.2)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xC0392B)
			roughness:  0.18
			metalness:  0.2
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.15, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.cylinder_geometry(0.4, -1.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xD4AC0D)
			roughness:  0.3
			metalness:  0.25
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [1.55, 0.15, 0.1]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.5, [0.3, 0.9, 0.4]!))
	sc.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.9, [0.6, 2.0, 1.5]!))
	sc.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.25))
	return sc
}

fn panel_b() cga.Scene {
	mut sc := cga.scene(none)
	sc.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.plane_geometry([0.0, 1.0, 0.0]!, 0.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xB0B0B0)
			roughness:  0.75
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
	sc.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.cylinder_geometry(0.5, -1.0)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xD4AC0D)
			roughness:  0.35
			metalness:  0.2
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [1.3, 0.0, -0.5]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.sphere_geometry(0.8)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0xC0392B)
			roughness:  0.3
			metalness:  0.0
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [-1.4, 0.8, 0.6]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          none
	}))
	sc.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.42, [0.4, 1.0, 0.35]!))
	sc.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.5, [0.0, 3.0, 2.5]!))
	sc.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.32))
	return sc
}

fn panel_c() cga.Scene {
	mut sc := cga.scene(cga.color_hex(0x101418))
	m0 := cga.translator([-2.2, 0.75, -0.6]!).gp(cga.motor_rotor([0.0, 1.0, 0.0]!, -0.5))
	m1 := cga.translator([2.0, 0.75, 1.0]!).gp(cga.motor_rotor([0.0, 1.0, 0.0]!, 1.1))
	for i in 0 .. 6 {
		m := m0.interpolate(m1, f64(i) / 5.0)
		sc.add_mesh(cga.mesh(cga.MeshParams{
			geometry:       cga.sphere_geometry(0.16)
			material:       cga.standard_material(cga.MaterialParams{
				color:      cga.color_hex(0x95A5A6)
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
			motor:          m
		}))
	}
	sc.add_mesh(cga.mesh(cga.MeshParams{
		geometry:       cga.box_geometry(0.65, 0.65, 0.65)
		material:       cga.standard_material(cga.MaterialParams{
			color:      cga.color_hex(0x27AE60)
			roughness:  0.55
			metalness:  0.0
			emissive:   cga.color_hex(0x000000)
			opacity:    1.0
			ior:        1.5
			absorption: 0.0
		})
		position:       [0.0, 0.0, 0.0]!
		rotation_axis:  [0.0, 0.0, 1.0]!
		rotation_angle: 0.0
		motor:          m1
	}))
	sc.add_light(cga.directional_light(cga.color_hex(0xFFFFFF), 0.45, [0.4, 1.0, 0.35]!))
	sc.add_light(cga.point_light(cga.color_hex(0xFFFFFF), 0.6, [-1.0, 2.5, 3.0]!))
	sc.add_light(cga.ambient_light(cga.color_hex(0xFFFFFF), 0.38))
	return sc
}

fn main() {
	render_panel(panel_a(), [0.0, 0.15, 3.2]!, [0.0, 0.15, 0.0]!, 'advantage_a')
	render_panel(panel_b(), [0.0, 1.6, 5.5]!, [0.0, 0.7, 0.0]!, 'advantage_b')
	render_panel(panel_c(), [0.2, 2.6, 5.8]!, [0.0, 0.75, 0.2]!, 'advantage_c')
	_ = math.pi
}
