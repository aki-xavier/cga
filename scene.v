module cga

// Mesh, Scene, PerspectiveCamera and OrbitControls (three.js-style surface).
import math

// Mesh binds a geometry, a material and a local pose (Object3D).
pub struct Mesh {
	Object3D
pub mut:
	geometry Geometry
	material Material
}

// mesh builds a Mesh (position/rotation_axis/rotation_angle define the pose;
// motor overrides them if given).
pub fn mesh(geometry Geometry, material Material, position [3]f64, rotation_axis [3]f64, rotation_angle f64, motor ?Multivector) Mesh {
	return Mesh{
		Object3D: object3d(position, rotation_axis, rotation_angle, motor, identity3())
		geometry: geometry
		material: material
	}
}

// Scene holds the object list, the light list and the background colour.
pub struct Scene {
pub mut:
	objects    []Mesh
	lights     []Light
	background Color
}

// scene builds a Scene (default sky-blue background).
pub fn scene(background ?Color) Scene {
	return Scene{
		background: if b := background { b } else { color_hex(0x87CEEB) }
	}
}

pub fn (mut s Scene) add_mesh(m Mesh) {
	s.objects << m
}

pub fn (mut s Scene) add_light(l Light) {
	s.lights << l
}

// PerspectiveCamera is a pinhole camera (world->camera motor from look_at).
pub struct PerspectiveCamera {
pub mut:
	fov      f64
	aspect   f64
	near     f64
	far      f64
	position [3]f64
	target   [3]f64
	up       [3]f64
	motor    Multivector
}

// perspective_camera builds a PerspectiveCamera.
pub fn perspective_camera(fov f64, aspect f64, near f64, far f64, position [3]f64, target [3]f64, up [3]f64) PerspectiveCamera {
	if fov <= 0.0 || fov >= 180.0 {
		panic('fov must be in (0, 180), got ${fov}')
	}
	return PerspectiveCamera{
		fov:      fov
		aspect:   aspect
		near:     near
		far:      far
		position: position
		target:   target
		up:       vec3_unit(up)
		motor:    motor_identity()
	}
}

// look_at builds the world->camera motor (camera basis = {right, -up, forward}).
pub fn (mut c PerspectiveCamera) look_at(target [3]f64, up ?[3]f64) {
	c.target = target
	if u := up {
		c.up = vec3_unit(u)
	}
	f :=
		vec3_unit([target[0] - c.position[0], target[1] - c.position[1], target[2] - c.position[2]]!)
	r := vec3_unit(vec3_cross(f, c.up))
	u := vec3_cross(r, f)
	mut mat := identity3()
	mat[0] = r
	mat[1] = [-u[0], -u[1], -u[2]]!
	mat[2] = f
	t := [-vec3_dot(mat[0], c.position), -vec3_dot(mat[1], c.position), -vec3_dot(mat[2], c.position)]!
	c.motor = motor_from_matrix(mat, t)
}

// OrbitControls is the static spherical orbit helper (update() repositions the
// camera).
pub struct OrbitControls {
pub mut:
	target    [3]f64
	azimuth   f64
	elevation f64
	radius    f64
}

pub fn orbit_controls(target [3]f64, azimuth f64, elevation f64, radius f64) OrbitControls {
	return OrbitControls{
		target:    target
		azimuth:   azimuth
		elevation: elevation
		radius:    radius
	}
}

// update repositions the camera (spherical orbit around `target`).
pub fn (o OrbitControls) update(mut camera PerspectiveCamera) {
	ce := math.cos(o.elevation)
	x := o.radius * ce * math.sin(o.azimuth)
	y := o.radius * math.sin(o.elevation)
	z := o.radius * ce * math.cos(o.azimuth)
	camera.position = [o.target[0] + x, o.target[1] + y, o.target[2] + z]!
	camera.look_at(o.target, none)
}
