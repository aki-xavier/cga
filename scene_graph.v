module cga

// Scene-graph layer: Vec3 helpers, Color, Object3D, Mesh, Scene,
// PerspectiveCamera and OrbitControls (the three.js-style surface).

import math

// --- Vec3 -------------------------------------------------------------------

pub fn vec3_unit(a [3]f64) [3]f64 {
	n := math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
	if n < 1e-12 {
		return [0.0, 0.0, 1.0]!
	}
	return [a[0] / n, a[1] / n, a[2] / n]!
}

pub fn vec3_cross(a [3]f64, b [3]f64) [3]f64 {
	return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]!
}

pub fn vec3_dot(a [3]f64, b [3]f64) f64 {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

// dir3 returns the euclidean (e1,e2,e3) part of a grade-1 vector.
pub fn dir3(a Multivector) [3]f64 {
	return a.euclidean_vector()
}

// --- Color ------------------------------------------------------------------

// Color is an sRGB colour (0-1 encoded components).  rgb() returns the linear
// components used for shading; the Renderer re-encodes on output.
pub struct Color {
pub:
	r f64
	g f64
	b f64
}

// color_hex builds a Color from a 0xRRGGBB integer.
pub fn color_hex(c int) Color {
	return Color{
		r: f64((c >> 16) & 0xFF) / 255.0
		g: f64((c >> 8) & 0xFF) / 255.0
		b: f64(c & 0xFF) / 255.0
	}
}

// color_rgb builds a Color from three sRGB-encoded 0-1 components.
pub fn color_rgb(r f64, g f64, b f64) Color {
	return Color{
		r: r
		g: g
		b: b
	}
}

// srgb_to_linear decodes one sRGB component to linear (IEC 61966-2-1).
pub fn srgb_to_linear(c f64) f64 {
	if c <= 0.04045 {
		return c / 12.92
	}
	return math.pow((c + 0.055) / 1.055, 2.4)
}

// rgb returns the linear-space components.
pub fn (c Color) rgb() [3]f64 {
	return [srgb_to_linear(c.r), srgb_to_linear(c.g), srgb_to_linear(c.b)]!
}

// --- Object3D ---------------------------------------------------------------

// identity3 is the 3x3 identity matrix.
pub fn identity3() Mat3 {
	mut m := Mat3{}
	m[0][0] = 1.0
	m[1][1] = 1.0
	m[2][2] = 1.0
	return m
}

// is_identity3 reports whether a 3x3 matrix is the identity (within tolerance).
pub fn is_identity3(m Mat3) bool {
	for i in 0 .. 3 {
		for j in 0 .. 3 {
			want := if i == j { 1.0 } else { 0.0 }
			if math.abs(m[i][j] - want) > 1e-12 {
				return false
			}
		}
	}
	return true
}

pub struct Object3D {
pub mut:
	position       [3]f64
	rotation_axis  [3]f64
	rotation_angle f64
	motor_override ?Multivector
	linear         Mat3
}

// object3d builds an Object3D node (local pose = Motor, optional linear block).
pub fn object3d(position [3]f64, rotation_axis [3]f64, rotation_angle f64, motor ?Multivector, linear Mat3) Object3D {
	mut o := Object3D{}
	if m := motor {
		o.motor_override = m
		mtx := m.to_matrix()
		o.position = [mtx[3], mtx[7], mtx[11]]!
		o.rotation_axis = [0.0, 0.0, 1.0]!
		o.rotation_angle = 0.0
	} else {
		o.position = position
		o.rotation_axis = rotation_axis
		o.rotation_angle = rotation_angle
	}
	o.linear = linear
	return o
}

// motor returns the local pose motor (full motor if given, else T . R).
pub fn (o Object3D) motor() Multivector {
	if m := o.motor_override {
		return m
	}
	return translator(o.position).gp(motor_rotor(o.rotation_axis, o.rotation_angle))
}
