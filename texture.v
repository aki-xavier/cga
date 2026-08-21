module cga

// Immutable linear RGBA texture sampled on the MLX device.  Source images are
// decoded from PNG (sRGB -> linear) before entering the renderer.
import mlx
import stbi

pub enum WrapMode {
	repeat
	clamp
}

pub struct Texture {
pub:
	pixels    mlx.Array // (height, width, 4) float32 raw texels in [0,1]
	height    int
	width     int
	is_linear bool // true for data maps (metallicRoughness/normal), false for sRGB colour maps
}

// sRGB -> linear on an (N,3) encoded float array.
fn srgb_to_linear_arr(rgb mlx.Array) mlx.Array {
	return mlx.where(mlx.s_le(rgb, 0.04045), mlx.s_div(rgb, 12.92), mlx.s_pow(mlx.s_div(mlx.s_add(rgb, 0.055), 1.055),
		2.4))
}

// texture_from_rgba builds a texture from raw sRGB RGBA floats (0..1).
pub fn texture_from_rgba(rgba [][][]f64) Texture {
	h := rgba.len
	if h < 1 {
		panic('texture must have >= 1 row')
	}
	w := rgba[0].len
	mut flat := []f32{len: h * w * 4}
	mut k := 0
	for y in 0 .. h {
		if rgba[y].len != w || rgba[y][0].len != 4 {
			panic('texture rgba must be (height, width, 4)')
		}
		for x in 0 .. w {
			for c in 0 .. 4 {
				flat[k] = f32(rgba[y][x][c])
				k++
			}
		}
	}
	return Texture{
		pixels:    mlx.array_f32(flat, [h, w, 4])
		height:    h
		width:     w
		is_linear: false
	}
}

// texture_from_u8_rgba builds a Texture from raw sRGB RGBA bytes (row-major).
pub fn texture_from_u8_rgba(rgba []u8, w int, h int) Texture {
	return texture_from_raw(rgba, w, h, false)
}

fn texture_from_raw(rgba []u8, w int, h int, is_linear bool) Texture {
	mut flat := []f32{len: rgba.len}
	for i, b in rgba {
		flat[i] = f32(b) / 255.0
	}
	return Texture{
		pixels:    mlx.array_f32(flat, [h, w, 4])
		height:    h
		width:     w
		is_linear: is_linear
	}
}

// texture_load loads a PNG file and decodes it to linear RGBA.
pub fn texture_load(path string) !Texture {
	rgba, w, h := load_png_rgba(path)!
	return texture_from_u8_rgba(rgba, w, h)
}

// texture_from_png_bytes decodes a PNG (raw bytes) to a linear RGBA texture.
pub fn texture_from_png_bytes(data []u8) !Texture {
	rgba, w, h := decode_png_rgba(data)!
	return texture_from_u8_rgba(rgba, w, h)
}

// texture_from_bytes decodes image bytes (PNG / JPEG / ...) via stb_image to a
// linear RGBA texture (used for glTF-embedded textures, which are often JPEG).
pub fn texture_from_bytes(data []u8) !Texture {
	r, w, h := stbi_decode(data)!
	return texture_from_u8_rgba(r, w, h)
}

fn stbi_decode(data []u8) !([]u8, int, int) {
	img := stbi.load_from_memory(data.data, data.len, stbi.LoadParams{
		desired_channels: 4
	})!
	defer {
		unsafe { img.free() }
	}
	mut rgba := []u8{len: img.width * img.height * 4}
	unsafe {
		for i in 0 .. rgba.len {
			rgba[i] = img.data[i]
		}
	}
	return rgba, img.width, img.height
}

fn wrap_value(value mlx.Array, mode WrapMode) mlx.Array {
	return match mode {
		.repeat { value.subtract(value.floor()) }
		.clamp { mlx.s_clip(value, 0.0, 1.0) }
	}
}

// sample returns bilinearly interpolated RGBA texels for an (N,2) UV array.
pub fn (t Texture) sample(uv mlx.Array, wrap_s WrapMode, wrap_t WrapMode) mlx.Array {
	if uv.shape().len != 2 || uv.shape()[1] != 2 {
		panic('uv must have shape (count, 2)')
	}
	u := wrap_value(col(uv, 0), wrap_s)
	v := wrap_value(col(uv, 1), wrap_t)
	x := mlx.s_sub(mlx.s_mul(u, f64(t.width)), 0.5)
	y := mlx.s_sub(mlx.s_mul(mlx.s_rsub(v, 1.0), f64(t.height)), 0.5)
	x0 := x.floor().astype(.int32)
	y0 := y.floor().astype(.int32)
	fx := x.subtract(x0.astype(.float32)).expand_dims(1)
	fy := y.subtract(y0.astype(.float32)).expand_dims(1)
	mut x0r := x0
	mut x1 := x0.add(mlx.int_scalar(1))
	mut y0r := y0
	mut y1 := y0.add(mlx.int_scalar(1))
	if wrap_s == .repeat {
		x0r = x0.remainder(mlx.int_scalar(t.width))
		x1 = x1.remainder(mlx.int_scalar(t.width))
	} else {
		x0r = x0.clip(mlx.int_scalar(0), mlx.int_scalar(t.width - 1))
		x1 = x1.clip(mlx.int_scalar(0), mlx.int_scalar(t.width - 1))
	}
	if wrap_t == .repeat {
		y0r = y0.remainder(mlx.int_scalar(t.height))
		y1 = y1.remainder(mlx.int_scalar(t.height))
	} else {
		y0r = y0.clip(mlx.int_scalar(0), mlx.int_scalar(t.height - 1))
		y1 = y1.clip(mlx.int_scalar(0), mlx.int_scalar(t.height - 1))
	}
	flat := t.pixels.reshape([t.height * t.width, 4])
	c00 := flat.take_axis(y0r.multiply(mlx.int_scalar(t.width)).add(x0r), 0)
	c10 := flat.take_axis(y0r.multiply(mlx.int_scalar(t.width)).add(x1), 0)
	c01 := flat.take_axis(y1.multiply(mlx.int_scalar(t.width)).add(x0r), 0)
	c11 := flat.take_axis(y1.multiply(mlx.int_scalar(t.width)).add(x1), 0)
	omfx := mlx.fs(1.0).subtract(fx)
	omfy := mlx.fs(1.0).subtract(fy)
	top := c00.multiply(omfx).add(c10.multiply(fx))
	bot := c01.multiply(omfx).add(c11.multiply(fx))
	res := top.multiply(omfy).add(bot.multiply(fy)) // (count,4) raw interpolated
	rgb := res.take_axis(mlx.array_i32([i32(0), 1, 2], [3]), 1)
	lin := if t.is_linear {
		rgb
	} else {
		srgb_to_linear_arr(rgb)
	}
	a := res.take_axis(mlx.int_scalar(3), 1).expand_dims(1)
	return mlx.concatenate([lin, a], 1)
}
