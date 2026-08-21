module cga

// Minimal PNG writer (RGBA, 8-bit) so render output can be saved to disk
// without external dependencies.  Uses V's zlib + crc32 modules.
import hash.crc32
import compress.zlib
import os
import mlx

// save_frame_png writes an (H, W, 4) float32 render frame as a PNG file.
pub fn save_frame_png(path string, img mlx.Array) {
	sh := img.shape()
	h := sh[0]
	w := sh[1]
	data := img.data_f32()
	save_png_rgba(path, w, h, f32_rgba_to_u8(data))
}

// frame_to_png_bytes encodes an (H, W, 4) float32 render frame as PNG bytes.
pub fn frame_to_png_bytes(img mlx.Array) []u8 {
	sh := img.shape()
	h := sh[0]
	w := sh[1]
	data := img.data_f32()
	return encode_png_rgba(w, h, f32_rgba_to_u8(data))
}

// encode_png_rgba encodes an RGBA image (row-major bytes) as PNG bytes.
pub fn encode_png_rgba(width int, height int, rgba []u8) []u8 {
	// each scanline is prefixed with a 0 filter byte
	mut raw := []u8{len: height * (width * 4 + 1)}
	for y in 0 .. height {
		raw[y * (width * 4 + 1)] = 0
		for x in 0 .. width * 4 {
			raw[y * (width * 4 + 1) + 1 + x] = rgba[y * width * 4 + x]
		}
	}
	idat := zlib.compress(raw) or { panic('zlib compress failed: ${err}') }

	mut out := []u8{}
	out << [u8(0x89), 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A] // PNG signature
	mut ihdr := []u8{}
	ihdr << be32(width)
	ihdr << be32(height)
	ihdr << [u8(8), 6, 0, 0, 0] // bit depth 8, colour type 6 (RGBA), no compression/filter/interlace
	out << png_chunk('IHDR', ihdr)
	out << png_chunk('IDAT', idat)
	out << png_chunk('IEND', [])
	return out
}

// save_png_rgba writes an RGBA image (row-major bytes) as a PNG file.
pub fn save_png_rgba(path string, width int, height int, rgba []u8) {
	os.write_file(path, encode_png_rgba(width, height, rgba).bytestr()) or {
		panic('cannot write ${path}')
	}
}

// f32_rgba_to_u8 converts float RGBA (0..255) to byte RGBA.
pub fn f32_rgba_to_u8(data []f32) []u8 {
	mut out := []u8{len: data.len}
	for i, v in data {
		if v < 0.0 {
			out[i] = 0
		} else if v > 255.0 {
			out[i] = 255
		} else {
			out[i] = u8(v + 0.5)
		}
	}
	return out
}

fn png_chunk(typ string, data []u8) []u8 {
	mut crc_data := typ.bytes()
	crc_data << data
	crc := crc32.sum(crc_data)
	mut chunk := []u8{}
	chunk << be32(data.len)
	chunk << typ.bytes()
	chunk << data
	chunk << be32(int(crc))
	return chunk
}

fn be32(v int) []u8 {
	return [u8(v >> 24), u8((v >> 16) & 0xFF), u8((v >> 8) & 0xFF), u8(v & 0xFF)]
}

fn be32_at(b []u8, pos int) int {
	return int(u32(b[pos]) << 24 | u32(b[pos + 1]) << 16 | u32(b[pos + 2]) << 8 | u32(b[pos + 3]))
}

fn paeth(a int, b int, c int) int {
	p := a + b - c
	pa := if p > a { p - a } else { a - p }
	pb := if p > b { p - b } else { b - p }
	pc := if p > c { p - c } else { c - p }
	if pa <= pb && pa <= pc {
		return a
	}
	if pb <= pc {
		return b
	}
	return c
}

// load_png_rgba decodes an 8-bit non-interlaced PNG (greyscale / RGB / RGBA /
// greyscale+alpha) into RGBA bytes, returning (pixels, width, height).
pub fn load_png_rgba(path string) !([]u8, int, int) {
	data := os.read_bytes(path) or { return error('cannot read ${path}') }
	if data.len < 8 || data[0] != 0x89 || data[1] != 0x50 || data[2] != 0x4E || data[3] != 0x47 {
		return error('${path} is not a PNG')
	}
	mut pos := 8
	mut width := 0
	mut height := 0
	mut bit_depth := 0
	mut color_type := 0
	mut interlace := 0
	mut idat := []u8{}
	for pos + 8 <= data.len {
		length := be32_at(data, pos)
		pos += 4
		typ := data[pos..pos + 4].bytestr()
		pos += 4
		chunk := data[pos..pos + length]
		pos += length + 4 // skip crc
		if typ == 'IHDR' && chunk.len >= 13 {
			width = be32_at(chunk, 0)
			height = be32_at(chunk, 4)
			bit_depth = int(chunk[8])
			color_type = int(chunk[9])
			interlace = int(chunk[12])
		} else if typ == 'IDAT' {
			idat << chunk
		} else if typ == 'IEND' {
			break
		}
	}
	if bit_depth != 8 {
		return error('only 8-bit PNG supported (got ${bit_depth})')
	}
	if interlace != 0 {
		return error('interlaced PNG not supported')
	}
	bpp := match color_type {
		0 { 1 }
		2 { 3 }
		4 { 2 }
		6 { 4 }
		else { return error('unsupported PNG colour type ${color_type}') }
	}
	raw := zlib.decompress(idat) or { return error('PNG inflate failed: ${err}') }
	stride := width * bpp
	mut img := []u8{len: height * stride}
	// prev holds the already-decoded previous scanline (zeros for row 0); one
	// reusable buffer instead of a fresh allocation + clone per row.
	mut prev := []u8{len: stride}
	for y in 0 .. height {
		filter := int(raw[y * (stride + 1)])
		row_start := y * (stride + 1) + 1
		for x in 0 .. stride {
			cur := int(raw[row_start + x])
			left := if x >= bpp { int(img[y * stride + x - bpp]) } else { 0 }
			up := int(prev[x])
			ul := if x >= bpp { int(prev[x - bpp]) } else { 0 }
			val := match filter {
				1 { cur + left }
				2 { cur + up }
				3 { cur + (left + up) / 2 }
				4 { cur + paeth(left, up, ul) }
				else { cur }
			}
			img[y * stride + x] = u8(val & 0xFF)
		}
		copy(mut prev, img[(y * stride)..((y + 1) * stride)])
	}
	mut out := []u8{len: width * height * 4}
	mut i := 0
	for y in 0 .. height {
		for x in 0 .. width {
			off := y * stride + x * bpp
			match color_type {
				0 {
					g := img[off]
					out[i] = g
					out[i + 1] = g
					out[i + 2] = g
					out[i + 3] = 255
				}
				2 {
					out[i] = img[off]
					out[i + 1] = img[off + 1]
					out[i + 2] = img[off + 2]
					out[i + 3] = 255
				}
				4 {
					g := img[off]
					out[i] = g
					out[i + 1] = g
					out[i + 2] = g
					out[i + 3] = img[off + 1]
				}
				6 {
					out[i] = img[off]
					out[i + 1] = img[off + 1]
					out[i + 2] = img[off + 2]
					out[i + 3] = img[off + 3]
				}
				else {}
			}
			i += 4
		}
	}
	return out, width, height
}
