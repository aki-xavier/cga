module cga

// Minimal animated GIF89a encoder (pure stdlib, no dependencies).
//
// Encodes a sequence of RGBA frames with a global 256-colour palette
// (median-cut quantisation over all frames) and the LZW image compression
// required by the GIF format.  Used by the demos to assemble their PNG frames
// into an animated .gif entirely in V.

import os

const gif_palette_entries = 256

// ---- colour quantisation --------------------------------------------------

struct GCColor {
	r int
	g int
	b int
	c int
}

// gif_palette builds a 256-entry global palette (RGB triples) covering every
// frame, via a 5-bit histogram + median cut.
fn gif_palette(frames [][]u8, w int, h int) []u8 {
	mut hist := map[u32]int{}
	for frame in frames {
		mut p := 0
		for _ in 0 .. w * h {
			r := int(frame[p]) * int(frame[p + 3]) / 255
			g := int(frame[p + 1]) * int(frame[p + 3]) / 255
			b := int(frame[p + 2]) * int(frame[p + 3]) / 255
			key := (u32(r) >> 3 << 10) | (u32(g) >> 3 << 5) | (u32(b) >> 3)
			hist[key] = hist[key] + 1
			p += 4
		}
	}
	mut colors := []GCColor{}
	for k, cnt in hist {
		colors << GCColor{
			r: (int(k >> 10) & 31) * 8 + 4
			g: (int(k >> 5) & 31) * 8 + 4
			b: (int(k) & 31) * 8 + 4
			c: cnt
		}
	}
	if colors.len == 0 {
		colors << GCColor{r: 0, g: 0, b: 0, c: 1}
	}
	// median cut: repeatedly split the largest box until 256 boxes.
	mut boxes := [][]GCColor{}
	boxes << colors
	for boxes.len < gif_palette_entries {
		mut bi := -1
		mut blen := 1
		for i in 0 .. boxes.len {
			if boxes[i].len > blen {
				blen = boxes[i].len
				bi = i
			}
		}
		if bi < 0 {
			break
		}
		box := boxes[bi]
		mut rmin := 255
		mut rmax := 0
		mut gmin := 255
		mut gmax := 0
		mut bmin := 255
		mut bmax := 0
		for cc in box {
			if cc.r < rmin {
				rmin = cc.r
			}
			if cc.r > rmax {
				rmax = cc.r
			}
			if cc.g < gmin {
				gmin = cc.g
			}
			if cc.g > gmax {
				gmax = cc.g
			}
			if cc.b < bmin {
				bmin = cc.b
			}
			if cc.b > bmax {
				bmax = cc.b
			}
		}
		rr := rmax - rmin
		gg := gmax - gmin
		bb := bmax - bmin
		mut ch := 0
		if gg > rr && gg >= bb {
			ch = 1
		}
		if bb > rr && bb > gg {
			ch = 2
		}
		mut sorted := box.clone()
		if ch == 0 {
			sorted.sort_with_compare(gc_cmp_r)
		} else if ch == 1 {
			sorted.sort_with_compare(gc_cmp_g)
		} else {
			sorted.sort_with_compare(gc_cmp_b)
		}
		mid := sorted.len / 2
		boxes[bi] = sorted[..mid]
		boxes << sorted[mid..]
	}
	mut pal := []u8{len: gif_palette_entries * 3}
	for i in 0 .. gif_palette_entries {
		if i < boxes.len {
			mut sr := 0
			mut sg := 0
			mut sb := 0
			mut sc := 0
			for cc in boxes[i] {
				sr += cc.r * cc.c
				sg += cc.g * cc.c
				sb += cc.b * cc.c
				sc += cc.c
			}
			if sc > 0 {
				pal[i * 3] = u8(sr / sc)
				pal[i * 3 + 1] = u8(sg / sc)
				pal[i * 3 + 2] = u8(sb / sc)
			}
		}
	}
	return pal
}

fn gc_cmp_r(a &GCColor, b &GCColor) int {
	return if a.r < b.r {
		-1
	} else if a.r > b.r {
		1
	} else {
		0
	}
}

fn gc_cmp_g(a &GCColor, b &GCColor) int {
	return if a.g < b.g {
		-1
	} else if a.g > b.g {
		1
	} else {
		0
	}
}

fn gc_cmp_b(a &GCColor, b &GCColor) int {
	return if a.b < b.b {
		-1
	} else if a.b > b.b {
		1
	} else {
		0
	}
}

// gif_palette_lut maps every 5-bit colour to its nearest palette index.
fn gif_palette_lut(pal []u8) []u8 {
	mut lut := []u8{len: 32768}
	for r5 in 0 .. 32 {
		for g5 in 0 .. 32 {
			for b5 in 0 .. 32 {
				r := r5 * 8 + 4
				g := g5 * 8 + 4
				b := b5 * 8 + 4
				mut best := 0
				mut bestd := 1 << 30
				for i in 0 .. gif_palette_entries {
					dr := r - int(pal[i * 3])
					dg := g - int(pal[i * 3 + 1])
					db := b - int(pal[i * 3 + 2])
					d := dr * dr + dg * dg + db * db
					if d < bestd {
						bestd = d
						best = i
					}
				}
				lut[r5 * 1024 + g5 * 32 + b5] = u8(best)
			}
		}
	}
	return lut
}

// gif_quantize maps one RGBA frame to palette indices using the LUT.
fn gif_quantize(frame []u8, w int, h int, lut []u8) []u8 {
	mut out := []u8{len: w * h}
	mut p := 0
	for i in 0 .. w * h {
		r := int(frame[p]) * int(frame[p + 3]) / 255
		g := int(frame[p + 1]) * int(frame[p + 3]) / 255
		b := int(frame[p + 2]) * int(frame[p + 3]) / 255
		out[i] = lut[int(u32(r) >> 3) * 1024 + int(u32(g) >> 3) * 32 + int(u32(b) >> 3)]
		p += 4
	}
	return out
}

// ---- LZW compression (GIF variant, LSB-first, variable width) ------------

struct GBitWriter {
mut:
	out   []u8
	bits  u32
	nbits int
}

fn (mut w GBitWriter) write(code u32, width int) {
	w.bits |= code << w.nbits
	w.nbits += width
	for w.nbits >= 8 {
		w.out << u8(w.bits & 0xFF)
		w.bits >>= 8
		w.nbits -= 8
	}
}

// gif_lzw compresses a stream of palette indices using the GIF LZW scheme.
fn gif_lzw(indices []u8, min_code_size int) []u8 {
	clear := u32(1) << u32(min_code_size)
	eoi := clear + 1
	mut code_size := min_code_size + 1
	mut next_code := clear + 1
	mut overflow := u32(1) << u32(min_code_size + 1)
	mut table := map[u32]u32{}

	mut w := GBitWriter{}
	w.write(clear, code_size)

	if indices.len == 0 {
		w.write(eoi, code_size)
		if w.nbits > 0 {
			w.out << u8(w.bits & 0xFF)
		}
		return w.out
	}

	mut code := u32(indices[0])
	for i in 1 .. indices.len {
		x := u32(indices[i])
		key := code << 8 | x
		if key in table {
			code = table[key]
			continue
		}
		w.write(code, code_size)
		code = x
		// assign the next dictionary code, growing the width or resetting
		// the table when full.
		next_code++
		if next_code == overflow {
			code_size++
			overflow <<= 1
		}
		if next_code == u32(4095) {
			w.write(clear, code_size)
			code_size = min_code_size + 1
			next_code = clear + 1
			overflow = clear << 1
			table = map[u32]u32{}
			continue
		}
		table[key] = next_code
	}
	w.write(code, code_size)
	// mirror the decoder's post-read dictionary step before the EOI code.
	next_code++
	if next_code == overflow {
		code_size++
		overflow <<= 1
	}
	if next_code == u32(4095) {
		w.write(clear, code_size)
		code_size = min_code_size + 1
	}
	w.write(eoi, code_size)
	if w.nbits > 0 {
		w.out << u8(w.bits & 0xFF)
	}
	return w.out
}

// ---- GIF assembly ---------------------------------------------------------

fn gif_u16le(v int) []u8 {
	return [u8(v & 0xFF), u8((v >> 8) & 0xFF)]
}

// gif_sub_blocks wraps compressed bytes into length-prefixed GIF sub-blocks.
fn gif_sub_blocks(data []u8) []u8 {
	mut out := []u8{}
	mut i := 0
	for i < data.len {
		n := if data.len - i > 255 { 255 } else { data.len - i }
		out << u8(n)
		out << data[i..i + n]
		i += n
	}
	out << u8(0)
	return out
}

// encode_gif_rgba encodes a sequence of RGBA frames (each `[]u8` of length
// w*h*4) as an animated GIF89a with a global palette and `delay_cs`
// centiseconds between frames.
pub fn encode_gif_rgba(frames [][]u8, w int, h int, delay_cs int) []u8 {
	pal := gif_palette(frames, w, h)
	lut := gif_palette_lut(pal)

	mut out := []u8{}
	out << 'GIF89a'.bytes()
	out << gif_u16le(w)
	out << gif_u16le(h)
	// packed: global colour table | colour resolution 7 | size 7 (=256 entries)
	out << [u8(0xF7), 0x00, 0x00]
	out << pal
	// NETSCAPE2.0 loop extension (loop forever)
	out << [u8(0x21), 0xFF, 0x0B]
	out << 'NETSCAPE2.0'.bytes()
	out << [u8(0x03), 0x01]
	out << gif_u16le(0)
	out << u8(0x00)
	for frame in frames {
		idx := gif_quantize(frame, w, h, lut)
		// graphic control extension: disposal method 1 (do not dispose)
		out << [u8(0x21), 0xF9, 0x04, 0x04]
		out << gif_u16le(delay_cs)
		out << [u8(0x00), 0x00]
		// image descriptor
		out << u8(0x2C)
		out << gif_u16le(0)
		out << gif_u16le(0)
		out << gif_u16le(w)
		out << gif_u16le(h)
		out << u8(0x00)
		// LZW minimum code size + image data sub-blocks
		out << u8(8)
		out << gif_sub_blocks(gif_lzw(idx, 8))
	}
	out << u8(0x3B)
	return out
}

// save_gif writes an animated GIF to `path`.
pub fn save_gif(path string, frames [][]u8, w int, h int, delay_cs int) {
	os.write_file(path, encode_gif_rgba(frames, w, h, delay_cs).bytestr()) or {
		panic('cannot write ${path}')
	}
}
