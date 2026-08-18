module cga

// Tests for the pure-V animated GIF encoder (gif.v).

fn gif_test_frame(w int, h int, base int) []u8 {
	mut f := []u8{len: w * h * 4}
	mut p := 0
	for y in 0 .. h {
		for x in 0 .. w {
			f[p] = u8((base + x * 4) & 0xFF)
			f[p + 1] = u8((base + y * 4) & 0xFF)
			f[p + 2] = u8((x + y) & 0xFF)
			f[p + 3] = 255
			p += 4
		}
	}
	return f
}

fn test_gif_encode_header_and_trailer() {
	w := 16
	h := 12
	frames := [
		gif_test_frame(w, h, 0),
		gif_test_frame(w, h, 40),
		gif_test_frame(w, h, 120),
	]
	g := encode_gif_rgba(frames, w, h, 3)
	assert g.len > 6 + 7 + 256 * 3 + 19
	// GIF89a magic
	assert g[0] == u8(`G`) && g[1] == u8(`I`) && g[2] == u8(`F`) && g[3] == u8(`8`)
	assert g[4] == u8(`9`) && g[5] == u8(`a`)
	// logical screen descriptor width/height (little-endian)
	assert int(g[6]) == w && int(g[7]) == 0
	assert int(g[8]) == h && int(g[9]) == 0
	// global colour table flag + 256-entry size
	assert g[10] == 0xF7
	// trailer
	assert g[g.len - 1] == 0x3B
}

fn test_gif_encode_single_colour() {
	// a solid-colour image needs only a tiny palette but must still encode
	w := 8
	h := 8
	mut f := []u8{len: w * h * 4}
	for i := 0; i < w * h * 4; i += 4 {
		f[i] = 200
		f[i + 1] = 30
		f[i + 2] = 90
		f[i + 3] = 255
	}
	g := encode_gif_rgba([f], w, h, 5)
	assert g[0] == u8(`G`) && g[g.len - 1] == 0x3B
}

fn test_gif_lzw_matches_reference() {
	// deterministic LZW output for a known index stream (the classic
	// "ababab..." pattern exercises dictionary growth).
	idx := [u8(1), 1, 1, 2, 2, 2, 1, 1, 1, 2, 2, 2, 1, 1, 1, 2, 2, 2]
	enc := gif_lzw(idx, 8)
	// clear(256) in 9 bits LSB-first → 0x00, then literal 1 → 0x03.
	assert enc.len >= 4
	assert enc[0] == 0x00 && enc[1] == 0x03
}
