module main

// CGS render CLI: v run examples/render_cgs.v <file.cgs> [out.png] [w h aa]
import cga
import os

fn main() {
	if os.args.len < 2 {
		eprintln('usage: render_cgs <file.cgs> [out.png] [w h aa]')
		exit(1)
	}
	src := os.args[1]
	out := if os.args.len > 2 { os.args[2] } else { src.all_before_last('.') + '.png' }
	w := if os.args.len > 3 { os.args[3].int() } else { 640 }
	h := if os.args.len > 4 { os.args[4].int() } else { 480 }
	aa := if os.args.len > 5 { os.args[5].int() } else { 2 }
	text := os.read_file(src) or { panic('cannot read ${src}') }
	sc, mut cam := cga.cgs_load(text, src.all_before_last('/'))
	cam.aspect = f64(w) / f64(h)
	mut r := cga.renderer(w, h, aa, 3)
	// render_scene_with_splats handles scenes with splat layers (plain
	// Renderer.render would leave them invisible)
	img := cga.render_scene_with_splats(sc, mut r, cam)
	cga.save_frame_png(out, img)
	println('saved ${out}')
}
