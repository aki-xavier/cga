module main

// server.v — CGS editor web server (port of cga_py/scene_lang/render_server.py
// plus the gpui editor's preview backend).
//
// Serves the web editor at `/` and renders CGS text to PNG at `POST /render`.
// A CGS parse error returns HTTP 400 with the message (the parser is
// Result-based, so it no longer crashes the server).
//
//   v -gc boehm run editor/server.v        (from the repo root)
//   open http://127.0.0.1:8123
import cga
import mlx
import net.http
import os
import time

const port = 8123

const index_html_path = os.dir(@FILE) + '/web/index.html'

struct EditorHandler {
	jobs chan RenderJob
}

// RenderJob is one /render request handed to the render thread.
struct RenderJob {
	text  string
	w     int
	h     int
	aa    int
	reply chan RenderReply
}

// RenderReply is the render thread's answer (exactly one of png/err set).
struct RenderReply {
	png []u8
	err string
}

fn (mut handler EditorHandler) handle(req http.Request) http.Response {
	path := req.url.all_before('?')
	query := req.url.all_after('?')
	if req.method == .get && path == '/health' {
		return text_response(200, 'text/plain', 'ok')
	}
	if req.method == .get && (path == '/' || path == '/index.html') {
		html := os.read_file(index_html_path) or {
			return text_response(500, 'text/plain', 'editor web/index.html not found')
		}
		return text_response(200, 'text/html; charset=utf-8', html)
	}
	if req.method == .post && path == '/render' {
		w := query_param(query, 'w', 720)
		hgt := query_param(query, 'h', 500)
		aa := query_param(query, 'aa', 1)
		reply := chan RenderReply{cap: 1}
		// Non-blocking enqueue: when the render queue is full the worker thread
		// must not block on the send (that would exhaust the http worker pool
		// under concurrent renders); back off with 503 instead.
		if handler.jobs.try_push(RenderJob{
			text:  req.data
			w:     w
			h:     hgt
			aa:    aa
			reply: reply
		}) != .success {
			return text_response(503, 'text/plain', 'render queue full, retry shortly')
		}
		res := <-reply
		if res.err != '' {
			return text_response(400, 'text/plain; charset=utf-8', res.err)
		}
		return bytes_response(200, 'image/png', res.png)
	}
	return text_response(404, 'text/plain', 'not found')
}

// text_response builds an HTTP response with a text body.
fn text_response(status int, content_type string, body string) http.Response {
	mut resp := http.Response{}
	resp.status_code = status
	resp.header.set(.content_type, content_type)
	resp.body = body
	return resp
}

// bytes_response builds an HTTP response with a binary body.
fn bytes_response(status int, content_type string, body []u8) http.Response {
	mut resp := http.Response{}
	resp.status_code = status
	resp.header.set(.content_type, content_type)
	resp.body = body.bytestr()
	return resp
}

// query_param reads an int query parameter with a default.
fn query_param(query string, key string, def int) int {
	for pair in query.split('&') {
		kv := pair.split('=')
		if kv.len == 2 && kv[0] == key {
			return kv[1].int()
		}
	}
	return def
}

// render_cgs parses + renders CGS text to PNG bytes (returns a parse error
// instead of crashing on bad CGS).  The http server dispatches requests on
// worker threads, and MLX's default stream is thread-local — register the
// shared stream on this thread before touching any MLX array, or eval
// panics with "no Stream(gpu, 1) in current thread".  Pasted CGS has no
// source file, so asset paths (material.map / mesh) resolve against the
// bundled examples/ directory (map="assets/brick.png" works).
// render_cgs parses + renders CGS text to PNG bytes (returns a parse error
// instead of crashing on bad CGS).  The http server dispatches requests on
// worker threads, and MLX's default stream is thread-local — register the
// shared stream on this thread before touching any MLX array, or eval
// panics with "no Stream(gpu, 1) in current thread".  Pasted CGS has no
// source file, so asset paths (material.map / mesh) resolve against the
// bundled examples/cgs/ directory (map="assets/brick.png" works).
fn render_cgs(text string, w int, h int, aa int) ![]u8 {
	if aa < 1 {
		return error('aa must be >= 1, got ${aa}')
	}
	if w < 1 || h < 1 || w > 4096 || h > 4096 {
		return error('render size out of range (1..4096), got ${w}x${h}')
	}
	mlx.default_stream().set_default()
	asset_root := os.dir(@FILE) + '/../examples/cgs'
	sc, mut cam := cga.cgs_load_result(text, asset_root)!
	cam.aspect = f64(w) / f64(h)
	mut r := cga.renderer(w, h, aa, 3)
	img := r.render(sc, cam)
	return cga.frame_to_png_bytes(img)
}

// render_loop is the single render thread: all MLX work happens here, pinned
// to this thread.  V's http server dispatches requests on a pool of worker
// threads and MLX's default stream is thread-local; registering the stream
// per request on a pool thread proved flaky under thread recycling
// (intermittent "no Stream(gpu, 1) in current thread" panics that kill the
// process), so /render jobs are funnelled here instead.  Renders serialize —
// fine for an editor preview.
fn render_loop(jobs chan RenderJob) {
	mlx.default_stream().set_default()
	for {
		job := <-jobs
		png := render_cgs(job.text, job.w, job.h, job.aa) or {
			job.reply <- RenderReply{
				err: err.msg()
			}
			continue
		}
		job.reply <- RenderReply{
			png: png
		}
	}
}

fn main() {
	jobs := chan RenderJob{cap: 16}
	spawn render_loop(jobs)
	mut server := http.Server{
		addr:                 '127.0.0.1:${port}'
		handler:              EditorHandler{
			jobs: jobs
		}
		show_startup_message: true
	}
	server.listen_and_serve()
	// keep alive (listen_and_serve returns after the listener is set up)
	for {
		time.sleep(1 * time.second)
	}
}
