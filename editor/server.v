module main

// server.v — CGS editor web server (port of cga_py/scene_lang/render_server.py
// plus the gpui editor's preview backend).
//
// Serves the web editor at `/` and renders CGS text to PNG at `POST /render`.
//
//   v -gc boehm run editor/server.v        (from the repo root)
//   open http://127.0.0.1:8123
//
// NOTE: a CGS parse error currently panics the server process (cgs_load uses
// `panic`, which V cannot catch).  Converting cgs_load to a Result-based API is
// the follow-up needed for graceful HTTP 400 responses.

import cga
import json
import net.http
import os
import time

const port = 8123

const index_html_path = os.dir(@FILE) + '/web/index.html'

struct EditorHandler {}

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
		png := render_cgs(req.data, w, hgt, aa)
		return bytes_response(200, 'image/png', png)
	}
	if req.method == .post && path == '/params' {
		return text_response(200, 'application/json', json.encode(extract_params(req.data)))
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

// render_cgs parses + renders CGS text to PNG bytes.
fn render_cgs(text string, w int, h int, aa int) []u8 {
	sc, mut cam := cga.cgs_load(text, '')
	cam.aspect = f64(w) / f64(h)
	mut r := cga.renderer(w, h, aa, 3)
	img := r.render(sc, cam)
	return cga.frame_to_png_bytes(img)
}

fn main() {
	mut server := http.Server{
		addr: '127.0.0.1:${port}'
		handler: EditorHandler{}
		show_startup_message: true
	}
	server.listen_and_serve()
	// keep alive (listen_and_serve returns after the listener is set up)
	for {
		time.sleep(1 * time.second)
	}
}
