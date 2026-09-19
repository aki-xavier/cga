// main.rs — CGS editor web server (serves the editor UI + renders CGS to PNG).
//
// Serves the web editor at `/` and renders CGS text to PNG at `POST /render`.
// A CGS parse error returns HTTP 400 with the message (the parser is
// Result-based, so it no longer crashes the server).
//
//   cargo run -p cga-editor        (from the rust/ workspace root)
//   open http://127.0.0.1:8123

// Syntax highlighting runs client-side in web/index.html; the server never
// calls this module, which is kept compiled and tested alongside it.
#[allow(dead_code)]
mod highlight;

use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::Arc;
use tiny_http::{Header, Method, Request, Response, Server};

const PORT: u16 = 8123;

// index.html is embedded so the binary is self-contained.
const INDEX_HTML: &str = include_str!("../web/index.html");

// Pasted CGS has no source file, so asset paths (material.map / mesh) resolve
// against the bundled examples/cgs/ directory (map="assets/brick.png" works).
// rust/crates/cga-editor -> repo root is three levels up.
const ASSET_ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/cgs");

// Number of http worker threads answering requests.  A slow render only blocks
// the worker that issued it.
const HTTP_WORKERS: usize = 4;

// RenderJob is one /render request handed to the render thread.
struct RenderJob {
    text: String,
    w: i32,
    h: i32,
    aa: i32,
    reply: mpsc::Sender<RenderReply>,
}

// RenderReply is the render thread's answer (exactly one of png/err set).
struct RenderReply {
    png: Vec<u8>,
    err: String,
}

fn handle(req: &mut Request, jobs: &SyncSender<RenderJob>) -> Response<std::io::Cursor<Vec<u8>>> {
    let url = req.url().to_string();
    let path = url.split('?').next().unwrap_or(&url);
    let query = match url.find('?') {
        Some(i) => &url[i + 1..],
        None => "",
    };
    if req.method() == &Method::Get && path == "/health" {
        return text_response(200, "text/plain", "ok");
    }
    if req.method() == &Method::Get && (path == "/" || path == "/index.html") {
        return text_response(200, "text/html; charset=utf-8", INDEX_HTML);
    }
    if req.method() == &Method::Post && path == "/render" {
        let w = query_param(query, "w", 720);
        let h = query_param(query, "h", 500);
        let aa = query_param(query, "aa", 1);
        let mut body = String::new();
        if req.as_reader().read_to_string(&mut body).is_err() {
            return text_response(
                400,
                "text/plain; charset=utf-8",
                "invalid UTF-8 request body",
            );
        }
        let (reply_tx, reply_rx) = mpsc::channel::<RenderReply>();
        // Non-blocking enqueue: when the render queue is full the worker thread
        // must not block on the send (that would exhaust the http worker pool
        // under concurrent renders); back off with 503 instead.
        let job = RenderJob {
            text: body,
            w,
            h,
            aa,
            reply: reply_tx,
        };
        if let Err(e) = jobs.try_send(job) {
            return match e {
                TrySendError::Full(_) => {
                    text_response(503, "text/plain", "render queue full, retry shortly")
                }
                TrySendError::Disconnected(_) => {
                    text_response(500, "text/plain", "render thread died")
                }
            };
        }
        return match reply_rx.recv() {
            Ok(res) => {
                if !res.err.is_empty() {
                    text_response(400, "text/plain; charset=utf-8", &res.err)
                } else {
                    bytes_response(200, "image/png", res.png)
                }
            }
            Err(_) => text_response(500, "text/plain", "render thread died"),
        };
    }
    text_response(404, "text/plain", "not found")
}

// text_response builds an HTTP response with a text body.
fn text_response(
    status: u16,
    content_type: &str,
    body: &str,
) -> Response<std::io::Cursor<Vec<u8>>> {
    bytes_response(status, content_type, body.as_bytes().to_vec())
}

// bytes_response builds an HTTP response with a binary body.
fn bytes_response(
    status: u16,
    content_type: &str,
    body: Vec<u8>,
) -> Response<std::io::Cursor<Vec<u8>>> {
    let header = Header::from_bytes(&b"Content-Type"[..], content_type.as_bytes())
        .expect("valid content-type header");
    Response::from_data(body)
        .with_status_code(status)
        .with_header(header)
}

// query_param reads an int query parameter with a default.  A present but
// non-numeric value parses to 0, which the render size check below then rejects
// with a 400.
fn query_param(query: &str, key: &str, def: i32) -> i32 {
    for pair in query.split('&') {
        let kv: Vec<&str> = pair.split('=').collect();
        if kv.len() == 2 && kv[0] == key {
            return kv[1].parse().unwrap_or(0);
        }
    }
    def
}

// render_cgs parses + renders CGS text to PNG bytes (returns a parse error
// instead of crashing on bad CGS).  The http server dispatches requests on
// worker threads, and MLX's default stream is thread-local — all renders run
// on the single render thread (see render_loop), which holds the default
// stream for its whole lifetime.
fn render_cgs(text: &str, w: i32, h: i32, aa: i32) -> Result<Vec<u8>, String> {
    if aa < 1 {
        return Err(format!("aa must be >= 1, got {aa}"));
    }
    if !(1..=4096).contains(&w) || !(1..=4096).contains(&h) {
        return Err(format!("render size out of range (1..4096), got {w}x{h}"));
    }
    let (sc, mut cam) = cga_gpu::cgs_load_result(text, ASSET_ROOT)?;
    cam.aspect = f64::from(w) / f64::from(h);
    let mut r = cga_gpu::renderer(w, h, aa, 3);
    let img = r.render(sc, cam);
    let png = cga_gpu::frame_to_png_bytes(&img);
    // A render leaves thousands of dead mlx arrays whose Metal buffers are only
    // released when the Rust `Array` handles drop; RAII handles that, but MLX's
    // caching allocator then hoards the freed Metal buffers without reusing
    // them (the cache grows past 2GB over a handful of identical 8MB-active
    // renders), so hand the cache back to the OS after each render.
    let _ = mlx_rs::memory::clear_cache();
    Ok(png)
}

// render_loop is the single render thread: all MLX work happens here, pinned
// to this thread.  The http server dispatches requests on a pool of worker
// threads and MLX's default stream is thread-local; registering the stream
// per request on a pool thread proved flaky under thread recycling
// (intermittent "no Stream(gpu, 1) in current thread" panics that kill the
// process), so /render jobs are funnelled here instead.  Renders serialize —
// fine for an editor preview.
fn render_loop(jobs: Receiver<RenderJob>) {
    let stream = mlx_rs::Stream::gpu();
    mlx_rs::with_stream(&stream, || {
        while let Ok(job) = jobs.recv() {
            // Catch a renderer panic here so one bad CGS turns into a 400
            // instead of a dead server.
            let reply = match std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                render_cgs(&job.text, job.w, job.h, job.aa)
            })) {
                Ok(Ok(png)) => RenderReply {
                    png,
                    err: String::new(),
                },
                Ok(Err(err)) => RenderReply {
                    png: Vec::new(),
                    err,
                },
                Err(_) => RenderReply {
                    png: Vec::new(),
                    err: "render failed (internal error)".to_string(),
                },
            };
            let _ = job.reply.send(reply);
        }
    });
}

fn main() {
    let (jobs_tx, jobs_rx) = mpsc::sync_channel::<RenderJob>(16);
    std::thread::spawn(move || render_loop(jobs_rx));
    let addr = format!("127.0.0.1:{PORT}");
    let server =
        Arc::new(Server::http(&addr).unwrap_or_else(|e| panic!("cannot bind {addr}: {e}")));
    println!("[cga-editor] Running app on http://{addr}/");
    let mut workers = Vec::new();
    for _ in 0..HTTP_WORKERS {
        let server = server.clone();
        let jobs_tx = jobs_tx.clone();
        workers.push(std::thread::spawn(move || {
            while let Ok(mut req) = server.recv() {
                let resp = handle(&mut req, &jobs_tx);
                // respond consumes the request; errors mean the client
                // went away — keep serving.
                let _ = req.respond(resp);
            }
        }));
    }
    // keep alive (the worker threads do the serving)
    for w in workers {
        let _ = w.join();
    }
}
