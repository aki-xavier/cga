#[allow(dead_code)]
mod highlight;

use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::Arc;
use tiny_http::{Header, Method, Request, Response, Server};

const PORT: u16 = 8123;

const INDEX_HTML: &str = include_str!("../web/index.html");

const ASSET_ROOT: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/cgs");

const HTTP_WORKERS: usize = 4;

struct RenderJob {
    text: String,
    w: i32,
    h: i32,
    aa: i32,
    reply: mpsc::Sender<RenderReply>,
}

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

fn text_response(
    status: u16,
    content_type: &str,
    body: &str,
) -> Response<std::io::Cursor<Vec<u8>>> {
    bytes_response(status, content_type, body.as_bytes().to_vec())
}

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

fn query_param(query: &str, key: &str, def: i32) -> i32 {
    for pair in query.split('&') {
        let kv: Vec<&str> = pair.split('=').collect();
        if kv.len() == 2 && kv[0] == key {
            return kv[1].parse().unwrap_or(0);
        }
    }
    def
}

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

    let _ = mlx_rs::memory::clear_cache();
    Ok(png)
}

fn render_loop(jobs: Receiver<RenderJob>) {
    let stream = mlx_rs::Stream::gpu();
    mlx_rs::with_stream(&stream, || {
        while let Ok(job) = jobs.recv() {
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

                let _ = req.respond(resp);
            }
        }));
    }

    for w in workers {
        let _ = w.join();
    }
}
