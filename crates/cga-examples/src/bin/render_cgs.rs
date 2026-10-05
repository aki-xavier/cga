use cga_gpu::*;
use clap::Parser;

#[derive(Parser)]
struct Args {
    /// Input .cgs scene
    src: String,
    /// Output PNG (default: <src> minus extension + .png)
    out: Option<String>,
    /// Image width
    #[arg(default_value_t = 640)]
    w: i32,
    /// Image height
    #[arg(default_value_t = 480)]
    h: i32,
    /// Samples per pixel (anti-aliasing)
    #[arg(default_value_t = 2)]
    aa: i32,
}

fn main() {
    let args = Args::parse();
    let src = &args.src;

    let out = args.out.unwrap_or_else(|| match src.rfind('.') {
        Some(i) => format!("{}.png", &src[..i]),
        None => format!("{src}.png"),
    });
    let (w, h, aa) = (args.w, args.h, args.aa);
    let text = std::fs::read_to_string(src).unwrap_or_else(|_| panic!("cannot read {src}"));

    let asset_root = match src.rfind('/') {
        Some(i) => &src[..i],
        None => src.as_str(),
    };
    let (sc, mut cam) = cgs_load(&text, asset_root);
    cam.aspect = f64::from(w) / f64::from(h);
    let mut r = Renderer::new(w, h, aa, 3);
    let img = r.render(sc, cam);
    save_frame_png(&out, &img);
    println!("saved {out}");
}
