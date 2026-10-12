use cga_examples::{load_scene, replace_ext};
use cga_host::*;
use clap::Parser;

#[derive(Parser)]
struct Args {
    /// Input .jsx scene (imports of ./x.css are followed)
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
    /// Ignore transparency: render everything opaque (no reflection/refraction,
    /// transparent occluders shadow like solid ones)
    #[arg(long)]
    opaque: bool,
}

fn main() {
    let args = Args::parse();
    let src = &args.src;
    let out = args.out.unwrap_or_else(|| replace_ext(src, "png"));
    let (text, css, dir) = load_scene(src);
    let mode = if args.opaque {
        RenderMode::IgnoreOpacity
    } else {
        RenderMode::Normal
    };
    let img = render_jsx_png_mode(&text, css.as_deref(), &dir, args.w, args.h, args.aa, mode)
        .unwrap_or_else(|e| panic!("{e}"));
    std::fs::write(&out, &img.png).unwrap();
    println!(
        "saved {out} ({}x{}, aa={}, mode={})",
        img.width,
        img.height,
        args.aa,
        if args.opaque { "opaque" } else { "normal" }
    );
}
