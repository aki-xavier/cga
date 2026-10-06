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
}

fn main() {
    let args = Args::parse();
    let src = &args.src;
    let out = args.out.unwrap_or_else(|| match src.rfind('.') {
        Some(i) => format!("{}.png", &src[..i]),
        None => format!("{src}.png"),
    });
    let text = std::fs::read_to_string(src).unwrap_or_else(|_| panic!("cannot read {src}"));
    let dir = match src.rfind('/') {
        Some(i) => &src[..i],
        None => ".",
    };
    // Follow `import './x.css'` references (lightningcss parses the result).
    let mut css = String::new();
    let mut rest = text.as_str();
    while let Some(i) = rest.find(".css") {
        let head = &rest[..i + 4];
        if let Some(q) = head.rfind(['\'', '"']) {
            let path = &head[q + 1..];
            if path.starts_with("./") || path.starts_with("../") {
                css.push_str(
                    &std::fs::read_to_string(format!("{dir}/{path}"))
                        .unwrap_or_else(|_| panic!("cannot read {dir}/{path}")),
                );
                css.push('\n');
            }
        }
        rest = &rest[i + 4..];
    }
    let img = render_jsx_png(
        &text,
        if css.is_empty() { None } else { Some(&css) },
        dir,
        args.w,
        args.h,
        args.aa,
    )
    .unwrap_or_else(|e| panic!("{e}"));
    std::fs::write(&out, &img.png).unwrap();
    println!("saved {out} ({}x{}, aa={})", img.width, img.height, args.aa);
}
