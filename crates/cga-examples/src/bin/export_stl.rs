//! `export_stl`：JSX 场景 → STL（默认二进制，`--ascii` 出文本）。
//!
//! 非实体几何（平面/圆）自动跳过并计数；场景里没有实体时报错。
use cga_host::*;
use clap::Parser;

#[derive(Parser)]
struct Args {
    /// Input .jsx scene (imports of ./x.css are followed)
    src: String,
    /// Output STL (default: <src> minus extension + .stl)
    out: Option<String>,
    /// Tessellation step (smaller = finer)
    #[arg(default_value_t = 0.05)]
    step: f64,
    /// Write ASCII STL instead of binary
    #[arg(long)]
    ascii: bool,
}

/// 读取 JSX 并跟随 `import './x.css'` 引用（与 render_jsx 同规则）。
pub fn load_scene(src: &str) -> (String, Option<String>, String) {
    let text = std::fs::read_to_string(src).unwrap_or_else(|_| panic!("cannot read {src}"));
    let dir = match src.rfind('/') {
        Some(i) => src[..i].to_string(),
        None => ".".to_string(),
    };
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
    (text, if css.is_empty() { None } else { Some(css) }, dir)
}

fn main() {
    let args = Args::parse();
    let out = args.out.unwrap_or_else(|| match args.src.rfind('.') {
        Some(i) => format!("{}.stl", &args.src[..i]),
        None => format!("{}.stl", args.src),
    });
    let (text, css, dir) = load_scene(&args.src);
    if args.ascii {
        let name = std::path::Path::new(&args.src)
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "scene".to_string());
        let (stl, skipped) = scene_to_stl_ascii(&text, css.as_deref(), &dir, args.step, &name)
            .unwrap_or_else(|e| panic!("{e}"));
        std::fs::write(&out, stl).unwrap();
        println!(
            "saved {out} (ASCII STL, step={}, skipped {}: {})",
            args.step,
            skipped.len(),
            skipped.join("; ")
        );
    } else {
        let (stl, skipped) =
            scene_to_stl(&text, css.as_deref(), &dir, args.step).unwrap_or_else(|e| panic!("{e}"));
        let facets = if stl.len() >= 84 {
            u32::from_le_bytes([stl[80], stl[81], stl[82], stl[83]])
        } else {
            0
        };
        std::fs::write(&out, stl).unwrap();
        println!(
            "saved {out} (binary STL, {facets} facets, step={}, skipped {}: {})",
            args.step,
            skipped.len(),
            skipped.join("; ")
        );
    }
}
