//! `export_urdf`：JSX 场景 → URDF（+ 非图元链接几何的 STL 网格文件）。
//!
//! 网格文件写到 `<out 所在目录>/meshes/<link>_<slot>.stl`，URDF 里以相对路径引用。
use cga_host::*;
use clap::Parser;

#[derive(Parser)]
struct Args {
    /// Input .jsx scene (imports of ./x.css are followed)
    src: String,
    /// Output .urdf (default: <src> minus extension + .urdf)
    out: Option<String>,
    /// Robot name (default: input file stem)
    #[arg(long)]
    name: Option<String>,
    /// Tessellation step for non-primitive link geometry
    #[arg(default_value_t = 0.05)]
    step: f64,
}

fn main() {
    let args = Args::parse();
    let out = args.out.unwrap_or_else(|| match args.src.rfind('.') {
        Some(i) => format!("{}.urdf", &args.src[..i]),
        None => format!("{}.urdf", args.src),
    });
    let robot = args.name.unwrap_or_else(|| {
        std::path::Path::new(&args.src)
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "robot".to_string())
    });
    let text =
        std::fs::read_to_string(&args.src).unwrap_or_else(|_| panic!("cannot read {}", args.src));
    let dir = match args.src.rfind('/') {
        Some(i) => args.src[..i].to_string(),
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
    let e = jsx_to_urdf(
        &text,
        if css.is_empty() { None } else { Some(&css) },
        &dir,
        &robot,
        args.step,
    )
    .unwrap_or_else(|e| panic!("{e}"));
    let base = std::path::Path::new(&out)
        .parent()
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| std::path::PathBuf::from("."));
    std::fs::create_dir_all(&base).unwrap();
    std::fs::write(&out, &e.xml).unwrap();
    // 网格文件与 URDF 同目录（URDF 里是相对路径 meshes/…）
    for (rel, data) in &e.meshes {
        let path = base.join(rel);
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).unwrap();
        }
        std::fs::write(&path, data).unwrap();
    }
    println!(
        "saved {out} (robot \"{robot}\", {} mesh file(s), step={})",
        e.meshes.len(),
        args.step
    );
}
