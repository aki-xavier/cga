use clap::Parser;

#[derive(Parser)]
struct Args {
    /// Input .jsx scene (imports of ./x.css are followed)
    src: String,
    /// Pose override, repeatable: --set name=value
    #[arg(long = "set", value_name = "NAME=VALUE")]
    set: Vec<String>,
}

fn main() {
    let args = Args::parse();
    let src = &args.src;
    let text = std::fs::read_to_string(src).unwrap_or_else(|e| {
        eprintln!("report_jsx: cannot read {src}: {e}");
        std::process::exit(1);
    });
    let pose: Vec<(String, f64)> = args
        .set
        .iter()
        .map(|s| {
            let (n, v) = s.split_once('=').unwrap_or_else(|| {
                eprintln!("report_jsx: --set expects NAME=VALUE, got {s}");
                std::process::exit(1);
            });
            let x: f64 = v.parse().unwrap_or_else(|_| {
                eprintln!("report_jsx: --set bad value in {s}");
                std::process::exit(1);
            });
            (n.to_string(), x)
        })
        .collect();
    let dir = match src.rfind('/') {
        Some(i) => &src[..i],
        None => ".",
    };
    // 跟随 import './x.css'
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
    match cga_host::run_jsx_pose(
        &text,
        if css.is_empty() { None } else { Some(&css) },
        dir,
        &pose,
    ) {
        Ok(run) => print!(
            "{}",
            cga_host::scene_report::scene_report(
                &run.scene,
                &run.camera,
                &run.tags,
                &run.kinematics,
                &run.mass_props
            )
        ),
        Err(e) => {
            eprintln!("report_jsx: {e}");
            std::process::exit(1);
        }
    }
}
