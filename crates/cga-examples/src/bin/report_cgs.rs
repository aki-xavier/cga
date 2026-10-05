fn main() {
    use clap::Parser;
    #[derive(Parser)]
    struct Args {
        /// Input .cgs scene
        src: String,
        /// Pose override, repeatable: --set name=value
        #[arg(long = "set", value_name = "NAME=VALUE")]
        set: Vec<String>,
    }
    let args = Args::parse();
    let src = &args.src;
    let text = std::fs::read_to_string(src).unwrap_or_else(|e| {
        eprintln!("report_cgs: cannot read {src}: {e}");
        std::process::exit(1);
    });
    let pose: Vec<(String, f64)> = args
        .set
        .iter()
        .map(|s| {
            let (n, v) = s.split_once('=').unwrap_or_else(|| {
                eprintln!("report_cgs: --set expects NAME=VALUE, got {s}");
                std::process::exit(1);
            });
            let x: f64 = v.parse().unwrap_or_else(|_| {
                eprintln!("report_cgs: --set bad value in {s}");
                std::process::exit(1);
            });
            (n.to_string(), x)
        })
        .collect();
    let asset_root = match src.rfind('/') {
        Some(i) => &src[..i],
        None => ".",
    };
    match cga_gpu::cgs_report_pose(&text, asset_root, &pose) {
        Ok(rep) => print!("{rep}"),
        Err(e) => {
            eprintln!("report_cgs: {e}");
            std::process::exit(1);
        }
    }
}
