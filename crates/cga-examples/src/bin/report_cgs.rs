fn main() {
    use clap::Parser;
    #[derive(Parser)]
    struct Args {
        /// Input .cgs scene
        src: String,
    }
    let args = Args::parse();
    let src = &args.src;
    let text = std::fs::read_to_string(src).unwrap_or_else(|e| {
        eprintln!("report_cgs: cannot read {src}: {e}");
        std::process::exit(1);
    });
    let asset_root = match src.rfind('/') {
        Some(i) => &src[..i],
        None => ".",
    };
    match cga_gpu::cgs_report(&text, asset_root) {
        Ok(rep) => print!("{rep}"),
        Err(e) => {
            eprintln!("report_cgs: {e}");
            std::process::exit(1);
        }
    }
}
