//! report_cgs: CGS file -> Scene Report on stdout (`docs/scene-report.md`).
//!
//! usage: report_cgs <file.cgs>
//! Report generation itself is a total function; load errors and I/O failures
//! go to stderr with exit(1) — a CLI choice, not part of the language error
//! contract.

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 2 {
        eprintln!("usage: report_cgs <file.cgs>");
        std::process::exit(1);
    }
    let src = &args[1];
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
