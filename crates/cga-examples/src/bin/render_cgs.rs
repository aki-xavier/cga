use cga_gpu::*;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: render_cgs <file.cgs> [out.png] [w h aa]");
        std::process::exit(1);
    }
    let src = &args[1];

    let out = if args.len() > 2 {
        args[2].clone()
    } else {
        match src.rfind('.') {
            Some(i) => format!("{}.png", &src[..i]),
            None => format!("{src}.png"),
        }
    };
    let w: i32 = if args.len() > 3 {
        args[3].parse().unwrap_or(0)
    } else {
        640
    };
    let h: i32 = if args.len() > 4 {
        args[4].parse().unwrap_or(0)
    } else {
        480
    };
    let aa: i32 = if args.len() > 5 {
        args[5].parse().unwrap_or(0)
    } else {
        2
    };
    let text = std::fs::read_to_string(src).unwrap_or_else(|_| panic!("cannot read {src}"));

    let asset_root = match src.rfind('/') {
        Some(i) => &src[..i],
        None => src.as_str(),
    };
    let (sc, mut cam) = cgs_load(&text, asset_root);
    cam.aspect = f64::from(w) / f64::from(h);
    let mut r = renderer(w, h, aa, 3);
    let img = r.render(sc, cam);
    save_frame_png(&out, &img);
    println!("saved {out}");
}
