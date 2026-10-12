//! 逐帧渲染：宿主每帧推入输入 `IN.t`，场景按 React 语义**局部**重渲染（props 监听），
//! 然后重建场景并出图。hook 状态跨帧保留（示例场景用 useEffect 计帧数）。
use cga_examples::load_scene;
use cga_host::*;
use clap::Parser;

#[derive(Parser)]
struct Args {
    /// Input .jsx scene (imports of ./x.css are followed)
    src: String,
    /// Output prefix (frames are written as <prefix>_00.png, <prefix>_01.png, …)
    #[arg(default_value = "frame")]
    prefix: String,
    /// Image width
    #[arg(default_value_t = 320)]
    w: i32,
    /// Image height
    #[arg(default_value_t = 240)]
    h: i32,
    /// Samples per pixel (anti-aliasing)
    #[arg(default_value_t = 1)]
    aa: i32,
    /// Number of frames
    #[arg(default_value_t = 6)]
    frames: i32,
}

fn main() {
    let args = Args::parse();
    let (text, css, dir) = load_scene(&args.src);
    let root = match dir.rfind('/') {
        Some(i) => dir[..i].to_string(),
        None => ".".to_string(),
    };
    let mut sess =
        SceneSession::open(&text, css.as_deref(), &root).unwrap_or_else(|e| panic!("{e}"));
    for i in 0..args.frames {
        let t = f64::from(i) / f64::from(args.frames);
        let stats = sess
            .set_input(&format!("{{\"t\":{t:.6}}}"))
            .unwrap_or_else(|e| panic!("frame {i}: {e}"));
        // 增量渲染：只重追值可能变化的光线，与全帧渲染逐位一致。
        let (img, istats) = sess
            .render_incremental(args.w, args.h, args.aa, RenderMode::Normal)
            .unwrap_or_else(|e| panic!("frame {i}: {e}"));
        let out = format!("{}_{i:02}.png", args.prefix);
        std::fs::write(&out, &img.png).unwrap_or_else(|e| panic!("write {out}: {e}"));
        let c = sess.counters().unwrap_or_default();
        let b = sess.build_stats();
        println!(
            "{out}: t={t:.3} objects={} drain rounds={} create={} update={} reused={}/{} dirty={}/{} full={}({})",
            sess.run().scene.objects.len(),
            stats.rounds,
            c.create,
            c.update,
            b.reused_objects,
            b.total_objects,
            istats.dirty,
            istats.total,
            istats.full,
            istats.reason,
        );
        sess.reset_counters().ok();
    }
}
