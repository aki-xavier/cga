use cga_host::*;

// 运动副全景汇演：生成 → 场景报告 → 无头渲染，全部库级 API，无 CLI 无窗口。
// 生成物包含全部 8 种关节 + 两种高副（齿轮耦合 / 凸轮接触）。
fn main() {
    let text = gen_pairs_showcase();

    let out = "examples/pairs";
    let _ = std::fs::create_dir_all(out);
    std::fs::write(format!("{out}/pairs.jsx"), &text).unwrap();

    let run = run_jsx(&text, None, ".").expect("run");
    let report =
        cga_host::scene_report::scene_report(&run.scene, &run.camera, &run.tags, &run.kinematics);
    std::fs::write(format!("{out}/pairs.txt"), &report).unwrap();

    let img = render_jsx_png(&text, None, ".", 640, 480, 2).expect("headless render");
    std::fs::write(format!("{out}/pairs.png"), &img.png).unwrap();

    println!(
        "saved {out}/pairs.jsx + pairs.png + pairs.txt ({} lines, {} joints, {} gears, {} cams)",
        text.lines().count(),
        run.kinematics.joints.len(),
        run.kinematics.gears.len(),
        run.kinematics.cams.len()
    );
}
