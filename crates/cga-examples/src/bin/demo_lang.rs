use cga_host::*;

// 生成 / 无头渲染汇演: 结构化参数 → 逐行平铺 JSX(可 diff、必然可解析)
// → 无头渲染 PNG + 场景报告文本, 全部库级 API, 无 CLI 无窗口
fn main() {
    let text = gen_flange_assembly(
        &FlangeSpec::default(),
        &BoltCircleSpec::default(),
        &GearSpec::default(),
        &BasePlateSpec::default(),
    );

    let out = "examples/lang";
    let _ = std::fs::create_dir_all(out);
    std::fs::write(format!("{out}/generated_flange.jsx"), &text).unwrap();

    let img = render_jsx_png(&text, None, ".", 640, 480, 2).expect("headless render");
    std::fs::write(format!("{out}/demo_lang.png"), &img.png).unwrap();

    let run = run_jsx(&text, None, ".").expect("run");
    let report =
        cga_host::scene_report::scene_report(&run.scene, &run.camera, &run.tags, &run.kinematics);
    std::fs::write(format!("{out}/report.txt"), &report).unwrap();

    println!(
        "saved {out}/generated_flange.jsx ({} lines) + demo_lang.png + report.txt ({} lines)",
        text.lines().count(),
        report.lines().count()
    );
}
