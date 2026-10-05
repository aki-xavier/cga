use cga_gpu::*;

// 生成 / 无头渲染汇演: 结构化参数 → 逐行平铺 CGS(可 diff、必然可解析)
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
    std::fs::write(format!("{out}/generated_flange.cgs"), &text).unwrap();

    let img = render_cgs_png(&text, ".", 640, 480, 2).expect("headless render");
    std::fs::write(format!("{out}/demo_lang.png"), &img.png).unwrap();

    let report = cgs_report(&text, ".").expect("scene report");
    std::fs::write(format!("{out}/report.txt"), &report).unwrap();

    println!(
        "saved {out}/generated_flange.cgs ({} lines) + demo_lang.png + report.txt ({} lines)",
        text.lines().count(),
        report.lines().count()
    );
}
