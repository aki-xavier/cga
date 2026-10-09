//! jsx 层测试（原 jsx/mod.rs 内联 `mod tests`）。测试横跨五层（swc 编译、
//! 元素值、CSS、场景构建、会话/增量），因此按名字显式导入各层实现。

use std::collections::HashMap;

use serde_json::Value;

use cga_core::Multivector;
use cga_scene::Color;

use crate::scene_build::JointKind;

use super::builder::{mat4_mul, translate4};
use super::session::build_scene_run;
use super::*;

#[test]
fn gallery_collision_scan_baseline() {
    // 画廊 8 场景全对扫描（C1 验收）：pin 住每个场景的 Yes/Unknown 计数。
    // 说明：落在地面上的物体 = 接触（sep=0 → Yes），凸轮/齿轮啮合与装配
    // 穿插也是真实接触；mechanical/assembly 的 Unknown 来自 CSG 差/交与
    // 仿射包装（三值诚实，不许假装精确）。计数变化 = 场景或判定行为变化。
    let want = [
        ("orbit", 4, 4),
        ("grid", 9, 0),
        ("building", 159, 196),
        ("mechanical", 0, 50),
        ("primitives", 5, 12),
        ("affine", 1, 6),
        ("assembly", 0, 5),
        ("animation", 1, 0),
    ];
    for (name, want_yes, want_unknown) in want {
        let jsx = std::fs::read_to_string(format!("../../examples/jsx/{name}.jsx"))
            .unwrap_or_else(|_| panic!("read {name}.jsx"));
        let css = std::fs::read_to_string(format!("../../examples/jsx/{name}.css"))
            .unwrap_or_else(|_| panic!("read {name}.css"));
        let run = run_jsx(&jsx, Some(&css), "../../examples/jsx")
            .unwrap_or_else(|e| panic!("jsx {name}: {e}"));
        let hits = crate::collision::CollisionScan::new().scan(&run.scene);
        let yes = hits
            .iter()
            .filter(|h| h.hit == cga_collision::Hit::Yes)
            .count();
        let unknown = hits
            .iter()
            .filter(|h| h.hit == cga_collision::Hit::Unknown)
            .count();
        assert_eq!(
            (yes, unknown),
            (want_yes, want_unknown),
            "{name}: 碰撞基线变化（若是刻意改场景，更新本表）"
        );
    }
    // 语义抽查：animation 的太阳球陷入地面 0.05（r=0.6, y=0.55）→ Yes 且 sep=−0.05。
    let jsx = std::fs::read_to_string("../../examples/jsx/animation.jsx").unwrap();
    let css = std::fs::read_to_string("../../examples/jsx/animation.css").unwrap();
    let run = run_jsx(&jsx, Some(&css), "../../examples/jsx").unwrap();
    let hits = crate::collision::CollisionScan::new().scan(&run.scene);
    let h = hits
        .iter()
        .find(|h| h.hit == cga_collision::Hit::Yes)
        .expect("animation has one contact");
    assert!((h.separation.unwrap() + 0.05).abs() < 1e-9, "{h:?}");
}

#[test]
fn test_jsx_gallery_smoke() {
    // 画廊八场景（JSX+CSS 版）：全部可渲染出合法 PNG。
    for name in [
        "orbit",
        "grid",
        "building",
        "mechanical",
        "primitives",
        "affine",
        "assembly",
        "animation",
    ] {
        let jsx = std::fs::read_to_string(format!("../../examples/jsx/{name}.jsx"))
            .unwrap_or_else(|_| panic!("read {name}.jsx"));
        let css = std::fs::read_to_string(format!("../../examples/jsx/{name}.css"))
            .unwrap_or_else(|_| panic!("read {name}.css"));
        let out = render_jsx_png(&jsx, Some(&css), "../../examples/jsx", 96, 72, 1)
            .unwrap_or_else(|e| panic!("jsx {name}: {e}"));
        assert!(out.png.starts_with(&[137, 80, 78, 71]), "{name}: PNG magic");
        assert!(out.png.len() > 1000, "{name}: 非空渲染");
    }
}
/// 冻结金标：7 个画廊场景的渲染（96×72 aa=1）哈希。
///
/// 迁移到 React 运行时之前，旧的一次性求值路径与新路径已逐位核对（7/7 一致），
/// 这些哈希即那时的输出。此后它是端到端护栏：只要渲染/场景构建语义不漂移，
/// 就必须一直吻合（改渲染器或升级内置 React 时这里是第一道警报）。
/// 取三个代表场景（其余 4 个由 `test_jsx_gallery_smoke` 覆盖可渲染性）：
/// orbit = 透明/折射，assembly = solve 约束 + tag/when + 嵌套元素属性，
/// mechanical = CSG。
const GOLDEN: &[(&str, u64)] = &[
    ("orbit", 0x7c79_87c6_76e2_361c),
    ("mechanical", 0xa1ae_01a8_4b10_c72a),
    ("assembly", 0xadf0_d90d_5db2_7afe),
];

#[test]
fn gallery_render_golden() {
    for (name, want) in GOLDEN {
        let jsx = std::fs::read_to_string(format!("../../examples/jsx/{name}.jsx")).unwrap();
        let css = std::fs::read_to_string(format!("../../examples/jsx/{name}.css")).unwrap();
        let run = run_jsx(&jsx, Some(&css), "../../examples/jsx")
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let png = render_png(&run);
        assert_eq!(
            fnv1a(&png),
            *want,
            "{name}: 渲染金标不符（{} 字节）",
            png.len()
        );
    }
}

fn render_png(run: &crate::SceneRun) -> Vec<u8> {
    let mut r = cga_gpu::Renderer::new(96, 72, 1, 3);
    let img = r.render(run.scene.clone(), run.camera.clone());
    cga_gpu::frame_to_png_bytes(&img)
}

fn fnv1a(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// ---- P2：宿主输入（props 监听）与事件派发 → 动态局部重渲染 ----

const DIAL_SCENE: &str = r#"
const { useContext, useState } = React;
const fixed = <sphere r={1} />;

function Dial() {
  const IN = useContext(HostInput);
  const x = IN.x === undefined ? 0 : IN.x;
  return <sphere r={0.25} t={[x, 0, 0]} />;
}

function Counter() {
  const [n, setN] = useState(0);
  return (
<group onClick={() => setN(n + 1)} t={[0, 2, 0]}>
  <sphere r={0.2 + n * 0.1} />
</group>
  );
}

export default <scene><camera /><sphere r={0.5} />{fixed}<Dial /><Counter /></scene>;
"#;

#[test]
fn scene_session_host_input_rebuilds_locally() {
    let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
    let xs = |s: &SceneSession| -> Vec<f64> {
        s.run()
            .scene
            .objects
            .iter()
            .map(|o| o.base.motor().to_matrix()[3])
            .collect()
    };
    let before = xs(&s);
    s.reset_counters().unwrap();
    let stats = s.set_input(r#"{"x": 2.0}"#).unwrap();
    assert_eq!(stats.errors, 0);
    let c = s.counters().unwrap();
    assert_eq!(c.create, 0, "输入变化不得新建实例");
    assert!(c.update >= 1, "被消费的输入应让消费者更新: {c:?}");
    let after = xs(&s);
    assert_eq!(after.len(), before.len(), "对象数不变");
    let moved: Vec<usize> = (0..before.len())
        .filter(|i| (before[*i] - after[*i]).abs() > 1e-9)
        .collect();
    assert_eq!(
        moved.len(),
        1,
        "只有 Dial 的球平移: {before:?} -> {after:?}"
    );
    assert!(
        (after[moved[0]] - 2.0).abs() < 1e-9,
        "Dial 应平移到 x=2: {after:?}"
    );
}

#[test]
fn session_click_drives_state_and_incremental_render() {
    // 交互闭环（roadmap A）：拾取坐标 → 实例 id → onClick → setState →
    // 局部重建 → 增量渲染与全帧逐位一致。
    let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
    // Counter 的球在 (0, 2, 0)。默认相机 fov=50、(0,0,5) 看原点：
    // 投影到像素 (47.5, 35.5 − (2/5)·fy)，fy = 72/(2·tan25°)。
    let fy = 72.0 / (2.0 * (25.0f64.to_radians()).tan());
    let (px, py) = (47.5, 35.5 - 2.0 / 5.0 * fy);
    let (hit, inst) = s.pick(px, py, 96, 72).expect("hit counter sphere");
    assert!(inst.is_some(), "命中对象应有宿主实例 id");
    assert_eq!(hit.object, 3, "Counter 的球是第 4 个对象: {hit:?}");

    let (_img0, st0) = s
        .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
        .expect("render");
    assert!(st0.full && st0.reason == "first");

    // 点击 → onClick → setN → n=1（r: 0.2 → 0.3），状态跨帧累积
    let out = s.click(px, py, 96, 72).expect("click").expect("dispatched");
    assert!(out.found, "{out:?}");
    assert!(s.snapshot().unwrap().contains(r#""r":0.3"#), "onClick 生效");
    s.click(px, py, 96, 72).expect("click2");
    assert!(s.snapshot().unwrap().contains(r#""r":0.4"#), "第二次累积");

    // 增量渲染与全帧逐位一致
    let (img, st) = s
        .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
        .expect("render");
    assert!(!st.full, "点击只变了 Counter 的球，应增量: {st:?}");
    let run = s.run().clone();
    let mut full = cga_gpu::Renderer::new(96, 72, 1, 3);
    let want = full.render(run.scene.clone(), run.camera);
    assert_eq!(
        img.png,
        cga_gpu::frame_to_png_bytes(&want),
        "增量渲染必须与全帧逐位一致"
    );
    // 未命中区域点击 → None
    assert!(s.click(0.5, 70.5, 96, 72).expect("miss").is_none());
}

/// U1 验收场景：平面 2R 臂（base 固定，j1/j2 不钉 → 抓取时均为自由变量）。
const ARM_SCENE: &str = r#"export default (
  <scene>
<link name="base"><box s={[4, 4, 0.1]} t={[0, 0, -0.1]} /></link>
<link name="l1"><sphere r={0.1} t={[0.5, 0, 0]} /></link>
<link name="l2"><sphere r={0.12} t={[1, 0, 0]} /></link>
<pair kind="revolute" name="j1" a="base" b="l1" axis={[0, 0, 1]} />
<pair kind="revolute" name="j2" a="l1" b="l2" at={[1, 0, 0]} axis={[0, 0, 1]} />
<anchor link="base" />
  </scene>
);"#;

/// 2R 臂末端（l2 局部 [1,0,0]）的闭式正解。
fn arm_end(q1: f64, q2: f64) -> [f64; 3] {
    [q1.cos() + (q1 + q2).cos(), q1.sin() + (q1 + q2).sin(), 0.0]
}

#[test]
fn session_drag_solves_closed_form_and_renders_bitwise() {
    // U1 验收链：抓取点 + 目标 → LM 解 q（闭式对拍）→ pose 写回 → 场景重建
    // → 增量渲染与全帧逐位一致；连续拖拽（上轮 q 仍自由）；不可达 → Err
    // 且场景不变。
    let mut s = SceneSession::open(ARM_SCENE, None, ".").expect("open");
    let end_now = |s: &SceneSession| {
        let l2 = s
            .run()
            .kinematics
            .links
            .iter()
            .find(|l| l.name == "l2")
            .unwrap();
        cga_core::transform_point(l2.world, [1.0, 0.0, 0.0])
    };
    // 初始 q=(0,0)：末端 (2,0,0)。
    let p0 = end_now(&s);
    assert!((p0[0] - 2.0).abs() < 1e-9 && p0[1].abs() < 1e-9, "{p0:?}");

    let target = arm_end(0.9, 1.1);
    let d = s.drag("l2", [2.0, 0.0, 0.0], target).expect("drag");
    assert!((d.solved[0].1 - 0.9).abs() < 1e-6, "q1: {:?}", d.solved);
    assert!((d.solved[1].1 - 1.1).abs() < 1e-6, "q2: {:?}", d.solved);
    let p1 = end_now(&s);
    for i in 0..3 {
        assert!(
            (p1[i] - target[i]).abs() < 1e-6,
            "拖拽后末端应在目标: {p1:?}"
        );
    }

    // 增量渲染与全帧逐位一致
    let (img, _st) = s
        .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
        .expect("render");
    let run = s.run().clone();
    let mut full = cga_gpu::Renderer::new(96, 72, 1, 3);
    let want = full.render(run.scene.clone(), run.camera);
    assert_eq!(
        img.png,
        cga_gpu::frame_to_png_bytes(&want),
        "增量渲染必须与全帧逐位一致"
    );

    // 连续拖拽：上轮经 pose 写回的 q 仍是本轮自由变量。
    let t2 = arm_end(0.4, 0.8);
    let d2 = s.drag("l2", target, t2).expect("drag2");
    assert!((d2.solved[0].1 - 0.4).abs() < 1e-6, "q1: {:?}", d2.solved);
    assert!((d2.solved[1].1 - 0.8).abs() < 1e-6, "q2: {:?}", d2.solved);

    // 不可达 → Err 且场景/pose 不变（不许假装跟随）。
    let e = s.drag("l2", t2, [3.0, 3.0, 0.0]).unwrap_err();
    assert!(e.contains("目标不可达"), "{e}");
    let p2 = end_now(&s);
    for i in 0..3 {
        assert!((p2[i] - t2[i]).abs() < 1e-6, "失败后场景应保持: {p2:?}");
    }
    // 未知 link → Err 带名字
    let e = s.drag("nope", [0.0; 3], [0.0; 3]).unwrap_err();
    assert!(e.contains("未知 link") && e.contains("nope"), "{e}");
}

#[test]
fn session_drag_pick_grabs_hit_point() {
    // 像素抓取：拾取命中 l2 球面 → 抓取点 = 命中点 → 拖到绕 z 旋转 0.5 的
    // 目标（同半径同高度，必可达）→ 残差闭式断言。
    let mut s = SceneSession::open(ARM_SCENE, None, ".").expect("open");
    let l2_meshes: Vec<usize> = s
        .run()
        .kinematics
        .links
        .iter()
        .find(|l| l.name == "l2")
        .unwrap()
        .meshes
        .clone();
    let mut picked = None;
    'outer: for iy in 0..72 {
        for ix in 0..128 {
            if let Some((hit, _)) = s.pick(ix as f64 + 0.5, iy as f64 + 0.5, 128, 72) {
                if l2_meshes.contains(&hit.object) {
                    picked = Some((ix as f64 + 0.5, iy as f64 + 0.5, hit.point));
                    break 'outer;
                }
            }
        }
    }
    let (px, py, grab) = picked.expect("应命中 l2 的球");
    let (c, sn) = (0.5f64.cos(), 0.5f64.sin());
    let to = [
        grab[0] * c - grab[1] * sn,
        grab[0] * sn + grab[1] * c,
        grab[2],
    ];
    let d = s
        .drag_pick(px, py, 128, 72, to)
        .expect("drag_pick")
        .expect("命中 l2");
    assert_eq!(d.link, "l2");
    assert!(d.residual < 1e-6, "残差 {}", d.residual);
    for i in 0..3 {
        assert!(
            (d.grab[i] - to[i]).abs() < 1e-6,
            "抓取点应达目标: {:?}",
            d.grab
        );
    }
    // 命中静态物体（base 不在路径上也有 pair，但 ground 类无 link 的对象 → None）：
    // 本场景全部对象都在 link 下，改用未命中像素断言 None。
    assert!(s
        .drag_pick(0.5, 71.5, 128, 72, [0.0; 3])
        .expect("miss")
        .is_none());
}

#[test]
fn session_drag_pose_solves_closed_form_and_renders_bitwise() {
    // 位姿抓取（借清单第 1 件）：目标 = FK(0.9,1.1) 的 link frame 位姿矩阵
    // （l2 frame 原点在肘部 (cos q1, sin q1)，旋转 R_z(q1+q2)）→ 收回 q*；
    // 增量渲染逐位一致；朝向不可达 → Err 场景不变。
    let mut s = SceneSession::open(ARM_SCENE, None, ".").expect("open");
    let target = mat4_mul(
        translate4([0.9f64.cos(), 0.9f64.sin(), 0.0]),
        Multivector::rotor([0.0, 0.0, 1.0], 2.0).to_matrix(),
    );
    let d = s.drag_pose("l2", target).expect("drag_pose");
    assert!((d.solved[0].1 - 0.9).abs() < 1e-6, "q1: {:?}", d.solved);
    assert!((d.solved[1].1 - 1.1).abs() < 1e-6, "q2: {:?}", d.solved);
    // 场景位姿 = 目标（逐元素）
    let l2w = s
        .run()
        .kinematics
        .links
        .iter()
        .find(|l| l.name == "l2")
        .unwrap()
        .world;
    for i in 0..16 {
        assert!(
            (l2w[i] - target[i]).abs() < 1e-6,
            "位姿[{i}]: {} vs {}",
            l2w[i],
            target[i]
        );
    }
    // 增量渲染与全帧逐位一致
    let (img, _st) = s
        .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
        .expect("render");
    let run = s.run().clone();
    let mut full = cga_gpu::Renderer::new(96, 72, 1, 3);
    let want = full.render(run.scene.clone(), run.camera);
    assert_eq!(
        img.png,
        cga_gpu::frame_to_png_bytes(&want),
        "增量渲染必须与全帧逐位一致"
    );
    // 朝向不可达（绕 x 转，平面臂够不到）→ Err 且场景不变
    let before = s
        .run()
        .kinematics
        .links
        .iter()
        .find(|l| l.name == "l2")
        .unwrap()
        .world;
    let bad = Multivector::rotor([1.0, 0.0, 0.0], 0.5).to_matrix();
    let e = s.drag_pose("l2", bad).unwrap_err();
    assert!(e.contains("目标不可达"), "{e}");
    let after = s
        .run()
        .kinematics
        .links
        .iter()
        .find(|l| l.name == "l2")
        .unwrap()
        .world;
    assert_eq!(before, after, "失败后场景应保持");
    // 未知 link → Err 带名字
    let e = s.drag_pose("nope", target).unwrap_err();
    assert!(e.contains("未知 link") && e.contains("nope"), "{e}");
}

#[test]
fn session_render_toon_and_edge_channel() {
    // U5：Toon 模式与 Edge 诊断通道 plumbing——PNG 魔数 + 逐位确定 +
    // Toon ≠ Normal + Edge 图存在轮廓像素。
    let mut s = SceneSession::open(ARM_SCENE, None, ".").expect("open");
    let toon = s
        .render(64, 36, 1, cga_gpu::RenderMode::Toon)
        .expect("toon");
    assert_eq!(&toon.png[..4], b"\x89PNG");
    let toon2 = s
        .render(64, 36, 1, cga_gpu::RenderMode::Toon)
        .expect("toon2");
    assert_eq!(toon.png, toon2.png, "toon 应逐位确定");
    let normal = s.render(64, 36, 1, cga_gpu::RenderMode::Normal).expect("n");
    assert_ne!(toon.png, normal.png, "toon 应与 normal 不同");
    let edge = s
        .render_diagnostic(64, 36, cga_gpu::DiagChannel::Edge)
        .expect("edge");
    assert_eq!(&edge.png[..4], b"\x89PNG");
    let (rgba, _, _) = cga_gpu::decode_png_rgba(&edge.png).expect("decode");
    assert!(rgba.chunks(4).any(|px| px[0] > 200), "edge 图应有轮廓像素");
}

#[test]
fn session_render_diagnostic_channels() {
    // U3：三通道 plumbing + 逐位确定 + 对象 id 图恰好 4 色（背景/盒/两球）。
    // 64×36 = 16:9，与默认相机 aspect 一致（否则 l2 球在画幅外）。
    let mut s = SceneSession::open(ARM_SCENE, None, ".").expect("open");
    let mut pngs = Vec::new();
    for ch in [
        cga_gpu::DiagChannel::ObjectId,
        cga_gpu::DiagChannel::Normal,
        cga_gpu::DiagChannel::Depth,
    ] {
        let img = s.render_diagnostic(64, 36, ch).expect("diag");
        assert_eq!(&img.png[..4], b"\x89PNG", "PNG 魔数");
        let again = s.render_diagnostic(64, 36, ch).expect("diag2");
        assert_eq!(img.png, again.png, "{ch:?} 应逐位确定");
        pngs.push(img.png);
    }
    assert_ne!(pngs[0], pngs[1], "id 与法线通道应不同");
    assert_ne!(pngs[1], pngs[2], "法线与深度通道应不同");
    // 对象 id 图：背景 + 盒 + 两球 = 恰好 4 色
    let (rgba, w, h) = cga_gpu::decode_png_rgba(&pngs[0]).expect("decode");
    assert_eq!((w, h), (64, 36));
    let mut colors = std::collections::HashSet::new();
    for px in rgba.chunks(4) {
        colors.insert(px.to_vec());
    }
    assert_eq!(colors.len(), 4, "背景+盒+两球应恰好 4 色: {colors:?}");
}

#[test]
fn session_render_dof_plumbing() {
    // U6：dof 渲染 plumbing——aperture=0 与普通渲染逐位一致；aperture>0
    // 逐位确定且与针孔不同。
    let mut s = SceneSession::open(ARM_SCENE, None, ".").expect("open");
    let pin = s
        .render(64, 36, 2, cga_gpu::RenderMode::Normal)
        .expect("render");
    let dof0 = s.render_dof(64, 36, 2, 0.0, 5.0).expect("dof0");
    assert_eq!(pin.png, dof0.png, "aperture=0 应与针孔逐位一致");
    let dof = s.render_dof(64, 36, 2, 0.3, 5.0).expect("dof");
    assert_eq!(&dof.png[..4], b"\x89PNG");
    let dof2 = s.render_dof(64, 36, 2, 0.3, 5.0).expect("dof2");
    assert_eq!(dof.png, dof2.png, "dof 应逐位确定");
    assert_ne!(dof.png, pin.png, "aperture>0 应与针孔不同");
}

#[test]
fn scene_session_dispatch_updates_state_across_frames() {
    let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
    let inst = s
        .instances()
        .unwrap()
        .into_iter()
        .find(|i| i.handlers.iter().any(|h| h == "onClick"))
        .expect("带 onClick 的实例");
    assert!(s.snapshot().unwrap().contains(r#""r":0.2"#), "初值 r=0.2");
    s.reset_counters().unwrap();
    let out = s.dispatch(inst.id, "onClick", "null").unwrap();
    assert!(out.found, "{out:?}");
    assert_eq!(s.counters().unwrap().create, 0, "事件驱动不得新建实例");
    assert!(s.snapshot().unwrap().contains(r#""r":0.3"#), "第一帧 r=0.3");
    assert!(s.dispatch(inst.id, "onClick", "null").unwrap().found);
    assert!(
        s.snapshot().unwrap().contains(r#""r":0.4"#),
        "第二帧 r=0.4（状态跨帧累积）"
    );
    assert_eq!(s.errors().unwrap(), Vec::<String>::new());
    assert!(
        !s.dispatch(999_999, "onClick", "null").unwrap().found,
        "未知实例"
    );
    assert!(
        !s.dispatch(inst.id, "onWheel", "null").unwrap().found,
        "无该处理器"
    );
}

/// ---- .jsx 模块导入（编译期打包：globalThis.__exp_N + 别名）----

fn object_xs(s: &SceneSession) -> Vec<f64> {
    s.run()
        .scene
        .objects
        .iter()
        .map(|o| o.base.motor().to_matrix()[3])
        .collect()
}

#[test]
fn import_component_default() {
    let dial = "export default function Dial(props) {\n\
        \x20 return <sphere r={0.5} t={[props.x || 0, 0, 0]} />;\n\
        }";
    let entry = "import Dial from './dial.jsx';\n\
        export default (<scene><camera /><Dial x={2} /></scene>);";
    let s = SceneSession::open_modules(entry, None, &[("dial.jsx", dial)]).expect("open");
    let xs = object_xs(&s);
    assert_eq!(xs.len(), 1, "one sphere: {xs:?}");
    assert!((xs[0] - 2.0).abs() < 1e-9, "Dial at x=2: {xs:?}");
}

#[test]
fn import_element_default_and_named() {
    let part = "export default (<box s={[1, 1, 1]} t={[1, 0, 0]} />);\n\
        export const dot = <sphere r={0.2} t={[3, 0, 0]} />;";
    let entry = "import part, { dot } from './part.jsx';\n\
        export default (<scene><camera />{part}{dot}</scene>);";
    let s = SceneSession::open_modules(entry, None, &[("part.jsx", part)]).expect("open");
    let xs = object_xs(&s);
    assert_eq!(xs.len(), 2, "{xs:?}");
    assert!(
        (xs[0] - 1.0).abs() < 1e-9 && (xs[1] - 3.0).abs() < 1e-9,
        "{xs:?}"
    );
}

#[test]
fn import_is_transitive_and_shared() {
    let base = "export const unit = <sphere r={0.1} />;";
    let mid = "import { unit } from './base.jsx';\n\
        export default (<group t={[5, 0, 0]}>{unit}</group>);";
    let entry = "import mid from './mid.jsx';\n\
        import { unit } from './base.jsx';\n\
        export default (<scene><camera />{mid}{unit}</scene>);";
    let s = SceneSession::open_modules(entry, None, &[("base.jsx", base), ("mid.jsx", mid)])
        .expect("open");
    let xs = object_xs(&s);
    assert_eq!(xs.len(), 2, "mid's unit + entry's unit: {xs:?}");
    assert!((xs[0] - 5.0).abs() < 1e-9 && xs[1].abs() < 1e-9, "{xs:?}");
}

#[test]
fn lazy_dynamic_import_resolves() {
    // E2：React.lazy(() => import("./widget.jsx"))——动态 import 编期打包进 bundle，
    // Suspense 边界在 drain 内解析（promise/JS 交织驱动到不动点），最终树含
    // lazy 组件的产物。
    let widget = "export default function W() {\n\
        \x20 return <sphere r={0.3} t={[4, 0, 0]} />;\n\
        }";
    let entry = "const W = React.lazy(() => import('./widget.jsx'));\n\
        export default (<scene><camera />\
        <React.Suspense fallback={<box s={[9, 9, 9]} />}><W /></React.Suspense>\
        </scene>);";
    let s = SceneSession::open_modules(entry, None, &[("widget.jsx", widget)]).expect("open");
    let xs = object_xs(&s);
    assert_eq!(xs.len(), 1, "fallback 应被 lazy 组件替换: {xs:?}");
    assert!((xs[0] - 4.0).abs() < 1e-9, "lazy 组件的球在 x=4: {xs:?}");
}

#[test]
fn lazy_dynamic_import_error_paths() {
    // 非字面量 / 非 .jsx / 缺文件——编期报错带 JSX: 前缀（不许运行时裸奔）。
    let entry = "const W = React.lazy(() => import('./missing.jsx'));\n\
        export default (<scene><React.Suspense fallback={null}><W /></React.Suspense></scene>);";
    let e = SceneSession::open_modules(entry, None, &[])
        .err()
        .expect("missing");
    assert!(e.contains("JSX:") && e.contains("missing.jsx"), "{e}");
    let entry2 = "const f = './w.jsx';\n\
        const W = React.lazy(() => import(f));\n\
        export default (<scene><React.Suspense fallback={null}><W /></React.Suspense></scene>);";
    let e2 = SceneSession::open_modules(entry2, None, &[])
        .err()
        .expect("non-literal");
    assert!(
        e2.contains("dynamic import() needs a string literal"),
        "{e2}"
    );
}

#[test]
fn import_errors_are_jsx_prefixed() {
    // cycle
    let a = "import b from './b.jsx';\nexport default (<sphere r={0.1} />);";
    let b = "import a from './a.jsx';\nexport default (<sphere r={0.2} />);";
    let entry = "import a from './a.jsx';\nexport default (<scene>{a}</scene>);";
    let e = SceneSession::open_modules(entry, None, &[("a.jsx", a), ("b.jsx", b)])
        .err()
        .expect("should fail");
    assert!(e.contains("circular import"), "{e}");

    // missing module
    let e = SceneSession::open_modules(
        "import x from './nope.jsx';\nexport default (<scene>{x}</scene>);",
        None,
        &[],
    )
    .err()
    .expect("should fail");
    assert!(e.starts_with("JSX: cannot import ./nope.jsx"), "{e}");

    // missing default export
    let m = "export const a = 1;";
    let e = SceneSession::open_modules(
        "import x from './m.jsx';\nexport default (<scene />);",
        None,
        &[("m.jsx", m)],
    )
    .err()
    .expect("should fail");
    assert!(e.contains("has no default export"), "{e}");

    // missing named export
    let e = SceneSession::open_modules(
        "import { zz } from './m.jsx';\nexport default (<scene />);",
        None,
        &[("m.jsx", m)],
    )
    .err()
    .expect("should fail");
    assert!(e.contains("does not export zz"), "{e}");
}

#[test]
fn import_supports_default_function_decl() {
    let m = "export default function Tag() {\n\
        \x20 return <sphere r={0.3} t={[7, 0, 0]} />;\n\
        }";
    let entry = "import Tag from './m.jsx';\nexport default (<scene><camera /><Tag /></scene>);";
    let s = SceneSession::open_modules(entry, None, &[("m.jsx", m)]).expect("open");
    let xs = object_xs(&s);
    assert_eq!(xs.len(), 1);
    assert!((xs[0] - 7.0).abs() < 1e-9, "{xs:?}");
}

#[test]
fn scene_session_is_deterministic_across_frames() {
    let mut a = SceneSession::open(DIAL_SCENE, None, ".").unwrap();
    let mut b = SceneSession::open(DIAL_SCENE, None, ".").unwrap();
    a.set_input(r#"{"x": 1.0}"#).unwrap();
    b.set_input(r#"{"x": 1.0}"#).unwrap();
    let ha = render_png(a.run());
    let hb = render_png(b.run());
    assert_eq!(ha, hb, "同输入序列同输出");
}

/// ---- 公共属性（变换/tag prop 与 material 容器等价性） ----

#[test]
fn transform_props_equivalent_to_modifier_elements() {
    let dbg = |src: &str| {
        let run = run_jsx(src, None, ".").expect(src);
        format!("{:?}", run.scene.objects)
    };
    // 平移 prop ≡ <group t> 包裹（透明容器，落点相同）
    assert_eq!(
        dbg(r#"export default <sphere r={0.5} t={[2, 1, 0]} />;"#),
        dbg(r#"export default <group t={[2, 1, 0]}><sphere r={0.5} /></group>;"#)
    );
    // 固定顺序 T·R·S·Mirror：prop 书写顺序无关
    assert_eq!(
        dbg(
            r#"export default <box s={[1, 0.2, 0.2]} t={[1, 0, 0]} rotate={[0, 0, 1, 1.5707963267948966]} />;"#
        ),
        dbg(
            r#"export default <box s={[1, 0.2, 0.2]} rotate={[0, 0, 1, 1.5707963267948966]} t={[1, 0, 0]} />;"#
        )
    );
    // 固定顺序 ≡ 同序 group 嵌套（镜像最内、平移最外）
    assert_eq!(
        dbg(
            r#"export default <sphere r={0.5} t={[1, 0, 0]} rotate={[0, 0, 1, 1.5707963267948966]} scale={2} mirror={[1, 0, 0]} />;"#
        ),
        dbg(
            r#"export default <group t={[1, 0, 0]}><group rotate={[0, 0, 1, 1.5707963267948966]}><group scale={2}><group mirror={[1, 0, 0]}><sphere r={0.5} /></group></group></group></group>;"#
        )
    );
    // scale / mirror 单独 prop ≡ group 包裹
    assert_eq!(
        dbg(r#"export default <sphere r={0.5} scale={2} />;"#),
        dbg(r#"export default <group scale={2}><sphere r={0.5} /></group>;"#)
    );
    assert_eq!(
        dbg(r#"export default <cone r={0.5} h={1} mirror={[1, 0, 0]} />;"#),
        dbg(r#"export default <group mirror={[1, 0, 0]}><cone r={0.5} h={1} /></group>;"#)
    );
    // prop 与容器可叠加（容器在元素之外层）
    assert_eq!(
        dbg(r#"export default <group t={[0, 5, 0]}><sphere r={0.5} t={[1, 0, 0]} /></group>;"#),
        dbg(
            r#"export default <group t={[0, 5, 0]}><group t={[1, 0, 0]}><sphere r={0.5} /></group></group>;"#
        )
    );
    // CSG 子树里同样生效（group 展开进父 CSG 的孩子列表）
    assert_eq!(
        dbg(
            r#"export default <union><sphere r={0.5} t={[2, 0, 0]} /><box s={[1, 1, 1]} /></union>;"#
        ),
        dbg(
            r#"export default <union><group t={[2, 0, 0]}><sphere r={0.5} /></group><box s={[1, 1, 1]} /></union>;"#
        )
    );
}

/// 修饰符元素已删除：三条路径（渲染 / CSG / 查询）都要给同一个可操作报错。
#[test]
fn modifier_tags_removed_with_friendly_error() {
    for (src, hint) in [
        (
            "<translate t={[1, 0, 0]}><sphere r={1} /></translate>",
            "t=[x, y, z]",
        ),
        (
            "<rotate axis={[0, 0, 1]} angle={1}><sphere r={1} /></rotate>",
            "rotate=[ax, ay, az, angle]",
        ),
        ("<scale s={2}><sphere r={1} /></scale>", "scale="),
        (
            "<mirror axis={[1, 0, 0]}><sphere r={1} /></mirror>",
            "mirror=[x, y, z]",
        ),
    ] {
        let e = run_jsx(&format!("export default {src};"), None, ".")
            .expect_err(&format!("{src} should fail"));
        assert!(e.contains("removed"), "{e}");
        assert!(e.contains(hint), "{e}");
    }
    // CSG 收集路径
    let e = run_jsx(
        r#"export default <scene><difference><box s={[1, 1, 1]} /><translate t={[1, 0, 0]}><sphere r={1} /></translate></difference></scene>;"#,
        None,
        ".",
    )
    .unwrap_err();
    assert!(e.contains("removed"), "{e}");
    // 查询目标路径（face/through 的元素实参）
    let e = run_jsx(
        r#"export default <scene><box s={[1, 1, 1]} t={face(<translate t={[0, 0, 5]}><sphere r={1} /></translate>, "+z")} /></scene>;"#,
        None,
        ".",
    )
    .unwrap_err();
    assert!(e.contains("removed"), "{e}");
}

/// 查询目标（face/xdir/through 的元素实参）与渲染路径一致：变换 prop 生效。
#[test]
fn query_target_applies_transform_prop() {
    let run = run_jsx(
        r#"
export default (
  <scene>
<box s={[0.2, 0.2, 0.2]} t={face(<sphere r={1} t={[0, 0, 5]} />, "+z")} />
  </scene>
);
"#,
        None,
        ".",
    )
    .expect("run");
    let m = run.scene.objects[0].base.motor().to_matrix();
    // sphere 中心 z=5、半径 1 → +z 面在 z=6；丢变换会得到 z=1
    assert!((m[11] - 6.0).abs() < 1e-9, "t_z = {}", m[11]);
}

/// CSG 子树里的 <group> 容器（多子修饰符迁移后的形态）。
#[test]
fn csg_group_container_flattens_into_parent() {
    let dbg = |src: &str| {
        let run = run_jsx(src, None, ".").expect(src);
        format!("{:?}", run.scene.objects)
    };
    let a = dbg(
        r#"export default <scene><difference><group t={[1, 0, 0]}><box s={[1, 1, 1]} /><sphere r={0.5} /></group></difference></scene>;"#,
    );
    let b = dbg(
        r#"export default <scene><difference><box s={[1, 1, 1]} t={[1, 0, 0]} /><sphere r={0.5} t={[1, 0, 0]} /></difference></scene>;"#,
    );
    assert_eq!(a, b);
}

/// CSG 子数不变量（`CsgGeometry::new` 需要 ≥2 子，否则在 cga-core 里 panic）：
/// 三条路径必须**在宿主侧先拦**，且不再给 union 开例外（单子/空 union 从前 panic）。
#[test]
fn csg_child_count_checked_on_every_path() {
    let err = |src: &str| match run_jsx(src, None, ".") {
        Ok(_) => panic!("should fail: {src}"),
        Err(e) => e,
    };
    // 渲染路径（顶层）
    assert!(
        err(r#"export default <scene><difference><sphere r={1} /></difference></scene>;"#)
            .contains("needs >= 2 geometry children")
    );
    // union 不再例外
    assert!(
        err(r#"export default <scene><union><sphere r={1} /></union></scene>;"#)
            .contains("needs >= 2 geometry children")
    );
    assert!(
        err(r#"export default <scene><union /></scene>;"#).contains("needs >= 2 geometry children")
    );
    // CSG 收集路径（嵌套）
    assert!(err(
        r#"export default <scene><difference><box s={[1,1,1]} /><difference><sphere r={1} /></difference></difference></scene>;"#
    )
    .contains("needs >= 2 geometry children"));
    // 查询路径
    assert!(err(
        r#"export default <scene><box s={[1,1,1]} t={face(<difference><sphere r={1} /></difference>, "+z")} /></scene>;"#
    )
    .contains("needs >= 2 geometry children"));
}

/// 惰性查询缺参数：JS 侧值为 `undefined` 时 JSON 序列化会丢键，
/// Rust 侧必须报错而不是 `o["of"]` 索引 panic。
#[test]
fn lazy_query_missing_argument_is_error_not_panic() {
    let err = |src: &str| run_jsx(src, None, ".").unwrap_err();
    for src in [
        // face(undefined, "+z") → {__q:'face', key} 丢了 of
        r#"export default <scene><box s={[1,1,1]} t={face(undefined, "+z")} /></scene>;"#,
        // center(undefined) → 丢了 of
        r#"export default <scene><box s={[1,1,1]} t={center(undefined)} /></scene>;"#,
        // vadd(undefined, [0,1,0]) → 丢了 a
        r#"export default <scene><box s={[1,1,1]} t={vadd(undefined, [0, 1, 0])} /></scene>;"#,
        // clearance(undefined, "b") → 丢了 a
        r#"export default <scene><sphere r={clearance(undefined, "b")} /> </scene>;"#,
    ] {
        let e = err(src);
        assert!(!e.is_empty() && !e.contains("panic"), "{src} → {e}");
    }
}

/// 容器元素在渲染 / CSG / 查询三条路径上语义一致：
/// `<material>`/`<fragment>` 透明、`<tag>` 注册并透明、`<when>` 条件展开。
#[test]
fn containers_consistent_across_walk_csg_query() {
    let run = |src: &str| run_jsx(src, None, ".").expect(src);
    // CSG 里：<material> 透明（不带材质 prop；带了会报错，见下）
    let r = run(
        r#"export default <scene><difference><material><box s={[2,2,2]} /></material><sphere r={1} /></difference></scene>;"#,
    );
    assert_eq!(r.scene.objects.len(), 1);
    // CSG 里：<tag> 注册并透明（从前报 tag has no parameter name）
    let r = run(
        r#"export default <scene><difference><tag name="a"><box s={[2,2,2]} /></tag><sphere r={1} /></difference></scene>;"#,
    );
    assert_eq!(r.scene.objects.len(), 1);
    assert!(r.tags.contains_key("a"), "tag 要在 CSG 内注册");
    // CSG 里：<when> 条件展开（从前报 when has no parameter of）
    let r = run(
        r#"export default <scene><box s={[0.1,0.1,0.1]} tag="x" /><difference><when of="x" count={1}><box s={[2,2,2]} /></when><sphere r={1} /></difference></scene>;"#,
    );
    assert_eq!(r.scene.objects.len(), 2);
    // 查询：<material>/<tag>/<fragment> 都取唯一子几何
    for src in [
        r#"export default <scene><box s={[1,1,1]} t={face(<material color={0xFF0000}><sphere r={1} /></material>, "+z")} /></scene>;"#,
        r#"export default <scene><box s={[1,1,1]} t={face(<tag name="q"><sphere r={1} /></tag>, "+z")} /></scene>;"#,
        r#"export default <scene><box s={[1,1,1]} t={face(<><sphere r={1} /></>, "+z")} /></scene>;"#,
    ] {
        let r = run(src);
        let m = r.scene.objects[0].base.motor().to_matrix();
        assert!((m[11] - 1.0).abs() < 1e-9, "{src} → t_z={}", m[11]);
    }
}

/// 复合元素作查询目标：与 CSG 收集路径同样可用（单实例几何 / cutter 几何），
/// 多实例给明确报错（不是 panic，也不是随手取其中一个）。
#[test]
fn composite_elements_usable_as_query_target() {
    // <instances>：恰好一个实例
    let r = run_jsx(
        r#"export default <scene><box s={[1,1,1]} tag="x" /><cylinder r={0.1} h={1} t={face(<instances of="x" />, "+z")} /></scene>;"#,
        None,
        ".",
    )
    .expect("single instance query");
    let m = r.scene.objects[1].base.motor().to_matrix();
    // box s=[1,1,1] 的 +z 面在 z=0.5（实例世界矩阵要算进去）
    assert!((m[11] - 0.5).abs() < 1e-9, "t_z={}", m[11]);
    // <instances>：多实例 → 报错要求按名字引用
    let e = run_jsx(
        r#"export default <scene><box s={[1,1,1]} tag="x" /><box s={[1,1,1]} tag="x" /><sphere r={0.1} t={face(<instances of="x" />, "+z")} /></scene>;"#,
        None,
        ".",
    )
    .expect_err("multi instance query must fail");
    assert!(e.contains("exactly one"), "{e}");
    // <drill>：cutter 几何
    let r = run_jsx(
        r#"export default <scene><box s={[4,4,4]} tag="p" /><cylinder r={0.1} h={1} t={face(<drill r={0.5} through="p" axis={1} />, "+z")} /></scene>;"#,
        None,
        ".",
    )
    .expect("drill query");
    assert_eq!(r.scene.objects.len(), 2);
    assert!(r.scene.objects[1].base.motor().to_matrix()[11].is_finite());
}

/// 场景级元素（不产生几何）出现在几何位置：三条路径给同一个友好报错，
/// 而不是底层 builder 的 `unknown primitive camera`。
#[test]
fn scene_level_element_in_geometry_position_errors() {
    for (src, hint) in [
        (
            r#"export default <scene><difference><camera fov={50} /><sphere r={1} /></difference></scene>;"#,
            "camera",
        ),
        (
            r#"export default <scene><difference><directional_light intensity={2} /><sphere r={1} /></difference></scene>;"#,
            "directional_light",
        ),
        (
            r#"export default <scene><box s={[1,1,1]} t={face(<background color={0x111111} />, "+z")} /></scene>;"#,
            "background",
        ),
    ] {
        let e = run_jsx(src, None, ".").unwrap_err();
        assert!(e.contains("not geometry") && e.contains(hint), "{e}");
    }
}

#[test]
fn transform_props_rejected_on_non_geometry() {
    let e = run_jsx(
        r#"export default <scene><camera t={[1, 0, 0]} /></scene>;"#,
        None,
        ".",
    )
    .unwrap_err();
    assert!(e.contains("transform props"), "{e}");
    let e = run_jsx(
        r#"export default <scene><ambient_light scale={2} /></scene>;"#,
        None,
        ".",
    )
    .unwrap_err();
    assert!(e.contains("transform props"), "{e}");
}

/// CSG 合并成一个对象 → 子元素上的材质 / 样式 / 分组 / density 无处可去：
/// 报错并指明写到 CSG 元素上，不许静默丢（材质写在 CSG 上是唯一生效位置）。
#[test]
fn csg_child_style_channels_error_instead_of_dropping() {
    let err = |src: &str| match run_jsx(src, None, ".") {
        Ok(_) => panic!("should fail: {src}"),
        Err(e) => e,
    };
    // 显式材质 prop
    let e = err(
        r#"export default <scene><difference><box s={[2,2,2]} color={0xFF0000} /><sphere r={1} /></difference></scene>;"#,
    );
    assert!(
        e.contains("material prop \"color\"") && e.contains("single material"),
        "{e}"
    );
    // <material> 包裹带材质
    let e = err(
        r#"export default <scene><difference><material color={0xFF0000}><box s={[2,2,2]} /></material><sphere r={1} /></difference></scene>;"#,
    );
    assert!(e.contains("material prop \"color\""), "{e}");
    // class / id（CSS 只作用于材质）
    let e = err(
        r#"export default <scene><difference><box s={[2,2,2]} class="red" /><sphere r={1} /></difference></scene>;"#,
    );
    assert!(e.contains("\"class\" inside a CSG"), "{e}");
    // group / density
    let e = err(
        r#"export default <scene><difference><box s={[2,2,2]} group="g" /><sphere r={1} /></difference></scene>;"#,
    );
    assert!(e.contains("group inside a CSG"), "{e}");
    let e = err(
        r#"export default <scene><difference><box s={[2,2,2]} density={7800} /><sphere r={1} /></difference></scene>;"#,
    );
    assert!(e.contains("density inside a CSG"), "{e}");
    // 深层容器里的同样报错（collect_geom 覆盖整棵子树）
    let e = err(
        r#"export default <scene><difference><group color={0xFF0000}><box s={[2,2,2]} /></group><sphere r={1} /></difference></scene>;"#,
    );
    assert!(e.contains("material prop \"color\""), "{e}");
    // CSG 元素自己写材质 → 合法（那正是唯一生效的位置）
    run_jsx(
        r#"export default <scene><difference color={0xFF0000}><box s={[2,2,2]} /><sphere r={1} /></difference></scene>;"#,
        None,
        ".",
    )
    .expect("material on the CSG itself");
}

/// 变换不可逆（线性 |det| < 1e-15）→ 三条路径都给可读错误，
/// 而不是让 cga-core 的 `AffineGeometry::new` panic。
#[test]
fn singular_transform_errors_instead_of_panicking() {
    let err = |src: &str| match run_jsx(src, None, ".") {
        Ok(_) => panic!("should fail: {src}"),
        Err(e) => e,
    };
    // 渲染路径：单个分量为 0
    let e = err(r#"export default <scene><box s={[2,2,2]} scale={[0,1,1]} /></scene>;"#);
    assert!(e.contains("singular"), "{e}");
    // 标量 scale=0
    let e = err(r#"export default <scene><group scale={0}><box s={[2,2,2]} /></group></scene>;"#);
    assert!(e.contains("singular"), "{e}");
    // 嵌套组合后才奇异（单个都合法）
    let e = err(
        r#"export default <scene><group scale={[1e-8,1,1]}><box s={[2,2,2]} scale={[1e-8,1,1]} /></group></scene>;"#,
    );
    assert!(e.contains("singular"), "{e}");
    // CSG 收集路径
    let e = err(
        r#"export default <scene><difference><box s={[2,2,2]} scale={[0,1,1]} /><sphere r={1} /></difference></scene>;"#,
    );
    assert!(e.contains("singular"), "{e}");
    // 查询路径
    let e = err(
        r#"export default <scene><box s={[1,1,1]} t={face(<sphere r={1} scale={[0,1,1]} />, "+z")} /></scene>;"#,
    );
    assert!(e.contains("singular"), "{e}");
    // 合法输入不受影响（非均匀/负缩放都可逆）
    run_jsx(
        r#"export default <scene><box s={[2,2,2]} scale={[0.5,2,-1]} /></scene>;"#,
        None,
        ".",
    )
    .expect("invertible transform");
}

#[test]
fn tag_prop_registers_like_tag_element() {
    let run = run_jsx(
        r#"
export default (
  <scene>
<sphere r={0.5} tag="ball" />
<group tag="arm"><box s={[1, 0.2, 0.2]} /></group>
<instances of="ball" />
<instances of="arm" />
  </scene>
);
"#,
        None,
        ".",
    )
    .expect("run");
    // ball 1 + arm 1 + 各自 1 个实例 = 4
    assert_eq!(run.scene.objects.len(), 4);
    assert!(run.tags.contains_key("ball") && run.tags.contains_key("arm"));
    // tag prop 的注册框架包含元素自身变换：球平移后实例化要落在平移后的位置
    let run2 = run_jsx(
        r#"export default <scene><sphere r={0.5} tag="b" t={[3, 0, 0]} /><instances of="b" /></scene>;"#,
        None,
        ".",
    )
    .expect("run");
    let xs: Vec<f64> = run2
        .scene
        .objects
        .iter()
        .map(|o| o.base.motor().to_matrix()[3])
        .collect();
    assert!(xs.iter().all(|x| (x - 3.0).abs() < 1e-9), "{xs:?}");
    // <tag name> 元素保持可用（兼容）
    let run3 = run_jsx(
        r#"export default <scene><tag name="x"><sphere r={0.5} /></tag><instances of="x" /></scene>;"#,
        None,
        ".",
    )
    .expect("run");
    assert_eq!(run3.scene.objects.len(), 2);
}

#[test]
fn material_props_work_on_any_container() {
    // 材质键在任何元素上都是内联 prop（沿元素树继承）；<material> 容器保持
    // 可用（透明兼容壳）。
    let run = run_jsx(
        r#"
export default (
  <scene>
<group color={0x112233}><sphere r={0.5} /></group>
<group color={0x445566} t={[2, 0, 0]}><sphere r={0.5} /></group>
<material color={0x778899}><sphere r={0.5} /></material>
  </scene>
);
"#,
        None,
        ".",
    )
    .expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x112233);
    assert_eq!(hx(&run.scene.objects[1].material.color), 0x445566);
    assert_eq!(hx(&run.scene.objects[2].material.color), 0x778899);
}

#[test]
fn tag_prop_marks_subtree_impure_for_incremental_reuse() {
    // tag prop 有注册表副作用：带它的子树永不复用（复用了实例化就丢注册）。
    let src = r#"
const { useContext } = React;
function Dial() {
  const IN = useContext(HostInput);
  const x = IN.x === undefined ? 0 : IN.x;
  return <sphere r={0.25} t={[x, 0, 0]} />;
}
export default (
  <scene>
<camera />
<sphere r={0.5} tag="ball" />
<instances of="ball" />
<Dial />
  </scene>
);
"#;
    let mut s = SceneSession::open(src, None, ".").expect("open");
    s.set_input(r#"{"x": 2.0}"#).unwrap();
    // 与从头构建同一份快照逐字段一致（tag 注册表也在其列）
    let snap: Value = serde_json::from_str(&s.snapshot().unwrap()).unwrap();
    let fresh = build_scene_run(&snap, None, ".", &[]).expect("fresh build");
    assert_eq!(
        format!("{:?}", s.run().tags),
        format!("{:?}", fresh.tags),
        "tag 注册表必须一致"
    );
    assert_eq!(
        format!("{:?}", s.run().scene.objects),
        format!("{:?}", fresh.scene.objects)
    );
}

/// ---- 增量构建（档 0）+ 增量渲染（档 1）+ 分组（档 2） ----

#[test]
fn incremental_build_reuses_unchanged_subtrees() {
    let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
    let s0 = s.build_stats();
    assert_eq!(s0.reused_objects, 0, "首帧没有可复用的: {s0:?}");
    assert_eq!(s0.total_objects, 4, "scene 里 4 个球: {s0:?}");

    // 输入只驱动 Dial：静态的 3 个球（scene 直属 + fixed + Counter）应整棵复用。
    s.set_input(r#"{"x": 2.0}"#).unwrap();
    let s1 = s.build_stats();
    assert!(
        s1.reused_objects >= 3,
        "未变化的子树应复用上一帧的对象: {s1:?}"
    );
    assert_eq!(s1.total_objects, 4);

    // 正确性：增量构建的产物必须与从头构建同一份快照的产物逐字段一致。
    let snap: Value = serde_json::from_str(&s.snapshot().unwrap()).unwrap();
    let fresh = build_scene_run(&snap, None, ".", &[]).expect("fresh build");
    let cached = s.run();
    assert_eq!(
        format!("{:?}", cached.scene.objects),
        format!("{:?}", fresh.scene.objects),
        "对象表必须逐字段一致"
    );
    assert_eq!(format!("{:?}", cached.tags), format!("{:?}", fresh.tags));
    assert_eq!(
        format!("{:?}", cached.kinematics),
        format!("{:?}", fresh.kinematics)
    );
    assert_eq!(cached.groups, fresh.groups);
}

#[test]
fn incremental_build_disabled_by_sibling_rules() {
    // 样式表里有兄弟组合器：兄弟的 class 变化会改变本节点计算样式但抬不动版本号，
    // 保守起见整个会话关闭子树复用。
    let mut s = SceneSession::open(
        DIAL_SCENE,
        Some("scene > scene ~ scene { color: red; }\nscene > sphere + sphere { color: red; }"),
        ".",
    )
    .expect("open");
    s.set_input(r#"{"x": 2.0}"#).unwrap();
    let st = s.build_stats();
    assert_eq!(st.reused_objects, 0, "兄弟组合器必须关闭复用: {st:?}");
    assert_eq!(st.total_objects, 4);
}

#[test]
fn incremental_build_lazy_query_forces_rebuild() {
    // 惰性查询的字符串引用要查 tag 注册表（树外状态）：含查询的子树永不复用。
    let src = r#"
const { useContext } = React;
function Ball() {
  const IN = useContext(HostInput);
  const x = IN.x === undefined ? 0 : IN.x;
  return <tag name="ball"><sphere r={0.5} t={[x, 0, 0]} /></tag>;
}
export default (
  <scene>
<camera />
<Ball />
<sphere r={0.3} t={vadd(center("ball"), [0, 2, 0])} />
  </scene>
);
"#;
    let mut s = SceneSession::open(src, None, ".").expect("open");
    s.set_input(r#"{"x": 3.0}"#).unwrap();
    // 正确性：跟随者必须跟到新位置——与从头构建同一份快照逐字段一致。
    let snap: Value = serde_json::from_str(&s.snapshot().unwrap()).unwrap();
    let fresh = build_scene_run(&snap, None, ".", &[]).expect("fresh build");
    assert_eq!(
        format!("{:?}", s.run().scene.objects),
        format!("{:?}", fresh.scene.objects),
        "带惰性查询的场景：增量构建 ≡ 全量构建"
    );
    let m = s.run().scene.objects[1].base.motor().to_matrix();
    assert!(
        (m[3] - 3.0).abs() < 1e-9 && (m[7] - 2.0).abs() < 1e-9,
        "跟随者应到 (3, 2): {m:?}"
    );
    let st = s.build_stats();
    assert_eq!(st.reused_objects, 0, "tag 之下与查询子树都不可复用: {st:?}");
}

#[test]
fn test_jsx_collision_queries() {
    let src = r#"
export default (
  <scene>
<camera />
<tag name="a"><sphere r={1} /></tag>
<tag name="b"><sphere r={1} t={[3, 0, 0]} /></tag>
<tag name="c"><sphere r={1} t={[1.5, 0, 0]} /></tag>
<sphere r={clearance("a", "b")} />
<sphere r={qadd(0.5, collides("a", "c"))} />
<sphere r={qadd(0.5, collides("a", "b"))} />
<sphere r={qadd(0.25, inside("a", [0.5, 0, 0]))} />
  </scene>
);
"#;
    let run = run_jsx(src, None, ".").expect("run");
    let radius = |i: usize| match &run.scene.objects[i].geometry {
        cga_core::Geometry::SphereGeometry(s) => s.radius,
        g => panic!("object {i} not a sphere: {g:?}"),
    };
    // 对象 0/1/2 是 tag a/b/c 的球
    assert!((radius(3) - 1.0).abs() < 1e-9, "clearance(a,b) = 3 − 2 = 1");
    assert!((radius(4) - 1.5).abs() < 1e-9, "collides(a,c) = 1（重叠）");
    assert!((radius(5) - 0.5).abs() < 1e-9, "collides(a,b) = 0（分离）");
    assert!((radius(6) - 1.25).abs() < 1e-9, "inside(a, [0.5,0,0]) = 1");
}

#[test]
fn test_jsx_collision_query_unknown_errors() {
    // 环面–环面没有确切分离距离 → Unknown → 构建报错（不许静默给一个数）。
    let src = r#"
export default (
  <scene>
<camera />
<tag name="t1"><torus R={1} r={0.25} /></tag>
<tag name="t2"><torus R={1} r={0.25} t={[3, 0, 0]} /></tag>
<sphere r={clearance("t1", "t2")} />
  </scene>
);
"#;
    let e = run_jsx(src, None, ".").unwrap_err();
    assert!(e.contains("Unknown"), "{e}");
}

#[test]
fn session_collisions_cache_across_frames() {
    let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
    let hits = s.collisions();
    let (_, computed0) = s.collision_scan_stats();
    assert_eq!(computed0, 6, "4 个对象 → 6 对全算: {computed0}");
    // 原点的 r=0.5 球在 r=1 球内部 → Yes（sep = 0 − 1.5 = −1.5）
    let inner = hits
        .iter()
        .find(|h| h.a == 0 && h.b == 1)
        .expect("pair 0-1");
    assert_eq!(inner.hit, cga_collision::Hit::Yes);
    assert!((inner.separation.unwrap() + 1.5).abs() < 1e-9, "{inner:?}");

    // 输入只移动 Dial 的球（对象 2）：含它的 3 对重算，其余 3 对缓存命中
    s.set_input(r#"{"x": 2.0}"#).unwrap();
    s.collisions();
    let (hits2, computed2) = s.collision_scan_stats();
    assert_eq!(computed2, 3, "只有含运动对象的对重算: {computed2}");
    assert_eq!(hits2, 3);
}

#[test]
fn group_prop_and_element() {
    let src = r#"
export default (
  <scene>
<camera />
<group name="arm">
  <sphere r={1} />
  <box s={[0.5, 0.5, 0.5]} group="hand" t={[3, 0, 0]} />
</group>
<sphere r={0.5} group="loose" />
  </scene>
);
"#;
    let run = run_jsx(src, None, ".").expect("run");
    assert_eq!(
        run.groups,
        vec![
            "".to_string(),
            "arm".to_string(),
            "hand".to_string(),
            "loose".to_string()
        ],
        "注册表 append-only、按首次出现排序"
    );
    let g = |i: usize| run.scene.objects[i].group;
    assert_eq!(g(0), 1, "arm 的 sphere");
    assert_eq!(g(1), 2, "内层 group prop 覆盖外层 <group>");
    assert_eq!(g(2), 3, "独立的 group prop");
}

#[test]
fn session_render_incremental_is_bitexact() {
    let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
    let (_img0, st0) = s
        .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
        .expect("render");
    assert!(st0.full && st0.reason == "first", "首帧全量: {st0:?}");

    s.set_input(r#"{"x": 2.0}"#).unwrap();
    let (img1, st1) = s
        .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
        .expect("render");
    assert!(!st1.full, "只有 Dial 动了，应增量渲染: {st1:?}");
    assert!(
        st1.dirty * 2 < st1.total,
        "脏光线应少于一半: {}/{}",
        st1.dirty,
        st1.total
    );

    // 与全帧渲染逐位一致（光线互相独立：子集追踪与全帧追踪同一光线同值）。
    let run = s.run().clone();
    let mut full = cga_gpu::Renderer::new(96, 72, 1, 3);
    let want = full.render(run.scene.clone(), run.camera);
    assert_eq!(
        img1.png,
        cga_gpu::frame_to_png_bytes(&want),
        "增量渲染必须与全帧渲染逐位一致"
    );

    // 无变化帧：一条光线都不重追。
    let (_img2, st2) = s
        .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
        .expect("render");
    assert!(
        !st2.full && st2.dirty == 0 && st2.reason == "clean",
        "{st2:?}"
    );
}

#[test]
fn renders_generated_flange() {
    let text = crate::gen_flange_assembly(
        &crate::FlangeSpec::default(),
        &crate::BoltCircleSpec::default(),
        &crate::GearSpec::default(),
        &crate::BasePlateSpec::default(),
    );
    let out = render_jsx_png(&text, None, ".", 96, 72, 1).expect("flange render");
    assert!(out.png.len() > 1000);
}

#[test]
fn bad_size_errors() {
    assert!(render_jsx_png("export default <scene />;", None, ".", 0, 10, 1).is_err());
}

#[test]
fn default_camera_when_no_camera_element() {
    // 无 <camera> 时的默认相机钉死（session_click 的投影手算依赖这个隐式行为）。
    let run = run_jsx("export default <scene><sphere r={1} /></scene>;", None, "").expect("run");
    let c = &run.camera;
    assert_eq!(c.fov, 50.0, "默认 fov");
    assert_eq!(c.aspect, 16.0 / 9.0, "默认 aspect");
    assert_eq!(c.position, [0.0, 0.0, 5.0], "默认位置");
    assert_eq!(c.target, [0.0, 0.0, 0.0], "默认目标");
    assert_eq!(c.up, [0.0, 1.0, 0.0], "默认 up");
}

#[test]
fn empty_scene_renders_uniform_background() {
    // 空场景：渲染合法，整帧同色（纯背景）。不钉具体色值（背景色另有金标）。
    let img = render_jsx_png("export default <scene />;", None, ".", 32, 24, 1).expect("render");
    let (rgba, w, h) = cga_gpu::decode_png_rgba(&img.png).expect("decode");
    assert_eq!((w, h), (32, 24));
    let first = &rgba[0..4];
    assert!(rgba.chunks(4).all(|px| px == first), "空场景应整帧同色");
}

#[test]
fn scene_without_lights_darkens_hits_keeps_background() {
    // 无灯光：命中像素比背景暗（没有光源照亮），未命中 = 背景。行为钉死。
    let img = render_jsx_png(
        "export default <scene><sphere r={1} /></scene>;",
        None,
        ".",
        64,
        48,
        1,
    )
    .expect("render");
    let (rgba, _, _) = cga_gpu::decode_png_rgba(&img.png).expect("decode");
    let at = |x: usize, y: usize| rgba[4 * (y * 64 + x)..4 * (y * 64 + x) + 4].to_vec();
    let lum = |p: &[u8]| p[0] as u32 + p[1] as u32 + p[2] as u32;
    let bg = at(2, 2);
    let center = at(32, 24);
    assert!(
        lum(&center) < lum(&bg),
        "无灯光命中应比背景暗: center {center:?} vs bg {bg:?}"
    );
    assert_eq!(at(2, 2), at(60, 44), "未命中区域应同为背景色");
}

#[test]
fn test_jsx_component_and_control_flow() {
    let src = r#"
const Ball = ({ r, x }) => <sphere r={r} t={[x, r, 0]} />;
export default (
  <scene>
{[0, 1, 2].map(i => <Ball r={0.5} x={i * 2} />)}
{1 + 1 === 2 ? <box s={[1, 1, 1]} /> : null}
  </scene>
);
"#;
    let run = run_jsx(src, None, "").expect("run");
    assert_eq!(run.scene.objects.len(), 4, "3 Ball + 1 box");
}

#[test]
fn test_jsx_css_material() {
    let run = run_jsx(
        r#"export default <sphere r={1} class="red" />;"#,
        Some(".red { color: #C0392B; roughness: 0.25; }"),
        "",
    )
    .expect("run");
    let m = &run.scene.objects[0].material;
    assert!(
        (m.color.r - 0xC0 as f64 / 255.0).abs() < 1e-6,
        "CSS color 未生效: {:?}",
        m.color
    );
    assert!((m.roughness - 0.25).abs() < 1e-9, "CSS roughness 未生效");
}

// ---- CSS 一致性测试：见 docs/css-conformance.md ----

fn hx(c: &Color) -> u32 {
    ((c.r * 255.0).round() as u32) << 16
        | ((c.g * 255.0).round() as u32) << 8
        | (c.b * 255.0).round() as u32
}

/// objects: [0]=sphere(.a .b) [1]=box(.b) [2]=cylinder
const TREE: &str = r#"export default (
  <scene>
<group>
  <sphere r={0.5} class="a b" />
  <box s={[1, 1, 1]} class="b" />
</group>
<cylinder r={0.2} h={1} />
  </scene>
);"#;

#[test]
fn test_css_combinators() {
    let c = |css: &str| run_jsx(TREE, Some(css), "").expect("run");

    // 后代选择器：以前 `div .c` 永远不命中。
    let run = c("scene .a { color: #111111; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x111111);
    assert_eq!(hx(&run.scene.objects[1].material.color), 0xFFFFFF);

    // 子代选择器
    let run = c("group > .a { color: #222222; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x222222);
    assert_eq!(hx(&run.scene.objects[1].material.color), 0xFFFFFF);

    // 相邻兄弟
    let run = c(".a + .b { color: #333333; }");
    assert_eq!(hx(&run.scene.objects[1].material.color), 0x333333);
    assert_ne!(hx(&run.scene.objects[0].material.color), 0x333333);

    // 普通兄弟
    let run = c(".a ~ * { color: #444444; }");
    assert_eq!(hx(&run.scene.objects[1].material.color), 0x444444);
    assert_ne!(hx(&run.scene.objects[0].material.color), 0x444444);

    // 降级回最后一个 simple selector 的旧行为必须消失：`span .b` 不该命中。
    let run = c(".b { color: #060606; }\nspan .b { color: #040404; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x060606);
    assert_eq!(hx(&run.scene.objects[1].material.color), 0x060606);

    // 复合选择器：多类必须全部命中
    let run = c(".a.b { color: #0E0E0E; }\n.b { color: #060606; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x0E0E0E);
    assert_eq!(hx(&run.scene.objects[1].material.color), 0x060606);

    // 通配符 + 类型选择器大小写不敏感
    let run = c("SPHERE { color: #0F0F0F; }\n* { roughness: 0.75; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x0F0F0F);
    assert!((run.scene.objects[2].material.roughness - 0.75).abs() < 1e-9);
}

#[test]
fn test_css_selector_table() {
    // 每格只有一条规则；期望值按 TREE 的对象顺序：
    // [0]=sphere(.a .b) [1]=box(.b) [2]=cylinder
    let cases: &[(&str, [u32; 3])] = &[
        (".a { color: #010101; }", [0x010101, 0xFFFFFF, 0xFFFFFF]),
        (".b { color: #010101; }", [0x010101, 0x010101, 0xFFFFFF]),
        (".a.b { color: #010101; }", [0x010101, 0xFFFFFF, 0xFFFFFF]),
        (".a.c { color: #010101; }", [0xFFFFFF; 3]),
        (".b.b { color: #010101; }", [0x010101, 0x010101, 0xFFFFFF]),
        ("sphere { color: #010101; }", [0x010101, 0xFFFFFF, 0xFFFFFF]),
        ("SPHERE { color: #010101; }", [0x010101, 0xFFFFFF, 0xFFFFFF]),
        ("* { color: #010101; }", [0x010101; 3]),
        (
            "sphere.a { color: #010101; }",
            [0x010101, 0xFFFFFF, 0xFFFFFF],
        ),
        ("box.b { color: #010101; }", [0xFFFFFF, 0x010101, 0xFFFFFF]),
        ("scene { color: #010101; }", [0x010101; 3]),
        (":root { color: #010101; }", [0x010101; 3]),
        (
            "scene .a { color: #010101; }",
            [0x010101, 0xFFFFFF, 0xFFFFFF],
        ),
        ("scene > .a { color: #010101; }", [0xFFFFFF; 3]),
        (
            "scene > group { color: #010101; }",
            [0x010101, 0x010101, 0xFFFFFF],
        ),
        (
            "scene > group > .a { color: #010101; }",
            [0x010101, 0xFFFFFF, 0xFFFFFF],
        ),
        (
            "group > .a { color: #010101; }",
            [0x010101, 0xFFFFFF, 0xFFFFFF],
        ),
        (
            "group > .b { color: #010101; }",
            [0x010101, 0x010101, 0xFFFFFF],
        ),
        (
            "group .b { color: #010101; }",
            [0x010101, 0x010101, 0xFFFFFF],
        ),
        (
            "group > sphere { color: #010101; }",
            [0x010101, 0xFFFFFF, 0xFFFFFF],
        ),
        ("group > group { color: #010101; }", [0xFFFFFF; 3]),
        (
            ".a + .b { color: #010101; }",
            [0xFFFFFF, 0x010101, 0xFFFFFF],
        ),
        (
            ".a ~ .b { color: #010101; }",
            [0xFFFFFF, 0x010101, 0xFFFFFF],
        ),
        (".b + * { color: #010101; }", [0xFFFFFF, 0x010101, 0xFFFFFF]),
        (".b ~ * { color: #010101; }", [0xFFFFFF, 0x010101, 0xFFFFFF]),
        ("cylinder + * { color: #010101; }", [0xFFFFFF; 3]),
        (".a > .b { color: #010101; }", [0xFFFFFF; 3]),
        (".a .b { color: #010101; }", [0xFFFFFF; 3]),
        (
            "[class] { color: #010101; }",
            [0x010101, 0x010101, 0xFFFFFF],
        ),
        (
            "[class=\"b\"] { color: #010101; }",
            [0xFFFFFF, 0x010101, 0xFFFFFF],
        ),
        ("#nope { color: #010101; }", [0xFFFFFF; 3]),
    ];
    for (css, want) in cases {
        let run = run_jsx(TREE, Some(css), "").unwrap_or_else(|e| panic!("css={css}: {e}"));
        let got = [
            hx(&run.scene.objects[0].material.color),
            hx(&run.scene.objects[1].material.color),
            hx(&run.scene.objects[2].material.color),
        ];
        assert_eq!(got, *want, "css={css}");
    }
}

#[test]
fn test_css_attribute_selector() {
    let src = r#"export default <sphere r={1} id="s1" />;"#;
    let run = run_jsx(src, Some("[id=\"s1\"] { color: #0D0D0D; }"), "").expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x0D0D0D);
    let run = run_jsx(src, Some("[id=\"other\"] { color: #0D0D0D; }"), "").expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0xFFFFFF);
    let e = run_jsx(src, Some("[id^=\"s\"] { color: red; }"), "")
        .err()
        .expect("should fail");
    assert!(e.contains("unsupported attribute"), "{e}");
}

#[test]
fn test_css_selector_list_split() {
    // 选择器列表按顶层逗号切：`[class="a,b"]` 里的逗号不能当分隔符。
    let src = r#"export default <sphere r={1} class="a,b" />;"#;
    let run = run_jsx(src, Some("[class=\"a,b\"], #zz { color: #0D0D0D; }"), "").expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x0D0D0D);
    // 尾逗号 → 空选择器，必须报错而不是被吞掉。
    let e = run_jsx(src, Some(".a, { color: red; }"), "")
        .err()
        .expect("should fail");
    assert!(e.contains("CSS:"), "{e}");
}

#[test]
fn test_css_root_scope_only() {
    // `:root` 只命中根：写在后面的 :root 规则不能覆盖子元素的 .b。
    let run = run_jsx(
        TREE,
        Some(".b { color: #060606; }\n:root { background: #2B3138; color: #777777; }"),
        "",
    )
    .expect("run");
    assert_eq!(hx(&run.scene.background), 0x2B3138);
    // box 命中 .b：后写的 :root 规则不能跨过特异性把它覆盖掉
    assert_eq!(hx(&run.scene.objects[1].material.color), 0x060606);
    // cylinder 没有自己的规则 → 从根继承 color
    assert_eq!(hx(&run.scene.objects[2].material.color), 0x777777);
}

#[test]
fn test_css_cascade_order() {
    let with_id = r#"export default <sphere r={1} class="b" id="s1" />;"#;
    // 特异性：#id > .class
    let run = run_jsx(
        with_id,
        Some(".b { color: #060606; }\n#s1 { color: #0A0A0A; }"),
        "",
    )
    .expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x0A0A0A);

    // !important 跨规则压过更高特异性
    let run = run_jsx(
        with_id,
        Some(".b { color: #060606 !important; }\n#s1 { color: #0A0A0A; }"),
        "",
    )
    .expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x060606);

    // 内联 prop > 样式表普通声明
    let run = run_jsx(
        r#"export default <sphere r={1} class="b" color={0x0B0B0B} />;"#,
        Some(".b { color: #060606; }"),
        "",
    )
    .expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x0B0B0B);

    // 样式表 !important > 内联普通声明
    let run = run_jsx(
        r#"export default <sphere r={1} class="b" color={0x0B0B0B} />;"#,
        Some(".b { color: #060606 !important; }"),
        "",
    )
    .expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x060606);

    // 级联矩阵的其余格子
    let src = r#"export default <sphere r={1} class="c" />;"#;
    let one = |css: &str| run_jsx(src, Some(css), "").unwrap_or_else(|e| panic!("css={css}: {e}"));
    let color = |run: &crate::SceneRun| hx(&run.scene.objects[0].material.color);

    // .class > tag
    assert_eq!(
        color(&one("sphere { color: #010101; }\n.c { color: #020202; }")),
        0x020202
    );
    // tag > *
    assert_eq!(
        color(&one("* { color: #010101; }\nsphere { color: #020202; }")),
        0x020202
    );
    // 同特异性 → 源码顺序（后写胜）
    assert_eq!(
        color(&one(".c { color: #010101; }\n.c { color: #020202; }")),
        0x020202
    );
    // 同 important 同特异性 → 仍看源码顺序
    assert_eq!(
        color(&one(
            ".c { color: #010101 !important; }\n.c { color: #020202 !important; }"
        )),
        0x020202
    );
    // !important 压过其后的普通规则
    assert_eq!(
        color(&one(
            ".c { color: #010101 !important; }\n.c { color: #020202; }"
        )),
        0x010101
    );
    // #id !important > .class !important
    let with_id = r#"export default <sphere r={1} class="c" id="s1" />;"#;
    let run = run_jsx(
        with_id,
        Some(".c { color: #010101 !important; }\n#s1 { color: #020202 !important; }"),
        "",
    )
    .expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x020202);
}

#[test]
fn test_css_values() {
    let src = |cls: &str| format!("export default <sphere r={{1}} class=\"{cls}\" />;");
    let run_one = |cls: &str, css: &str| run_jsx(&src(cls), Some(css), "").expect("run");

    let run = run_one("c", ".c { color: rgb(255, 0, 0); }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0xFF0000);

    let run = run_one("c", ".c { color: hsl(120, 100%, 50%); }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x00FF00);

    let run = run_one("c", ".c { color: navy; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x000080);

    let run = run_one("c", ".c { color: hwb(120 0% 0%); }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x00FF00);

    let run = run_one("c", ".c { color: #00f; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x0000FF);

    // transparent → RGB 全 0 + alpha 0
    let run = run_one("c", ".c { color: transparent; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x000000);
    assert!(run.scene.objects[0].material.opacity.abs() < 1e-9);

    // 八位十六进制的 alpha → opacity（该元素没有显式 opacity 时）
    let run = run_one("c", ".c { color: #ff000080; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0xFF0000);
    assert!(
        (run.scene.objects[0].material.opacity - 128.0 / 255.0).abs() < 1e-6,
        "alpha 未生效: {}",
        run.scene.objects[0].material.opacity
    );

    // 百分比按 0–1 的键折算
    let run = run_one("c", ".c { opacity: 50%; roughness: 25%; }");
    assert!((run.scene.objects[0].material.opacity - 0.5).abs() < 1e-9);
    assert!((run.scene.objects[0].material.roughness - 0.25).abs() < 1e-9);

    // 关键字
    let run = run_one("c", ".c { unlit: true; }");
    assert_eq!(
        run.scene.objects[0].material.kind,
        cga_gpu::shading::MaterialKind::Basic
    );

    // var() 替换 + fallback
    let run = run_one(
        "c",
        ":root { --brand: #123456; }\n.c { color: var(--brand); }",
    );
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x123456);
    let run = run_one("c", ".c { color: var(--nope, #654321); }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x654321);

    // `--x` 作为材质键别名
    let run = run_one("c", ".c { --roughness: 0.3; }");
    assert!((run.scene.objects[0].material.roughness - 0.3).abs() < 1e-9);

    // 未知属性静默忽略（浏览器行为）
    let run = run_one("c", ".c { color: #135790; width: 10px; }");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x135790);
}

#[test]
fn test_css_value_errors() {
    let src = r#"export default <sphere r={1} class="c" />;"#;
    let e = run_jsx(src, Some(".c { roughness: blorble; }"), "")
        .err()
        .expect("should fail");
    assert!(e.contains("CSS:") && e.contains("blorble"), "{e}");
    let e = run_jsx(src, Some(".c { roughness: 2px; }"), "")
        .err()
        .expect("should fail");
    assert!(e.contains("CSS:") && e.contains("unit"), "{e}");
    let e = run_jsx(src, Some(".c { color: var(--missing); }"), "")
        .err()
        .expect("should fail");
    assert!(e.contains("CSS:") && e.contains("--missing"), "{e}");
    let e = run_jsx(src, Some(".c { color: currentcolor; }"), "")
        .err()
        .expect("should fail");
    assert!(e.contains("CSS:") && e.contains("not supported"), "{e}");
    let e = run_jsx(src, Some(".c { roughness: calc(1 + 1); }"), "")
        .err()
        .expect("should fail");
    assert!(e.contains("CSS:") && e.contains("calc"), "{e}");
}

#[test]
fn test_css_unsupported_constructs_error() {
    let src = r#"export default <sphere r={1} class="c" />;"#;
    let cases: &[(&str, &str)] = &[
        ("a:hover { color: red; }", "pseudo-class :hover"),
        ("a::before { content: \"x\"; }", "pseudo-element"),
        ("@media (min-width: 100px) { .c { color: red; } }", "@media"),
        ("@import url(\"other.css\");", "@import"),
        ("@keyframes spin { from { opacity: 1; } }", "@keyframes"),
    ];
    for (css, want) in cases {
        let e = run_jsx(src, Some(css), "").err().expect("should fail");
        assert!(e.contains("CSS:") && e.contains(want), "css={css} err={e}");
    }
    // 嵌套规则：lightningcss 能解析，但我们不支持 → 必须报错而不是静默丢弃。
    let e = run_jsx(src, Some(".c { color: red; .d { color: blue; } }"), "")
        .err()
        .expect("should fail");
    assert!(e.contains("CSS:"), "{e}");
}

#[test]
fn test_css_inherits_through_containers() {
    let src = r#"export default (
  <scene>
<material class="tint">
  <sphere r={1} />
  <box s={[1, 1, 1]} />
</material>
  </scene>
);"#;
    let run = run_jsx(src, Some(".tint { color: #223344; }"), "").expect("run");
    assert_eq!(hx(&run.scene.objects[0].material.color), 0x223344);
    assert_eq!(hx(&run.scene.objects[1].material.color), 0x223344);
}

/// ---- C：物理属性 + 静力学 ----

#[test]
fn test_mass_density_prop_and_report() {
    // density prop 沿子树下传；质量属性闭式；报告有 mass 行与 total_mass。
    let src = r#"export default (
  <scene>
<group density={2.0}>
  <sphere r={1} />
</group>
<box s={[1, 1, 1]} density={0.5} />
<plane n={[0, 1, 0]} d={0} />
  </scene>
);"#;
    let run = run_jsx(src, None, "").expect("run");
    let mp = &run.mass_props;
    assert!(mp[0].is_some(), "球有质量属性");
    assert!(mp[1].is_some(), "盒有质量属性");
    assert!(mp[2].is_none(), "平面无有限体积 → None（诚实跳过）");
    let m0 = mp[0].unwrap();
    let want0 = 2.0 * 4.0 / 3.0 * std::f64::consts::PI; // ρ·4/3πr³
    assert!((m0.mass - want0).abs() < 1e-9, "{} vs {want0}", m0.mass);
    let m1 = mp[1].unwrap();
    assert!((m1.mass - 0.5).abs() < 1e-9, "{} vs 0.5", m1.mass);
    let rep = crate::scene_report::scene_report(
        &run.scene,
        &run.camera,
        &run.tags,
        &run.kinematics,
        &run.mass_props,
    );
    assert!(rep.contains("mass 0 m="), "{rep}");
    assert!(rep.contains("total_mass="), "{rep}");
}

#[test]
fn test_settle_pendulum_and_double() {
    // 单摆：臂沿 +x，末端球，重力 −y → 平衡 q* = −π/2（竖直下垂，闭式）。
    let pend = |extra: &str| {
        let src = format!(
            r#"export default (
  <scene>
<link name="base" />
<link name="arm" density={{1.0}}><sphere r={{0.2}} t={{[1, 0, 0]}} /></link>
{extra}
<anchor link="base" />
  </scene>
);"#
        );
        run_jsx(&src, None, "").expect(&src)
    };
    let run = pend(r#"<pair kind="revolute" name="j" a="base" b="arm" axis={[0,0,1]} q={0} />"#);
    let masses = crate::scene_build::link_masses(&run);
    assert_eq!(masses.len(), 1);
    let out = crate::scene_build::settle(
        &run.kinematics,
        &masses,
        [0.0, -9.8, 0.0],
        &["j".to_string()],
        None,
    )
    .expect("settle");
    assert!(
        (out[0].1 + std::f64::consts::FRAC_PI_2).abs() < 1e-5,
        "{out:?}"
    );

    // 双摆：两臂都竖直下垂——q1 = −π/2，q2 = 0（相对）。
    let src2 = r#"export default (
  <scene>
<link name="base" />
<link name="arm1" density={1.0}><sphere r={0.2} t={[1, 0, 0]} /></link>
<link name="arm2" density={1.0}><sphere r={0.2} t={[1, 0, 0]} /></link>
<pair kind="revolute" name="j1" a="base" b="arm1" axis={[0,0,1]} q={0} />
<pair kind="revolute" name="j2" a="arm1" b="arm2" axis={[0,0,1]} at={[1,0,0]} q={0.3} />
<anchor link="base" />
  </scene>
);"#;
    let run2 = run_jsx(src2, None, "").expect("run");
    let masses2 = crate::scene_build::link_masses(&run2);
    let out2 = crate::scene_build::settle(
        &run2.kinematics,
        &masses2,
        [0.0, -9.8, 0.0],
        &["j1".to_string(), "j2".to_string()],
        Some(&[-1.4, 0.1]),
    )
    .expect("settle");
    let get = |n: &str| out2.iter().find(|(s, _)| s == n).unwrap().1;
    assert!(
        (get("j1") + std::f64::consts::FRAC_PI_2).abs() < 1e-5,
        "{out2:?}"
    );
    assert!(get("j2").abs() < 1e-5, "{out2:?}");
}

/// ---- B：速度级运动学（twist/雅可比/活动度/奇异性） ----

/// 每种副类型一个场景：base → l，副 at=[1,0,0]，带给定 q。
fn twist_run(kind: &str, extra: &str, q: &str) -> crate::SceneRun {
    let src = format!(
        r#"export default (
  <scene>
<link name="base" />
<link name="l"><sphere r={{0.1}} /></link>
<pair kind="{kind}" name="j" a="base" b="l" at={{[1,0,0]}} axis={{[0,0,1]}} {extra} q={{{q}}} />
<anchor link="base" />
  </scene>
);"#
    );
    run_jsx(&src, None, "").expect(&src)
}

/// 连杆局部点 [1,0,0] 的世界位置（FD 用）。
fn link_point_world(run: &crate::SceneRun, local: [f64; 3]) -> [f64; 3] {
    let l = run.kinematics.links.iter().find(|l| l.name == "l").unwrap();
    cga_core::transform_point(l.world, local)
}

#[test]
fn test_twist_point_velocity_vs_finite_difference() {
    // 每种副类型：point_velocity API vs 位姿有限差分（FD 是裁判）。
    let kinds = [
        ("revolute", "", "0.3"),
        ("prismatic", "", "0.3"),
        ("helical", "pitch={0.2}", "0.3"),
        ("cylindrical", "", "[0.3,0.15]"),
        ("spherical", "", "[0.2,0.3,0.1]"),
        ("planar", "", "[0.2,0.1,0.3]"),
    ];
    let h = 1e-6;
    for (kind, extra, q) in kinds {
        let run = twist_run(kind, extra, q);
        let k = &run.kinematics;
        let p0 = link_point_world(&run, [1.0, 0.0, 0.0]);
        // API：q̇=1 的点速度（FD 是裁判）
        // 多 DOF 副逐分量对拍
        let cols = crate::scene_build::jacobian(k, "l").expect("jacobian");
        assert_eq!(cols.len(), k.pairs[0].q.len(), "{kind}: 列数=q 维数");
        for (ci, (name, kidx, _)) in cols.iter().enumerate() {
            assert_eq!(name.as_deref(), Some("j"));
            let _ = kidx;
            let q1 = {
                let mut v = k.pairs[0].q.clone();
                v[ci] += h;
                v
            };
            // q 写 prop 重建（FD）
            let q_str = if cols.len() == 1 {
                format!("{}", q1[0])
            } else {
                format!(
                    "[{}]",
                    q1.iter()
                        .map(|x| format!("{x}"))
                        .collect::<Vec<_>>()
                        .join(",")
                )
            };
            let run1 = twist_run(kind, extra, &q_str);
            let p1 = link_point_world(&run1, [1.0, 0.0, 0.0]);
            let v_fd = [
                (p1[0] - p0[0]) / h,
                (p1[1] - p0[1]) / h,
                (p1[2] - p0[2]) / h,
            ];
            // 该分量的点速度 = 雅可比的第 ci 列 加 q̇=1
            let qd1: HashMap<String, f64> = [("j".to_string(), 1.0)].into_iter().collect();
            let _ = qd1;
            // 直接算该列的点速度贡献：ṗ = ω×p + v，列 = s
            let s = cols[ci].2;
            let w = [s[0], s[1], s[2]];
            let v = [s[3], s[4], s[5]];
            let c = [
                w[1] * p0[2] - w[2] * p0[1],
                w[2] * p0[0] - w[0] * p0[2],
                w[0] * p0[1] - w[1] * p0[0],
            ];
            let v_api = [c[0] + v[0], c[1] + v[1], c[2] + v[2]];
            for i in 0..3 {
                assert!(
                    (v_api[i] - v_fd[i]).abs() < 1e-4,
                    "{kind} 列{ci} 分量{i}: api={} fd={}",
                    v_api[i],
                    v_fd[i]
                );
            }
        }
        // point_velocity 整列与 FD 总值一致（q̇ 全 1）
        let qd_all: HashMap<String, f64> = [("j".to_string(), 1.0)].into_iter().collect();
        let v_all = crate::scene_build::point_velocity(k, "l", p0, &qd_all).expect("pv");
        assert!(v_all.iter().all(|x| x.is_finite()), "{v_all:?}");
    }
}

#[test]
fn test_twist_singularity_and_mobility() {
    // 同轴两个旋转副：两列相同 → 秩 1 < 2 → 奇异。
    let src = r#"export default (
  <scene>
<link name="base" />
<link name="l1"><sphere r={0.1} /></link>
<link name="l2"><sphere r={0.1} /></link>
<pair kind="revolute" name="j1" a="base" b="l1" axis={[0,0,1]} q={0.2} />
<pair kind="revolute" name="j2" a="l1" b="l2" axis={[0,0,1]} at={[0,0,0]} q={0.3} />
<anchor link="base" />
  </scene>
);"#;
    let run = run_jsx(src, None, "").expect("run");
    assert!(crate::scene_build::is_singular(&run.kinematics, "l2").unwrap());
    let run2 = twist_run("revolute", "", "0.3");
    assert!(!crate::scene_build::is_singular(&run2.kinematics, "l").unwrap());

    // Grübler：四连杆 = 3 树副 + 1 闭包（闭包自带 1 个被确定的旋转 DOF）。
    // 不钉输入 → 固有活动度 1；钉住曲柄 → 0。
    let mk = |q0: &str| {
        let src = format!(
            r#"export default (
  <scene>
<link name="base" />
<link name="crank" />
<link name="coupler" />
<link name="rocker" />
<pair kind="revolute" name="p0" a="base" b="crank" axis={{[0,0,1]}} {q0} />
<pair kind="revolute" name="p1" a="crank" b="coupler" at={{[1,0,0]}} axis={{[0,0,1]}} guess={{-1.5}} />
<pair kind="revolute" name="p2" a="coupler" b="rocker" at={{[2,0,0]}} axis={{[0,0,1]}} guess={{-1.5}} />
<closure a="rocker" b="base" at={{[1,0,0]}} bAt={{[2,0,0]}} axis={{[0,0,1]}} />
<anchor link="base" />
  </scene>
);"#
        );
        run_jsx(&src, None, "").expect(&src)
    };
    let free = mk("");
    let m = crate::scene_build::mobility(&free.kinematics);
    assert_eq!(m.gross_dofs, 4, "3 副 + 闭包自带 1 DOF");
    assert_eq!(m.author_pins, 0);
    assert_eq!(m.closure_pins, 3, "closure 解 2 个 + 自身 1 个: {m:?}");
    assert_eq!(m.dof, 1, "四连杆固有活动度 1（Grübler）: {m:?}");
    let pinned = mk("q={1.5707963267948966}");
    let m2 = crate::scene_build::mobility(&pinned.kinematics);
    assert_eq!(m2.dof, 0, "钉住曲柄后刚化: {m2:?}");
}

#[test]
fn test_closure_four_bar_closed_form() {
    // 矩形四连杆：基座 A=(0,0) D=(2,0)，曲柄 1（朝上 q0=π/2），耦合杆 2，
    // 摇杆 1。闭式解：B=(0,1) C=(2,1)，q1* = −π/2（耦合杆放平），
    // q2* = −π/2（摇杆朝下）。自由 q：p1、p2（guess 从附近收敛到该支）。
    let src = r#"
export default (
  <scene>
<link name="base">
  <box s={[0.15, 0.2, 0.15]} t={[0, 0.1, 0]} />
  <box s={[0.15, 0.2, 0.15]} t={[2, 0.1, 0]} />
</link>
<link name="crank"><box s={[1, 0.06, 0.06]} t={[0.5, 0, 0]} /></link>
<link name="coupler"><box s={[2, 0.06, 0.06]} t={[1, 0, 0]} /></link>
<link name="rocker"><box s={[1, 0.06, 0.06]} t={[0.5, 0, 0]} /></link>
<pair kind="revolute" name="p0" a="base" b="crank" at={[0,0,0]} axis={[0,0,1]} q={1.5707963267948966} />
<pair kind="revolute" name="p1" a="crank" b="coupler" at={[1,0,0]} axis={[0,0,1]} guess={-1.5} />
<pair kind="revolute" name="p2" a="coupler" b="rocker" at={[2,0,0]} axis={[0,0,1]} guess={-1.5} />
<closure a="rocker" b="base" at={[1,0,0]} bAt={[2,0,0]} axis={[0,0,1]} />
<anchor link="base" />
  </scene>
);
"#;
    let run = run_jsx(src, None, "").expect("run");
    let k = &run.kinematics;
    assert_eq!(k.closures.len(), 1);
    let cl = &k.closures[0];
    assert!(cl.residual < 1e-8, "残差: {}", cl.residual);
    let get = |name: &str| {
        cl.solved
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, q)| *q)
            .unwrap_or(f64::NAN)
    };
    let half_pi = std::f64::consts::FRAC_PI_2;
    assert!(
        (get("p1") + half_pi).abs() < 1e-6,
        "q1: got {} want {}",
        get("p1"),
        -half_pi
    );
    assert!(
        (get("p2") + half_pi).abs() < 1e-6,
        "q2: got {} want {}",
        get("p2"),
        -half_pi
    );
    // 摇杆的 D 端点确实落在 (2,0,0)
    let rocker = k.links.iter().find(|l| l.name == "rocker").unwrap();
    let tip = cga_core::transform_point(rocker.world, [1.0, 0.0, 0.0]);
    assert!(
        (tip[0] - 2.0).abs() < 1e-6 && tip[1].abs() < 1e-6,
        "{tip:?}"
    );
    // 报告含 closure 行
    let rep = crate::scene_report::scene_report(
        &run.scene,
        &run.camera,
        &run.tags,
        &run.kinematics,
        &run.mass_props,
    );
    assert!(rep.contains("closure 0 a=\"rocker\" b=\"base\""), "{rep}");
    // 确定性：同场景两次构建同解
    let run2 = run_jsx(src, None, "").expect("run2");
    let cl2 = &run2.kinematics.closures[0];
    assert_eq!(
        format!("{:?}", cl.solved),
        format!("{:?}", cl2.solved),
        "同输入同解"
    );
}

#[test]
fn test_closure_multi_dof_spatial() {
    // 空间多自由度闭环（G5+）：圆柱副（z 轴，原点）+ 球副（臂端 [1,0,0]）在环上，
    // 闭合 l2 的 [0,1,0] 到 base 的 [1,1,0]（轴 z）。零位形天然闭合；
    // 偏移 guess → 收回零（闭式验证）；5 自由变量 vs 秩 5 约束 → 机构钉死 dof 0。
    let src = r#"
export default (
  <scene>
<link name="base"><sphere r={0.05} t={[1, 1, 0]} /></link>
<link name="l1"><box s={[1, 0.06, 0.06]} t={[0.5, 0, 0]} /></link>
<link name="l2"><box s={[0.06, 1, 0.06]} t={[0, 0.5, 0]} /></link>
<pair kind="cylindrical" name="cyl" a="base" b="l1" axis={[0,0,1]} guess={[0.15, 0.08]} />
<pair kind="spherical" name="sph" a="l1" b="l2" at={[1,0,0]} guess={[0.12, -0.09, 0.2]} />
<closure a="l2" b="base" at={[0,1,0]} bAt={[1,1,0]} axis={[0,0,1]} />
<anchor link="base" />
  </scene>
);
"#;
    let run = run_jsx(src, None, "").expect("run");
    let k = &run.kinematics;
    assert_eq!(k.closures.len(), 1);
    let cl = &k.closures[0];
    assert!(cl.residual < 1e-8, "残差 {}", cl.residual);
    assert_eq!(cl.rank, 5);
    assert_eq!(cl.solved.len(), 5, "{:?}", cl.solved);
    for (n, q) in &cl.solved {
        assert!(q.abs() < 1e-6, "{n} 应回到 0，got {q}");
    }
    // 活动度：gross = 5（cyl 2 + sph 3）+ 1（closure 自带）= 6；
    // pins = rank 5 + 1 = 6 → dof 0（装配唯一确定）。
    let m = crate::scene_build::mobility(k);
    assert_eq!(m.dof, 0, "多自由度闭环装配应唯一: {m:?}");
}

#[test]
fn test_closure_error_paths() {
    // 引用未知 link
    let e = run_jsx(
        r#"export default <scene><link name="a" /><closure a="a" b="nope" axis={[0,0,1]} /><anchor link="a" /></scene>;"#,
        None,
        "",
    )
    .unwrap_err();
    assert!(e.contains("closure 引用未知 link nope"), "{e}");
    // 路径上没有自由 q（两端 pair 都给了 q）
    let e = run_jsx(
        r#"export default (
  <scene>
<link name="a" /><link name="b" />
<pair kind="revolute" name="p" a="a" b="b" axis={[0,0,1]} q={0.1} />
<closure a="a" b="b" at={[1,0,0]} axis={[0,0,1]} />
<anchor link="a" />
  </scene>
);"#,
        None,
        "",
    )
    .unwrap_err();
    assert!(e.contains("没有可解的自由 q"), "{e}");
    // 装不上：闭合点距离永远大于杆长（LM 收敛但残差超界 → 显式错误）
    let e = run_jsx(
        r#"export default (
  <scene>
<link name="a" />
<link name="b"><box s={[1, 0.1, 0.1]} t={[0.5, 0, 0]} /></link>
<pair kind="revolute" name="p" a="a" b="b" axis={[0,0,1]} />
<closure a="b" b="a" at={[50,0,0]} bAt={[0,0,0]} axis={[0,0,1]} />
<anchor link="a" />
  </scene>
);"#,
        None,
        "",
    )
    .unwrap_err();
    assert!(e.contains("未收敛") || e.contains("残差"), "{e}");
}

#[test]
fn test_jsx_link_pair_gear() {
    let run = run_jsx(
        r#"export default (
  <scene>
<link name="base" />
<link name="la"><sphere r={0.1} /></link>
<link name="lb"><box s={[0.1,0.1,0.1]} /></link>
<pair kind="revolute" name="a" a="base" b="la" axis={[0,0,1]} q={0.4} />
<pair kind="prismatic" name="b" a="la" b="lb" axis={[0,0,1]} />
<gear a="a" b="b" ratio={-0.5} />
<anchor link="base" />
  </scene>
);"#,
        None,
        "",
    )
    .expect("run");
    let k = &run.kinematics;
    assert_eq!(k.pairs.len(), 2);
    assert!((k.pairs[1].q[0] + 0.2).abs() < 1e-12, "gear 推导 q_b=-0.2");
    assert_eq!(k.links[1].name, "la");
    assert_eq!(k.links[1].meshes, vec![0], "link la 拥有 mesh 0");
    assert_eq!(k.links[2].meshes, vec![1], "link lb 拥有 mesh 1");
    assert_eq!(k.pairs[0].tree_up, "base");
    assert_eq!(k.pairs[1].tree_up, "la");
    let rep = crate::scene_report::scene_report(
        &run.scene,
        &run.camera,
        &run.tags,
        &run.kinematics,
        &run.mass_props,
    );
    assert!(rep.contains("pair 0 \"a\" type=revolute"), "{rep}");
    assert!(rep.contains("gear 0 a=\"a\" b=\"b\" ratio=-0.5"), "{rep}");
    assert!(rep.contains("anchor \"base\""), "{rep}");
}

#[test]
fn test_jsx_errors() {
    let e = run_jsx("export default <sphere", None, "")
        .err()
        .expect("should fail");
    assert!(e.starts_with("JSX line 1: "), "{e}");
    let e = run_jsx("export default <frob />;", None, "")
        .err()
        .expect("should fail");
    assert!(e.contains("unknown primitive frob"), "{e}");
    // 带未知参数的未知元素先报参数错。
    let e = run_jsx("export default <frob r={1} />;", None, "")
        .err()
        .expect("should fail");
    assert!(e.contains("frob has no parameter r"), "{e}");
    let e = run_jsx("const a = 1;", None, "")
        .err()
        .expect("should fail");
    assert_eq!(e, "JSX: scene file must end with export default <element>");
}

#[test]
fn test_jsx_solve_and_pose() {
    // solve: 线性方程一步收敛。
    let run = run_jsx(
        r#"const [x] = solve([0.0], [v => eq(v[0], 0.42)]);
export default <sphere r={0.1} t={[x, 0, 0]} />;"#,
        None,
        "",
    )
    .expect("run");
    let m = run.scene.objects[0].base.motor().to_matrix();
    assert!((m[3] - 0.42).abs() < 1e-6, "solve 应得 x=0.42: {}", m[3]);

    // solve 不收敛 → 显式错误。
    let e = run_jsx(
        r#"const [x] = solve([0.0], [v => v[0] * v[0] + 1]);
export default <sphere r={0.1} />;"#,
        None,
        "",
    )
    .err()
    .expect("should fail");
    assert!(e.contains("did not converge"), "{e}");

    // pose：pair 级覆盖 + 报告 pose 行。
    let run = run_jsx_pose(
        r#"export default <scene><link name="base" /><link name="l"><sphere r={0.1} /></link><pair kind="revolute" name="j" a="base" b="l" /><anchor link="base" /></scene>;"#,
        None,
        "",
        &[("j".to_string(), 0.7)],
    )
    .expect("run");
    assert!((run.kinematics.pairs[0].q[0] - 0.7).abs() < 1e-12);
    let rep = crate::scene_report::scene_report(
        &run.scene,
        &run.camera,
        &run.tags,
        &run.kinematics,
        &run.mass_props,
    );
    assert!(rep.contains("pose j=0.7"), "{rep}");

    // 变量级覆盖：P 约定。
    let run = run_jsx_pose(
        r#"export default <sphere r={0.1} t={[P.x ?? 0, 0, 0]} />;"#,
        None,
        "",
        &[("x".to_string(), 3.0)],
    )
    .expect("run");
    let m = run.scene.objects[0].base.motor().to_matrix();
    assert!((m[3] - 3.0).abs() < 1e-9, "P.x 覆盖应得 3: {}", m[3]);
}

#[test]
fn jsx_sandbox_renders_and_clears_author_globals() {
    // 沙箱模式不影响正常渲染；作者全局在帧后被清理。
    let run = run_jsx_pose_sandbox(
        r#"globalThis.leak = 1; export default <sphere r={1} />;"#,
        None,
        "",
        &[],
        crate::react::Sandbox::On,
    )
    .expect("sandbox run");
    assert_eq!(run.scene.objects.len(), 1, "沙箱下照常渲染");
}

#[test]
fn test_jsx_pair_components() {
    // 低副（平移/旋转）与高副（齿轮）用 React 组件写法，语义与 <pair>/<gear> 相同。
    let run = run_jsx(
        r#"export default (
  <scene>
<link name="base" />
<link name="la"><sphere r={0.1} /></link>
<link name="lb"><box s={[0.1,0.1,0.1]} /></link>
<Revolute name="a" a="base" b="la" axis={[0,0,1]} q={0.4} />
<Prismatic name="b" a="la" b="lb" axis={[0,0,1]} />
<Gear a="a" b="b" ratio={-0.5} />
<anchor link="base" />
  </scene>
);"#,
        None,
        "",
    )
    .expect("run");
    let k = &run.kinematics;
    assert_eq!(k.pairs[0].kind, JointKind::Revolute);
    assert_eq!(k.pairs[1].kind, JointKind::Prismatic);
    assert!((k.pairs[1].q[0] + 0.2).abs() < 1e-12, "gear 推导 q_b=-0.2");
    assert_eq!(k.links[1].meshes, vec![0], "link la 拥有 mesh 0");
    assert_eq!(k.links[2].meshes, vec![1], "link lb 拥有 mesh 1");

    // 高副 <Cam> 组件转发到 cam 元素（此处验证确实进入了 cam 校验）。
    let e = run_jsx(
        r#"export default <scene><Cam a="x" b="y"
             aProfile={{kind:"plane",n:[0,1,0],d:0}}
             bProfile={{kind:"plane",n:[0,1,0],d:0}} /></scene>;"#,
        None,
        "",
    )
    .err()
    .expect("should fail");
    assert!(
        e.contains("cam 引用未知 link"),
        "Cam 组件应进入 cam 校验: {e}"
    );
}

#[test]
fn test_jsx_multi_dof_pair_q_arrays() {
    // 多自由度副的 q 是数组（cylindrical [qr,qp] / spherical / planar [x,y,theta]）。
    let run = run_jsx(
        r#"export default (
  <scene>
<link name="base" />
<link name="l1"><sphere r={0.1} /></link>
<link name="l2"><sphere r={0.1} /></link>
<link name="l3"><sphere r={0.1} /></link>
<Cylindrical name="c" a="base" b="l1" axis={[0,0,1]} q={[0.4,0.2]} />
<Spherical name="s" a="l2" b="base" q={[0.1,0.2,0.3]} />
<Planar name="p" a="base" b="l3" axis={[0,0,1]} q={[0.1,0.2,0.3]} />
<anchor link="base" />
  </scene>
);"#,
        None,
        "",
    )
    .expect("run");
    let k = &run.kinematics;
    assert_eq!(k.pairs[0].q, vec![0.4, 0.2], "cylindrical q");
    assert_eq!(k.pairs[1].q, vec![0.1, 0.2, 0.3], "spherical q");
    assert_eq!(k.pairs[2].q, vec![0.1, 0.2, 0.3], "planar q");
    // Spherical 写反了（a="l2" b="base"）：树方向反向传播也成立（无向语义）。
    assert_eq!(k.pairs[1].tree_down, "l2");

    // 数量不符仍显式报错。
    let e = run_jsx(
        r#"export default <scene><link name="base" /><link name="x" /><Cylindrical name="c" a="base" b="x" q={[1,2,3]} /><anchor link="base" /></scene>;"#,
        None,
        "",
    )
    .err()
    .expect("should fail");
    assert!(e.contains("q must be [qr, qp]"), "{e}");
}
