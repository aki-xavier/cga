//! cga_mcp 的端到端测试（U2）：子进程跑 stdio MCP 服务器，逐行 JSON-RPC。
//! 验收链：initialize → tools/list → scene_open → mobility → drag → render
//! （PNG 回图）→ collisions → mass → 错误路径（cga 的 Err 原文透传为 isError）。

use serde_json::{json, Value};
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

struct Mcp {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
    next_id: i64,
}

impl Mcp {
    fn spawn() -> Mcp {
        let mut child = Command::new(env!("CARGO_BIN_EXE_cga_mcp"))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .spawn()
            .expect("spawn cga_mcp");
        let stdin = child.stdin.take().unwrap();
        let stdout = BufReader::new(child.stdout.take().unwrap());
        Mcp {
            child,
            stdin,
            stdout,
            next_id: 0,
        }
    }

    fn rpc(&mut self, method: &str, params: Value) -> Value {
        self.next_id += 1;
        let req = json!({
            "jsonrpc": "2.0",
            "id": self.next_id,
            "method": method,
            "params": params,
        });
        writeln!(self.stdin, "{req}").unwrap();
        self.stdin.flush().unwrap();
        let mut line = String::new();
        let n = self.stdout.read_line(&mut line).expect("read response");
        assert!(n > 0, "服务器提前关闭 stdout（疑似 panic）");
        serde_json::from_str(&line).unwrap_or_else(|e| panic!("响应不是 JSON: {e}: {line}"))
    }

    fn tool(&mut self, name: &str, args: Value) -> Value {
        self.rpc("tools/call", json!({"name": name, "arguments": args}))["result"].clone()
    }

    fn tool_text(&mut self, name: &str, args: Value) -> (bool, String) {
        let r = self.tool(name, args);
        let is_err = r["isError"].as_bool().unwrap_or(false);
        let text = r["content"][0]["text"].as_str().unwrap_or("").to_string();
        (is_err, text)
    }
}

impl Drop for Mcp {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

/// 与 U1 会话测试同构的 2R 臂（球带 density，质量断言用）。
const ARM_JSX: &str = r#"export default (
  <scene>
    <link name="base"><box s={[4, 4, 0.1]} t={[0, 0, -0.1]} /></link>
    <link name="l1"><sphere r={0.1} t={[0.5, 0, 0]} density={1000} /></link>
    <link name="l2"><sphere r={0.12} t={[1, 0, 0]} density={1000} /></link>
    <pair kind="revolute" name="j1" a="base" b="l1" axis={[0, 0, 1]} />
    <pair kind="revolute" name="j2" a="l1" b="l2" at={[1, 0, 0]} axis={[0, 0, 1]} />
    <anchor link="base" />
  </scene>
);"#;

#[test]
fn mcp_stdio_full_loop() {
    let mut m = Mcp::spawn();

    // initialize 握手
    let r = m.rpc("initialize", json!({"protocolVersion": "2025-06-18"}));
    assert_eq!(r["result"]["protocolVersion"], "2025-06-18");
    assert_eq!(r["result"]["serverInfo"]["name"], "cga-mcp");

    // tools/list：10 个工具齐
    let r = m.rpc("tools/list", json!({}));
    let names: Vec<&str> = r["result"]["tools"]
        .as_array()
        .unwrap()
        .iter()
        .map(|t| t["name"].as_str().unwrap())
        .collect();
    for want in [
        "scene_open",
        "scene_close",
        "scene_report",
        "scene_render",
        "scene_pick",
        "scene_drag",
        "scene_drag_pose",
        "scene_collisions",
        "scene_mass",
        "scene_mobility",
    ] {
        assert!(names.contains(&want), "缺工具 {want}: {names:?}");
    }

    // 未 open 就 render → isError，错误有指导性
    let (err, text) = m.tool_text("scene_render", json!({}));
    assert!(err && text.contains("scene_open"), "{text}");

    // scene_open：摘要带活动度
    let (err, text) = m.tool_text("scene_open", json!({"jsx": ARM_JSX}));
    assert!(!err, "{text}");
    assert!(text.contains("mobility dof=2"), "{text}");
    assert!(text.contains("j1(revolute)"), "{text}");

    // mobility：Grübler dof = 2
    let (err, text) = m.tool_text("scene_mobility", json!({}));
    assert!(!err, "{text}");
    let mob: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(mob["dof"], 2);
    assert_eq!(mob["gross_dofs"], 2);

    // drag：闭式目标 FK(0.9,1.1) → 收回 q*
    let (q1, q2) = (0.9f64, 1.1f64);
    let to = [q1.cos() + (q1 + q2).cos(), q1.sin() + (q1 + q2).sin(), 0.0];
    let (err, text) = m.tool_text(
        "scene_drag",
        json!({"link": "l2", "from": [2.0, 0.0, 0.0], "to": to}),
    );
    assert!(!err, "{text}");
    let d: Value = serde_json::from_str(&text).unwrap();
    let solved = d["solved"].as_array().unwrap();
    assert!(
        (solved[0][1].as_f64().unwrap() - 0.9).abs() < 1e-6,
        "{text}"
    );
    assert!(
        (solved[1][1].as_f64().unwrap() - 1.1).abs() < 1e-6,
        "{text}"
    );
    assert!(d["residual"].as_f64().unwrap() < 1e-6, "{text}");

    // render：image content 是 PNG（base64 前缀 \x89PNG → "iVBORw0KG"）
    let r = m.tool("scene_render", json!({"w": 64, "h": 48, "aa": 1}));
    assert_eq!(r["isError"], false);
    let img = &r["content"][0];
    assert_eq!(img["type"], "image");
    assert_eq!(img["mimeType"], "image/png");
    assert!(
        img["data"].as_str().unwrap().starts_with("iVBORw0KG"),
        "image data 不是 PNG"
    );
    assert!(
        r["content"][1]["text"]
            .as_str()
            .unwrap()
            .contains("rendered 64x48"),
        "{:?}",
        r["content"][1]
    );

    // 诊断通道（U3）：channel=id 出图；未知 channel → isError
    let r = m.tool("scene_render", json!({"w": 64, "h": 48, "channel": "id"}));
    assert_eq!(r["isError"], false);
    assert_eq!(r["content"][0]["mimeType"], "image/png");
    assert!(
        r["content"][1]["text"]
            .as_str()
            .unwrap()
            .contains("channel=id"),
        "{:?}",
        r["content"][1]
    );
    let (err, text) = m.tool_text("scene_render", json!({"channel": "bogus"}));
    assert!(err && text.contains("channel"), "{text}");

    // 薄透镜景深（U6）：aperture+focal 出图；aperture>0 缺 focal → isError
    let r = m.tool(
        "scene_render",
        json!({"w": 64, "h": 48, "aa": 1, "aperture": 0.4, "focal": 5.0}),
    );
    assert_eq!(r["isError"], false);
    assert_eq!(r["content"][0]["mimeType"], "image/png");
    assert!(
        r["content"][1]["text"].as_str().unwrap().contains("dof("),
        "{:?}",
        r["content"][1]
    );
    let (err, text) = m.tool_text("scene_render", json!({"aperture": 0.4}));
    assert!(err && text.contains("focal"), "{text}");

    // collisions：球心 z=0、盒顶 z=−0.05，两球真实穿入盒体——认证 Yes +
    // 解析分离度（r=0.1 穿入 0.05、r=0.12 穿入 0.07）；两球之间 No。
    let (err, text) = m.tool_text("scene_collisions", json!({}));
    assert!(!err, "{text}");
    let c: Value = serde_json::from_str(&text).unwrap();
    let pairs = c["pairs"].as_array().unwrap();
    assert_eq!(pairs.len(), 3, "{text}");
    let find = |a: usize, b: usize| {
        pairs
            .iter()
            .find(|p| p["a"] == a && p["b"] == b)
            .unwrap()
            .clone()
    };
    assert_eq!(find(0, 1)["hit"], "Yes", "{text}");
    assert!(
        (find(0, 1)["separation"].as_f64().unwrap() + 0.05).abs() < 1e-9,
        "{text}"
    );
    assert_eq!(find(0, 2)["hit"], "Yes", "{text}");
    assert!(
        (find(0, 2)["separation"].as_f64().unwrap() + 0.07).abs() < 1e-9,
        "{text}"
    );
    assert_eq!(find(1, 2)["hit"], "No", "{text}");

    // mass：两球闭式质量（density=1000）：4/3π(0.1³+0.12³)·1000
    let (err, text) = m.tool_text("scene_mass", json!({}));
    assert!(!err, "{text}");
    let mass: Value = serde_json::from_str(&text).unwrap();
    let want = 1000.0 * 4.0 / 3.0 * std::f64::consts::PI * (0.1f64.powi(3) + 0.12f64.powi(3));
    let total = mass["total_mass"].as_f64().unwrap();
    assert!((total - want).abs() < 1e-6, "total {total} vs 闭式 {want}");

    // report：文本报告非空且带相机行
    let (err, text) = m.tool_text("scene_report", json!({}));
    assert!(!err && text.contains("camera("), "{text}");

    // scene_close 后再 render → isError
    let (err, _) = m.tool_text("scene_close", json!({}));
    assert!(!err);
    let (err, text) = m.tool_text("scene_render", json!({}));
    assert!(err && text.contains("scene_open"), "{text}");
}

#[test]
fn mcp_stdio_error_paths() {
    let mut m = Mcp::spawn();
    m.rpc("initialize", json!({}));

    // 坏 JSX（有 link 缺 anchor）→ isError，cga 错误原文透传
    let (err, text) = m.tool_text(
        "scene_open",
        json!({"jsx": "export default <scene><link name=\"x\" /></scene>;"}),
    );
    assert!(err, "{text}");
    assert!(text.contains("anchor"), "{text}");

    // 未知工具 → isError
    let (err, text) = m.tool_text("nope", json!({}));
    assert!(err && text.contains("未知工具"), "{text}");

    // 未知方法 → JSON-RPC -32601
    let r = m.rpc("tea/brew", json!({}));
    assert_eq!(r["error"]["code"], -32601);

    // drag 参数缺 to → isError（参数校验原文）
    let (err, text) = m.tool_text("scene_open", json!({"jsx": ARM_JSX}));
    assert!(!err, "{text}");
    let (err, text) = m.tool_text("scene_drag", json!({"link": "l2", "from": [0.0, 0.0, 0.0]}));
    assert!(err && text.contains("to"), "{text}");

    // 不可达拖拽 → cga 的诚实 Err 透传，且场景未变（mobility 仍 2）
    let (err, text) = m.tool_text(
        "scene_drag",
        json!({"link": "l2", "from": [2.0, 0.0, 0.0], "to": [3.0, 3.0, 0.0]}),
    );
    assert!(err, "{text}");
    assert!(text.contains("drag"), "{text}");
    let (_, text) = m.tool_text("scene_mobility", json!({}));
    assert_eq!(serde_json::from_str::<Value>(&text).unwrap()["dof"], 2);
}
