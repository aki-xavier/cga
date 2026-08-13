//! CGS Editor — OpenSCAD 风格的 .cgs 实时预览编辑器。
//!
//! 左侧代码编辑 (gpui-component Editor), 右侧实时预览: 编辑防抖 300ms 后
//! POST 到常驻 Python 渲染服务 (cga.scene_lang.render_server), 返回 PNG
//! 显示; 解析/渲染错误显示在状态栏。服务未运行时自动拉起
//! (uv run python -m cga.scene_lang.render_server, cwd = 仓库根)。

use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use gpui::*;
use gpui_component::input::{Editor, EditorState, InputEvent, TabSize};
use gpui_component::{button::*, *};

const SERVER: &str = "http://127.0.0.1:8123";
const DEBOUNCE: Duration = Duration::from_millis(300);
const PREVIEW_W: u32 = 720;
const PREVIEW_H: u32 = 540;

struct CgsEditor {
    editor: Entity<EditorState>,
    preview: Option<Arc<Image>>,
    status: SharedString,
    status_ok: bool,
    generation: u64,
    file: PathBuf,
}

/// CGS 文本 → 渲染服务 → PNG 字节 (错误 → Err(服务端消息))。
fn render_via_server(text: &str) -> Result<Vec<u8>, String> {
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .http_status_as_error(false)
        .timeout_global(Some(Duration::from_secs(30)))
        .build()
        .into();
    let url = format!("{SERVER}/render?w={PREVIEW_W}&h={PREVIEW_H}&aa=1");
    let mut resp = agent
        .post(&url)
        .content_type("text/plain; charset=utf-8")
        .send(text.as_bytes())
        .map_err(|e| format!("渲染服务不可达: {e}"))?;
    let body = resp
        .body_mut()
        .read_to_vec()
        .map_err(|e| format!("读取响应失败: {e}"))?;
    if resp.status() == 200 {
        Ok(body)
    } else {
        Err(String::from_utf8_lossy(&body).to_string())
    }
}

fn server_healthy() -> bool {
    ureq::get(format!("{SERVER}/health"))
        .call()
        .map(|mut r| r.body_mut().read_to_string().map(|s| s == "ok").unwrap_or(false))
        .unwrap_or(false)
}

/// 服务不在则拉起 (cwd = 仓库根, 即 editor/ 的上一级), 轮询等待就绪。
fn ensure_server() {
    if server_healthy() {
        return;
    }
    let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("editor/ 应有父目录")
        .to_path_buf();
    let _ = std::process::Command::new("uv")
        .args(["run", "python", "-m", "cga.scene_lang.render_server", "8123"])
        .current_dir(&repo)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn();
    let deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < deadline {
        if server_healthy() {
            return;
        }
        std::thread::sleep(Duration::from_millis(300));
    }
}

impl CgsEditor {
    fn new(initial: &str, file: PathBuf, window: &mut Window, cx: &mut Context<Self>) -> Self {
        let editor = cx.new(|cx| {
            EditorState::new("cgs", window, cx)
                .line_number(true)
                .tab_size(TabSize {
                    tab_size: 2,
                    hard_tabs: false,
                })
                .default_value(initial)
        });
        cx.subscribe(&editor, |this, _state, event: &InputEvent, cx| {
            if matches!(event, InputEvent::Change) {
                this.schedule_render(cx);
            }
        })
        .detach();
        let this = Self {
            editor,
            preview: None,
            status: "渲染服务启动中…".into(),
            status_ok: false,
            generation: 0,
            file,
        };
        // 启动: 确保服务在线 → 首帧渲染
        cx.spawn(async move |this, cx| {
            let ok = smol::unblock(|| {
                ensure_server();
                server_healthy()
            })
            .await;
            this.update(cx, |this, cx| {
                this.status = if ok { "就绪" } else { "渲染服务启动失败" }.into();
                this.status_ok = ok;
                this.schedule_render(cx);
            })
            .ok();
        })
        .detach();
        this
    }

    fn schedule_render(&mut self, cx: &mut Context<Self>) {
        self.generation += 1;
        let gen = self.generation;
        let text = self.editor.read(cx).value().to_string();
        cx.spawn(async move |this, cx| {
            smol::Timer::after(DEBOUNCE).await;
            let t0 = Instant::now();
            let result = smol::unblock(move || render_via_server(&text)).await;
            let elapsed = t0.elapsed();
            this.update(cx, |this, cx| {
                if this.generation != gen {
                    return; // 期间又有新编辑, 丢弃本次结果
                }
                match result {
                    Ok(png) => {
                        this.preview = Some(Arc::new(Image::from_bytes(ImageFormat::Png, png)));
                        this.status = format!("已渲染 ({:.0}ms)", elapsed.as_millis() as f64)
                            .into();
                        this.status_ok = true;
                    }
                    Err(msg) => {
                        this.status = msg.into();
                        this.status_ok = false;
                    }
                }
                cx.notify();
            })
            .ok();
        })
        .detach();
    }

    fn save(&mut self, cx: &mut Context<Self>) {
        let text = self.editor.read(cx).value().to_string();
        match std::fs::write(&self.file, text) {
            Ok(()) => {
                self.status = format!("已保存 {}", self.file.display()).into();
                self.status_ok = true;
            }
            Err(e) => {
                self.status = format!("保存失败: {e}").into();
                self.status_ok = false;
            }
        }
        cx.notify();
    }
}

impl Render for CgsEditor {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let status_color = if self.status_ok {
            rgb(0x2e7d32)
        } else {
            rgb(0xc62828)
        };
        div()
            .h_flex()
            .size_full()
            .bg(rgb(0x1e1e28))
            .child(
                div()
                    .v_flex()
                    .flex_1()
                    .h_full()
                    .child(
                        div()
                            .h_flex()
                            .items_center()
                            .gap_2()
                            .px_2()
                            .py_1()
                            .child(
                                div()
                                    .flex_1()
                                    .text_color(rgb(0xaaaaaa))
                                    .child(self.file.display().to_string()),
                            )
                            .child(
                                Button::new("save")
                                    .label("保存")
                                    .on_click(cx.listener(|this, _ev, _window, cx| this.save(cx))),
                            ),
                    )
                    .child(div().flex_1().child(Editor::new(&self.editor).size_full())),
            )
            .child(
                div()
                    .v_flex()
                    .w(px((PREVIEW_W + 16) as f32))
                    .h_full()
                    .p_2()
                    .gap_2()
                    .child(
                        div()
                            .w_full()
                            .h(px(PREVIEW_H as f32))
                            .bg(rgb(0x101018))
                            .items_center()
                            .justify_center()
                            .child(match &self.preview {
                                Some(image) => img(image.clone())
                                    .object_fit(ObjectFit::Contain)
                                    .size_full()
                                    .into_any_element(),
                                None => div()
                                    .text_color(rgb(0x888888))
                                    .child("预览加载中…")
                                    .into_any_element(),
                            }),
                    )
                    .child(
                        div()
                            .text_sm()
                            .text_color(status_color)
                            .child(self.status.clone()),
                    ),
            )
    }
}

fn main() {
    let app = gpui_platform::application().with_assets(gpui_component_assets::Assets);
    app.run(move |cx| {
        gpui_component::init(cx);
        cx.spawn(async move |cx| {
            cx.open_window(
                WindowOptions {
                    titlebar: Some(TitlebarOptions {
                        title: Some("CGS Editor".into()),
                        ..Default::default()
                    }),
                    ..Default::default()
                },
                |window, cx| {
                    let repo = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                        .parent()
                        .expect("editor/ 应有父目录")
                        .to_path_buf();
                    let file = repo.join("examples/orbit.cgs");
                    let initial = std::fs::read_to_string(&file).unwrap_or_else(|_| {
                        "sphere(r=1);\n".to_string()
                    });
                    let view = cx.new(|cx| CgsEditor::new(&initial, file, window, cx));
                    cx.new(|cx| Root::new(view, window, cx))
                },
            )
            .expect("打开窗口失败");
        })
        .detach();
    });
}
