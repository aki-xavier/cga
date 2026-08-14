//! CGS Editor — OpenSCAD 风格的 .cgs 实时预览编辑器。
//!
//! 多文件标签页 + CGS 语法高亮 + 拖拽调参 + 实时预览:
//! 顶部标签栏 (打开/新建/保存), 左侧代码编辑 (gpui-component Editor,
//! CGS 高亮), 右侧实时预览 (编辑防抖 300ms → 常驻渲染服务 → PNG) +
//! 可拖拽参数面板 (数值滑块直写回源文本)。

use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use gpui::*;
use gpui_base::input::InputEditorStyle;
use gpui_component::input::{Editor, EditorState, InputEvent, InputHighlighter, TabSize};
use gpui_component::slider::{Slider, SliderEvent, SliderState};
use gpui_component::{button::*, *};

mod highlight;
mod params;

use highlight::{CgsHighlighter, CgsHighlightStyles};
use params::{extract_params, format_number, replace_range, RawParam};

const SERVER: &str = "http://127.0.0.1:8123";
const DEBOUNCE: Duration = Duration::from_millis(300);
const PREVIEW_W: u32 = 720;
const PREVIEW_H: u32 = 500;

/// 一个打开的文档 (标签页)。
struct Document {
    path: Option<PathBuf>,
    title: String,
    editor: Entity<EditorState>,
}

/// 一个可拖拽参数 (提取结果 + 滑块实体)。
struct Param {
    raw: RawParam,
    slider: Entity<SliderState>,
}

struct CgsEditor {
    docs: Vec<Document>,
    active: usize,
    preview: Option<Arc<Image>>,
    status: SharedString,
    status_ok: bool,
    generation: u64,
    params: Vec<Param>,
    dragging_param: bool,
    /// 滑块写回待应用到编辑器文本 (下一帧 render 时应用)。
    pending_text: Option<String>,
}

fn title_of(path: &Path) -> String {
    path.file_name()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "未命名".to_string())
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
    fn new(initial_files: Vec<PathBuf>, window: &mut Window, cx: &mut Context<Self>) -> Self {
        let mut this = Self {
            docs: Vec::new(),
            active: 0,
            preview: None,
            status: "渲染服务启动中…".into(),
            status_ok: false,
            generation: 0,
            params: Vec::new(),
            dragging_param: false,
            pending_text: None,
        };
        for path in initial_files {
            let text = std::fs::read_to_string(&path).unwrap_or_default();
            this.add_document(Some(path), text, window, cx);
        }
        if this.docs.is_empty() {
            this.add_document(None, "sphere(r=1);\n".to_string(), window, cx);
        }
        // 启动: 确保服务在线 → 首帧渲染 + 参数提取
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
                this.refresh_params(cx);
            })
            .ok();
        })
        .detach();
        this
    }

    fn add_document(
        &mut self,
        path: Option<PathBuf>,
        initial: String,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) {
        let title = path.as_ref().map(|p| title_of(p)).unwrap_or_else(|| "未命名".to_string());
        let editor = cx.new(|cx| {
            EditorState::new("cgs", window, cx)
                .line_number(true)
                .tab_size(TabSize {
                    tab_size: 2,
                    hard_tabs: false,
                })
                .default_value(initial)
        });
        // CGS 高亮 + 主题
        let base = editor.read(cx).base_state().clone();
        base.update(cx, |state, cx| {
            state.set_highlighter_factory(
                Rc::new(|_language: &str| {
                    Some(Box::new(CgsHighlighter::new()) as Box<dyn InputHighlighter>)
                }),
                cx,
            );
            state.set_editor_style(InputEditorStyle {
                foreground: rgb(0xdfe2ea).into(),
                muted_foreground: rgb(0x8a90a0).into(),
                background: rgb(0x1e1e28).into(),
                caret: rgb(0xdfe2ea).into(),
                selection: hsla(0.55, 0.7, 0.6, 0.3),
                highlight_styles: Arc::new(CgsHighlightStyles),
                ..Default::default()
            });
        });
        // 编辑 → 防抖渲染
        cx.subscribe(&editor, |this, _state, event: &InputEvent, cx| {
            if matches!(event, InputEvent::Change) {
                this.schedule_render(cx);
            }
        })
        .detach();
        self.docs.push(Document {
            path,
            title,
            editor,
        });
    }

    fn active_editor(&self) -> &Entity<EditorState> {
        &self.docs[self.active].editor
    }

    fn schedule_render(&mut self, cx: &mut Context<Self>) {
        self.generation += 1;
        let gen = self.generation;
        let text = self.active_editor().read(cx).value().to_string();
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
                        this.preview =
                            Some(Arc::new(Image::from_bytes(ImageFormat::Png, png)));
                        this.status =
                            format!("已渲染 ({:.0}ms)", elapsed.as_millis() as f64).into();
                        this.status_ok = true;
                    }
                    Err(msg) => {
                        this.status = msg.into();
                        this.status_ok = false;
                    }
                }
                // 输入导致的文本变化 → 重建参数滑块 (拖拽期间跳过, 避免打断拖拽)
                if !this.dragging_param {
                    this.refresh_params(cx);
                }
                cx.notify();
            })
            .ok();
        })
        .detach();
    }

    fn refresh_params(&mut self, cx: &mut Context<Self>) {
        // 若存在尚未应用到编辑器的滑块写回, 以它为准 (避免释放滑块时读到旧文本)。
        let text = match &self.pending_text {
            Some(text) => text.clone(),
            None => self.active_editor().read(cx).value().to_string(),
        };
        let raws = extract_params(&text);
        let mut params = Vec::with_capacity(raws.len());
        for raw in raws {
            let label = raw.label.clone();
            let ordinal = raw.ordinal;
            let slider = cx.new(|_| {
                SliderState::new()
                    .min(raw.min)
                    .max(raw.max)
                    .step(raw.step)
                    .default_value(raw.value as f32)
            });
            cx.subscribe(&slider, move |this, _slider, event: &SliderEvent, cx| {
                let (value, is_release) = match event {
                    SliderEvent::Change(v) => (v.end(), false),
                    SliderEvent::Release(v) => (v.end(), true),
                };
                this.on_slider(label.clone(), ordinal, value, is_release, cx);
            })
            .detach();
            params.push(Param { raw, slider });
        }
        self.params = params;
        cx.notify();
    }

    fn on_slider(
        &mut self,
        label: String,
        ordinal: usize,
        value: f32,
        is_release: bool,
        cx: &mut Context<Self>,
    ) {
        if !is_release {
            self.dragging_param = true;
        }
        // 依据 (label, ordinal) 在最新文本中重新定位目标字面量
        let current = self.active_editor().read(cx).value().to_string();
        let raws = extract_params(&current);
        if let Some(raw) = raws
            .iter()
            .find(|p| p.label == label && p.ordinal == ordinal)
        {
            self.pending_text = Some(replace_range(
                &current,
                raw.range.clone(),
                &format_number(value as f64),
            ));
        }
        if is_release {
            self.dragging_param = false;
            self.refresh_params(cx);
        }
        cx.notify();
    }

    fn switch_tab(&mut self, i: usize, cx: &mut Context<Self>) {
        if i >= self.docs.len() || i == self.active {
            return;
        }
        self.pending_text = None; // 丢弃未及应用的滑块写回, 避免串到新标签
        self.active = i;
        self.schedule_render(cx);
        self.refresh_params(cx);
        cx.notify();
    }

    fn close_tab(&mut self, i: usize, cx: &mut Context<Self>) {
        if i >= self.docs.len() || self.docs.len() <= 1 {
            return;
        }
        self.pending_text = None;
        self.docs.remove(i);
        if i < self.active {
            self.active -= 1;
        } else if i == self.active && self.active >= self.docs.len() {
            self.active = self.docs.len() - 1;
        }
        self.schedule_render(cx);
        self.refresh_params(cx);
        cx.notify();
    }

    fn open_dialog(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let receiver = cx.prompt_for_paths(PathPromptOptions {
            files: true,
            directories: false,
            multiple: true,
            prompt: Some("打开 CGS 文件".into()),
        });
        cx.spawn_in(window, async move |this, cx| {
            let Ok(Ok(Some(paths))) = receiver.await else {
                return;
            };
            this.update_in(cx, |this, window, cx| {
                this.pending_text = None;
                for path in paths {
                    let text = std::fs::read_to_string(&path).unwrap_or_default();
                    this.add_document(Some(path), text, window, cx);
                    this.active = this.docs.len() - 1;
                }
                this.schedule_render(cx);
                this.refresh_params(cx);
                cx.notify();
            })
            .ok();
        })
        .detach();
    }

    fn save_active(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let text = self.active_editor().read(cx).value().to_string();
        let path = self.docs[self.active].path.clone();
        match path {
            Some(path) => {
                match std::fs::write(&path, text) {
                    Ok(()) => {
                        self.status = format!("已保存 {}", path.display()).into();
                        self.status_ok = true;
                    }
                    Err(e) => {
                        self.status = format!("保存失败: {e}").into();
                        self.status_ok = false;
                    }
                }
                cx.notify();
            }
            None => self.save_as(window, cx),
        }
    }

    fn save_as(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let receiver = cx.prompt_for_new_path(Path::new("."), Some("scene.cgs"));
        cx.spawn_in(window, async move |this, cx| {
            let Ok(Ok(Some(path))) = receiver.await else {
                return;
            };
            this.update_in(cx, |this, _window, cx| {
                let text = this.active_editor().read(cx).value().to_string();
                match std::fs::write(&path, text) {
                    Ok(()) => {
                        this.docs[this.active].path = Some(path.clone());
                        this.docs[this.active].title = title_of(&path);
                        this.status = format!("已保存 {}", path.display()).into();
                        this.status_ok = true;
                    }
                    Err(e) => {
                        this.status = format!("保存失败: {e}").into();
                        this.status_ok = false;
                    }
                }
                cx.notify();
            })
            .ok();
        })
        .detach();
    }
}

impl CgsEditor {
    fn tab_bar(&self, cx: &mut Context<Self>) -> impl IntoElement {
        div()
            .h_flex()
            .items_center()
            .gap_1()
            .px_2()
            .py_1()
            .bg(rgb(0x16161e))
            .children(
                self.docs
                    .iter()
                    .enumerate()
                    .map(|(i, doc)| self.tab_element(i, doc, cx).into_any_element()),
            )
            .child(Button::new("new").label("＋新建").on_click(cx.listener(
                |this, _ev, window, cx| {
                    this.pending_text = None;
                    this.add_document(None, "sphere(r=1);\n".to_string(), window, cx);
                    this.active = this.docs.len() - 1;
                    this.schedule_render(cx);
                    this.refresh_params(cx);
                    cx.notify();
                },
            )))
            .child(
                Button::new("open")
                    .label("打开…")
                    .on_click(cx.listener(|this, _ev, window, cx| {
                        this.open_dialog(window, cx);
                    })),
            )
            .child(div().flex_1())
            .child(Button::new("save").label("保存").on_click(cx.listener(
                |this, _ev, window, cx| this.save_active(window, cx),
            )))
    }

    fn tab_element(&self, i: usize, doc: &Document, cx: &mut Context<Self>) -> impl IntoElement {
        let active = i == self.active;
        let title = doc.title.clone();
        div()
            .id(("tab", i))
            .h_flex()
            .items_center()
            .gap_1()
            .px_2()
            .py(px(4.))
            .rounded_md()
            .bg(if active { rgb(0x2a2a3a) } else { rgb(0x1e1e28) })
            .text_color(if active { rgb(0xffffff) } else { rgb(0x9aa0ac) })
            .on_click(cx.listener(move |this, _ev, _window, cx| {
                this.switch_tab(i, cx);
            }))
            .child(title)
            .child(
                div()
                    .id(("close", i))
                    .px_1()
                    .text_color(rgb(0x888888))
                    .on_click(cx.listener(move |this, _ev, _window, cx| {
                        cx.stop_propagation();
                        this.close_tab(i, cx);
                    }))
                    .child("×"),
            )
    }

    fn preview_panel(&self) -> impl IntoElement {
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
            })
    }

    fn param_row(&self, p: &Param, cx: &App) -> impl IntoElement {
        let value = p.slider.read(cx).value().end();
        div()
            .v_flex()
            .px_2()
            .py_1()
            .gap_1()
            .child(
                div()
                    .h_flex()
                    .items_center()
                    .justify_between()
                    .child(div().text_sm().text_color(rgb(0x9aa0ac)).child(p.raw.label.clone()))
                    .child(
                        div()
                            .text_sm()
                            .text_color(rgb(0xdfe2ea))
                            .child(format!("{value:.3}")),
                    ),
            )
            .child(Slider::new(&p.slider))
    }

    fn param_panel(&self, cx: &App) -> impl IntoElement {
        let mut panel = div().v_flex().flex_1().gap_1();
        panel = panel.child(
            div()
                .text_sm()
                .text_color(rgb(0x7a8090))
                .child(format!("参数 ({} 个可拖拽)", self.params.len())),
        );
        if self.params.is_empty() {
            panel = panel.child(
                div()
                    .text_sm()
                    .text_color(rgb(0x6a7080))
                    .child("无数值参数"),
            );
        } else {
            panel = panel.child(
                div()
                    .id("params-scroll")
                    .v_flex()
                    .flex_1()
                    .overflow_y_scroll()
                    .gap_1()
                    .children(
                        self.params
                            .iter()
                            .map(|p| self.param_row(p, cx).into_any_element()),
                    ),
            );
        }
        panel
    }
}

impl Render for CgsEditor {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        // 应用滑块写回 (render 拥有 &mut Window, 是自然的 set_value 时机)
        if let Some(text) = self.pending_text.take() {
            let editor = self.active_editor().clone();
            let offset = editor.read(cx).base_state().read(cx).scroll_offset();
            editor.update(cx, |e, cx| {
                e.set_value(text, window, cx);
                e.base_state().update(cx, |s, cx| s.set_scroll_offset(offset, cx));
            });
            self.schedule_render(cx);
        }

        let status_color = if self.status_ok {
            rgb(0x2e7d32)
        } else {
            rgb(0xc62828)
        };
        let editor = self.active_editor().clone();

        div()
            .h_flex()
            .size_full()
            .bg(rgb(0x1e1e28))
            .child(
                div()
                    .v_flex()
                    .flex_1()
                    .h_full()
                    .child(self.tab_bar(cx))
                    .child(div().flex_1().child(Editor::new(&editor).size_full())),
            )
            .child(
                div()
                    .v_flex()
                    .w(px((PREVIEW_W + 16) as f32))
                    .h_full()
                    .p_2()
                    .gap_2()
                    .child(self.preview_panel())
                    .child(
                        div()
                            .text_sm()
                            .text_color(status_color)
                            .child(self.status.clone()),
                    )
                    .child(self.param_panel(cx)),
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
                    window_bounds: Some(WindowBounds::Windowed(Bounds::new(
                        point(px(80.), px(60.)),
                        size(px(1320.), px(880.)),
                    ))),
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
                    let mut files: Vec<PathBuf> = Vec::new();
                    if let Ok(rd) = std::fs::read_dir(repo.join("examples")) {
                        for entry in rd.flatten() {
                            let p = entry.path();
                            if p.extension().and_then(|s| s.to_str()) == Some("cgs") {
                                files.push(p);
                            }
                        }
                    }
                    files.sort();
                    let view = cx.new(|cx| CgsEditor::new(files, window, cx));
                    cx.new(|cx| Root::new(view, window, cx))
                },
            )
            .expect("打开窗口失败");
        })
        .detach();
    });
}
