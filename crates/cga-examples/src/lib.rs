use mlx_rs::Array;

pub fn data_f32(img: &Array) -> Vec<f32> {
    img.eval().unwrap();
    img.as_slice::<f32>().to_vec()
}

pub fn mlx_frame_gc() {
    let _ = mlx_rs::memory::clear_cache();
}

/// 读取 JSX 源文件（失败时 panic `cannot read {src}`）并跟随其中的
/// `import './x.css'` 引用；返回 `(源文本, 拼接后的 CSS, 源文件所在目录)`。
pub fn load_scene(src: &str) -> (String, Option<String>, String) {
    let text = std::fs::read_to_string(src).unwrap_or_else(|_| panic!("cannot read {src}"));
    let dir = src_dir(src);
    let css = collect_css(&text, &dir);
    (text, css, dir)
}

/// `src` 所在目录；不含 `/` 时为 `.`。
pub fn src_dir(src: &str) -> String {
    match src.rfind('/') {
        Some(i) => src[..i].to_string(),
        None => ".".to_string(),
    }
}

/// 跟随 `import './x.css'` 引用：只收 `./`、`../` 开头的相对路径，多个文件以
/// `'\n'` 分隔；一个都没收集到时返回 `None`。
pub fn collect_css(text: &str, dir: &str) -> Option<String> {
    let mut css = String::new();
    let mut rest = text;
    while let Some(i) = rest.find(".css") {
        let head = &rest[..i + 4];
        if let Some(q) = head.rfind(['\'', '"']) {
            let path = &head[q + 1..];
            if path.starts_with("./") || path.starts_with("../") {
                css.push_str(
                    &std::fs::read_to_string(format!("{dir}/{path}"))
                        .unwrap_or_else(|_| panic!("cannot read {dir}/{path}")),
                );
                css.push('\n');
            }
        }
        rest = &rest[i + 4..];
    }
    if css.is_empty() {
        None
    } else {
        Some(css)
    }
}

/// 把 `path` 最后一个 `.` 起的扩展名换成 `ext`（没有 `.` 时直接追加）。
pub fn replace_ext(path: &str, ext: &str) -> String {
    match path.rfind('.') {
        Some(i) => format!("{}.{ext}", &path[..i]),
        None => format!("{path}.{ext}"),
    }
}

pub mod gif;
pub use gif::*;
