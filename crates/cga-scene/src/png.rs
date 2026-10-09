//! PNG 编解码（CPU，`image` crate）。从 cga-gpu 的 image_io 抽出——它与
//! GPU 帧输出（Array → PNG，留在 cga-gpu）是不同的关注面。

pub fn encode_png_rgba(width: i32, height: i32, rgba: &[u8]) -> Vec<u8> {
    let img = image::RgbaImage::from_raw(width as u32, height as u32, rgba.to_vec())
        .expect("RGBA buffer size does not match width*height*4");
    let mut out = std::io::Cursor::new(Vec::new());
    img.write_to(&mut out, image::ImageFormat::Png)
        .expect("png encode failed");
    out.into_inner()
}

pub fn save_png_rgba(path: &str, width: i32, height: i32, rgba: &[u8]) {
    std::fs::write(path, encode_png_rgba(width, height, rgba))
        .unwrap_or_else(|_| panic!("cannot write {path}"));
}

pub fn load_png_rgba(path: &str) -> Result<(Vec<u8>, i32, i32), String> {
    let data = std::fs::read(path).map_err(|_| format!("cannot read {path}"))?;
    decode_png_rgba(&data)
}

pub fn decode_png_rgba(data: &[u8]) -> Result<(Vec<u8>, i32, i32), String> {
    if data.len() < 8 || data[0] != 0x89 || data[1] != 0x50 || data[2] != 0x4E || data[3] != 0x47 {
        return Err("not a PNG".to_string());
    }
    let img = image::load_from_memory_with_format(data, image::ImageFormat::Png)
        .map_err(|e| format!("PNG inflate failed: {e}"))?;
    let rgba = img.to_rgba8();
    let (w, h) = rgba.dimensions();
    Ok((rgba.into_raw(), w as i32, h as i32))
}
