// Minimal PNG writer (RGBA, 8-bit) so render output can be saved to disk
// without external dependencies.  Uses V's zlib + crc32 modules.
//
// Port note: the V original hand-rolled the PNG codec (zlib + crc32); the Rust
// port uses the `image` crate instead. Public names/signatures are unchanged.

use mlx_rs::Array;

// save_frame_png writes an (H, W, 4) float32 render frame as a PNG file.
pub fn save_frame_png(path: &str, img: &Array) {
    let sh = img.shape();
    let h = sh[0];
    let w = sh[1];
    img.eval().unwrap();
    let data = img.as_slice::<f32>().to_vec();
    save_png_rgba(path, w, h, &f32_rgba_to_u8(&data));
}

// frame_to_png_bytes encodes an (H, W, 4) float32 render frame as PNG bytes.
pub fn frame_to_png_bytes(img: &Array) -> Vec<u8> {
    let sh = img.shape();
    let h = sh[0];
    let w = sh[1];
    img.eval().unwrap();
    let data = img.as_slice::<f32>().to_vec();
    encode_png_rgba(w, h, &f32_rgba_to_u8(&data))
}

// encode_png_rgba encodes an RGBA image (row-major bytes) as PNG bytes.
pub fn encode_png_rgba(width: i32, height: i32, rgba: &[u8]) -> Vec<u8> {
    let img = image::RgbaImage::from_raw(width as u32, height as u32, rgba.to_vec())
        .expect("RGBA buffer size does not match width*height*4");
    let mut out = std::io::Cursor::new(Vec::new());
    img.write_to(&mut out, image::ImageFormat::Png)
        .expect("png encode failed");
    out.into_inner()
}

// save_png_rgba writes an RGBA image (row-major bytes) as a PNG file.
pub fn save_png_rgba(path: &str, width: i32, height: i32, rgba: &[u8]) {
    std::fs::write(path, encode_png_rgba(width, height, rgba))
        .unwrap_or_else(|_| panic!("cannot write {path}"));
}

// f32_rgba_to_u8 converts float RGBA (0..255) to byte RGBA.
pub fn f32_rgba_to_u8(data: &[f32]) -> Vec<u8> {
    let mut out = vec![0u8; data.len()];
    for (i, v) in data.iter().enumerate() {
        let v = *v;
        if v < 0.0 {
            out[i] = 0;
        } else if v > 255.0 {
            out[i] = 255;
        } else {
            out[i] = (v + 0.5) as u8;
        }
    }
    out
}

// load_png_rgba decodes an 8-bit non-interlaced PNG (greyscale / RGB / RGBA /
// greyscale+alpha) into RGBA bytes, returning (pixels, width, height).
pub fn load_png_rgba(path: &str) -> Result<(Vec<u8>, i32, i32), String> {
    let data = std::fs::read(path).map_err(|_| format!("cannot read {path}"))?;
    decode_png_rgba(&data)
}

// decode_png_rgba decodes an 8-bit non-interlaced PNG (greyscale / RGB / RGBA /
// greyscale+alpha) from raw bytes into RGBA bytes, returning (pixels, w, h).
//
// Port note: the `image` crate accepts a superset of what the V decoder
// supported (16-bit, interlaced and palette PNGs are decoded and converted to
// RGBA8 instead of being rejected).
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
