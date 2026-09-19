// Minimal animated GIF89a encoder (pure stdlib, no dependencies).
//
// Encodes a sequence of RGBA frames with a global 256-colour palette
// (median-cut quantisation over all frames) and the LZW image compression
// required by the GIF format.  Used by the demos to assemble their PNG frames
// into an animated .gif.

use std::cmp::Ordering;
use std::collections::HashMap;

pub const GIF_PALETTE_ENTRIES: usize = 256;

// ---- colour quantisation --------------------------------------------------

#[derive(Clone, Debug)]
struct GCColor {
    r: i32,
    g: i32,
    b: i32,
    c: i32,
}

// gif_palette builds a 256-entry global palette (RGB triples) covering every
// frame, via a 5-bit histogram + median cut.
fn gif_palette(frames: &[Vec<u8>], w: usize, h: usize) -> Vec<u8> {
    let mut hist: HashMap<u32, i32> = HashMap::new();
    for frame in frames {
        let mut p = 0;
        for _ in 0..w * h {
            let r = (frame[p] as i32) * (frame[p + 3] as i32) / 255;
            let g = (frame[p + 1] as i32) * (frame[p + 3] as i32) / 255;
            let b = (frame[p + 2] as i32) * (frame[p + 3] as i32) / 255;
            let key = ((r as u32) >> 3 << 10) | ((g as u32) >> 3 << 5) | ((b as u32) >> 3);
            *hist.entry(key).or_insert(0) += 1;
            p += 4;
        }
    }
    let mut colors: Vec<GCColor> = Vec::new();
    for (k, cnt) in hist.iter() {
        colors.push(GCColor {
            r: ((k >> 10) as i32 & 31) * 8 + 4,
            g: ((k >> 5) as i32 & 31) * 8 + 4,
            b: (*k as i32 & 31) * 8 + 4,
            c: *cnt,
        });
    }
    if colors.is_empty() {
        colors.push(GCColor {
            r: 0,
            g: 0,
            b: 0,
            c: 1,
        });
    }
    // median cut: repeatedly split the largest box until 256 boxes.
    let mut boxes: Vec<Vec<GCColor>> = Vec::new();
    boxes.push(colors);
    while boxes.len() < GIF_PALETTE_ENTRIES {
        // split the FIRST box with maximum length (> 1).
        let maxlen = boxes.iter().map(|b| b.len()).max().unwrap_or(0);
        if maxlen <= 1 {
            break;
        }
        let bi = boxes.iter().position(|b| b.len() == maxlen).unwrap();
        let box_ = &boxes[bi];
        let mut rmin = 255;
        let mut rmax = 0;
        let mut gmin = 255;
        let mut gmax = 0;
        let mut bmin = 255;
        let mut bmax = 0;
        for cc in box_.iter() {
            if cc.r < rmin {
                rmin = cc.r;
            }
            if cc.r > rmax {
                rmax = cc.r;
            }
            if cc.g < gmin {
                gmin = cc.g;
            }
            if cc.g > gmax {
                gmax = cc.g;
            }
            if cc.b < bmin {
                bmin = cc.b;
            }
            if cc.b > bmax {
                bmax = cc.b;
            }
        }
        let rr = rmax - rmin;
        let gg = gmax - gmin;
        let bb = bmax - bmin;
        let mut ch = 0;
        if gg > rr && gg >= bb {
            ch = 1;
        }
        if bb > rr && bb > gg {
            ch = 2;
        }
        let mut sorted = boxes[bi].clone();
        if ch == 0 {
            sorted.sort_by(gc_cmp_r);
        } else if ch == 1 {
            sorted.sort_by(gc_cmp_g);
        } else {
            sorted.sort_by(gc_cmp_b);
        }
        let mid = sorted.len() / 2;
        boxes[bi] = sorted[..mid].to_vec();
        boxes.push(sorted[mid..].to_vec());
    }
    let mut pal = vec![0u8; GIF_PALETTE_ENTRIES * 3];
    for i in 0..GIF_PALETTE_ENTRIES {
        if i < boxes.len() {
            let mut sr = 0;
            let mut sg = 0;
            let mut sb = 0;
            let mut sc = 0;
            for cc in boxes[i].iter() {
                sr += cc.r * cc.c;
                sg += cc.g * cc.c;
                sb += cc.b * cc.c;
                sc += cc.c;
            }
            if sc > 0 {
                pal[i * 3] = (sr / sc) as u8;
                pal[i * 3 + 1] = (sg / sc) as u8;
                pal[i * 3 + 2] = (sb / sc) as u8;
            }
        }
    }
    pal
}

fn gc_cmp_r(a: &GCColor, b: &GCColor) -> Ordering {
    if a.r < b.r {
        Ordering::Less
    } else if a.r > b.r {
        Ordering::Greater
    } else {
        Ordering::Equal
    }
}

fn gc_cmp_g(a: &GCColor, b: &GCColor) -> Ordering {
    if a.g < b.g {
        Ordering::Less
    } else if a.g > b.g {
        Ordering::Greater
    } else {
        Ordering::Equal
    }
}

fn gc_cmp_b(a: &GCColor, b: &GCColor) -> Ordering {
    if a.b < b.b {
        Ordering::Less
    } else if a.b > b.b {
        Ordering::Greater
    } else {
        Ordering::Equal
    }
}

// gif_palette_lut maps every 5-bit colour to its nearest palette index.
fn gif_palette_lut(pal: &[u8]) -> Vec<u8> {
    let mut lut = vec![0u8; 32768];
    for r5 in 0..32i32 {
        for g5 in 0..32i32 {
            for b5 in 0..32i32 {
                let r = r5 * 8 + 4;
                let g = g5 * 8 + 4;
                let b = b5 * 8 + 4;
                let mut best = 0;
                let mut bestd = 1 << 30;
                for i in 0..GIF_PALETTE_ENTRIES {
                    let dr = r - pal[i * 3] as i32;
                    let dg = g - pal[i * 3 + 1] as i32;
                    let db = b - pal[i * 3 + 2] as i32;
                    let d = dr * dr + dg * dg + db * db;
                    if d < bestd {
                        bestd = d;
                        best = i;
                    }
                }
                lut[(r5 * 1024 + g5 * 32 + b5) as usize] = best as u8;
            }
        }
    }
    lut
}

// gif_quantize maps one RGBA frame to palette indices using the LUT.
fn gif_quantize(frame: &[u8], w: usize, h: usize, lut: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; w * h];
    for (o, px) in out.iter_mut().zip(frame.as_chunks::<4>().0) {
        let r = (px[0] as i32) * (px[3] as i32) / 255;
        let g = (px[1] as i32) * (px[3] as i32) / 255;
        let b = (px[2] as i32) * (px[3] as i32) / 255;
        *o = lut[((r as u32) >> 3) as usize * 1024
            + ((g as u32) >> 3) as usize * 32
            + ((b as u32) >> 3) as usize];
    }
    out
}

// ---- LZW compression (GIF variant, LSB-first, variable width) ------------

struct GBitWriter {
    out: Vec<u8>,
    bits: u32,
    nbits: i32,
}

impl GBitWriter {
    fn write(&mut self, code: u32, width: i32) {
        self.bits |= code << self.nbits;
        self.nbits += width;
        while self.nbits >= 8 {
            self.out.push((self.bits & 0xFF) as u8);
            self.bits >>= 8;
            self.nbits -= 8;
        }
    }
}

// gif_lzw compresses a stream of palette indices using the GIF LZW scheme.
//
// `overflow <<= 1` after the final dictionary step is intentionally dead:
// it mirrors the decoder's bookkeeping even though nothing reads `overflow`
// afterwards.
#[allow(unused_assignments)]
fn gif_lzw(indices: &[u8], min_code_size: u32) -> Vec<u8> {
    let clear: u32 = 1u32 << min_code_size;
    let eoi = clear + 1;
    let mut code_size = min_code_size + 1;
    let mut next_code = clear + 1;
    let mut overflow: u32 = 1u32 << (min_code_size + 1);
    let mut table: HashMap<u32, u32> = HashMap::new();

    let mut w = GBitWriter {
        out: Vec::new(),
        bits: 0,
        nbits: 0,
    };
    w.write(clear, code_size as i32);

    if indices.is_empty() {
        w.write(eoi, code_size as i32);
        if w.nbits > 0 {
            w.out.push((w.bits & 0xFF) as u8);
        }
        return w.out;
    }

    let mut code = indices[0] as u32;
    for &xi in &indices[1..] {
        let x = xi as u32;
        let key = code << 8 | x;
        if let Some(&c) = table.get(&key) {
            code = c;
            continue;
        }
        w.write(code, code_size as i32);
        code = x;
        // assign the next dictionary code, growing the width or resetting
        // the table when full.
        next_code += 1;
        if next_code == overflow {
            code_size += 1;
            overflow <<= 1;
        }
        if next_code == 4095 {
            w.write(clear, code_size as i32);
            code_size = min_code_size + 1;
            next_code = clear + 1;
            overflow = clear << 1;
            table = HashMap::new();
            continue;
        }
        table.insert(key, next_code);
    }
    w.write(code, code_size as i32);
    // mirror the decoder's post-read dictionary step before the EOI code.
    next_code += 1;
    if next_code == overflow {
        code_size += 1;
        overflow <<= 1;
    }
    if next_code == 4095 {
        w.write(clear, code_size as i32);
        code_size = min_code_size + 1;
    }
    w.write(eoi, code_size as i32);
    if w.nbits > 0 {
        w.out.push((w.bits & 0xFF) as u8);
    }
    w.out
}

// ---- GIF assembly ---------------------------------------------------------

fn gif_u16le(v: i32) -> Vec<u8> {
    vec![(v & 0xFF) as u8, ((v >> 8) & 0xFF) as u8]
}

// gif_sub_blocks wraps compressed bytes into length-prefixed GIF sub-blocks.
fn gif_sub_blocks(data: &[u8]) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::new();
    let mut i = 0;
    while i < data.len() {
        let n = if data.len() - i > 255 {
            255
        } else {
            data.len() - i
        };
        out.push(n as u8);
        out.extend_from_slice(&data[i..i + n]);
        i += n;
    }
    out.push(0);
    out
}

// encode_gif_rgba encodes a sequence of RGBA frames (each `Vec<u8>` of length
// w*h*4) as an animated GIF89a with a global palette and `delay_cs`
// centiseconds between frames.
pub fn encode_gif_rgba(frames: &[Vec<u8>], w: usize, h: usize, delay_cs: i32) -> Vec<u8> {
    let pal = gif_palette(frames, w, h);
    let lut = gif_palette_lut(&pal);

    let mut out: Vec<u8> = Vec::new();
    out.extend_from_slice(b"GIF89a");
    out.extend_from_slice(&gif_u16le(w as i32));
    out.extend_from_slice(&gif_u16le(h as i32));
    // packed: global colour table | colour resolution 7 | size 7 (=256 entries)
    out.extend_from_slice(&[0xF7, 0x00, 0x00]);
    out.extend_from_slice(&pal);
    // NETSCAPE2.0 loop extension (loop forever)
    out.extend_from_slice(&[0x21, 0xFF, 0x0B]);
    out.extend_from_slice(b"NETSCAPE2.0");
    out.extend_from_slice(&[0x03, 0x01]);
    out.extend_from_slice(&gif_u16le(0));
    out.push(0x00);
    for frame in frames {
        let idx = gif_quantize(frame, w, h, &lut);
        // graphic control extension: disposal method 1 (do not dispose)
        out.extend_from_slice(&[0x21, 0xF9, 0x04, 0x04]);
        out.extend_from_slice(&gif_u16le(delay_cs));
        out.extend_from_slice(&[0x00, 0x00]);
        // image descriptor
        out.push(0x2C);
        out.extend_from_slice(&gif_u16le(0));
        out.extend_from_slice(&gif_u16le(0));
        out.extend_from_slice(&gif_u16le(w as i32));
        out.extend_from_slice(&gif_u16le(h as i32));
        out.push(0x00);
        // LZW minimum code size + image data sub-blocks
        out.push(8);
        out.extend_from_slice(&gif_sub_blocks(&gif_lzw(&idx, 8)));
    }
    out.push(0x3B);
    out
}

// save_gif writes an animated GIF to `path`.
pub fn save_gif(path: &str, frames: &[Vec<u8>], w: usize, h: usize, delay_cs: i32) {
    std::fs::write(path, encode_gif_rgba(frames, w, h, delay_cs))
        .unwrap_or_else(|_| panic!("cannot write {path}"));
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gif_test_frame(w: usize, h: usize, base: i32) -> Vec<u8> {
        let mut f = vec![0u8; w * h * 4];
        let mut p = 0;
        for y in 0..h {
            for x in 0..w {
                f[p] = ((base + x as i32 * 4) & 0xFF) as u8;
                f[p + 1] = ((base + y as i32 * 4) & 0xFF) as u8;
                f[p + 2] = ((x + y) as i32 & 0xFF) as u8;
                f[p + 3] = 255;
                p += 4;
            }
        }
        f
    }

    #[test]
    fn test_gif_encode_header_and_trailer() {
        let w = 16;
        let h = 12;
        let frames = vec![
            gif_test_frame(w, h, 0),
            gif_test_frame(w, h, 40),
            gif_test_frame(w, h, 120),
        ];
        let g = encode_gif_rgba(&frames, w, h, 3);
        assert!(g.len() > 6 + 7 + 256 * 3 + 19);
        // GIF89a magic
        assert!(g[0] == b'G' && g[1] == b'I' && g[2] == b'F' && g[3] == b'8');
        assert!(g[4] == b'9' && g[5] == b'a');
        // logical screen descriptor width/height (little-endian)
        assert!(g[6] as usize == w && g[7] == 0);
        assert!(g[8] as usize == h && g[9] == 0);
        // global colour table flag + 256-entry size
        assert!(g[10] == 0xF7);
        // trailer
        assert!(g[g.len() - 1] == 0x3B);
    }

    #[test]
    fn test_gif_encode_single_colour() {
        // a solid-colour image needs only a tiny palette but must still encode
        let w = 8;
        let h = 8;
        let mut f = vec![0u8; w * h * 4];
        let mut i = 0;
        while i < w * h * 4 {
            f[i] = 200;
            f[i + 1] = 30;
            f[i + 2] = 90;
            f[i + 3] = 255;
            i += 4;
        }
        let g = encode_gif_rgba(&[f], w, h, 5);
        assert!(g[0] == b'G' && g[g.len() - 1] == 0x3B);
    }

    #[test]
    fn test_gif_lzw_matches_reference() {
        // deterministic LZW output for a known index stream (the classic
        // "ababab..." pattern exercises dictionary growth).
        let idx: Vec<u8> = vec![1, 1, 1, 2, 2, 2, 1, 1, 1, 2, 2, 2, 1, 1, 1, 2, 2, 2];
        let enc = gif_lzw(&idx, 8);
        // clear(256) in 9 bits LSB-first → 0x00, then literal 1 → 0x03.
        assert!(enc.len() >= 4);
        assert!(enc[0] == 0x00 && enc[1] == 0x03);
    }
}
