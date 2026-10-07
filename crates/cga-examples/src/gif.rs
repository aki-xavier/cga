use image::codecs::gif::{GifEncoder, Repeat};
use image::{Frame, RgbaImage};

pub fn encode_gif_rgba(frames: &[Vec<u8>], w: usize, h: usize, delay_cs: i32) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::new();
    {
        let mut enc = GifEncoder::new(&mut out);
        enc.set_repeat(Repeat::Infinite)
            .expect("gif encoder set_repeat");
        let delay = image::Delay::from_numer_denom_ms(delay_cs.max(1) as u32 * 10, 1);
        for f in frames {
            let img = RgbaImage::from_raw(w as u32, h as u32, f.clone())
                .expect("frame buffer size mismatch");
            enc.encode_frame(Frame::from_parts(img, 0, 0, delay))
                .expect("gif encode_frame");
        }
    }
    out
}

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
        assert!(g.len() > 6 + 7);

        assert!(g[0] == b'G' && g[1] == b'I' && g[2] == b'F' && g[3] == b'8');
        assert!(g[4] == b'9' && g[5] == b'a');

        assert!(g[6] as usize == w && g[7] == 0);
        assert!(g[8] as usize == h && g[9] == 0);

        assert!(g[g.len() - 1] == 0x3B);
    }

    #[test]
    fn test_gif_encode_single_colour() {
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
}
