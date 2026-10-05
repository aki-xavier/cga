use super::*;

#[derive(Clone, Debug)]
pub struct Texture {
    pub pixels: Array,
    pub height: i32,
    pub width: i32,
    pub is_linear: bool,
}
impl Texture {
    pub fn from_rgba(rgba: &[Vec<Vec<f64>>]) -> Texture {
        let h = rgba.len();
        if h < 1 {
            panic!("texture must have >= 1 row");
        }
        let w = rgba[0].len();
        let mut flat = Vec::with_capacity(h * w * 4);
        for row in rgba {
            if row.len() != w || row[0].len() != 4 {
                panic!("texture rgba must be (height, width, 4)");
            }
            for px in row {
                for &c in px {
                    flat.push(c as f32);
                }
            }
        }
        Texture {
            pixels: Array::from_slice(&flat, &[h as i32, w as i32, 4]),
            height: h as i32,
            width: w as i32,
            is_linear: false,
        }
    }
}
impl Texture {
    pub fn from_u8_rgba(rgba: &[u8], w: i32, h: i32) -> Texture {
        Self::from_raw(rgba, w, h, false)
    }

    fn from_raw(rgba: &[u8], w: i32, h: i32, is_linear: bool) -> Texture {
        let mut flat = vec![0f32; rgba.len()];
        for (i, b) in rgba.iter().enumerate() {
            flat[i] = *b as f32 / 255.0;
        }
        Texture {
            pixels: Array::from_slice(&flat, &[h, w, 4]),
            height: h,
            width: w,
            is_linear,
        }
    }
}
impl Texture {
    pub fn sample(&self, uv: &Array, wrap_s: WrapMode, wrap_t: WrapMode) -> Array {
        let t = self;
        let sh = uv.shape();
        if sh.len() != 2 || sh[1] != 2 {
            panic!("uv must have shape (count, 2)");
        }
        let u = wrap_value(&col(uv, 0), wrap_s);
        let v = wrap_value(&col(uv, 1), wrap_t);
        let x = s_sub(&s_mul(&u, t.width as f64), 0.5);
        let y = s_sub(&s_mul(&s_rsub(&v, 1.0), t.height as f64), 0.5);
        let x0 = ck(ck(x.floor()).as_type::<i32>());
        let y0 = ck(ck(y.floor()).as_type::<i32>());
        let fx = ck(ck(x.subtract(ck(x0.as_type::<f32>()))).expand_dims(1));
        let fy = ck(ck(y.subtract(ck(y0.as_type::<f32>()))).expand_dims(1));
        let one = Array::from_int(1);
        let x1 = ck(x0.add(&one));
        let y1 = ck(y0.add(&one));
        let (x0r, x1) = if wrap_s == WrapMode::Repeat {
            let w = Array::from_int(t.width);
            (ck(x0.remainder(&w)), ck(x1.remainder(&w)))
        } else {
            let lo = Array::from_int(0);
            let hi = Array::from_int(t.width - 1);
            (
                ck(ops::clip(&x0, (&lo, &hi))),
                ck(ops::clip(&x1, (&lo, &hi))),
            )
        };
        let (y0r, y1) = if wrap_t == WrapMode::Repeat {
            let h = Array::from_int(t.height);
            (ck(y0.remainder(&h)), ck(y1.remainder(&h)))
        } else {
            let lo = Array::from_int(0);
            let hi = Array::from_int(t.height - 1);
            (
                ck(ops::clip(&y0, (&lo, &hi))),
                ck(ops::clip(&y1, (&lo, &hi))),
            )
        };
        let flat = ck(t.pixels.reshape(&[t.height * t.width, 4]));
        let warr = Array::from_int(t.width);
        let c00 = ck(flat.take_axis(ck(ck(y0r.multiply(&warr)).add(&x0r)), 0));
        let c10 = ck(flat.take_axis(ck(ck(y0r.multiply(&warr)).add(&x1)), 0));
        let c01 = ck(flat.take_axis(ck(ck(y1.multiply(&warr)).add(&x0r)), 0));
        let c11 = ck(flat.take_axis(ck(ck(y1.multiply(&warr)).add(&x1)), 0));
        let omfx = ck(fs(1.0).subtract(&fx));
        let omfy = ck(fs(1.0).subtract(&fy));
        let top = ck(ck(c00.multiply(&omfx)).add(ck(c10.multiply(&fx))));
        let bot = ck(ck(c01.multiply(&omfx)).add(ck(c11.multiply(&fx))));
        let res = ck(ck(top.multiply(&omfy)).add(ck(bot.multiply(&fy))));
        let rgb = ck(res.take_axis(Array::from_slice(&[0i32, 1, 2], &[3]), 1));
        let lin = if t.is_linear {
            rgb
        } else {
            srgb_to_linear_arr(&rgb)
        };
        let a = ck(ck(res.take_axis(Array::from_int(3), 1)).expand_dims(1));
        ck(ops::concatenate(&[&lin, &a], 1))
    }
}
