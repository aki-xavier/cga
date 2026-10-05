pub struct HeadlessImage {
    pub width: i32,
    pub height: i32,
    pub png: Vec<u8>,
}

pub fn render_cgs_png(
    text: &str,
    asset_root: &str,
    w: i32,
    h: i32,
    aa: i32,
) -> Result<HeadlessImage, String> {
    if w <= 0 || h <= 0 {
        return Err(format!("headless: bad size {w}x{h}"));
    }
    let (scene, mut cam) = crate::cgs_load_result(text, asset_root)?;
    cam.aspect = f64::from(w) / f64::from(h);
    let mut r = crate::Renderer::new(w, h, aa, 3);
    let img = r.render(scene, cam);
    Ok(HeadlessImage {
        width: w,
        height: h,
        png: crate::frame_to_png_bytes(&img),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renders_orbit_to_png() {
        let text = include_str!("../../../examples/cgs/orbit.cgs");
        let out = render_cgs_png(text, "examples/cgs", 96, 72, 1).expect("headless render");
        assert_eq!(out.width, 96);
        assert_eq!(out.height, 72);

        assert!(out.png.starts_with(&[137, 80, 78, 71, 13, 10, 26, 10]));
        assert!(out.png.len() > 1000);
    }

    #[test]
    fn renders_generated_flange() {
        let text = crate::gen_flange_assembly(
            &crate::FlangeSpec::default(),
            &crate::BoltCircleSpec::default(),
            &crate::GearSpec::default(),
            &crate::BasePlateSpec::default(),
        );
        let out = render_cgs_png(&text, ".", 96, 72, 1).expect("flange render");
        assert!(out.png.len() > 1000);
    }

    #[test]
    fn bad_size_errors() {
        assert!(render_cgs_png("background(color=0x000000);", ".", 0, 10, 1).is_err());
    }
}
