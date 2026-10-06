pub struct HeadlessImage {
    pub width: i32,
    pub height: i32,
    pub png: Vec<u8>,
}

#[cfg(test)]
mod tests {

    #[test]
    fn renders_orbit_to_png() {
        let text = include_str!("../../../examples/jsx/orbit.jsx");
        let css = include_str!("../../../examples/jsx/orbit.css");
        let out = crate::render_jsx_png(text, Some(css), "examples/jsx", 96, 72, 1)
            .expect("headless render");
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
        let out = crate::render_jsx_png(&text, None, ".", 96, 72, 1).expect("flange render");
        assert!(out.png.len() > 1000);
    }

    #[test]
    fn bad_size_errors() {
        assert!(crate::render_jsx_png("export default <scene />;", None, ".", 0, 10, 1).is_err());
    }
}
