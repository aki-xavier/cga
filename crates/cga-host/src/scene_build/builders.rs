//! Geometry/material construction, parameter defaults and validation.
//! Statement parsing lives in the JSX host (crate::jsx).
use super::*;

pub struct Builders {
    pub asset_root: String,
}
impl Builders {
    pub(crate) fn new(asset_root: &str) -> Builders {
        Builders {
            asset_root: asset_root.to_string(),
        }
    }

    pub(crate) fn local_bounds(&self, geo: &Geometry) -> Option<[[f64; 3]; 2]> {
        cga_gpu::geometry_ops::geom_bounds(&geo.identity_params())
    }

    pub(crate) fn resolve(
        &self,
        name: &str,
        pos: Vec<ArgValue>,
        kw: HashMap<String, ArgValue>,
        _line: i32,
    ) -> Result<HashMap<String, ArgValue>, String> {
        let names = sig_names(name);
        let defaults = sig_defaults(name);
        let mut merged: HashMap<String, ArgValue> = HashMap::new();
        for (k, v) in &defaults {
            merged.insert(k.clone(), v.clone());
        }
        if pos.len() > names.len() {
            return Err(format!("build: {name} too many positional args"));
        }
        for (i, pname) in names.iter().enumerate() {
            if i < pos.len() {
                merged.insert(pname.to_string(), pos[i].clone());
            }
        }
        for (k, v) in kw {
            if !names.contains(&k.as_str()) && !defaults.contains_key(&k) {
                return Err(format!("build: {name} has no parameter {k}"));
            }
            merged.insert(k, v);
        }
        for pname in &names {
            if let Some(ArgValue::Num(x)) = merged.get(*pname) {
                if x.is_nan() && *pname == "cylinder.h" {
                    merged.insert(pname.to_string(), ArgValue::Num(-1.0));
                }
            }
        }
        Ok(merged)
    }

    pub(crate) fn build_geometry(
        &mut self,
        name: &str,
        args: &HashMap<String, ArgValue>,
        line: i32,
    ) -> Result<Geometry, String> {
        validate_geometry_params(name, args, line)?;
        match name {
            "sphere" => {
                let r = val_num(
                    &args.get("r").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "sphere.r",
                )?;
                Ok(Geometry::SphereGeometry(SphereGeometry::new(r)))
            }
            "plane" => {
                let n = val_vec3(
                    &args.get("n").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "plane.n",
                )?;
                let d = val_num(
                    &args.get("d").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "plane.d",
                )?;
                Ok(Geometry::PlaneGeometry(PlaneGeometry::new(n, d)))
            }
            "cylinder" => {
                let h = val_num(
                    &args.get("h").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "cylinder.h",
                )?;
                let r = val_num(
                    &args.get("r").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "cylinder.r",
                )?;
                Ok(Geometry::CylinderGeometry(CylinderGeometry::new(
                    r,
                    if h < 0.0 { -1.0 } else { h },
                )))
            }
            "box" => {
                let s = val_vec3(
                    &args.get("s").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "box.s",
                )?;
                Ok(Geometry::BoxGeometry(BoxGeometry::new(s[0], s[1], s[2])))
            }
            "circle" => {
                let r = val_num(
                    &args.get("r").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "circle.r",
                )?;
                Ok(Geometry::CircleGeometry(CircleGeometry::new(r)))
            }
            "cone" => {
                let r = val_num(
                    &args.get("r").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "cone.r",
                )?;
                let h = val_num(
                    &args.get("h").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "cone.h",
                )?;
                Ok(Geometry::ConeGeometry(ConeGeometry::new(r, h)))
            }
            "torus" => {
                let r1 = val_num(
                    &args.get("R").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "torus.R",
                )?;
                let r2 = val_num(
                    &args.get("r").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "torus.r",
                )?;
                let arc = val_num(
                    &args
                        .get("arc")
                        .cloned()
                        .unwrap_or(ArgValue::Num(std::f64::consts::TAU)),
                    line,
                    "torus.arc",
                )?;
                if arc >= std::f64::consts::TAU {
                    Ok(Geometry::TorusGeometry(TorusGeometry::new(r1, r2)))
                } else {
                    Ok(Geometry::TorusGeometry(TorusGeometry::tube(r1, r2, arc)))
                }
            }
            "cyclide" => {
                let a = val_num(
                    &args.get("a").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "cyclide.a",
                )?;
                let b = val_num(
                    &args.get("b").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "cyclide.b",
                )?;
                let d = val_num(
                    &args.get("d").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "cyclide.d",
                )?;
                Ok(Geometry::CyclideGeometry(CyclideGeometry::new(
                    a,
                    b,
                    d,
                    [0.0, 0.0, 0.0],
                )))
            }
            "ellipsoid" => {
                let r = val_vec3(
                    &args.get("radii").cloned().unwrap_or(ArgValue::Num(0.0)),
                    line,
                    "ellipsoid.radii",
                )?;
                Ok(Geometry::EllipsoidGeometry(EllipsoidGeometry::new(
                    r[0], r[1], r[2],
                )))
            }
            _ => Err(format!("build: unknown primitive {name}")),
        }
    }

    pub(crate) fn build_material(
        &self,
        mat: &HashMap<String, ArgValue>,
    ) -> Result<Material, String> {
        let color = match mat.get("color") {
            Some(v) => {
                let c = val_num(v, 0, "color")?;
                Color::from_hex(c as i32)
            }
            None => Color::from_hex(0xFFFFFF),
        };
        let mut tex = None;
        if let Some(v) = mat.get("map") {
            match v {
                ArgValue::Str(path) => {
                    if !path.is_empty() {
                        if self.asset_root.is_empty() {
                            return Err(
                                "build: material.map needs an explicit asset_root".to_string()
                            );
                        }
                        tex = Some(texture_load(&format!("{}/{}", self.asset_root, path))?);
                    }
                }
                _ => return Err("build: material.map needs a string path".to_string()),
            }
        }
        if let Some(u) = mat.get("unlit") {
            if arg_truthy(u) {
                let op = match mat.get("opacity") {
                    Some(o) => val_num(o, 0, "opacity")?,
                    None => 1.0,
                };
                return Ok(Material::basic(color, clamp01(op)));
            }
        }
        let roughness = val_opt_num(
            &mat.get("roughness").cloned().unwrap_or(ArgValue::Num(-1.0)),
            0.5,
        );
        let metalness = val_opt_num(
            &mat.get("metalness").cloned().unwrap_or(ArgValue::Num(-1.0)),
            0.0,
        );
        let emissive = match mat.get("emissive") {
            Some(v) => {
                let ev = val_num(v, 0, "emissive")?;
                if ev < 0.0 {
                    Color::from_hex(0x000000)
                } else {
                    Color::from_hex(ev as i32)
                }
            }
            None => Color::from_hex(0x000000),
        };
        let opacity = val_opt_num(
            &mat.get("opacity").cloned().unwrap_or(ArgValue::Num(-1.0)),
            1.0,
        );
        let ior = val_opt_num(&mat.get("ior").cloned().unwrap_or(ArgValue::Num(-1.0)), 1.5);
        let absorption = val_opt_num(
            &mat.get("absorption")
                .cloned()
                .unwrap_or(ArgValue::Num(-1.0)),
            0.0,
        );
        let mut m = Material::standard(MaterialParams {
            color,
            roughness,
            metalness,
            emissive,
            opacity,
            ior,
            absorption,
        });
        if let Some(t) = tex {
            m.map = Some(t);
        }
        Ok(m)
    }
}
