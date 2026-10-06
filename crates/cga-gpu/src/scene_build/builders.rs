//! Builders-only residue of the retired CGS parser (R4): geometry/material
//! construction, parameter defaults and validation. Statement parsing lives
//! in the JSX host (crate::jsx).
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
        crate::geometry_ops::geom_bounds(&geo.identity_params())
    }

    pub(crate) fn resolve(
        &self,
        name: &str,
        pos: Vec<CgsValue>,
        kw: HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<HashMap<String, CgsValue>, String> {
        let names = cgs_sig_names(name);
        let defaults = cgs_sig_defaults(name);
        let mut merged: HashMap<String, CgsValue> = HashMap::new();
        for (k, v) in &defaults {
            merged.insert(k.clone(), v.clone());
        }
        if pos.len() > names.len() {
            return Err(format!("CGS line {line}: {name} too many positional args"));
        }
        for (i, pname) in names.iter().enumerate() {
            if i < pos.len() {
                merged.insert(pname.to_string(), pos[i].clone());
            }
        }
        for (k, v) in kw {
            if !names.contains(&k.as_str()) && !defaults.contains_key(&k) {
                return Err(format!("CGS line {line}: {name} has no parameter {k}"));
            }
            merged.insert(k, v);
        }
        for pname in &names {
            if let Some(CgsValue::Num(x)) = merged.get(*pname) {
                if x.is_nan() && *pname == "cylinder.h" {
                    merged.insert(pname.to_string(), CgsValue::Num(-1.0));
                }
            }
        }
        Ok(merged)
    }

    pub(crate) fn build_geometry(
        &mut self,
        name: &str,
        args: &HashMap<String, CgsValue>,
        line: i32,
    ) -> Result<Geometry, String> {
        validate_geometry_params(name, args, line)?;
        match name {
            "sphere" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "sphere.r",
                )?;
                Ok(Geometry::SphereGeometry(SphereGeometry::new(r)))
            }
            "plane" => {
                let n = cgs_vec3(
                    &args.get("n").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "plane.n",
                )?;
                let d = cgs_num(
                    &args.get("d").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "plane.d",
                )?;
                Ok(Geometry::PlaneGeometry(PlaneGeometry::new(n, d)))
            }
            "cylinder" => {
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cylinder.h",
                )?;
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cylinder.r",
                )?;
                Ok(Geometry::CylinderGeometry(CylinderGeometry::new(
                    r,
                    if h < 0.0 { -1.0 } else { h },
                )))
            }
            "box" => {
                let s = cgs_vec3(
                    &args.get("s").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "box.s",
                )?;
                Ok(Geometry::BoxGeometry(BoxGeometry::new(s[0], s[1], s[2])))
            }
            "circle" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "circle.r",
                )?;
                Ok(Geometry::CircleGeometry(CircleGeometry::new(r)))
            }
            "cone" => {
                let r = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cone.r",
                )?;
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cone.h",
                )?;
                Ok(Geometry::ConeGeometry(ConeGeometry::new(r, h)))
            }
            "torus" => {
                let r1 = cgs_num(
                    &args.get("R").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "torus.R",
                )?;
                let r2 = cgs_num(
                    &args.get("r").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "torus.r",
                )?;
                let arc = cgs_num(
                    &args
                        .get("arc")
                        .cloned()
                        .unwrap_or(CgsValue::Num(std::f64::consts::TAU)),
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
                let a = cgs_num(
                    &args.get("a").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cyclide.a",
                )?;
                let b = cgs_num(
                    &args.get("b").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "cyclide.b",
                )?;
                let d = cgs_num(
                    &args.get("d").cloned().unwrap_or(CgsValue::Num(0.0)),
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
                let r = cgs_vec3(
                    &args.get("radii").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "ellipsoid.radii",
                )?;
                Ok(Geometry::EllipsoidGeometry(EllipsoidGeometry::new(
                    r[0], r[1], r[2],
                )))
            }
            "bezier" => {
                let raw = args.get("points").cloned().unwrap_or(CgsValue::Num(0.0));
                let pts = points16(&raw, line, "bezier.points")?;
                let thickness = cgs_num(
                    &args.get("thickness").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "bezier.thickness",
                )?;
                let div = cgs_num(
                    &args.get("div").cloned().unwrap_or(CgsValue::Num(8.0)),
                    line,
                    "bezier.div",
                )? as usize;
                Ok(Geometry::BezierPatchGeometry(BezierPatchGeometry::new(
                    &pts, thickness, div,
                )))
            }
            "extrude" => {
                let prof = profile2d(
                    &args.get("profile").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "extrude.profile",
                )?;
                let h = cgs_num(
                    &args.get("h").cloned().unwrap_or(CgsValue::Num(0.0)),
                    line,
                    "extrude.h",
                )?;
                let (v, f) = extrude(&prof, h);
                Ok(Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f)))
            }
            "loft" => {
                let raw = args.get("profiles").cloned().unwrap_or(CgsValue::Num(0.0));
                match raw {
                    CgsValue::List(list) => {
                        if list.len() < 2 {
                            return Err(format!(
                                "CGS line {line}: loft.profiles needs >= 2 sections"
                            ));
                        }
                        let mut profiles: Vec<Vec<[f64; 2]>> = Vec::new();
                        for p in &list {
                            profiles.push(profile2d(p, line, "loft.profiles[i]")?);
                        }
                        let zsraw = args.get("zs").cloned().unwrap_or(CgsValue::Num(0.0));
                        match zsraw {
                            CgsValue::Vec3(v3) => {
                                let (v, f) = loft(&profiles, &[v3.x, v3.y, v3.z]);
                                Ok(Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f)))
                            }
                            CgsValue::List(zlist) => {
                                let mut zs: Vec<f64> = Vec::new();
                                for z in &zlist {
                                    zs.push(cgs_num(z, line, "loft.zs[i]")?);
                                }
                                let (v, f) = loft(&profiles, &zs);
                                Ok(Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f)))
                            }
                            _ => Err(format!("CGS line {line}: loft.zs needs a list")),
                        }
                    }
                    _ => Err(format!("CGS line {line}: loft.profiles needs a list")),
                }
            }
            "mesh" => {
                let p = args.get("file").cloned().unwrap_or(CgsValue::Num(0.0));
                match p {
                    CgsValue::Str(path) => {
                        if self.asset_root.is_empty() {
                            return Err(format!(
                                "CGS line {line}: mesh needs an explicit asset_root"
                            ));
                        }
                        let full = format!("{}/{}", self.asset_root, path);
                        if full.ends_with(".obj") {
                            let (v, f) = load_obj(&full)?;
                            return Ok(Geometry::TrimeshGeometry(TrimeshGeometry::new(&v, &f)));
                        }
                        if full.ends_with(".glb") || full.ends_with(".gltf") {
                            let loaded = load_gltf(&full)?;
                            return Ok(gltf_to_geometry(&loaded));
                        }
                        Err(format!(
                            "CGS line {line}: unsupported mesh file \"{path}\" (use .obj/.glb/.gltf)"
                        ))
                    }
                    _ => Err(format!("CGS line {line}: mesh.file needs a string path")),
                }
            }
            _ => Err(format!("CGS line {line}: unknown primitive {name}")),
        }
    }

    pub(crate) fn build_material(
        &self,
        mat: &HashMap<String, CgsValue>,
    ) -> Result<Material, String> {
        let color = match mat.get("color") {
            Some(v) => {
                let c = cgs_num(v, 0, "color")?;
                Color::from_hex(c as i32)
            }
            None => Color::from_hex(0xFFFFFF),
        };
        let mut tex = None;
        if let Some(v) = mat.get("map") {
            match v {
                CgsValue::Str(path) => {
                    if !path.is_empty() {
                        if self.asset_root.is_empty() {
                            return Err("CGS material.map needs an explicit asset_root".to_string());
                        }
                        tex = Some(texture_load(&format!("{}/{}", self.asset_root, path))?);
                    }
                }
                _ => return Err("CGS material.map needs a string path".to_string()),
            }
        }
        if let Some(u) = mat.get("unlit") {
            if cgs_truthy(u) {
                let op = match mat.get("opacity") {
                    Some(o) => cgs_num(o, 0, "opacity")?,
                    None => 1.0,
                };
                return Ok(Material::basic(color, clamp01(op)));
            }
        }
        let roughness = cgs_opt_num(
            &mat.get("roughness").cloned().unwrap_or(CgsValue::Num(-1.0)),
            0.5,
        );
        let metalness = cgs_opt_num(
            &mat.get("metalness").cloned().unwrap_or(CgsValue::Num(-1.0)),
            0.0,
        );
        let emissive = match mat.get("emissive") {
            Some(v) => {
                let ev = cgs_num(v, 0, "emissive")?;
                if ev < 0.0 {
                    Color::from_hex(0x000000)
                } else {
                    Color::from_hex(ev as i32)
                }
            }
            None => Color::from_hex(0x000000),
        };
        let opacity = cgs_opt_num(
            &mat.get("opacity").cloned().unwrap_or(CgsValue::Num(-1.0)),
            1.0,
        );
        let ior = cgs_opt_num(&mat.get("ior").cloned().unwrap_or(CgsValue::Num(-1.0)), 1.5);
        let absorption = cgs_opt_num(
            &mat.get("absorption")
                .cloned()
                .unwrap_or(CgsValue::Num(-1.0)),
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
