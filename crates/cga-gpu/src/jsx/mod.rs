//! JSX + CSS scene host (docs/cgs-react-css.md, route C-with-A: real JS via
//! boa, JSX compiled by swc, CSS parsed by lightningcss).
//!
//! Pipeline: .jsx source → swc (parse JSX → h() calls) → boa executes real
//! JS (components, map, Math, …) → element tree → scene builder (reuses the
//! CGS geometry/material builders and the kinematics registry) → `CgsRun`.
//!
//! Conventions:
//! - The scene is the `export default <element>` value.
//! - Builtin elements are PascalCase globals bound to lowercase strings, or
//!   lowercase tags directly: `<Sphere r={1}/>` ≡ `<sphere r={1}/>`.
//! - CSS matches by tag / `.class` / `#id`; material properties inherit down
//!   the element tree; inline props beat matched CSS.

use std::collections::HashMap;

use boa_engine::{Context, Source};
use serde_json::Value;
use swc_core::common::{sync::Lrc, FileName, Globals, Mark, SourceMap, GLOBALS};
use swc_core::ecma::ast::{
    BindingIdent, Decl, Ident, ModuleDecl, ModuleItem, Pat, Stmt, VarDecl, VarDeclKind,
    VarDeclarator,
};
use swc_core::ecma::codegen::{text_writer::JsWriter, Config as CodegenConfig, Emitter};
use swc_core::ecma::parser::{lexer::Lexer, EsSyntax, Parser, StringInput, Syntax};
use swc_core::ecma::transforms::base::{hygiene, resolver};
use swc_core::ecma::transforms::react::{jsx, Options as ReactOptions, Runtime};
use swc_core::ecma::visit::VisitMutWith;

use cga_core::Multivector;

use crate::scene::{Mesh, MeshParams, PerspectiveCamera, Scene};
use crate::scene_graph::Color;
use crate::scene_lang::{
    cam_solve, joint_motion, rpy4, CamProfile, CamRel, CamSolved, CgsValue, Driven, GearRel,
    JointDef, JointKind, Kinematics, SceneLoader, TagInstance, TagRegistry,
};
use crate::shading::Light;

const PRELUDE: &str = r#"
function h(tag, props, ...children) {
  const flat = [];
  const push = (c) => {
    if (c === null || c === undefined || c === true || c === false) return;
    if (Array.isArray(c)) { c.forEach(push); return; }
    flat.push(c);
  };
  children.forEach(push);
  if (typeof tag === 'function') return tag({ ...(props || {}), children: flat });
  return { __el: true, tag, props: props || {}, children: flat };
}
const Fragment = 'fragment';
const Sphere='sphere',Plane='plane',Cylinder='cylinder',Box='box',Circle='circle',Cone='cone',
Torus='torus',Cyclide='cyclide',Ellipsoid='ellipsoid',Bezier='bezier',Extrude='extrude',
Loft='loft',Mesh='mesh',Translate='translate',Rotate='rotate',Scale='scale',Mirror='mirror',
Material='material',Union='union',Difference='difference',Intersection='intersection',
AmbientLight='ambient_light',DirectionalLight='directional_light',PointLight='point_light',
Camera='camera',Background='background',Scene='scene',Joint='joint',Gear='gear',Cam='cam',
Tag='tag';
function __dump(v) {
  if (v === null || v === undefined) return null;
  if (Array.isArray(v)) return v.map(__dump);
  if (typeof v === 'object') {
    if (v.__el) return { t: String(v.tag), p: __dump(v.props), c: v.children.map(__dump) };
    const o = {};
    for (const k of Object.keys(v)) o[k] = __dump(v[k]);
    return o;
  }
  return v;
}
"#;

/// swc: parse JSX, rewrite `export default` → `const __scene`, strip .css
/// imports, transform JSX → h() calls, print JS.
fn compile_jsx(src: &str) -> Result<String, String> {
    let cm: Lrc<SourceMap> = Default::default();
    let fm = cm.new_source_file(Lrc::new(FileName::Anon), src.to_string());
    let lexer = Lexer::new(
        Syntax::Es(EsSyntax {
            jsx: true,
            ..Default::default()
        }),
        Default::default(),
        StringInput::from(&*fm),
        None,
    );
    let mut parser = Parser::new_from(lexer);
    let mut module = parser.parse_module().map_err(|e| {
        use swc_core::common::Spanned;
        let line = cm
            .lookup_line(e.span().lo())
            .map(|l| l.line + 1)
            .unwrap_or(0);
        format!("JSX line {line}: {}", e.kind().msg())
    })?;
    let mut has_scene = false;
    let mut body: Vec<ModuleItem> = Vec::new();
    for item in module.body {
        match item {
            ModuleItem::ModuleDecl(ModuleDecl::Import(imp)) => {
                let src = imp.src.value.to_string_lossy().into_owned();
                if !src.ends_with(".css") {
                    return Err(format!("JSX: only .css imports are supported, got {src}"));
                }
            }
            ModuleItem::ModuleDecl(ModuleDecl::ExportDefaultExpr(e)) => {
                has_scene = true;
                body.push(ModuleItem::Stmt(Stmt::Decl(Decl::Var(Box::new(VarDecl {
                    span: e.span,
                    ctxt: Default::default(),
                    kind: VarDeclKind::Const,
                    declare: false,
                    decls: vec![VarDeclarator {
                        span: e.span,
                        name: Pat::Ident(BindingIdent {
                            id: Ident::new_no_ctxt("__scene".into(), e.span),
                            type_ann: None,
                        }),
                        init: Some(e.expr),
                        definite: false,
                    }],
                })))));
            }
            other => body.push(other),
        }
    }
    if !has_scene {
        return Err("JSX: scene file must end with export default <element>".to_string());
    }
    module.body = body;
    GLOBALS.set(&Globals::new(), || {
        let unresolved = Mark::new();
        let top = Mark::new();
        module.visit_mut_with(&mut resolver(unresolved, top, false));
        module.visit_mut_with(&mut jsx(
            cm.clone(),
            None::<swc_core::common::comments::SingleThreadedComments>,
            ReactOptions {
                pragma: Some("h".into()),
                pragma_frag: Some("Fragment".into()),
                runtime: Some(Runtime::Classic),
                ..Default::default()
            },
            unresolved,
            top,
        ));
        module.visit_mut_with(&mut hygiene::hygiene());
        let mut buf = vec![];
        let mut emitter = Emitter {
            cfg: CodegenConfig::default(),
            cm: cm.clone(),
            comments: None,
            wr: JsWriter::new(cm, "", &mut buf, None),
        };
        emitter
            .emit_module(&module)
            .map_err(|e| format!("JSX: codegen failed: {e}"))?;
        String::from_utf8(buf).map_err(|e| format!("JSX: codegen utf8: {e}"))
    })
}

/// boa executes the compiled module; the scene element tree comes back as
/// JSON (all prop values are numbers/strings/bools/arrays/objects).
fn eval_js(js: &str) -> Result<Value, String> {
    let mut ctx = Context::default();
    ctx.eval(Source::from_bytes(&format!(
        "{PRELUDE}{js}\nglobalThis.__out = JSON.stringify(__dump(__scene));"
    )))
    .map_err(|e| format!("JSX eval: {e}"))?;
    let out = ctx
        .global_object()
        .get(boa_engine::js_string!("__out"), &mut ctx)
        .map_err(|e| format!("JSX eval: {e}"))?;
    let s = out
        .to_string(&mut ctx)
        .map_err(|e| format!("JSX eval: {e}"))?
        .to_std_string_escaped();
    serde_json::from_str(&s).map_err(|e| format!("JSX: scene serialization failed: {e}"))
}

#[derive(Clone, Debug)]
pub struct El {
    pub tag: String,
    pub props: HashMap<String, Value>,
    pub children: Vec<El>,
}

fn to_el(v: &Value) -> Result<El, String> {
    let obj = v
        .as_object()
        .ok_or_else(|| format!("JSX: scene is not an element: {v}"))?;
    let tag = obj
        .get("t")
        .and_then(Value::as_str)
        .ok_or_else(|| "JSX: element without tag".to_string())?;
    let props: HashMap<String, Value> = obj
        .get("p")
        .and_then(Value::as_object)
        .map(|m| m.iter().map(|(k, v)| (k.clone(), v.clone())).collect())
        .unwrap_or_default();
    let mut children = Vec::new();
    if let Some(c) = obj.get("c").and_then(Value::as_array) {
        for c in c {
            if c.is_null() {
                continue;
            }
            children.push(to_el(c)?);
        }
    }
    Ok(El {
        tag: tag.to_string(),
        props,
        children,
    })
}

// ---- prop readers ----

fn prop<'a>(el: &'a El, key: &str) -> Option<&'a Value> {
    el.props.get(key).or_else(|| {
        // camelCase ↔ snake_case 双写兼容
        let snake: String = key
            .chars()
            .flat_map(|c| {
                if c.is_uppercase() {
                    vec!['_', c.to_ascii_lowercase()]
                } else {
                    vec![c]
                }
            })
            .collect();
        el.props.get(&snake)
    })
}

fn p_num(el: &El, key: &str) -> Result<Option<f64>, String> {
    match prop(el, key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Number(n)) => Ok(n.as_f64()),
        Some(v) => Err(format!("JSX: {key} must be a number, got {v}")),
    }
}

fn p_vec3(el: &El, key: &str) -> Result<Option<[f64; 3]>, String> {
    match prop(el, key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Array(a)) if a.len() == 3 => {
            let mut out = [0.0; 3];
            for i in 0..3 {
                out[i] = a[i]
                    .as_f64()
                    .ok_or_else(|| format!("JSX: {key}[{i}] must be a number"))?;
            }
            Ok(Some(out))
        }
        Some(v) => Err(format!("JSX: {key} must be [x, y, z], got {v}")),
    }
}

fn p_str(el: &El, key: &str) -> Result<Option<String>, String> {
    match prop(el, key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(s)) => Ok(Some(s.clone())),
        Some(v) => Err(format!("JSX: {key} must be a string, got {v}")),
    }
}

fn p_num_list(el: &El, key: &str) -> Result<Option<Vec<f64>>, String> {
    match prop(el, key) {
        None | Some(Value::Null) => Ok(None),
        Some(Value::Array(a)) => a
            .iter()
            .map(|v| {
                v.as_f64()
                    .ok_or_else(|| format!("JSX: {key} items must be numbers"))
                    .map(Some)
            })
            .collect::<Result<Vec<_>, _>>()
            .map(|v| Some(v.into_iter().flatten().collect())),
        Some(v) => Err(format!("JSX: {key} must be a number list, got {v}")),
    }
}

fn cgs_of(v: &Value) -> CgsValue {
    match v {
        Value::Number(n) => CgsValue::Num(n.as_f64().unwrap_or(0.0)),
        Value::Bool(b) => CgsValue::Bool(*b),
        Value::String(s) => CgsValue::Str(s.clone()),
        Value::Array(a) => CgsValue::List(a.iter().map(cgs_of).collect()),
        _ => CgsValue::Num(0.0),
    }
}

const MATERIAL_KEYS: [&str; 8] = [
    "color",
    "roughness",
    "metalness",
    "emissive",
    "opacity",
    "ior",
    "absorption",
    "map",
];

// ---- CSS ----

#[derive(Clone, Debug, Default)]
struct SimpleSel {
    tag: Option<String>,
    class: Option<String>,
    id: Option<String>,
    scene: bool,
}

#[derive(Clone, Debug)]
struct StyleRule {
    sel: SimpleSel,
    props: Vec<(String, String)>,
}

fn parse_selector_part(part: &str) -> SimpleSel {
    let mut sel = SimpleSel::default();
    let mut rest = part.trim();
    if rest == ":root" || rest == "scene" {
        sel.scene = true;
        return sel;
    }
    while !rest.is_empty() {
        if let Some(r) = rest.strip_prefix('.') {
            let end = r.find(['.', '#']).unwrap_or(r.len());
            sel.class = Some(r[..end].to_string());
            rest = &r[end..];
        } else if let Some(r) = rest.strip_prefix('#') {
            let end = r.find(['.', '#']).unwrap_or(r.len());
            sel.id = Some(r[..end].to_string());
            rest = &r[end..];
        } else {
            let end = rest.find(['.', '#']).unwrap_or(rest.len());
            sel.tag = Some(rest[..end].to_ascii_lowercase());
            rest = &rest[end..];
        }
    }
    sel
}

fn parse_css(src: &str) -> Result<Vec<StyleRule>, String> {
    let ss = lightningcss::stylesheet::StyleSheet::parse(
        src,
        lightningcss::stylesheet::ParserOptions::default(),
    )
    .map_err(|e| format!("CSS: {e}"))?;
    let mut rules = Vec::new();
    for rule in &ss.rules.0 {
        let lightningcss::rules::CssRule::Style(style) = rule else {
            continue;
        };
        let sel_text = style.selectors.to_string();
        let mut props = Vec::new();
        for d in style
            .declarations
            .declarations
            .iter()
            .chain(style.declarations.important_declarations.iter())
        {
            if let Ok(v) = d.value_to_css_string(lightningcss::printer::PrinterOptions::default()) {
                props.push((d.property_id().name().to_string(), v));
            }
        }
        for part in sel_text.split(',') {
            rules.push(StyleRule {
                sel: parse_selector_part(part),
                props: props.clone(),
            });
        }
    }
    Ok(rules)
}

/// Matched material props for one element (cascade = source order).
fn css_match(rules: &[StyleRule], el: &El) -> Vec<(String, String)> {
    let class = p_str(el, "class").ok().flatten();
    let class2 = p_str(el, "className").ok().flatten();
    let id = p_str(el, "id").ok().flatten();
    let mut out = Vec::new();
    for r in rules {
        let s = &r.sel;
        let ok = (s.tag.is_none() || s.tag.as_deref() == Some(el.tag.as_str()))
            && (s.class.is_none() || s.class == class || s.class == class2)
            && (s.id.is_none() || s.id == id);
        if ok {
            out.extend(r.props.iter().cloned());
        }
    }
    out
}

fn css_value_to_arg(name: &str, raw: &str) -> Option<CgsValue> {
    let raw = raw.trim().trim_matches('"').trim_matches('\'');
    match name {
        "color" | "emissive" | "background" | "background-color" => {
            let h = raw.strip_prefix('#')?;
            let expanded = match h.len() {
                3 => h.chars().flat_map(|c| [c, c]).collect::<String>(),
                _ => h.to_string(),
            };
            let v = i32::from_str_radix(&expanded, 16).ok()?;
            Some(CgsValue::Num(v as f64))
        }
        "map" => {
            let p = raw
                .strip_prefix("url(")
                .and_then(|s| s.strip_suffix(')'))
                .unwrap_or(raw)
                .trim_matches('"')
                .trim_matches('\'');
            Some(CgsValue::Str(p.to_string()))
        }
        _ => raw.parse::<f64>().ok().map(CgsValue::Num),
    }
}

// ---- scene builder ----

struct Builder {
    loader: SceneLoader,
    kin: Kinematics,
    joint_stack: Vec<String>,
    driven_by: HashMap<String, Driven>,
    tags: TagRegistry,
    pending_tags: Vec<(String, [f64; 16])>,
    rules: Vec<StyleRule>,
}

fn mat4_mul(a: [f64; 16], b: [f64; 16]) -> [f64; 16] {
    cga_core::mat4_mul(a, b)
}

fn translate4(t: [f64; 3]) -> [f64; 16] {
    [
        1.0, 0.0, 0.0, t[0], 0.0, 1.0, 0.0, t[1], 0.0, 0.0, 1.0, t[2], 0.0, 0.0, 0.0, 1.0,
    ]
}

impl Builder {
    fn material_for(
        &self,
        el: &El,
        mat: &HashMap<String, CgsValue>,
    ) -> Result<crate::shading::Material, String> {
        let mut merged: HashMap<String, CgsValue> = mat.clone();
        for (k, v) in css_match(&self.rules, el) {
            let key = k.trim_start_matches("--").to_string();
            if let Some(val) = css_value_to_arg(&k, &v) {
                merged.insert(key, val);
            }
        }
        for k in MATERIAL_KEYS {
            if let Some(v) = el.props.get(k) {
                merged.insert(k.to_string(), cgs_of(v));
            }
        }
        self.loader
            .build_material(&merged)
            .map_err(|e| e.replacen("CGS line 0: ", "JSX: ", 1))
    }

    fn emit(
        &mut self,
        scene: &mut Scene,
        geo: cga_core::Geometry,
        world: [f64; 16],
        mat: crate::shading::Material,
    ) {
        let (motor, lin) = cga_core::decompose_rigid(world);
        let g2 = if crate::scene_graph::is_identity3(lin) {
            geo
        } else {
            cga_core::Geometry::AffineGeometry(cga_core::AffineGeometry::new(geo, lin))
        };
        scene.add_mesh(Mesh::new(MeshParams {
            geometry: g2,
            material: mat,
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: Some(motor),
        }));
        let idx = scene.objects.len() - 1;
        if let Some(jname) = self.joint_stack.last().cloned() {
            if let Some(j) = self.kin.joints.iter_mut().rev().find(|j| j.name == jname) {
                j.meshes.push(idx);
            }
        }
        let _ = world;
        for (name, entry) in self.pending_tags.clone() {
            let rel = mat4_mul(mat4_inv(entry), world);
            self.tags.entry(name).or_default().push(TagInstance {
                geo: scene.objects[idx].geometry.clone(),
                world,
                rel,
            });
        }
    }

    fn walk(
        &mut self,
        el: &El,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scene: &mut Scene,
        cam: &mut Option<PerspectiveCamera>,
    ) -> Result<(), String> {
        match el.tag.as_str() {
            "fragment" | "scene" => {
                if el.tag == "scene" {
                    if let Some(c) = p_str(el, "background")? {
                        scene.background = parse_hex(&c)?;
                    }
                    if let Some(b) = p_num(el, "background")? {
                        scene.background = Color::from_hex(b as i32);
                    }
                }
                for c in &el.children {
                    self.walk(c, ctx, mat, scene, cam)?;
                }
                Ok(())
            }
            "translate" | "rotate" | "scale" | "mirror" => {
                let m2 = mat4_mul(ctx, self.modifier_matrix(el)?);
                for c in &el.children {
                    self.walk(c, m2, mat, scene, cam)?;
                }
                Ok(())
            }
            "material" => {
                let mut merged = mat.clone();
                for (k, v) in &el.props {
                    if MATERIAL_KEYS.contains(&k.as_str()) {
                        merged.insert(k.clone(), cgs_of(v));
                    }
                }
                for c in &el.children {
                    self.walk(c, ctx, &merged, scene, cam)?;
                }
                Ok(())
            }
            "union" | "difference" | "intersection" => {
                let mut kids = Vec::new();
                for c in &el.children {
                    self.collect_geom(c, ctx, &mut kids)?;
                }
                if kids.len() < 2 && el.tag != "union" {
                    return Err(format!("JSX: {} needs >= 2 geometry children", el.tag));
                }
                let op = match el.tag.as_str() {
                    "union" => cga_core::CsgOp::Union,
                    "difference" => cga_core::CsgOp::Difference,
                    _ => cga_core::CsgOp::Intersection,
                };
                let material = self.material_for(el, mat)?;
                self.emit(
                    scene,
                    cga_core::Geometry::CsgGeometry(cga_core::CsgGeometry::new(op, kids)),
                    ctx,
                    material,
                );
                Ok(())
            }
            "ambient_light" | "directional_light" | "point_light" => {
                let intensity = p_num(el, "intensity")?.unwrap_or(1.0);
                let color = match p_num(el, "color")? {
                    Some(c) => Color::from_hex(c as i32),
                    None => Color::from_hex(0xFFFFFF),
                };
                let l = match el.tag.as_str() {
                    "ambient_light" => Light::ambient(color, intensity),
                    "directional_light" => Light::directional(
                        color,
                        intensity,
                        p_vec3(el, "direction")?.unwrap_or([0.0, 1.0, 0.0]),
                    ),
                    _ => Light::point(
                        color,
                        intensity,
                        p_vec3(el, "position")?.unwrap_or([0.0, 0.0, 0.0]),
                    ),
                };
                scene.add_light(l);
                Ok(())
            }
            "camera" => {
                let fov = p_num(el, "fov")?.unwrap_or(50.0);
                if fov <= 0.0 || fov >= 180.0 {
                    return Err(format!("JSX: camera fov must be in (0, 180), got {fov}"));
                }
                let position = p_vec3(el, "position")?.unwrap_or([0.0, 0.0, 5.0]);
                let target = p_vec3(el, "target")?.unwrap_or([0.0, 0.0, 0.0]);
                let mut c = PerspectiveCamera::new(
                    fov,
                    p_num(el, "aspect")?.unwrap_or(16.0 / 9.0),
                    0.1,
                    100.0,
                    position,
                    target,
                    [0.0, 1.0, 0.0],
                );
                c.look_at(target, None);
                *cam = Some(c);
                Ok(())
            }
            "background" => {
                let c = p_num(el, "color")?.unwrap_or(0.0);
                scene.background = Color::from_hex(c as i32);
                Ok(())
            }
            "tag" => {
                let name = p_str(el, "name")?.ok_or_else(|| "JSX: tag needs a name".to_string())?;
                self.pending_tags.push((name, ctx));
                for c in &el.children {
                    self.walk(c, ctx, mat, scene, cam)?;
                }
                self.pending_tags.pop();
                Ok(())
            }
            "joint" => self.joint_el(el, ctx, mat, scene, cam),
            "gear" => self.gear_el(el),
            "cam" => self.cam_el(el),
            _ => self.primitive_el(el, ctx, mat, scene),
        }
    }

    fn collect_geom(
        &mut self,
        el: &El,
        ctx: [f64; 16],
        kids: &mut Vec<cga_core::Geometry>,
    ) -> Result<(), String> {
        match el.tag.as_str() {
            "translate" | "rotate" | "scale" | "mirror" => {
                let m = self.modifier_matrix(el)?;
                self.collect_geom_children(el, mat4_mul(ctx, m), kids)
            }
            "union" | "difference" | "intersection" => {
                let mut inner = Vec::new();
                for c in &el.children {
                    self.collect_geom(c, ctx, &mut inner)?;
                }
                let op = match el.tag.as_str() {
                    "union" => cga_core::CsgOp::Union,
                    "difference" => cga_core::CsgOp::Difference,
                    _ => cga_core::CsgOp::Intersection,
                };
                kids.push(cga_core::Geometry::CsgGeometry(cga_core::CsgGeometry::new(
                    op, inner,
                )));
                Ok(())
            }
            _ => {
                let geo = self.build_geo(el)?;
                let (motor, lin) = cga_core::decompose_rigid(ctx);
                kids.push(cga_core::Geometry::AffineGeometry(
                    cga_core::AffineGeometry::with_motor(geo, motor, lin),
                ));
                Ok(())
            }
        }
    }

    fn collect_geom_children(
        &mut self,
        el: &El,
        ctx: [f64; 16],
        kids: &mut Vec<cga_core::Geometry>,
    ) -> Result<(), String> {
        for c in &el.children {
            self.collect_geom(c, ctx, kids)?;
        }
        Ok(())
    }

    fn modifier_matrix(&self, el: &El) -> Result<[f64; 16], String> {
        Ok(match el.tag.as_str() {
            "translate" => translate4(p_vec3(el, "t")?.unwrap_or([0.0; 3])),
            "rotate" => {
                let ax = p_vec3(el, "axis")?.unwrap_or([0.0, 0.0, 1.0]);
                let ang = p_num(el, "angle")?.unwrap_or(0.0);
                Multivector::rotor(ax, ang).to_matrix()
            }
            "scale" => {
                let s = p_num(el, "s")?;
                let sv = p_vec3(el, "s")?;
                let v = match (s, sv) {
                    (Some(x), None) => [x, x, x],
                    (None, Some(v)) => v,
                    _ => [1.0, 1.0, 1.0],
                };
                let mut m = cga_core::mat4_identity();
                m[0] = v[0];
                m[5] = v[1];
                m[10] = v[2];
                m
            }
            _ => {
                let ax = p_vec3(el, "axis")?.unwrap_or([0.0, 0.0, 1.0]);
                let n = (ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]).sqrt();
                if n < 1e-12 {
                    return Err("JSX: mirror axis must be nonzero".to_string());
                }
                let (x, y, z) = (ax[0] / n, ax[1] / n, ax[2] / n);
                [
                    1.0 - 2.0 * x * x,
                    -2.0 * x * y,
                    -2.0 * x * z,
                    0.0,
                    -2.0 * x * y,
                    1.0 - 2.0 * y * y,
                    -2.0 * y * z,
                    0.0,
                    -2.0 * x * z,
                    -2.0 * y * z,
                    1.0 - 2.0 * z * z,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ]
            }
        })
    }

    fn build_geo(&mut self, el: &El) -> Result<cga_core::Geometry, String> {
        let mut args: HashMap<String, CgsValue> = HashMap::new();
        for (k, v) in &el.props {
            if MATERIAL_KEYS.contains(&k.as_str()) || k == "class" || k == "className" || k == "id"
            {
                continue;
            }
            args.insert(k.clone(), cgs_of(v));
        }
        self.loader
            .build_geometry(&el.tag, &args, 0)
            .map_err(|e| e.replacen("CGS line 0: ", "JSX: ", 1))
    }

    fn primitive_el(
        &mut self,
        el: &El,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scene: &mut Scene,
    ) -> Result<(), String> {
        let geo = self.build_geo(el)?;
        let material = self.material_for(el, mat)?;
        self.emit(scene, geo, ctx, material);
        Ok(())
    }

    fn joint_el(
        &mut self,
        el: &El,
        ctx: [f64; 16],
        mat: &HashMap<String, CgsValue>,
        scene: &mut Scene,
        cam: &mut Option<PerspectiveCamera>,
    ) -> Result<(), String> {
        let name = p_str(el, "name")?.ok_or_else(|| "JSX: joint needs a name".to_string())?;
        let type_err = "JSX: joint.type must be \"revolute\", \"continuous\", \"prismatic\", \"helical\", \"cylindrical\", \"spherical\", \"planar\" or \"fixed\"";
        let kind = match p_str(el, "type")?.as_deref() {
            Some(s) => JointKind::parse(s).ok_or_else(|| type_err.to_string())?,
            None => return Err(type_err.to_string()),
        };
        if self.kin.joints.iter().any(|j| j.name == name) {
            return Err(format!("JSX: duplicate joint name {name}"));
        }
        let axis = p_vec3(el, "axis")?.unwrap_or([0.0, 0.0, 1.0]);
        let an = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if an < 1e-12 {
            return Err("JSX: joint.axis must be nonzero".to_string());
        }
        let axis = [axis[0] / an, axis[1] / an, axis[2] / an];
        let at = p_vec3(el, "at")?.unwrap_or([0.0; 3]);
        let rpy = p_vec3(el, "rpy")?.unwrap_or([0.0; 3]);
        let limit: Option<[f64; 2]> = match p_num_list(el, "limit")? {
            Some(l) if l.len() == 2 => Some([l[0], l[1]]),
            Some(_) => return Err("JSX: joint.limit must be [lo, hi]".to_string()),
            None => None,
        };
        let pitch = p_num(el, "pitch")?;
        if kind == JointKind::Helical && pitch.is_none() {
            return Err("JSX: helical joint needs pitch=".to_string());
        }
        let driven = self.driven_by.remove(&name);
        let q: Vec<f64> = match driven {
            Some(Driven::Gear(rel)) => {
                if !kind.is_1dof() {
                    return Err(format!("JSX: gear needs 1-DOF joints, got {}", kind.name()));
                }
                let dq = self
                    .kin
                    .joints
                    .iter()
                    .find(|j| j.name == rel.driver)
                    .map(|j| j.q[0])
                    .unwrap_or(0.0);
                vec![rel.ratio * dq + rel.offset]
            }
            Some(Driven::Cam(rel)) => {
                if !kind.is_1dof() {
                    return Err(format!(
                        "JSX: cam needs a 1-DOF driven joint, got {}",
                        kind.name()
                    ));
                }
                let [lo, hi] =
                    limit.ok_or_else(|| "JSX: cam driven joint needs limit=".to_string())?;
                let driver = self
                    .kin
                    .joints
                    .iter()
                    .find(|j| j.name == rel.driver)
                    .cloned()
                    .ok_or_else(|| format!("JSX: unknown joint {}", rel.driver))?;
                let q = cam_solve(
                    &rel,
                    &driver,
                    ctx,
                    &kind,
                    axis,
                    at,
                    pitch.unwrap_or(0.0),
                    lo,
                    hi,
                    0,
                )?;
                self.kin.cams.push(CamSolved {
                    driver: rel.driver.clone(),
                    driven: name.clone(),
                    q,
                });
                vec![q]
            }
            None => {
                let arity = kind.q_arity();
                if arity == 0 {
                    Vec::new()
                } else if let Some(x) = p_num(el, "q")? {
                    if arity != 1 {
                        return Err(format!(
                            "JSX: {} joint q must be {}",
                            kind.name(),
                            kind.q_shape()
                        ));
                    }
                    vec![x]
                } else if let Some(l) = p_num_list(el, "q")? {
                    if l.len() != arity {
                        return Err(format!(
                            "JSX: {} joint q must be {}",
                            kind.name(),
                            kind.q_shape()
                        ));
                    }
                    l
                } else {
                    vec![0.0; arity]
                }
            }
        };
        if kind.is_1dof() {
            if let Some([lo, hi]) = limit {
                let x = q[0];
                if x < lo || x > hi {
                    return Err(format!(
                        "JSX: joint {name} q={x} outside limit [{lo}, {hi}]"
                    ));
                }
            }
        }
        let m = joint_motion(&kind, axis, &q, pitch.unwrap_or(0.0));
        let world = mat4_mul(mat4_mul(ctx, mat4_mul(translate4(at), rpy4(rpy))), m);
        let parent = self.joint_stack.last().cloned();
        self.kin.joints.push(JointDef {
            name: name.clone(),
            kind,
            axis,
            at,
            rpy,
            q,
            pitch,
            limit,
            parent,
            meshes: Vec::new(),
            world,
        });
        self.joint_stack.push(name);
        for c in &el.children {
            self.walk(c, world, mat, scene, cam)?;
        }
        self.joint_stack.pop();
        Ok(())
    }

    fn gear_el(&mut self, el: &El) -> Result<(), String> {
        let driver = p_str(el, "driver")?.ok_or_else(|| "JSX: gear needs driver".to_string())?;
        let driven = p_str(el, "driven")?.ok_or_else(|| "JSX: gear needs driven".to_string())?;
        let ratio = p_num(el, "ratio")?.ok_or_else(|| "JSX: gear needs ratio".to_string())?;
        let offset = p_num(el, "offset")?.unwrap_or(0.0);
        let dj = self
            .kin
            .joints
            .iter()
            .find(|j| j.name == driver)
            .ok_or_else(|| format!("JSX: unknown joint {driver}"))?;
        if !dj.kind.is_1dof() {
            return Err(format!(
                "JSX: gear needs 1-DOF joints, got {}",
                dj.kind.name()
            ));
        }
        if self.kin.joints.iter().any(|j| j.name == driven) {
            return Err(format!("JSX: gear must precede the driven joint {driven}"));
        }
        if self.driven_by.contains_key(&driven) {
            return Err(format!("JSX: joint {driven} is already driven"));
        }
        let rel = GearRel {
            driver,
            driven: driven.clone(),
            ratio,
            offset,
        };
        self.driven_by.insert(driven, Driven::Gear(rel.clone()));
        self.kin.gears.push(rel);
        Ok(())
    }

    fn cam_el(&mut self, el: &El) -> Result<(), String> {
        let driver = p_str(el, "driver")?.ok_or_else(|| "JSX: cam needs driver".to_string())?;
        let driven = p_str(el, "driven")?.ok_or_else(|| "JSX: cam needs driven".to_string())?;
        let prof = |key: &str| -> Result<CamProfile, String> {
            let v = prop(el, key).ok_or_else(|| format!("JSX: cam needs {key}"))?;
            let o = v
                .as_object()
                .ok_or_else(|| format!("JSX: cam {key} must be a profile object"))?;
            let kind = o
                .get("kind")
                .and_then(Value::as_str)
                .ok_or_else(|| format!("JSX: cam {key} needs kind"))?;
            let num = |k: &str| o.get(k).and_then(Value::as_f64);
            let v3 = |k: &str| -> Option<[f64; 3]> {
                o.get(k)?
                    .as_array()?
                    .iter()
                    .map(Value::as_f64)
                    .collect::<Option<Vec<_>>>()?
                    .try_into()
                    .ok()
            };
            match kind {
                "circle" => Ok(CamProfile::Circle {
                    c: v3("c").unwrap_or([0.0; 3]),
                    n: v3("n").unwrap_or([0.0, 0.0, 1.0]),
                    r: num("r").ok_or_else(|| format!("JSX: cam {key} circle needs r"))?,
                }),
                "plane" => Ok(CamProfile::Plane {
                    n: v3("n").unwrap_or([0.0, 1.0, 0.0]),
                    d: num("d").unwrap_or(0.0),
                }),
                _ => Err("JSX: cam profiles must be circle or plane".to_string()),
            }
        };
        let driver_profile = prof("driverProfile")?;
        let driven_profile = prof("drivenProfile")?;
        if !self.kin.joints.iter().any(|j| j.name == driver) {
            return Err(format!("JSX: unknown joint {driver}"));
        }
        if self.kin.joints.iter().any(|j| j.name == driven) {
            return Err(format!("JSX: cam must precede the driven joint {driven}"));
        }
        if self.driven_by.contains_key(&driven) {
            return Err(format!("JSX: joint {driven} is already driven"));
        }
        self.driven_by.insert(
            driven.clone(),
            Driven::Cam(CamRel {
                driver,
                driven,
                driver_profile,
                driven_profile,
            }),
        );
        Ok(())
    }
}

fn mat4_inv(m: [f64; 16]) -> [f64; 16] {
    let mut r = [0.0; 16];
    for i in 0..3 {
        for j in 0..3 {
            r[i * 4 + j] = m[j * 4 + i];
        }
    }
    let t = [m[3], m[7], m[11]];
    for i in 0..3 {
        r[i * 4 + 3] = -(r[i * 4] * t[0] + r[i * 4 + 1] * t[1] + r[i * 4 + 2] * t[2]);
    }
    r[15] = 1.0;
    r
}

fn parse_hex(s: &str) -> Result<Color, String> {
    let h = s.trim().trim_start_matches('#');
    let v = i32::from_str_radix(h, 16).map_err(|_| format!("JSX: bad color {s}"))?;
    Ok(Color::from_hex(v))
}

/// Evaluate a JSX scene (+ optional CSS) into a `CgsRun`.
pub fn run_jsx(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
) -> Result<crate::CgsRun, String> {
    let js = compile_jsx(jsx_src)?;
    let v = eval_js(&js)?;
    let root = to_el(&v)?;
    let rules = match css_src {
        Some(c) => parse_css(c)?,
        None => Vec::new(),
    };
    // :root / scene 规则的 background
    let mut scene = Scene::new(None);
    for r in &rules {
        if r.sel.scene {
            for (k, v) in &r.props {
                if k == "background" || k == "background-color" {
                    if let Some(CgsValue::Num(c)) = css_value_to_arg(k, v) {
                        scene.background = Color::from_hex(c as i32);
                    }
                }
            }
        }
    }
    let mut cam: Option<PerspectiveCamera> = None;
    let mut b = Builder {
        loader: SceneLoader::stub(asset_root),
        kin: Kinematics::default(),
        joint_stack: Vec::new(),
        driven_by: HashMap::new(),
        tags: TagRegistry::new(),
        pending_tags: Vec::new(),
        rules,
    };
    b.walk(
        &root,
        cga_core::mat4_identity(),
        &HashMap::new(),
        &mut scene,
        &mut cam,
    )?;
    let cam = cam.unwrap_or_else(|| {
        let mut c = PerspectiveCamera::new(
            50.0,
            16.0 / 9.0,
            0.1,
            100.0,
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        c.look_at([0.0, 0.0, 0.0], None);
        c
    });
    Ok(crate::CgsRun {
        scene,
        camera: cam,
        tags: b.tags,
        kinematics: b.kin,
    })
}

/// Headless render of a JSX scene.
pub fn render_jsx_png(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
    w: i32,
    h: i32,
    aa: i32,
) -> Result<crate::HeadlessImage, String> {
    if w <= 0 || h <= 0 {
        return Err(format!("headless: bad size {w}x{h}"));
    }
    let mut run = run_jsx(jsx_src, css_src, asset_root)?;
    run.camera.aspect = f64::from(w) / f64::from(h);
    let mut r = crate::Renderer::new(w, h, aa, 3);
    let img = r.render(run.scene, run.camera);
    Ok(crate::HeadlessImage {
        width: w,
        height: h,
        png: crate::frame_to_png_bytes(&img),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const ORBIT_JSX: &str = include_str!("../../../../examples/jsx/orbit.jsx");
    const ORBIT_CSS: &str = include_str!("../../../../examples/jsx/orbit.css");
    const ORBIT_CGS: &str = include_str!("../../../../examples/cgs/orbit.cgs");

    #[test]
    fn test_jsx_orbit_parity_with_cgs() {
        // 同一场景的两种写法必须渲染出逐字节相同的 PNG。
        let a = crate::render_cgs_png(ORBIT_CGS, "examples/cgs", 96, 72, 1).expect("cgs render");
        let b = render_jsx_png(ORBIT_JSX, Some(ORBIT_CSS), "examples/jsx", 96, 72, 1)
            .expect("jsx render");
        assert_eq!(a.png, b.png, "JSX 与 CGS 渲染必须逐字节一致");
    }

    #[test]
    fn test_jsx_component_and_control_flow() {
        let src = r#"
const Ball = ({ r, x }) => <translate t={[x, r, 0]}><sphere r={r} /></translate>;
export default (
  <scene>
    {[0, 1, 2].map(i => <Ball r={0.5} x={i * 2} />)}
    {1 + 1 === 2 ? <box s={[1, 1, 1]} /> : null}
  </scene>
);
"#;
        let run = run_jsx(src, None, "").expect("run");
        assert_eq!(run.scene.objects.len(), 4, "3 Ball + 1 box");
    }

    #[test]
    fn test_jsx_css_material() {
        let run = run_jsx(
            r#"export default <sphere r={1} class="red" />;"#,
            Some(".red { color: #C0392B; roughness: 0.25; }"),
            "",
        )
        .expect("run");
        let m = &run.scene.objects[0].material;
        assert!(
            (m.color.r - 0xC0 as f64 / 255.0).abs() < 1e-6,
            "CSS color 未生效: {:?}",
            m.color
        );
        assert!((m.roughness - 0.25).abs() < 1e-9, "CSS roughness 未生效");
    }

    #[test]
    fn test_jsx_joint_gear() {
        let run = run_jsx(
            r#"export default (
  <scene>
    <joint name="a" type="revolute" axis={[0,0,1]} q={0.4}><sphere r={0.1} /></joint>
    <gear driver="a" driven="b" ratio={-0.5} />
    <joint name="b" type="prismatic" axis={[0,0,1]}><box s={[0.1,0.1,0.1]} /></joint>
  </scene>
);"#,
            None,
            "",
        )
        .expect("run");
        let k = &run.kinematics;
        assert_eq!(k.joints.len(), 2);
        assert!((k.joints[1].q[0] + 0.2).abs() < 1e-12, "gear 推导 q_b=-0.2");
        assert_eq!(k.joints[0].meshes, vec![0], "joint a 拥有 mesh 0");
        assert_eq!(k.joints[1].meshes, vec![1], "joint b 拥有 mesh 1");
        let rep = run.scene.report(&run.camera, &run.tags, &run.kinematics);
        assert!(rep.contains("joint 0 \"a\" type=revolute"), "{rep}");
        assert!(
            rep.contains("gear 0 driver=\"a\" driven=\"b\" ratio=-0.5"),
            "{rep}"
        );
    }

    #[test]
    fn test_jsx_errors() {
        let e = run_jsx("export default <sphere", None, "").unwrap_err();
        assert!(e.starts_with("JSX line 1: "), "{e}");
        let e = run_jsx("export default <frob r={1} />;", None, "").unwrap_err();
        assert!(e.contains("unknown primitive frob"), "{e}");
        let e = run_jsx("const a = 1;", None, "").unwrap_err();
        assert_eq!(e, "JSX: scene file must end with export default <element>");
    }
}
