//! JSX + CSS scene host (docs/jsx-css-host.md: real JS via
//! boa, JSX compiled by swc, CSS parsed by lightningcss).
//!
//! Pipeline: .jsx source → swc (parse JSX → h() calls) → boa executes real
//! JS (components, map, Math, …) → element tree → scene builder (reuses the
//! geometry/material builders and the kinematics registry) → `SceneRun`.
//!
//! Conventions:
//! - The scene is the `export default <element>` value.
//! - Builtin elements are PascalCase globals bound to lowercase strings, or
//!   lowercase tags directly: `<Sphere r={1}/>` ≡ `<sphere r={1}/>`.
//! - CSS matches by tag / `.class` / `#id`; material properties inherit down
//!   the element tree; inline props beat matched CSS.

use std::collections::HashMap;

use boa_engine::Context;
use serde_json::Value;
use swc_core::common::{sync::Lrc, FileName, Globals, Mark, SourceMap, GLOBALS};
use swc_core::ecma::ast::{
    BindingIdent, Decl, Ident, Module, ModuleDecl, ModuleItem, Pat, Stmt, VarDecl, VarDeclKind,
    VarDeclarator,
};
use swc_core::ecma::codegen::{text_writer::JsWriter, Config as CodegenConfig, Emitter};
use swc_core::ecma::parser::{lexer::Lexer, EsSyntax, Parser, StringInput, Syntax};
use swc_core::ecma::transforms::base::{hygiene, resolver};
use swc_core::ecma::transforms::react::{jsx, Options as ReactOptions, Runtime};
use swc_core::ecma::visit::VisitMutWith;

use cga_core::Multivector;

use crate::scene_build::{
    cam_solve, joint_motion, rpy4, ArgValue, Builders, CamProfile, CamRel, CamSolved, Driven,
    GearRel, JointDef, JointKind, Kinematics, TagInstance, TagRegistry,
};
use cga_gpu::scene::{Object, ObjectParams, PerspectiveCamera, Scene};
use cga_gpu::scene_graph::Color;
use cga_gpu::shading::Light;

/// 场景预置：元素名常量 + 查询辅助 + `h`/`Fragment`（`scene-prelude.js`），
/// 以及运动副 React 组件 `Revolute`/`Prismatic`/`Gear`/`Cam`…（`kinematics-pairs.js`）。
/// 编译期嵌入，按顺序拼接后注入到每个模块源码最前面。
const PRELUDE: &str = concat!(
    include_str!("../../assets/scene-prelude.js"),
    include_str!("../../assets/kinematics-pairs.js"),
);

/// swc: parse a JSX module, reporting errors as `JSX line N: ...`.
fn parse_jsx(src: &str) -> Result<(Lrc<SourceMap>, Module), String> {
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
    let module = parser.parse_module().map_err(|e| {
        use swc_core::common::Spanned;
        let line = cm
            .lookup_line(e.span().lo())
            .map(|l| l.line + 1)
            .unwrap_or(0);
        format!("JSX line {line}: {}", e.kind().msg())
    })?;
    Ok((cm, module))
}

/// Print a module back to JS text (no transforms).
fn codegen(cm: &Lrc<SourceMap>, module: &Module) -> Result<String, String> {
    let mut buf = vec![];
    let mut emitter = Emitter {
        cfg: CodegenConfig::default(),
        cm: cm.clone(),
        comments: None,
        wr: JsWriter::new(cm.clone(), "", &mut buf, None),
    };
    emitter
        .emit_module(module)
        .map_err(|e| format!("JSX: codegen failed: {e}"))?;
    String::from_utf8(buf).map_err(|e| format!("JSX: codegen utf8: {e}"))
}

/// resolver → JSX→h() → hygiene → print. Per module (marks are per call).
fn emit_js(cm: &Lrc<SourceMap>, mut module: Module) -> Result<String, String> {
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
        codegen(cm, &module)
    })
}

/// A module's export surface (validated at import time, bundle-time errors).
#[derive(Clone, Default)]
struct Exports {
    has_default: bool,
    /// exported name -> local ident inside the module
    named: HashMap<String, String>,
}

/// Compile-time bundler: `import ... from './x.jsx'` recursively compiles
/// the target and wraps it as `globalThis.__exp_N = (() => { ...; return
/// {default, ...named}; })()`; the import site becomes plain alias consts.
/// Post-order emission (dependencies first), memoized (diamond imports
/// evaluate once), cycles are errors. Scopes are real JS function scopes,
/// so per-module top-level names can never collide.
struct Bundler<'a> {
    resolve: &'a mut dyn FnMut(&str) -> Result<String, String>,
    done: HashMap<String, (usize, Exports)>,
    stack: Vec<String>,
    out: String,
    next: usize,
}

impl Bundler<'_> {
    fn bundle_import(&mut self, spec: &str) -> Result<(usize, Exports), String> {
        let key = spec.strip_prefix("./").unwrap_or(spec).to_string();
        if let Some(hit) = self.done.get(&key) {
            return Ok(hit.clone());
        }
        if self.stack.contains(&key) {
            return Err(format!(
                "JSX: circular import: {} -> {key}",
                self.stack.join(" -> ")
            ));
        }
        let src = (self.resolve)(&key).map_err(|e| format!("JSX: cannot import {spec}: {e}"))?;
        self.stack.push(key.clone());
        let id = self.next;
        self.next += 1;
        let exports = self.compile_module(id, &src);
        self.stack.pop();
        let exports = exports?;
        self.done.insert(key, (id, exports.clone()));
        Ok((id, exports))
    }

    /// Compile one imported module to its `globalThis.__exp_N = ...` text.
    fn compile_module(&mut self, id: usize, src: &str) -> Result<Exports, String> {
        let (cm, module) = parse_jsx(src)?;
        let rw = self.rewrite(module.body, false)?;
        let body_text = codegen(
            &cm,
            &Module {
                body: rw.items,
                ..module
            },
        )?;
        let mut ret = String::new();
        if let Some(d) = &rw.default {
            ret.push_str(&format!("default: {d}, "));
        }
        for (exported, local) in &rw.named {
            ret.push_str(&format!("{exported}: {local}, "));
        }
        let wrapped = format!(
            "globalThis.__exp_{id} = (() => {{\n{body_text}\nreturn {{ {ret} }};\n}})();\n"
        );
        let (cm2, wrapped_mod) = parse_jsx(&wrapped)?;
        let text = emit_js(&cm2, wrapped_mod)?;
        self.out.push_str(&text);
        Ok(Exports {
            has_default: rw.default.is_some(),
            named: rw.named.into_iter().collect(),
        })
    }

    /// Rewrite import/export declarations of one module body. `is_entry`:
    /// the scene file itself (its default export becomes `__scene`).
    fn rewrite(&mut self, body: Vec<ModuleItem>, is_entry: bool) -> Result<Rewritten, String> {
        let mut items: Vec<ModuleItem> = Vec::new();
        let mut rw = Rewritten::default();
        for item in body {
            match item {
                ModuleItem::ModuleDecl(ModuleDecl::Import(imp)) => {
                    let spec = imp.src.value.to_string_lossy().into_owned();
                    if spec.ends_with(".css") {
                        continue; // css imports stay stripped
                    }
                    if !spec.ends_with(".jsx") {
                        return Err(format!("JSX: unsupported import {spec} (only .jsx/.css)"));
                    }
                    let (id, exports) = self.bundle_import(&spec)?;
                    let mut snippet = String::new();
                    for s in &imp.specifiers {
                        use swc_core::ecma::ast::ImportSpecifier as IS;
                        match s {
                            IS::Default(d) => {
                                if !exports.has_default {
                                    return Err(format!(
                                        "JSX: module {spec} has no default export"
                                    ));
                                }
                                snippet.push_str(&format!(
                                    "const {} = globalThis.__exp_{id}.default;\n",
                                    d.local.sym
                                ));
                            }
                            IS::Named(n) => {
                                let orig = match &n.imported {
                                    Some(m) => export_name(m),
                                    None => n.local.sym.to_string(),
                                };
                                if !exports.named.contains_key(&orig) {
                                    return Err(format!(
                                        "JSX: module {spec} does not export {orig}"
                                    ));
                                }
                                snippet.push_str(&format!(
                                    "const {} = globalThis.__exp_{id}.{orig};\n",
                                    n.local.sym
                                ));
                            }
                            IS::Namespace(ns) => {
                                snippet.push_str(&format!(
                                    "const {} = globalThis.__exp_{id};\n",
                                    ns.local.sym
                                ));
                            }
                        }
                    }
                    if !snippet.is_empty() {
                        let (_, alias_mod) = parse_jsx(&snippet)?;
                        items.extend(alias_mod.body);
                    }
                }
                ModuleItem::ModuleDecl(ModuleDecl::ExportDefaultExpr(e)) => {
                    rw.default = Some("__scene".into());
                    items.push(const_stmt(e.span, "__scene", *e.expr));
                }
                ModuleItem::ModuleDecl(ModuleDecl::ExportDefaultDecl(d)) => {
                    use swc_core::ecma::ast::DefaultDecl as DD;
                    match d.decl {
                        DD::Fn(f) => match f.ident {
                            Some(id) => {
                                rw.default = Some(id.sym.to_string());
                                items.push(ModuleItem::Stmt(Stmt::Decl(Decl::Fn(
                                    swc_core::ecma::ast::FnDecl {
                                        ident: id,
                                        declare: false,
                                        function: f.function,
                                    },
                                ))));
                            }
                            None => {
                                rw.default = Some("__scene".into());
                                items.push(const_stmt(
                                    d.span,
                                    "__scene",
                                    swc_core::ecma::ast::Expr::Fn(swc_core::ecma::ast::FnExpr {
                                        ident: None,
                                        function: f.function,
                                    }),
                                ));
                            }
                        },
                        DD::Class(c) => match c.ident {
                            Some(id) => {
                                rw.default = Some(id.sym.to_string());
                                items.push(ModuleItem::Stmt(Stmt::Decl(Decl::Class(
                                    swc_core::ecma::ast::ClassDecl {
                                        ident: id,
                                        declare: false,
                                        class: c.class,
                                    },
                                ))));
                            }
                            None => {
                                rw.default = Some("__scene".into());
                                items.push(const_stmt(
                                    d.span,
                                    "__scene",
                                    swc_core::ecma::ast::Expr::Class(
                                        swc_core::ecma::ast::ClassExpr {
                                            ident: None,
                                            class: c.class,
                                        },
                                    ),
                                ));
                            }
                        },
                        other => {
                            return Err(format!(
                                "JSX: unsupported export default declaration: {other:?}"
                            ))
                        }
                    }
                }
                ModuleItem::ModuleDecl(ModuleDecl::ExportDecl(d)) => {
                    for name in decl_names(&d.decl) {
                        rw.named.push((name.clone(), name));
                    }
                    items.push(ModuleItem::Stmt(Stmt::Decl(d.decl)));
                }
                ModuleItem::ModuleDecl(ModuleDecl::ExportNamed(n)) => {
                    if n.src.is_some() {
                        return Err("JSX: re-export (export ... from) is not supported".into());
                    }
                    for s in &n.specifiers {
                        use swc_core::ecma::ast::ExportSpecifier as ES;
                        match s {
                            ES::Named(ns) => {
                                let local = export_name(&ns.orig);
                                let exported = match &ns.exported {
                                    Some(m) => export_name(m),
                                    None => local.clone(),
                                };
                                rw.named.push((exported, local));
                            }
                            other => {
                                return Err(format!("JSX: unsupported export specifier: {other:?}"))
                            }
                        }
                    }
                }
                ModuleItem::ModuleDecl(ModuleDecl::ExportAll(_)) => {
                    return Err("JSX: export * is not supported".into());
                }
                other => items.push(other),
            }
        }
        if is_entry && rw.default.is_none() {
            return Err("JSX: scene file must end with export default <element>".into());
        }
        rw.items = items;
        Ok(rw)
    }
}

#[derive(Default)]
struct Rewritten {
    items: Vec<ModuleItem>,
    default: Option<String>,
    /// (exported name, local ident)
    named: Vec<(String, String)>,
}

fn export_name(m: &swc_core::ecma::ast::ModuleExportName) -> String {
    use swc_core::ecma::ast::ModuleExportName as MEN;
    match m {
        MEN::Ident(i) => i.sym.to_string(),
        MEN::Str(s) => s.value.to_string_lossy().into_owned(),
    }
}

/// Top-level binding names introduced by a declaration (idents only).
fn decl_names(decl: &Decl) -> Vec<String> {
    let mut out = Vec::new();
    match decl {
        Decl::Fn(f) => out.push(f.ident.sym.to_string()),
        Decl::Class(c) => out.push(c.ident.sym.to_string()),
        Decl::Var(v) => {
            for d in &v.decls {
                collect_pat_idents(&d.name, &mut out);
            }
        }
        _ => {}
    }
    out
}

fn collect_pat_idents(pat: &Pat, out: &mut Vec<String>) {
    match pat {
        Pat::Ident(b) => out.push(b.id.sym.to_string()),
        Pat::Array(a) => {
            for e in a.elems.iter().flatten() {
                collect_pat_idents(e, out);
            }
        }
        Pat::Object(o) => {
            for p in &o.props {
                use swc_core::ecma::ast::ObjectPatProp as OPP;
                match p {
                    OPP::KeyValue(kv) => collect_pat_idents(&kv.value, out),
                    OPP::Assign(a) => out.push(a.key.sym.to_string()),
                    OPP::Rest(r) => collect_pat_idents(&r.arg, out),
                }
            }
        }
        Pat::Assign(a) => collect_pat_idents(&a.left, out),
        Pat::Rest(r) => collect_pat_idents(&r.arg, out),
        _ => {}
    }
}

/// `const <name> = <init>;`
fn const_stmt(
    span: swc_core::common::Span,
    name: &str,
    init: swc_core::ecma::ast::Expr,
) -> ModuleItem {
    ModuleItem::Stmt(Stmt::Decl(Decl::Var(Box::new(VarDecl {
        span,
        ctxt: Default::default(),
        kind: VarDeclKind::Const,
        declare: false,
        decls: vec![VarDeclarator {
            span,
            name: Pat::Ident(BindingIdent {
                id: Ident::new_no_ctxt(name.into(), span),
                type_ann: None,
            }),
            init: Some(Box::new(init)),
            definite: false,
        }],
    }))))
}

/// Compile entry JSX + transitively imported `.jsx` modules into one JS
/// body. `resolve` maps an import specifier (`./dial.jsx`; the `./` is
/// stripped) to its source text. `.css` imports are ignored as before.
/// Cycles, missing modules and missing exports are `JSX:`-prefixed errors.
pub fn compile_jsx_with(
    src: &str,
    resolve: &mut dyn FnMut(&str) -> Result<String, String>,
) -> Result<String, String> {
    let mut b = Bundler {
        resolve,
        done: HashMap::new(),
        stack: Vec::new(),
        out: String::new(),
        next: 1,
    };
    let (cm, module) = parse_jsx(src)?;
    let rw = b.rewrite(module.body, true)?;
    let entry = emit_js(
        &cm,
        Module {
            body: rw.items,
            ..module
        },
    )?;
    Ok(format!("{}{entry}", b.out))
}

/// Disk resolver for `.jsx` imports, rooted at `asset_root`.
fn disk_resolver(asset_root: &str) -> impl FnMut(&str) -> Result<String, String> + '_ {
    move |spec| {
        let key = spec.strip_prefix("./").unwrap_or(spec);
        std::fs::read_to_string(format!("{asset_root}/{key}"))
            .map_err(|e| format!("cannot read {key}: {e}"))
    }
}

/// boa host: `solve([x0, ...], [v => residual, ...])` — Levenberg-damped
/// Gauss–Newton with a numeric Jacobian (JSX-side constrain). Residuals come
/// from prelude helpers: eq/le/ge.
fn solve_host(
    _: &boa_engine::JsValue,
    args: &[boa_engine::JsValue],
    ctx: &mut Context,
) -> boa_engine::JsResult<boa_engine::JsValue> {
    use boa_engine::value::TryIntoJs;
    use boa_engine::{js_string, JsNativeError};
    let bad = |m: &str| JsNativeError::typ().with_message(m.to_string());
    let read_f64_array =
        |v: &boa_engine::JsValue, ctx: &mut Context| -> Result<Vec<f64>, boa_engine::JsError> {
            let o = v
                .as_object()
                .ok_or_else(|| boa_engine::JsError::from(bad("JSX: solve needs arrays")))?;
            let n = o.get(js_string!("length"), ctx)?.to_number(ctx)? as usize;
            let mut out = Vec::with_capacity(n);
            for i in 0..n {
                out.push(o.get(i, ctx)?.to_number(ctx)?);
            }
            Ok(out)
        };
    let init = read_f64_array(
        args.first()
            .ok_or_else(|| bad("JSX: solve needs (init array, residuals)"))?,
        ctx,
    )
    .map_err(|e| e)?;
    let fns: Vec<boa_engine::JsObject> = match args.get(1).and_then(|v| v.as_object()) {
        Some(o) => {
            let n = o.get(js_string!("length"), ctx)?.to_number(ctx)? as usize;
            let mut out = Vec::with_capacity(n);
            for i in 0..n {
                let f = o.get(i, ctx)?;
                out.push(
                    f.as_callable()
                        .ok_or_else(|| bad("JSX: solve residual is not a function"))?,
                );
            }
            out
        }
        None => return Err(bad("JSX: solve residuals must be an array of functions").into()),
    };
    let n = init.len();
    let m = fns.len();
    let eval = |v: &[f64], ctx: &mut Context| -> Result<Vec<f64>, boa_engine::JsError> {
        let arr = v.to_vec().try_into_js(ctx)?;
        fns.iter()
            .map(|f| {
                f.call(&boa_engine::JsValue::undefined(), &[arr.clone()], ctx)
                    .and_then(|r| r.to_number(ctx))
            })
            .collect()
    };
    let mut v = init;
    let mut r = eval(&v, ctx)?;
    let mut lambda = 1e-6f64;
    for _ in 0..100 {
        let rmax = r.iter().fold(0.0f64, |a, b| a.max(b.abs()));
        if rmax < 1e-9 {
            return Ok(v.clone().try_into_js(ctx)?);
        }
        // 数值雅可比
        let mut j = vec![vec![0.0f64; n]; m];
        for c in 0..n {
            let h = 1e-7 * (1.0 + v[c].abs());
            let mut vp = v.clone();
            vp[c] += h;
            let rp = eval(&vp, ctx)?;
            for (row, jr) in j.iter_mut().enumerate() {
                jr[c] = (rp[row] - r[row]) / h;
            }
        }
        // Levenberg：试步 → 变好接受，变差加阻尼重试
        let mut accepted = false;
        for _ in 0..20 {
            // 解 (JᵀJ + λI) δ = −Jᵀr
            let mut a = vec![vec![0.0f64; n]; n];
            let mut b = vec![0.0f64; n];
            for row in 0..m {
                for c1 in 0..n {
                    b[c1] -= j[row][c1] * r[row];
                    for c2 in 0..n {
                        a[c1][c2] += j[row][c1] * j[row][c2];
                    }
                }
            }
            for c in 0..n {
                a[c][c] += lambda;
            }
            // 高斯消元
            let mut ok = true;
            for c in 0..n {
                let mut piv = c;
                for rr in c + 1..n {
                    if a[rr][c].abs() > a[piv][c].abs() {
                        piv = rr;
                    }
                }
                if a[piv][c].abs() < 1e-18 {
                    ok = false;
                    break;
                }
                a.swap(c, piv);
                b.swap(c, piv);
                for rr in c + 1..n {
                    let f = a[rr][c] / a[c][c];
                    for cc in c..n {
                        a[rr][cc] -= f * a[c][cc];
                    }
                    b[rr] -= f * b[c];
                }
            }
            if !ok {
                lambda *= 4.0;
                continue;
            }
            let mut d = vec![0.0f64; n];
            for c in (0..n).rev() {
                let mut s = b[c];
                for cc in c + 1..n {
                    s -= a[c][cc] * d[cc];
                }
                d[c] = s / a[c][c];
            }
            let vt: Vec<f64> = v.iter().zip(&d).map(|(x, dx)| x + dx).collect();
            let rt = eval(&vt, ctx)?;
            let rmax_t = rt.iter().fold(0.0f64, |a2, b2| a2.max(b2.abs()));
            if rmax_t < rmax {
                v = vt;
                r = rt;
                lambda = (lambda / 4.0).max(1e-12);
                accepted = true;
                break;
            }
            lambda *= 4.0;
            if !lambda.is_finite() || lambda > 1e12 {
                break;
            }
        }
        if !accepted {
            break;
        }
    }
    Err(JsNativeError::typ()
        .with_message("JSX: solve did not converge".to_string())
        .into())
}

/// boa executes the compiled module; the scene element tree comes back as
/// JSON (all prop values are numbers/strings/bools/arrays/objects).
#[derive(Clone, Debug)]
pub struct El {
    pub tag: String,
    pub props: HashMap<String, Value>,
    pub children: Vec<El>,
    /// 自身提交版本（快照 `__v`）：创建 / props 更新 / 子节点增删移动时递增。
    /// 子树版本见 [`El::s`]。0 = 无版本（嵌套在 props 里的元素树），增量构建
    /// 永不复用无版本子树。
    pub v: u64,
    /// 子树版本（快照 `__s`）：自身与全部后代 `v` 的最大值。
    pub s: u64,
    /// 增量构建缓存：本子树最近一帧产出的 `scene.objects` 区间。只在会话缓存的
    /// 上一帧树上填充；新帧由 `annotate_reuse` 标注后，`walk` 据此整棵复用。
    pub cache: Option<std::ops::Range<usize>>,
}

fn to_el(v: &Value) -> Result<El, String> {
    let obj = v
        .as_object()
        .ok_or_else(|| format!("JSX: scene is not an element: {v}"))?;
    let tag = obj.get("t").and_then(Value::as_str).ok_or_else(|| {
        let t = v.to_string();
        let t = if t.len() > 240 {
            format!("{}…", &t[..240])
        } else {
            t
        };
        format!("JSX: element without tag: {t}")
    })?;
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
    let v = obj.get("__v").and_then(Value::as_u64).unwrap_or(0);
    let s = obj.get("__s").and_then(Value::as_u64).unwrap_or(0);
    Ok(El {
        tag: tag.to_string(),
        props,
        children,
        v,
        s,
        cache: None,
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

fn to_arg(v: &Value) -> ArgValue {
    match v {
        Value::Number(n) => ArgValue::Num(n.as_f64().unwrap_or(0.0)),
        Value::Bool(b) => ArgValue::Bool(*b),
        Value::String(s) => ArgValue::Str(s.clone()),
        Value::Array(a) => ArgValue::List(a.iter().map(to_arg).collect()),
        _ => ArgValue::Num(0.0),
    }
}

const MATERIAL_KEYS: [&str; 9] = [
    "color",
    "roughness",
    "metalness",
    "emissive",
    "opacity",
    "ior",
    "absorption",
    "map",
    "unlit",
];

// ---- CSS ----
//
// 解析交给 lightningcss（语法与值的归一化都是真 CSS）；这里只做三件事：
// 选择器编译、级联、值转换。差距清单与阶段见 docs/css-conformance.md。

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
struct Specificity(u16, u16, u16, u16);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Combinator {
    Descendant,
    Child,
    NextSibling,
    LaterSibling,
}

/// 一个复合选择器：type / `.class` / `#id` / `[attr]` / `:root` / `*` 的组合。
#[derive(Clone, Debug, Default)]
struct Compound {
    /// `:root` 或裸 `scene`：只匹配深度为 0 的根元素。
    root: bool,
    tag: Option<String>,
    classes: Vec<String>,
    id: Option<String>,
    attrs: Vec<(String, Option<String>)>,
}

/// 复杂选择器。`parts` 最右是匹配目标；`parts[i].0` 描述它与左侧
/// （祖先或前兄弟）之间的连接。
#[derive(Clone, Debug)]
struct ComplexSelector {
    parts: Vec<(Combinator, Compound)>,
    specificity: Specificity,
    /// 目标复合选择器带根标记 → 根作用域规则（背景等由 `build_scene_run` 消费）。
    scene_scope: bool,
    text: String,
}

#[derive(Clone, Debug)]
struct StyleDecl {
    name: String,
    value: String,
    important: bool,
}

#[derive(Clone, Debug)]
struct StyleRule {
    sel: ComplexSelector,
    decls: Vec<StyleDecl>,
}

fn css_err(what: &str, msg: &str) -> String {
    format!("CSS: {what}: {msg}")
}

/// lightningcss 会把 `::before` 归一成 `:before`，所以伪元素按名字识别。
const PSEUDO_ELEMENT_NAMES: &[&str] = &["before", "after", "first-line", "first-letter"];

fn skip_ws(b: &[u8], i: &mut usize) -> bool {
    let start = *i;
    while *i < b.len() && b[*i].is_ascii_whitespace() {
        *i += 1;
    }
    *i > start
}

fn scan_ident(b: &[u8], i: &mut usize) -> Option<String> {
    let start = *i;
    while *i < b.len() {
        let c = b[*i];
        if c.is_ascii_alphanumeric() || c == b'-' || c == b'_' || c >= 0x80 {
            *i += 1;
        } else {
            break;
        }
    }
    if *i == start {
        None
    } else {
        Some(String::from_utf8_lossy(&b[start..*i]).into_owned())
    }
}

fn parse_attr(b: &[u8], i: &mut usize, sel: &str, c: &mut Compound) -> Result<(), String> {
    *i += 1; // '['
    skip_ws(b, i);
    let name =
        scan_ident(b, i).ok_or_else(|| css_err(sel, "expected an attribute name after '['"))?;
    skip_ws(b, i);
    if *i < b.len() && b[*i] == b'=' {
        *i += 1;
        skip_ws(b, i);
        if *i >= b.len() {
            return Err(css_err(sel, "unterminated attribute selector"));
        }
        let value = if b[*i] == b'"' || b[*i] == b'\'' {
            let quote = b[*i];
            *i += 1;
            let start = *i;
            while *i < b.len() && b[*i] != quote {
                *i += 1;
            }
            if *i >= b.len() {
                return Err(css_err(sel, "unterminated attribute value"));
            }
            let v = String::from_utf8_lossy(&b[start..*i]).into_owned();
            *i += 1;
            v
        } else {
            let start = *i;
            while *i < b.len() && b[*i] != b']' {
                *i += 1;
            }
            String::from_utf8_lossy(&b[start..*i]).trim().to_string()
        };
        skip_ws(b, i);
        if *i >= b.len() || b[*i] != b']' {
            return Err(css_err(
                sel,
                "unsupported attribute operator: only [attr] and [attr=value] are supported",
            ));
        }
        *i += 1;
        c.attrs.push((name, Some(value)));
    } else if *i < b.len() && b[*i] == b']' {
        *i += 1;
        c.attrs.push((name, None));
    } else {
        return Err(css_err(
            sel,
            "unsupported attribute selector or operator: only [attr] and [attr=value] are supported",
        ));
    }
    Ok(())
}

/// 把 lightningcss 归一化过的（逗号分隔的）单条选择器编译成结构化的
/// `ComplexSelector`。无法识别的构造一律报错，不做静默降级。
fn parse_selector(text: &str) -> Result<ComplexSelector, String> {
    let s = text.trim();
    if s == ":root" || s == "scene" {
        let specificity = if s == ":root" {
            Specificity(0, 1, 0, 0)
        } else {
            Specificity(0, 0, 1, 0)
        };
        return Ok(ComplexSelector {
            parts: vec![(
                Combinator::Descendant,
                Compound {
                    root: true,
                    ..Compound::default()
                },
            )],
            specificity,
            scene_scope: true,
            text: s.to_string(),
        });
    }

    let b = s.as_bytes();
    let mut i = 0usize;
    let mut parts: Vec<(Combinator, Compound)> = Vec::new();
    let mut specificity = Specificity::default();
    let mut pending = Combinator::Descendant;
    skip_ws(b, &mut i);
    if i >= b.len() {
        return Err(css_err(s, "empty selector"));
    }

    loop {
        // 一个复合选择器
        let mut c = Compound::default();
        let mut any = false;
        loop {
            if i >= b.len() {
                break;
            }
            match b[i] {
                b'.' => {
                    i += 1;
                    let n = scan_ident(b, &mut i)
                        .ok_or_else(|| css_err(s, "expected a class name after '.'"))?;
                    c.classes.push(n);
                    specificity.1 += 1;
                    any = true;
                }
                b'#' => {
                    i += 1;
                    let n = scan_ident(b, &mut i)
                        .ok_or_else(|| css_err(s, "expected an id after '#'"))?;
                    c.id = Some(n);
                    specificity.0 += 1;
                    any = true;
                }
                b'*' => {
                    i += 1;
                    any = true;
                }
                b'[' => {
                    parse_attr(b, &mut i, s, &mut c)?;
                    specificity.1 += 1;
                    any = true;
                }
                b':' => {
                    if i + 1 < b.len() && b[i + 1] == b':' {
                        return Err(css_err(s, "pseudo-elements (::) are not supported"));
                    }
                    i += 1;
                    let n = scan_ident(b, &mut i)
                        .ok_or_else(|| css_err(s, "expected a pseudo-class name after ':'"))?;
                    if n.eq_ignore_ascii_case("root") {
                        c.root = true;
                        specificity.1 += 1;
                    } else if PSEUDO_ELEMENT_NAMES
                        .iter()
                        .any(|p| p.eq_ignore_ascii_case(&n))
                    {
                        // lightningcss 会把 `::before` 归一成 `:before`，这里按名字识别。
                        return Err(css_err(s, &format!("pseudo-element :{n} is not supported")));
                    } else {
                        return Err(css_err(
                            s,
                            &format!("unsupported pseudo-class :{n}: only :root is supported"),
                        ));
                    }
                    any = true;
                }
                b'>' | b'+' | b'~' => break,
                ch if ch.is_ascii_whitespace() => break,
                _ => {
                    if c.root
                        || c.tag.is_some()
                        || !c.classes.is_empty()
                        || c.id.is_some()
                        || !c.attrs.is_empty()
                    {
                        return Err(css_err(
                            s,
                            "a type selector must come first in a compound selector",
                        ));
                    }
                    let n =
                        scan_ident(b, &mut i).ok_or_else(|| css_err(s, "expected a type name"))?;
                    c.tag = Some(n.to_ascii_lowercase());
                    specificity.2 += 1;
                    any = true;
                }
            }
        }
        if !any {
            return Err(css_err(s, "expected a selector"));
        }
        parts.push((pending, c));

        let saw_ws = skip_ws(b, &mut i);
        if i >= b.len() {
            break;
        }
        match b[i] {
            b'>' => {
                i += 1;
                pending = Combinator::Child;
            }
            b'+' => {
                i += 1;
                pending = Combinator::NextSibling;
            }
            b'~' => {
                i += 1;
                pending = Combinator::LaterSibling;
            }
            _ => {
                if !saw_ws {
                    return Err(css_err(s, "expected a combinator between selectors"));
                }
                pending = Combinator::Descendant;
            }
        }
        skip_ws(b, &mut i);
        if i >= b.len() {
            return Err(css_err(s, "dangling combinator at the end of the selector"));
        }
    }

    let scene_scope = parts.last().map(|(_, c)| c.root).unwrap_or(false);
    Ok(ComplexSelector {
        parts,
        specificity,
        scene_scope,
        text: s.to_string(),
    })
}

/// 遍历时为选择器匹配维护的位置。`stack` 从根元素到当前元素，
/// `stack[d].el` 是深度 `d` 的元素，`stack[d].index` 是它在其父节点中的下标。
#[derive(Clone, Copy)]
struct Frame<'a> {
    el: &'a El,
    index: usize,
}

impl<'a> Frame<'a> {
    fn new(el: &'a El, index: usize) -> Self {
        Frame { el, index }
    }
}

/// 匹配过程中的任意节点：可能不在 `stack` 上（兄弟节点），所以自带 `el`。
#[derive(Clone, Copy)]
struct Node<'a> {
    el: &'a El,
    depth: usize,
    index: usize,
}

fn sel_matches<'a>(sel: &ComplexSelector, stack: &[Frame<'a>]) -> bool {
    let Some(last) = stack.last() else {
        return false;
    };
    let cur = Node {
        el: last.el,
        depth: stack.len() - 1,
        index: last.index,
    };
    match_from(sel, stack, sel.parts.len() - 1, &cur)
}

/// 从右向左匹配：`parts[i]` 已经落在 `n` 上，再向左找 `parts[i-1]`。
fn match_from<'a>(sel: &ComplexSelector, stack: &[Frame<'a>], i: usize, n: &Node<'a>) -> bool {
    if !compound_matches(&sel.parts[i].1, n.el, n.depth) {
        return false;
    }
    if i == 0 {
        return true;
    }
    let target = i - 1;
    match sel.parts[i].0 {
        Combinator::Child => {
            parent_node(n, stack).is_some_and(|p| match_from(sel, stack, target, &p))
        }
        Combinator::Descendant => (0..n.depth).rev().any(|d| {
            let a = Node {
                el: stack[d].el,
                depth: d,
                index: stack[d].index,
            };
            match_from(sel, stack, target, &a)
        }),
        Combinator::NextSibling => {
            prev_sibling(n, stack).is_some_and(|p| match_from(sel, stack, target, &p))
        }
        Combinator::LaterSibling => prev_siblings(n, stack)
            .into_iter()
            .any(|p| match_from(sel, stack, target, &p)),
    }
}

fn parent_node<'a>(n: &Node<'a>, stack: &[Frame<'a>]) -> Option<Node<'a>> {
    if n.depth == 0 {
        return None;
    }
    Some(Node {
        el: stack[n.depth - 1].el,
        depth: n.depth - 1,
        index: stack[n.depth - 1].index,
    })
}

fn prev_sibling<'a>(n: &Node<'a>, stack: &[Frame<'a>]) -> Option<Node<'a>> {
    if n.depth == 0 || n.index == 0 {
        return None;
    }
    let parent = stack[n.depth - 1].el;
    Some(Node {
        el: &parent.children[n.index - 1],
        depth: n.depth,
        index: n.index - 1,
    })
}

fn prev_siblings<'a>(n: &Node<'a>, stack: &[Frame<'a>]) -> Vec<Node<'a>> {
    if n.depth == 0 {
        return Vec::new();
    }
    let parent = stack[n.depth - 1].el;
    (0..n.index)
        .rev()
        .map(|k| Node {
            el: &parent.children[k],
            depth: n.depth,
            index: k,
        })
        .collect()
}

fn compound_matches(c: &Compound, el: &El, depth: usize) -> bool {
    if c.root && depth != 0 {
        return false;
    }
    if let Some(tag) = &c.tag {
        if !el.tag.eq_ignore_ascii_case(tag) {
            return false;
        }
    }
    if let Some(id) = &c.id {
        match el.props.get("id").and_then(prop_scalar) {
            Some(got) if got == *id => {}
            _ => return false,
        }
    }
    if !c.classes.is_empty() {
        let mine = el_classes(el);
        for cl in &c.classes {
            if !mine.iter().any(|m| m == cl) {
                return false;
            }
        }
    }
    for (key, want) in &c.attrs {
        match el.props.get(key) {
            None | Some(Value::Null) => return false,
            Some(v) => {
                if let Some(want) = want {
                    match prop_scalar(v) {
                        Some(got) if got == *want => {}
                        _ => return false,
                    }
                }
            }
        }
    }
    true
}

fn el_classes(el: &El) -> Vec<String> {
    let raw = el.props.get("class").or_else(|| el.props.get("className"));
    match raw {
        Some(Value::String(s)) => s.split_whitespace().map(str::to_string).collect(),
        _ => Vec::new(),
    }
}

/// 属性选择器里把标量属性变成可比较的字符串。
fn prop_scalar(v: &Value) -> Option<String> {
    match v {
        Value::String(s) => Some(s.clone()),
        Value::Bool(b) => Some(if *b { "true" } else { "false" }.to_string()),
        Value::Number(n) => {
            if let Some(i) = n.as_i64() {
                Some(i.to_string())
            } else if let Some(u) = n.as_u64() {
                Some(u.to_string())
            } else {
                let f = n.as_f64()?;
                if f.fract() == 0.0 && f.abs() < 1e15 {
                    Some(format!("{}", f as i64))
                } else {
                    Some(format!("{f}"))
                }
            }
        }
        _ => None,
    }
}

fn parse_css(src: &str) -> Result<Vec<StyleRule>, String> {
    let ss = lightningcss::stylesheet::StyleSheet::parse(
        src,
        lightningcss::stylesheet::ParserOptions::default(),
    )
    .map_err(|e| css_err("", &format!("syntax error: {e}")))?;
    let mut rules = Vec::new();
    for rule in &ss.rules.0 {
        let lightningcss::rules::CssRule::Style(style) = rule else {
            let kind = match rule {
                lightningcss::rules::CssRule::Media(_) => "@media",
                lightningcss::rules::CssRule::Import(_) => "@import",
                lightningcss::rules::CssRule::Keyframes(_) => "@keyframes",
                lightningcss::rules::CssRule::Supports(_) => "@supports",
                lightningcss::rules::CssRule::Nesting(_) => "a nested rule",
                lightningcss::rules::CssRule::FontFace(_) => "@font-face",
                _ => "an at-rule",
            };
            return Err(css_err(
                kind,
                "not supported: only plain style rules are supported here",
            ));
        };
        let sel_text = style.selectors.to_string();
        if !style.rules.0.is_empty() {
            return Err(css_err(
                &sel_text,
                "CSS nesting is not supported: only flat style rules are supported",
            ));
        }
        let mut decls = Vec::new();
        for (list, important) in [
            (&style.declarations.declarations, false),
            (&style.declarations.important_declarations, true),
        ] {
            for d in list {
                let name = d.property_id().name().to_string();
                // 序列化失败不能静默丢（docs/css-conformance.md §7）。
                let value = d
                    .value_to_css_string(lightningcss::printer::PrinterOptions::default())
                    .map_err(|e| css_err(&sel_text, &format!("cannot serialize `{name}`: {e}")))?;
                decls.push(StyleDecl {
                    name,
                    value,
                    important,
                });
            }
        }
        for part in split_selector_list(&sel_text) {
            let sel = parse_selector(&part)?;
            rules.push(StyleRule {
                sel,
                decls: decls.clone(),
            });
        }
    }
    Ok(rules)
}

/// 按顶层逗号切分选择器列表：属性/函数里的逗号（`[class="a,b"]`、`rgb(1,2,3)`）不算。
fn split_selector_list(s: &str) -> Vec<String> {
    let b = s.as_bytes();
    let mut out = Vec::new();
    let mut start = 0usize;
    let mut quote = 0u8;
    let mut depth = 0usize;
    for (i, &c) in b.iter().enumerate() {
        if quote != 0 {
            if c == quote && b.get(i.wrapping_sub(1)) != Some(&b'\\') {
                quote = 0;
            }
            continue;
        }
        match c {
            b'"' | b'\'' => quote = c,
            b'[' | b'(' => depth += 1,
            b']' | b')' => depth = depth.saturating_sub(1),
            b',' if depth == 0 => {
                out.push(s[start..i].trim().to_string());
                start = i + 1;
            }
            _ => {}
        }
    }
    out.push(s[start..].trim().to_string());
    out
}

// -- 级联 --
//
// 层级由低到高：继承 < 样式表 < 内联 prop < 样式表 !important < 内联 !important。
// 同一层内按特异性、再按源码顺序。
fn cascade_rank(important: bool, inline: bool) -> u8 {
    match (important, inline) {
        (false, false) => 1,
        (false, true) => 2,
        (true, false) => 3,
        (true, true) => 4,
    }
}

enum PendingValue {
    Css(String),
    Json(Value),
}

struct PendingDecl {
    rank: u8,
    specificity: Specificity,
    order: usize,
    key: String,
    /// 选择器原文，只用于错误信息。
    source: String,
    value: PendingValue,
}

// -- 值 --

const CSS_UNITS: &[&str] = &[
    "px", "em", "rem", "ex", "ch", "vh", "vw", "vmin", "vmax", "pt", "pc", "in", "cm", "mm", "q",
    "deg", "rad", "grad", "turn", "s", "ms", "hz", "khz", "dpi", "dpcm", "dppx", "fr",
];

fn css_number(raw: &str) -> Result<f64, String> {
    let v = raw.trim();
    for f in ["calc(", "min(", "max(", "clamp("] {
        if v.starts_with(f) {
            return Err(format!(
                "`{v}` is not supported: material values must be plain numbers"
            ));
        }
    }
    if let Some(p) = v.strip_suffix('%') {
        let n = p
            .trim()
            .parse::<f64>()
            .map_err(|_| format!("`{v}` is not a percentage"))?;
        return Ok(n / 100.0);
    }
    for u in CSS_UNITS {
        if let Some(p) = v.strip_suffix(u) {
            if p.chars()
                .last()
                .map(|c| c.is_ascii_digit() || c == '.')
                .unwrap_or(false)
            {
                return Err(format!(
                    "`{v}` has a unit: material values are unitless (write a plain number)"
                ));
            }
        }
    }
    v.parse::<f64>()
        .map_err(|_| format!("`{v}` is not a number"))
}

fn css_bool(raw: &str) -> Result<bool, String> {
    match raw.trim().to_ascii_lowercase().as_str() {
        "true" | "yes" | "on" | "1" => Ok(true),
        "false" | "no" | "off" | "0" => Ok(false),
        v => Err(format!("`{v}` is not a boolean")),
    }
}

/// 颜色 → `(0xRRGGBB, alpha ∈ [0,1])`。named / hex / rgb / hsl / hwb 全都由
/// lightningcss 解析，所以判据是 CSS 本身而不是我们自己的正则。
fn css_color(raw: &str) -> Result<(f64, f64), String> {
    use lightningcss::traits::Parse;
    let text = raw.trim();
    let color = lightningcss::values::color::CssColor::parse_string(text)
        .map_err(|_| format!("`{text}` is not a valid color"))?;
    match color {
        lightningcss::values::color::CssColor::RGBA(rgba) => {
            let hex =
                (u32::from(rgba.red) << 16) | (u32::from(rgba.green) << 8) | u32::from(rgba.blue);
            Ok((f64::from(hex), f64::from(rgba.alpha) / 255.0))
        }
        lightningcss::values::color::CssColor::CurrentColor => {
            Err("`currentcolor` is not supported here".to_string())
        }
        _ => Err(format!("unsupported color space in `{text}`")),
    }
}

fn css_value(name: &str, raw: &str) -> Result<ArgValue, String> {
    match name {
        "color" | "emissive" => {
            let (hex, _) = css_color(raw)?;
            Ok(ArgValue::Num(hex))
        }
        "map" => {
            let v = raw.trim();
            let p = v
                .strip_prefix("url(")
                .and_then(|s| s.strip_suffix(')'))
                .unwrap_or(v);
            Ok(ArgValue::Str(
                p.trim().trim_matches('"').trim_matches('\'').to_string(),
            ))
        }
        "unlit" => Ok(ArgValue::Bool(css_bool(raw)?)),
        _ => css_number(raw).map(ArgValue::Num),
    }
}

/// `var(--x)` / `var(--x, fallback)` 文本替换。未定义且没有 fallback → 报错。
fn var_substitute(raw: &str, customs: &HashMap<String, String>) -> Result<String, String> {
    if !raw.contains("var(") {
        return Ok(raw.to_string());
    }
    let mut out = String::new();
    let mut rest = raw;
    while let Some(p) = rest.find("var(") {
        out.push_str(&rest[..p]);
        let after = &rest[p + 4..];
        let bytes = after.as_bytes();
        let mut depth = 1usize;
        let mut j = 0usize;
        while j < bytes.len() {
            match bytes[j] {
                b'(' => depth += 1,
                b')' => {
                    depth -= 1;
                    if depth == 0 {
                        break;
                    }
                }
                _ => {}
            }
            j += 1;
        }
        if j >= bytes.len() {
            return Err("unterminated var()".to_string());
        }
        let inner = &after[..j];
        // fallback = 顶层第一个逗号之后的内容
        let mut k = 0usize;
        let mut d2 = 0usize;
        let ib = inner.as_bytes();
        while k < ib.len() {
            match ib[k] {
                b'(' => d2 += 1,
                b')' => d2 = d2.saturating_sub(1),
                b',' if d2 == 0 => break,
                _ => {}
            }
            k += 1;
        }
        let (name, fallback) = if k < ib.len() {
            (&inner[..k], Some(&inner[k + 1..]))
        } else {
            (inner, None)
        };
        let name = name.trim();
        if !name.starts_with("--") {
            return Err(format!("var({name}) must be a custom property (--name)"));
        }
        let value = match customs.get(name) {
            Some(v) => v.clone(),
            None => match fallback {
                Some(f) => var_substitute(f.trim(), customs)?,
                None => return Err(format!("var({name}) is not defined")),
            },
        };
        out.push_str(&value);
        rest = &after[j + 1..];
    }
    out.push_str(rest);
    Ok(out)
}

// ---- scene builder ----

struct Builder<'p> {
    loader: Builders,
    kin: Kinematics,
    joint_stack: Vec<String>,
    driven_by: HashMap<String, Driven>,
    tags: TagRegistry,
    pending_tags: Vec<(String, [f64; 16])>,
    rules: Vec<StyleRule>,
    pose: HashMap<String, f64>,
    /// 分组注册表：id → 名字，0 = "" 未分组。append-only，跨帧稳定（复用的对象带旧 id）。
    groups: Vec<String>,
    group_stack: Vec<u32>,
    /// 增量构建：上一帧的对象表（`annotate_reuse` 标注的复用区间从这里克隆）。
    prev_objects: Option<&'p [Object]>,
    /// 本帧每个节点的产出区间（构建结束后写回缓存树，供下一帧复用）。
    ranges: HashMap<*const El, std::ops::Range<usize>>,
    reused_subtrees: usize,
    reused_objects: usize,
}

fn mat4_mul(a: [f64; 16], b: [f64; 16]) -> [f64; 16] {
    cga_core::mat4_mul(a, b)
}

fn translate4(t: [f64; 3]) -> [f64; 16] {
    [
        1.0, 0.0, 0.0, t[0], 0.0, 1.0, 0.0, t[1], 0.0, 0.0, 1.0, t[2], 0.0, 0.0, 0.0, 1.0,
    ]
}

impl<'p> Builder<'p> {
    /// 元素的计算样式：先继承，再按级联叠加本元素命中的样式表声明与内联 prop。
    /// 只继承材质键与自定义属性（`--*`），见 docs/css-conformance.md P2。
    fn style_for(
        &self,
        el: &El,
        stack: &[Frame<'_>],
        inherited: &HashMap<String, ArgValue>,
    ) -> Result<HashMap<String, ArgValue>, String> {
        let mut out: HashMap<String, ArgValue> = HashMap::new();
        let mut customs: HashMap<String, String> = HashMap::new();
        for k in MATERIAL_KEYS {
            if let Some(v) = inherited.get(k) {
                out.insert((*k).to_string(), v.clone());
            }
        }
        for (k, v) in inherited {
            if k.starts_with("--") {
                out.insert(k.clone(), v.clone());
                if let ArgValue::Str(s) = v {
                    customs.insert(k.clone(), s.clone());
                }
            }
        }

        let mut pending: Vec<PendingDecl> = Vec::new();
        for (order, rule) in self.rules.iter().enumerate() {
            if !sel_matches(&rule.sel, stack) {
                continue;
            }
            for d in &rule.decls {
                // 未知属性静默忽略，与浏览器一致；已知属性取到坏值才会报错。
                if !(d.name.starts_with("--") || MATERIAL_KEYS.contains(&d.name.as_str())) {
                    continue;
                }
                let rank = cascade_rank(d.important, false);
                let specificity = rule.sel.specificity;
                let source = rule.sel.text.clone();
                pending.push(PendingDecl {
                    rank,
                    specificity,
                    order,
                    key: d.name.clone(),
                    source: source.clone(),
                    value: PendingValue::Css(d.value.clone()),
                });
                // `--roughness` 这类：既是自定义属性，也作为材质键别名写入。
                if let Some(alias) = d.name.strip_prefix("--") {
                    if MATERIAL_KEYS.contains(&alias) {
                        pending.push(PendingDecl {
                            rank,
                            specificity,
                            order,
                            key: alias.to_string(),
                            source,
                            value: PendingValue::Css(d.value.clone()),
                        });
                    }
                }
            }
        }
        for (order, k) in MATERIAL_KEYS.iter().enumerate() {
            if let Some(v) = el.props.get(*k) {
                pending.push(PendingDecl {
                    rank: cascade_rank(false, true),
                    specificity: Specificity::default(),
                    order,
                    key: (*k).to_string(),
                    source: "<inline>".to_string(),
                    value: PendingValue::Json(v.clone()),
                });
            }
        }
        pending.sort_by(|a, b| {
            a.rank
                .cmp(&b.rank)
                .then(a.specificity.cmp(&b.specificity))
                .then(a.order.cmp(&b.order))
        });

        // 1) 自定义属性先落地，供 var() 替换使用。
        for d in &pending {
            if !d.key.starts_with("--") {
                continue;
            }
            let raw = match &d.value {
                PendingValue::Css(s) => s.clone(),
                PendingValue::Json(v) => prop_scalar(v).unwrap_or_default(),
            };
            let val = var_substitute(&raw, &customs)
                .map_err(|e| css_err(&format!("{} {}", d.source, d.key), &e))?;
            customs.insert(d.key.clone(), val);
        }
        for (k, v) in &customs {
            out.insert(k.clone(), ArgValue::Str(v.clone()));
        }

        // 2) 常规声明。
        let mut color_alpha: Option<f64> = None;
        for d in &pending {
            if d.key.starts_with("--") {
                continue;
            }
            let value = match &d.value {
                PendingValue::Json(v) => to_arg(v),
                PendingValue::Css(raw) => {
                    let raw = var_substitute(raw, &customs)
                        .map_err(|e| css_err(&format!("{} {}", d.source, d.key), &e))?;
                    if d.key == "color" {
                        if let Ok((_, alpha)) = css_color(&raw) {
                            color_alpha = Some(alpha);
                        }
                    }
                    css_value(&d.key, &raw)
                        .map_err(|e| css_err(&format!("{} {}", d.source, d.key), &e))?
                }
            };
            out.insert(d.key.clone(), value);
        }
        // 颜色的 alpha 位（`#rrggbbaa` / `rgba()`）只有在元素没拿到 opacity 时才生效。
        if let Some(alpha) = color_alpha {
            if !out.contains_key("opacity") {
                out.insert("opacity".to_string(), ArgValue::Num(alpha));
            }
        }
        Ok(out)
    }

    fn build_material(
        &self,
        style: &HashMap<String, ArgValue>,
    ) -> Result<cga_gpu::shading::Material, String> {
        self.loader
            .build_material(style)
            .map_err(|e| e.replacen("build: ", "JSX: ", 1))
    }

    fn register_tags(&mut self, geo: &cga_core::Geometry, world: [f64; 16], emit: [f64; 16]) {
        for (name, entry) in self.pending_tags.clone() {
            let rel = mat4_mul(mat4_inv(entry), emit);
            self.tags.entry(name).or_default().push(TagInstance {
                geo: geo.clone(),
                world,
                rel,
            });
        }
    }

    fn emit(
        &mut self,
        scene: &mut Scene,
        geo: cga_core::Geometry,
        world: [f64; 16],
        mat: cga_gpu::shading::Material,
    ) {
        let (motor, lin) = cga_core::decompose_rigid(world);
        let g2 = if cga_gpu::scene_graph::is_identity3(lin) {
            geo.clone()
        } else {
            cga_core::Geometry::AffineGeometry(cga_core::AffineGeometry::new(geo.clone(), lin))
        };
        let group = self.group_stack.last().copied().unwrap_or(0);
        scene.add_object(
            Object::new(ObjectParams {
                geometry: g2,
                material: mat,
                position: [0.0, 0.0, 0.0],
                rotation_axis: [0.0, 0.0, 1.0],
                rotation_angle: 0.0,
                motor: Some(motor),
            })
            .with_group(group),
        );
        let idx = scene.objects.len() - 1;
        if let Some(jname) = self.joint_stack.last().cloned() {
            if let Some(j) = self.kin.joints.iter_mut().rev().find(|j| j.name == jname) {
                j.meshes.push(idx);
            }
        }
        self.register_tags(&geo, world, world);
    }

    /// 入口：压入位置帧、算出本元素的计算样式，再分派。
    fn walk<'a>(
        &mut self,
        cur: Frame<'a>,
        stack: &mut Vec<Frame<'a>>,
        ctx: [f64; 16],
        mat: &HashMap<String, ArgValue>,
        scene: &mut Scene,
        cam: &mut Option<PerspectiveCamera>,
    ) -> Result<(), String> {
        let start = scene.objects.len();
        if let Some(range) = cur.el.cache.clone() {
            // 增量构建（档 0）：annotate_reuse 已核对"子树未变、纯、路径干净"，
            // 整棵复用上一帧的产出对象。
            let prev = self
                .prev_objects
                .expect("JSX: cache range without previous objects");
            scene.objects.extend_from_slice(&prev[range]);
            self.reused_subtrees += 1;
            self.reused_objects += scene.objects.len() - start;
            self.ranges
                .insert(cur.el as *const El, start..scene.objects.len());
            return Ok(());
        }
        stack.push(cur);
        let r = self.walk_pushed(cur.el, stack, ctx, mat, scene, cam);
        stack.pop();
        self.ranges
            .insert(cur.el as *const El, start..scene.objects.len());
        r
    }

    fn walk_pushed<'a>(
        &mut self,
        el: &'a El,
        stack: &mut Vec<Frame<'a>>,
        ctx: [f64; 16],
        mat: &HashMap<String, ArgValue>,
        scene: &mut Scene,
        cam: &mut Option<PerspectiveCamera>,
    ) -> Result<(), String> {
        let style = self.style_for(el, stack, mat)?;
        // 分组（档 2）：`<group name>` 元素或任意元素的 `group` prop，
        // 子树产出的对象都打上这个分组 id。
        let group = if el.tag == "group" {
            Some(p_str(el, "name")?.ok_or_else(|| "JSX: group needs a name".to_string())?)
        } else {
            p_str(el, "group")?
        };
        let gid = group.map(|name| self.group_id(&name));
        if let Some(g) = gid {
            self.group_stack.push(g);
        }
        let r = self.walk_inner(el, stack, ctx, &style, scene, cam);
        if gid.is_some() {
            self.group_stack.pop();
        }
        r
    }

    /// 分组名 → id（append-only：跨帧复用的对象带旧 id，注册表只能涨不能缩）。
    fn group_id(&mut self, name: &str) -> u32 {
        if let Some(i) = self.groups.iter().position(|g| g == name) {
            return i as u32;
        }
        self.groups.push(name.to_string());
        (self.groups.len() - 1) as u32
    }

    fn walk_inner<'a>(
        &mut self,
        el: &'a El,
        stack: &mut Vec<Frame<'a>>,
        ctx: [f64; 16],
        mat: &HashMap<String, ArgValue>,
        scene: &mut Scene,
        cam: &mut Option<PerspectiveCamera>,
    ) -> Result<(), String> {
        match el.tag.as_str() {
            "fragment" | "scene" => {
                if el.tag == "scene" {
                    if let Some(b) = p_num(el, "background")? {
                        scene.background = Color::from_hex(b as i32);
                    } else if let Some(c) = p_str(el, "background")? {
                        scene.background = parse_hex(&c)?;
                    }
                }
                for (i, c) in el.children.iter().enumerate() {
                    self.walk(Frame::new(c, i), stack, ctx, mat, scene, cam)?;
                }
                Ok(())
            }
            "translate" | "rotate" | "scale" | "mirror" => {
                let m2 = mat4_mul(ctx, self.modifier_matrix(el)?);
                for (i, c) in el.children.iter().enumerate() {
                    self.walk(Frame::new(c, i), stack, m2, mat, scene, cam)?;
                }
                Ok(())
            }
            "material" => {
                // 自身的内联 prop 已经在 style 里（Inline 级），直接下传。
                for (i, c) in el.children.iter().enumerate() {
                    self.walk(Frame::new(c, i), stack, ctx, mat, scene, cam)?;
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
                let material = self.build_material(mat)?;
                let geo = cga_core::Geometry::CsgGeometry(cga_core::CsgGeometry::new(op, kids));
                self.emit(scene, geo, cga_core::mat4_identity(), material);
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
                for (i, c) in el.children.iter().enumerate() {
                    self.walk(Frame::new(c, i), stack, ctx, mat, scene, cam)?;
                }
                self.pending_tags.pop();
                Ok(())
            }
            "joint" => self.joint_el(el, stack, ctx, mat, scene, cam),
            "gear" => self.gear_el(el),
            "cam" => self.cam_el(el),
            "drill" => {
                let (geo, w) = self.drill_cutter(el, ctx)?;
                let material = self.build_material(mat)?;
                self.emit(scene, geo, w, material);
                Ok(())
            }
            "instances" => {
                let name = p_str(el, "of")?.ok_or_else(|| "JSX: instances needs of".to_string())?;
                for inst in self.instances_of(&name)? {
                    let material = self.build_material(mat)?;
                    self.emit(scene, inst.geo.clone(), inst.world, material);
                }
                Ok(())
            }
            "when" => {
                let of = p_str(el, "of")?.ok_or_else(|| "JSX: when needs of".to_string())?;
                let count =
                    p_num(el, "count")?.ok_or_else(|| "JSX: when needs count".to_string())?;
                let n = self.tags.get(&of).map(|v| v.len()).unwrap_or(0);
                if (n as f64 - count).abs() < 1e-9 {
                    for (i, c) in el.children.iter().enumerate() {
                        self.walk(Frame::new(c, i), stack, ctx, mat, scene, cam)?;
                    }
                }
                Ok(())
            }
            "group" => {
                for (i, c) in el.children.iter().enumerate() {
                    self.walk(Frame::new(c, i), stack, ctx, mat, scene, cam)?;
                }
                Ok(())
            }
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
                let g = cga_core::Geometry::CsgGeometry(cga_core::CsgGeometry::new(op, inner));
                self.register_tags(&g, cga_core::mat4_identity(), ctx);
                kids.push(g);
                Ok(())
            }
            _ => {
                if el.tag == "drill" {
                    let (geo, w) = self.drill_cutter(el, ctx)?;
                    let (motor, lin) = cga_core::decompose_rigid(w);
                    kids.push(cga_core::Geometry::AffineGeometry(
                        cga_core::AffineGeometry::with_motor(geo, motor, lin),
                    ));
                    return Ok(());
                }
                if el.tag == "instances" {
                    let name =
                        p_str(el, "of")?.ok_or_else(|| "JSX: instances needs of".to_string())?;
                    for inst in self.instances_of(&name)? {
                        let (motor, lin) = cga_core::decompose_rigid(inst.world);
                        kids.push(cga_core::Geometry::AffineGeometry(
                            cga_core::AffineGeometry::with_motor(inst.geo.clone(), motor, lin),
                        ));
                    }
                    return Ok(());
                }
                let geo = self.build_geo(el)?;
                self.register_tags(&geo, ctx, ctx);
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

    fn modifier_matrix(&mut self, el: &El) -> Result<[f64; 16], String> {
        Ok(match el.tag.as_str() {
            "translate" => translate4(self.p_vec3_lazy(el, "t")?.unwrap_or([0.0; 3])),
            "rotate" => {
                let ax = self.p_vec3_lazy(el, "axis")?.unwrap_or([0.0, 0.0, 1.0]);
                let ang = self.p_num_lazy(el, "angle")?.unwrap_or(0.0);
                Multivector::rotor(ax, ang).to_matrix()
            }
            "scale" => {
                let v = match prop(el, "s") {
                    None => [1.0, 1.0, 1.0],
                    Some(Value::Number(_)) => {
                        let x = p_num(el, "s")?.unwrap_or(1.0);
                        [x, x, x]
                    }
                    Some(_) => self.p_vec3_lazy(el, "s")?.unwrap_or([1.0, 1.0, 1.0]),
                };
                let mut m = cga_core::mat4_identity();
                m[0] = v[0];
                m[5] = v[1];
                m[10] = v[2];
                m
            }
            _ => {
                let ax = self.p_vec3_lazy(el, "axis")?.unwrap_or([0.0, 0.0, 1.0]);
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
        let mut args: HashMap<String, ArgValue> = HashMap::new();
        for (k, v) in &el.props {
            if MATERIAL_KEYS.contains(&k.as_str())
                || k == "class"
                || k == "className"
                || k == "id"
                || k == "group"
            {
                continue;
            }
            // 标量惰性查询（clearance/collides/inside）在构建期解析成数值。
            // 此前 {__q} 对象在几何参数里会被 to_arg 静默成 0——现在要么给真值，要么报错。
            args.insert(
                k.clone(),
                match v {
                    Value::Object(o) if o.contains_key("__q") => {
                        ArgValue::Num(self.eval_lazy_num(o, k)?)
                    }
                    _ => to_arg(v),
                },
            );
        }
        let args = self
            .loader
            .resolve(&el.tag, Vec::new(), args, 0)
            .map_err(|e| e.replacen("build: ", "JSX: ", 1))?;
        self.loader
            .build_geometry(&el.tag, &args, 0)
            .map_err(|e| e.replacen("build: ", "JSX: ", 1))
    }

    fn primitive_el(
        &mut self,
        el: &El,
        ctx: [f64; 16],
        mat: &HashMap<String, ArgValue>,
        scene: &mut Scene,
    ) -> Result<(), String> {
        let geo = self.build_geo(el)?;
        let material = self.build_material(mat)?;
        self.emit(scene, geo, ctx, material);
        Ok(())
    }

    fn joint_el<'a>(
        &mut self,
        el: &'a El,
        stack: &mut Vec<Frame<'a>>,
        ctx: [f64; 16],
        mat: &HashMap<String, ArgValue>,
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
        let at = self.p_vec3_lazy(el, "at")?.unwrap_or([0.0; 3]);
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
                } else if let Some(&o) = self.pose.get(&name) {
                    let has_q = prop(el, "q")
                        .map(|v| !matches!(v, Value::Null))
                        .unwrap_or(false);
                    if has_q {
                        return Err(format!(
                            "JSX: joint {name} has a pose override, q must be omitted"
                        ));
                    }
                    if !kind.is_1dof() {
                        return Err(format!(
                            "JSX: pose override needs a 1-DOF joint, got {}",
                            kind.name()
                        ));
                    }
                    vec![o]
                } else {
                    // q 允许单数（1-DOF）或数组（多 DOF）；不要用 p_num 探类型，
                    // 它对数组会直接报错，导致多 DOF 关节无法书写。
                    match prop(el, "q") {
                        None | Some(Value::Null) => vec![0.0; arity],
                        Some(Value::Number(_)) => {
                            if arity != 1 {
                                return Err(format!(
                                    "JSX: {} joint q must be {}",
                                    kind.name(),
                                    kind.q_shape()
                                ));
                            }
                            vec![p_num(el, "q")?.unwrap_or(0.0)]
                        }
                        Some(Value::Array(_)) => {
                            let l = p_num_list(el, "q")?.unwrap_or_default();
                            if l.len() != arity {
                                return Err(format!(
                                    "JSX: {} joint q must be {}",
                                    kind.name(),
                                    kind.q_shape()
                                ));
                            }
                            l
                        }
                        Some(v) => {
                            return Err(format!(
                                "JSX: {name} q must be a number or number list, got {v}"
                            ))
                        }
                    }
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
        for (i, c) in el.children.iter().enumerate() {
            self.walk(Frame::new(c, i), stack, world, mat, scene, cam)?;
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

// ---- R2 平权块：惰性查询 / drill / instances / when / solve / pose ----

impl<'p> Builder<'p> {
    /// Element → (raw geometry, transform) for queries and drill targets.
    /// Element targets are evaluated in the identity frame (they are
    /// top-level definitions by convention).
    fn el_geometry(
        &mut self,
        el: &El,
        m: [f64; 16],
    ) -> Result<(cga_core::Geometry, [f64; 16]), String> {
        match el.tag.as_str() {
            "translate" | "rotate" | "scale" | "mirror" => {
                let m2 = mat4_mul(m, self.modifier_matrix(el)?);
                if el.children.len() != 1 {
                    return Err(format!(
                        "JSX: {} query target needs exactly one child",
                        el.tag
                    ));
                }
                self.el_geometry(&el.children[0], m2)
            }
            "material" => {
                if el.children.len() != 1 {
                    return Err(format!(
                        "JSX: {} query target needs exactly one child",
                        el.tag
                    ));
                }
                self.el_geometry(&el.children[0], m)
            }
            "union" | "difference" | "intersection" => {
                let mut kids = Vec::new();
                for c in &el.children {
                    self.collect_geom(c, m, &mut kids)?;
                }
                let op = match el.tag.as_str() {
                    "union" => cga_core::CsgOp::Union,
                    "difference" => cga_core::CsgOp::Difference,
                    _ => cga_core::CsgOp::Intersection,
                };
                Ok((
                    cga_core::Geometry::CsgGeometry(cga_core::CsgGeometry::new(op, kids)),
                    cga_core::mat4_identity(),
                ))
            }
            _ => Ok((self.build_geo(el)?, m)),
        }
    }

    /// (transform, world bounds) of a query target: tag name or element.
    fn query_target(
        &mut self,
        of: &Value,
        what: &str,
    ) -> Result<([f64; 16], Option<[[f64; 3]; 2]>), String> {
        match of {
            Value::String(s) => {
                let insts = match self.tags.get(s) {
                    Some(list) if !list.is_empty() => list,
                    _ => return Err(format!("JSX: unknown reference \"{s}\"")),
                };
                let mut acc: Option<[[f64; 3]; 2]> = None;
                for i in insts {
                    if let Some(b) = self
                        .loader
                        .local_bounds(&i.geo)
                        .map(|b| crate::scene_build::transform_bbox(b, i.world))
                    {
                        acc = Some(match acc {
                            None => b,
                            Some(a) => [
                                [
                                    a[0][0].min(b[0][0]),
                                    a[0][1].min(b[0][1]),
                                    a[0][2].min(b[0][2]),
                                ],
                                [
                                    a[1][0].max(b[1][0]),
                                    a[1][1].max(b[1][1]),
                                    a[1][2].max(b[1][2]),
                                ],
                            ],
                        });
                    }
                }
                Ok((insts[0].world, acc))
            }
            Value::Object(_) => {
                let el = to_el(of)?;
                let (geo, m4) = self.el_geometry(&el, cga_core::mat4_identity())?;
                let b = self
                    .loader
                    .local_bounds(&geo)
                    .map(|b| crate::scene_build::transform_bbox(b, m4));
                Ok((m4, b))
            }
            _ => Err(format!("JSX: {what} needs a reference name or element")),
        }
    }

    fn face_of(
        &mut self,
        of: &Value,
        key: &str,
        what: &str,
    ) -> Result<([f64; 3], [f64; 3]), String> {
        let (axis, sign) = crate::scene_build::parse_face_key(key, 0, what)?;
        let (m, p, n) = match of {
            Value::String(s) => {
                let insts = self
                    .tags
                    .get(s)
                    .filter(|l| !l.is_empty())
                    .ok_or_else(|| format!("JSX: unknown reference \"{s}\""))?;
                let (p, n) = crate::scene_build::face_local(&insts[0].geo, axis, sign)
                    .ok_or_else(|| format!("JSX: {what}: reference has no finite bounds"))?;
                (insts[0].world, p, n)
            }
            Value::Object(_) => {
                let el = to_el(of)?;
                let (geo, m4) = self.el_geometry(&el, cga_core::mat4_identity())?;
                let (p, n) = crate::scene_build::face_local(&geo, axis, sign)
                    .ok_or_else(|| format!("JSX: {what}: reference has no finite bounds"))?;
                (m4, p, n)
            }
            _ => return Err(format!("JSX: {what} needs a reference name or element")),
        };
        Ok((
            cga_core::transform_point(m, p),
            crate::scene_build::transform_normal(m, n),
        ))
    }

    /// Resolve a prop value to a point: plain [x,y,z] or a lazy query node.
    fn resolve_vec3(&mut self, v: &Value, what: &str) -> Result<[f64; 3], String> {
        match v {
            Value::Array(a) if a.len() == 3 => {
                let mut out = [0.0; 3];
                for i in 0..3 {
                    if a[i].is_object() {
                        let o = a[i].as_object().unwrap();
                        if o.contains_key("__q") {
                            // 标量查询不支持；保持简单：惰性节点只在整向量位置出现
                            return Err(format!("JSX: {what}: nested query in vector component"));
                        }
                    }
                    out[i] = a[i]
                        .as_f64()
                        .ok_or_else(|| format!("JSX: {what}[{i}] must be a number"))?;
                }
                Ok(out)
            }
            Value::Object(o) if o.contains_key("__q") => self.eval_lazy(o, what),
            _ => Err(format!("JSX: {what} must be [x, y, z] or a query, got {v}")),
        }
    }

    fn eval_lazy(
        &mut self,
        o: &serde_json::Map<String, Value>,
        what: &str,
    ) -> Result<[f64; 3], String> {
        let q = o["__q"].as_str().unwrap_or("");
        match q {
            "vadd" | "vsub" => {
                let a = self.resolve_vec3(&o["a"], what)?;
                let b = self.resolve_vec3(&o["b"], what)?;
                let s = if q == "vadd" { 1.0 } else { -1.0 };
                Ok([a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]])
            }
            "vscale" => {
                let a = self.resolve_vec3(&o["a"], what)?;
                let s = o["s"].as_f64().unwrap_or(1.0);
                Ok([a[0] * s, a[1] * s, a[2] * s])
            }
            "face" | "fnrm" => {
                let key = o["key"].as_str().unwrap_or("+z");
                let (p, n) = self.face_of(&o["of"], key, q)?;
                Ok(if q == "face" { p } else { n })
            }
            "xdir" | "ydir" | "zdir" => {
                let (m, _) = self.query_target(&o["of"], q)?;
                let col = match q {
                    "xdir" => [m[0], m[4], m[8]],
                    "ydir" => [m[1], m[5], m[9]],
                    _ => [m[2], m[6], m[10]],
                };
                let n = (col[0] * col[0] + col[1] * col[1] + col[2] * col[2]).sqrt();
                if n < 1e-12 {
                    return Err(format!("JSX: {q} is degenerate"));
                }
                Ok([col[0] / n, col[1] / n, col[2] / n])
            }
            _ => {
                let (_, b) = self.query_target(&o["of"], q)?;
                let b = b.ok_or_else(|| format!("JSX: {q}: reference has no finite bounds"))?;
                Ok(match q {
                    "center" => [
                        (b[0][0] + b[1][0]) / 2.0,
                        (b[0][1] + b[1][1]) / 2.0,
                        (b[0][2] + b[1][2]) / 2.0,
                    ],
                    "lo" => b[0],
                    "hi" => b[1],
                    "size" => [b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]],
                    _ => return Err(format!("JSX: unknown query {q}")),
                })
            }
        }
    }

    /// p_vec3 的惰性版：先按数组解析，失败再按查询节点解析。
    fn p_vec3_lazy(&mut self, el: &El, key: &str) -> Result<Option<[f64; 3]>, String> {
        match prop(el, key) {
            None => Ok(None),
            Some(v) => self.resolve_vec3(v, key).map(Some),
        }
    }

    /// p_num 的惰性版：标量查询节点（`clearance`/`collides`/`inside`）在构建期解析。
    fn p_num_lazy(&mut self, el: &El, key: &str) -> Result<Option<f64>, String> {
        match prop(el, key) {
            None | Some(Value::Null) => Ok(None),
            Some(Value::Object(o)) if o.contains_key("__q") => self.eval_lazy_num(o, key).map(Some),
            Some(_) => p_num(el, key),
        }
    }

    /// 标量惰性查询（C1，`docs/collision-plan.md` D5 修正：走 `{__q}` 通道而非
    /// `solve()` 的求值期语义——求值期场景还没建成）。三值里的 Unknown 不许变成
    /// 数字（D2）：报错并写明原因。
    fn eval_lazy_num(
        &mut self,
        o: &serde_json::Map<String, Value>,
        what: &str,
    ) -> Result<f64, String> {
        let q = o["__q"].as_str().unwrap_or("");
        match q {
            "clearance" => {
                let a = self.query_solids(&o["a"], what)?;
                let b = self.query_solids(&o["b"], what)?;
                let mut best: Option<f64> = None;
                for (ga, wa) in &a {
                    for (gb, wb) in &b {
                        if let Some(d) = cga_core::collision::separation(ga, *wa, gb, *wb) {
                            best = Some(best.map_or(d, |x: f64| x.min(d)));
                        }
                    }
                }
                best.ok_or_else(|| {
                    format!("JSX: {what}: clearance 是 Unknown（不支持的几何对，不许假装精确）")
                })
            }
            "collides" => {
                let a = self.query_solids(&o["a"], what)?;
                let b = self.query_solids(&o["b"], what)?;
                let mut any_unknown = false;
                for (ga, wa) in &a {
                    for (gb, wb) in &b {
                        match cga_core::collision::overlap(ga, *wa, gb, *wb) {
                            cga_core::collision::Hit::Yes => return Ok(1.0),
                            cga_core::collision::Hit::No => {}
                            cga_core::collision::Hit::Unknown => any_unknown = true,
                        }
                    }
                }
                if any_unknown {
                    return Err(format!(
                        "JSX: {what}: collides 是 Unknown（不支持的几何对，不许假装精确）"
                    ));
                }
                Ok(0.0)
            }
            "inside" => {
                let solids = self.query_solids(&o["of"], what)?;
                let pv = o
                    .get("p")
                    .ok_or_else(|| format!("JSX: {what}: inside needs p=[x,y,z]"))?;
                let Value::Array(a) = pv else {
                    return Err(format!("JSX: {what}: inside p must be [x,y,z]"));
                };
                if a.len() != 3 {
                    return Err(format!("JSX: {what}: inside p must be [x,y,z]"));
                }
                let mut p = [0.0; 3];
                for i in 0..3 {
                    p[i] = a[i]
                        .as_f64()
                        .ok_or_else(|| format!("JSX: {what}: inside p[{i}] must be a number"))?;
                }
                let mut any_unknown = false;
                for (g, w) in &solids {
                    match cga_core::collision::contains_point(g, *w, p) {
                        cga_core::collision::Hit::Yes => return Ok(1.0),
                        cga_core::collision::Hit::No => {}
                        cga_core::collision::Hit::Unknown => any_unknown = true,
                    }
                }
                if any_unknown {
                    return Err(format!(
                        "JSX: {what}: inside 是 Unknown（不支持的几何，不许假装精确）"
                    ));
                }
                Ok(0.0)
            }
            "qadd" | "qsub" | "qmul" | "qdiv" => {
                let a = self.eval_num_value(&o["a"], what)?;
                let b = self.eval_num_value(&o["b"], what)?;
                Ok(match q {
                    "qadd" => a + b,
                    "qsub" => a - b,
                    "qmul" => a * b,
                    _ => a / b,
                })
            }
            _ => Err(format!("JSX: {what}: unknown scalar query {q}")),
        }
    }

    /// 数值或标量查询节点（`qadd`/`qmul` 等的操作数）。
    fn eval_num_value(&mut self, v: &Value, what: &str) -> Result<f64, String> {
        match v {
            Value::Number(n) => n.as_f64().ok_or_else(|| format!("JSX: {what}: bad number")),
            Value::Object(o) if o.contains_key("__q") => self.eval_lazy_num(o, what),
            _ => Err(format!(
                "JSX: {what}: must be a number or scalar query, got {v}"
            )),
        }
    }

    /// 引用 → 实体列表（几何 + 世界变换）：tag 名查注册表；内联元素直接构建。
    /// 与 `instances`/`when` 同规则：tag 必须先定义后使用。
    fn query_solids(
        &mut self,
        of: &Value,
        what: &str,
    ) -> Result<Vec<(cga_core::Geometry, [f64; 16])>, String> {
        match of {
            Value::String(s) => {
                let insts = self
                    .tags
                    .get(s)
                    .filter(|v| !v.is_empty())
                    .ok_or_else(|| format!("JSX: {what}: unknown reference \"{s}\""))?;
                Ok(insts.iter().map(|i| (i.geo.clone(), i.world)).collect())
            }
            Value::Object(_) => {
                let el = to_el(of)?;
                let (g, m) = self.el_geometry(&el, cga_core::mat4_identity())?;
                Ok(vec![(g, m)])
            }
            _ => Err(format!("JSX: {what}: needs a reference name or element")),
        }
    }

    /// drill 切刀：几何 + 世界变换。
    fn drill_cutter(
        &mut self,
        el: &El,
        ctx: [f64; 16],
    ) -> Result<(cga_core::Geometry, [f64; 16]), String> {
        let r = p_num(el, "r")?.ok_or_else(|| "JSX: drill needs r=".to_string())?;
        if !(r > 0.0) {
            return Err("JSX: drill.r must be > 0".to_string());
        }
        let ax = match p_num(el, "axis")? {
            Some(a) if a == 0.0 || a == 1.0 || a == 2.0 => a as usize,
            Some(_) => return Err("JSX: drill.axis must be 0, 1 or 2 (X/Y/Z)".to_string()),
            None => return Err("JSX: drill needs axis= (0/1/2)".to_string()),
        };
        // through：tag 名或元素
        let (rel, fbox): ([f64; 16], Option<[[f64; 3]; 2]>) = match prop(el, "through") {
            Some(Value::String(s)) => {
                let insts = match self.tags.get(s) {
                    Some(list) if !list.is_empty() => list.clone(),
                    _ => return Err(format!("JSX: unknown reference \"{s}\"")),
                };
                let rel0 = insts[0].rel;
                let f = mat4_mul(ctx, rel0);
                let finv = mat4_inv(f);
                let mut acc: Option<[[f64; 3]; 2]> = None;
                for i in &insts {
                    if let Some(b) = self
                        .loader
                        .local_bounds(&i.geo)
                        .map(|b| crate::scene_build::transform_bbox(b, i.world))
                    {
                        acc = Some(match acc {
                            None => b,
                            Some(a) => [
                                [
                                    a[0][0].min(b[0][0]),
                                    a[0][1].min(b[0][1]),
                                    a[0][2].min(b[0][2]),
                                ],
                                [
                                    a[1][0].max(b[1][0]),
                                    a[1][1].max(b[1][1]),
                                    a[1][2].max(b[1][2]),
                                ],
                            ],
                        });
                    }
                }
                let b = acc.ok_or_else(|| "JSX: drill target has no finite bounds".to_string())?;
                (rel0, Some(crate::scene_build::transform_bbox(b, finv)))
            }
            Some(v @ Value::Object(_)) => {
                let t = to_el(v)?;
                let (geo, m4) = self.el_geometry(&t, cga_core::mat4_identity())?;
                let b = self
                    .loader
                    .local_bounds(&geo)
                    .ok_or_else(|| "JSX: drill target has no finite bounds".to_string())?;
                (m4, Some(b))
            }
            Some(_) => {
                return Err("JSX: drill.through needs a reference name or geometry".to_string())
            }
            None => (cga_core::mat4_identity(), None),
        };
        // from/to：数字或 "name:key" 面引用
        let end = |v: Option<&Value>| -> Result<Option<f64>, String> {
            let f = mat4_mul(ctx, rel);
            match v {
                None => Ok(None),
                Some(Value::Number(n)) => Ok(n.as_f64()),
                Some(Value::String(s)) => {
                    let (nm, key) = s
                        .split_once(':')
                        .ok_or_else(|| "JSX: drill face ref must be \"name:key\"".to_string())?;
                    let (axis, sign) = crate::scene_build::parse_face_key(key, 0, "drill")?;
                    if axis != ax {
                        return Err(format!(
                            "JSX: drill face key \"{key}\" does not match axis={ax}"
                        ));
                    }
                    let insts = self
                        .tags
                        .get(nm)
                        .filter(|l| !l.is_empty())
                        .ok_or_else(|| format!("JSX: unknown reference \"{nm}\""))?;
                    let (p_local, _) = crate::scene_build::face_local(&insts[0].geo, axis, sign)
                        .ok_or_else(|| {
                            "JSX: drill face reference has no finite bounds".to_string()
                        })?;
                    let p_world = cga_core::transform_point(insts[0].world, p_local);
                    Ok(Some(cga_core::transform_point(mat4_inv(f), p_world)[ax]))
                }
                _ => Err("JSX: drill from/to must be a number or \"name:key\"".to_string()),
            }
        };
        let a0 = match end(prop(el, "from"))? {
            Some(x) => x,
            None => fbox
                .map(|b| b[0][ax])
                .ok_or_else(|| "JSX: drill needs through=<name or geometry>".to_string())?,
        };
        let a1 = match end(prop(el, "to"))? {
            Some(x) => x,
            None => fbox
                .map(|b| b[1][ax])
                .ok_or_else(|| "JSX: drill needs through=<name or geometry>".to_string())?,
        };
        if !(a1 > a0) {
            return Err(format!(
                "JSX: drill extent must be non-empty ({a0} .. {a1})"
            ));
        }
        let center = (a0 + a1) / 2.0;
        let rmat = match ax {
            0 => Multivector::rotor([0.0, 1.0, 0.0], std::f64::consts::FRAC_PI_2).to_matrix(),
            1 => Multivector::rotor([1.0, 0.0, 0.0], -std::f64::consts::FRAC_PI_2).to_matrix(),
            _ => cga_core::mat4_identity(),
        };
        let mut off = [0.0, 0.0, 0.0];
        off[ax] = center;
        let f = mat4_mul(ctx, rel);
        let w = mat4_mul(mat4_mul(f, translate4(off)), rmat);
        Ok((
            cga_core::Geometry::CylinderGeometry(cga_core::CylinderGeometry::new(r, a1 - a0)),
            w,
        ))
    }

    fn instances_of(&self, name: &str) -> Result<Vec<TagInstance>, String> {
        self.tags
            .get(name)
            .filter(|l| !l.is_empty())
            .cloned()
            .ok_or_else(|| format!("JSX: unknown reference \"{name}\""))
    }
}

/// A rendered frame in PNG form (headless rendering).
pub struct HeadlessImage {
    pub width: i32,
    pub height: i32,
    pub png: Vec<u8>,
}

/// Evaluate a JSX scene (+ optional CSS) into a `SceneRun`.
pub fn run_jsx(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
) -> Result<crate::SceneRun, String> {
    run_jsx_pose(jsx_src, css_src, asset_root, &[])
}

/// JSX scene with pose overrides: injected as the `P` global for JS
/// variables, and matched by name for 1-DOF joints whose `q` is omitted.
pub fn run_jsx_pose(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
    pose: &[(String, f64)],
) -> Result<crate::SceneRun, String> {
    run_jsx_pose_sandbox(
        jsx_src,
        css_src,
        asset_root,
        pose,
        crate::react::Sandbox::from_env(),
    )
}

/// 同 [`run_jsx_pose`]，并可选沙箱隔离（见 [`crate::react::Sandbox`]）。
pub fn run_jsx_pose_sandbox(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
    pose: &[(String, f64)],
    sandbox: crate::react::Sandbox,
) -> Result<crate::SceneRun, String> {
    let js = compile_jsx_with(jsx_src, &mut disk_resolver(asset_root))?;
    let v = eval_js_react(&js, pose, sandbox)?;
    build_scene_run(&v, css_src, asset_root, pose)
}

/// React 路径：真 React 渲染（hooks/状态/副作用可用），把已提交的场景实例树快照成
/// `{t,p,c}` JSON——与旧元素树同形，于是场景构建一行都不用改。用帧事务一次取下快照。
fn eval_js_react(
    js: &str,
    pose: &[(String, f64)],
    sandbox: crate::react::Sandbox,
) -> Result<Value, String> {
    use crate::react;
    let module = format!("{PRELUDE}{js}");
    let pose_map: HashMap<String, f64> = pose.iter().cloned().collect();
    react::with_session(react::Runtime::from_env(), |sess| {
        sess.register_global("solve", 2, solve_host)?;
        sess.unmount()?; // 清掉上一个场景（顺带跑其副作用清理）
        let out = sess.frame(react::FrameAction::Mount {
            src: &module,
            pose: &pose_map,
            sandbox,
        })?;
        if out.stats.errors > 0 {
            let detail = sess
                .logs()?
                .into_iter()
                .filter(|l| l.level == "error")
                .map(|l| l.text)
                .collect::<Vec<_>>()
                .join("\n");
            return Err(format!("JSX: 渲染期异常:\n{detail}"));
        }
        if !out.errors.is_empty() {
            return Err(format!("JSX: React 错误: {}", out.errors.join("; ")));
        }
        Ok(out.snapshot)
    })
}

// ---- 增量构建（档 0）-------------------------------------------------------

/// 有全局副作用或依赖全局状态的元素：永不作为复用锚点，且其存在使祖先子树不纯。
/// 不在表内的非特殊标签一律是图元（纯）。`drill` 只依赖自身 props 与 ctx，是纯的。
const SIDE_EFFECT_TAGS: &[&str] = &[
    "ambient_light",
    "directional_light",
    "point_light",
    "camera",
    "background",
    "tag",
    "joint",
    "gear",
    "cam",
    "instances",
    "when",
];

/// 动态上下文（`pending_tags` / `joint_stack` / tag 注册表查询）：处于这些元素
/// 之下的子树即使内容未变也不能复用——它们的产出还依赖树外的状态。
const DYNAMIC_TAGS: &[&str] = &["tag", "joint", "when"];

/// props 里（递归）是否含惰性查询 `{__q: …}`：字符串引用（`center("ball")` 等）
/// 要查 tag 注册表——那是树外状态，版本号抬不动它。含查询的元素不纯。
fn has_lazy_query(v: &Value) -> bool {
    match v {
        Value::Object(m) => m.contains_key("__q") || m.values().any(has_lazy_query),
        Value::Array(a) => a.iter().any(has_lazy_query),
        _ => false,
    }
}

/// 子树是否"纯"：不含任何副作用元素，产出只取决于自身 props、祖先变换与继承样式。
/// `scene` 带 `background` prop 时也算有副作用（写 `scene.background`）。
fn pure_subtree(el: &El) -> bool {
    if SIDE_EFFECT_TAGS.contains(&el.tag.as_str()) {
        return false;
    }
    if el.tag == "scene" && prop(el, "background").is_some() {
        return false;
    }
    if el.props.values().any(has_lazy_query) {
        return false;
    }
    el.children.iter().all(pure_subtree)
}

/// 增量构建的旁路标注：找出"整棵未变且纯"的子树，把上一帧的对象区间标到
/// `el.cache`，`walk` 走到时整棵复用。复用条件（全部满足）：
/// - 结构配对成功：同位置、同 tag、同子节点数、有版本号；
/// - 祖先链自身版本全不变（`path_clean`）——祖先的 props 与子节点顺序决定本
///   节点的变换、继承样式与 CSS 兄弟位置；
/// - 子树版本 `__s` 不变——子树内容不变；
/// - 不在 tag / joint / when 之下，且子树纯；
/// - 样式表不含兄弟组合器（`allow`，见 [`Builder::sibling_rules`]）。
fn annotate_reuse(
    new: &mut El,
    prev: Option<&El>,
    path_clean: bool,
    under_dynamic: bool,
    allow: bool,
) {
    let pair =
        prev.filter(|p| p.tag == new.tag && p.children.len() == new.children.len() && p.v != 0);
    let own_clean = pair.is_some_and(|p| p.v == new.v);
    let clean_here = path_clean && own_clean;
    let subtree_clean = pair.is_some_and(|p| p.s == new.s);
    if allow && clean_here && subtree_clean && !under_dynamic && pure_subtree(new) {
        if let Some(range) = pair.and_then(|p| p.cache.clone()) {
            new.cache = Some(range);
            return; // 整棵复用，不必下钻
        }
    }
    // 惰性查询的字符串引用查 tag 注册表（树外状态）：版本号反映不了查询结果的
    // 变化。含查询的节点自身不能当锚点（pure_subtree 已挡），它的 ctx 输出也不能
    // 信——子节点的路径同样不再干净。
    let clean_for_children = clean_here && !new.props.values().any(has_lazy_query);
    let dynamic = under_dynamic || DYNAMIC_TAGS.contains(&new.tag.as_str());
    for (i, c) in new.children.iter_mut().enumerate() {
        annotate_reuse(
            c,
            pair.and_then(|p| p.children.get(i)),
            clean_for_children,
            dynamic,
            allow,
        );
    }
}

/// 构建结束后把本帧的对象区间写回缓存树。复用锚点的子节点按平移量从上一帧
/// 搬区间，保住更细粒度的复用能力（锚点下一帧可能不再是锚点）。
fn apply_ranges(
    new: &mut El,
    prev: Option<&El>,
    ranges: &HashMap<*const El, std::ops::Range<usize>>,
) {
    if let Some(r) = ranges.get(&(new as *const El)) {
        let fresh = r.clone();
        if let (Some(p), Some(old)) = (prev, new.cache.clone()) {
            // 复用锚点：old 是上一帧区间，fresh 是本帧区间，差值即平移量。
            let delta = fresh.start as i64 - old.start as i64;
            shift_child_ranges(new, p, delta);
        }
        new.cache = Some(fresh);
    }
    for (i, c) in new.children.iter_mut().enumerate() {
        apply_ranges(c, prev.and_then(|p| p.children.get(i)), ranges);
    }
}

fn shift_child_ranges(new: &mut El, prev: &El, delta: i64) {
    for (nc, pc) in new.children.iter_mut().zip(prev.children.iter()) {
        if let Some(r) = &pc.cache {
            let s = (r.start as i64 + delta) as usize;
            let e = (r.end as i64 + delta) as usize;
            nc.cache = Some(s..e);
        }
        shift_child_ranges(nc, pc, delta);
    }
}

/// 增量构建的跨帧缓存（档 0）：上一帧的元素树（带每个节点的产出区间）、
/// 对象表与分组注册表。
pub struct BuildCache {
    tree: El,
    objects: Vec<Object>,
    groups: Vec<String>,
}

/// 一次构建的增量统计。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BuildStats {
    /// 整棵复用的子树数。
    pub reused_subtrees: usize,
    /// 复用（而非重建）的对象数。
    pub reused_objects: usize,
    /// 本帧对象总数。
    pub total_objects: usize,
}

fn build_scene_run_cached(
    v: &Value,
    css_src: Option<&str>,
    asset_root: &str,
    pose: &[(String, f64)],
    cache: Option<&BuildCache>,
) -> Result<(crate::SceneRun, El, BuildStats), String> {
    let mut root = to_el(v)?;
    let rules = match css_src {
        Some(c) => parse_css(c)?,
        None => Vec::new(),
    };
    let sibling_rules = rules.iter().any(|r| {
        r.sel
            .parts
            .iter()
            .any(|(c, _)| matches!(c, Combinator::NextSibling | Combinator::LaterSibling))
    });
    if let Some(c) = cache {
        annotate_reuse(&mut root, Some(&c.tree), true, false, !sibling_rules);
    }
    // 根作用域（`:root` / `scene`）规则的 background
    let mut scene = Scene::new(None);
    let mut customs: HashMap<String, String> = HashMap::new();
    for r in rules.iter().filter(|r| r.sel.scene_scope) {
        for d in r.decls.iter().filter(|d| d.name.starts_with("--")) {
            let v = var_substitute(&d.value, &customs)
                .map_err(|e| css_err(&format!("{} {}", r.sel.text, d.name), &e))?;
            customs.insert(d.name.clone(), v);
        }
    }
    for r in rules.iter().filter(|r| r.sel.scene_scope) {
        for d in &r.decls {
            if d.name == "background" || d.name == "background-color" {
                let raw = var_substitute(&d.value, &customs)
                    .map_err(|e| css_err(&format!("{} {}", r.sel.text, d.name), &e))?;
                let (hex, _) = css_color(&raw)
                    .map_err(|e| css_err(&format!("{} {}", r.sel.text, d.name), &e))?;
                scene.background = Color::from_hex(hex as i32);
            }
        }
    }
    let mut cam: Option<PerspectiveCamera> = None;
    let mut b = Builder {
        loader: Builders::new(asset_root),
        kin: Kinematics::default(),
        joint_stack: Vec::new(),
        driven_by: HashMap::new(),
        tags: TagRegistry::new(),
        pending_tags: Vec::new(),
        rules,
        pose: pose.iter().cloned().collect(),
        groups: cache
            .map(|c| c.groups.clone())
            .unwrap_or_else(|| vec![String::new()]),
        group_stack: Vec::new(),
        prev_objects: cache.map(|c| c.objects.as_slice()),
        ranges: HashMap::new(),
        reused_subtrees: 0,
        reused_objects: 0,
    };
    let mut stack = Vec::new();
    b.walk(
        Frame::new(&root, 0),
        &mut stack,
        cga_core::mat4_identity(),
        &HashMap::new(),
        &mut scene,
        &mut cam,
    )?;
    let stats = BuildStats {
        reused_subtrees: b.reused_subtrees,
        reused_objects: b.reused_objects,
        total_objects: scene.objects.len(),
    };
    // 把本帧各节点的产出区间写回树，作为下一帧的复用缓存。
    apply_ranges(&mut root, cache.map(|c| &c.tree), &b.ranges);
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
    let mut kin = b.kin;
    let mut ps: Vec<(String, f64)> = pose.to_vec();
    ps.sort_by(|a, b| a.0.cmp(&b.0));
    kin.pose = ps;
    Ok((
        crate::SceneRun {
            scene,
            camera: cam,
            tags: b.tags,
            kinematics: kin,
            groups: b.groups,
        },
        root,
        stats,
    ))
}

/// 由 `{t,p,c}` 树构建场景（CSS 匹配、材质继承、惰性查询解析都在这里）。
fn build_scene_run(
    v: &Value,
    css_src: Option<&str>,
    asset_root: &str,
    pose: &[(String, f64)],
) -> Result<crate::SceneRun, String> {
    Ok(build_scene_run_cached(v, css_src, asset_root, pose, None)?.0)
}

/// Headless render of a JSX scene.
pub fn render_jsx_png(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
    w: i32,
    h: i32,
    aa: i32,
) -> Result<HeadlessImage, String> {
    render_jsx_png_mode(
        jsx_src,
        css_src,
        asset_root,
        w,
        h,
        aa,
        cga_gpu::RenderMode::Normal,
    )
}

/// 同 `render_jsx_png`，但可选渲染模式：`Normal`（透明度生效，含反射/折射）或
/// `IgnoreOpacity`（一切当不透明，不发射次级光线）。
#[allow(clippy::too_many_arguments)]
pub fn render_jsx_png_mode(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
    w: i32,
    h: i32,
    aa: i32,
    mode: cga_gpu::RenderMode,
) -> Result<HeadlessImage, String> {
    if w <= 0 || h <= 0 {
        return Err(format!("headless: bad size {w}x{h}"));
    }
    let mut run = run_jsx(jsx_src, css_src, asset_root)?;
    run.camera.aspect = f64::from(w) / f64::from(h);
    let mut r = cga_gpu::Renderer::new(w, h, aa, 3).with_mode(mode);
    let img = r.render(run.scene, run.camera);
    Ok(HeadlessImage {
        width: w,
        height: h,
        png: cga_gpu::frame_to_png_bytes(&img),
    })
}

/// 长驻场景会话：模块求值一次，之后按帧驱动（宿主输入 / 事件派发），每帧从已提交的
/// 实例树重建 `SceneRun` 并可渲染。
///
/// 这就是"props 监听 + 动态局部重渲染"在宿主侧的落点：输入走 React context（改值只让
/// 消费者重渲染），事件由宿主派发（自定义渲染器不自动派发事件），重建只反映已变化的实例。
pub struct SceneSession {
    react: crate::react::ReactSession,
    css: Option<String>,
    asset_root: String,
    pose: HashMap<String, f64>,
    run: Option<crate::SceneRun>,
    /// 增量构建缓存（档 0）：上一帧的元素树（带产出区间）+ 对象表 + 分组注册表。
    cache: Option<BuildCache>,
    /// 最近一次构建的增量统计。
    stats: BuildStats,
    /// 增量渲染器（档 1）：尺寸/模式匹配时跨帧复用。
    incr: Option<cga_gpu::IncrementalRenderer>,
    /// 碰撞扫描器（C1）：指纹缓存跨帧复用，没变的对象对不重算。
    col_scan: crate::collision::CollisionScan,
}

impl SceneSession {
    pub fn open(
        jsx_src: &str,
        css_src: Option<&str>,
        asset_root: &str,
    ) -> Result<SceneSession, String> {
        SceneSession::open_pose(jsx_src, css_src, asset_root, &[])
    }

    /// 同 [`SceneSession::open`]，并注入 pose 覆盖（`P.x`）。
    pub fn open_pose(
        jsx_src: &str,
        css_src: Option<&str>,
        asset_root: &str,
        pose: &[(String, f64)],
    ) -> Result<SceneSession, String> {
        SceneSession::open_pose_sandbox(
            jsx_src,
            css_src,
            asset_root,
            pose,
            crate::react::Sandbox::from_env(),
        )
    }

    /// 同 [`SceneSession::open_pose`]，并可选沙箱隔离（见 [`crate::react::Sandbox`]）。
    pub fn open_pose_sandbox(
        jsx_src: &str,
        css_src: Option<&str>,
        asset_root: &str,
        pose: &[(String, f64)],
        sandbox: crate::react::Sandbox,
    ) -> Result<SceneSession, String> {
        let js = compile_jsx_with(jsx_src, &mut disk_resolver(asset_root))?;
        Self::open_compiled(js, css_src, asset_root, pose, sandbox)
    }

    /// 同 [`SceneSession::open`]，但 `.jsx` 导入从 `modules`（specifier →
    /// 源码，`"./dial.jsx"` 与 `"dial.jsx"` 等价）解析——内存模块表，
    /// 适合编辑器多标签页，不需要文件系统。
    pub fn open_modules(
        jsx_src: &str,
        css_src: Option<&str>,
        modules: &[(&str, &str)],
    ) -> Result<SceneSession, String> {
        let map: HashMap<&str, &str> = modules.iter().copied().collect();
        let mut resolve = move |spec: &str| {
            let key = spec.strip_prefix("./").unwrap_or(spec);
            map.get(key)
                .map(|s| s.to_string())
                .ok_or_else(|| format!("unknown module {spec}"))
        };
        let js = compile_jsx_with(jsx_src, &mut resolve)?;
        Self::open_compiled(js, css_src, ".", &[], crate::react::Sandbox::from_env())
    }

    /// 共享的会话建立：编译产物 js（不含 PRELUDE）→ 挂载 → 首帧构建。
    fn open_compiled(
        js: String,
        css_src: Option<&str>,
        asset_root: &str,
        pose: &[(String, f64)],
        sandbox: crate::react::Sandbox,
    ) -> Result<SceneSession, String> {
        let module = format!("{PRELUDE}{js}");
        let pose_map: HashMap<String, f64> = pose.iter().cloned().collect();
        let mut react = crate::react::ReactSession::new_with(
            crate::react::Runtime::from_env(),
            crate::react::SessionOptions { sandbox },
        )?;
        react.register_global("solve", 2, solve_host)?;
        // 首帧由帧事务一次完成：挂载 + drain + 快照。
        let out = react.frame(crate::react::FrameAction::Mount {
            src: &module,
            pose: &pose_map,
            sandbox,
        })?;
        if out.stats.errors > 0 {
            let detail = react
                .logs()?
                .into_iter()
                .filter(|l| l.level == "error")
                .map(|l| l.text)
                .collect::<Vec<_>>()
                .join("\n");
            return Err(format!("JSX: 渲染期异常:\n{detail}"));
        }
        let mut sess = SceneSession {
            react,
            css: css_src.map(str::to_string),
            asset_root: asset_root.to_string(),
            pose: pose_map,
            run: None,
            cache: None,
            stats: BuildStats::default(),
            incr: None,
            col_scan: crate::collision::CollisionScan::new(),
        };
        sess.build_from(&out.snapshot)?;
        Ok(sess)
    }

    /// 从一份 `{t,p,c}` 快照构建 `SceneRun`。
    ///
    /// 增量构建（档 0）：带 `__v`/`__s` 版本的快照与上一帧缓存配对，未变且无副作用的
    /// 子树整棵复用上一帧的产出对象；统计见 [`SceneSession::build_stats`]。
    /// CSS 匹配、材质继承、运动学求解对**未复用**的部分照常重跑。
    fn build_from(&mut self, v: &Value) -> Result<(), String> {
        let pose: Vec<(String, f64)> = self.pose.iter().map(|(k, v)| (k.clone(), *v)).collect();
        let (run, tree, stats) = build_scene_run_cached(
            v,
            self.css.as_deref(),
            &self.asset_root,
            &pose,
            self.cache.as_ref(),
        )?;
        self.cache = Some(BuildCache {
            tree,
            objects: run.scene.objects.clone(),
            groups: run.groups.clone(),
        });
        self.stats = stats;
        self.run = Some(run);
        Ok(())
    }

    /// 最近一次构建的增量统计（复用了几棵子树 / 几个对象）。
    pub fn build_stats(&self) -> BuildStats {
        self.stats
    }

    /// 从当前实例树重建 `SceneRun`（CSS 匹配、材质继承、惰性查询都会重跑）。
    pub fn rebuild(&mut self) -> Result<(), String> {
        let dump = self.react.snapshot()?;
        let v: Value =
            serde_json::from_str(&dump).map_err(|e| format!("JSX: 场景序列化失败: {e}"))?;
        self.build_from(&v)
    }

    pub fn run(&self) -> &crate::SceneRun {
        self.run
            .as_ref()
            .expect("SceneSession: rebuild 之前没有场景")
    }

    /// 推入宿主输入（props 监听）→ 重渲染 → 跑到静止 → 重建场景（一帧事务，单次跨线程）。
    pub fn set_input(&mut self, json: &str) -> Result<crate::react::DrainStats, String> {
        let out = self
            .react
            .frame(crate::react::FrameAction::Input { json })?;
        self.build_from(&out.snapshot)?;
        Ok(out.stats)
    }

    /// 事件派发 → 跑到静止 → 重建场景（一帧事务，单次跨线程）。
    pub fn dispatch(
        &mut self,
        id: i64,
        prop: &str,
        payload_json: &str,
    ) -> Result<crate::react::DispatchOutcome, String> {
        let out = self.react.frame(crate::react::FrameAction::Event {
            id,
            prop,
            payload: payload_json,
        })?;
        self.build_from(&out.snapshot)?;
        Ok(out.dispatch.unwrap_or_default())
    }

    pub fn instances(&mut self) -> Result<Vec<crate::react::InstanceInfo>, String> {
        self.react.instances().map_err(String::from)
    }

    /// 当前已提交实例树的 `{t,p,c}` JSON（调试 / 测试用）。
    pub fn snapshot(&mut self) -> Result<String, String> {
        self.react.snapshot().map_err(String::from)
    }

    pub fn counters(&mut self) -> Result<crate::react::Counters, String> {
        self.react.counters().map_err(String::from)
    }

    /// 碰撞扫描（C1）：全对 broad+narrow，同组（非 0 且相等）免检；指纹缓存跨帧
    /// 复用——没变的对象对不重算（命中数见返回后 `collision_scan_stats`）。
    pub fn collisions(&mut self) -> Vec<crate::collision::PairHit> {
        // 直接借字段：run 与 col_scan 是不相交的两个字段。
        let run = self
            .run
            .as_ref()
            .expect("SceneSession: rebuild 之前没有场景");
        self.col_scan.scan(&run.scene)
    }

    /// 上次 `collisions()` 的缓存命中数与窄相计算数。
    pub fn collision_scan_stats(&self) -> (usize, usize) {
        (self.col_scan.cache_hits, self.col_scan.computed)
    }

    pub fn reset_counters(&mut self) -> Result<(), String> {
        self.react.reset_counters().map_err(String::from)
    }

    /// 跑到静止并重建场景（一帧事务）。
    pub fn drain(&mut self) -> Result<crate::react::DrainStats, String> {
        let out = self.react.frame(crate::react::FrameAction::None)?;
        self.build_from(&out.snapshot)?;
        Ok(out.stats)
    }

    pub fn errors(&mut self) -> Result<Vec<String>, String> {
        self.react.errors().map_err(String::from)
    }

    pub fn logs(&mut self) -> Result<Vec<crate::react::LogEntry>, String> {
        self.react.logs().map_err(String::from)
    }

    /// 重建当前场景并渲染。
    pub fn render(
        &mut self,
        w: i32,
        h: i32,
        aa: i32,
        mode: cga_gpu::RenderMode,
    ) -> Result<crate::HeadlessImage, String> {
        self.rebuild()?;
        let run = self.run();
        let mut r = cga_gpu::Renderer::new(w, h, aa, 3).with_mode(mode);
        let img = r.render(run.scene.clone(), run.camera.clone());
        Ok(crate::HeadlessImage {
            width: w,
            height: h,
            png: cga_gpu::frame_to_png_bytes(&img),
        })
    }

    /// 用持久化的增量渲染器渲染**当前**场景（档 1）：只重追值可能变化的光线。
    /// 帧动作（`set_input` / `dispatch` / `drain`）已经重建过 `SceneRun`，
    /// 本方法不再重建。与全帧渲染逐位一致（测试看守）；返回增量统计。
    /// 尺寸 / 模式变化会重建内部渲染器（首帧全量）。
    pub fn render_incremental(
        &mut self,
        w: i32,
        h: i32,
        aa: i32,
        mode: cga_gpu::RenderMode,
    ) -> Result<(crate::HeadlessImage, cga_gpu::IncrementalStats), String> {
        let reuse = match &self.incr {
            Some(r) => r.width() == w && r.height() == h && r.aa() == aa && r.mode() == mode,
            None => false,
        };
        if !reuse {
            self.incr = Some(cga_gpu::IncrementalRenderer::new(w, h, aa, 3).with_mode(mode));
        }
        // 直接借字段：run 与 incr 是不相交的两个字段。
        let run = self
            .run
            .as_ref()
            .expect("SceneSession: rebuild 之前没有场景");
        let r = self.incr.as_mut().expect("incremental renderer");
        let (img, stats) = r.render(&run.scene, &run.camera);
        Ok((
            crate::HeadlessImage {
                width: w,
                height: h,
                png: cga_gpu::frame_to_png_bytes(&img),
            },
            stats,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gallery_collision_scan_baseline() {
        // 画廊 8 场景全对扫描（C1 验收）：pin 住每个场景的 Yes/Unknown 计数。
        // 说明：落在地面上的物体 = 接触（sep=0 → Yes），凸轮/齿轮啮合与装配
        // 穿插也是真实接触；mechanical/assembly 的 Unknown 来自 CSG 差/交与
        // 仿射包装（三值诚实，不许假装精确）。计数变化 = 场景或判定行为变化。
        let want = [
            ("orbit", 4, 4),
            ("grid", 9, 0),
            ("building", 159, 196),
            ("mechanical", 0, 50),
            ("primitives", 5, 12),
            ("affine", 1, 6),
            ("assembly", 0, 5),
            ("animation", 1, 0),
        ];
        for (name, want_yes, want_unknown) in want {
            let jsx = std::fs::read_to_string(format!("../../examples/jsx/{name}.jsx"))
                .unwrap_or_else(|_| panic!("read {name}.jsx"));
            let css = std::fs::read_to_string(format!("../../examples/jsx/{name}.css"))
                .unwrap_or_else(|_| panic!("read {name}.css"));
            let run = run_jsx(&jsx, Some(&css), "../../examples/jsx")
                .unwrap_or_else(|e| panic!("jsx {name}: {e}"));
            let hits = crate::collision::CollisionScan::new().scan(&run.scene);
            let yes = hits
                .iter()
                .filter(|h| h.hit == cga_core::collision::Hit::Yes)
                .count();
            let unknown = hits
                .iter()
                .filter(|h| h.hit == cga_core::collision::Hit::Unknown)
                .count();
            assert_eq!(
                (yes, unknown),
                (want_yes, want_unknown),
                "{name}: 碰撞基线变化（若是刻意改场景，更新本表）"
            );
        }
        // 语义抽查：animation 的太阳球陷入地面 0.05（r=0.6, y=0.55）→ Yes 且 sep=−0.05。
        let jsx = std::fs::read_to_string("../../examples/jsx/animation.jsx").unwrap();
        let css = std::fs::read_to_string("../../examples/jsx/animation.css").unwrap();
        let run = run_jsx(&jsx, Some(&css), "../../examples/jsx").unwrap();
        let hits = crate::collision::CollisionScan::new().scan(&run.scene);
        let h = hits
            .iter()
            .find(|h| h.hit == cga_core::collision::Hit::Yes)
            .expect("animation has one contact");
        assert!((h.separation.unwrap() + 0.05).abs() < 1e-9, "{h:?}");
    }

    #[test]
    fn test_jsx_gallery_smoke() {
        // 画廊八场景（JSX+CSS 版）：全部可渲染出合法 PNG。
        for name in [
            "orbit",
            "grid",
            "building",
            "mechanical",
            "primitives",
            "affine",
            "assembly",
            "animation",
        ] {
            let jsx = std::fs::read_to_string(format!("../../examples/jsx/{name}.jsx"))
                .unwrap_or_else(|_| panic!("read {name}.jsx"));
            let css = std::fs::read_to_string(format!("../../examples/jsx/{name}.css"))
                .unwrap_or_else(|_| panic!("read {name}.css"));
            let out = render_jsx_png(&jsx, Some(&css), "../../examples/jsx", 96, 72, 1)
                .unwrap_or_else(|e| panic!("jsx {name}: {e}"));
            assert!(out.png.starts_with(&[137, 80, 78, 71]), "{name}: PNG magic");
            assert!(out.png.len() > 1000, "{name}: 非空渲染");
        }
    }
    /// 冻结金标：7 个画廊场景的渲染（96×72 aa=1）哈希。
    ///
    /// 迁移到 React 运行时之前，旧的一次性求值路径与新路径已逐位核对（7/7 一致），
    /// 这些哈希即那时的输出。此后它是端到端护栏：只要渲染/场景构建语义不漂移，
    /// 就必须一直吻合（改渲染器或升级内置 React 时这里是第一道警报）。
    /// 取三个代表场景（其余 4 个由 `test_jsx_gallery_smoke` 覆盖可渲染性）：
    /// orbit = 透明/折射，assembly = solve 约束 + tag/when + 嵌套元素属性，
    /// mechanical = CSG。
    const GOLDEN: &[(&str, u64)] = &[
        ("orbit", 0x7c79_87c6_76e2_361c),
        ("mechanical", 0xa1ae_01a8_4b10_c72a),
        ("assembly", 0xadf0_d90d_5db2_7afe),
    ];

    #[test]
    fn gallery_render_golden() {
        for (name, want) in GOLDEN {
            let jsx = std::fs::read_to_string(format!("../../examples/jsx/{name}.jsx")).unwrap();
            let css = std::fs::read_to_string(format!("../../examples/jsx/{name}.css")).unwrap();
            let run = run_jsx(&jsx, Some(&css), "../../examples/jsx")
                .unwrap_or_else(|e| panic!("{name}: {e}"));
            let png = render_png(&run);
            assert_eq!(
                fnv1a(&png),
                *want,
                "{name}: 渲染金标不符（{} 字节）",
                png.len()
            );
        }
    }

    fn render_png(run: &crate::SceneRun) -> Vec<u8> {
        let mut r = cga_gpu::Renderer::new(96, 72, 1, 3);
        let img = r.render(run.scene.clone(), run.camera.clone());
        cga_gpu::frame_to_png_bytes(&img)
    }

    fn fnv1a(bytes: &[u8]) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in bytes {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }

    /// ---- P2：宿主输入（props 监听）与事件派发 → 动态局部重渲染 ----

    const DIAL_SCENE: &str = r#"
const { useContext, useState } = React;
const fixed = <sphere r={1} />;

function Dial() {
  const IN = useContext(HostInput);
  const x = IN.x === undefined ? 0 : IN.x;
  return <translate t={[x, 0, 0]}><sphere r={0.25} /></translate>;
}

function Counter() {
  const [n, setN] = useState(0);
  return (
    <translate t={[0, 2, 0]} onClick={() => setN(n + 1)}>
      <sphere r={0.2 + n * 0.1} />
    </translate>
  );
}

export default <scene><camera /><sphere r={0.5} />{fixed}<Dial /><Counter /></scene>;
"#;

    #[test]
    fn scene_session_host_input_rebuilds_locally() {
        let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
        let xs = |s: &SceneSession| -> Vec<f64> {
            s.run()
                .scene
                .objects
                .iter()
                .map(|o| o.base.motor().to_matrix()[3])
                .collect()
        };
        let before = xs(&s);
        s.reset_counters().unwrap();
        let stats = s.set_input(r#"{"x": 2.0}"#).unwrap();
        assert_eq!(stats.errors, 0);
        let c = s.counters().unwrap();
        assert_eq!(c.create, 0, "输入变化不得新建实例");
        assert!(c.update >= 1, "被消费的输入应让消费者更新: {c:?}");
        let after = xs(&s);
        assert_eq!(after.len(), before.len(), "对象数不变");
        let moved: Vec<usize> = (0..before.len())
            .filter(|i| (before[*i] - after[*i]).abs() > 1e-9)
            .collect();
        assert_eq!(
            moved.len(),
            1,
            "只有 Dial 的球平移: {before:?} -> {after:?}"
        );
        assert!(
            (after[moved[0]] - 2.0).abs() < 1e-9,
            "Dial 应平移到 x=2: {after:?}"
        );
    }

    #[test]
    fn scene_session_dispatch_updates_state_across_frames() {
        let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
        let inst = s
            .instances()
            .unwrap()
            .into_iter()
            .find(|i| i.handlers.iter().any(|h| h == "onClick"))
            .expect("带 onClick 的实例");
        assert!(s.snapshot().unwrap().contains(r#""r":0.2"#), "初值 r=0.2");
        s.reset_counters().unwrap();
        let out = s.dispatch(inst.id, "onClick", "null").unwrap();
        assert!(out.found, "{out:?}");
        assert_eq!(s.counters().unwrap().create, 0, "事件驱动不得新建实例");
        assert!(s.snapshot().unwrap().contains(r#""r":0.3"#), "第一帧 r=0.3");
        assert!(s.dispatch(inst.id, "onClick", "null").unwrap().found);
        assert!(
            s.snapshot().unwrap().contains(r#""r":0.4"#),
            "第二帧 r=0.4（状态跨帧累积）"
        );
        assert_eq!(s.errors().unwrap(), Vec::<String>::new());
        assert!(
            !s.dispatch(999_999, "onClick", "null").unwrap().found,
            "未知实例"
        );
        assert!(
            !s.dispatch(inst.id, "onWheel", "null").unwrap().found,
            "无该处理器"
        );
    }

    /// ---- .jsx 模块导入（编译期打包：globalThis.__exp_N + 别名）----

    fn object_xs(s: &SceneSession) -> Vec<f64> {
        s.run()
            .scene
            .objects
            .iter()
            .map(|o| o.base.motor().to_matrix()[3])
            .collect()
    }

    #[test]
    fn import_component_default() {
        let dial = "export default function Dial(props) {\n\
            \x20 return <translate t={[props.x || 0, 0, 0]}><sphere r={0.5} /></translate>;\n\
            }";
        let entry = "import Dial from './dial.jsx';\n\
            export default (<scene><camera /><Dial x={2} /></scene>);";
        let s = SceneSession::open_modules(entry, None, &[("dial.jsx", dial)]).expect("open");
        let xs = object_xs(&s);
        assert_eq!(xs.len(), 1, "one sphere: {xs:?}");
        assert!((xs[0] - 2.0).abs() < 1e-9, "Dial at x=2: {xs:?}");
    }

    #[test]
    fn import_element_default_and_named() {
        let part = "export default (<translate t={[1, 0, 0]}><box s={[1, 1, 1]} /></translate>);\n\
            export const dot = <translate t={[3, 0, 0]}><sphere r={0.2} /></translate>;";
        let entry = "import part, { dot } from './part.jsx';\n\
            export default (<scene><camera />{part}{dot}</scene>);";
        let s = SceneSession::open_modules(entry, None, &[("part.jsx", part)]).expect("open");
        let xs = object_xs(&s);
        assert_eq!(xs.len(), 2, "{xs:?}");
        assert!(
            (xs[0] - 1.0).abs() < 1e-9 && (xs[1] - 3.0).abs() < 1e-9,
            "{xs:?}"
        );
    }

    #[test]
    fn import_is_transitive_and_shared() {
        let base = "export const unit = <sphere r={0.1} />;";
        let mid = "import { unit } from './base.jsx';\n\
            export default (<translate t={[5, 0, 0]}>{unit}</translate>);";
        let entry = "import mid from './mid.jsx';\n\
            import { unit } from './base.jsx';\n\
            export default (<scene><camera />{mid}{unit}</scene>);";
        let s = SceneSession::open_modules(entry, None, &[("base.jsx", base), ("mid.jsx", mid)])
            .expect("open");
        let xs = object_xs(&s);
        assert_eq!(xs.len(), 2, "mid's unit + entry's unit: {xs:?}");
        assert!((xs[0] - 5.0).abs() < 1e-9 && xs[1].abs() < 1e-9, "{xs:?}");
    }

    #[test]
    fn import_errors_are_jsx_prefixed() {
        // cycle
        let a = "import b from './b.jsx';\nexport default (<sphere r={0.1} />);";
        let b = "import a from './a.jsx';\nexport default (<sphere r={0.2} />);";
        let entry = "import a from './a.jsx';\nexport default (<scene>{a}</scene>);";
        let e = SceneSession::open_modules(entry, None, &[("a.jsx", a), ("b.jsx", b)])
            .err()
            .expect("should fail");
        assert!(e.contains("circular import"), "{e}");

        // missing module
        let e = SceneSession::open_modules(
            "import x from './nope.jsx';\nexport default (<scene>{x}</scene>);",
            None,
            &[],
        )
        .err()
        .expect("should fail");
        assert!(e.starts_with("JSX: cannot import ./nope.jsx"), "{e}");

        // missing default export
        let m = "export const a = 1;";
        let e = SceneSession::open_modules(
            "import x from './m.jsx';\nexport default (<scene />);",
            None,
            &[("m.jsx", m)],
        )
        .err()
        .expect("should fail");
        assert!(e.contains("has no default export"), "{e}");

        // missing named export
        let e = SceneSession::open_modules(
            "import { zz } from './m.jsx';\nexport default (<scene />);",
            None,
            &[("m.jsx", m)],
        )
        .err()
        .expect("should fail");
        assert!(e.contains("does not export zz"), "{e}");
    }

    #[test]
    fn import_supports_default_function_decl() {
        let m = "export default function Tag() {\n\
            \x20 return <translate t={[7, 0, 0]}><sphere r={0.3} /></translate>;\n\
            }";
        let entry =
            "import Tag from './m.jsx';\nexport default (<scene><camera /><Tag /></scene>);";
        let s = SceneSession::open_modules(entry, None, &[("m.jsx", m)]).expect("open");
        let xs = object_xs(&s);
        assert_eq!(xs.len(), 1);
        assert!((xs[0] - 7.0).abs() < 1e-9, "{xs:?}");
    }

    #[test]
    fn scene_session_is_deterministic_across_frames() {
        let mut a = SceneSession::open(DIAL_SCENE, None, ".").unwrap();
        let mut b = SceneSession::open(DIAL_SCENE, None, ".").unwrap();
        a.set_input(r#"{"x": 1.0}"#).unwrap();
        b.set_input(r#"{"x": 1.0}"#).unwrap();
        let ha = render_png(a.run());
        let hb = render_png(b.run());
        assert_eq!(ha, hb, "同输入序列同输出");
    }

    /// ---- 增量构建（档 0）+ 增量渲染（档 1）+ 分组（档 2） ----

    #[test]
    fn incremental_build_reuses_unchanged_subtrees() {
        let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
        let s0 = s.build_stats();
        assert_eq!(s0.reused_objects, 0, "首帧没有可复用的: {s0:?}");
        assert_eq!(s0.total_objects, 4, "scene 里 4 个球: {s0:?}");

        // 输入只驱动 Dial：静态的 3 个球（scene 直属 + fixed + Counter）应整棵复用。
        s.set_input(r#"{"x": 2.0}"#).unwrap();
        let s1 = s.build_stats();
        assert!(
            s1.reused_objects >= 3,
            "未变化的子树应复用上一帧的对象: {s1:?}"
        );
        assert_eq!(s1.total_objects, 4);

        // 正确性：增量构建的产物必须与从头构建同一份快照的产物逐字段一致。
        let snap: Value = serde_json::from_str(&s.snapshot().unwrap()).unwrap();
        let fresh = build_scene_run(&snap, None, ".", &[]).expect("fresh build");
        let cached = s.run();
        assert_eq!(
            format!("{:?}", cached.scene.objects),
            format!("{:?}", fresh.scene.objects),
            "对象表必须逐字段一致"
        );
        assert_eq!(format!("{:?}", cached.tags), format!("{:?}", fresh.tags));
        assert_eq!(
            format!("{:?}", cached.kinematics),
            format!("{:?}", fresh.kinematics)
        );
        assert_eq!(cached.groups, fresh.groups);
    }

    #[test]
    fn incremental_build_disabled_by_sibling_rules() {
        // 样式表里有兄弟组合器：兄弟的 class 变化会改变本节点计算样式但抬不动版本号，
        // 保守起见整个会话关闭子树复用。
        let mut s = SceneSession::open(
            DIAL_SCENE,
            Some("scene > scene ~ scene { color: red; }\nscene > sphere + sphere { color: red; }"),
            ".",
        )
        .expect("open");
        s.set_input(r#"{"x": 2.0}"#).unwrap();
        let st = s.build_stats();
        assert_eq!(st.reused_objects, 0, "兄弟组合器必须关闭复用: {st:?}");
        assert_eq!(st.total_objects, 4);
    }

    #[test]
    fn incremental_build_lazy_query_forces_rebuild() {
        // 惰性查询的字符串引用要查 tag 注册表（树外状态）：含查询的子树永不复用。
        let src = r#"
const { useContext } = React;
function Ball() {
  const IN = useContext(HostInput);
  const x = IN.x === undefined ? 0 : IN.x;
  return <tag name="ball"><translate t={[x, 0, 0]}><sphere r={0.5} /></translate></tag>;
}
export default (
  <scene>
    <camera />
    <Ball />
    <translate t={vadd(center("ball"), [0, 2, 0])}><sphere r={0.3} /></translate>
  </scene>
);
"#;
        let mut s = SceneSession::open(src, None, ".").expect("open");
        s.set_input(r#"{"x": 3.0}"#).unwrap();
        // 正确性：跟随者必须跟到新位置——与从头构建同一份快照逐字段一致。
        let snap: Value = serde_json::from_str(&s.snapshot().unwrap()).unwrap();
        let fresh = build_scene_run(&snap, None, ".", &[]).expect("fresh build");
        assert_eq!(
            format!("{:?}", s.run().scene.objects),
            format!("{:?}", fresh.scene.objects),
            "带惰性查询的场景：增量构建 ≡ 全量构建"
        );
        let m = s.run().scene.objects[1].base.motor().to_matrix();
        assert!(
            (m[3] - 3.0).abs() < 1e-9 && (m[7] - 2.0).abs() < 1e-9,
            "跟随者应到 (3, 2): {m:?}"
        );
        let st = s.build_stats();
        assert_eq!(st.reused_objects, 0, "tag 之下与查询子树都不可复用: {st:?}");
    }

    #[test]
    fn test_jsx_collision_queries() {
        let src = r#"
export default (
  <scene>
    <camera />
    <tag name="a"><sphere r={1} /></tag>
    <tag name="b"><translate t={[3, 0, 0]}><sphere r={1} /></translate></tag>
    <tag name="c"><translate t={[1.5, 0, 0]}><sphere r={1} /></translate></tag>
    <sphere r={clearance("a", "b")} />
    <sphere r={qadd(0.5, collides("a", "c"))} />
    <sphere r={qadd(0.5, collides("a", "b"))} />
    <sphere r={qadd(0.25, inside("a", [0.5, 0, 0]))} />
  </scene>
);
"#;
        let run = run_jsx(src, None, ".").expect("run");
        let radius = |i: usize| match &run.scene.objects[i].geometry {
            cga_core::Geometry::SphereGeometry(s) => s.radius,
            g => panic!("object {i} not a sphere: {g:?}"),
        };
        // 对象 0/1/2 是 tag a/b/c 的球
        assert!((radius(3) - 1.0).abs() < 1e-9, "clearance(a,b) = 3 − 2 = 1");
        assert!((radius(4) - 1.5).abs() < 1e-9, "collides(a,c) = 1（重叠）");
        assert!((radius(5) - 0.5).abs() < 1e-9, "collides(a,b) = 0（分离）");
        assert!((radius(6) - 1.25).abs() < 1e-9, "inside(a, [0.5,0,0]) = 1");
    }

    #[test]
    fn test_jsx_collision_query_unknown_errors() {
        // 环面–环面没有确切分离距离 → Unknown → 构建报错（不许静默给一个数）。
        let src = r#"
export default (
  <scene>
    <camera />
    <tag name="t1"><torus R={1} r={0.25} /></tag>
    <tag name="t2"><translate t={[3, 0, 0]}><torus R={1} r={0.25} /></translate></tag>
    <sphere r={clearance("t1", "t2")} />
  </scene>
);
"#;
        let e = run_jsx(src, None, ".").unwrap_err();
        assert!(e.contains("Unknown"), "{e}");
    }

    #[test]
    fn session_collisions_cache_across_frames() {
        let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
        let hits = s.collisions();
        let (_, computed0) = s.collision_scan_stats();
        assert_eq!(computed0, 6, "4 个对象 → 6 对全算: {computed0}");
        // 原点的 r=0.5 球在 r=1 球内部 → Yes（sep = 0 − 1.5 = −1.5）
        let inner = hits
            .iter()
            .find(|h| h.a == 0 && h.b == 1)
            .expect("pair 0-1");
        assert_eq!(inner.hit, cga_core::collision::Hit::Yes);
        assert!((inner.separation.unwrap() + 1.5).abs() < 1e-9, "{inner:?}");

        // 输入只移动 Dial 的球（对象 2）：含它的 3 对重算，其余 3 对缓存命中
        s.set_input(r#"{"x": 2.0}"#).unwrap();
        s.collisions();
        let (hits2, computed2) = s.collision_scan_stats();
        assert_eq!(computed2, 3, "只有含运动对象的对重算: {computed2}");
        assert_eq!(hits2, 3);
    }

    #[test]
    fn group_prop_and_element() {
        let src = r#"
export default (
  <scene>
    <camera />
    <group name="arm">
      <sphere r={1} />
      <translate t={[3, 0, 0]}><box s={[0.5, 0.5, 0.5]} group="hand" /></translate>
    </group>
    <sphere r={0.5} group="loose" />
  </scene>
);
"#;
        let run = run_jsx(src, None, ".").expect("run");
        assert_eq!(
            run.groups,
            vec![
                "".to_string(),
                "arm".to_string(),
                "hand".to_string(),
                "loose".to_string()
            ],
            "注册表 append-only、按首次出现排序"
        );
        let g = |i: usize| run.scene.objects[i].group;
        assert_eq!(g(0), 1, "arm 的 sphere");
        assert_eq!(g(1), 2, "内层 group prop 覆盖外层 <group>");
        assert_eq!(g(2), 3, "独立的 group prop");
    }

    #[test]
    fn session_render_incremental_is_bitexact() {
        let mut s = SceneSession::open(DIAL_SCENE, None, ".").expect("open");
        let (_img0, st0) = s
            .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
            .expect("render");
        assert!(st0.full && st0.reason == "first", "首帧全量: {st0:?}");

        s.set_input(r#"{"x": 2.0}"#).unwrap();
        let (img1, st1) = s
            .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
            .expect("render");
        assert!(!st1.full, "只有 Dial 动了，应增量渲染: {st1:?}");
        assert!(
            st1.dirty * 2 < st1.total,
            "脏光线应少于一半: {}/{}",
            st1.dirty,
            st1.total
        );

        // 与全帧渲染逐位一致（光线互相独立：子集追踪与全帧追踪同一光线同值）。
        let run = s.run().clone();
        let mut full = cga_gpu::Renderer::new(96, 72, 1, 3);
        let want = full.render(run.scene.clone(), run.camera);
        assert_eq!(
            img1.png,
            cga_gpu::frame_to_png_bytes(&want),
            "增量渲染必须与全帧渲染逐位一致"
        );

        // 无变化帧：一条光线都不重追。
        let (_img2, st2) = s
            .render_incremental(96, 72, 1, cga_gpu::RenderMode::Normal)
            .expect("render");
        assert!(
            !st2.full && st2.dirty == 0 && st2.reason == "clean",
            "{st2:?}"
        );
    }

    #[test]
    fn renders_generated_flange() {
        let text = crate::gen_flange_assembly(
            &crate::FlangeSpec::default(),
            &crate::BoltCircleSpec::default(),
            &crate::GearSpec::default(),
            &crate::BasePlateSpec::default(),
        );
        let out = render_jsx_png(&text, None, ".", 96, 72, 1).expect("flange render");
        assert!(out.png.len() > 1000);
    }

    #[test]
    fn bad_size_errors() {
        assert!(render_jsx_png("export default <scene />;", None, ".", 0, 10, 1).is_err());
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

    // ---- CSS 一致性测试：见 docs/css-conformance.md ----

    fn hx(c: &Color) -> u32 {
        ((c.r * 255.0).round() as u32) << 16
            | ((c.g * 255.0).round() as u32) << 8
            | (c.b * 255.0).round() as u32
    }

    /// objects: [0]=sphere(.a .b) [1]=box(.b) [2]=cylinder
    const TREE: &str = r#"export default (
  <scene>
    <translate>
      <sphere r={0.5} class="a b" />
      <box s={[1, 1, 1]} class="b" />
    </translate>
    <cylinder r={0.2} h={1} />
  </scene>
);"#;

    #[test]
    fn test_css_combinators() {
        let c = |css: &str| run_jsx(TREE, Some(css), "").expect("run");

        // 后代选择器：以前 `div .c` 永远不命中。
        let run = c("scene .a { color: #111111; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x111111);
        assert_eq!(hx(&run.scene.objects[1].material.color), 0xFFFFFF);

        // 子代选择器
        let run = c("translate > .a { color: #222222; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x222222);
        assert_eq!(hx(&run.scene.objects[1].material.color), 0xFFFFFF);

        // 相邻兄弟
        let run = c(".a + .b { color: #333333; }");
        assert_eq!(hx(&run.scene.objects[1].material.color), 0x333333);
        assert_ne!(hx(&run.scene.objects[0].material.color), 0x333333);

        // 普通兄弟
        let run = c(".a ~ * { color: #444444; }");
        assert_eq!(hx(&run.scene.objects[1].material.color), 0x444444);
        assert_ne!(hx(&run.scene.objects[0].material.color), 0x444444);

        // 降级回最后一个 simple selector 的旧行为必须消失：`span .b` 不该命中。
        let run = c(".b { color: #060606; }\nspan .b { color: #040404; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x060606);
        assert_eq!(hx(&run.scene.objects[1].material.color), 0x060606);

        // 复合选择器：多类必须全部命中
        let run = c(".a.b { color: #0E0E0E; }\n.b { color: #060606; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x0E0E0E);
        assert_eq!(hx(&run.scene.objects[1].material.color), 0x060606);

        // 通配符 + 类型选择器大小写不敏感
        let run = c("SPHERE { color: #0F0F0F; }\n* { roughness: 0.75; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x0F0F0F);
        assert!((run.scene.objects[2].material.roughness - 0.75).abs() < 1e-9);
    }

    #[test]
    fn test_css_selector_table() {
        // 每格只有一条规则；期望值按 TREE 的对象顺序：
        // [0]=sphere(.a .b) [1]=box(.b) [2]=cylinder
        let cases: &[(&str, [u32; 3])] = &[
            (".a { color: #010101; }", [0x010101, 0xFFFFFF, 0xFFFFFF]),
            (".b { color: #010101; }", [0x010101, 0x010101, 0xFFFFFF]),
            (".a.b { color: #010101; }", [0x010101, 0xFFFFFF, 0xFFFFFF]),
            (".a.c { color: #010101; }", [0xFFFFFF; 3]),
            (".b.b { color: #010101; }", [0x010101, 0x010101, 0xFFFFFF]),
            ("sphere { color: #010101; }", [0x010101, 0xFFFFFF, 0xFFFFFF]),
            ("SPHERE { color: #010101; }", [0x010101, 0xFFFFFF, 0xFFFFFF]),
            ("* { color: #010101; }", [0x010101; 3]),
            (
                "sphere.a { color: #010101; }",
                [0x010101, 0xFFFFFF, 0xFFFFFF],
            ),
            ("box.b { color: #010101; }", [0xFFFFFF, 0x010101, 0xFFFFFF]),
            ("scene { color: #010101; }", [0x010101; 3]),
            (":root { color: #010101; }", [0x010101; 3]),
            (
                "scene .a { color: #010101; }",
                [0x010101, 0xFFFFFF, 0xFFFFFF],
            ),
            ("scene > .a { color: #010101; }", [0xFFFFFF; 3]),
            (
                "scene > translate { color: #010101; }",
                [0x010101, 0x010101, 0xFFFFFF],
            ),
            (
                "scene > translate > .a { color: #010101; }",
                [0x010101, 0xFFFFFF, 0xFFFFFF],
            ),
            (
                "translate > .a { color: #010101; }",
                [0x010101, 0xFFFFFF, 0xFFFFFF],
            ),
            (
                "translate > .b { color: #010101; }",
                [0x010101, 0x010101, 0xFFFFFF],
            ),
            (
                "translate .b { color: #010101; }",
                [0x010101, 0x010101, 0xFFFFFF],
            ),
            (
                "translate > sphere { color: #010101; }",
                [0x010101, 0xFFFFFF, 0xFFFFFF],
            ),
            ("translate > translate { color: #010101; }", [0xFFFFFF; 3]),
            (
                ".a + .b { color: #010101; }",
                [0xFFFFFF, 0x010101, 0xFFFFFF],
            ),
            (
                ".a ~ .b { color: #010101; }",
                [0xFFFFFF, 0x010101, 0xFFFFFF],
            ),
            (".b + * { color: #010101; }", [0xFFFFFF, 0x010101, 0xFFFFFF]),
            (".b ~ * { color: #010101; }", [0xFFFFFF, 0x010101, 0xFFFFFF]),
            ("cylinder + * { color: #010101; }", [0xFFFFFF; 3]),
            (".a > .b { color: #010101; }", [0xFFFFFF; 3]),
            (".a .b { color: #010101; }", [0xFFFFFF; 3]),
            (
                "[class] { color: #010101; }",
                [0x010101, 0x010101, 0xFFFFFF],
            ),
            (
                "[class=\"b\"] { color: #010101; }",
                [0xFFFFFF, 0x010101, 0xFFFFFF],
            ),
            ("#nope { color: #010101; }", [0xFFFFFF; 3]),
        ];
        for (css, want) in cases {
            let run = run_jsx(TREE, Some(css), "").unwrap_or_else(|e| panic!("css={css}: {e}"));
            let got = [
                hx(&run.scene.objects[0].material.color),
                hx(&run.scene.objects[1].material.color),
                hx(&run.scene.objects[2].material.color),
            ];
            assert_eq!(got, *want, "css={css}");
        }
    }

    #[test]
    fn test_css_attribute_selector() {
        let src = r#"export default <sphere r={1} id="s1" />;"#;
        let run = run_jsx(src, Some("[id=\"s1\"] { color: #0D0D0D; }"), "").expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x0D0D0D);
        let run = run_jsx(src, Some("[id=\"other\"] { color: #0D0D0D; }"), "").expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0xFFFFFF);
        let e = run_jsx(src, Some("[id^=\"s\"] { color: red; }"), "")
            .err()
            .expect("should fail");
        assert!(e.contains("unsupported attribute"), "{e}");
    }

    #[test]
    fn test_css_selector_list_split() {
        // 选择器列表按顶层逗号切：`[class="a,b"]` 里的逗号不能当分隔符。
        let src = r#"export default <sphere r={1} class="a,b" />;"#;
        let run = run_jsx(src, Some("[class=\"a,b\"], #zz { color: #0D0D0D; }"), "").expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x0D0D0D);
        // 尾逗号 → 空选择器，必须报错而不是被吞掉。
        let e = run_jsx(src, Some(".a, { color: red; }"), "")
            .err()
            .expect("should fail");
        assert!(e.contains("CSS:"), "{e}");
    }

    #[test]
    fn test_css_root_scope_only() {
        // `:root` 只命中根：写在后面的 :root 规则不能覆盖子元素的 .b。
        let run = run_jsx(
            TREE,
            Some(".b { color: #060606; }\n:root { background: #2B3138; color: #777777; }"),
            "",
        )
        .expect("run");
        assert_eq!(hx(&run.scene.background), 0x2B3138);
        // box 命中 .b：后写的 :root 规则不能跨过特异性把它覆盖掉
        assert_eq!(hx(&run.scene.objects[1].material.color), 0x060606);
        // cylinder 没有自己的规则 → 从根继承 color
        assert_eq!(hx(&run.scene.objects[2].material.color), 0x777777);
    }

    #[test]
    fn test_css_cascade_order() {
        let with_id = r#"export default <sphere r={1} class="b" id="s1" />;"#;
        // 特异性：#id > .class
        let run = run_jsx(
            with_id,
            Some(".b { color: #060606; }\n#s1 { color: #0A0A0A; }"),
            "",
        )
        .expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x0A0A0A);

        // !important 跨规则压过更高特异性
        let run = run_jsx(
            with_id,
            Some(".b { color: #060606 !important; }\n#s1 { color: #0A0A0A; }"),
            "",
        )
        .expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x060606);

        // 内联 prop > 样式表普通声明
        let run = run_jsx(
            r#"export default <sphere r={1} class="b" color={0x0B0B0B} />;"#,
            Some(".b { color: #060606; }"),
            "",
        )
        .expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x0B0B0B);

        // 样式表 !important > 内联普通声明
        let run = run_jsx(
            r#"export default <sphere r={1} class="b" color={0x0B0B0B} />;"#,
            Some(".b { color: #060606 !important; }"),
            "",
        )
        .expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x060606);

        // 级联矩阵的其余格子
        let src = r#"export default <sphere r={1} class="c" />;"#;
        let one =
            |css: &str| run_jsx(src, Some(css), "").unwrap_or_else(|e| panic!("css={css}: {e}"));
        let color = |run: &crate::SceneRun| hx(&run.scene.objects[0].material.color);

        // .class > tag
        assert_eq!(
            color(&one("sphere { color: #010101; }\n.c { color: #020202; }")),
            0x020202
        );
        // tag > *
        assert_eq!(
            color(&one("* { color: #010101; }\nsphere { color: #020202; }")),
            0x020202
        );
        // 同特异性 → 源码顺序（后写胜）
        assert_eq!(
            color(&one(".c { color: #010101; }\n.c { color: #020202; }")),
            0x020202
        );
        // 同 important 同特异性 → 仍看源码顺序
        assert_eq!(
            color(&one(
                ".c { color: #010101 !important; }\n.c { color: #020202 !important; }"
            )),
            0x020202
        );
        // !important 压过其后的普通规则
        assert_eq!(
            color(&one(
                ".c { color: #010101 !important; }\n.c { color: #020202; }"
            )),
            0x010101
        );
        // #id !important > .class !important
        let with_id = r#"export default <sphere r={1} class="c" id="s1" />;"#;
        let run = run_jsx(
            with_id,
            Some(".c { color: #010101 !important; }\n#s1 { color: #020202 !important; }"),
            "",
        )
        .expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x020202);
    }

    #[test]
    fn test_css_values() {
        let src = |cls: &str| format!("export default <sphere r={{1}} class=\"{cls}\" />;");
        let run_one = |cls: &str, css: &str| run_jsx(&src(cls), Some(css), "").expect("run");

        let run = run_one("c", ".c { color: rgb(255, 0, 0); }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0xFF0000);

        let run = run_one("c", ".c { color: hsl(120, 100%, 50%); }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x00FF00);

        let run = run_one("c", ".c { color: navy; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x000080);

        let run = run_one("c", ".c { color: hwb(120 0% 0%); }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x00FF00);

        let run = run_one("c", ".c { color: #00f; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x0000FF);

        // transparent → RGB 全 0 + alpha 0
        let run = run_one("c", ".c { color: transparent; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x000000);
        assert!(run.scene.objects[0].material.opacity.abs() < 1e-9);

        // 八位十六进制的 alpha → opacity（该元素没有显式 opacity 时）
        let run = run_one("c", ".c { color: #ff000080; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0xFF0000);
        assert!(
            (run.scene.objects[0].material.opacity - 128.0 / 255.0).abs() < 1e-6,
            "alpha 未生效: {}",
            run.scene.objects[0].material.opacity
        );

        // 百分比按 0–1 的键折算
        let run = run_one("c", ".c { opacity: 50%; roughness: 25%; }");
        assert!((run.scene.objects[0].material.opacity - 0.5).abs() < 1e-9);
        assert!((run.scene.objects[0].material.roughness - 0.25).abs() < 1e-9);

        // 关键字
        let run = run_one("c", ".c { unlit: true; }");
        assert_eq!(
            run.scene.objects[0].material.kind,
            cga_gpu::shading::MaterialKind::Basic
        );

        // var() 替换 + fallback
        let run = run_one(
            "c",
            ":root { --brand: #123456; }\n.c { color: var(--brand); }",
        );
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x123456);
        let run = run_one("c", ".c { color: var(--nope, #654321); }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x654321);

        // `--x` 作为材质键别名
        let run = run_one("c", ".c { --roughness: 0.3; }");
        assert!((run.scene.objects[0].material.roughness - 0.3).abs() < 1e-9);

        // 未知属性静默忽略（浏览器行为）
        let run = run_one("c", ".c { color: #135790; width: 10px; }");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x135790);
    }

    #[test]
    fn test_css_value_errors() {
        let src = r#"export default <sphere r={1} class="c" />;"#;
        let e = run_jsx(src, Some(".c { roughness: blorble; }"), "")
            .err()
            .expect("should fail");
        assert!(e.contains("CSS:") && e.contains("blorble"), "{e}");
        let e = run_jsx(src, Some(".c { roughness: 2px; }"), "")
            .err()
            .expect("should fail");
        assert!(e.contains("CSS:") && e.contains("unit"), "{e}");
        let e = run_jsx(src, Some(".c { color: var(--missing); }"), "")
            .err()
            .expect("should fail");
        assert!(e.contains("CSS:") && e.contains("--missing"), "{e}");
        let e = run_jsx(src, Some(".c { color: currentcolor; }"), "")
            .err()
            .expect("should fail");
        assert!(e.contains("CSS:") && e.contains("not supported"), "{e}");
        let e = run_jsx(src, Some(".c { roughness: calc(1 + 1); }"), "")
            .err()
            .expect("should fail");
        assert!(e.contains("CSS:") && e.contains("calc"), "{e}");
    }

    #[test]
    fn test_css_unsupported_constructs_error() {
        let src = r#"export default <sphere r={1} class="c" />;"#;
        let cases: &[(&str, &str)] = &[
            ("a:hover { color: red; }", "pseudo-class :hover"),
            ("a::before { content: \"x\"; }", "pseudo-element"),
            ("@media (min-width: 100px) { .c { color: red; } }", "@media"),
            ("@import url(\"other.css\");", "@import"),
            ("@keyframes spin { from { opacity: 1; } }", "@keyframes"),
        ];
        for (css, want) in cases {
            let e = run_jsx(src, Some(css), "").err().expect("should fail");
            assert!(e.contains("CSS:") && e.contains(want), "css={css} err={e}");
        }
        // 嵌套规则：lightningcss 能解析，但我们不支持 → 必须报错而不是静默丢弃。
        let e = run_jsx(src, Some(".c { color: red; .d { color: blue; } }"), "")
            .err()
            .expect("should fail");
        assert!(e.contains("CSS:"), "{e}");
    }

    #[test]
    fn test_css_inherits_through_containers() {
        let src = r#"export default (
  <scene>
    <material class="tint">
      <sphere r={1} />
      <box s={[1, 1, 1]} />
    </material>
  </scene>
);"#;
        let run = run_jsx(src, Some(".tint { color: #223344; }"), "").expect("run");
        assert_eq!(hx(&run.scene.objects[0].material.color), 0x223344);
        assert_eq!(hx(&run.scene.objects[1].material.color), 0x223344);
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
        let rep =
            crate::scene_report::scene_report(&run.scene, &run.camera, &run.tags, &run.kinematics);
        assert!(rep.contains("joint 0 \"a\" type=revolute"), "{rep}");
        assert!(
            rep.contains("gear 0 driver=\"a\" driven=\"b\" ratio=-0.5"),
            "{rep}"
        );
    }

    #[test]
    fn test_jsx_errors() {
        let e = run_jsx("export default <sphere", None, "")
            .err()
            .expect("should fail");
        assert!(e.starts_with("JSX line 1: "), "{e}");
        let e = run_jsx("export default <frob />;", None, "")
            .err()
            .expect("should fail");
        assert!(e.contains("unknown primitive frob"), "{e}");
        // 带未知参数的未知元素先报参数错。
        let e = run_jsx("export default <frob r={1} />;", None, "")
            .err()
            .expect("should fail");
        assert!(e.contains("frob has no parameter r"), "{e}");
        let e = run_jsx("const a = 1;", None, "")
            .err()
            .expect("should fail");
        assert_eq!(e, "JSX: scene file must end with export default <element>");
    }

    #[test]
    fn test_jsx_solve_and_pose() {
        // solve: 线性方程一步收敛。
        let run = run_jsx(
            r#"const [x] = solve([0.0], [v => eq(v[0], 0.42)]);
export default <translate t={[x, 0, 0]}><sphere r={0.1} /></translate>;"#,
            None,
            "",
        )
        .expect("run");
        let m = run.scene.objects[0].base.motor().to_matrix();
        assert!((m[3] - 0.42).abs() < 1e-6, "solve 应得 x=0.42: {}", m[3]);

        // solve 不收敛 → 显式错误。
        let e = run_jsx(
            r#"const [x] = solve([0.0], [v => v[0] * v[0] + 1]);
export default <sphere r={0.1} />;"#,
            None,
            "",
        )
        .err()
        .expect("should fail");
        assert!(e.contains("did not converge"), "{e}");

        // pose：关节级覆盖 + 报告 pose 行。
        let run = run_jsx_pose(
            r#"export default <joint name="j" type="revolute"><sphere r={0.1} /></joint>;"#,
            None,
            "",
            &[("j".to_string(), 0.7)],
        )
        .expect("run");
        assert!((run.kinematics.joints[0].q[0] - 0.7).abs() < 1e-12);
        let rep =
            crate::scene_report::scene_report(&run.scene, &run.camera, &run.tags, &run.kinematics);
        assert!(rep.contains("pose j=0.7"), "{rep}");

        // 变量级覆盖：P 约定。
        let run = run_jsx_pose(
            r#"export default <translate t={[P.x ?? 0, 0, 0]}><sphere r={0.1} /></translate>;"#,
            None,
            "",
            &[("x".to_string(), 3.0)],
        )
        .expect("run");
        let m = run.scene.objects[0].base.motor().to_matrix();
        assert!((m[3] - 3.0).abs() < 1e-9, "P.x 覆盖应得 3: {}", m[3]);
    }

    #[test]
    fn jsx_sandbox_renders_and_clears_author_globals() {
        // 沙箱模式不影响正常渲染；作者全局在帧后被清理。
        let run = run_jsx_pose_sandbox(
            r#"globalThis.leak = 1; export default <sphere r={1} />;"#,
            None,
            "",
            &[],
            crate::react::Sandbox::On,
        )
        .expect("sandbox run");
        assert_eq!(run.scene.objects.len(), 1, "沙箱下照常渲染");
    }

    #[test]
    fn test_jsx_joint_pair_components() {
        // 低副（平移/旋转）与高副（齿轮）用 React 组件写法，语义与 <joint>/<gear> 相同。
        let run = run_jsx(
            r#"export default (
  <scene>
    <Revolute name="a" axis={[0,0,1]} q={0.4}><sphere r={0.1} /></Revolute>
    <Gear driver="a" driven="b" ratio={-0.5} />
    <Prismatic name="b" axis={[0,0,1]}><box s={[0.1,0.1,0.1]} /></Prismatic>
  </scene>
);"#,
            None,
            "",
        )
        .expect("run");
        let k = &run.kinematics;
        assert_eq!(k.joints.len(), 2);
        assert_eq!(k.joints[0].kind, JointKind::Revolute);
        assert_eq!(k.joints[1].kind, JointKind::Prismatic);
        assert!((k.joints[1].q[0] + 0.2).abs() < 1e-12, "gear 推导 q_b=-0.2");
        assert_eq!(k.joints[0].meshes, vec![0], "Revolute 拥有 mesh 0");
        assert_eq!(k.joints[1].meshes, vec![1], "Prismatic 拥有 mesh 1");

        // 高副 <Cam> 组件转发到 cam 元素（此处验证确实进入了 cam 分支）。
        let e = run_jsx(
            r#"export default <scene><Cam driver="a" driven="b"
                 driverProfile={{kind:"plane",n:[0,1,0],d:0}}
                 drivenProfile={{kind:"plane",n:[0,1,0],d:0}} /></scene>;"#,
            None,
            "",
        )
        .err()
        .expect("should fail");
        assert!(
            e.contains("unknown joint a"),
            "Cam 组件应进入 cam 校验: {e}"
        );
    }

    #[test]
    fn test_jsx_multi_dof_joint_q_arrays() {
        // 多自由度关节的 q 是数组（cylindrical [qr,qp] / spherical / planar [x,y,theta]）。
        let run = run_jsx(
            r#"export default (
  <scene>
    <Cylindrical name="c" axis={[0,0,1]} q={[0.4,0.2]}><sphere r={0.1} /></Cylindrical>
    <Spherical name="s" q={[0.1,0.2,0.3]}><sphere r={0.1} /></Spherical>
    <Planar name="p" axis={[0,0,1]} q={[0.1,0.2,0.3]}><sphere r={0.1} /></Planar>
  </scene>
);"#,
            None,
            "",
        )
        .expect("run");
        let k = &run.kinematics;
        assert_eq!(k.joints[0].q, vec![0.4, 0.2], "cylindrical q");
        assert_eq!(k.joints[1].q, vec![0.1, 0.2, 0.3], "spherical q");
        assert_eq!(k.joints[2].q, vec![0.1, 0.2, 0.3], "planar q");

        // 数量不符仍显式报错。
        let e = run_jsx(
            r#"export default <scene><Cylindrical name="c" q={[1,2,3]} /></scene>;"#,
            None,
            "",
        )
        .err()
        .expect("should fail");
        assert!(e.contains("q must be [qr, qp]"), "{e}");
    }
}
