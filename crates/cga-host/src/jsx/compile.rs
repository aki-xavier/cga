//! swc 编译层：JSX 解析 → `h()` 调用改写 → 模块/导入打包（`Bundler`）→
//! 代码生成。`PRELUDE`（场景预置 + 运动副 React 组件）也在这里拼装。
//! boa 执行在 `session`，场景构建在 `builder`。

use std::collections::HashMap;

use swc_core::common::{sync::Lrc, FileName, Globals, Mark, SourceMap, GLOBALS};
use swc_core::ecma::ast::{
    BindingIdent, CallExpr, Callee, Decl, Expr, ExprOrSpread, Ident, Lit, Module, ModuleDecl,
    ModuleItem, Number, Pat, Stmt, VarDecl, VarDeclKind, VarDeclarator,
};
use swc_core::ecma::codegen::{text_writer::JsWriter, Config as CodegenConfig, Emitter};
use swc_core::ecma::parser::{lexer::Lexer, EsSyntax, Parser, StringInput, Syntax};
use swc_core::ecma::transforms::base::{hygiene, resolver};
use swc_core::ecma::transforms::react::{jsx, Options as ReactOptions, Runtime};
use swc_core::ecma::visit::{VisitMut, VisitMutWith};

/// 场景预置：元素名常量 + 查询辅助 + `h`/`Fragment`（`scene-prelude.js`），
/// 以及运动副 React 组件 `Revolute`/`Prismatic`/`Gear`/`Cam`…（`kinematics-pairs.js`）。
/// 编译期嵌入，按顺序拼接后注入到每个模块源码最前面。
pub(crate) const PRELUDE: &str = concat!(
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

/// E2：动态 `import("./x.jsx")` 打包——字面量 .jsx 规格在编期解析进 bundle
/// （与静态 import 同一套：模块体在加载时求值一次，Promise 只包命名空间）。
/// 非字面量 / 非 .jsx → 编期报错（不许运行时裸奔）。
struct DynImportV<'a, 'b> {
    b: &'a mut Bundler<'b>,
    errs: Vec<String>,
}

impl VisitMut for DynImportV<'_, '_> {
    fn visit_mut_call_expr(&mut self, e: &mut CallExpr) {
        e.visit_mut_children_with(self);
        if !matches!(e.callee, Callee::Import(_)) {
            return;
        }
        let spec = match e.args.as_slice() {
            [ExprOrSpread { spread: None, expr }] => match &**expr {
                Expr::Lit(Lit::Str(s)) => s.value.to_string_lossy().into_owned(),
                _ => {
                    self.errs
                        .push("JSX: dynamic import() needs a string literal".to_string());
                    return;
                }
            },
            _ => {
                self.errs
                    .push("JSX: dynamic import() takes exactly one argument".to_string());
                return;
            }
        };
        if !spec.ends_with(".jsx") {
            self.errs.push(format!(
                "JSX: unsupported dynamic import {spec} (only .jsx literals)"
            ));
            return;
        }
        match self.b.bundle_import(&spec) {
            Ok((id, _)) => {
                let span = e.span;
                // import("…") → __cga_dyn_import(id)
                e.callee = Callee::Expr(Box::new(Expr::Ident(Ident::new_no_ctxt(
                    "__cga_dyn_import".into(),
                    span,
                ))));
                e.args = vec![ExprOrSpread {
                    spread: None,
                    expr: Box::new(Expr::Lit(Lit::Num(Number {
                        span,
                        value: id as f64,
                        raw: None,
                    }))),
                }];
            }
            Err(err) => self.errs.push(err),
        }
    }
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
    fn rewrite(&mut self, mut body: Vec<ModuleItem>, is_entry: bool) -> Result<Rewritten, String> {
        // E2：先打包动态 import()（递归 bundle 在这里发生），再走静态 import 重写。
        {
            let mut v = DynImportV {
                b: self,
                errs: Vec::new(),
            };
            body.visit_mut_with(&mut v);
            if let Some(e) = v.errs.into_iter().next() {
                return Err(e);
            }
        }
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
pub(crate) fn disk_resolver(asset_root: &str) -> impl FnMut(&str) -> Result<String, String> + '_ {
    move |spec| {
        let key = spec.strip_prefix("./").unwrap_or(spec);
        std::fs::read_to_string(format!("{asset_root}/{key}"))
            .map_err(|e| format!("cannot read {key}: {e}"))
    }
}
