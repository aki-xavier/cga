//! 会话层：boa 执行（宿主函数、模块求值）、`run_jsx*` 入口、声明收集、
//! 惰性子树分析与增量缓存、`SceneSession`（拖拽/拾取/渲染事务）。

use std::collections::HashMap;

use boa_engine::Context;
use serde_json::Value;

use cga_scene::{Color, Object, PerspectiveCamera, Scene};

use crate::scene_build::{
    solve_graph, CamDecl, CamProfile, ClosureDecl, GearDecl, GraphDecl, JointKind, PairDecl,
};

use super::builder::{mat4_inv, Builder, BuilderStart};
use super::compile::{compile_jsx_with, disk_resolver, PRELUDE};
use super::css::*;
use super::element::*;

/// boa host: `solve([x0, ...], [v => residual, ...])` — Levenberg-damped
/// Gauss–Newton with a numeric Jacobian (JSX-side constrain). Residuals come
/// from prelude helpers: eq/le/ge.
pub(crate) fn solve_host(
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
/// variables, and matched by name for 1-DOF pairs whose `q` is omitted.
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
pub(crate) fn eval_js_react(
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
        sess.clear_errors()?; // 清掉上一个场景的 React 错误缓冲（池化复用）
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

/// cam profile 解析（`<cam>` 的 aProfile/bProfile）。
pub(crate) fn parse_cam_profile(el: &El, key: &str) -> Result<CamProfile, String> {
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
}

/// Pass 1：收集图声明（docs/kinematics-graph.md；只读标签与 props，不建几何）。
/// 校验的其余部分（未知引用/闭环/孤岛/q/gear/cam）都在 `solve_graph` 里。
pub(crate) fn collect_decl(el: &El, decl: &mut GraphDecl) -> Result<(), String> {
    match el.tag.as_str() {
        "link" => {
            let name = p_str(el, "name")?.ok_or_else(|| "JSX: link needs a name".to_string())?;
            decl.links.push(name);
        }
        "pair" => {
            let kind_err = "JSX: pair.kind must be \"revolute\", \"continuous\", \"prismatic\", \"helical\", \"cylindrical\", \"spherical\", \"planar\" or \"fixed\"";
            let kind = match p_str(el, "kind")?.or(p_str(el, "type")?).as_deref() {
                Some(s) => JointKind::parse(s).ok_or_else(|| kind_err.to_string())?,
                None => return Err(kind_err.to_string()),
            };
            let arity = kind.q_arity();
            let limit = match p_num_list(el, "limit")? {
                Some(l) if l.len() == 2 => Some([l[0], l[1]]),
                Some(l) => return Err(format!("JSX: pair limit must be [lo, hi], got {l:?}")),
                None => None,
            };
            let (q_init, q_given) = match prop(el, "q") {
                None | Some(Value::Null) => (Vec::new(), false),
                Some(Value::Number(_)) => {
                    if arity != 1 {
                        return Err(format!(
                            "JSX: {} pair q must be {}",
                            kind.name(),
                            kind.q_shape()
                        ));
                    }
                    (vec![p_num(el, "q")?.unwrap_or(0.0)], true)
                }
                Some(Value::Array(_)) => {
                    let l = p_num_list(el, "q")?.unwrap_or_default();
                    if l.len() != arity {
                        return Err(format!(
                            "JSX: {} pair q must be {}",
                            kind.name(),
                            kind.q_shape()
                        ));
                    }
                    (l, true)
                }
                Some(v) => {
                    return Err(format!(
                        "JSX: pair q must be a number or number list, got {v}"
                    ))
                }
            };
            let q_guess = match prop(el, "guess") {
                None | Some(Value::Null) => Vec::new(),
                Some(Value::Number(_)) => vec![p_num(el, "guess")?.unwrap_or(0.0)],
                Some(Value::Array(_)) => p_num_list(el, "guess")?.unwrap_or_default(),
                Some(v) => {
                    return Err(format!(
                        "JSX: pair guess must be a number or number list, got {v}"
                    ))
                }
            };
            if !q_guess.is_empty() && q_guess.len() != kind.q_arity() {
                return Err(format!(
                    "JSX: pair guess 分量数必须等于 {} 的自由度数 {}",
                    kind.name(),
                    kind.q_arity()
                ));
            }
            decl.pairs.push(PairDecl {
                name: p_str(el, "name")?,
                kind,
                a: p_str(el, "a")?.ok_or_else(|| "JSX: pair needs a".to_string())?,
                b: p_str(el, "b")?.ok_or_else(|| "JSX: pair needs b".to_string())?,
                at: p_vec3(el, "at")?.unwrap_or([0.0; 3]),
                axis: p_vec3(el, "axis")?.unwrap_or([0.0, 0.0, 1.0]),
                rpy: p_vec3(el, "rpy")?.unwrap_or([0.0; 3]),
                q_init,
                q_guess,
                q_given,
                pitch: p_num(el, "pitch")?,
                limit,
            });
        }
        "anchor" => {
            let l = p_str(el, "link")?.ok_or_else(|| "JSX: anchor needs link".to_string())?;
            if decl.anchor.replace(l).is_some() {
                return Err("JSX: anchor 至多一个".to_string());
            }
        }
        "gear" => {
            decl.gears.push(GearDecl {
                a: p_str(el, "a")?.ok_or_else(|| "JSX: gear needs a".to_string())?,
                b: p_str(el, "b")?.ok_or_else(|| "JSX: gear needs b".to_string())?,
                ratio: p_num(el, "ratio")?.ok_or_else(|| "JSX: gear needs ratio".to_string())?,
                offset: p_num(el, "offset")?.unwrap_or(0.0),
            });
        }
        "cam" => {
            decl.cams.push(CamDecl {
                a: p_str(el, "a")?.ok_or_else(|| "JSX: cam needs a".to_string())?,
                b: p_str(el, "b")?.ok_or_else(|| "JSX: cam needs b".to_string())?,
                a_profile: parse_cam_profile(el, "aProfile")?,
                b_profile: parse_cam_profile(el, "bProfile")?,
            });
        }
        "closure" => {
            decl.closures.push(ClosureDecl {
                a: p_str(el, "a")?.ok_or_else(|| "JSX: closure needs a".to_string())?,
                b: p_str(el, "b")?.ok_or_else(|| "JSX: closure needs b".to_string())?,
                at: p_vec3(el, "at")?.unwrap_or([0.0; 3]),
                b_at: p_vec3(el, "bAt")?.unwrap_or([0.0; 3]),
                axis: p_vec3(el, "axis")?.unwrap_or([0.0, 0.0, 1.0]),
            });
        }
        "joint" => {
            return Err(
                "JSX: <joint> 已被 <link>/<pair>/<anchor> 取代（运动学图模型，见 docs/kinematics-graph.md）"
                    .to_string(),
            );
        }
        _ => {}
    }
    for c in &el.children {
        collect_decl(c, decl)?;
    }
    Ok(())
}

// ---- 增量构建（档 0）-------------------------------------------------------

/// 有全局副作用或依赖全局状态的元素：永不作为复用锚点，且其存在使祖先子树不纯。
/// 不在表内的非特殊标签一律是图元（纯）。`drill` 只依赖自身 props 与 ctx，是纯的。
pub(crate) const SIDE_EFFECT_TAGS: &[&str] = &[
    "ambient_light",
    "directional_light",
    "point_light",
    "camera",
    "background",
    "tag",
    "link",
    "pair",
    "anchor",
    "closure",
    "gear",
    "cam",
    "instances",
    "when",
];

/// 动态上下文（`pending_tags` / `cur_link` / tag 注册表查询）：处于这些元素
/// 之下的子树即使内容未变也不能复用——它们的产出还依赖树外的状态。
pub(crate) const DYNAMIC_TAGS: &[&str] = &["tag", "link", "when"];

/// props 里（递归）是否含惰性查询 `{__q: …}`：字符串引用（`center("ball")` 等）
/// 要查 tag 注册表——那是树外状态，版本号抬不动它。含查询的元素不纯。
pub(crate) fn has_lazy_query(v: &Value) -> bool {
    match v {
        Value::Object(m) => m.contains_key("__q") || m.values().any(has_lazy_query),
        Value::Array(a) => a.iter().any(has_lazy_query),
        _ => false,
    }
}

/// 子树是否"纯"：不含任何副作用元素，产出只取决于自身 props、祖先变换与继承样式。
/// `scene` 带 `background` prop 时也算有副作用（写 `scene.background`）。
/// 带 `tag` prop 的元素有注册表副作用（公共属性 RFC 后与 <tag> 元素同义）。
pub(crate) fn pure_subtree(el: &El) -> bool {
    if SIDE_EFFECT_TAGS.contains(&el.tag.as_str()) {
        return false;
    }
    if el.tag == "scene" && prop(el, "background").is_some() {
        return false;
    }
    if prop(el, "tag").is_some() {
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
pub(crate) fn annotate_reuse(
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
pub(crate) fn apply_ranges(
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

pub(crate) fn shift_child_ranges(new: &mut El, prev: &El, delta: i64) {
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
    object_instances: Vec<Option<i64>>,
    mass_props: Vec<Option<cga_mesh::MassProps>>,
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

pub(crate) fn build_scene_run_cached(
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
    let cam: Option<PerspectiveCamera> = None;
    // 图模型（docs/kinematics-graph.md）：pass 1 收集声明 → 图求解 → pass 2 注入。
    let mut decl = GraphDecl::default();
    collect_decl(&root, &mut decl)?;
    let pose_map: HashMap<String, f64> = pose.iter().cloned().collect();
    let solution = solve_graph(&decl, &pose_map)?;
    let link_worlds: HashMap<String, [f64; 16]> = decl
        .links
        .iter()
        .cloned()
        .zip(solution.link_world.iter().copied())
        .collect();
    let b = Builder::start(BuilderStart {
        asset_root,
        link_worlds,
        solution_pairs: solution.pairs,
        solution_cams: solution.cams,
        solution_closures: solution.closures,
        rules,
        groups: cache
            .map(|c| c.groups.clone())
            .unwrap_or_else(|| vec![String::new()]),
        prev_objects: cache.map(|c| c.objects.as_slice()),
        prev_object_instances: cache.map(|c| c.object_instances.as_slice()),
        prev_mass_props: cache.map(|c| c.mass_props.as_slice()),
    });
    let (run, ranges, stats) = b.build_all(&root, scene, cam, pose)?;
    // 把本帧各节点的产出区间写回树，作为下一帧的复用缓存。
    apply_ranges(&mut root, cache.map(|c| &c.tree), &ranges);
    Ok((run, root, stats))
}

/// 由 `{t,p,c}` 树构建场景（CSS 匹配、材质继承、惰性查询解析都在这里）。
pub(crate) fn build_scene_run(
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
    /// 拖拽（U1）写回的 pair 名：这些 pose 项是上一轮 drag 的产物，下一次
    /// drag 仍把它们当自由变量（作者/pose 钉的则始终固定）。
    drag_owned: std::collections::HashSet<String>,
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
            drag_owned: std::collections::HashSet::new(),
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
            object_instances: run.object_instances.clone(),
            mass_props: run.mass_props.clone(),
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

    /// 拾取（交互闭环，docs/roadmap.md §3.A）：像素 (x, y) 的最近命中 +
    /// 对象来自的 React 宿主实例 id。
    pub fn pick(&self, x: f64, y: f64, w: i32, h: i32) -> Option<(cga_gpu::PickHit, Option<i64>)> {
        let run = self.run();
        let hit = cga_gpu::pick(&run.scene, &run.camera, x, y, w, h)?;
        let inst = run.object_instances.get(hit.object).copied().flatten();
        Some((hit, inst))
    }

    /// 点击：拾取 → 派发 `onClick`（payload 带命中点/法线/对象下标）→ 重建
    /// （一帧事务）。未命中或命中链上没有处理器 → `Ok(None)`。
    pub fn click(
        &mut self,
        x: f64,
        y: f64,
        w: i32,
        h: i32,
    ) -> Result<Option<crate::react::DispatchOutcome>, String> {
        let Some((hit, inst)) = self.pick(x, y, w, h) else {
            return Ok(None);
        };
        let Some(id) = inst else { return Ok(None) };
        let payload = serde_json::json!({
            "point": hit.point,
            "normal": hit.normal,
            "t": hit.t,
            "object": hit.object,
        });
        Ok(Some(self.dispatch(id, "onClick", &payload.to_string())?))
    }

    /// 拖拽 q 写回（drag / drag_pose 共用）：pose 覆盖 → 重建一帧；重建失败
    /// 回滚 pose 保持会话一致。成功的 pair 记入 `drag_owned`（连续拖拽仍自由）。
    fn apply_drag_q(&mut self, solved: &[(String, f64)]) -> Result<(), String> {
        let old: Vec<(String, Option<f64>)> = solved
            .iter()
            .map(|(n, _)| (n.clone(), self.pose.get(n).copied()))
            .collect();
        for (n, q) in solved {
            self.pose.insert(n.clone(), *q);
        }
        if let Err(e) = self.rebuild() {
            for (n, v) in old {
                match v {
                    Some(v) => {
                        self.pose.insert(n, v);
                    }
                    None => {
                        self.pose.remove(&n);
                    }
                }
            }
            let _ = self.rebuild();
            return Err(e);
        }
        for (n, _) in solved {
            self.drag_owned.insert(n.clone());
        }
        Ok(())
    }

    /// 抓取拖拽（U1，docs/ue58-inspirations.md §3.U1）：把 link 上当前位于世界点
    /// `from` 的点解到世界点 `to`——anchor→link 路径上自由 1-DOF pair 的 q 经
    /// pose 通道写回并重建场景（一帧）。同一机构可连续拖拽：上轮写回的 q 仍是
    /// 本轮的自由变量。不可达 / 不收敛 / 越限 / V1 边界（cam·gear·闭链·多 DOF，
    /// 见 `drag_solve` 文档）→ Err，场景与 pose 保持不变。
    pub fn drag(
        &mut self,
        link: &str,
        from: [f64; 3],
        to: [f64; 3],
    ) -> Result<crate::scene_build::DragSolved, String> {
        let solved = {
            let run = self.run();
            let l = run
                .kinematics
                .links
                .iter()
                .find(|l| l.name == link)
                .ok_or_else(|| format!("JSX: drag 引用未知 link {link}"))?;
            // 世界点 → link 局部系（解算器在图求解位姿系里工作）。
            let grab_local = cga_core::transform_point(mat4_inv(l.world), from);
            crate::scene_build::drag_solve(&run.kinematics, link, grab_local, to, &self.drag_owned)?
        };
        self.apply_drag_q(&solved.solved)?;
        Ok(solved)
    }

    /// 位姿抓取拖拽（U1 借清单第 1 件）：把 link 的 frame 拖到目标位姿 `to`
    /// （4×4 齐次矩阵，旋转部分必须正规）。残差 = 位置差 + rotvec(R_c·R_tᵀ)
    /// （motor 对数形式，无轴叉积的对跖退化）。写回/回滚/连续拖拽与
    /// [`SceneSession::drag`] 同一套。
    pub fn drag_pose(
        &mut self,
        link: &str,
        to: [f64; 16],
    ) -> Result<crate::scene_build::DragPoseSolved, String> {
        let solved = {
            let run = self.run();
            if !run.kinematics.links.iter().any(|l| l.name == link) {
                return Err(format!("JSX: drag_pose 引用未知 link {link}"));
            }
            crate::scene_build::drag_pose_solve(&run.kinematics, link, to, &self.drag_owned)?
        };
        self.apply_drag_q(&solved.solved)?;
        Ok(solved)
    }

    /// 像素抓取拖拽：拾取 → 命中对象所属 link → [`SceneSession::drag`]
    /// （抓取点 = 命中点）。未命中或命中对象不在任何 link 下 → Ok(None)。
    pub fn drag_pick(
        &mut self,
        x: f64,
        y: f64,
        w: i32,
        h: i32,
        to: [f64; 3],
    ) -> Result<Option<crate::scene_build::DragSolved>, String> {
        let Some((hit, link)) = ({
            let run = self.run();
            cga_gpu::pick(&run.scene, &run.camera, x, y, w, h).map(|hit| {
                let link = run
                    .kinematics
                    .links
                    .iter()
                    .find(|l| l.meshes.contains(&hit.object))
                    .map(|l| l.name.clone());
                (hit, link)
            })
        }) else {
            return Ok(None);
        };
        let Some(link) = link else { return Ok(None) };
        Ok(Some(self.drag(&link, hit.point, to)?))
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

    /// 诊断渲染（U3，docs/ue58-inspirations.md §3.U3）：headless 引擎的"视口"——
    /// 对象 id 调色板 / 相机系法线 / 深度三通道之一出图。主光线求交副产品，
    /// 逐位确定（同输入同输出，GPU 侧测试看守）。
    pub fn render_diagnostic(
        &mut self,
        w: i32,
        h: i32,
        channel: cga_gpu::DiagChannel,
    ) -> Result<crate::HeadlessImage, String> {
        self.rebuild()?;
        let run = self.run();
        let img = cga_gpu::render_diagnostic(&run.scene, &run.camera, w, h, channel);
        Ok(crate::HeadlessImage {
            width: w,
            height: h,
            png: cga_gpu::frame_to_png_bytes(&img),
        })
    }

    /// 薄透镜景深渲染（U6，docs/ue58-inspirations.md §3.U6）：`aperture` 光圈
    /// 半径（0 = 针孔，与普通渲染逐位一致）、`focal` 对焦距离。确定性 Vogel
    /// 光圈采样。全帧渲染（不走增量渲染器——dof 与帧间指纹的交互另行立项）。
    pub fn render_dof(
        &mut self,
        w: i32,
        h: i32,
        aa: i32,
        aperture: f64,
        focal: f64,
    ) -> Result<crate::HeadlessImage, String> {
        self.rebuild()?;
        let run = self.run();
        let mut r = cga_gpu::Renderer::new(w, h, aa, 3).with_dof(aperture, focal);
        let img = r.render(run.scene.clone(), run.camera.clone());
        Ok(crate::HeadlessImage {
            width: w,
            height: h,
            png: cga_gpu::frame_to_png_bytes(&img),
        })
    }
}
