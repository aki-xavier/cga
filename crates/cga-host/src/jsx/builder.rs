//! 场景构建层：`Builder` 把 `El` 树走成 `SceneRun`（几何/材质/运动学/标签/
//! 增量复用）。底层几何与材质构造复用 `crate::scene_build`。

use std::collections::HashMap;

use serde_json::Value;

use cga_core::Multivector;
use cga_gpu::shading::Light;
use cga_scene::Color;

use crate::scene_build::{
    ArgValue, Builders, CamSolved, ClosureSolved, GearRel, Kinematics, LinkDef, PairDef,
    TagInstance, TagRegistry,
};
use cga_scene::{Object, ObjectParams, PerspectiveCamera, Scene};

use super::css::*;
use super::element::*;
use super::session::BuildStats;

// ---- scene builder ----

/// 一次 `build_all` 的产物：`SceneRun` + 本帧各节点的对象区间（写回缓存树
/// 用，指针键 =元素身份）+ 增量统计。
pub(crate) type BuiltRun = (
    crate::SceneRun,
    HashMap<*const El, std::ops::Range<usize>>,
    BuildStats,
);

/// 构造参数（一次构建的全部外部输入）：图解、缓存与规则。
pub(crate) struct BuilderStart<'p> {
    pub asset_root: &'p str,
    pub link_worlds: HashMap<String, [f64; 16]>,
    pub solution_pairs: Vec<PairDef>,
    pub solution_cams: Vec<CamSolved>,
    pub solution_closures: Vec<ClosureSolved>,
    pub rules: Vec<StyleRule>,
    pub groups: Vec<String>,
    pub prev_objects: Option<&'p [Object]>,
    pub prev_object_instances: Option<&'p [Option<i64>]>,
    pub prev_mass_props: Option<&'p [Option<cga_mesh::MassProps>]>,
}

pub(crate) struct Builder<'p> {
    loader: Builders,
    kin: Kinematics,
    /// 图模型（docs/kinematics-graph.md）：图求解注入的连杆位姿与求解记录。
    link_worlds: HashMap<String, [f64; 16]>,
    pair_records: Vec<PairDef>,
    cam_records: Vec<CamSolved>,
    closure_records: Vec<ClosureSolved>,
    pair_out: usize,
    cam_out: usize,
    closure_out: usize,
    /// 当前正在发射的 link（`kin.links` 下标；发射与复用都登记 meshes）。
    cur_link: Option<usize>,
    /// 与 scene.objects 平行：每个对象来自的 React 宿主实例 id（拾取 → 事件派发）。
    object_instances: Vec<Option<i64>>,
    /// 与 scene.objects 平行：质量属性（`density` prop 链上的对象才有）。
    mass_props: Vec<Option<cga_mesh::MassProps>>,
    /// 密度继承通道（prop `density` 沿子树下传）。
    density_stack: Vec<f64>,
    tags: TagRegistry,
    pending_tags: Vec<(String, [f64; 16])>,
    rules: Vec<StyleRule>,
    /// 分组注册表：id → 名字，0 = "" 未分组。append-only，跨帧稳定（复用的对象带旧 id）。
    groups: Vec<String>,
    group_stack: Vec<u32>,
    /// 增量构建：上一帧的对象表（`annotate_reuse` 标注的复用区间从这里克隆）。
    prev_objects: Option<&'p [Object]>,
    /// 上一帧的对象→实例映射（复用区间随同克隆）。
    prev_object_instances: Option<&'p [Option<i64>]>,
    /// 上一帧的质量属性（复用区间随同克隆）。
    prev_mass_props: Option<&'p [Option<cga_mesh::MassProps>]>,
    /// 本帧每个节点的产出区间（构建结束后写回缓存树，供下一帧复用）。
    ranges: HashMap<*const El, std::ops::Range<usize>>,
    reused_subtrees: usize,
    reused_objects: usize,
}

pub(crate) fn mat4_mul(a: [f64; 16], b: [f64; 16]) -> [f64; 16] {
    cga_core::mat4_mul(a, b)
}

pub(crate) fn translate4(t: [f64; 3]) -> [f64; 16] {
    [
        1.0, 0.0, 0.0, t[0], 0.0, 1.0, 0.0, t[1], 0.0, 0.0, 1.0, t[2], 0.0, 0.0, 0.0, 1.0,
    ]
}

pub(crate) fn scale_matrix(v: [f64; 3]) -> [f64; 16] {
    let mut m = cga_core::mat4_identity();
    m[0] = v[0];
    m[5] = v[1];
    m[10] = v[2];
    m
}

/// 组合变换的可逆性：线性部分 |det| < 1e-15 时 cga-core 的 `AffineGeometry::new`
/// 直接 panic（几何被压成零体积）。宿主在三条路径的矩阵组合点先给可读错误。
pub(crate) fn check_invertible(tag: &str, m: &[f64; 16]) -> Result<(), String> {
    let det = m[0] * (m[5] * m[10] - m[6] * m[9]) - m[1] * (m[4] * m[10] - m[6] * m[8])
        + m[2] * (m[4] * m[9] - m[5] * m[8]);
    if det.abs() < 1e-15 {
        return Err(format!(
            "JSX: <{tag}> transform is singular (linear |det|={det:.3e} < 1e-15) — \
             geometry would flatten to zero volume (check scale/mirror)"
        ));
    }
    Ok(())
}

pub(crate) fn mirror_matrix(axis: [f64; 3]) -> Result<[f64; 16], String> {
    let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    if n < 1e-12 {
        return Err("JSX: mirror axis must be nonzero".to_string());
    }
    let (x, y, z) = (axis[0] / n, axis[1] / n, axis[2] / n);
    Ok([
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
    ])
}

/// 已删除的修饰符元素 → 统一报错（渲染 / CSG / 查询三条路径都经 `build_geo`）。
pub(crate) fn deleted_modifier_tag(tag: &str) -> Option<String> {
    let hint = match tag {
        "translate" => "t=[x, y, z]",
        "rotate" => "rotate=[ax, ay, az, angle]",
        "scale" => "scale=数 | [x, y, z]",
        "mirror" => "mirror=[x, y, z]",
        _ => return None,
    };
    Some(format!(
        "JSX: <{tag}> element was removed — use the transform prop {hint} instead \
         (composition order is fixed T·R·S·Mirror, see docs/jsx-css-host.md §2)"
    ))
}

/// CSG 构造前的宿主侧校验：`CsgGeometry::new` 需要 ≥2 子，违反会在 cga-core 里
/// panic。渲染 / CSG 收集 / 查询三条路径都必须先在这里拦下（union 不开例外——
/// 它同样走 `CsgGeometry::new`）。
pub(crate) fn csg_children_check(tag: &str, n: usize) -> Result<(), String> {
    if n < 2 {
        return Err(format!("JSX: {tag} needs >= 2 geometry children, got {n}"));
    }
    Ok(())
}

/// 不产生几何的元素出现在几何位置（CSG 子、查询目标）→ 统一可读报错，
/// 而不是让底层 builder 报 `unknown primitive camera` 之类。
pub(crate) fn non_geometry_tag(tag: &str) -> Option<String> {
    let hint = match tag {
        "scene" => "the scene root cannot appear inside geometry",
        "camera" | "background" | "ambient_light" | "directional_light" | "point_light" => {
            "camera and lights do not produce geometry"
        }
        "pair" | "anchor" | "gear" | "cam" | "closure" => {
            "kinematics elements are graph edges/nodes, not geometry"
        }
        "link" => "<link> geometry comes from the solved graph and cannot be nested here",
        "joint" => "<joint> was replaced by <link>/<pair>/<anchor> — see docs/kinematics-graph.md",
        _ => return None,
    };
    Some(format!("JSX: <{tag}> is not geometry — {hint}"))
}

/// CSG 子树里"无处可去"的通道：`<difference>` 等合并成**一个**对象，只能有一个
/// 材质、一个分组、一份质量属性，所以子元素上的材质 / 样式（CSS 只作用于材质）/
/// 分组 / density 落不到任何地方——报错，不许静默丢。
pub(crate) fn csg_dropped_channel(tag: &str, props: &HashMap<String, Value>) -> Option<String> {
    let where_to = "put it on the <difference>/<union>/<intersection> element itself";
    for k in MATERIAL_KEYS {
        if props.contains_key(k) {
            return Some(format!(
                "JSX: <{tag}> has material prop \"{k}\" inside a CSG — a CSG emits a single \
                 material; {where_to}"
            ));
        }
    }
    for k in ["class", "className", "id"] {
        if props.contains_key(k) {
            return Some(format!(
                "JSX: <{tag}> has \"{k}\" inside a CSG — CSS/`id` only style materials and a \
                 CSG emits a single material; {where_to}"
            ));
        }
    }
    if props.contains_key("group") {
        return Some(format!(
            "JSX: <{tag}> has group inside a CSG — a CSG emits a single object; {where_to}"
        ));
    }
    if props.contains_key("density") {
        return Some(format!(
            "JSX: <{tag}> has density inside a CSG — mass props belong to the emitted \
             object; {where_to}"
        ));
    }
    None
}

/// 惰性查询取参数：JS 侧值为 `undefined` 时 JSON 序列化会**丢掉该键**，
/// 缺键必须报错，不能用 `o["k"]` 索引 panic。
pub(crate) fn q_arg<'a>(
    o: &'a serde_json::Map<String, Value>,
    k: &str,
    q: &str,
) -> Result<&'a Value, String> {
    o.get(k).ok_or_else(|| {
        format!("JSX: query {q}(…) is missing its \"{k}\" argument (was it undefined?)")
    })
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
        src: Option<i64>,
    ) {
        let (motor, lin) = cga_core::decompose_rigid(world);
        let g2 = if cga_scene::is_identity3(lin) {
            geo.clone()
        } else {
            cga_core::Geometry::AffineGeometry(cga_core::AffineGeometry::new(geo.clone(), lin))
        };
        let group = self.group_stack.last().copied().unwrap_or(0);
        // 质量属性（C1）：density 链上的对象才有；CSG/仿射走网格积分（比发射慢，
        // 只在带 density 时付出）。在对象构造前算（g2 随后被移走）。
        let mp = self
            .density_stack
            .last()
            .and_then(|&d| cga_mesh::mass_properties(&g2, world, d));
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
        self.object_instances.push(src);
        self.mass_props.push(mp);
        let idx = scene.objects.len() - 1;
        if let Some(li) = self.cur_link {
            self.kin.links[li].meshes.push(idx);
        }
        self.register_tags(&geo, world, world);
    }

    /// 入口：压入位置帧、算出本元素的计算样式，再分派。
    pub(crate) fn start(s: BuilderStart<'p>) -> Builder<'p> {
        Builder {
            loader: Builders::new(s.asset_root),
            kin: Kinematics::default(),
            link_worlds: s.link_worlds,
            pair_records: s.solution_pairs,
            cam_records: s.solution_cams,
            closure_records: s.solution_closures,
            pair_out: 0,
            cam_out: 0,
            closure_out: 0,
            cur_link: None,
            tags: TagRegistry::new(),
            pending_tags: Vec::new(),
            rules: s.rules,
            groups: s.groups,
            group_stack: Vec::new(),
            prev_objects: s.prev_objects,
            prev_object_instances: s.prev_object_instances,
            prev_mass_props: s.prev_mass_props,
            object_instances: Vec::new(),
            mass_props: Vec::new(),
            density_stack: Vec::new(),
            ranges: HashMap::new(),
            reused_subtrees: 0,
            reused_objects: 0,
        }
    }

    /// 走完整棵元素树，产出 `SceneRun` + 本帧各节点的对象区间 + 增量统计。
    /// 字段不外泄：调用方只拿 [`BuildStats`] 与 ranges（写回缓存树用）。
    pub(crate) fn build_all(
        mut self,
        root: &El,
        mut scene: Scene,
        mut cam: Option<PerspectiveCamera>,
        pose: &[(String, f64)],
    ) -> Result<BuiltRun, String> {
        let mut stack = Vec::new();
        self.walk(
            Frame::new(root, 0),
            &mut stack,
            cga_core::mat4_identity(),
            &HashMap::new(),
            &mut scene,
            &mut cam,
        )?;
        let stats = BuildStats {
            reused_subtrees: self.reused_subtrees,
            reused_objects: self.reused_objects,
            total_objects: scene.objects.len(),
        };
        let mut kin = self.kin;
        let mut ps: Vec<(String, f64)> = pose.to_vec();
        ps.sort_by(|a, b| a.0.cmp(&b.0));
        kin.pose = ps;
        let camera = cam.unwrap_or_else(|| {
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
        Ok((
            crate::SceneRun {
                scene,
                camera,
                tags: std::mem::take(&mut self.tags),
                kinematics: kin,
                groups: std::mem::take(&mut self.groups),
                object_instances: std::mem::take(&mut self.object_instances),
                mass_props: std::mem::take(&mut self.mass_props),
            },
            self.ranges,
            stats,
        ))
    }

    pub(crate) fn walk<'a>(
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
            scene.objects.extend_from_slice(&prev[range.clone()]);
            // 对象→实例映射随同复用（__id 跨帧稳定——实例只在创建时分配 id）。
            if let Some(ids) = self.prev_object_instances {
                self.object_instances.extend_from_slice(&ids[range.clone()]);
            }
            if let Some(mp) = self.prev_mass_props {
                self.mass_props.extend_from_slice(&mp[range.clone()]);
            }
            self.reused_subtrees += 1;
            self.reused_objects += scene.objects.len() - start;
            // 复用路径也要登记当前 link 的 meshes（对象换了下标区间）。
            if let Some(li) = self.cur_link {
                for idx in start..scene.objects.len() {
                    self.kin.links[li].meshes.push(idx);
                }
            }
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
        // 公共属性（2026-10-07 RFC）：变换 prop（t/rotate/scale/mirror，
        // 固定顺序 T·R·S·Mirror）+ tag prop（注册表命名）。修饰符元素
        // （translate/rotate/scale/mirror）已删除——变换只能写成 prop。
        // <link> 也豁免：它的 frame 来自图求解（link_el 自己处理变换与 tag prop）。
        let local = if el.tag == "link" {
            None
        } else {
            self.local_matrix(el)?
        };
        let ctx = match local {
            Some(m) => mat4_mul(ctx, m),
            None => ctx,
        };
        check_invertible(&el.tag, &ctx)?;
        // 分组（档 2）：`<group name>` 元素或任意元素的 `group` prop，
        // 子树产出的对象都打上这个分组 id。无名的 <group> 是透明容器
        // （共享变换/tag/材质 prop 的落点）。
        let group = if el.tag == "group" {
            p_str(el, "name")?
        } else {
            p_str(el, "group")?
        };
        let gid = group.map(|name| self.group_id(&name));
        if let Some(g) = gid {
            self.group_stack.push(g);
        }
        // 密度 prop（C1）：沿子树下传，emit 时计算质量属性。
        let density = p_num(el, "density")?;
        if let Some(dv) = density {
            self.density_stack.push(dv);
        }
        // tag prop：与 <tag name> 元素同语义，注册框架取本元素的 frame
        // （自身变换 prop 之后）。<link> 的 tag prop 由 link_el 处理。
        let tag = if el.tag == "link" {
            None
        } else {
            p_str(el, "tag")?
        };
        let has_tag = tag.is_some();
        if let Some(name) = tag {
            self.pending_tags.push((name, ctx));
        }
        let r = self.walk_inner(el, stack, ctx, &style, scene, cam);
        if density.is_some() {
            self.density_stack.pop();
        }
        if has_tag {
            self.pending_tags.pop();
        }
        if gid.is_some() {
            self.group_stack.pop();
        }
        r
    }

    /// 公共变换 prop → 局部矩阵。固定合成顺序 **T·R·S·Mirror**（镜像最先、
    /// 平移最后）。`rotate` = `[ax, ay, az, angle]` 四元列表。非几何元素
    /// （camera/灯光/background/gear/cam）带这些 prop 报错——不许静默丢。
    fn local_matrix(&mut self, el: &El) -> Result<Option<[f64; 16]>, String> {
        let has = |k: &str| matches!(prop(el, k), Some(v) if !v.is_null());
        if !(has("t") || has("rotate") || has("scale") || has("mirror")) {
            return Ok(None);
        }
        match el.tag.as_str() {
            "camera" | "ambient_light" | "directional_light" | "point_light" | "background"
            | "gear" | "cam" => {
                return Err(format!(
                    "JSX: transform props (t/rotate/scale/mirror) not supported on <{}>",
                    el.tag
                ))
            }
            _ => {}
        }
        let mut m = cga_core::mat4_identity();
        if let Some(ax) = self.p_vec3_lazy(el, "mirror")? {
            m = mirror_matrix(ax)?;
        }
        if has("scale") {
            let v = match prop(el, "scale") {
                Some(Value::Number(_)) => {
                    let x = p_num(el, "scale")?.unwrap_or(1.0);
                    [x, x, x]
                }
                _ => self.p_vec3_lazy(el, "scale")?.unwrap_or([1.0; 3]),
            };
            m = mat4_mul(scale_matrix(v), m);
        }
        if has("rotate") {
            let l = p_num_list(el, "rotate")?
                .ok_or_else(|| "JSX: rotate must be [ax, ay, az, angle]".to_string())?;
            if l.len() != 4 {
                return Err(format!(
                    "JSX: rotate must be [ax, ay, az, angle], got {l:?}"
                ));
            }
            m = mat4_mul(Multivector::rotor([l[0], l[1], l[2]], l[3]).to_matrix(), m);
        }
        if let Some(t) = self.p_vec3_lazy(el, "t")? {
            m = mat4_mul(translate4(t), m);
        }
        Ok(Some(m))
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
                csg_children_check(&el.tag, kids.len())?;
                let op = match el.tag.as_str() {
                    "union" => cga_core::CsgOp::Union,
                    "difference" => cga_core::CsgOp::Difference,
                    _ => cga_core::CsgOp::Intersection,
                };
                let material = self.build_material(mat)?;
                let geo = cga_core::Geometry::CsgGeometry(cga_core::CsgGeometry::new(op, kids));
                self.emit(scene, geo, cga_core::mat4_identity(), material, el.instance_id);
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
            "link" => self.link_el(el, stack, mat, scene, cam),
            "pair" => self.pair_el(el),
            "anchor" => self.anchor_el(el),
            "joint" => Err(
                "JSX: <joint> 已被 <link>/<pair>/<anchor> 取代（运动学图模型，见 docs/kinematics-graph.md）"
                    .to_string(),
            ),
            "gear" => self.gear_el(el),
            "cam" => self.cam_el(el),
            "closure" => self.closure_el(el),
            "drill" => {
                let (geo, w) = self.drill_cutter(el, ctx)?;
                let material = self.build_material(mat)?;
                self.emit(scene, geo, w, material, el.instance_id);
                Ok(())
            }
            "instances" => {
                let name = p_str(el, "of")?.ok_or_else(|| "JSX: instances needs of".to_string())?;
                for inst in self.instances_of(&name)? {
                    let material = self.build_material(mat)?;
                    self.emit(scene, inst.geo.clone(), inst.world, material, el.instance_id);
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

    /// 收集 CSG 子树的几何。**只从 CSG 分支进入**——因此这里遇到的每个元素都严格
    /// 位于某个 `<difference>`/`<union>`/`<intersection>` 内部，它产出的对象会被
    /// 合并成一个，接不住材质 / 样式 / 分组 / density 这些"每个对象一份"的通道。
    fn collect_geom(
        &mut self,
        el: &El,
        ctx: [f64; 16],
        kids: &mut Vec<cga_core::Geometry>,
    ) -> Result<(), String> {
        if let Some(msg) = csg_dropped_channel(&el.tag, &el.props) {
            return Err(msg);
        }
        // 公共变换 prop 在 CSG 子树里同样生效（与 walk 路径一致）。
        let ctx = match self.local_matrix(el)? {
            Some(m) => mat4_mul(ctx, m),
            None => ctx,
        };
        check_invertible(&el.tag, &ctx)?;
        match el.tag.as_str() {
            // 透明容器（与渲染路径同构）：material/fragment 自身不产生几何。
            "material" | "fragment" => {
                for c in &el.children {
                    self.collect_geom(c, ctx, kids)?;
                }
                Ok(())
            }
            // <group>：透明容器。变换 prop 已在上面生效，子元素按同一 ctx 展开
            // （多子变换容器的展开语义由它承接）。
            "group" => {
                for c in &el.children {
                    self.collect_geom(c, ctx, kids)?;
                }
                Ok(())
            }
            // <tag name>：注册名并透明下传（叶子的 register_tags 消费 pending_tags）。
            "tag" => {
                let name = p_str(el, "name")?.ok_or_else(|| "JSX: tag needs a name".to_string())?;
                self.pending_tags.push((name, ctx));
                for c in &el.children {
                    self.collect_geom(c, ctx, kids)?;
                }
                self.pending_tags.pop();
                Ok(())
            }
            // <when>：条件容器（与渲染路径同规则），不命中就什么都不展开。
            "when" => {
                let of = p_str(el, "of")?.ok_or_else(|| "JSX: when needs of".to_string())?;
                let count =
                    p_num(el, "count")?.ok_or_else(|| "JSX: when needs count".to_string())?;
                let n = self.tags.get(&of).map(|v| v.len()).unwrap_or(0);
                if (n as f64 - count).abs() < 1e-9 {
                    for c in &el.children {
                        self.collect_geom(c, ctx, kids)?;
                    }
                }
                Ok(())
            }
            "union" | "difference" | "intersection" => {
                let mut inner = Vec::new();
                for c in &el.children {
                    self.collect_geom(c, ctx, &mut inner)?;
                }
                csg_children_check(&el.tag, inner.len())?;
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

    fn build_geo(&mut self, el: &El) -> Result<cga_core::Geometry, String> {
        // 修饰符元素已删除：三条路径（渲染 / CSG 收集 / 查询）都落到 build_geo，
        // 这里给唯一的、可操作的报错。
        if let Some(msg) = deleted_modifier_tag(&el.tag) {
            return Err(msg);
        }
        // 场景级元素不产生几何：放错位置时报同一个可读错误。
        if let Some(msg) = non_geometry_tag(&el.tag) {
            return Err(msg);
        }
        let mut args: HashMap<String, ArgValue> = HashMap::new();
        for (k, v) in &el.props {
            if MATERIAL_KEYS.contains(&k.as_str())
                || k == "class"
                || k == "className"
                || k == "id"
                || k == "group"
                || k == "t"
                || k == "rotate"
                || k == "scale"
                || k == "mirror"
                || k == "tag"
                || k == "density"
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
        self.emit(scene, geo, ctx, material, el.instance_id);
        Ok(())
    }

    /// <link>：体图节点。frame 来自图求解（`link_worlds`），自身的变换 prop 在
    /// 求解位姿之上复合；父 ctx 被忽略（D6：树内位置只决定几何归属）。
    fn link_el<'a>(
        &mut self,
        el: &'a El,
        stack: &mut Vec<Frame<'a>>,
        mat: &HashMap<String, ArgValue>,
        scene: &mut Scene,
        cam: &mut Option<PerspectiveCamera>,
    ) -> Result<(), String> {
        if self.cur_link.is_some() {
            return Err("JSX: <link> 不能嵌套".to_string());
        }
        let name = p_str(el, "name")?.ok_or_else(|| "JSX: link needs a name".to_string())?;
        let world = *self
            .link_worlds
            .get(&name)
            .ok_or_else(|| format!("JSX: link {name} 没有求解位姿"))?;
        let idx = self.kin.links.len();
        self.kin.links.push(LinkDef {
            name: name.clone(),
            meshes: Vec::new(),
            world,
        });
        // 自身的变换 prop 在求解位姿之上复合。
        let local = self.local_matrix(el)?;
        let ctx_link = match local {
            Some(m) => mat4_mul(world, m),
            None => world,
        };
        // link 的 tag prop：注册框架 = 求解位姿（含自身变换）而非父 ctx。
        let tag = p_str(el, "tag")?;
        let has_tag = tag.is_some();
        if let Some(t) = tag {
            self.pending_tags.push((t, ctx_link));
        }
        self.cur_link = Some(idx);
        let mut r = Ok(());
        for (i, c) in el.children.iter().enumerate() {
            if let Err(e) = self.walk(Frame::new(c, i), stack, ctx_link, mat, scene, cam) {
                r = Err(e);
                break;
            }
        }
        self.cur_link = None;
        if has_tag {
            self.pending_tags.pop();
        }
        r
    }

    /// <pair>：无向运动副。pass 2 只是按声明序取出求解记录（校验在图求解里）。
    fn pair_el(&mut self, el: &El) -> Result<(), String> {
        if !el.children.is_empty() {
            return Err("JSX: <pair> 不能有子内容（约束是边，不是容器）".to_string());
        }
        let Some(p) = self.pair_records.get(self.pair_out).cloned() else {
            return Err("JSX: pair 记录与声明数量不符（内部错误）".to_string());
        };
        self.pair_out += 1;
        self.kin.pairs.push(p);
        Ok(())
    }

    fn anchor_el(&mut self, el: &El) -> Result<(), String> {
        let l = p_str(el, "link")?.ok_or_else(|| "JSX: anchor needs link".to_string())?;
        self.kin.anchor = Some(l);
        Ok(())
    }

    /// <gear a b ratio offset>：q 图上的方程（无方向；求解方向已在图求解里推导）。
    fn gear_el(&mut self, el: &El) -> Result<(), String> {
        self.kin.gears.push(GearRel {
            a: p_str(el, "a")?.ok_or_else(|| "JSX: gear needs a".to_string())?,
            b: p_str(el, "b")?.ok_or_else(|| "JSX: gear needs b".to_string())?,
            ratio: p_num(el, "ratio")?.ok_or_else(|| "JSX: gear needs ratio".to_string())?,
            offset: p_num(el, "offset")?.unwrap_or(0.0),
        });
        Ok(())
    }

    /// <cam a b aProfile bProfile>：接触约束（求解在图求解里完成）。
    fn cam_el(&mut self, el: &El) -> Result<(), String> {
        let _ = el;
        let Some(c) = self.cam_records.get(self.cam_out).cloned() else {
            return Err("JSX: cam 记录与声明数量不符（内部错误）".to_string());
        };
        self.cam_out += 1;
        self.kin.cams.push(c);
        Ok(())
    }

    /// <closure a b at bAt axis>：环路闭包约束（G5；求解在图求解里完成）。
    fn closure_el(&mut self, el: &El) -> Result<(), String> {
        let _ = el;
        let Some(c) = self.closure_records.get(self.closure_out).cloned() else {
            return Err("JSX: closure 记录与声明数量不符（内部错误）".to_string());
        };
        self.closure_out += 1;
        self.kin.closures.push(c);
        Ok(())
    }
}

pub(crate) fn mat4_inv(m: [f64; 16]) -> [f64; 16] {
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

pub(crate) fn parse_hex(s: &str) -> Result<Color, String> {
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
        // 公共变换 prop 对查询目标同样生效（与渲染路径一致：`<sphere t=…>`
        // 作目标时不再丢变换）。
        let m = match self.local_matrix(el)? {
            Some(l) => mat4_mul(m, l),
            None => m,
        };
        check_invertible(&el.tag, &m)?;
        match el.tag.as_str() {
            // 透明容器：目标几何从唯一子元素取（多子变换容器由 <group> 承接）。
            "group" | "material" | "fragment" | "tag" => {
                if el.children.len() != 1 {
                    return Err(format!(
                        "JSX: {} query target needs exactly one child",
                        el.tag
                    ));
                }
                self.el_geometry(&el.children[0], m)
            }
            // 条件容器：与渲染路径同规则（count 命中才展开）。
            "when" => {
                let of = p_str(el, "of")?.ok_or_else(|| "JSX: when needs of".to_string())?;
                let count =
                    p_num(el, "count")?.ok_or_else(|| "JSX: when needs count".to_string())?;
                let n = self.tags.get(&of).map(|v| v.len()).unwrap_or(0);
                if (n as f64 - count).abs() > 1e-9 {
                    return Err(format!(
                        "JSX: <when of=\"{of}\" count={count}> does not hold ({n} registered) — no query target geometry"
                    ));
                }
                if el.children.len() != 1 {
                    return Err(format!(
                        "JSX: {} query target needs exactly one child",
                        el.tag
                    ));
                }
                self.el_geometry(&el.children[0], m)
            }
            // 实例容器：查询目标必须是单一几何。
            "instances" => {
                let name = p_str(el, "of")?.ok_or_else(|| "JSX: instances needs of".to_string())?;
                let insts = match self.tags.get(&name) {
                    Some(l) if !l.is_empty() => l,
                    _ => return Err(format!("JSX: unknown reference \"{name}\"")),
                };
                if insts.len() != 1 {
                    return Err(format!(
                        "JSX: <instances of=\"{name}\"> has {} instances — a query target needs exactly one; reference the tag by name instead",
                        insts.len()
                    ));
                }
                let (geo, world) = (insts[0].geo.clone(), insts[0].world);
                Ok((geo, mat4_mul(m, world)))
            }
            // 钻孔 cutter：与渲染路径同源。
            "drill" => {
                let (geo, w) = self.drill_cutter(el, m)?;
                Ok((geo, w))
            }
            "union" | "difference" | "intersection" => {
                let mut kids = Vec::new();
                for c in &el.children {
                    self.collect_geom(c, m, &mut kids)?;
                }
                csg_children_check(&el.tag, kids.len())?;
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
                let a = self.resolve_vec3(q_arg(o, "a", q)?, what)?;
                let b = self.resolve_vec3(q_arg(o, "b", q)?, what)?;
                let s = if q == "vadd" { 1.0 } else { -1.0 };
                Ok([a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]])
            }
            "vscale" => {
                let a = self.resolve_vec3(q_arg(o, "a", q)?, what)?;
                let s = o.get("s").and_then(Value::as_f64).unwrap_or(1.0);
                Ok([a[0] * s, a[1] * s, a[2] * s])
            }
            "face" | "fnrm" => {
                let key = o.get("key").and_then(Value::as_str).unwrap_or("+z");
                let (p, n) = self.face_of(q_arg(o, "of", q)?, key, q)?;
                Ok(if q == "face" { p } else { n })
            }
            "xdir" | "ydir" | "zdir" => {
                let (m, _) = self.query_target(q_arg(o, "of", q)?, q)?;
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
                let (_, b) = self.query_target(q_arg(o, "of", q)?, q)?;
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
                let a = self.query_solids(q_arg(o, "a", q)?, what)?;
                let b = self.query_solids(q_arg(o, "b", q)?, what)?;
                let mut best: Option<f64> = None;
                for (ga, wa) in &a {
                    for (gb, wb) in &b {
                        if let Some(d) = cga_collision::separation(ga, *wa, gb, *wb) {
                            best = Some(best.map_or(d, |x: f64| x.min(d)));
                        }
                    }
                }
                best.ok_or_else(|| {
                    format!("JSX: {what}: clearance 是 Unknown（不支持的几何对，不许假装精确）")
                })
            }
            "collides" => {
                let a = self.query_solids(q_arg(o, "a", q)?, what)?;
                let b = self.query_solids(q_arg(o, "b", q)?, what)?;
                let mut any_unknown = false;
                for (ga, wa) in &a {
                    for (gb, wb) in &b {
                        match cga_collision::overlap(ga, *wa, gb, *wb) {
                            cga_collision::Hit::Yes => return Ok(1.0),
                            cga_collision::Hit::No => {}
                            cga_collision::Hit::Unknown => any_unknown = true,
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
                let solids = self.query_solids(q_arg(o, "of", q)?, what)?;
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
                    match cga_collision::contains_point(g, *w, p) {
                        cga_collision::Hit::Yes => return Ok(1.0),
                        cga_collision::Hit::No => {}
                        cga_collision::Hit::Unknown => any_unknown = true,
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
                let a = self.eval_num_value(q_arg(o, "a", q)?, what)?;
                let b = self.eval_num_value(q_arg(o, "b", q)?, what)?;
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
