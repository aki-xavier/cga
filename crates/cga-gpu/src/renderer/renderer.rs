use super::*;

/// 遮挡物的**透过率**（着色端乘在可见性上：1 = 不遮挡，0 = 全遮）。
/// 正常模式取 `1 - opacity`；忽略透明度模式一律 0（当作不透明遮挡物）。
fn eff_occlusion(mode: RenderMode, obj: &Object) -> f64 {
    match mode {
        RenderMode::Normal => 1.0 - obj.material.opacity,
        RenderMode::IgnoreOpacity => 0.0,
    }
}

/// 阴影：遮挡物包围球 vs 阴影射线（p → 光源）的保守筛选，返回需要测试的光线下标。
/// `None` 表示不能界定（无界几何如平面）→ 调用方回退全量。
fn shadow_ray_indices(
    params: &GeometryParams,
    p: &Array,
    ld: &Array,
    far: &Array,
    n_rays: i32,
) -> Option<Vec<i32>> {
    let b = geom_bounds(params)?;
    let c = [
        0.5 * (b[0][0] + b[1][0]),
        0.5 * (b[0][1] + b[1][1]),
        0.5 * (b[0][2] + b[1][2]),
    ];
    let (dx, dy, dz) = (b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]);
    let r = 0.5 * (dx * dx + dy * dy + dz * dz).sqrt();
    if !(r.is_finite() && r >= 0.0) {
        return None;
    }
    // far：点光源是 (n,)，平行光是 0-d（= 无限远）
    let far_lim = if far.ndim() == 1 {
        ck(far.reshape(&[n_rays]))
    } else {
        ck(ops::broadcast_to(fs(1e30), &[n_rays]))
    };
    let to_c = ck(ck(p.negative()).add(&arr3v(c))); // c - p  (n,3)
    let proj = ck(ck(to_c.multiply(ld)).sum_axes(&[-1], false)); // (n,)
    let perp2 =
        ck(ck(ck(to_c.multiply(&to_c)).sum_axes(&[-1], false)).subtract(&ck(proj.multiply(&proj))));
    let mask = ck(s_le(&perp2, r * r).logical_and(&s_ge(&proj, -r)));
    let lim = ck(far_lim.add(&fs(r)));
    let mask = ck(mask.logical_and(&ck(lim.gt(&proj))));
    mask.eval().unwrap();
    let cnt = ck(mask.sum(None)).item_cast::<i32>();
    if cnt * 2 >= n_rays {
        // 子集收益不足 2×：走全量，省掉索引回读与 gather/scatter 开销
        return None;
    }
    if cnt == 0 {
        return Some(Vec::new());
    }
    let mut idx: Vec<i32> = Vec::with_capacity(cnt as usize);
    let data = mask.as_slice::<bool>();
    for (i, &v) in data.iter().enumerate() {
        if v {
            idx.push(i as i32);
        }
    }
    Some(idx)
}

/// 次级光线（反射/折射）：对象包围球 vs 光线束的保守筛选，返回需要求交的光线下标。
/// `None` = 不能界定（无界几何如平面）或子集收益不足（≥50% 命中）。
fn ray_object_subset(params: &GeometryParams, o: &Array, d: &Array) -> Option<Vec<i32>> {
    let b = geom_bounds(params)?;
    let c = [
        0.5 * (b[0][0] + b[1][0]),
        0.5 * (b[0][1] + b[1][1]),
        0.5 * (b[0][2] + b[1][2]),
    ];
    let (dx, dy, dz) = (b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]);
    let r = 0.5 * (dx * dx + dy * dy + dz * dz).sqrt();
    if !(r.is_finite() && r >= 0.0) {
        return None;
    }
    let n = o.shape()[0];
    let to_c = ck(ck(o.negative()).add(&arr3v(c)));
    let proj = ck(ck(to_c.multiply(d)).sum_axes(&[-1], false));
    let perp2 =
        ck(ck(ck(to_c.multiply(&to_c)).sum_axes(&[-1], false)).subtract(&ck(proj.multiply(&proj))));
    let mask = ck(s_le(&perp2, r * r).logical_and(&s_ge(&proj, -r)));
    mask.eval().unwrap();
    let cnt = ck(mask.sum(None)).item_cast::<i32>();
    if cnt * 2 >= n {
        return None; // 收益不足 2×：走全量
    }
    if cnt == 0 {
        return Some(Vec::new());
    }
    let mut idx: Vec<i32> = Vec::with_capacity(cnt as usize);
    let data = mask.as_slice::<bool>();
    for (i, &v) in data.iter().enumerate() {
        if v {
            idx.push(i as i32);
        }
    }
    Some(idx)
}

/// 主光线：对象屏幕包围盒内的光线下标（保守剔除）。
///
/// 光线包按 `(子采样 j,i) × 基础像素 (y,x)` 展开：`idx = (j*k+i)·(w·h) + y·w + x`。
/// 包围盒由相机空间 AABB 的 8 个角投影得到，外扩 1 个基础像素（子采样落点 +
/// 浮点余量）。返回 `None` 表示不能用子集（无界几何如平面、或 AABB 跨近平面）
/// ——调用方回退全量光线。返回空表表示对象完全在画面外（连同相机后方）。
fn primary_ray_indices(
    params: &GeometryParams,
    cam: &PerspectiveCamera,
    w: i32,
    h: i32,
    k: i32,
) -> Option<Vec<i32>> {
    let b = geom_bounds(params)?;
    let [lo, hi] = b;
    if hi[2] <= 1e-6 {
        return Some(Vec::new()); // 全在相机后方
    }
    if lo[2] <= 1e-6 {
        return None; // 跨近平面：投影会发散，保守回退
    }
    let fy = f64::from(h) / (2.0 * (cam.fov.to_radians() / 2.0).tan());
    let fx = fy * cam.aspect;
    let cx = f64::from(w - 1) / 2.0;
    let cy = f64::from(h - 1) / 2.0;
    let (mut x0, mut x1) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY);
    for c in 0..8 {
        let p = [
            if c & 1 == 0 { lo[0] } else { hi[0] },
            if c & 2 == 0 { lo[1] } else { hi[1] },
            if c & 4 == 0 { lo[2] } else { hi[2] },
        ];
        let sx = fx * p[0] / p[2] + cx;
        let sy = fy * p[1] / p[2] + cy;
        x0 = x0.min(sx);
        x1 = x1.max(sx);
        y0 = y0.min(sy);
        y1 = y1.max(sy);
    }
    let bx0 = (x0.floor() as i32 - 1).max(0);
    let by0 = (y0.floor() as i32 - 1).max(0);
    let bx1 = (x1.ceil() as i32 + 1).min(w - 1);
    let by1 = (y1.ceil() as i32 + 1).min(h - 1);
    if bx1 < bx0 || by1 < by0 || !x0.is_finite() {
        return Some(Vec::new());
    }
    let base = w * h;
    let area = ((bx1 - bx0 + 1) as i64) * ((by1 - by0 + 1) as i64) * (k as i64) * (k as i64);
    let mut idx: Vec<i32> = Vec::with_capacity(area as usize);
    for y in by0..=by1 {
        for x in bx0..=bx1 {
            for j in 0..k {
                for i in 0..k {
                    idx.push((j * k + i) * base + y * w + x);
                }
            }
        }
    }
    Some(idx)
}

/// 光线追踪的渲染模式。
///
/// - `Normal`：材质透明度生效——`opacity < 1` 的表面发射反射/折射次级光线（Whitted），
///   半透明遮挡物按 `1 - opacity` 削弱阴影。
/// - `IgnoreOpacity`：把一切当不透明——不发射任何次级光线，透明度不削弱阴影，
///   材质的不透明部分照常着色。用于"实体预览"与大幅提速（玻璃场景可快一个数量级）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RenderMode {
    #[default]
    Normal,
    IgnoreOpacity,
}

#[derive(Debug)]
pub struct Renderer {
    pub width: i32,
    pub height: i32,
    pub aa: i32,
    pub max_depth: i32,
    pub cam: Option<PerspectiveCamera>,
    /// 渲染模式（默认 `Normal`）。
    pub mode: RenderMode,
}
impl Renderer {
    pub fn new(width: i32, height: i32, aa: i32, max_depth: i32) -> Renderer {
        if aa < 1 {
            panic!("aa must be >= 1, got {}", aa);
        }

        if width < 1 || height < 1 {
            panic!("renderer needs a positive extent, got {}x{}", width, height);
        }
        Renderer {
            width,
            height,
            aa,
            max_depth,
            cam: None,
            mode: RenderMode::Normal,
        }
    }

    /// 设定渲染模式（链式）。
    pub fn with_mode(mut self, mode: RenderMode) -> Renderer {
        self.mode = mode;
        self
    }

    pub fn render_frame(
        scene: Scene,
        camera: PerspectiveCamera,
        width: i32,
        height: i32,
        aa: i32,
    ) -> Array {
        let mut r = Self::new(width, height, aa, 3);
        r.render(scene, camera)
    }

    fn build_rays(&mut self) -> Array {
        let hh = self.height;
        let ww = self.width;
        let cam = self.cam.unwrap_or_else(|| panic!("no camera"));
        let fy = f64::from(hh) / (2.0 * (cam.fov.to_radians() / 2.0).tan());
        let fx = fy * cam.aspect;
        let cx = f64::from(ww - 1) / 2.0;
        let cy = f64::from(hh - 1) / 2.0;
        let u0 = s_div(&s_sub(&ck(ops::arange::<i32, f32>(0, ww, 1)), cx), fx);
        let v0 = s_div(&s_sub(&ck(ops::arange::<i32, f32>(0, hh, 1)), cy), fy);
        let z = ck(ops::ones::<f32>(&[hh, ww]));
        let mut dirs: Vec<Array> = Vec::new();
        let k = self.aa;
        for j in 0..k {
            for i in 0..k {
                let off_u = (f64::from(i) + 0.5) / f64::from(k) - 0.5;
                let off_v = (f64::from(j) + 0.5) / f64::from(k) - 0.5;
                let du = off_u / fx;
                let dv = off_v / fy;
                let u = ck(ops::broadcast_to(
                    ck(s_add(&u0, du).expand_dims(0)),
                    &[hh, ww],
                ));
                let v = ck(ops::broadcast_to(
                    ck(s_add(&v0, dv).expand_dims(1)),
                    &[hh, ww],
                ));
                dirs.push(ck(ops::stack(&[&u, &v, &z], -1)));
            }
        }
        let rays = ck(ck(ops::concatenate(&dirs, 0)).reshape(&[-1, 3]));
        let n = ck(ck(ck(rays.multiply(&rays)).sum_axes(&[-1], true)).sqrt());
        ck(rays.divide(&n))
    }

    pub fn render(&mut self, scene: Scene, camera: PerspectiveCamera) -> Array {
        self.render_with_truth(scene, camera).0
    }

    pub fn render_with_truth(&mut self, scene: Scene, camera: PerspectiveCamera) -> (Array, Truth) {
        self.cam = Some(camera);
        let rays = self.build_rays();
        let o = ck(ops::zeros_like(&rays));
        let n_rays = o.shape()[0];
        let bg = ck(ops::broadcast_to(
            arr3v(scene.background.rgb()),
            &[n_rays, 3],
        ));
        let mut lit: Vec<Light> = Vec::new();
        let mut ambient: Option<Light> = None;
        for light in &scene.lights {
            if light.kind == LightKind::Ambient {
                ambient = Some(*light);
            } else {
                lit.push(light.to_camera(camera.motor));
            }
        }

        // 场景即全部对象（解析图元 + CSG + 仿射包装）：纯光线追踪。
        let s2 = scene.clone();
        // 相机空间参数 + 对象陈述参数全场景各建一次（阴影遮挡表 / UV 陈述共用）
        let mut params_list: Vec<GeometryParams> = Vec::with_capacity(s2.objects.len());
        let mut stated_list: Vec<GeometryParams> = Vec::with_capacity(s2.objects.len());
        for obj in s2.objects.iter() {
            params_list.push(geom_to_camera(
                &obj.geometry,
                &camera.motor.compose(&obj.motor()),
            ));
            stated_list.push(geom_to_camera(&obj.geometry, &obj.motor()));
        }
        let in_medium = ck(ops::zeros::<bool>(&[n_rays]));
        let sigma = ck(ops::zeros::<f32>(&[n_rays]));
        let (rgb_sr, _t, truth) = self.trace(
            &s2,
            &params_list,
            &stated_list,
            &o,
            &rays,
            &lit,
            ambient,
            &bg,
            &in_medium,
            &sigma,
            0,
        );
        // SSAA：子采样均值降采样
        let s = self.aa * self.aa;
        let mut rgb = if s > 1 {
            ck(ck(rgb_sr.reshape(&[s, n_rays / s, 3])).mean_axes(&[0], false))
        } else {
            rgb_sr
        };
        rgb = ck(rgb.reshape(&[self.height * self.width, 3]));
        rgb = s_clip(&rgb, 0.0, 1.0);
        rgb = ck(ops::select(
            s_le(&rgb, 0.0031308),
            s_mul(&rgb, 12.92),
            s_sub(&s_mul(&s_pow(&rgb, 1.0 / 2.4), 1.055), 0.055),
        ));
        let mut rgba = ck(ops::concatenate(
            &[&rgb, &ck(ops::ones::<f32>(&[n_rays / s, 1]))],
            -1,
        ));
        rgba = s_clip(&s_add(&s_mul(&rgba, 255.0), 0.5), 0.0, 255.0);
        (
            ck(rgba.reshape(&[self.height, self.width, 4])),
            truth.expect("the primary pass states a truth"),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn trace(
        &self,
        scene: &Scene,
        params_list: &[GeometryParams],
        stated_list: &[GeometryParams],
        o: &Array,
        d: &Array,
        lit: &[Light],
        ambient: Option<Light>,
        bg: &Array,
        in_medium: &Array,
        sigma: &Array,
        depth: i32,
    ) -> (Array, Array, Option<Truth>) {
        let (hit, t, n0, local, op, ior, abso, index, vis) = self.nearest(
            scene,
            params_list,
            stated_list,
            o,
            d,
            lit,
            ambient,
            depth == 0,
        );
        let mut cos_i = ck(ck(ck(d.multiply(&n0)).sum_axes(&[-1], true)).negative());
        let n = ck(ops::select(s_lt(&cos_i, 0.0), ck(n0.negative()), &n0));
        cos_i = ck(cos_i.abs());
        let mut result = ck(ops::select(ck(hit.expand_dims(1)), &local, bg));

        let truth = if depth == 0 {
            Some(Truth {
                hit: hit.clone(),
                t: t.clone(),
                normal: n.clone(),
                index,
                vis,
            })
        } else {
            None
        };
        if depth < self.max_depth && self.mode == RenderMode::Normal {
            let need = ck(hit.logical_and(s_lt(&op, 1.0)));
            let need_c = ck(need.contiguous());
            need_c.eval().unwrap();
            let m_need = ck(ck(need_c.as_type::<i32>()).sum(None)).item_cast::<i32>();
            if m_need > 0 {
                // 递归只对“需要折射/反射的像素”做：把光线包裁剪成 need 的子集，
                // 递归返回后散布回全量（其余像素由 result 保留）。此前递归在全量
                // 光线包上跑，未命中的像素白算——玻璃像素占比越小浪费越大。
                let full = m_need >= o.shape()[0];
                let idx: Vec<i32> = if full {
                    Vec::new()
                } else {
                    let mut v: Vec<i32> = Vec::with_capacity(m_need as usize);
                    for (i, &b) in need_c.as_slice::<bool>().iter().enumerate() {
                        if b {
                            v.push(i as i32);
                        }
                    }
                    v
                };
                let ids = if full {
                    None
                } else {
                    Some(Array::from_slice(&idx, &[idx.len() as i32]))
                };
                // 子集化输入（full 时直接用原数组）
                let g = |a: &Array| -> Array {
                    match &ids {
                        Some(id) => ck(a.take_axis(id, 0)),
                        None => a.clone(),
                    }
                };
                let (o_s, d_s) = (g(o), g(d));
                let n_s = g(&n);
                let cos_i_s = g(&cos_i);
                let op_s = g(&op);
                let ior_s = g(&ior);
                let abso_s = g(&abso);
                let t_s = g(&t);
                let local_s = g(&local);
                let in_medium_s = g(in_medium);
                let sigma_s = g(sigma);
                let bg_s = g(bg);

                let eta = ck(ops::select(
                    ck(in_medium_s.expand_dims(1)),
                    ck(ior_s.expand_dims(1)),
                    ck(fs(1.0).divide(ck(ior_s.expand_dims(1)))),
                ));
                let k = ck(fs(1.0).subtract(ck(ck(eta.multiply(&eta))
                    .multiply(ck(fs(1.0).subtract(ck(cos_i_s.multiply(&cos_i_s))))))));
                let cos_t = ck(s_max(&k, 0.0).sqrt());
                let gg = ck(fs(1.0).divide(&eta));
                let rs = ck(ck(cos_i_s.subtract(ck(gg.multiply(&cos_t))))
                    .divide(s_max(&ck(cos_i_s.add(ck(gg.multiply(&cos_t)))), 1e-12)));
                let rp = ck(ck(cos_t.subtract(ck(gg.multiply(&cos_i_s))))
                    .divide(s_max(&ck(cos_t.add(ck(gg.multiply(&cos_i_s)))), 1e-12)));
                let mut fres = s_mul(&ck(ck(rs.multiply(&rs)).add(ck(rp.multiply(&rp)))), 0.5);
                fres = ck(ops::select(s_le(&k, 0.0), ck(ops::ones_like(&fres)), &fres));
                let p = ck(o_s.add(ck(ck(t_s.expand_dims(1)).multiply(&d_s))));
                let d_r = ck(d_s.add(ck(n_s.multiply(s_mul(&cos_i_s, 2.0)))));
                let d_t = ck(ck(d_s.multiply(&eta)).add(ck(
                    n_s.multiply(ck(ck(eta.multiply(&cos_i_s)).subtract(&cos_t)))
                )));
                let entering = ck(in_medium_s.logical_not());
                let sig_next = ck(ops::select(&entering, &abso_s, fs(0.0)));
                let (refl, _, _) = self.trace(
                    scene,
                    params_list,
                    stated_list,
                    &ck(p.add(s_mul(&n_s, 1e-3))),
                    &d_r,
                    lit,
                    ambient,
                    &bg_s,
                    &in_medium_s,
                    &sigma_s,
                    depth + 1,
                );
                let (refr, _, _) = self.trace(
                    scene,
                    params_list,
                    stated_list,
                    &ck(p.subtract(s_mul(&n_s, 1e-3))),
                    &d_t,
                    lit,
                    ambient,
                    &bg_s,
                    &entering,
                    &sig_next,
                    depth + 1,
                );
                let body = ck(ck(ck(op_s.expand_dims(1)).multiply(&local_s))
                    .add(ck(
                        ck(fs(1.0).subtract(ck(op_s.expand_dims(1)))).multiply(&refr)
                    )));
                let glass =
                    ck(ck(fres.multiply(&refl))
                        .add(ck(ck(fs(1.0).subtract(&fres)).multiply(&body))));
                result = match &ids {
                    None => ck(ops::select(ck(need.expand_dims(1)), &glass, &result)),
                    Some(id) => ck(ops::indexing::scatter_single(
                        &result,
                        id,
                        &ck(glass.reshape(&[idx.len() as i32, 1, 3])),
                        0,
                    )),
                };
            }
        }
        let att = ck(ops::select(
            ck(ck(in_medium.logical_and(&hit)).expand_dims(1)),
            ck(ck(ck(ck(sigma.negative()).multiply(&t)).expand_dims(1)).exp()),
            fs(1.0),
        ));
        (ck(result.multiply(&att)), t, truth)
    }

    fn nearest(
        &self,
        scene: &Scene,
        params_list: &[GeometryParams],
        stated_list: &[GeometryParams],
        o: &Array,
        d: &Array,
        lit: &[Light],
        ambient: Option<Light>,
        primary: bool,
    ) -> (
        Array,
        Array,
        Array,
        Array,
        Array,
        Array,
        Array,
        Array,
        Vec<Array>,
    ) {
        let oshape = o.shape();
        let dshape = d.shape();
        if oshape.len() != 2
            || oshape[1] != 3
            || dshape.len() != 2
            || dshape[1] != 3
            || dshape[0] != oshape[0]
            || oshape[0] < 1
        {
            panic!(
                "nearest needs matching ray bundles of shape (N, 3) with N >= 1, got o={:?} d={:?}",
                oshape, dshape
            );
        }
        let n_rays = o.shape()[0];
        let mut best_t = ck(ops::full::<f32>(&[n_rays], &fs(f64::INFINITY)));
        let mut best_n = ck(ops::zeros::<f32>(&[n_rays, 3]));
        let mut best_uv = ck(ops::zeros::<f32>(&[n_rays, 2]));
        let mut best_idx = ck(ops::zeros::<i32>(&[n_rays]));
        let objs = &scene.objects;
        // 临时排查开关：CGA_NO_CULL=1 关闭所有剔除子集（对照渲染）
        let cull_on = std::env::var("CGA_NO_CULL").is_err();
        let cam = self.cam.unwrap_or_else(|| panic!("no camera"));
        let frame = view_frame(&cam);
        for (i, _obj) in objs.iter().enumerate() {
            let params = &params_list[i];
            // 主光线逐对象屏幕区间子集（保守剔除）：只对落在对象屏幕包围盒内的
            // 光线求交。无界几何/跨近平面 → None 回退全量；空表 → 画面外，跳过。
            let subset: Option<Vec<i32>> = if !cull_on {
                None
            } else if primary {
                primary_ray_indices(params, &cam, self.width, self.height, self.aa)
            } else {
                // 次级光线（反射/折射）：包络球筛选（主光线的屏幕区间不适用）
                ray_object_subset(params, o, d)
            };
            let ids: Option<Array> = match &subset {
                Some(v) if v.is_empty() => continue,
                Some(v) => Some(Array::from_slice(v, &[v.len() as i32])),
                None => None,
            };
            let (o_u, d_u) = match &ids {
                Some(ids_a) => (ck(o.take_axis(ids_a, 0)), ck(d.take_axis(ids_a, 0))),
                None => (o.clone(), d.clone()),
            };
            let (t, n_i, mask) = geom_intersect(params, &o_u, &d_u);
            let hit_point = ck(o_u.add(ck(ck(t.expand_dims(1)).multiply(&d_u))));
            let uv_i = geom_uv(
                &stated_list[i],
                &outward(&hit_point, &frame.0, &frame.1),
                &outward(&n_i, &frame.0, &[0.0, 0.0, 0.0]),
            );
            match &ids {
                None => {
                    let nearer = ck(mask.logical_and(ck(t.lt(&best_t))));
                    best_t = ck(ops::select(&nearer, &t, &best_t));
                    best_n = ck(ops::select(ck(nearer.expand_dims(1)), &n_i, &best_n));
                    best_uv = ck(ops::select(ck(nearer.expand_dims(1)), &uv_i, &best_uv));
                    best_idx = ck(ops::select(
                        &nearer,
                        ck(ops::full::<i32>(&[n_rays], &Array::from_int(i as i32))),
                        &best_idx,
                    ));
                }
                Some(ids_a) => {
                    // 子集：先取回该子集当前的胜者，再用 scatter 写回（下标唯一）
                    let len = d_u.shape()[0];
                    let bt_s = ck(best_t.take_axis(ids_a, 0));
                    let bn_s = ck(best_n.take_axis(ids_a, 0));
                    let bu_s = ck(best_uv.take_axis(ids_a, 0));
                    let bi_s = ck(best_idx.take_axis(ids_a, 0));
                    let nearer = ck(mask.logical_and(ck(t.lt(&bt_s))));
                    let ne1 = ck(nearer.expand_dims(1));
                    best_t = ck(ops::indexing::scatter_single(
                        &best_t,
                        ids_a,
                        &ck(ck(ops::select(&nearer, &t, &bt_s)).reshape(&[len, 1])),
                        0,
                    ));
                    best_n = ck(ops::indexing::scatter_single(
                        &best_n,
                        ids_a,
                        &ck(ck(ops::select(&ne1, &n_i, &bn_s)).reshape(&[len, 1, 3])),
                        0,
                    ));
                    best_uv = ck(ops::indexing::scatter_single(
                        &best_uv,
                        ids_a,
                        &ck(ck(ops::select(&ne1, &uv_i, &bu_s)).reshape(&[len, 1, 2])),
                        0,
                    ));
                    best_idx = ck(ops::indexing::scatter_single(
                        &best_idx,
                        ids_a,
                        &ck(ck(ops::select(
                            &nearer,
                            &ck(ops::full::<i32>(&[len], &Array::from_int(i as i32))),
                            &bi_s,
                        ))
                        .reshape(&[len, 1])),
                        0,
                    ));
                }
            }
        }
        let hit = ck(best_t.is_finite());

        let mut op = ck(ops::ones::<f32>(&[n_rays]));
        let mut ior = ck(ops::full::<f32>(&[n_rays], &fs(1.5)));
        let mut abso = ck(ops::zeros::<f32>(&[n_rays]));
        if !objs.is_empty() {
            let mut op_arr: Vec<Array> = Vec::new();
            let mut ior_arr: Vec<Array> = Vec::new();
            let mut abso_arr: Vec<Array> = Vec::new();
            for obj in objs {
                op_arr.push(fs(obj.material.opacity));
                ior_arr.push(fs(obj.material.ior));
                abso_arr.push(fs(obj.material.absorption));
            }
            let ops_ = ck(ops::stack(&op_arr, 0));
            let iors = ck(ops::stack(&ior_arr, 0));
            let absos = ck(ops::stack(&abso_arr, 0));
            op = ck(ops_.take_axis(&best_idx, 0));
            if self.mode == RenderMode::IgnoreOpacity {
                op = ck(ops::ones_like(&op));
            }
            ior = ck(iors.take_axis(&best_idx, 0));
            abso = ck(absos.take_axis(&best_idx, 0));
        }
        let cos_i = ck(ck(ck(d.multiply(&best_n)).sum_axes(&[-1], true)).negative());
        best_n = ck(ops::select(
            s_lt(&cos_i, 0.0),
            ck(best_n.negative()),
            &best_n,
        ));
        let p = ck(o.add(ck(ck(best_t.expand_dims(1)).multiply(d))));

        let p_s = ck(p.add(s_mul(&best_n, 1e-3)));
        let mut vis: Vec<Array> = Vec::new();
        for light in lit {
            let (ld, _) = light.direction_at(&p);
            let far = light.far(&p);
            let mut v = ck(ops::ones::<f32>(&[n_rays]));
            for (j, obj) in objs.iter().enumerate() {
                // 逐对象阴影子集：只有“打到光源的射线”靠近该对象时才需要测试
                let sh_sub = if cull_on {
                    shadow_ray_indices(&params_list[j], &p_s, &ld, &far, n_rays)
                } else {
                    None
                };
                match sh_sub {
                    Some(v_i) if v_i.is_empty() => continue,
                    Some(v_i) => {
                        let len = v_i.len() as i32;
                        let ids = Array::from_slice(&v_i, &[len]);
                        let ps_s = ck(p_s.take_axis(&ids, 0));
                        let ld_s = ck(ld.take_axis(&ids, 0));
                        let far_s = if far.ndim() == 1 {
                            ck(far.take_axis(&ids, 0))
                        } else {
                            far.clone()
                        };
                        let (st, m) = geom_shadow(&params_list[j], &ps_s, &ld_s);
                        let occ = if far_s.ndim() == 1 {
                            ck(m.logical_and(ck(st.lt(&far_s))))
                        } else {
                            m
                        };
                        let f_s = ck(ops::select(
                            &occ,
                            fs(eff_occlusion(self.mode, obj)),
                            fs(1.0),
                        ));
                        // 合并回全量：v *= (1 + 散布(f_s - 1))
                        let delta = ck(ops::indexing::scatter_single(
                            &ck(ops::zeros::<f32>(&[n_rays])),
                            &ids,
                            &ck(ck(f_s.subtract(fs(1.0))).reshape(&[len, 1])),
                            0,
                        ));
                        v = ck(v.multiply(&ck(delta.add(fs(1.0)))));
                    }
                    None => {
                        let (st, m) = geom_shadow(&params_list[j], &p_s, &ld);
                        let occ = if far.ndim() == 1 {
                            ck(m.logical_and(ck(st.lt(&far))))
                        } else {
                            m
                        };
                        v = ck(v.multiply(ck(ops::select(
                            &occ,
                            fs(eff_occlusion(self.mode, obj)),
                            fs(1.0),
                        ))));
                    }
                }
            }
            vis.push(v);
        }

        let mut acc = ck(ops::zeros::<f32>(&[n_rays, 3]));
        if !objs.is_empty() {
            let mut em_arr: Vec<Array> = Vec::new();
            let mut diff_arr: Vec<Array> = Vec::new();
            let mut spec_arr: Vec<Array> = Vec::new();
            let mut expo_arr: Vec<Array> = Vec::new();
            for obj in objs {
                let (em, diff, spec, expo) = obj.material.shade_params();
                em_arr.push(arr3v(em));
                diff_arr.push(arr3v(diff));
                spec_arr.push(arr3v(spec));
                expo_arr.push(fs(expo));
            }
            let emissive = ck(ck(ops::stack(&em_arr, 0)).take_axis(&best_idx, 0));
            let diff = ck(ck(ops::stack(&diff_arr, 0)).take_axis(&best_idx, 0));
            let spec = ck(ck(ops::stack(&spec_arr, 0)).take_axis(&best_idx, 0));
            let expo = ck(ck(ck(ops::stack(&expo_arr, 0)).take_axis(&best_idx, 0)).expand_dims(1));
            acc = shade_batched(
                &emissive, &diff, &spec, &expo, &p, &best_n, d, lit, ambient, &vis,
            );
            for (i, obj) in objs.iter().enumerate() {
                if let Some(tex) = &obj.material.map {
                    let sampled = ck(tex
                        .sample(&best_uv, WrapMode::Repeat, WrapMode::Repeat)
                        .take_axis(Array::from_slice(&[0_i32, 1, 2], &[3]), 1));
                    acc = ck(ops::select(
                        ck(ck(best_idx.eq(Array::from_int(i as i32))).expand_dims(1)),
                        ck(acc.multiply(&sampled)),
                        &acc,
                    ));
                }
            }
        }
        (hit, best_t, best_n, acc, op, ior, abso, best_idx, vis)
    }
}
