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
    let [lo, hi] = geom_bounds(params)?;
    aabb_ray_indices(lo, hi, cam, w, h, k)
}

/// 相机空间 AABB → 覆盖的主光线下标。
/// `None` = 跨近平面无法保守界定（调用方回退全量）；`Some(vec![])` = 画面外。
fn aabb_ray_indices(
    lo: [f64; 3],
    hi: [f64; 3],
    cam: &PerspectiveCamera,
    w: i32,
    h: i32,
    k: i32,
) -> Option<Vec<i32>> {
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

/// 相机空间包围球 → 覆盖的主光线下标（增量渲染的阴影足迹用）。
fn sphere_ray_indices(
    c: [f64; 3],
    r: f64,
    cam: &PerspectiveCamera,
    w: i32,
    h: i32,
    k: i32,
) -> Option<Vec<i32>> {
    aabb_ray_indices(
        [c[0] - r, c[1] - r, c[2] - r],
        [c[0] + r, c[1] + r, c[2] + r],
        cam,
        w,
        h,
        k,
    )
}

// ---- 相机空间小向量工具（增量渲染的脏集计算，全在 CPU f64 上做） ----

fn v3_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn v3_add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn v3_scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn v3_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn v3_len(a: [f64; 3]) -> f64 {
    v3_dot(a, a).sqrt()
}

/// AABB → 包围球。
fn aabb_sphere(b: &[[f64; 3]; 2]) -> ([f64; 3], f64) {
    let c = [
        0.5 * (b[0][0] + b[1][0]),
        0.5 * (b[0][1] + b[1][1]),
        0.5 * (b[0][2] + b[1][2]),
    ];
    let d = [b[1][0] - b[0][0], b[1][1] - b[0][1], b[1][2] - b[0][2]];
    (c, 0.5 * v3_len(d))
}

/// 遮挡物包围球 `occ` 在灯光下是否可能把阴影投到接收者包围球 `rcv` 上（保守：
/// 只许误报不许漏报）。方向光的 `direction` 指向光源，阴影沿反方向延伸。
fn shadow_reaches(light: &Light, occ: ([f64; 3], f64), rcv: ([f64; 3], f64)) -> bool {
    const SLACK: f64 = 1.02;
    let (oc, or_) = occ;
    let (rc, rr) = rcv;
    match light.kind {
        LightKind::Ambient => false,
        LightKind::Directional => {
            let d = light.direction;
            let ab = v3_sub(rc, oc);
            let u = v3_dot(ab, d); // 接收者中心朝光源方向超出遮挡物多少
            if u > rr * SLACK {
                return false; // 整体在遮挡物的迎光面之外
            }
            let perp2 = (v3_dot(ab, ab) - u * u).max(0.0);
            let rsum = (or_ + rr) * SLACK;
            perp2 <= rsum * rsum
        }
        LightKind::Point => {
            let pl = light.position;
            let vo = v3_sub(oc, pl);
            let vb = v3_sub(rc, pl);
            let do_ = v3_len(vo);
            let db = v3_len(vb);
            if do_ < 1e-9 || db < 1e-9 {
                return true; // 光源贴着包围球：无法界定，按可能投影处理
            }
            if db + rr * SLACK < do_ - or_ * SLACK {
                return false; // 接收者比遮挡物更靠近光源
            }
            // 夹角 ≤ asin(or/do) + asin(rr/db)
            let sin_o = (or_ * SLACK / do_).min(1.0);
            let sin_b = (rr * SLACK / db).min(1.0);
            let cos_ang = v3_dot(vo, vb) / (do_ * db);
            let cos_lim =
                (1.0 - sin_o * sin_o).sqrt() * (1.0 - sin_b * sin_b).sqrt() - sin_o * sin_b;
            cos_ang >= cos_lim
        }
    }
}

/// 遮挡物包围球在平面（`n·x = d`）上的阴影足迹（保守圆盘，相机空间）。
/// `None` = 无法保守界定（光近平行平面、平面在光源与遮挡物之间等）→ 调用方回退全帧。
/// 环境光不投影，不会走到这里（`lit` 只含非环境光）。
fn shadow_on_plane(
    light: &Light,
    occ: ([f64; 3], f64),
    n: [f64; 3],
    d: f64,
) -> Option<([f64; 3], f64)> {
    const SLACK: f64 = 1.05;
    let (oc, r) = occ;
    match light.kind {
        LightKind::Ambient => None,
        LightKind::Directional => {
            let ld = light.direction; // 指向光源；阴影沿 -ld 延伸
            let dn = v3_dot(ld, n);
            if dn.abs() < 0.05 {
                return None; // 光近平行平面：阴影拉得极长
            }
            // oc - t·ld 落在平面上：n·(oc - t·ld) = d
            let t = (v3_dot(oc, n) - d) / dn;
            let c = v3_sub(oc, v3_scale(ld, t));
            // 斜射把圆拉成椭圆，用 r/|n·ld| 保守覆盖
            Some((c, r / dn.abs() * SLACK + 0.02))
        }
        LightKind::Point => {
            let pl = light.position;
            let v = v3_sub(oc, pl);
            let dist_o = v3_len(v);
            let vn = v3_dot(v, n);
            if dist_o < 1e-9 || vn.abs() < 1e-9 {
                return None;
            }
            // pl + s·v 落在平面上
            let s = (d - v3_dot(pl, n)) / vn;
            if s <= 1.0 {
                return None; // 平面不在遮挡物的背光侧
            }
            let c = v3_add(pl, v3_scale(v, s));
            let dist = (dist_o * s).max(1e-6);
            // 锥体放大：半径随距离线性扩张
            let cone_r = r * SLACK * dist / (dist_o - r).max(1e-6);
            Some((c, cone_r + 0.02))
        }
    }
}

/// 拾取命中（交互闭环，docs/roadmap.md §3.A）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PickHit {
    /// `scene.objects` 下标。
    pub object: usize,
    /// 世界坐标命中点。
    pub point: [f64; 3],
    /// 世界坐标外向法线。
    pub normal: [f64; 3],
    /// 光线参数（相机空间深度）。
    pub t: f64,
}

/// 拾取：从像素 `(x, y)`（像素坐标，左上角原点，与 `build_rays` 的排布一致）
/// 发一条光线，返回最近的命中。解析求交（不走 MLX 渲染管线）。
pub fn pick(
    scene: &Scene,
    camera: &PerspectiveCamera,
    x: f64,
    y: f64,
    width: i32,
    height: i32,
) -> Option<PickHit> {
    let (w, h) = (f64::from(width), f64::from(height));
    let fy = h / (2.0 * (camera.fov.to_radians() / 2.0).tan());
    let fx = fy * camera.aspect;
    let cx = (w - 1.0) / 2.0;
    let cy = (h - 1.0) / 2.0;
    let d = [(x - cx) / fx, (y - cy) / fy, 1.0];
    let dl = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    let d = [d[0] / dl, d[1] / dl, d[2] / dl];
    let o = ck(ops::zeros::<f32>(&[1, 3]));
    let d_a = ck(ops::broadcast_to(arr3v([d[0], d[1], d[2]]), &[1, 3]));
    let mut best: Option<PickHit> = None;
    for (i, obj) in scene.objects.iter().enumerate() {
        let params = geom_to_camera(&obj.geometry, &camera.motor.compose(&obj.motor()));
        let (t, n, mask) = geom_intersect(&params, &o, &d_a);
        t.eval().unwrap();
        n.eval().unwrap();
        mask.eval().unwrap();
        if !mask.as_slice::<bool>()[0] {
            continue;
        }
        let ti = f64::from(t.as_slice::<f32>()[0]);
        if let Some(b) = best {
            if ti >= b.t {
                continue;
            }
        }
        // 命中点与法线回世界系
        let pc = [ti * d[0], ti * d[1], ti * d[2]];
        let pw = camera
            .motor
            .reverse()
            .apply(&Multivector::point(pc[0], pc[1], pc[2]))
            .coords();
        let nc = n.as_slice::<f32>();
        let nw = camera
            .motor
            .reverse()
            .apply(&Multivector::vector(
                f64::from(nc[0]),
                f64::from(nc[1]),
                f64::from(nc[2]),
                0.0,
                0.0,
            ))
            .dir3();
        best = Some(PickHit {
            object: i,
            point: pw,
            normal: nw,
            t: ti,
        });
    }
    best
}

/// 增量渲染统计。
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IncrementalStats {
    /// 本帧实际重追的主光线数（全帧时 = `total`）。
    pub dirty: usize,
    /// 主光线总数（宽 × 高 × aa²）。
    pub total: usize,
    /// 是否退化为全帧重渲（原因见 `reason`）。
    pub full: bool,
    /// 退化原因：first / camera / lights / background / count / unbounded / threshold；
    /// 增量帧为 `objects`，完全无变化为 `clean`。
    pub reason: &'static str,
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
    /// 主光线逐对象屏幕区间剔除。增量渲染的子集光线包下标不是全帧下标，
    /// 子集追踪时必须关闭（次级光线的包络球剔除不受影响，仍可用）。
    pub(crate) cull_primary: bool,
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
            cull_primary: true,
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

    /// 一帧的预处理：光线包、背景、相机空间灯光、逐对象的相机空间/陈述参数。
    /// 全帧渲染与增量渲染共用（增量渲染还要拿 `params_list` 做帧间 diff）。
    fn prep(&mut self, scene: &Scene, camera: &PerspectiveCamera) -> Prep {
        self.cam = Some(*camera);
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
        // 相机空间参数 + 对象陈述参数全场景各建一次（阴影遮挡表 / UV 陈述共用）
        let mut params_list: Vec<GeometryParams> = Vec::with_capacity(scene.objects.len());
        let mut stated_list: Vec<GeometryParams> = Vec::with_capacity(scene.objects.len());
        for obj in scene.objects.iter() {
            params_list.push(geom_to_camera(
                &obj.geometry,
                &camera.motor.compose(&obj.motor()),
            ));
            stated_list.push(geom_to_camera(&obj.geometry, &obj.motor()));
        }
        Prep {
            rays,
            o,
            bg,
            lit,
            ambient,
            params_list,
            stated_list,
        }
    }

    /// SSAA 降采样 + sRGB 编码 + RGBA8：光线级 `(n_rays, 3)` → `(h, w, 4)`。
    fn resolve(&self, rgb_sr: &Array) -> Array {
        let n_rays = rgb_sr.shape()[0];
        // SSAA：子采样均值降采样
        let s = self.aa * self.aa;
        let mut rgb = if s > 1 {
            ck(ck(rgb_sr.reshape(&[s, n_rays / s, 3])).mean_axes(&[0], false))
        } else {
            rgb_sr.clone()
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
        ck(rgba.reshape(&[self.height, self.width, 4]))
    }

    pub fn render_with_truth(&mut self, scene: Scene, camera: PerspectiveCamera) -> (Array, Truth) {
        let p = self.prep(&scene, &camera);
        let n_rays = p.o.shape()[0];
        let in_medium = ck(ops::zeros::<bool>(&[n_rays]));
        let sigma = ck(ops::zeros::<f32>(&[n_rays]));
        let (rgb_sr, _t, truth) = self.trace(
            &scene,
            &p.params_list,
            &p.stated_list,
            &p.o,
            &p.rays,
            &p.lit,
            p.ambient,
            &p.bg,
            &in_medium,
            &sigma,
            0,
        );
        (
            self.resolve(&rgb_sr),
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
            } else if primary && self.cull_primary {
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

/// 一帧的预处理结果（全帧与增量渲染共用）。
struct Prep {
    rays: Array,
    o: Array,
    bg: Array,
    lit: Vec<Light>,
    ambient: Option<Light>,
    params_list: Vec<GeometryParams>,
    stated_list: Vec<GeometryParams>,
}

/// 稳定的指纹（帧间 diff 用）：`DefaultHasher::new()` 固定密钥，跨进程一致。
fn fp<T: std::fmt::Debug>(x: &T) -> u64 {
    use std::hash::Hasher;
    let mut h = std::collections::hash_map::DefaultHasher::new();
    std::hash::Hash::hash(&format!("{x:?}"), &mut h);
    h.finish()
}

/// 材质指纹。纹理只取尺寸/色彩空间签名：同一会话内资产文件不变；
/// 纹理内容真的变了请调用 [`IncrementalRenderer::invalidate`]。
fn material_fp(m: &crate::shading::Material) -> u64 {
    let map_sig = m.map.as_ref().map(|t| (t.width, t.height, t.is_linear));
    fp(&(
        m.kind,
        m.color,
        m.roughness,
        m.metalness,
        m.emissive,
        m.opacity,
        m.ior,
        m.absorption,
        map_sig,
    ))
}

/// 增量渲染器：缓存上一帧的光线级结果，只重追「值可能变化」的光线。
///
/// 脏集规则（保守，宁可多画不能漏画）：
/// - 首帧 / 相机 / 灯光 / 背景 / 对象数量变化 → 全帧；
/// - 变化对象的新、旧屏幕包围盒（新旧位置都要：离开的位置可能露出后面的东西）；
/// - 形状或透明度变化 → 阴影级联：可能被它（新旧两个位置）投影的接收对象一并置脏；
///   无界接收面是平面时解析阴影足迹，无法界定时 → 全帧；
/// - 正常模式下场景含透明对象且有任意变化 → 所有透明对象的像素一并置脏
///   （反射/折射次级光线可达任意对象）；
/// - 变化对象自身无法界定包围盒（无界几何）→ 全帧；
/// - 脏光线 ≥ 总数一半 → 全帧（与渲染器既有的 2× 回退一致）。
///
/// 正确性不变量：光线之间互相独立，同一光线在子集追踪与全帧追踪中取值逐位一致
/// （渲染器既有的剔除机制已依赖这一点）；测试用「增量 == 全帧」逐位断言看守。
/// 注意这**不是**分层合成：帧间复用的是上一帧的像素值，不是某个"图层"。
pub struct IncrementalRenderer {
    r: Renderer,
    prev: Option<PrevFrame>,
}

struct PrevFrame {
    /// 上一帧光线级颜色（n_rays, 3），未降采样。
    rays_rgb: Array,
    /// 上一帧最终图像（h, w, 4），完全无变化时直接返回。
    image: Array,
    camera_fp: u64,
    lights_fp: u64,
    bg_fp: u64,
    shape_fps: Vec<u64>,
    full_fps: Vec<u64>,
    opacity: Vec<f64>,
    /// 上一帧的相机空间参数（相机不变时才有效；相机变化已回退全帧）。
    params: Vec<GeometryParams>,
}

/// 一次渲染需要的指纹包。
struct Fps {
    camera: u64,
    lights: u64,
    bg: u64,
    shape: Vec<u64>,
    full: Vec<u64>,
    opacity: Vec<f64>,
}

impl IncrementalRenderer {
    pub fn new(width: i32, height: i32, aa: i32, max_depth: i32) -> IncrementalRenderer {
        IncrementalRenderer {
            r: Renderer::new(width, height, aa, max_depth),
            prev: None,
        }
    }

    /// 设定渲染模式（链式）。
    pub fn with_mode(mut self, mode: RenderMode) -> IncrementalRenderer {
        self.r.mode = mode;
        self
    }

    pub fn width(&self) -> i32 {
        self.r.width
    }
    pub fn height(&self) -> i32 {
        self.r.height
    }
    pub fn aa(&self) -> i32 {
        self.r.aa
    }
    pub fn mode(&self) -> RenderMode {
        self.r.mode
    }

    /// 强制下一帧全量（场景在渲染器视野之外被改写时用，比如纹理文件变了）。
    pub fn invalidate(&mut self) {
        self.prev = None;
    }

    pub fn render(
        &mut self,
        scene: &Scene,
        camera: &PerspectiveCamera,
    ) -> (Array, IncrementalStats) {
        let (w, h, k) = (self.r.width, self.r.height, self.r.aa);
        let total = (w as usize) * (h as usize) * (k as usize) * (k as usize);
        let fps = Fps {
            camera: fp(camera),
            lights: fp(&scene.lights),
            bg: fp(&scene.background),
            shape: scene
                .objects
                .iter()
                .map(|o| fp(&(o.motor(), &o.geometry)))
                .collect(),
            full: scene
                .objects
                .iter()
                .map(|o| fp(&(o.motor(), &o.geometry)) ^ material_fp(&o.material))
                .collect(),
            opacity: scene.objects.iter().map(|o| o.material.opacity).collect(),
        };
        let p = self.r.prep(scene, camera);

        let full_reason = match &self.prev {
            None => Some("first"),
            Some(prev) => {
                if prev.camera_fp != fps.camera {
                    Some("camera")
                } else if prev.lights_fp != fps.lights {
                    Some("lights")
                } else if prev.bg_fp != fps.bg {
                    Some("background")
                } else if prev.full_fps.len() != fps.full.len() {
                    Some("count")
                } else {
                    None
                }
            }
        };
        if let Some(reason) = full_reason {
            return self.render_full(scene, p, fps, total, reason);
        }
        let prev = self.prev.as_ref().expect("prev checked");
        let changed: Vec<usize> = (0..fps.full.len())
            .filter(|&i| fps.full[i] != prev.full_fps[i])
            .collect();
        if changed.is_empty() {
            return (
                prev.image.clone(),
                IncrementalStats {
                    dirty: 0,
                    total,
                    full: false,
                    reason: "clean",
                },
            );
        }

        let n_rays = p.rays.shape()[0] as usize;
        let mut dirty = vec![false; n_rays];
        // 返回 false = 无法保守界定，调用方回退全帧。
        let mut ok = true;
        let add = |dirty: &mut [bool], idx: Option<Vec<i32>>| -> bool {
            match idx {
                None => false,
                Some(v) => {
                    for i in v {
                        dirty[i as usize] = true;
                    }
                    true
                }
            }
        };
        for &i in &changed {
            ok = ok
                && add(
                    &mut dirty,
                    primary_ray_indices(&p.params_list[i], camera, w, h, k),
                );
            ok = ok
                && add(
                    &mut dirty,
                    primary_ray_indices(&prev.params[i], camera, w, h, k),
                );
        }
        // 透明级联：反射/折射次级光线可达任意对象。
        if ok && self.r.mode == RenderMode::Normal {
            for (j, &op) in fps.opacity.iter().enumerate() {
                if op < 1.0 {
                    ok = ok
                        && add(
                            &mut dirty,
                            primary_ray_indices(&p.params_list[j], camera, w, h, k),
                        );
                }
            }
        }
        // 阴影级联：形状/透明度变化的对象，在新旧两个遮挡位置上能投到谁谁脏。
        if ok && !p.lit.is_empty() {
            let sphere_of = |pa: &GeometryParams| geom_bounds(pa).map(|b| aabb_sphere(&b));
            let spheres: Vec<Option<([f64; 3], f64)>> =
                p.params_list.iter().map(sphere_of).collect();
            let prev_spheres: Vec<Option<([f64; 3], f64)>> =
                prev.params.iter().map(sphere_of).collect();
            'outer: for &i in &changed {
                let moved = fps.shape[i] != prev.shape_fps[i];
                let op_changed = (fps.opacity[i] - prev.opacity[i]).abs() > 1e-12;
                if !moved && !op_changed {
                    continue;
                }
                for occ in [prev_spheres[i], spheres[i]].into_iter().flatten() {
                    for light in &p.lit {
                        for j in 0..scene.objects.len() {
                            match &p.params_list[j] {
                                GeometryParams::PlaneParams(pl) => {
                                    // 无界接收面：解析阴影足迹
                                    match shadow_on_plane(light, occ, pl.n, pl.d) {
                                        Some((c, r)) => {
                                            if !add(
                                                &mut dirty,
                                                sphere_ray_indices(c, r, camera, w, h, k),
                                            ) {
                                                ok = false;
                                                break 'outer;
                                            }
                                        }
                                        None => {
                                            ok = false;
                                            break 'outer;
                                        }
                                    }
                                }
                                params => {
                                    let Some(rb) = sphere_of(params) else {
                                        ok = false;
                                        break 'outer;
                                    };
                                    if shadow_reaches(light, occ, rb)
                                        && !add(
                                            &mut dirty,
                                            primary_ray_indices(params, camera, w, h, k),
                                        )
                                    {
                                        ok = false;
                                        break 'outer;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        let dirty_count = dirty.iter().filter(|&&b| b).count();
        if !ok {
            return self.render_full(scene, p, fps, total, "unbounded");
        }
        if dirty_count * 2 >= n_rays {
            return self.render_full(scene, p, fps, total, "threshold");
        }

        // 子集追踪：只重追脏光线，散布回上一帧的光线级缓存。
        let ids: Vec<i32> = dirty
            .iter()
            .enumerate()
            .filter(|(_, &b)| b)
            .map(|(i, _)| i as i32)
            .collect();
        let ids_a = Array::from_slice(&ids, &[ids.len() as i32]);
        let n_sub = ids.len() as i32;
        let d_sub = ck(p.rays.take_axis(&ids_a, 0));
        let o_sub = ck(ops::zeros_like(&d_sub));
        let bg_sub = ck(ops::broadcast_to(
            arr3v(scene.background.rgb()),
            &[n_sub, 3],
        ));
        let in_medium = ck(ops::zeros::<bool>(&[n_sub]));
        let sigma = ck(ops::zeros::<f32>(&[n_sub]));
        self.r.cull_primary = false; // 子集光线包：全帧屏幕区间的下标不适用
        let (rgb_sub, _t, _truth) = self.r.trace(
            scene,
            &p.params_list,
            &p.stated_list,
            &o_sub,
            &d_sub,
            &p.lit,
            p.ambient,
            &bg_sub,
            &in_medium,
            &sigma,
            0,
        );
        self.r.cull_primary = true;
        let rays_rgb = ck(ops::indexing::scatter_single(
            &prev.rays_rgb,
            &ids_a,
            ck(rgb_sub.reshape(&[n_sub, 1, 3])),
            0,
        ));
        let image = self.r.resolve(&rays_rgb);
        self.prev = Some(PrevFrame {
            rays_rgb,
            image: image.clone(),
            camera_fp: fps.camera,
            lights_fp: fps.lights,
            bg_fp: fps.bg,
            shape_fps: fps.shape,
            full_fps: fps.full,
            opacity: fps.opacity,
            params: p.params_list,
        });
        (
            image,
            IncrementalStats {
                dirty: dirty_count,
                total,
                full: false,
                reason: "objects",
            },
        )
    }

    fn render_full(
        &mut self,
        scene: &Scene,
        p: Prep,
        fps: Fps,
        total: usize,
        reason: &'static str,
    ) -> (Array, IncrementalStats) {
        let n_rays = p.o.shape()[0];
        let in_medium = ck(ops::zeros::<bool>(&[n_rays]));
        let sigma = ck(ops::zeros::<f32>(&[n_rays]));
        let (rgb_sr, _t, _truth) = self.r.trace(
            scene,
            &p.params_list,
            &p.stated_list,
            &p.o,
            &p.rays,
            &p.lit,
            p.ambient,
            &p.bg,
            &in_medium,
            &sigma,
            0,
        );
        let image = self.r.resolve(&rgb_sr);
        self.prev = Some(PrevFrame {
            rays_rgb: rgb_sr,
            image: image.clone(),
            camera_fp: fps.camera,
            lights_fp: fps.lights,
            bg_fp: fps.bg,
            shape_fps: fps.shape,
            full_fps: fps.full,
            opacity: fps.opacity,
            params: p.params_list,
        });
        (
            image,
            IncrementalStats {
                dirty: total,
                total,
                full: true,
                reason,
            },
        )
    }
}
