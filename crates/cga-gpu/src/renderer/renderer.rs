use super::*;

#[derive(Debug)]
pub struct Renderer {
    pub width: i32,
    pub height: i32,
    pub aa: i32,
    pub max_depth: i32,
    pub cam: Option<PerspectiveCamera>,
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
        }
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

        let mut mesh_objs: Vec<Mesh> = Vec::new();
        for obj in &scene.objects {
            if matches!(
                obj.geometry,
                Geometry::TrimeshGeometry(_) | Geometry::BezierPatchGeometry(_)
            ) {
                mesh_objs.push(obj.clone());
            }
        }
        // 追踪场景保留全部对象：主可见性在 nearest(primary) 里跳过网格类
        // （由光栅路径负责），但网格参与阴影遮挡（D2）与反射/折射（D3）。
        let s2 = scene.clone();
        // 相机空间参数 + 对象陈述参数全场景各建一次（阴影遮挡表/UV 陈述共用；
        // 此前 nearest 在每个追踪深度重建一次，大网格下是 O(depth×faces) 的克隆）。
        let mut params_list: Vec<GeometryParams> = Vec::with_capacity(s2.objects.len());
        let mut stated_list: Vec<GeometryParams> = Vec::with_capacity(s2.objects.len());
        let mut opacities: Vec<f32> = Vec::with_capacity(s2.objects.len());
        let mut is_mesh: Vec<bool> = Vec::with_capacity(s2.objects.len());
        for obj in &s2.objects {
            params_list.push(geom_to_camera(
                &obj.geometry,
                &camera.motor.compose(&obj.motor()),
            ));
            stated_list.push(geom_to_camera(&obj.geometry, &obj.motor()));
            opacities.push(obj.material.opacity as f32);
            is_mesh.push(matches!(
                obj.geometry,
                Geometry::TrimeshGeometry(_) | Geometry::BezierPatchGeometry(_)
            ));
        }
        let in_medium = ck(ops::zeros::<bool>(&[n_rays]));
        let sigma = ck(ops::zeros::<f32>(&[n_rays]));
        let (rgb_sr, t, truth) = self.trace(
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
            &is_mesh,
            0,
        );
        let k = self.aa;
        let (ww, hh) = (self.width, self.height);
        // super-res 重排：光线按 (j,i) 子采样索引堆叠，重排到 (y*k+j, x*k+i) 栅格
        let mut rgb = ck(ck(rgb_sr.reshape(&[k, k, hh, ww, 3]))
            .transpose_axes(&[2, 0, 3, 1, 4])
            .unwrap()
            .reshape(&[hh * k, ww * k, 3]));
        let mut dz = ck(t.multiply(ck(rays.take_axis(Array::from_int(2), 1))));
        dz = ck(ck(dz.reshape(&[k, k, hh, ww]))
            .transpose_axes(&[2, 0, 3, 1])
            .unwrap()
            .reshape(&[hh * k, ww * k]));
        if !mesh_objs.is_empty() {
            let (sw, sh) = (ww * k, hh * k);
            let fy2 = f64::from(sh) / (2.0 * (camera.fov.to_radians() / 2.0).tan());
            let fx2 = fy2 * camera.aspect;
            let rr = rasterize_meshes(
                &mesh_objs,
                &camera,
                sw,
                sh,
                fx2,
                fy2,
                f64::from(sw - 1) / 2.0,
                f64::from(sh - 1) / 2.0,
                &lit,
                ambient,
                &params_list,
                &opacities,
                &is_mesh,
            );
            let closer = ck(ck(rr.depth.lt(&dz)).logical_and(&rr.hit));
            // D4：半透明网格与光线结果 alpha 混合（无折射弯曲的近似，见 docs）
            let op = ck(rr.opacity.expand_dims(2));
            let blended =
                ck(ck(rr.color.multiply(&op)).add(&ck(rgb.multiply(&ck(fs(1.0).subtract(&op))))));
            rgb = ck(ops::select(&ck(closer.expand_dims(2)), &blended, &rgb));
        }
        // 单点降采样：颜色与深度在同一 super-res 栅格上合成（消边界 halo，D7）
        rgb = ck(ck(rgb.reshape(&[hh, k, ww, k, 3])).mean_axes(&[1, 3], false));
        rgb = ck(rgb.reshape(&[hh * ww, 3]));
        rgb = s_clip(&rgb, 0.0, 1.0);
        rgb = ck(ops::select(
            s_le(&rgb, 0.0031308),
            s_mul(&rgb, 12.92),
            s_sub(&s_mul(&s_pow(&rgb, 1.0 / 2.4), 1.055), 0.055),
        ));
        let mut rgba = ck(ops::concatenate(
            &[&rgb, &ck(ops::ones::<f32>(&[hh * ww, 1]))],
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
        is_mesh: &[bool],
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
            is_mesh,
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
        if depth < self.max_depth {
            let need = ck(hit.logical_and(s_lt(&op, 1.0)));
            if ck(need.sum(None)).item_cast::<f32>() > 0.0 {
                let eta = ck(ops::select(
                    ck(in_medium.expand_dims(1)),
                    ck(ior.expand_dims(1)),
                    ck(fs(1.0).divide(ck(ior.expand_dims(1)))),
                ));
                let k = ck(fs(1.0).subtract(ck(ck(eta.multiply(&eta))
                    .multiply(ck(fs(1.0).subtract(ck(cos_i.multiply(&cos_i))))))));
                let cos_t = ck(s_max(&k, 0.0).sqrt());
                let g = ck(fs(1.0).divide(&eta));
                let rs = ck(ck(cos_i.subtract(ck(g.multiply(&cos_t))))
                    .divide(s_max(&ck(cos_i.add(ck(g.multiply(&cos_t)))), 1e-12)));
                let rp = ck(ck(cos_t.subtract(ck(g.multiply(&cos_i))))
                    .divide(s_max(&ck(cos_t.add(ck(g.multiply(&cos_i)))), 1e-12)));
                let mut fres = s_mul(&ck(ck(rs.multiply(&rs)).add(ck(rp.multiply(&rp)))), 0.5);
                fres = ck(ops::select(s_le(&k, 0.0), ck(ops::ones_like(&fres)), &fres));
                let p = ck(o.add(ck(ck(t.expand_dims(1)).multiply(d))));
                let d_r = ck(d.add(ck(n.multiply(s_mul(&cos_i, 2.0)))));
                let d_t = ck(ck(d.multiply(&eta))
                    .add(ck(n.multiply(ck(ck(eta.multiply(&cos_i)).subtract(&cos_t))))));
                let entering = ck(in_medium.logical_not());
                let sig_next = ck(ops::select(&entering, &abso, fs(0.0)));
                let (refl, _, _) = self.trace(
                    scene,
                    params_list,
                    stated_list,
                    &ck(p.add(s_mul(&n, 1e-3))),
                    &d_r,
                    lit,
                    ambient,
                    bg,
                    in_medium,
                    sigma,
                    is_mesh,
                    depth + 1,
                );
                let (refr, _, _) = self.trace(
                    scene,
                    params_list,
                    stated_list,
                    &ck(p.subtract(s_mul(&n, 1e-3))),
                    &d_t,
                    lit,
                    ambient,
                    bg,
                    &entering,
                    &sig_next,
                    is_mesh,
                    depth + 1,
                );
                let body = ck(ck(ck(op.expand_dims(1)).multiply(&local))
                    .add(ck(
                        ck(fs(1.0).subtract(ck(op.expand_dims(1)))).multiply(&refr)
                    )));
                let glass =
                    ck(ck(fres.multiply(&refl))
                        .add(ck(ck(fs(1.0).subtract(&fres)).multiply(&body))));
                result = ck(ops::select(ck(need.expand_dims(1)), &glass, &result));
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
        is_mesh: &[bool],
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
        let cam = self.cam.unwrap_or_else(|| panic!("no camera"));
        let frame = view_frame(&cam);
        for (i, obj) in objs.iter().enumerate() {
            let params = &params_list[i];
            if primary
                && matches!(
                    obj.geometry,
                    Geometry::TrimeshGeometry(_) | Geometry::BezierPatchGeometry(_)
                )
            {
                // 主可见性交给光栅路径；网格类仅在阴影（下方）和
                // 非主光线（反射/折射，primary=false）里参与求交。
                continue;
            }
            if primary {
                if let Some(b) = geom_bounds(params) {
                    if b[1][2] <= 1e-6 {
                        continue;
                    }
                }
            }
            let (t, n_i, mask) = geom_intersect(params, o, d);
            let hit_point = ck(o.add(ck(ck(t.expand_dims(1)).multiply(d))));
            let uv_i = if matches!(
                obj.geometry,
                Geometry::TrimeshGeometry(_) | Geometry::BezierPatchGeometry(_)
            ) {
                // 网格类 UV 由光栅路径透视校正插值；此处避免大数组克隆
                ck(ops::zeros::<f32>(&[n_rays, 2]))
            } else {
                geom_uv(
                    &stated_list[i],
                    &outward(&hit_point, &frame.0, &frame.1),
                    &outward(&n_i, &frame.0, &[0.0, 0.0, 0.0]),
                )
            };
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
        let hit = ck(best_t.is_finite());
        // 网格遮挡物的阴影测试是 O(光线×面数)；没有一条光线命中任何物体时
        // 无表面需要可见性（纯网格场景的主光线全部由光栅路径着色），跳过。
        let any_hit = ck(hit.sum(None)).item_cast::<i32>() > 0;

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
                if is_mesh.get(j).copied().unwrap_or(false) && !any_hit {
                    continue; // 无命中 → 无表面需要阴影
                }
                let (st, m) = geom_shadow(&params_list[j], &p_s, &ld);
                let occ = if far.ndim() == 1 {
                    ck(m.logical_and(ck(st.lt(&far))))
                } else {
                    m
                };
                v = ck(v.multiply(ck(ops::select(
                    &occ,
                    fs(1.0 - obj.material.opacity),
                    fs(1.0),
                ))));
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
