use crate::geom_kernels::geom_to_camera;
use crate::geometry_ops::{geom_bounds, geom_intersect, geom_shadow, geom_uv};
use crate::mesh_raster::rasterize_meshes;
use crate::mlxops::*;
use crate::scene::{Mesh, PerspectiveCamera, Scene};
use crate::scene_graph::{vec3_dot, vec3_unit};
use crate::shading::{
    light_direction_at, light_far, light_to_camera, shade_batched, Light, LightKind,
};
use crate::texture::WrapMode;
use cga_core::{vec3_cross, Geometry, GeometryParams};
use mlx_rs::{ops, Array};

fn view_frame(camera: &PerspectiveCamera) -> ([[f64; 3]; 3], [f64; 3]) {
    let forward = vec3_unit([
        camera.target[0] - camera.position[0],
        camera.target[1] - camera.position[1],
        camera.target[2] - camera.position[2],
    ]);
    let right = vec3_unit(vec3_cross(forward, camera.up));
    let up = vec3_cross(right, forward);
    let basis = [right, [-up[0], -up[1], -up[2]], forward];
    let offset = [
        vec3_dot(basis[0], camera.position),
        vec3_dot(basis[1], camera.position),
        vec3_dot(basis[2], camera.position),
    ];
    (basis, offset)
}

fn outward(points: &Array, basis: &[[f64; 3]; 3], offset: &[f64; 3]) -> Array {
    let q = ck(points.add(&arr3v(*offset)));
    let mut out = ck(ops::zeros::<f32>(&[1, 3]));
    for axis in 0..3usize {
        let at = ck(ck(q.take_axis(Array::from_int(axis as i32), 1)).expand_dims(1));
        let term = ck(at.multiply(&arr3v(basis[axis])));
        out = if axis == 0 { term } else { ck(out.add(&term)) };
    }
    out
}

#[derive(Clone, Debug)]
pub struct Truth {
    pub hit: Array,

    pub t: Array,

    pub normal: Array,

    pub index: Array,

    pub vis: Vec<Array>,
}

#[derive(Debug)]
pub struct Renderer {
    pub width: i32,
    pub height: i32,
    pub aa: i32,
    pub max_depth: i32,
    pub cam: Option<PerspectiveCamera>,
}

pub fn renderer(width: i32, height: i32, aa: i32, max_depth: i32) -> Renderer {
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

impl Renderer {
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
                lit.push(light_to_camera(*light, camera.motor));
            }
        }

        let mut ray_objs: Vec<Mesh> = Vec::new();
        let mut mesh_objs: Vec<Mesh> = Vec::new();
        for obj in &scene.objects {
            if matches!(obj.geometry, Geometry::TrimeshGeometry(_)) {
                mesh_objs.push(obj.clone());
            } else {
                ray_objs.push(obj.clone());
            }
        }
        let mut s2 = scene.clone();
        s2.objects = ray_objs;
        let in_medium = ck(ops::zeros::<bool>(&[n_rays]));
        let sigma = ck(ops::zeros::<f32>(&[n_rays]));
        let (mut rgb, t, truth) =
            self.trace(&s2, &o, &rays, &lit, ambient, &bg, &in_medium, &sigma, 0);
        let s = self.aa * self.aa;
        if s > 1 {
            rgb = ck(ck(rgb.reshape(&[s, n_rays / s, 3])).mean_axes(&[0], false));
        }

        let mut dz = ck(t.multiply(ck(rays.take_axis(Array::from_int(2), 1))));
        if s > 1 {
            dz = ck(ck(dz.reshape(&[s, n_rays / s])).min_axes(&[0], false));
        }
        if !mesh_objs.is_empty() {
            let hh = self.height;
            let ww = self.width;
            let fy = f64::from(hh) / (2.0 * (camera.fov.to_radians() / 2.0).tan());
            let fx = fy * camera.aspect;
            let cx = f64::from(ww - 1) / 2.0;
            let cy = f64::from(hh - 1) / 2.0;
            let rr = rasterize_meshes(&mesh_objs, &camera, ww, hh, fx, fy, cx, cy, &lit, ambient);
            let rt_d = ck(dz.reshape(&[hh, ww]));
            let closer = ck(rr.depth.lt(&rt_d));

            rgb = ck(ops::select(
                ck(ck(closer.reshape(&[hh * ww])).expand_dims(1)),
                ck(rr.color.reshape(&[hh * ww, 3])),
                &rgb,
            ));
        }
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
        o: &Array,
        d: &Array,
        lit: &[Light],
        ambient: Option<Light>,
        bg: &Array,
        in_medium: &Array,
        sigma: &Array,
        depth: i32,
    ) -> (Array, Array, Option<Truth>) {
        let (hit, t, n0, local, op, ior, abso, index, vis) =
            self.nearest(scene, o, d, lit, ambient, depth == 0);
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
                    &ck(p.add(s_mul(&n, 1e-3))),
                    &d_r,
                    lit,
                    ambient,
                    bg,
                    in_medium,
                    sigma,
                    depth + 1,
                );
                let (refr, _, _) = self.trace(
                    scene,
                    &ck(p.subtract(s_mul(&n, 1e-3))),
                    &d_t,
                    lit,
                    ambient,
                    bg,
                    &entering,
                    &sig_next,
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
        let cam = self.cam.unwrap_or_else(|| panic!("no camera"));
        let frame = view_frame(&cam);
        let mut params_list: Vec<GeometryParams> = Vec::new();
        for (i, obj) in objs.iter().enumerate() {
            let wm = cam.motor.compose(&obj.motor());
            let params = geom_to_camera(&obj.geometry, &wm);
            params_list.push(params.clone());
            if primary {
                if let Some(b) = geom_bounds(&params) {
                    if b[1][2] <= 1e-6 {
                        continue;
                    }
                }
            }
            let (t, n_i, mask) = geom_intersect(&params, o, d);
            let hit_point = ck(o.add(ck(ck(t.expand_dims(1)).multiply(d))));
            let stated = geom_to_camera(&obj.geometry, &obj.motor());
            let uv_i = geom_uv(
                &stated,
                &outward(&hit_point, &frame.0, &frame.1),
                &outward(&n_i, &frame.0, &[0.0, 0.0, 0.0]),
            );
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
            let (ld, _) = light_direction_at(*light, &p);
            let far = light_far(*light, &p);
            let mut v = ck(ops::ones::<f32>(&[n_rays]));
            for (j, obj) in objs.iter().enumerate() {
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

pub fn render_frame(
    scene: Scene,
    camera: PerspectiveCamera,
    width: i32,
    height: i32,
    aa: i32,
) -> Array {
    let mut r = renderer(width, height, aa, 3);
    r.render(scene, camera)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image_io::save_frame_png;
    use crate::scene::{mesh, perspective_camera, scene, MeshParams};
    use crate::scene_graph::{color_hex, srgb_to_linear};
    use crate::shading::{
        ambient_light, basic_material, directional_light, standard_material, MaterialParams,
    };
    use cga_core::{
        box_geometry, cone_geometry, cyclide_geometry, ellipsoid_geometry, plane_geometry,
        sphere_geometry, torus_geometry, trimesh_geometry, Geometry,
    };
    use mlx_rs::Array;

    const ARTIFACTS: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../artifacts/tests");

    fn ensure_artifacts() {
        std::fs::create_dir_all(ARTIFACTS).unwrap();
    }

    fn data_f32(img: &Array) -> Vec<f32> {
        img.eval().unwrap();
        img.as_slice::<f32>().to_vec()
    }

    fn std_red_material() -> crate::shading::Material {
        standard_material(MaterialParams {
            color: color_hex(0xC0392B),
            roughness: 0.3,
            metalness: 0.1,
            emissive: color_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        })
    }

    fn render_center(geom: Geometry, pos: [f64; 3], name: &str) -> [f32; 3] {
        let mut sc = scene(None);
        sc.add_mesh(mesh(MeshParams {
            geometry: geom,
            material: std_red_material(),
            position: pos,
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]));
        sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3));
        let mut cam = perspective_camera(
            40.0,
            1.0,
            0.1,
            100.0,
            [pos[0], pos[1], pos[2] + 4.0],
            pos,
            [0.0, 1.0, 0.0],
        );
        cam.look_at(pos, None);
        let img = render_frame(sc, cam, 120, 120, 1);
        ensure_artifacts();
        save_frame_png(&format!("{}/{}.png", ARTIFACTS, name), &img);
        let data = data_f32(&img);
        let idx = 60 * 120 * 4 + 60 * 4;
        [data[idx], data[idx + 1], data[idx + 2]]
    }

    #[test]
    fn test_render_sphere_hits() {
        let c = render_center(
            Geometry::SphereGeometry(sphere_geometry(1.0)),
            [0.0, 0.0, 0.0],
            "sphere",
        );
        assert!(c[0] > c[2]);
        assert!(c[0] < 250.0);
    }

    #[test]
    fn test_render_cone_hits() {
        let c = render_center(
            Geometry::ConeGeometry(cone_geometry(0.8, 2.0)),
            [0.0, 0.0, 0.0],
            "cone",
        );
        assert!(c[0] > c[2]);
    }

    #[test]
    fn test_render_ellipsoid_hits() {
        let c = render_center(
            Geometry::EllipsoidGeometry(ellipsoid_geometry(1.0, 0.6, 0.8)),
            [0.0, 0.0, 0.0],
            "ellipsoid",
        );
        assert!(c[0] > c[2]);
    }

    #[test]
    fn test_render_cyclide_nonempty() {
        let mut sc = scene(None);
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::CyclideGeometry(cyclide_geometry(1.0, 0.98, 0.3, [0.0, 0.0, 0.0])),
            material: std_red_material(),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]));
        sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3));
        let mut cam = perspective_camera(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let img = render_frame(sc, cam, 120, 120, 1);
        ensure_artifacts();
        save_frame_png(&format!("{}/cyclide.png", ARTIFACTS), &img);
        let data = data_f32(&img);
        let mut hit = 0;
        for i in 0..120 * 120 {
            if data[i * 4] > data[i * 4 + 2] + 20.0 {
                hit += 1;
            }
        }
        assert!(hit > 100);
    }

    #[test]
    fn test_render_torus_nonempty() {
        let mut sc = scene(None);
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::TorusGeometry(torus_geometry(1.0, 0.3)),
            material: std_red_material(),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]));
        sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3));
        let mut cam = perspective_camera(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let img = render_frame(sc, cam, 120, 120, 1);
        ensure_artifacts();
        save_frame_png(&format!("{}/torus.png", ARTIFACTS), &img);
        let data = data_f32(&img);
        let mut nonbg = 0;
        for i in 0..120 * 120 {
            if data[i * 4] < 200.0 {
                nonbg += 1;
            }
        }
        assert!(nonbg > 100);
    }

    #[test]
    fn test_render_trimesh_nonempty() {
        let mut sc = scene(None);
        let verts: [[f64; 3]; 4] = [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [-0.5, 0.866, 0.0],
            [0.0, 0.0, -1.0],
        ];
        let faces: [[i32; 3]; 4] = [[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]];
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::TrimeshGeometry(trimesh_geometry(&verts, &faces)),
            material: std_red_material(),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.5, 1.0, 0.5]));
        sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.3));
        let mut cam = perspective_camera(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let img = render_frame(sc, cam, 120, 120, 1);
        ensure_artifacts();
        save_frame_png(&format!("{}/trimesh.png", ARTIFACTS), &img);
        let data = data_f32(&img);
        let mut hit = 0;
        for i in 0..120 * 120 {
            if data[i * 4] > data[i * 4 + 2] + 20.0 {
                hit += 1;
            }
        }
        assert!(hit > 100);
    }

    fn rq_linear_to_srgb255(lum: f64) -> i32 {
        let l = lum.clamp(0.0, 1.0);
        let s = if l <= 0.0031308 {
            12.92 * l
        } else {
            1.055 * l.powf(1.0 / 2.4) - 0.055
        };
        let v = (s * 255.0 + 0.5) as i32;
        if v < 0 {
            return 0;
        }
        if v > 255 {
            return 255;
        }
        v
    }

    fn rq_wall_scene() -> Scene {
        let mut sc = scene(None);
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::PlaneGeometry(plane_geometry([0.0, 0.0, -1.0], -4.0)),
            material: basic_material(color_hex(0xCC3333), 1.0),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc
    }

    fn rq_head_on_cam() -> PerspectiveCamera {
        let mut cam = perspective_camera(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 1.0], None);
        cam
    }

    fn rq_px(img: &Array, w: usize, row: usize, col: usize) -> [f32; 3] {
        let d = data_f32(img);
        let idx = (row * w + col) * 4;
        [d[idx], d[idx + 1], d[idx + 2]]
    }

    #[test]
    fn test_render_srgb_roundtrip() {
        let mut r = renderer(64, 64, 1, 3);
        let img = r.render(rq_wall_scene(), rq_head_on_cam());
        let p = rq_px(&img, 64, 32, 32);
        assert!(p[0] as i32 == 204);
        assert!(p[1] as i32 == 51);
        assert!(p[2] as i32 == 51);
    }

    #[test]
    fn test_render_ior1_invisible() {
        let mut sc = rq_wall_scene();

        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::SphereGeometry(sphere_geometry(0.8)),
            material: standard_material(MaterialParams {
                color: color_hex(0xAAD4FF),
                roughness: 0.5,
                metalness: 0.0,
                emissive: color_hex(0x000000),
                opacity: 0.0,
                ior: 1.0,
                absorption: 0.0,
            }),
            position: [0.0, 0.0, 2.2],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut r = renderer(64, 64, 1, 3);
        let cam = rq_head_on_cam();
        let img_a = r.render(sc, cam);
        let a = rq_px(&img_a, 64, 32, 32);
        let img_b = r.render(rq_wall_scene(), cam);
        let b = rq_px(&img_b, 64, 32, 32);
        for i in 0..3 {
            assert!((f64::from(a[i]) - f64::from(b[i])).abs() <= 1.0);
        }
    }

    fn rq_slab(depth: f64, absorption: f64) -> [f32; 3] {
        let mut sc = rq_wall_scene();
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::BoxGeometry(box_geometry(3.0, 3.0, depth)),
            material: standard_material(MaterialParams {
                color: color_hex(0xFFFFFF),
                roughness: 0.5,
                metalness: 0.0,
                emissive: color_hex(0x000000),
                opacity: 0.0,
                ior: 1.0,
                absorption,
            }),
            position: [0.0, 0.0, 2.5],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut r = renderer(64, 64, 1, 3);
        let img = r.render(sc, rq_head_on_cam());
        rq_px(&img, 64, 32, 32)
    }

    #[test]
    fn test_render_beer_absorption() {
        let wall = [204.0 / 255.0, 51.0 / 255.0, 51.0 / 255.0];
        for depth in [0.2, 1.0] {
            let px = rq_slab(depth, 0.8);
            let trans = (-0.8_f64 * depth).exp();
            for i in 0..3 {
                let want = rq_linear_to_srgb255(trans * srgb_to_linear(wall[i]));
                assert!((f64::from(px[i]) - f64::from(want)).abs() <= 2.0);
            }
        }

        let px0 = rq_slab(1.0, 0.0);
        assert!(px0[0] as i32 == 204 && px0[1] as i32 == 51 && px0[2] as i32 == 51);
    }

    fn rq_shadow_scene(opacity: Option<f64>) -> Scene {
        let mut sc = scene(None);
        sc.add_mesh(mesh(MeshParams {
            geometry: Geometry::PlaneGeometry(plane_geometry([0.0, 1.0, 0.0], 0.0)),
            material: standard_material(MaterialParams {
                color: color_hex(0xFFFFFF),
                roughness: 1.0,
                metalness: 0.0,
                emissive: color_hex(0x000000),
                opacity: 1.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(directional_light(color_hex(0xFFFFFF), 0.8, [0.0, 1.0, 0.0]));
        sc.add_light(ambient_light(color_hex(0xFFFFFF), 0.2));
        if let Some(o) = opacity {
            sc.add_mesh(mesh(MeshParams {
                geometry: Geometry::SphereGeometry(sphere_geometry(0.5)),
                material: standard_material(MaterialParams {
                    color: color_hex(0xFFFFFF),
                    roughness: 1.0,
                    metalness: 0.0,
                    emissive: color_hex(0x000000),
                    opacity: o,
                    ior: 1.0,
                    absorption: 0.0,
                }),
                position: [2.0, 1.5, 3.0],
                rotation_axis: [0.0, 0.0, 1.0],
                rotation_angle: 0.0,
                motor: None,
            }));
        }
        sc
    }

    fn rq_shadow_cam() -> PerspectiveCamera {
        let mut cam = perspective_camera(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.8, -1.0],
            [2.0, 0.0, 3.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([2.0, 0.0, 3.0], None);
        cam
    }

    #[test]
    fn test_render_shadow_umbra_is_ambient_only() {
        let mut r = renderer(96, 96, 1, 3);
        let img = r.render(rq_shadow_scene(Some(1.0)), rq_shadow_cam());
        let p = rq_px(&img, 96, 48, 48);
        let want = rq_linear_to_srgb255(0.2);
        assert!((f64::from(p[0]) - f64::from(want)).abs() <= 2.0);
    }
}
