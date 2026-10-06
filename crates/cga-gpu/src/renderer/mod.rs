use crate::geom_kernels::geom_to_camera;
use crate::geometry_ops::{geom_bounds, geom_intersect, geom_shadow, geom_uv};
use crate::mlxops::*;
use crate::scene::{Object, PerspectiveCamera, Scene};
use crate::scene_graph::{vec3_dot, vec3_unit};
use crate::shading::{shade_batched, Light, LightKind};
use crate::texture::WrapMode;
use cga_core::{vec3_cross, GeometryParams};
use mlx_rs::{ops, Array};

pub mod truth;
pub use self::truth::*;
pub mod renderer;
pub use self::renderer::*;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::image_io::save_frame_png;
    use crate::scene::{Object, ObjectParams, PerspectiveCamera};
    use crate::scene_graph::{srgb_to_linear, Color};
    use crate::shading::{Light, Material, MaterialParams};
    use cga_core::{
        BoxGeometry, ConeGeometry, CyclideGeometry, EllipsoidGeometry, Geometry, PlaneGeometry,
        SphereGeometry, TorusGeometry,
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
        Material::standard(MaterialParams {
            color: Color::from_hex(0xC0392B),
            roughness: 0.3,
            metalness: 0.1,
            emissive: Color::from_hex(0x000000),
            opacity: 1.0,
            ior: 1.5,
            absorption: 0.0,
        })
    }

    fn render_center(geom: Geometry, pos: [f64; 3], name: &str) -> [f32; 3] {
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: geom,
            material: std_red_material(),
            position: pos,
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.8,
            [0.5, 1.0, 0.5],
        ));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.3));
        let mut cam = PerspectiveCamera::new(
            40.0,
            1.0,
            0.1,
            100.0,
            [pos[0], pos[1], pos[2] + 4.0],
            pos,
            [0.0, 1.0, 0.0],
        );
        cam.look_at(pos, None);
        let img = Renderer::render_frame(sc, cam, 120, 120, 1);
        ensure_artifacts();
        save_frame_png(&format!("{}/{}.png", ARTIFACTS, name), &img);
        let data = data_f32(&img);
        let idx = 60 * 120 * 4 + 60 * 4;
        [data[idx], data[idx + 1], data[idx + 2]]
    }

    #[test]
    fn test_render_sphere_hits() {
        let c = render_center(
            Geometry::SphereGeometry(SphereGeometry::new(1.0)),
            [0.0, 0.0, 0.0],
            "sphere",
        );
        assert!(c[0] > c[2]);
        assert!(c[0] < 250.0);
    }

    #[test]
    fn test_render_cone_hits() {
        let c = render_center(
            Geometry::ConeGeometry(ConeGeometry::new(0.8, 2.0)),
            [0.0, 0.0, 0.0],
            "cone",
        );
        assert!(c[0] > c[2]);
    }

    #[test]
    fn test_render_ellipsoid_hits() {
        let c = render_center(
            Geometry::EllipsoidGeometry(EllipsoidGeometry::new(1.0, 0.6, 0.8)),
            [0.0, 0.0, 0.0],
            "ellipsoid",
        );
        assert!(c[0] > c[2]);
    }

    #[test]
    fn test_render_cyclide_nonempty() {
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::CyclideGeometry(CyclideGeometry::new(
                1.0,
                0.98,
                0.3,
                [0.0, 0.0, 0.0],
            )),
            material: std_red_material(),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.8,
            [0.5, 1.0, 0.5],
        ));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.3));
        let mut cam = PerspectiveCamera::new(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let img = Renderer::render_frame(sc, cam, 120, 120, 1);
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
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::TorusGeometry(TorusGeometry::new(1.0, 0.3)),
            material: std_red_material(),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.8,
            [0.5, 1.0, 0.5],
        ));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.3));
        let mut cam = PerspectiveCamera::new(
            40.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 4.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let img = Renderer::render_frame(sc, cam, 120, 120, 1);
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
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 0.0, -1.0], -4.0)),
            material: Material::basic(Color::from_hex(0xCC3333), 1.0),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc
    }

    fn rq_head_on_cam() -> PerspectiveCamera {
        let mut cam = PerspectiveCamera::new(
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
        let mut r = Renderer::new(64, 64, 1, 3);
        let img = r.render(rq_wall_scene(), rq_head_on_cam());
        let p = rq_px(&img, 64, 32, 32);
        assert!(p[0] as i32 == 204);
        assert!(p[1] as i32 == 51);
        assert!(p[2] as i32 == 51);
    }
    fn rq_slab(depth: f64, absorption: f64) -> [f32; 3] {
        let mut sc = rq_wall_scene();
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::BoxGeometry(BoxGeometry::new(3.0, 3.0, depth)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0xFFFFFF),
                roughness: 0.5,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 0.0,
                ior: 1.0,
                absorption,
            }),
            position: [0.0, 0.0, 2.5],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut r = Renderer::new(64, 64, 1, 3);
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
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0xFFFFFF),
                roughness: 1.0,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 1.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.8,
            [0.0, 1.0, 0.0],
        ));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.2));
        if let Some(o) = opacity {
            sc.add_object(Object::new(ObjectParams {
                geometry: Geometry::SphereGeometry(SphereGeometry::new(0.5)),
                material: Material::standard(MaterialParams {
                    color: Color::from_hex(0xFFFFFF),
                    roughness: 1.0,
                    metalness: 0.0,
                    emissive: Color::from_hex(0x000000),
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
        let mut cam = PerspectiveCamera::new(
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
    fn test_render_glass_recursion_partial_subset() {
        // 玻璃只覆盖画面中心的一小块：折射递归走“need 子集”路径（此前递归在全量
        // 光线包上跑，subset 分支漏 gather 会在这里炸/串位）。中心应见墙，边缘仍是墙。
        let mut sc = rq_wall_scene();
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.3)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0xAAD4FF),
                roughness: 0.05,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 0.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [0.0, 0.0, 2.2],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut r = Renderer::new(96, 96, 1, 3);
        let img = r.render(sc, rq_head_on_cam());
        let center = rq_px(&img, 96, 48, 48);
        let corner = rq_px(&img, 96, 2, 2);
        // 边缘（玻璃外）就是墙色 0xCC3333 → (204,51,51)
        for c in 0..3 {
            let want = [204.0, 51.0, 51.0][c];
            assert!(
                (f64::from(corner[c]) - want).abs() <= 3.0,
                "边缘应见墙色：corner={corner:?}"
            );
        }
        // 中心透过玻璃：正入射透射所见为墙色，叠加 ~4% Fresnel 反射的浅蓝
        assert!(
            center[0] > 150.0
                && f64::from(center[0]) - f64::from(center[1]) > 80.0
                && f64::from(center[0]) - f64::from(center[2]) > 60.0,
            "中心应透过玻璃见到红墙：center={center:?}"
        );
    }

    #[test]
    fn test_render_ignore_opacity_is_noop_without_transparency() {
        // 不透明场景：忽略透明度模式必须与正常模式**逐位一致**（模式不得有副作用）
        let sc = rq_shadow_scene(Some(1.0));
        let mut a = Renderer::new(96, 96, 1, 3);
        let img_a = a.render(sc.clone(), rq_shadow_cam());
        let mut b = Renderer::new(96, 96, 1, 3).with_mode(RenderMode::IgnoreOpacity);
        let img_b = b.render(sc, rq_shadow_cam());
        img_a.eval().unwrap();
        img_b.eval().unwrap();
        assert_eq!(data_f32(&img_a), data_f32(&img_b));
    }

    #[test]
    fn test_render_ignore_opacity_disables_refraction() {
        // 玻璃球在墙前：正常模式中心透过折射见红墙；忽略透明度模式中心是玻璃自身的
        // 着色（不折射），两者必须不同。
        let mut sc = rq_wall_scene();
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.8)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0xAAD4FF),
                roughness: 0.1,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 0.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [0.0, 0.0, 2.2],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.8,
            [0.2, 1.0, 0.3],
        ));
        let mut n = Renderer::new(64, 64, 1, 3);
        let px_n = rq_px(&n.render(sc.clone(), rq_head_on_cam()), 64, 32, 32);
        let mut o = Renderer::new(64, 64, 1, 3).with_mode(RenderMode::IgnoreOpacity);
        let px_o = rq_px(&o.render(sc, rq_head_on_cam()), 64, 32, 32);
        // 正常模式：折射所见为墙（红主导）
        assert!(px_n[0] > px_n[2] + 60.0, "正常模式应折射见红墙：{px_n:?}");
        // 忽略透明度：不再折射（看不到红墙），而是球自身的不透明着色（命中点近镜面
        // 反射，可能是灰白高光），两模式必须有明显差异。
        assert!(
            (f64::from(px_o[0]) - f64::from(px_n[0])).abs() > 30.0,
            "两模式应有明显差异：{px_n:?} vs {px_o:?}"
        );
        assert!(
            f64::from(px_o[0]) < f64::from(px_n[0]) - 30.0,
            "忽略透明度不应再看到红墙：{px_n:?} vs {px_o:?}"
        );
    }

    #[test]
    fn test_render_ignore_opacity_transparent_occluder_fully_shadows() {
        // 完全不透明的遮挡物（opacity=0）：正常模式不投影（透过率 1，像素被照亮），
        // 忽略透明度模式按全不透明投影（本影只剩环境光）。
        let sc = rq_shadow_scene(Some(0.0));
        let want_lit = rq_linear_to_srgb255(0.8 * 0.7071 + 0.2); // 直射 + 环境（粗略上限）
        let mut n = Renderer::new(96, 96, 1, 3);
        let px_n = rq_px(&n.render(sc.clone(), rq_shadow_cam()), 96, 48, 48);
        let mut o = Renderer::new(96, 96, 1, 3).with_mode(RenderMode::IgnoreOpacity);
        let px_o = rq_px(&o.render(sc, rq_shadow_cam()), 96, 48, 48);
        let want_amb = rq_linear_to_srgb255(0.2);
        assert!(
            (f64::from(px_o[0]) - f64::from(want_amb)).abs() <= 2.0,
            "忽略透明度应投全阴影（只剩环境光）：{px_o:?} want≈{want_amb}"
        );
        assert!(
            f64::from(px_n[0]) > f64::from(want_amb) + 10.0,
            "正常模式透明遮挡物不应投全阴影：{px_n:?}（上限参考 {want_lit}）"
        );
    }

    #[test]
    fn test_render_shadow_umbra_is_ambient_only() {
        let mut r = Renderer::new(96, 96, 1, 3);
        let img = r.render(rq_shadow_scene(Some(1.0)), rq_shadow_cam());
        let p = rq_px(&img, 96, 48, 48);
        let want = rq_linear_to_srgb255(0.2);
        assert!((f64::from(p[0]) - f64::from(want)).abs() <= 2.0);
    }
    #[test]
    fn test_render_object_straddling_frame_edge() {
        // 剔除保守性：球心在画面外、只有一部分进入视野时，可见部分必须照常着色。
        // （逐对象屏幕区间子集不得漏掉任何落在对象包围盒内的光线。）
        let mut sc = Scene::new(None);
        let cam0 = rq_head_on_cam(); // 原点朝 +z，fov 40
                                     // 球心放到画面右侧之外，但球体仍覆盖画面右缘
        let off = 2.0;
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(1.2)),
            material: std_red_material(),
            position: [off, 0.0, 3.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 1.0));
        let mut r = Renderer::new(64, 64, 1, 3);
        let img = r.render(sc, cam0);
        let d = data_f32(&img);
        // 相机基座的 right = -x，所以世界 +x 的球出现在画面左缘：左缘被覆盖，
        // 右缘仍是天空背景（0x87CEEB：蓝 > 红）。
        let li = (32 * 64 + 0) * 4;
        let ri = (32 * 64 + 63) * 4;
        let (lr, lg, lb) = (d[li], d[li + 1], d[li + 2]);
        let (rr, _rg, rb) = (d[ri], d[ri + 1], d[ri + 2]);
        assert!(
            lr > lg && lr > 40.0,
            "画面左缘应被球（红）覆盖，得到 {lr}/{lg}/{lb}"
        );
        assert!(rb > rr, "画面右缘应仍是天空背景，得到 {rr}/../{rb}");
    }
}
