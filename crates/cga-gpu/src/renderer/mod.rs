use crate::geom_kernels::geom_to_camera;
use crate::geometry_ops::{geom_bounds, geom_intersect, geom_shadow, geom_uv};
use crate::mlxops::*;
use crate::scene::{Object, PerspectiveCamera, Scene};
use crate::scene_graph::{vec3_dot, vec3_unit};
use crate::shading::{shade_batched, Light, LightKind};
use crate::texture::WrapMode;
use cga_core::{vec3_cross, GeometryParams, Multivector};
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
        BoxGeometry, ConeGeometry, CyclideGeometry, CylinderGeometry, EllipsoidGeometry, Geometry,
        PlaneGeometry, SphereGeometry, TorusGeometry,
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
    // ---- 增量渲染（帧间）--------------------------------------------------

    fn incr_scene(offset: f64, glass: bool) -> Scene {
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0x7F8C8D),
                roughness: 0.9,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 1.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [0.0; 3],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.6)),
            material: std_red_material(),
            position: [0.0, 0.6, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.5)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0x2E86C1),
                roughness: 0.2,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: if glass { 0.35 } else { 1.0 },
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [offset, 0.5, 1.2],
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
        sc
    }

    fn incr_cam() -> PerspectiveCamera {
        let mut c = PerspectiveCamera::new(
            45.0,
            1.0,
            0.1,
            100.0,
            [0.0, 2.2, 5.0],
            [0.0, 0.5, 0.0],
            [0.0, 1.0, 0.0],
        );
        c.look_at([0.0, 0.5, 0.0], None);
        c
    }

    fn assert_same_image(a: &Array, b: &Array) {
        assert_eq!(data_f32(a), data_f32(b), "增量与全帧渲染必须逐位一致");
    }

    // ---- 拾取（交互闭环，docs/roadmap.md §3.A）--------------------------------

    #[test]
    fn test_pick_nearest_and_miss() {
        // 相机 (0,0,5) 看原点；球 r=0.6 在原点，另一个 r=0.5 在 (1.5, 0, -1)。
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.6)),
            material: std_red_material(),
            position: [0.0; 3],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.5)),
            material: std_red_material(),
            position: [1.5, 0.0, -1.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut cam = PerspectiveCamera::new(
            45.0,
            96.0 / 72.0,
            0.1,
            100.0,
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        // 正中：前球，命中点 ≈ (0,0,0.6)，法线朝相机（+z）
        let h = pick(&sc, &cam, 47.5, 35.5, 96, 72).expect("hit center");
        assert_eq!(h.object, 0);
        assert!((h.point[2] - 0.6).abs() < 1e-3, "{:?}", h.point);
        assert!(h.normal[2] > 0.99, "{:?}", h.normal);
        // 角落：未命中
        assert!(pick(&sc, &cam, 0.5, 0.5, 96, 72).is_none());
        // 右侧球的像素：相机空间 (1.5, 0, 6)（相机在原点上方 5 看 -z，世界 z=-1 → 深 6）
        let fx = 72.0 / (2.0 * (45.0f64.to_radians() / 2.0).tan()) * (96.0 / 72.0);
        let fy = 72.0 / (2.0 * (45.0f64.to_radians() / 2.0).tan());
        let px = 47.5 + 1.5 / 6.0 * fx;
        let py = 35.5; // y=0 → 中线
        let h2 = pick(&sc, &cam, px, py, 96, 72).expect("hit second");
        assert_eq!(h2.object, 1, "{h2:?}");
        let _ = fy;
    }

    #[test]
    fn test_pick_occlusion() {
        // 同一条光线上两个球：近者胜出。
        let mut sc = Scene::new(None);
        for z in [0.0, -2.0] {
            sc.add_object(Object::new(ObjectParams {
                geometry: Geometry::SphereGeometry(SphereGeometry::new(0.6)),
                material: std_red_material(),
                position: [0.0, 0.0, z],
                rotation_axis: [0.0, 0.0, 1.0],
                rotation_angle: 0.0,
                motor: None,
            }));
        }
        let mut cam = PerspectiveCamera::new(
            45.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let h = pick(&sc, &cam, 47.5, 35.5, 96, 72).expect("hit");
        assert_eq!(h.object, 0, "近者优先");
        assert!((h.t - 4.4).abs() < 0.1, "t={}（5 − 0.6）", h.t);
    }

    #[test]
    fn test_pick_plane() {
        // 地面平面：画面下半部命中平面。
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            material: std_red_material(),
            position: [0.0; 3],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut cam = PerspectiveCamera::new(
            45.0,
            96.0 / 72.0,
            0.1,
            100.0,
            [0.0, 2.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let h = pick(&sc, &cam, 47.5, 60.0, 96, 72).expect("hit ground");
        assert_eq!(h.object, 0);
        assert!(h.point[1].abs() < 1e-3, "在地面上: {:?}", h.point);
        assert!((h.normal[1] - 1.0).abs() < 1e-3, "法线 +y: {:?}", h.normal);
    }

    // ---- 诊断通道（U3，docs/ue58-inspirations.md §3.U3）----------------------

    fn diag_two_spheres() -> (Scene, PerspectiveCamera) {
        // 与 test_pick_nearest_and_miss 同场景：球0 r=0.6 在原点，球1 r=0.5 在 (1.5,0,−1)。
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.6)),
            material: std_red_material(),
            position: [0.0; 3],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.5)),
            material: std_red_material(),
            position: [1.5, 0.0, -1.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut cam = PerspectiveCamera::new(
            45.0,
            96.0 / 72.0,
            0.1,
            100.0,
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        (sc, cam)
    }

    fn diag_px(img: &Array, x: usize, y: usize, w: usize) -> [f32; 3] {
        // 输出约定 [h, w, 4] f32 0..255（同 render()）：读出并归一到 0..1。
        let s = img.as_slice::<f32>();
        let i = 4 * (y * w + x);
        [s[i] / 255.0, s[i + 1] / 255.0, s[i + 2] / 255.0]
    }

    #[test]
    fn test_diag_object_id_and_miss() {
        // 对象下标 → 调色板：中心 = 球0 的色，右侧 = 球1 的色，角落 = 黑（未命中）。
        let (sc, cam) = diag_two_spheres();
        let img = render_diagnostic(&sc, &cam, 96, 72, DiagChannel::ObjectId);
        img.eval().unwrap();
        let center = diag_px(&img, 47, 35, 96);
        let want0 = diag_palette(0);
        for i in 0..3 {
            assert!(
                (center[i] - want0[i]).abs() < 1e-6,
                "中心应为球0色: {center:?}"
            );
        }
        // 右侧球（与 pick 测试同一像素公式）
        let fx = 72.0 / (2.0 * (45.0f64.to_radians() / 2.0).tan()) * (96.0 / 72.0);
        let px2 = (47.5 + 1.5 / 6.0 * fx) as usize;
        let second = diag_px(&img, px2, 35, 96);
        let want1 = diag_palette(1);
        for i in 0..3 {
            assert!((second[i] - want1[i]).abs() < 1e-6, "应为球1色: {second:?}");
        }
        assert_ne!(want0, want1, "相邻下标颜色应不同");
        assert_eq!(diag_px(&img, 0, 0, 96), [0.0; 3], "角落未命中应为黑");
    }

    #[test]
    fn test_diag_normal_sphere_center_and_plane_constancy() {
        // 球心像素：法线≈正对相机（球心投影在 (47.5,35.5)，本像素有亚像素倾斜
        // ~0.02）→ 相机系 ≈(0,0,−1) → RGB ≈ (0.5, 0.5, 0)。
        let (sc, cam) = diag_two_spheres();
        let img = render_diagnostic(&sc, &cam, 96, 72, DiagChannel::Normal);
        img.eval().unwrap();
        let c = diag_px(&img, 47, 35, 96);
        assert!((c[0] - 0.5).abs() < 0.03, "{c:?}");
        assert!((c[1] - 0.5).abs() < 0.03, "{c:?}");
        assert!(c[2] < 0.005, "正对相机的法线 z 应翻转到 ≈−1 → ≈0: {c:?}");
        assert_eq!(diag_px(&img, 0, 0, 96), [0.0; 3], "未命中应为黑");

        // 平面：所有命中像素法线一致 → 颜色处处相等（常数断言）。
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            material: std_red_material(),
            position: [0.0; 3],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut cam = PerspectiveCamera::new(
            45.0,
            96.0 / 72.0,
            0.1,
            100.0,
            [0.0, 2.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let img = render_diagnostic(&sc, &cam, 96, 72, DiagChannel::Normal);
        img.eval().unwrap();
        let a = diag_px(&img, 47, 60, 96);
        let b = diag_px(&img, 20, 65, 96);
        assert!(a.iter().any(|v| *v > 0.0), "平面处应命中: {a:?}");
        for i in 0..3 {
            assert!(
                (a[i] - b[i]).abs() < 1e-6,
                "平面法线色应处处相等: {a:?} vs {b:?}"
            );
        }
    }

    #[test]
    fn test_diag_depth_matches_pick_t() {
        // 深度灰度 = 1/(1+t)，t 与同像素 pick 一致（交叉验证既有已验证 API）。
        let (sc, cam) = diag_two_spheres();
        let img = render_diagnostic(&sc, &cam, 96, 72, DiagChannel::Depth);
        img.eval().unwrap();
        let hit = pick(&sc, &cam, 47.5, 35.5, 96, 72).expect("hit");
        let g = diag_px(&img, 47, 35, 96)[0];
        let want = 1.0 / (1.0 + hit.t as f32);
        assert!((g - want).abs() < 1e-3, "g={g} vs 1/(1+t)={want}");
        // 未命中 = 黑；深度通道是灰度（三通道相等）
        assert_eq!(diag_px(&img, 0, 0, 96), [0.0; 3]);
        let px3 = diag_px(&img, 47, 35, 96);
        assert_eq!(px3[0], px3[1]);
        assert_eq!(px3[1], px3[2]);
    }

    #[test]
    fn test_diag_bitwise_deterministic() {
        // 同输入同输出：同一通道渲两次，PNG 逐位一致；三通道内容两两不同。
        let (sc, cam) = diag_two_spheres();
        for ch in [
            DiagChannel::ObjectId,
            DiagChannel::Normal,
            DiagChannel::Depth,
        ] {
            let a = crate::image_io::frame_to_png_bytes(&render_diagnostic(&sc, &cam, 96, 72, ch));
            let b = crate::image_io::frame_to_png_bytes(&render_diagnostic(&sc, &cam, 96, 72, ch));
            assert_eq!(a, b, "{ch:?} 应逐位确定");
        }
        let id = crate::image_io::frame_to_png_bytes(&render_diagnostic(
            &sc,
            &cam,
            96,
            72,
            DiagChannel::ObjectId,
        ));
        let nm = crate::image_io::frame_to_png_bytes(&render_diagnostic(
            &sc,
            &cam,
            96,
            72,
            DiagChannel::Normal,
        ));
        let dp = crate::image_io::frame_to_png_bytes(&render_diagnostic(
            &sc,
            &cam,
            96,
            72,
            DiagChannel::Depth,
        ));
        assert_ne!(id, nm);
        assert_ne!(nm, dp);
        assert_ne!(id, dp);
    }

    // ---- BVH 求交内核 ------------------------------------------------------

    /// 混合场景：40 个受支持对象（球/盒/柱/锥/椭球 8×5 网格）+ 地面平面
    /// + torus（旧路径）+ 平行光/点光/环境光。
    fn bvh_mixed_scene() -> Scene {
        let mut sc = Scene::new(None);
        for i in 0..8 {
            for j in 0..5 {
                let x = (i as f64 - 3.5) * 2.2;
                let z = -(j as f64) * 2.2;
                let y = 0.55;
                let rot = (x * 0.7 + z * 0.3).rem_euclid(1.5);
                let (geom, name): (Geometry, &str) = match (i + j) % 5 {
                    0 => (Geometry::SphereGeometry(SphereGeometry::new(0.55)), "s"),
                    1 => (Geometry::BoxGeometry(BoxGeometry::new(0.5, 0.5, 0.5)), "b"),
                    2 => (
                        Geometry::CylinderGeometry(CylinderGeometry::new(0.4, 1.1)),
                        "c",
                    ),
                    3 => (Geometry::ConeGeometry(ConeGeometry::new(0.5, 1.1)), "k"),
                    _ => (
                        Geometry::EllipsoidGeometry(EllipsoidGeometry::new(0.6, 0.45, 0.5)),
                        "e",
                    ),
                };
                let _ = name;
                sc.add_object(Object::new(ObjectParams {
                    geometry: geom,
                    material: std_red_material(),
                    position: [x, y, z],
                    rotation_axis: [0.0, 1.0, 0.0],
                    rotation_angle: rot,
                    motor: None,
                }));
            }
        }
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::TorusGeometry(TorusGeometry::new(0.8, 0.25)),
            material: std_red_material(),
            position: [0.0, 0.6, 2.0],
            rotation_axis: [1.0, 0.0, 0.0],
            rotation_angle: 0.6,
            motor: None,
        }));
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::PlaneGeometry(PlaneGeometry::new([0.0, 1.0, 0.0], 0.0)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0x7F8C8D),
                roughness: 0.9,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 1.0,
                ior: 1.5,
                absorption: 0.0,
            }),
            position: [0.0; 3],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_light(Light::directional(
            Color::from_hex(0xFFFFFF),
            0.7,
            [0.5, 1.0, 0.5],
        ));
        sc.add_light(Light::point(
            Color::from_hex(0xFFFFFF),
            0.4,
            [3.0, 5.0, 4.0],
        ));
        sc.add_light(Light::ambient(Color::from_hex(0xFFFFFF), 0.3));
        sc
    }

    fn bvh_cam() -> PerspectiveCamera {
        let mut c = PerspectiveCamera::new(
            45.0,
            96.0 / 72.0,
            0.1,
            100.0,
            [0.0, 6.0, 9.0],
            [0.0, 0.5, -3.0],
            [0.0, 1.0, 0.0],
        );
        c.look_at([0.0, 0.5, -3.0], None);
        c
    }

    #[test]
    fn test_bvh_kernel_engages_and_matches_legacy() {
        let sc = bvh_mixed_scene();
        let cam = bvh_cam();

        // 内核确实启用（force），且 40 个受支持对象入核、平面/torus 留旧路径
        let mut r = Renderer::new(96, 72, 1, 3);
        r.bvh_override = Some(true);
        let p = r.prep(&sc, &cam);
        let bvh = p.bvh.as_ref().expect("force 模式下必须建出 BVH");
        let n_kernel = bvh.legacy.iter().filter(|&&l| !l).count();
        assert_eq!(n_kernel, 40, "40 个受支持对象入核: {n_kernel}");
        assert_eq!(bvh.legacy.len(), 42);
        assert!(bvh.legacy[40] && bvh.legacy[41], "torus/平面留旧路径");

        // 整帧对比：内核路径 vs 旧路径（f32 算序差异 → 容差比较；
        // 旧路径本身也被画廊金标钉着，这里是"新引擎不劣于旧引擎"的断言）
        let img_bvh = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            data_f32(&r.render(sc.clone(), cam))
        }));
        if img_bvh.is_err() {
            println!(
                "LAST_MLX_ERROR: {}",
                cga_fastmetal::take_last_error_string()
            );
            panic!("bvh render failed");
        }
        let img_bvh = img_bvh.unwrap();
        let mut r2 = Renderer::new(96, 72, 1, 3);
        r2.bvh_override = Some(false);
        let img_legacy = data_f32(&r2.render(sc, cam));
        assert_eq!(img_bvh.len(), img_legacy.len());
        let mut n_diff = 0;
        let mut max_diff = 0.0f32;
        for (a, b) in img_bvh.iter().zip(img_legacy.iter()) {
            let d = (a - b).abs();
            max_diff = max_diff.max(d);
            if d > 1.0 {
                n_diff += 1;
            }
        }
        assert!(
            max_diff <= 8.0 && n_diff * 100 < img_bvh.len(),
            "BVH 内核与旧路径差异过大: max={max_diff} n_diff={n_diff}/{}",
            img_bvh.len()
        );
    }

    /// 阴影路径（点光源 + 平行光下的 vis 连乘）也由内核承担：上面整帧对比
    /// 已覆盖（两盏灯都开）；这里补一个解析断言：正对球的光线，阴影可见度
    /// 在球后为 0。
    #[test]
    fn test_bvh_shadow_occludes() {
        let mut sc = Scene::new(None);
        sc.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(1.0)),
            material: std_red_material(),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut cam = PerspectiveCamera::new(
            45.0,
            1.0,
            0.1,
            100.0,
            [0.0, 0.0, 5.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        );
        cam.look_at([0.0, 0.0, 0.0], None);
        let mut r = Renderer::new(8, 8, 1, 3);
        r.bvh_override = Some(true);
        let p = r.prep(&sc, &cam);
        let bvh = p.bvh.as_ref().expect("bvh");
        // 相机空间 +z 朝前：球心 (0,0,5) 半径 1。光线从 (−0.5,0,3) 向 +z：穿球。
        let o = Array::from_slice(&[-0.5f32, 0.0, 3.0], &[1, 3]);
        let d = Array::from_slice(&[0.0f32, 0.0, 1.0], &[1, 3]);
        let tmax = Array::from_slice(&[10.0f32], &[1]);
        let vis = bvh.trace_shadow(&o, &d, &tmax).expect("shadow kernel");
        vis.eval().unwrap();
        assert_eq!(vis.as_slice::<f32>(), &[0.0], "不透明球全遮挡");
        let (t, idx, _n) = bvh.trace_nearest(&o, &d).expect("nearest kernel");
        t.eval().unwrap();
        idx.eval().unwrap();
        let tv = t.as_slice::<f32>()[0];
        // 偏心 0.5 的弦：t = 2 − √(1−0.25) = 2 − √0.75 ≈ 1.13397
        assert!((tv - 1.1339746).abs() < 1e-4, "偏心弦入口 t: {tv}");
        assert_eq!(idx.as_slice::<i32>()[0], 0);
    }

    #[test]
    fn test_incremental_first_and_clean() {
        let mut r = IncrementalRenderer::new(96, 72, 1, 3);
        let sc = incr_scene(2.0, false);
        let cam = incr_cam();
        let (img1, s1) = r.render(&sc, &cam);
        assert!(s1.full && s1.reason == "first" && s1.dirty == s1.total);
        // 同一场景再来一帧：无变化 → 一条光线都不重追
        let (img2, s2) = r.render(&sc, &cam);
        assert!(!s2.full && s2.dirty == 0 && s2.reason == "clean");
        assert_same_image(&img1, &img2);
    }

    #[test]
    fn test_incremental_move_sphere_bitexact() {
        // 地面平面 + 平行光：运动球的新旧包围盒 + 它在地面上的阴影足迹必须置脏。
        let cam = incr_cam();
        let mut r = IncrementalRenderer::new(96, 72, 1, 3);
        r.render(&incr_scene(2.0, false), &cam);
        let sc2 = incr_scene(1.2, false);
        let (img, s) = r.render(&sc2, &cam);
        assert!(!s.full, "应增量渲染，得到 {s:?}");
        assert!(
            s.dirty * 2 < s.total,
            "脏光线应少于一半，得到 {}/{}",
            s.dirty,
            s.total
        );
        let want = Renderer::new(96, 72, 1, 3).render(sc2, cam);
        assert_same_image(&img, &want);
    }

    #[test]
    fn test_incremental_material_only_bitexact() {
        // 只改颜色（形状/透明度不变）：无阴影级联，只脏自身包围盒。
        let cam = incr_cam();
        let mut r = IncrementalRenderer::new(96, 72, 1, 3);
        r.render(&incr_scene(2.0, false), &cam);
        let mut sc2 = incr_scene(2.0, false);
        sc2.objects[1].material.color = Color::from_hex(0x27AE60);
        let (img, s) = r.render(&sc2, &cam);
        assert!(!s.full, "应增量渲染，得到 {s:?}");
        let want = Renderer::new(96, 72, 1, 3).render(sc2, cam);
        assert_same_image(&img, &want);
    }

    #[test]
    fn test_incremental_transparent_cascade_bitexact() {
        // 玻璃球 + 移动的不透明球：玻璃像素经透明级联置脏（次级光线可达）。
        let cam = incr_cam();
        let mut r = IncrementalRenderer::new(96, 72, 1, 3);
        r.render(&incr_scene(2.5, true), &cam);
        let sc2 = incr_scene(1.2, true);
        let (img, s) = r.render(&sc2, &cam);
        assert!(!s.full, "应增量渲染，得到 {s:?}");
        let want = Renderer::new(96, 72, 1, 3).render(sc2, cam);
        assert_same_image(&img, &want);
    }

    #[test]
    fn test_incremental_full_fallbacks() {
        let cam = incr_cam();
        // 相机变化 → 全帧
        let mut r = IncrementalRenderer::new(96, 72, 1, 3);
        let sc = incr_scene(2.0, false);
        r.render(&sc, &cam);
        let mut cam2 = incr_cam();
        cam2.position = [0.6, 2.4, 5.2];
        cam2.look_at([0.0, 0.5, 0.0], None);
        let (_, s) = r.render(&sc, &cam2);
        assert!(s.full && s.reason == "camera", "{s:?}");

        // 灯光变化 → 全帧
        let mut r = IncrementalRenderer::new(96, 72, 1, 3);
        r.render(&sc, &cam);
        let mut sc2 = incr_scene(2.0, false);
        sc2.lights[0].intensity = 0.55;
        let (_, s) = r.render(&sc2, &cam);
        assert!(s.full && s.reason == "lights", "{s:?}");

        // 对象数量变化 → 全帧
        let mut r = IncrementalRenderer::new(96, 72, 1, 3);
        r.render(&sc, &cam);
        let mut sc3 = incr_scene(2.0, false);
        sc3.add_object(Object::new(ObjectParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.3)),
            material: std_red_material(),
            position: [-1.5, 0.3, 0.5],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let (_, s) = r.render(&sc3, &cam);
        assert!(s.full && s.reason == "count", "{s:?}");

        // invalidate → 全帧
        let mut r = IncrementalRenderer::new(96, 72, 1, 3);
        r.render(&sc, &cam);
        r.invalidate();
        let (_, s) = r.render(&sc, &cam);
        assert!(s.full && s.reason == "first", "{s:?}");
    }
}
