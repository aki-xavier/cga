use crate::geom_kernels::geom_to_camera;
use crate::geometry_ops::{geom_bounds, geom_intersect, geom_shadow, geom_uv};
use crate::mesh_raster::rasterize_meshes;
use crate::mlxops::*;
use crate::scene::{Mesh, PerspectiveCamera, Scene};
use crate::scene_graph::{vec3_dot, vec3_unit};
use crate::shading::{shade_batched, Light, LightKind};
use crate::texture::WrapMode;
use cga_core::{vec3_cross, Geometry, GeometryParams};
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
    use crate::scene::{Mesh, MeshParams, PerspectiveCamera};
    use crate::scene_graph::{srgb_to_linear, Color};
    use crate::shading::{Light, Material, MaterialParams};
    use cga_core::{
        BoxGeometry, ConeGeometry, CyclideGeometry, EllipsoidGeometry, Geometry, PlaneGeometry,
        SphereGeometry, TorusGeometry, TrimeshGeometry,
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
        sc.add_mesh(Mesh::new(MeshParams {
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
        sc.add_mesh(Mesh::new(MeshParams {
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
        sc.add_mesh(Mesh::new(MeshParams {
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

    #[test]
    fn test_render_trimesh_nonempty() {
        let mut sc = Scene::new(None);
        let verts: [[f64; 3]; 4] = [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [-0.5, 0.866, 0.0],
            [0.0, 0.0, -1.0],
        ];
        let faces: [[i32; 3]; 4] = [[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]];
        sc.add_mesh(Mesh::new(MeshParams {
            geometry: Geometry::TrimeshGeometry(TrimeshGeometry::new(&verts, &faces)),
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
        let mut sc = Scene::new(None);
        sc.add_mesh(Mesh::new(MeshParams {
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

    #[test]
    fn test_render_ior1_invisible() {
        let mut sc = rq_wall_scene();

        sc.add_mesh(Mesh::new(MeshParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.8)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0xAAD4FF),
                roughness: 0.5,
                metalness: 0.0,
                emissive: Color::from_hex(0x000000),
                opacity: 0.0,
                ior: 1.0,
                absorption: 0.0,
            }),
            position: [0.0, 0.0, 2.2],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut r = Renderer::new(64, 64, 1, 3);
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
        sc.add_mesh(Mesh::new(MeshParams {
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
        sc.add_mesh(Mesh::new(MeshParams {
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
            sc.add_mesh(Mesh::new(MeshParams {
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
    fn test_render_shadow_umbra_is_ambient_only() {
        let mut r = Renderer::new(96, 96, 1, 3);
        let img = r.render(rq_shadow_scene(Some(1.0)), rq_shadow_cam());
        let p = rq_px(&img, 96, 48, 48);
        let want = rq_linear_to_srgb255(0.2);
        assert!((f64::from(p[0]) - f64::from(want)).abs() <= 2.0);
    }

    #[test]
    fn test_render_trimesh_casts_shadow() {
        // D2：网格遮挡物在光线追踪的平面上投出本影。
        let mut sc = rq_shadow_scene(None);
        let verts: [[f64; 3]; 4] = [
            [1.5, 1.5, 2.5],
            [2.5, 1.5, 2.5],
            [2.5, 1.5, 3.5],
            [1.5, 1.5, 3.5],
        ];
        let faces: [[i32; 3]; 2] = [[0, 1, 2], [0, 2, 3]];
        sc.add_mesh(Mesh::new(MeshParams {
            geometry: Geometry::TrimeshGeometry(TrimeshGeometry::new(&verts, &faces)),
            material: std_red_material(),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        let mut r = Renderer::new(96, 96, 1, 3);
        let img = r.render(sc, rq_shadow_cam());
        let p = rq_px(&img, 96, 48, 48);
        let want = rq_linear_to_srgb255(0.2);
        assert!((f64::from(p[0]) - f64::from(want)).abs() <= 2.0);
    }

    #[test]
    fn test_render_trimesh_visible_through_glass() {
        // D3：折射光线（depth>0）里网格参与求交——透过玻璃球可见网格墙。
        let mut sc = Scene::new(None); // 天蓝背景：无网格时中心 = 蓝
        let verts: [[f64; 3]; 4] = [
            [-4.0, -4.0, 4.0],
            [4.0, -4.0, 4.0],
            [4.0, 4.0, 4.0],
            [-4.0, 4.0, 4.0],
        ];
        let faces: [[i32; 3]; 2] = [[0, 2, 1], [0, 3, 2]]; // 法线朝 -z（向相机）
        sc.add_mesh(Mesh::new(MeshParams {
            geometry: Geometry::TrimeshGeometry(TrimeshGeometry::new(&verts, &faces)),
            material: Material::basic(Color::from_hex(0xCC3333), 1.0),
            position: [0.0, 0.0, 0.0],
            rotation_axis: [0.0, 0.0, 1.0],
            rotation_angle: 0.0,
            motor: None,
        }));
        sc.add_mesh(Mesh::new(MeshParams {
            geometry: Geometry::SphereGeometry(SphereGeometry::new(0.8)),
            material: Material::standard(MaterialParams {
                color: Color::from_hex(0xAAD4FF),
                roughness: 0.5,
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
        let mut r = Renderer::new(64, 64, 1, 3);
        let img = r.render(sc, rq_head_on_cam());
        let p = rq_px(&img, 64, 32, 32);
        assert!(
            p[0] > p[2] + 60.0,
            "中心像素应为折射所见红墙，得到 {p:?}（无 D3 时是天蓝背景）"
        );
    }
}
