use cga_core::BezierParams;
use mlx_rs::Array;

use crate::mlxops::*;

pub fn bezier_intersect(p: &BezierParams, o: &Array, d: &Array) -> (Array, Array, Array) {
    crate::trimesh_intersect(&p.to_trimesh_params(), o, d)
}

pub fn bezier_shadow(p: &BezierParams, o: &Array, d: &Array) -> (Array, Array) {
    crate::trimesh_shadow(&p.to_trimesh_params(), o, d)
}

pub fn bezier_uv(_p: &BezierParams, pos: &Array, _n: &Array) -> Array {
    ck(mlx_rs::ops::zeros::<f32>(&[pos.shape()[0], 2]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use cga_core::{BezierPatchGeometry, BoxGeometry, CsgGeometry, CsgOp, Geometry, Multivector};

    fn flat16() -> Vec<[f64; 3]> {
        let mut pts = Vec::new();
        for i in 0..4 {
            for j in 0..4 {
                pts.push([
                    -1.0 + i as f64 * (2.0 / 3.0),
                    -1.0 + j as f64 * (2.0 / 3.0),
                    0.0,
                ]);
            }
        }
        pts
    }

    fn slab() -> Geometry {
        Geometry::BezierPatchGeometry(BezierPatchGeometry::new(&flat16(), 0.2, 4))
    }

    fn ray(x: f64, y: f64, z: f64) -> Array {
        Array::from_slice(&[x as f32, y as f32, z as f32], &[1, 3])
    }

    fn hit(g: &Geometry, o: [f64; 3], d: [f64; 3]) -> (f32, bool) {
        let p = crate::geom_to_camera(g, &Multivector::identity());
        let (t, _, mask) =
            crate::geom_intersect(&p, &ray(o[0], o[1], o[2]), &ray(d[0], d[1], d[2]));
        t.eval().unwrap();
        mask.eval().unwrap();
        (t.item_cast::<f32>(), mask.as_slice::<bool>()[0])
    }

    fn contains(g: &Geometry, pts: &[[f64; 3]]) -> Vec<bool> {
        let p = crate::geom_to_camera(g, &Multivector::identity());
        let mut flat: Vec<f32> = Vec::with_capacity(pts.len() * 3);
        for q in pts {
            flat.push(q[0] as f32);
            flat.push(q[1] as f32);
            flat.push(q[2] as f32);
        }
        let pos = Array::from_slice(&flat, &[pts.len() as i32, 3]);
        let got = crate::geom_contains(&p, &pos);
        got.eval().unwrap();
        got.as_slice::<bool>().to_vec()
    }

    #[test]
    fn test_bezier_surface_hit() {
        let g = Geometry::BezierPatchGeometry(BezierPatchGeometry::new(&flat16(), 0.0, 4));
        let (t, m) = hit(&g, [0.2, -0.3, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 5.0).abs() < 1e-3);
        let (_, m2) = hit(&g, [3.0, 0.0, 5.0], [0.0, 0.0, -1.0]);
        assert!(!m2);
    }

    #[test]
    fn test_bezier_slab_contains() {
        assert_eq!(
            contains(
                &slab(),
                &[[0.1, 0.13, 0.02], [0.1, 0.13, 0.5], [3.0, 0.0, 0.0]]
            ),
            [true, false, false]
        );
    }

    #[test]
    fn test_bezier_slab_csg_difference() {
        let g = Geometry::CsgGeometry(CsgGeometry::new(
            CsgOp::Difference,
            vec![
                Geometry::BoxGeometry(BoxGeometry::new(4.0, 4.0, 4.0)),
                slab(),
            ],
        ));
        assert_eq!(
            contains(&g, &[[0.1, 0.13, 0.02], [0.1, 0.13, 0.5], [1.5, 1.5, 1.5]]),
            [false, true, true]
        );
        let (t, m) = hit(&g, [0.1, 0.13, 5.0], [0.0, 0.0, -1.0]);
        assert!(m);
        assert!((f64::from(t) - 3.0).abs() < 1e-3);
    }

    #[test]
    fn test_bezier_slab_bake_volume() {
        let p = crate::geom_to_camera(&slab(), &Multivector::identity());
        let mesh = p.bake(0.05).unwrap();
        let v = mesh.volume();
        assert!((v - 0.8).abs() < 0.12, "slab volume {v} != ~0.8");
    }
}
