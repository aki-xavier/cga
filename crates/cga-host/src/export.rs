//! 导出方向：JSX 场景 → 三角网 → STL（二进制 / ASCII）。
//!
//! 本仓库**不提供导入**：网格不作为场景表示，只在导出时由解析图元与 CSG 三角化
//! 产生（`GeometryParams::bake`，marching tetrahedra，认证水密）。
//! URDF 导出见 `urdf::jsx_to_urdf`（非图元链接几何走同一条三角化路径）。

use crate::run_jsx;
use cga_core::{stl_ascii, stl_binary, transform_point, BakedMesh};
use cga_gpu::scene::Scene;

/// 场景的实体几何 → 世界空间三角网（各对象的 motor 应用到顶点）。
/// 返回 `(三角网, 被跳过的对象说明)`（无界几何如平面/无限长圆柱、圆不是实体）。
pub fn scene_to_mesh(scene: &Scene, step: f64) -> Result<(BakedMesh, Vec<String>), String> {
    if !(step > 0.0) || !step.is_finite() {
        return Err(format!("export: bad step {step}"));
    }
    let mut verts: Vec<[f64; 3]> = Vec::new();
    let mut faces: Vec<[i32; 3]> = Vec::new();
    let mut skipped: Vec<String> = Vec::new();
    for (oi, obj) in scene.objects.iter().enumerate() {
        let params = obj.geometry.identity_params();
        // 无界几何（平面/半空间，含其仿射包装与含平面的 CSG）与圆不是实体：
        // 按烘焙的错误族跳过并计数；其余烘焙失败（网格超限、字段不可判定）照常报错。
        let baked = match params.bake(step) {
            Ok(b) => b,
            Err(e) if e.contains("unbounded geometry") || e.contains("not a solid") => {
                skipped.push(format!("object {oi}: {}", skip_reason(&e)));
                continue;
            }
            Err(e) => return Err(format!("export: bake failed: {e}")),
        };
        let m = obj.motor().to_matrix();
        let base = verts.len() as i32;
        for v in &baked.vertices {
            verts.push(transform_point(m, *v));
        }
        for f in &baked.faces {
            faces.push([f[0] + base, f[1] + base, f[2] + base]);
        }
    }
    if faces.is_empty() {
        return Err("export: scene has no solid geometry to tessellate".into());
    }
    Ok((
        BakedMesh {
            vertices: verts,
            faces,
        },
        skipped,
    ))
}

/// 跳过的原因（简短标签）。
fn skip_reason(e: &str) -> &'static str {
    if e.contains("circle") {
        "circle (not a solid)"
    } else {
        "unbounded (plane / infinite cylinder)"
    }
}

/// JSX 场景 → 二进制 STL 字节（+ 跳过的对象说明）。
pub fn scene_to_stl(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
    step: f64,
) -> Result<(Vec<u8>, Vec<String>), String> {
    let run = run_jsx(jsx_src, css_src, asset_root)?;
    let (mesh, skipped) = scene_to_mesh(&run.scene, step)?;
    Ok((stl_binary(&mesh.vertices, &mesh.faces), skipped))
}

/// JSX 场景 → ASCII STL 文本（+ 跳过的非实体个数）。
pub fn scene_to_stl_ascii(
    jsx_src: &str,
    css_src: Option<&str>,
    asset_root: &str,
    step: f64,
    name: &str,
) -> Result<(String, Vec<String>), String> {
    let run = run_jsx(jsx_src, css_src, asset_root)?;
    let (mesh, skipped) = scene_to_mesh(&run.scene, step)?;
    Ok((stl_ascii(name, &mesh.vertices, &mesh.faces), skipped))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn facets(bytes: &[u8]) -> usize {
        u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize
    }

    #[test]
    fn scene_exports_binary_stl() {
        let jsx = "export default <scene>\
            <difference><box s={[1,1,1]} /><sphere r={0.4} /></difference>\
            </scene>;";
        let (bytes, skipped) = scene_to_stl(jsx, None, "", 0.05).unwrap();
        assert!(skipped.is_empty(), "{skipped:?}");
        let n = facets(&bytes);
        assert!(n > 100, "应有可观面数：{n}");
        assert_eq!(bytes.len(), 84 + n * 50);
    }

    #[test]
    fn scene_stl_skips_non_solids() {
        // 地面平面（半空间）不是实体：跳过并计数，不报错
        let jsx = "export default <scene>\
            <plane n={[0,1,0]} d={0} /><sphere r={0.5} />\
            </scene>;";
        let (bytes, skipped) = scene_to_stl(jsx, None, "", 0.08).unwrap();
        assert_eq!(skipped.len(), 1, "{skipped:?}");
        assert!(skipped[0].contains("unbounded"), "{skipped:?}");
        assert!(facets(&bytes) > 50);
    }

    #[test]
    fn scene_stl_without_solids_errors() {
        let jsx = "export default <scene><plane n={[0,1,0]} d={0} /></scene>;";
        assert!(scene_to_stl(jsx, None, "", 0.08)
            .unwrap_err()
            .contains("no solid geometry"));
    }

    #[test]
    fn ascii_stl_round_trips_shape() {
        let jsx = "export default <sphere r={1} />;";
        let (text, _) = scene_to_stl_ascii(jsx, None, "", 0.1, "ball").unwrap();
        assert!(text.starts_with("solid ball\n"));
        assert!(text.ends_with("endsolid ball\n"));
        assert!(text.matches("facet normal").count() > 100);
    }

    #[test]
    fn moved_object_lands_in_world_space() {
        // 平移后的球：导出顶点应落在 [9,11] 区间（而不是原点附近）
        let jsx = "export default <translate t={[10,0,0]}><sphere r={1} /></translate>;";
        let run = crate::run_jsx(jsx, None, "").unwrap();
        let (mesh, _) = scene_to_mesh(&run.scene, 0.25).unwrap();
        let xs: Vec<f64> = mesh.vertices.iter().map(|v| v[0]).collect();
        let lo = xs.iter().cloned().fold(f64::INFINITY, f64::min);
        let hi = xs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            lo > 8.5 && hi < 11.5,
            "世界坐标应在 [9,11] 附近：{lo}..{hi}"
        );
    }
}
