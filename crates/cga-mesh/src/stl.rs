//! STL 编码（导出方向；本仓库不解析 STL）。
//!
//! 只做字节编码，不做几何：三角网来自 `GeometryParams::bake`（marching
//! tetrahedra，认证水密）或场景级导出（cga-host 把各对象的 motor 应用到顶点）。

/// 二进制 STL（80 字节头 + 三角数 + 每面 50 字节）。单位法线按右手定则现算；
/// 退化面写零法线（读取方通常忽略 STL 法线）。
pub fn stl_binary(verts: &[[f64; 3]], faces: &[[i32; 3]]) -> Vec<u8> {
    let mut out: Vec<u8> = Vec::with_capacity(84 + faces.len() * 50);
    out.extend_from_slice(&[0u8; 80]);
    out.extend_from_slice(&(faces.len() as u32).to_le_bytes());
    for f in faces {
        let a = verts[f[0] as usize];
        let b = verts[f[1] as usize];
        let c = verts[f[2] as usize];
        let (u, v) = (sub(b, a), sub(c, a));
        let n = cross(u, v);
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let n = if len > 0.0 {
            [n[0] / len, n[1] / len, n[2] / len]
        } else {
            [0.0; 3]
        };
        for x in n {
            out.extend_from_slice(&(x as f32).to_le_bytes());
        }
        for p in [a, b, c] {
            for x in p {
                out.extend_from_slice(&(x as f32).to_le_bytes());
            }
        }
        out.extend_from_slice(&0u16.to_le_bytes());
    }
    out
}

/// ASCII STL（可读、可 diff；文件更大）。
pub fn stl_ascii(name: &str, verts: &[[f64; 3]], faces: &[[i32; 3]]) -> String {
    let mut out = String::with_capacity(faces.len() * 180);
    out.push_str(&format!("solid {name}\n"));
    for f in faces {
        let a = verts[f[0] as usize];
        let b = verts[f[1] as usize];
        let c = verts[f[2] as usize];
        let n = cross(sub(b, a), sub(c, a));
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let n = if len > 0.0 {
            [n[0] / len, n[1] / len, n[2] / len]
        } else {
            [0.0; 3]
        };
        out.push_str(&format!(
            "  facet normal {:e} {:e} {:e}\n    outer loop\n",
            n[0], n[1], n[2]
        ));
        for p in [a, b, c] {
            out.push_str(&format!("      vertex {:e} {:e} {:e}\n", p[0], p[1], p[2]));
        }
        out.push_str("    endloop\n  endfacet\n");
    }
    out.push_str(&format!("endsolid {name}\n"));
    out
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tetra() -> (Vec<[f64; 3]>, Vec<[i32; 3]>) {
        (
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        )
    }

    #[test]
    fn binary_layout() {
        let (v, f) = tetra();
        let bytes = stl_binary(&v, &f);
        assert_eq!(bytes.len(), 84 + f.len() * 50);
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]);
        assert_eq!(n as usize, f.len());
        // 首个三角面三个顶点的 z 分量位置：84 + 12(n) + 3*4*3(first vertex xyz) …
        let first_vertex_x = f32::from_le_bytes([bytes[96], bytes[97], bytes[98], bytes[99]]);
        assert!((first_vertex_x - v[f[0][0] as usize][0] as f32).abs() < 1e-6);
    }

    #[test]
    fn ascii_has_one_facet_per_face() {
        let (v, f) = tetra();
        let text = stl_ascii("t", &v, &f);
        assert_eq!(text.matches("facet normal").count(), f.len());
        assert_eq!(text.matches("vertex ").count(), f.len() * 3);
        assert!(text.starts_with("solid t\n") && text.ends_with("endsolid t\n"));
    }
}
