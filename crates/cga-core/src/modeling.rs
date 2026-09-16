// Modeling builders: ear-clipping triangulation + extrude / loft.
// Pure data transforms producing watertight triangle meshes (vertices, faces).

// --- ear clipping -----------------------------------------------------------

// signed_area returns the signed area of a 2D profile (CCW positive).
pub fn signed_area(profile: &[[f64; 2]]) -> f64 {
    let n = profile.len();
    let mut acc = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        acc += profile[i][0] * profile[j][1] - profile[j][0] * profile[i][1];
    }
    0.5 * acc
}

fn cross2(o: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

// segs_intersect reports whether the open segments p1-p2 and p3-p4 cross.
fn segs_intersect(p1: [f64; 2], p2: [f64; 2], p3: [f64; 2], p4: [f64; 2]) -> bool {
    let d1 = cross2(p3, p4, p1);
    let d2 = cross2(p3, p4, p2);
    let d3 = cross2(p1, p2, p3);
    let d4 = cross2(p1, p2, p4);
    ((d1 > 1e-12 && d2 < -1e-12) || (d1 < -1e-12 && d2 > 1e-12))
        && ((d3 > 1e-12 && d4 < -1e-12) || (d3 < -1e-12 && d4 > 1e-12))
}

// validate_profile reports the conditions under which triangulate/extrude
// would panic (too few points, duplicate consecutive points, zero area,
// self-intersection) — for callers without panic recovery (the CGS server).
pub fn validate_profile(profile: &[[f64; 2]]) -> Result<(), String> {
    let n = profile.len();
    if n < 3 {
        return Err(format!("profile needs >= 3 points, got {n}"));
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let dx = profile[j][0] - profile[i][0];
        let dy = profile[j][1] - profile[i][1];
        if dx * dx + dy * dy < 1e-24 {
            return Err(format!(
                "profile has duplicate consecutive points ({i}/{j})"
            ));
        }
    }
    if signed_area(profile).abs() < 1e-12 {
        return Err("profile is degenerate (zero area / collinear)".to_string());
    }
    // non-adjacent edge crossings (ear clipping cannot handle those)
    for i in 0..n {
        for j in i + 1..n {
            // skip adjacent edges (sharing a vertex)
            if j == i || j == (i + 1) % n || i == (j + 1) % n {
                continue;
            }
            if segs_intersect(
                profile[i],
                profile[(i + 1) % n],
                profile[j],
                profile[(j + 1) % n],
            ) {
                return Err(format!("profile self-intersects (edges {i} and {j})"));
            }
        }
    }
    Ok(())
}

fn in_tri(p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
    cross2(a, b, p) >= -1e-12 && cross2(b, c, p) >= -1e-12 && cross2(c, a, p) >= -1e-12
}

// triangulate ear-clips a simple polygon (no self-intersection, no holes) into
// CCW triangle indices.
pub fn triangulate(profile: &[[f64; 2]]) -> Vec<[i32; 3]> {
    let n = profile.len();
    if n < 3 {
        panic!("profile needs >= 3 points, got {n}");
    }
    let pts = profile.to_vec();
    let mut idx: Vec<usize> = (0..n).collect();
    if signed_area(&pts) < 0.0 {
        // CW input: walk the vertices in reverse via idx.  NOTE: pts stays in
        // the original order — the ear loop below indexes pts THROUGH idx, so
        // permuting pts as well would cancel the reversal (that double
        // reversal made every CW profile fail; extrude/loft never hit it
        // because they pre-CCW their profiles).
        for (i, v) in idx.iter_mut().enumerate() {
            *v = n - 1 - i;
        }
    }
    if signed_area(&pts).abs() < 1e-12 {
        panic!("profile is degenerate (zero area / collinear)");
    }
    let mut tris: Vec<[i32; 3]> = Vec::new();
    while idx.len() > 3 {
        let m = idx.len();
        let mut clipped = false;
        for i in 0..m {
            let i0 = idx[(i + m - 1) % m];
            let i1 = idx[i];
            let i2 = idx[(i + 1) % m];
            let a = pts[i0];
            let b = pts[i1];
            let c = pts[i2];
            if cross2(a, b, c) <= 1e-12 {
                continue; // concave or collinear, not an ear
            }
            let mut inside = false;
            for &j in &idx {
                if j == i0 || j == i1 || j == i2 {
                    continue;
                }
                if in_tri(pts[j], a, b, c) {
                    inside = true;
                    break;
                }
            }
            if inside {
                continue;
            }
            tris.push([i0 as i32, i1 as i32, i2 as i32]);
            idx.remove(i);
            clipped = true;
            break;
        }
        if !clipped {
            panic!(
                "ear clipping failed: profile likely self-intersects ({} vertices left)",
                idx.len()
            );
        }
    }
    tris.push([idx[0] as i32, idx[1] as i32, idx[2] as i32]);
    tris
}

// --- extrude / loft ---------------------------------------------------------

fn ccw(profile: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let pts = profile.to_vec();
    if signed_area(&pts) < 0.0 {
        let mut rev = vec![[0.0; 2]; pts.len()];
        for i in 0..pts.len() {
            rev[i] = pts[pts.len() - 1 - i];
        }
        return rev;
    }
    pts
}

// extrude extrudes a 2D profile along +Z from 0 to height.
pub fn extrude(profile: &[[f64; 2]], height: f64) -> (Vec<[f64; 3]>, Vec<[i32; 3]>) {
    if height <= 0.0 {
        panic!("extrude height must be > 0, got {height}");
    }
    let pts = ccw(profile);
    let n = pts.len();
    let mut verts: Vec<[f64; 3]> = Vec::new();
    for p in &pts {
        verts.push([p[0], p[1], 0.0]);
    }
    for p in &pts {
        verts.push([p[0], p[1], height]);
    }
    let mut faces: Vec<[i32; 3]> = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        faces.push([i as i32, j as i32, (n + j) as i32]);
        faces.push([i as i32, (n + j) as i32, (n + i) as i32]);
    }
    for tri in triangulate(&pts) {
        faces.push([n as i32 + tri[0], n as i32 + tri[1], n as i32 + tri[2]]);
        faces.push([tri[2], tri[1], tri[0]]);
    }
    (verts, faces)
}

// loft lofts equal-vertex-count profiles at strictly increasing zs.
pub fn loft(profiles: &[Vec<[f64; 2]>], zs: &[f64]) -> (Vec<[f64; 3]>, Vec<[i32; 3]>) {
    if profiles.len() != zs.len() || profiles.len() < 2 {
        panic!(
            "loft needs >= 2 profiles with matching zs, got {} profiles / {} zs",
            profiles.len(),
            zs.len()
        );
    }
    for i in 0..zs.len() - 1 {
        if zs[i] >= zs[i + 1] {
            panic!("loft zs must be strictly increasing, got {zs:?}");
        }
    }
    let m = profiles[0].len();
    if m < 3 {
        panic!("loft profiles must have >= 3 vertices");
    }
    for p in profiles {
        if p.len() != m {
            panic!("loft profiles must share the same vertex count (>= 3)");
        }
    }
    let mut verts: Vec<[f64; 3]> = Vec::new();
    for (k, prof) in profiles.iter().enumerate() {
        for p in prof {
            verts.push([p[0], p[1], zs[k]]);
        }
    }
    let mut faces: Vec<[i32; 3]> = Vec::new();
    for k in 0..profiles.len() - 1 {
        let base = k * m;
        for i in 0..m {
            let j = (i + 1) % m;
            faces.push([(base + i) as i32, (base + j) as i32, (base + m + j) as i32]);
            faces.push([
                (base + i) as i32,
                (base + m + j) as i32,
                (base + m + i) as i32,
            ]);
        }
    }
    let top_off = ((profiles.len() - 1) * m) as i32;
    for tri in triangulate(&ccw(&profiles[profiles.len() - 1])) {
        faces.push([top_off + tri[0], top_off + tri[1], top_off + tri[2]]);
    }
    for tri in triangulate(&ccw(&profiles[0])) {
        faces.push([tri[2], tri[1], tri[0]]);
    }
    (verts, faces)
}

#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn test_signed_area() {
        let sq = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        assert!(signed_area(&sq) > 0.0);
        // reversed is negative
        let mut rev: Vec<[f64; 2]> = Vec::new();
        for i in 0..sq.len() {
            rev.push(sq[sq.len() - 1 - i]);
        }
        assert!(signed_area(&rev) < 0.0);
    }

    #[test]
    fn test_triangulate_square() {
        let sq = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let tris = triangulate(&sq);
        assert!(tris.len() == 2);
        for t in tris {
            assert!(t[0] != t[1] && t[1] != t[2] && t[2] != t[0]);
        }
    }

    #[test]
    fn test_triangulate_concave() {
        // L-shaped (concave) polygon, 6 vertices -> 4 triangles
        let l = vec![
            [0.0, 0.0],
            [2.0, 0.0],
            [2.0, 1.0],
            [1.0, 1.0],
            [1.0, 2.0],
            [0.0, 2.0],
        ];
        let tris = triangulate(&l);
        assert!(tris.len() == 4);
    }

    #[test]
    fn test_extrude_watertight() {
        let sq = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let (verts, faces) = extrude(&sq, 2.0);
        assert!(verts.len() == 8);
        // 4 side quads (8 tris) + 2 caps (4 tris) = 12
        assert!(faces.len() == 12);
        for f in faces {
            for i in f {
                assert!(i >= 0 && (i as usize) < verts.len());
            }
        }
    }

    #[test]
    fn test_loft_watertight() {
        let sq1 = vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
        let sq2 = vec![[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]];
        let (verts, faces) = loft(&[sq1, sq2], &[0.0, 3.0]);
        assert!(verts.len() == 8);
        assert!(faces.len() == 12);
        for f in faces {
            for i in f {
                assert!(i >= 0 && (i as usize) < verts.len());
            }
        }
    }

    #[test]
    fn test_transform_point() {
        let m = from_trs([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0], [1.0, 1.0, 1.0]);
        let p = transform_point(m, [0.0, 0.0, 0.0]);
        assert!((p[0] - 1.0).abs() < 1e-12);
        assert!((p[1] - 2.0).abs() < 1e-12);
        assert!((p[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_obj_roundtrip() {
        let verts = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let faces = vec![[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]];
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("cga_obj_roundtrip.obj");
        let path = path.to_str().unwrap();
        save_obj(
            path,
            &[ObjMesh {
                vertices: verts.clone(),
                faces: faces.clone(),
                ..Default::default()
            }],
        );
        let (v2, f2) = load_obj(path).unwrap();
        assert!(v2.len() == verts.len());
        assert!(f2.len() == faces.len());
        for i in 0..verts.len() {
            assert!((v2[i][0] - verts[i][0]).abs() < 1e-9);
            assert!((v2[i][1] - verts[i][1]).abs() < 1e-9);
            assert!((v2[i][2] - verts[i][2]).abs() < 1e-9);
        }
        for i in 0..faces.len() {
            assert!(f2[i] == faces[i]);
        }
    }
}
