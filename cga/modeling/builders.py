"""挤出/放样构建器: 2D 轮廓 → 水密三角网格 (vertices, faces)。

绕序约定 (外法向): 轮廓 CCW → 顶盖 CCW (+Z 法向), 底盖反转,
侧壁 (i, i+1) 边按 (底_i, 底_{i+1}, 顶_{i+1}) / (底_i, 顶_{i+1}, 顶_i)
使叉积朝外。
"""

from cga.modeling.earclip import signed_area, triangulate


def _ccw(profile: list[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = [(float(p[0]), float(p[1])) for p in profile]
    return pts if signed_area(pts) >= 0 else pts[::-1]


def extrude(
    profile: list[tuple[float, float]], height: float
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    """轮廓沿 +Z 挤出 0..height (OpenSCAD linear_extrude 约定)。"""
    if height <= 0:
        raise ValueError(f"extrude height must be > 0, got {height}")
    pts = _ccw(profile)
    n = len(pts)
    verts = [(x, y, 0.0) for x, y in pts] + [(x, y, float(height)) for x, y in pts]
    faces: list[tuple[int, int, int]] = []
    for i in range(n):
        j = (i + 1) % n
        faces.append((i, j, n + j))  # 侧壁外法向
        faces.append((i, n + j, n + i))
    for a, b, c in triangulate(pts):
        faces.append((n + a, n + b, n + c))  # 顶盖 +Z
        faces.append((c, b, a))  # 底盖 −Z (反绕)
    return verts, faces


def loft(
    profiles: list[list[tuple[float, float]]], zs: list[float]
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    """等点数多截面放样: profiles[k] 在高度 zs[k], 相邻截面间侧壁。

    全部截面必须点数相同 (点序对应); zs 严格递增; ≥2 个截面。
    两端用耳切封盖 (底盖反绕)。
    """
    if len(profiles) != len(zs) or len(profiles) < 2:
        raise ValueError(
            f"loft needs >= 2 profiles with matching zs, got "
            f"{len(profiles)} profiles / {len(zs)} zs"
        )
    if any(zs[i] >= zs[i + 1] for i in range(len(zs) - 1)):
        raise ValueError(f"loft zs must be strictly increasing, got {zs}")
    m = len(profiles[0])
    if m < 3 or any(len(p) != m for p in profiles):
        raise ValueError("loft profiles must share the same vertex count (>= 3)")

    verts: list[tuple[float, float, float]] = []
    for prof, z in zip(profiles, zs):
        verts.extend((float(x), float(y), float(z)) for x, y in prof)
    faces: list[tuple[int, int, int]] = []
    for k in range(len(profiles) - 1):
        base = k * m
        for i in range(m):
            j = (i + 1) % m
            faces.append((base + i, base + j, base + m + j))
            faces.append((base + i, base + m + j, base + m + i))
    top_off = (len(profiles) - 1) * m
    for a, b, c in triangulate(_ccw(profiles[-1])):
        faces.append((top_off + a, top_off + b, top_off + c))
    for a, b, c in triangulate(_ccw(profiles[0])):
        faces.append((c, b, a))
    return verts, faces
