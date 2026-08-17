"""耳切法 (ear clipping) 多边形三角化 —— 纯算法, 支持凹轮廓。

输入: [(x, y), ...] 简单多边形 (无自交无孔), 任意朝向 (内部统一 CCW)。
输出: [(i, j, k), ...] 顶点索引三元组 (相对输入点列, CCW 正面)。

限制 (如实标注): O(n²) 适合 ≤ ~10² 顶点; 带孔轮廓不支持 (用 CSG);
共线退化耳按 ε=1e-12 跳过, 全共线输入报 ValueError。
"""


def signed_area(profile: list[tuple[float, float]]) -> float:
    n = len(profile)
    return 0.5 * sum(
        profile[i][0] * profile[(i + 1) % n][1]
        - profile[(i + 1) % n][0] * profile[i][1]
        for i in range(n)
    )


def _cross2(o: tuple, a: tuple, b: tuple) -> float:
    """(a−o)×(b−o) 的 z 分量: >0 为左转。"""
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _in_tri(p: tuple, a: tuple, b: tuple, c: tuple) -> bool:
    """p 在 CCW 三角形 abc 内 (含边)。"""
    return (
        _cross2(a, b, p) >= -1e-12
        and _cross2(b, c, p) >= -1e-12
        and _cross2(c, a, p) >= -1e-12
    )


def triangulate(profile: list[tuple[float, float]]) -> list[tuple[int, int, int]]:
    """简单多边形 → 三角形索引列表 (CCW 正面)。"""
    n = len(profile)
    if n < 3:
        raise ValueError(f"profile needs >= 3 points, got {n}")
    pts = [tuple(map(float, p)) for p in profile]
    if signed_area(pts) < 0:  # 统一 CCW
        pts = pts[::-1]
        idx = list(range(n - 1, -1, -1))
    else:
        idx = list(range(n))
    if abs(signed_area(pts)) < 1e-12:
        raise ValueError("profile is degenerate (zero area / collinear)")

    tris = []
    guard = 0
    while len(idx) > 3:
        m = len(idx)
        clipped = False
        for i in range(m):
            i0, i1, i2 = idx[(i - 1) % m], idx[i], idx[(i + 1) % m]
            a, b, c = pts[i0], pts[i1], pts[i2]
            if _cross2(a, b, c) <= 1e-12:
                continue  # 凹点或共线, 非耳
            if any(_in_tri(pts[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(i)
            clipped = True
            break
        guard += 1
        if not clipped:
            raise ValueError(
                "ear clipping failed: profile likely self-intersects "
                f"({m} vertices left after {guard} clips)"
            )
    tris.append((idx[0], idx[1], idx[2]))
    return tris
