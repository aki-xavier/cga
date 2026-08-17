"""共享类型别名与 4x4 行主序矩阵纯函数 (内部模块, 非公共 API)。

项目约定: 变换是 4x4 行主序嵌套元组 ``Mat4``, 点按列向量右乘
``p' = M @ p``。glTF 的 node.matrix 是列主序, 读写边界处用
``to_column_major`` / ``from_column_major`` 转换。
"""

import math

Vec3 = tuple[float, float, float]
Tri = tuple[int, int, int]
Mat4 = tuple[tuple[float, ...], ...]
Mesh = tuple[list[Vec3], list[Tri]]
"""``(vertices, faces)``: 顶点坐标 + 三角形索引 (逆时针为正面)。"""

TransformedMesh = tuple[list[Vec3], list[Tri], Mat4]
"""``(vertices, faces, world_transform)``: load_gltf 的返回条目。"""


def identity() -> Mat4:
    """返回 4x4 单位矩阵。"""
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def mat_mul(a: Mat4, b: Mat4) -> Mat4:
    """行主序矩阵乘法 ``a @ b``。"""
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4))
        for i in range(4)
    )


def transform_point(m: Mat4, p: Vec3) -> Vec3:
    """``p' = M @ p`` (齐次坐标, w=1, 忽略投影 w 除法)。"""
    x, y, z = p
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
        m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
        m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3],
    )


def from_trs(
    translation: Vec3, rotation: tuple[float, float, float, float], scale: Vec3
) -> Mat4:
    """由 glTF TRS 构造行主序矩阵; rotation 为四元数 ``(x, y, z, w)``。"""
    tx, ty, tz = translation
    qx, qy, qz, qw = rotation
    sx, sy, sz = scale
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    # 单位四元数 → 旋转矩阵 (行主序)。
    rot = (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )
    return (
        (rot[0][0] * sx, rot[0][1] * sy, rot[0][2] * sz, tx),
        (rot[1][0] * sx, rot[1][1] * sy, rot[1][2] * sz, ty),
        (rot[2][0] * sx, rot[2][1] * sy, rot[2][2] * sz, tz),
        (0.0, 0.0, 0.0, 1.0),
    )


def to_column_major(m: Mat4) -> list[float]:
    """行主序 → glTF 列主序平铺 16 元素。"""
    return [m[i][j] for j in range(4) for i in range(4)]


def from_column_major(values: list[float]) -> Mat4:
    """glTF 列主序平铺 16 元素 → 行主序。"""
    if len(values) != 16:
        raise ValueError(f"node.matrix 需要 16 个元素, 得到 {len(values)} 个")
    return tuple(tuple(float(values[j * 4 + i]) for j in range(4)) for i in range(4))
