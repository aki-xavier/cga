"""纯 stdlib 网格交换 IO (OBJ / glTF 2.0)。

网格数据结构 = 纯 Python 元组:

  - vertices: ``list[tuple[float, float, float]]``
  - faces:    ``list[tuple[int, int, int]]`` (三角形索引, 逆时针为正面)
  - 变换:     4x4 行主序嵌套元组, ``None`` 表示恒等

公共 API:

  - load_obj(path)  -> ``(vertices, faces)``
  - save_obj(path, meshes) — meshes 为 ``(vertices, faces, transform|None)``,
    transform 非 None 时烘到世界系再写出
  - load_gltf(path) -> ``[(vertices, faces, world_transform), ...]``
    (.glb / .gltf, 每个 TRIANGLES primitive 一条)
  - save_glb(path, meshes) — meshes 为
    ``(vertices, faces, transform|None, color|None)``,
    transform 写 node.matrix, color 写 baseColorFactor

限制 (如实标注): 仅依赖标准库, 不 import mlx, 可独立导入;
OBJ 忽略 vt/vn/材质并合并所有组; glTF 不支持稀疏 accessor,
忽略法线/UV/蒙皮/动画, 仅读默认 scene。细节见各模块文档头。
"""

from cga.mesh_io.gltf import load_gltf, save_glb
from cga.mesh_io.obj import load_obj, save_obj

__all__ = [
    "load_gltf",
    "load_obj",
    "save_glb",
    "save_obj",
]
