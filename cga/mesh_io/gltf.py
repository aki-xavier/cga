"""glTF 2.0 读取 + GLB 写出。

``load_gltf`` 支持:
  - .glb (JSON chunk + BIN chunk);
  - .gltf (JSON 文本, buffer 走相对路径 .bin 或 data:base64 URI);
  - nodes 的 TRS 或 matrix, 含层级继承的世界变换;
  - meshes.primitives 中 mode=4 (TRIANGLES) 或未指定 mode (默认 4) 的图元,
    取 POSITION accessor 与 indices accessor; 每个 primitive 返回一条
    ``(vertices, faces, world_transform)``;
  - accessor componentType 5126 (float) / 5125 (uint) / 5123 (ushort) /
    5121 (ubyte), 以及 accessor.byteOffset 与 bufferView.byteStride。

限制 (如实标注):
  - 不支持稀疏 accessor (sparse), 遇到即 ValueError;
  - 无 indices 的 primitive 按顺序索引 (0..n-1) 处理;
  - 忽略法线/UV/蒙皮/动画/相机等非几何数据, 不校验材质;
  - 仅读取默认 scene (gltf.scene 或 scene 0)。
"""

import base64
import json
import struct
from pathlib import Path
from typing import Any, cast

from cga.mesh_io._common import (
    Mat4,
    TransformedMesh,
    Tri,
    Vec3,
    from_column_major,
    from_trs,
    identity,
    mat_mul,
)
from cga.mesh_io._glb_write import save_glb

__all__ = ["load_gltf", "save_glb"]

_GLB_MAGIC = 0x46546C67
_CHUNK_JSON = 0x4E4F534A
_CHUNK_BIN = 0x004E4942

# componentType → (struct 格式字符, 单分量字节数)。
_COMPONENTS: dict[int, tuple[str, int]] = {
    5121: ("B", 1),
    5123: ("H", 2),
    5125: ("I", 4),
    5126: ("f", 4),
}
_TYPE_SIZES = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}

Json = dict[str, Any]


def _load_container(path: Path) -> tuple[Json, bytes | None]:
    """按扩展名拆分出 glTF JSON 文档与 (可选的) GLB BIN chunk。"""
    data = path.read_bytes()
    if path.suffix.lower() == ".glb":
        if len(data) < 20 or struct.unpack_from("<I", data, 0)[0] != _GLB_MAGIC:
            raise ValueError(f"{path}: 不是合法的 GLB 文件 (magic 不匹配)")
        version, total = struct.unpack_from("<II", data, 4)
        if version != 2:
            raise ValueError(f"{path}: 仅支持 glTF 2.0, 文件版本为 {version}")
        offset, gltf, bin_chunk = 12, None, None
        while offset + 8 <= min(total, len(data)):
            clen, ctype = struct.unpack_from("<II", data, offset)
            chunk = data[offset + 8 : offset + 8 + clen]
            if ctype == _CHUNK_JSON:
                gltf = cast(Json, json.loads(chunk.decode("utf-8")))
            elif ctype == _CHUNK_BIN:
                bin_chunk = bytes(chunk)
            offset += 8 + clen
        if gltf is None:
            raise ValueError(f"{path}: GLB 缺少 JSON chunk")
        return gltf, bin_chunk
    return cast(Json, json.loads(data.decode("utf-8"))), None


def _resolve_buffers(gltf: Json, path: Path, bin_chunk: bytes | None) -> list[bytes]:
    """按 buffers 数组解析全部 buffer 的字节内容。"""
    result: list[bytes] = []
    for i, buf in enumerate(cast(list[Json], gltf.get("buffers", []))):
        uri = cast(str | None, buf.get("uri"))
        if uri is None:
            if bin_chunk is None:
                raise ValueError(f"buffer {i}: 无 uri 且不是 GLB, 无法取数据")
            result.append(bin_chunk)
        elif uri.startswith("data:"):
            _, _, payload = uri.partition(",")
            result.append(base64.b64decode(payload))
        else:
            result.append((path.parent / uri).read_bytes())
    return result


def _read_accessor(
    gltf: Json, buffers: list[bytes], index: int
) -> list[tuple[float, ...]]:
    """读取 accessor 为分量元组列表; 处理 byteOffset/byteStride, 拒绝稀疏。"""
    acc = cast(Json, gltf["accessors"][index])
    if "sparse" in acc:
        raise ValueError(f"accessor {index}: 不支持稀疏 (sparse) accessor")
    ctype = cast(int, acc["componentType"])
    if ctype not in _COMPONENTS:
        raise ValueError(f"accessor {index}: 不支持的 componentType {ctype}")
    fmt, csize = _COMPONENTS[ctype]
    ncomp = _TYPE_SIZES[cast(str, acc.get("type", "SCALAR"))]
    count = cast(int, acc["count"])
    esize = csize * ncomp
    bv = cast(Json, gltf["bufferViews"][acc["bufferView"]])
    blob = buffers[cast(int, bv["buffer"])]
    base = cast(int, bv.get("byteOffset", 0)) + cast(int, acc.get("byteOffset", 0))
    stride = cast(int, bv.get("byteStride", esize))
    unpack = struct.Struct(f"<{ncomp}{fmt}").unpack_from
    return [
        tuple(float(v) for v in unpack(blob, base + i * stride)) for i in range(count)
    ]


def _node_local_matrix(node: Json) -> Mat4:
    """节点局部变换: matrix (列主序) 或 TRS, 缺省为恒等。"""
    if "matrix" in node:
        return from_column_major([float(v) for v in cast(list[Any], node["matrix"])])
    t = cast(Vec3, tuple(node.get("translation", (0.0, 0.0, 0.0))))
    r = cast(
        tuple[float, float, float, float], tuple(node.get("rotation", (0, 0, 0, 1)))
    )
    s = cast(Vec3, tuple(node.get("scale", (1.0, 1.0, 1.0))))
    return from_trs(t, r, s)


def _extract_primitive(
    gltf: Json, buffers: list[bytes], prim: Json
) -> tuple[list[Vec3], list[Tri]]:
    """提取单个 TRIANGLES primitive 的顶点与三角形面。"""
    attrs = cast(Json, prim.get("attributes", {}))
    if "POSITION" not in attrs:
        raise ValueError("primitive 缺少 POSITION 属性")
    positions = _read_accessor(gltf, buffers, cast(int, attrs["POSITION"]))
    vertices: list[Vec3] = []
    for p in positions:
        xyz = list(p) + [0.0] * max(0, 3 - len(p))
        vertices.append((xyz[0], xyz[1], xyz[2]))
    faces: list[Tri] = []
    if "indices" in prim:
        acc = _read_accessor(gltf, buffers, cast(int, prim["indices"]))
        raw = [int(t[0]) for t in acc]
    else:
        raw = list(range(len(vertices)))
    if len(raw) % 3 != 0:
        raise ValueError(f"索引数 {len(raw)} 不是 3 的倍数, 无法组成三角形")
    for i in range(0, len(raw), 3):
        face = (raw[i], raw[i + 1], raw[i + 2])
        if max(face) >= len(vertices) or min(face) < 0:
            raise ValueError(f"索引 {face} 越界 (顶点数 {len(vertices)})")
        faces.append(face)
    return vertices, faces


def load_gltf(path: str | Path) -> list[TransformedMesh]:
    """读取 .glb / .gltf, 返回 ``[(vertices, faces, world_transform), ...]``。

    每个 mode=4 (或缺省 mode) 的 primitive 一条; 顶点是 mesh 局部坐标,
    世界变换在返回元组第三项 (节点层级继承后的行主序矩阵)。
    """
    p = Path(path)
    gltf, bin_chunk = _load_container(p)
    buffers = _resolve_buffers(gltf, p, bin_chunk)
    nodes = cast(list[Json], gltf.get("nodes", []))
    meshes = cast(list[Json], gltf.get("meshes", []))
    scenes = cast(list[Json], gltf.get("scenes", []))
    if not scenes:
        return []
    scene = scenes[cast(int, gltf.get("scene", 0))]
    out: list[TransformedMesh] = []

    def visit(node_index: int, parent_world: Mat4) -> None:
        node = nodes[node_index]
        world = mat_mul(parent_world, _node_local_matrix(node))
        if "mesh" in node:
            prims = meshes[cast(int, node["mesh"])].get("primitives", [])
            for prim in cast(list[Json], prims):
                if cast(int, prim.get("mode", 4)) != 4:
                    continue
                vertices, faces = _extract_primitive(gltf, buffers, prim)
                out.append((vertices, faces, world))
        for child in cast(list[int], node.get("children", [])):
            visit(child, world)

    for root in cast(list[int], scene.get("nodes", [])):
        visit(root, identity())
    return out
