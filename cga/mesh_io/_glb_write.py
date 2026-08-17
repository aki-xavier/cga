"""GLB 二进制写出 (save_glb 的实现, 经 cga.mesh_io.gltf re-export)。

每张输入网格生成: 一个 mesh (单 TRIANGLES primitive) + 一个 node
(transform 非 None 时写 node.matrix, 列主序)。索引一律 uint32,
POSITION accessor 必写 min/max。JSON chunk 以空格填充到 4 字节对齐,
BIN chunk 以零字节填充。
"""

import json
import struct
from pathlib import Path
from typing import Any

from cga.mesh_io._common import Mat4, Tri, Vec3, to_column_major

__all__ = ["save_glb"]

GlbMeshIn = tuple[list[Vec3], list[Tri], Mat4 | None, Vec3 | None]
"""save_glb 输入条目: ``(vertices, faces, transform|None, color|None)``。"""

_GLB_MAGIC = 0x46546C67
_CHUNK_JSON = 0x4E4F534A
_CHUNK_BIN = 0x004E4942


def _pad4(blob: bytes, fill: bytes) -> bytes:
    """把字节串用指定字节填充到 4 的倍数长度。"""
    return blob + fill * ((-len(blob)) % 4)


def _push_buffer(blob: bytearray, payload: bytes) -> tuple[int, int]:
    """把 payload 追加进 buffer (先对齐到 4), 返回 (byteOffset, byteLength)。"""
    blob += b"\x00" * ((-len(blob)) % 4)
    offset = len(blob)
    blob += payload
    return offset, len(payload)


def _build_document(
    meshes: list[GlbMeshIn],
) -> tuple[dict[str, Any], bytes]:
    """构建 glTF JSON 文档与 BIN buffer 字节。"""
    gltf: dict[str, Any] = {
        "asset": {"version": "2.0", "generator": "cga.mesh_io"},
        "scene": 0,
        "scenes": [{"nodes": []}],
        "nodes": [],
        "meshes": [],
        "accessors": [],
        "bufferViews": [],
        "buffers": [],
    }
    blob = bytearray()
    materials: list[dict[str, Any]] = []
    material_ids: dict[tuple[float, float, float], int] = {}

    for index, (vertices, faces, transform, color) in enumerate(meshes):
        if not vertices or not faces:
            raise ValueError(f"mesh {index}: 顶点或面为空, 无法写出 GLB")
        pos_payload = b"".join(struct.pack("<3f", *p) for p in vertices)
        flat = [i for face in faces for i in face]
        idx_payload = struct.pack(f"<{len(flat)}I", *flat)
        pos_off, pos_len = _push_buffer(blob, pos_payload)
        idx_off, idx_len = _push_buffer(blob, idx_payload)

        views = gltf["bufferViews"]
        views.append({"buffer": 0, "byteOffset": pos_off, "byteLength": pos_len})
        views.append({"buffer": 0, "byteOffset": idx_off, "byteLength": idx_len})

        accessors = gltf["accessors"]
        accessors.append(
            {
                "bufferView": len(views) - 2,
                "componentType": 5126,
                "count": len(vertices),
                "type": "VEC3",
                "min": [min(p[i] for p in vertices) for i in range(3)],
                "max": [max(p[i] for p in vertices) for i in range(3)],
            }
        )
        accessors.append(
            {
                "bufferView": len(views) - 1,
                "componentType": 5125,
                "count": len(flat),
                "type": "SCALAR",
            }
        )

        primitive: dict[str, Any] = {
            "attributes": {"POSITION": len(accessors) - 2},
            "indices": len(accessors) - 1,
            "mode": 4,
        }
        if color is not None:
            key = (float(color[0]), float(color[1]), float(color[2]))
            if key not in material_ids:
                material_ids[key] = len(materials)
                materials.append(
                    {
                        "pbrMetallicRoughness": {
                            "baseColorFactor": [*key, 1.0],
                            "metallicFactor": 0.0,
                            "roughnessFactor": 1.0,
                        }
                    }
                )
            primitive["material"] = material_ids[key]

        gltf["meshes"].append({"primitives": [primitive]})
        node: dict[str, Any] = {"mesh": index, "name": f"mesh_{index}"}
        if transform is not None:
            node["matrix"] = to_column_major(transform)
        gltf["nodes"].append(node)
        gltf["scenes"][0]["nodes"].append(index)

    if materials:
        gltf["materials"] = materials
    gltf["buffers"] = [{"byteLength": len(blob)}]
    return gltf, bytes(blob)


def save_glb(path: str | Path, meshes: list[GlbMeshIn]) -> None:
    """把多张网格写出为 GLB 文件。

    ``meshes`` 每项为 ``(vertices, faces, transform|None, color|None)``;
    transform 写入 node.matrix (内部自动做行主序→列主序转置),
    color = (r, g, b) 0..1 写入 baseColorFactor (alpha=1, metallic=0,
    roughness=1), None 则不指定材质。
    """
    gltf, blob = _build_document(meshes)
    json_chunk = _pad4(json.dumps(gltf, separators=(",", ":")).encode("utf-8"), b" ")
    bin_chunk = _pad4(blob, b"\x00")
    total = 12 + 8 + len(json_chunk) + 8 + len(bin_chunk)
    out = bytearray(struct.pack("<III", _GLB_MAGIC, 2, total))
    out += struct.pack("<II", len(json_chunk), _CHUNK_JSON) + json_chunk
    out += struct.pack("<II", len(bin_chunk), _CHUNK_BIN) + bin_chunk
    Path(path).write_bytes(bytes(out))
