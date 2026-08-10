"""CGA 模型 → WebGL 渲染数据导出。

导出两件事:
  1. **代数真值**: 32×32 基 blade 乘法表 + reverse 符号 —— 由本包 (Python)
     计算导出, JS 端用同一张表实现表驱动 gp。浏览器里跑的 CGA 与
     Python 端是同一个代数 (唯一的真值源是导出脚本)。
  2. **模型**: 机器人结构 (关节 origin = 32 分量 versor + 轴/限位, link
     几何参数 + 材质)。JS 端 FK 走 versor 乘积, 几何共轭走 sandwich
     (平面/球直接, 圆柱/盒/圆为解析刚体变换, 与 engine.to_camera 一致)。

用法: uv run python -m cga.webgl models/z1_arm.crdf.yaml -o webgl/z1.json
"""

from __future__ import annotations

import json
from pathlib import Path

import mlx.core as mx

from cga.multivector import Multivector
from cga.robot import Geometry, Robot, load_robot


def _basis_blade(i: int) -> Multivector:
    v = mx.zeros(32, dtype=mx.float32)
    v[i] = 1.0
    return Multivector(v)


def basis_product_table() -> list[list[list[float]] | None]:
    """32×32 基 blade 乘积表: table[i*32+j] = [[槽位, 符号], ...] 或 null。

    共形基下基 blade 乘积可产生多分量 (如 e1e∞·e0 → e1 + 三阶项),
    必须存全部分量, 不能只取单槽。"""
    table: list[list[list[float]] | None] = [None] * (32 * 32)
    for i in range(32):
        bi = _basis_blade(i)
        for j in range(32):
            r = bi.gp(_basis_blade(j))
            nz = [
                [k, 1.0 if float(r.values[k]) > 0 else -1.0]
                for k in range(32)
                if abs(float(r.values[k])) > 1e-6
            ]
            if nz:
                table[i * 32 + j] = nz
    return table


def reverse_signs() -> list[float]:
    """每槽位的 reverse 符号 (grade 保持, 同槽 ±1)。"""
    return [float(_basis_blade(i).reverse().values[i]) for i in range(32)]


def _versor(m) -> list[float]:
    return [float(x) for x in m.values]


def _geom_params(g: Geometry) -> dict:
    if g.blade == "cylinder":
        return {"radius": g.radius, "length": g.length}
    if g.blade == "box":
        return {"size": list(g.size) if g.size else None}
    if g.blade == "sphere":
        return {"radius": g.radius}
    if g.blade == "plane":
        return {"normal": list(g.normal) if g.normal else None, "distance": g.distance}
    if g.blade == "circle":
        return {"radius": g.radius}
    if g.blade == "mesh":
        return {"file": g.file, "scale": list(g.scale) if g.scale else None}
    raise ValueError(f"unknown blade {g.blade!r}")


def export_robot(robot: Robot) -> dict:
    """Robot → WebGL JSON (代数表 + 模型)。几何局部轴约定: 圆柱/圆 = +Z。"""
    links = []
    for lnk in robot.links:
        geoms = []
        for g in lnk.geometry:
            if "visual" not in g.role:
                continue
            geoms.append(
                {
                    "blade": g.blade,
                    "origin": _versor(g.origin),
                    "material": g.material,
                    "params": _geom_params(g),
                }
            )
        links.append({"name": lnk.name, "geometry": geoms})
    joints = []
    for j in robot.joints:
        joints.append(
            {
                "name": j.name,
                "type": j.type,
                "parent": j.parent,
                "child": j.child,
                "origin": _versor(j.origin),
                "axis": list(j.axis) if j.axis else None,
                "lower": j.lower,
                "upper": j.upper,
            }
        )
    return {
        "basis_table": basis_product_table(),
        "reverse_sign": reverse_signs(),
        "robot": {
            "name": robot.name,
            "base": robot.base,
            "materials": [
                {"name": m.name, "color": list(m.color)} for m in robot.materials
            ],
            "links": links,
            "joints": joints,
        },
    }


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="CRDF → WebGL JSON")
    ap.add_argument("model", type=Path, help="CRDF 模型文件")
    ap.add_argument("-o", "--output", type=Path, default=Path("webgl/model.json"))
    args = ap.parse_args()
    robot = load_robot(args.model)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(export_robot(robot)), encoding="utf-8")
    print(f"exported {args.output} ({args.output.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
