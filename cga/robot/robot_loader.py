"""CRDF 解析器: dict/YAML → Robot (含全部校验, 静态方法集合)。"""

from __future__ import annotations

from math import isfinite
from pathlib import Path

import yaml

from cga.motors import Motor
from cga.robot.geometry import Geometry
from cga.robot.inertial import Inertial
from cga.robot.joint import Joint
from cga.robot.link import Link
from cga.robot.material import Material
from cga.robot.robot import Robot
from cga.robot.robot_error import RobotError
from cga.robot.rotation import Rotation


class RobotLoader:
    """CRDF (YAML 文本或文件路径) → Robot。解析即校验。"""

    @staticmethod
    def need(d: dict, key: str, where: str):
        if key not in d:
            raise RobotError(f"{where}: missing required key {key!r}")
        return d[key]

    @staticmethod
    def vec3(v, where: str) -> tuple[float, float, float]:
        if not isinstance(v, (list, tuple)) or len(v) != 3:
            raise RobotError(f"{where}: expected 3 numbers, got {v!r}")
        out = (float(v[0]), float(v[1]), float(v[2]))
        if not all(isfinite(x) for x in out):
            raise RobotError(f"{where}: non-finite value {v!r}")
        return out

    @staticmethod
    def vec4(v, where: str) -> tuple[float, float, float, float]:
        if not isinstance(v, (list, tuple)) or len(v) != 4:
            raise RobotError(f"{where}: expected 4 numbers (rgba), got {v!r}")
        out = (float(v[0]), float(v[1]), float(v[2]), float(v[3]))
        if not all(isfinite(x) for x in out):
            raise RobotError(f"{where}: non-finite {v!r}")
        return out

    @staticmethod
    def motor(d: dict, where: str) -> Motor:
        """origin 两种写法 (xyz+rpy | motor) → Motor。"""
        if "motor" in d:
            if "xyz" in d or "rpy" in d:
                raise RobotError(f"{where}: origin 只允许 xyz/rpy 或 motor 一种写法")
            m = RobotLoader.need(d, "motor", where)
            axis = RobotLoader.vec3(
                RobotLoader.need(m, "axis", where), f"{where}.motor.axis"
            )
            angle = float(RobotLoader.need(m, "angle", where))
            t = RobotLoader.vec3(m.get("t", [0.0, 0.0, 0.0]), f"{where}.motor.t")
            return Motor(axis, angle, t)
        xyz = RobotLoader.vec3(d.get("xyz", [0.0, 0.0, 0.0]), f"{where}.xyz")
        rpy = RobotLoader.vec3(d.get("rpy", [0.0, 0.0, 0.0]), f"{where}.rpy")
        # URDF rpy = 外旋 X-Y-Z: R = Rz(γ)·Ry(β)·Rx(α)
        return Motor.from_matrix(Rotation.rpy_to_matrix(rpy), xyz)

    @staticmethod
    def inertial(d: dict, where: str) -> Inertial:
        mass = float(RobotLoader.need(d, "mass", where))
        if not isfinite(mass) or mass <= 0:
            raise RobotError(f"{where}: mass must be > 0, got {mass}")
        com = RobotLoader.vec3(d.get("com", [0.0, 0.0, 0.0]), f"{where}.com")
        i = RobotLoader.need(d, "inertia", where)
        ixx = float(i.get("ixx", 0.0))
        iyy = float(i.get("iyy", 0.0))
        izz = float(i.get("izz", 0.0))
        ixy = float(i.get("ixy", 0.0))
        ixz = float(i.get("ixz", 0.0))
        iyz = float(i.get("iyz", 0.0))
        for k, v in (("ixx", ixx), ("iyy", iyy), ("izz", izz)):
            if not isfinite(v) or v < 0:
                raise RobotError(f"{where}.inertia.{k}: must be >= 0")
        for k, v in (("ixy", ixy), ("ixz", ixz), ("iyz", iyz)):
            if not isfinite(v):
                raise RobotError(f"{where}.inertia.{k}: must be finite")
        return Inertial(
            mass=mass, com=com, ixx=ixx, iyy=iyy, izz=izz,
            ixy=ixy, ixz=ixz, iyz=iyz,
        )

    @staticmethod
    def with_geometry(g: Geometry, **kw) -> Geometry:
        """frozen dataclass 重建 (字段少, 显式一点)。"""
        return Geometry(
            blade=g.blade,
            origin=g.origin,
            role=g.role,
            material=g.material,
            radius=kw.get("radius", g.radius),
            length=kw.get("length", g.length),
            size=kw.get("size", g.size),
            normal=kw.get("normal", g.normal),
            distance=kw.get("distance", g.distance),
            file=kw.get("file", g.file),
            scale=kw.get("scale", g.scale),
        )

    @staticmethod
    def geometry(d: dict, where: str, materials: dict[str, Material]) -> Geometry:
        blade = RobotLoader.need(d, "blade", where)
        if blade not in Geometry.BLADES:
            raise RobotError(
                f"{where}: unknown blade {blade!r} "
                f"(支持 {Geometry.BLADES}; mesh 不在 v1 范围)"
            )
        origin = RobotLoader.motor(d.get("origin", {}), f"{where}.origin")
        roles = tuple(d.get("role", list(Geometry.ROLES)))
        if not roles or any(r not in Geometry.ROLES for r in roles):
            raise RobotError(
                f"{where}: role 必须是 {Geometry.ROLES} 的子集, got {roles!r}"
            )
        material = d.get("material")
        if material is not None and material not in materials:
            raise RobotError(
                f"{where}: material {material!r} 未在 robot.materials 定义"
            )
        g = Geometry(blade=blade, origin=origin, role=roles, material=material)
        if blade in ("cylinder", "sphere", "circle"):
            r = float(RobotLoader.need(d, "radius", where))
            if not isfinite(r) or r <= 0:
                raise RobotError(f"{where}: radius must be > 0, got {r}")
            g = RobotLoader.with_geometry(g, radius=r)
        if blade == "cylinder":
            ln = float(RobotLoader.need(d, "length", where))
            if not isfinite(ln) or ln <= 0:
                raise RobotError(f"{where}: length must be > 0, got {ln}")
            g = RobotLoader.with_geometry(g, length=ln)
        if blade == "box":
            size = RobotLoader.vec3(RobotLoader.need(d, "size", where), f"{where}.size")
            if any(s <= 0 for s in size):
                raise RobotError(f"{where}: box size 必须全 > 0, got {size}")
            g = RobotLoader.with_geometry(g, size=size)
        if blade == "plane":
            normal = RobotLoader.vec3(
                RobotLoader.need(d, "normal", where), f"{where}.normal"
            )
            if all(x == 0.0 for x in normal):
                raise RobotError(f"{where}: plane normal 不能为零向量")
            g = RobotLoader.with_geometry(
                g, normal=normal, distance=float(d.get("distance", 0.0))
            )
        if blade == "mesh":
            file = RobotLoader.need(d, "file", where)
            if not isinstance(file, str) or not file.strip():
                raise RobotError(f"{where}: mesh 需要非空 file 引用 (URDF 式文件引用)")
            scale = RobotLoader.vec3(d.get("scale", [1.0, 1.0, 1.0]), f"{where}.scale")
            g = RobotLoader.with_geometry(g, file=file, scale=scale)
        return g

    @staticmethod
    def joint(d: dict, where: str) -> Joint:
        jtype = RobotLoader.need(d, "type", where)
        if jtype not in Joint.MOVABLE + (Joint.FIXED,):
            raise RobotError(
                f"{where}: unknown joint type {jtype!r} "
                f"(支持 {Joint.MOVABLE + (Joint.FIXED,)})"
            )
        parent = RobotLoader.need(d, "parent", where)
        child = RobotLoader.need(d, "child", where)
        if parent == child:
            raise RobotError(f"{where}: parent 不能等于 child ({parent!r})")
        origin = RobotLoader.motor(d.get("origin", {}), f"{where}.origin")
        axis: tuple[float, float, float] | None = None
        if jtype in Joint.MOVABLE:
            a = RobotLoader.vec3(RobotLoader.need(d, "axis", where), f"{where}.axis")
            n = (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5
            if n <= 1e-12:
                raise RobotError(f"{where}: axis 不能为零向量, got {a}")
            axis = (a[0] / n, a[1] / n, a[2] / n)  # 归一化 (URDF 允许不归一, 此处统一)
        limit = d.get("limit")
        lower = upper = effort = velocity = None
        if limit is not None:
            lower = float(limit.get("lower")) if "lower" in limit else None
            upper = float(limit.get("upper")) if "upper" in limit else None
            effort = float(limit.get("effort")) if "effort" in limit else None
            velocity = float(limit.get("velocity")) if "velocity" in limit else None
            if lower is not None and upper is not None and lower >= upper:
                raise RobotError(
                    f"{where}: limit.lower ({lower}) 必须 < upper ({upper})"
                )
        if jtype == Joint.CONTINUOUS and (lower is not None or upper is not None):
            raise RobotError(f"{where}: continuous 关节不能设 lower/upper limit")
        dyn = d.get("dynamics", {})
        return Joint(
            name=RobotLoader.need(d, "name", where),
            type=jtype,
            parent=parent,
            child=child,
            origin=origin,
            axis=axis,
            lower=lower,
            upper=upper,
            effort=effort,
            velocity=velocity,
            damping=float(dyn["damping"]) if "damping" in dyn else None,
        )

    @staticmethod
    def link(d: dict, where: str, materials: dict[str, Material]) -> Link:
        geoms = []
        for i, gd in enumerate(d.get("geometry", [])):
            geoms.append(
                RobotLoader.geometry(gd, f"{where}.geometry[{i}]", materials)
            )
        inertial = (
            RobotLoader.inertial(d["inertial"], f"{where}.inertial")
            if "inertial" in d
            else None
        )
        return Link(
            name=RobotLoader.need(d, "name", where),
            geometry=tuple(geoms),
            inertial=inertial,
        )

    @staticmethod
    def load(source: str | Path) -> Robot:
        """解析 CRDF (YAML 文本或文件路径) → Robot (含全部校验)。

        str 默认按 YAML 文本处理; 若 str 是磁盘上存在的文件路径且不含
        换行 (无换行的 YAML 文档无意义), 按路径读取 —— 防 str 路径脚枪。
        """
        if isinstance(source, Path):
            p: Path | None = source
        elif "\n" not in source and Path(source).exists():
            # str 且无换行 (无换行的 YAML 文档无意义) 且磁盘存在 → 按路径读
            p = Path(source)
        else:
            p = None
        if p is not None:
            text = p.read_text(encoding="utf-8")
            where = f"{p}"
        else:
            text = source
            where = "<crdf>"
        data = yaml.safe_load(text)
        if not isinstance(data, dict) or "robot" not in data:
            raise RobotError(f"{where}: 顶层必须是 {{robot: ...}} (CRDF)")
        r = data["robot"]
        if not isinstance(r, dict):
            raise RobotError(f"{where}: robot 必须是映射")
        name = RobotLoader.need(r, "name", where)
        base = RobotLoader.need(r, "base", where)

        mats: dict[str, Material] = {}
        for i, m in enumerate(r.get("materials", [])):
            mw = f"{where}.materials[{i}]"
            mname = RobotLoader.need(m, "name", mw)
            color = RobotLoader.vec4(RobotLoader.need(m, "color", mw), f"{mw}.color")
            mats[mname] = Material(mname, color)
        links = tuple(
            RobotLoader.link(d, f"{where}.links[{i}]", mats)
            for i, d in enumerate(r.get("links", []))
        )
        joints = tuple(
            RobotLoader.joint(d, f"{where}.joints[{i}]")
            for i, d in enumerate(r.get("joints", []))
        )

        names = {lnk.name for lnk in links}
        if base not in names:
            raise RobotError(f"{where}: base {base!r} 不在 links 中")
        for j in joints:
            for ref, kind in ((j.parent, "parent"), (j.child, "child")):
                if ref not in names:
                    raise RobotError(f"{where}: joint {j.name!r}.{kind} {ref!r} 不存在")
        children = {j.child for j in joints}
        if base in children:
            raise RobotError(f"{where}: base link {base!r} 不能是任何 joint 的 child")
        seen_parents = set()
        for j in joints:
            if j.child in seen_parents:
                raise RobotError(f"{where}: link {j.child!r} 有多个父关节 (必须是树)")
            seen_parents.add(j.child)
        # 连通性: 从 base 出发能到达所有 link (单父 + 无环已保证树形)
        reachable = {base}
        changed = True
        while changed:
            changed = False
            for j in joints:
                if j.parent in reachable and j.child not in reachable:
                    reachable.add(j.child)
                    changed = True
        missing = {lnk.name for lnk in links} - reachable
        if missing:
            raise RobotError(f"{where}: links {sorted(missing)} 不可达 (未挂在树上)")

        return Robot(
            name=name,
            base=base,
            links=links,
            joints=joints,
            materials=tuple(mats.values()),
        )
