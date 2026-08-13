"""URDF ⇄ CRDF 转换器 (静态方法集合)。"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Any, ClassVar

import yaml

from cga.motors import Motor
from cga.robot import Geometry, Joint, Robot, RobotError, Rotation
from cga.urdf_io.dumper import Dumper


class UrdfConverter:
    """URDF XML ↔ CRDF YAML/Robot 双向转换。"""

    # URDF 支持的 blade (mesh 另行策略处理; plane/circle 是 CGA 扩展)
    URDF_BLADES: ClassVar[tuple[str, ...]] = ("cylinder", "box", "sphere")

    @staticmethod
    def motor_to_origin(m: Motor) -> dict[str, Any]:
        """Motor → {xyz, rpy} (rpy 全零则省略, 保持文件干净)。"""
        mt = m.to_matrix()
        xyz = [round(float(mt[i][3]), 8) for i in range(3)]
        r = [[float(mt[i][j]) for j in range(3)] for i in range(3)]
        a, b, c = Rotation.matrix_to_rpy(r)
        rpy = [round(v, 8) for v in (a, b, c)]
        out: dict[str, Any] = {"xyz": xyz}
        if any(abs(v) > 1e-9 for v in rpy):
            out["rpy"] = rpy
        return out

    # ── 导入: URDF XML → CRDF YAML ────────────────────────────────

    @staticmethod
    def geom_dict(
        kind: str, g: ET.Element, origin: dict, mesh: str = "skip"
    ) -> dict[str, Any] | None:
        if kind == "cylinder":
            r = float(g.get("radius", "0"))
            ln = float(g.get("length", "0"))
            return {"blade": "cylinder", "radius": r, "length": ln, "origin": origin}
        if kind == "sphere":
            return {
                "blade": "sphere",
                "radius": float(g.get("radius", "0")),
                "origin": origin,
            }
        if kind == "box":
            size = [float(v) for v in g.get("size", "0 0 0").split()]
            return {"blade": "box", "size": size, "origin": origin}
        if kind == "mesh":
            # 引擎 blade-only: mesh 不建模。skip (默认) = 忽略; keep = 保留
            # 文件引用 (interop round-trip, 引擎不渲染)
            if mesh == "skip":
                return None
            if mesh == "keep":
                return {
                    "blade": "mesh",
                    "file": g.get("filename", ""),
                    "scale": [float(v) for v in g.get("scale", "1 1 1").split()],
                    "origin": origin,
                }
            raise RobotError(
                "URDF geometry <mesh> 需要 mesh_policy='skip' (默认, 忽略 mesh) "
                "或 'keep' (保留文件引用 round-trip)"
            )
        raise RobotError(f"URDF geometry <{kind}> 不在 CRDF v1 范围")

    @staticmethod
    def role_geometry(
        role: str,
        link: ET.Element,
        mats: dict[str, list[float]],
        mesh: str = "skip",
    ) -> list[dict[str, Any]]:
        """提取 link 的 visual/collision 块 → 几何 dict (role 单元素, 导入保真)。

        材质: <material name=..> 直接引用; 内联 <material><color/></material>
        → 合成名 {link}_{role}_{i} 并登记 rgba 进 mats, 保证校验可过。
        """
        out = []
        for i, block in enumerate(link.findall(role)):
            o = block.find("origin")
            if o is not None:
                xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
                rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
                origin = UrdfConverter.motor_to_origin(
                    Motor.from_matrix(Rotation.rpy_to_matrix(tuple(rpy)), xyz)
                )
            else:
                origin = {"xyz": [0.0, 0.0, 0.0]}
            geo = block.find("geometry")
            if geo is None or len(geo) == 0:
                continue
            gd = UrdfConverter.geom_dict(geo[0].tag, geo[0], origin, mesh)
            if gd is None:
                continue  # mesh 跳过策略
            mat = block.find("material")
            name = None
            if mat is not None:
                name = mat.get("name")
                c = mat.find("color")
                if name is not None:
                    # 内联带名定义 (如 <material name="white"><color/></material>):
                    # 首次定义登记颜色, 后续 <material name=../> 引用可解析
                    if c is not None:
                        mats.setdefault(
                            name,
                            [float(v) for v in c.get("rgba", "1 1 1 1").split()],
                        )
                elif c is not None:  # 无名字内联颜色 → 合成名
                    name = f"{link.get('name')}_{role}_{i}"
                    mats[name] = [
                        float(v) for v in c.get("rgba", "1 1 1 1").split()
                    ]
            if name is not None:
                gd["material"] = name
            gd["role"] = [role]
            out.append(gd)
        return out

    @staticmethod
    def merge_roles(geoms: list[dict[str, Any]]) -> list[dict[str, Any]]:
        """完全相同 (blade/参数/origin/材质) 的 visual+collision 合并为一条
        role: [visual, collision] —— CRDF 的复用卖点; 导出时仍按 role 分写。"""
        by_key: dict[tuple, dict[str, Any]] = {}
        for g in geoms:
            rpy = tuple(g["origin"].get("rpy", [0.0, 0.0, 0.0]))
            # material 不进 key: URDF 的 collision 块不带材质, 而 visual 带 ——
            # 这正是要消除的冗余; 合并时保留首个非空材质 (通常是 visual 的)
            key = (
                g["blade"],
                g.get("radius"),
                g.get("length"),
                tuple(g.get("size", ())),
                tuple(g["origin"]["xyz"]),
                rpy,
            )
            if key in by_key:
                for r in g["role"]:
                    if r not in by_key[key]["role"]:
                        by_key[key]["role"].append(r)
                if (
                    by_key[key].get("material") is None
                    and g.get("material") is not None
                ):
                    by_key[key]["material"] = g["material"]
            else:
                by_key[key] = dict(g)
        return list(by_key.values())

    @staticmethod
    def urdf_to_crdf(urdf_xml: str, mesh_policy: str = "skip") -> str:
        """解析 URDF XML → CRDF YAML 文本。

        mesh_policy: "skip" (默认, 忽略 mesh —— 引擎 blade-only; 某 link 的
        visual 全被跳过时 collision 基本体提升为 [visual, collision]) |
        "keep" (保留 mesh 文件引用, round-trip, 引擎不渲染) | "error"
        (遇到 mesh 直接报错, 严格模式)。
        """
        if mesh_policy not in ("error", "skip", "keep"):
            raise RobotError(
                f"mesh_policy 必须是 'error'|'skip'|'keep', got {mesh_policy!r}"
            )
        try:
            root = ET.fromstring(urdf_xml)
        except ET.ParseError as e:
            raise RobotError(f"URDF XML 解析失败: {e}") from e
        if root.tag != "robot":
            raise RobotError(f"根元素必须是 <robot>, got <{root.tag}>")

        # 全局材质 (robot 级 <material name=..>), 名字可被 link 引用
        mats: dict[str, list[float]] = {}
        for m in root.findall("material"):
            name = m.get("name")
            c = m.find("color")
            if name and c is not None:
                mats[name] = [float(v) for v in c.get("rgba", "1 1 1 1").split()]

        links: list[dict] = []
        for link_el in root.findall("link"):
            ld: dict[str, Any] = {"name": link_el.get("name")}
            # URDF 里 visual/collision 分写 → 导入后完全相同者合并 (多角色
            # 复用), 仅视觉或仅碰撞的几何保持独立。
            vg = UrdfConverter.role_geometry("visual", link_el, mats, mesh_policy)
            cg = UrdfConverter.role_geometry("collision", link_el, mats, mesh_policy)
            if not vg and cg:
                # 视觉全是 mesh 被跳过 → 碰撞圆柱兼任视觉 (blade 化建模)
                for g in cg:
                    g["role"] = ["visual", "collision"]
            geoms = UrdfConverter.merge_roles(vg + cg)
            if geoms:
                ld["geometry"] = geoms
            inert = link_el.find("inertial")
            if inert is not None:
                i = inert.find("inertia")
                com = inert.find("origin")
                com_xyz = (
                    com.get("xyz", "0 0 0").split()
                    if com is not None
                    else ["0", "0", "0"]
                )
                ld["inertial"] = {
                    "mass": float(inert.find("mass").get("value", "0")),
                    "com": [float(v) for v in com_xyz],
                    "inertia": {
                        k: float(i.get(k, "0"))
                        for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
                    },
                }
            links.append(ld)

        joints: list[dict] = []
        for j in root.findall("joint"):
            jtype = j.get("type")
            if jtype not in Joint.MOVABLE + (Joint.FIXED,):
                raise RobotError(
                    f"URDF joint type {jtype!r} 不支持 "
                    "(v1: revolute/continuous/prismatic/fixed)"
                )
            o = j.find("origin")
            oxyz = o.get("xyz", "0 0 0").split() if o is not None else ["0", "0", "0"]
            orpy = o.get("rpy", "0 0 0").split() if o is not None else ["0", "0", "0"]
            xyz = [float(v) for v in oxyz]
            rpy = [float(v) for v in orpy]
            jd: dict[str, Any] = {
                "name": j.get("name"),
                "type": jtype,
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "origin": UrdfConverter.motor_to_origin(
                    Motor.from_matrix(Rotation.rpy_to_matrix(tuple(rpy)), xyz)
                ),
            }
            ax = j.find("axis")
            if ax is not None:
                jd["axis"] = [float(v) for v in ax.get("xyz", "0 0 1").split()]
            lim = j.find("limit")
            if lim is not None:
                jd["limit"] = {
                    k: float(lim.get(k))
                    for k in ("lower", "upper", "effort", "velocity")
                    if lim.get(k) is not None
                }
            dyn = j.find("dynamics")
            if dyn is not None and dyn.get("damping") is not None:
                jd["dynamics"] = {"damping": float(dyn.get("damping"))}
            joints.append(jd)

        robot = {
            "name": root.get("name", "robot"),
            "base": UrdfConverter.infer_base(joints),
        }
        if mats:
            robot["materials"] = [
                {"name": n, "color": list(c)} for n, c in mats.items()
            ]
        robot["links"] = links
        robot["joints"] = joints
        return yaml.dump(
            {"robot": robot}, Dumper=Dumper, sort_keys=False, allow_unicode=True
        )

    @staticmethod
    def infer_base(joints: list[dict]) -> str:
        children = {j["child"] for j in joints}
        parents = {j["parent"] for j in joints}
        candidates = parents - children
        if len(candidates) != 1:
            raise RobotError(
                f"无法唯一确定 base link (候选 {sorted(candidates)}); 请手工标注"
            )
        return sorted(candidates)[0]

    # ── 导出: CRDF Robot → URDF XML ───────────────────────────────

    @staticmethod
    def f(v: float, digits: int = 8) -> str:
        """浮点 → 紧凑十进制 (去尾零)。"""
        s = f"{v:.{digits}f}".rstrip("0").rstrip(".")
        return s if s not in ("", "-0") else "0"

    @staticmethod
    def xyz(m: Motor) -> tuple[float, float, float]:
        mt = m.to_matrix()
        return (float(mt[0][3]), float(mt[1][3]), float(mt[2][3]))

    @staticmethod
    def rpy(m: Motor) -> tuple[float, float, float]:
        mt = m.to_matrix()
        r = [[float(mt[i][j]) for j in range(3)] for i in range(3)]
        return Rotation.matrix_to_rpy(r)

    @staticmethod
    def add_origin(el: ET.Element, m: Motor) -> None:
        o = ET.SubElement(el, "origin")
        x, y, z = UrdfConverter.xyz(m)
        o.set("xyz", f"{UrdfConverter.f(x)} {UrdfConverter.f(y)} {UrdfConverter.f(z)}")
        a, b, c = UrdfConverter.rpy(m)
        if any(abs(v) > 1e-9 for v in (a, b, c)):
            o.set(
                "rpy",
                f"{UrdfConverter.f(a)} {UrdfConverter.f(b)} {UrdfConverter.f(c)}",
            )

    @staticmethod
    def add_geometry(el: ET.Element, g: Geometry) -> None:
        geo = ET.SubElement(el, "geometry")
        if g.blade == "cylinder":
            geo = ET.SubElement(geo, "cylinder")
            geo.set("radius", UrdfConverter.f(g.radius or 0.0))
            geo.set("length", UrdfConverter.f(g.length or 0.0))
        elif g.blade == "sphere":
            geo = ET.SubElement(geo, "sphere")
            geo.set("radius", UrdfConverter.f(g.radius or 0.0))
        elif g.blade == "box":
            geo = ET.SubElement(geo, "box")
            geo.set(
                "size",
                " ".join(UrdfConverter.f(v) for v in (g.size or (0.0, 0.0, 0.0))),
            )
        elif g.blade == "mesh":
            geo = ET.SubElement(geo, "mesh")
            geo.set("filename", g.file or "")
            if g.scale is not None:
                geo.set("scale", " ".join(UrdfConverter.f(v) for v in g.scale))
        else:
            raise RobotError(
                f"blade {g.blade!r} 无法导出为 URDF "
                f"(v1 仅 cylinder/box/sphere/mesh; plane/circle 是 CGA 扩展)"
            )

    @staticmethod
    def crdf_to_urdf(robot: Robot) -> str:
        """CRDF Robot → URDF XML 文本 (供 pydrake/UrdfScene 等消费)。"""
        f = UrdfConverter.f
        root = ET.Element("robot", {"name": robot.name})
        for m in robot.materials:
            me = ET.SubElement(root, "material", {"name": m.name})
            ET.SubElement(me, "color", {"rgba": " ".join(f(c) for c in m.color)})

        for link in robot.links:
            le = ET.SubElement(root, "link", {"name": link.name})
            for g in link.geometry:
                if "visual" in g.role:
                    ve = ET.SubElement(le, "visual")
                    UrdfConverter.add_origin(ve, g.origin)
                    UrdfConverter.add_geometry(ve, g)
                    if g.material:
                        ET.SubElement(ve, "material", {"name": g.material})
                if "collision" in g.role:
                    # URDF 惯例: collision 块不带 material (语义上无意义)
                    ce = ET.SubElement(le, "collision")
                    UrdfConverter.add_origin(ce, g.origin)
                    UrdfConverter.add_geometry(ce, g)
            inert = link.inertial
            if inert is not None:
                ie = ET.SubElement(le, "inertial")
                io = ET.SubElement(ie, "origin")
                cx, cy, cz = inert.com
                io.set("xyz", f"{f(cx)} {f(cy)} {f(cz)}")
                ET.SubElement(ie, "mass", {"value": f(inert.mass)})
                ET.SubElement(
                    ie,
                    "inertia",
                    {
                        "ixx": f(inert.ixx),
                        "iyy": f(inert.iyy),
                        "izz": f(inert.izz),
                        "ixy": f(inert.ixy),
                        "ixz": f(inert.ixz),
                        "iyz": f(inert.iyz),
                    },
                )

        for j in robot.joints:
            je = ET.SubElement(root, "joint", {"name": j.name, "type": j.type})
            ET.SubElement(je, "parent", {"link": j.parent})
            ET.SubElement(je, "child", {"link": j.child})
            UrdfConverter.add_origin(je, j.origin)
            if j.type in (Joint.REVOLUTE, Joint.CONTINUOUS, Joint.PRISMATIC):
                ax = j.axis
                assert ax is not None
                ET.SubElement(je, "axis", {"xyz": f"{f(ax[0])} {f(ax[1])} {f(ax[2])}"})
            if j.type != Joint.CONTINUOUS and (
                j.lower is not None or j.upper is not None
            ):
                lim = {}
                for k in ("lower", "upper", "effort", "velocity"):
                    v = getattr(j, k)
                    if v is not None:
                        lim[k] = f(v)
                ET.SubElement(je, "limit", lim)
            if j.damping is not None:
                ET.SubElement(je, "dynamics", {"damping": f(j.damping)})

        return ET.tostring(root, encoding="unicode")
