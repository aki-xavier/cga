"""CRDF — CGA Robot Description Format: 机器人描述 (YAML, 非 XML)。

URDF 的 CGA 版: 链接树 + 关节 + 几何 (blade) + 惯量, 用 YAML 表达。

与 URDF 的对应关系 (帧语义完全一致, 见 README "CRDF" 节):
  - joint.origin: 父 link frame → joint frame 的刚体变换, 两种写法等价:
      {xyz: [..], rpy: [..]}   (URDF 兼容, rpy = 外旋 X-Y-Z: R = Rz·Ry·Rx)
      {motor: {axis, angle, t}} (CGA 签名, Motor(axis, angle, t) 原样)
    载入时统一归一化为 Motor。
  - joint.axis: 表达在 joint frame (与 URDF 相同)。
  - link.geometry[].origin: 表达在 link frame。
  - link.inertial.com: 质心, 表达在 link frame。
  - 单位: 米/千克/弧度 (SI)。

CGA 特色 (URDF 没有的):
  - 几何是 blade: cylinder/box/sphere/plane/circle, 隐式精确, 无网格。
  - 同一几何可多角色复用: role: [visual, collision] (URDF 要写两遍)。
  - FK 走 Motor 链: M_child = M_parent · M_origin · Rot(axis, q) ——
    长链不积累矩阵正交性漂移 (versor 连乘保真)。

范围声明 (v1):
  - 无 mesh 引用 (任意三角网格几何不支持, blade 优先)。
  - 惯量只支持对角张量 (ixx/iyy/izz); 非零非对角 → 明确报错。
  - 无 SRDF/transmission/gazebo 语义 (纯运动学描述)。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path

import yaml

from cga.motors import Motor

# 关节类型
REVOLUTE = "revolute"
PRISMATIC = "prismatic"
CONTINUOUS = "continuous"
FIXED = "fixed"
MOVABLE = (REVOLUTE, PRISMATIC, CONTINUOUS)

# blade 类型
BLADES = ("cylinder", "box", "sphere", "plane", "circle")
ROLES = ("visual", "collision")


class RobotError(ValueError):
    """CRDF 解析/校验错误 (帧语义、图结构、几何、单位问题)。"""


@dataclass(frozen=True)
class Geometry:
    """link frame 里的一个几何 blade (可多角色复用)。

    blade 决定哪些参数字段有效:
      cylinder: radius, length (轴 = 局部 +Z, 与 URDF/engine 一致)
      box:      size = (w, h, d), 中心在原点
      sphere:   radius
      plane:    normal + distance (平面 n·x = d, CGA 扩展)
      circle:   radius (圆盘, 局部法向 +Z, CGA 扩展)
    """

    blade: str
    origin: Motor
    role: tuple[str, ...]
    material: str | None = None
    radius: float | None = None
    length: float | None = None
    size: tuple[float, float, float] | None = None
    normal: tuple[float, float, float] | None = None
    distance: float | None = None


@dataclass(frozen=True)
class Inertial:
    """link 惯量 (对角张量 v1, 质心 com 在 link frame)。"""

    mass: float
    com: tuple[float, float, float]
    ixx: float
    iyy: float
    izz: float


@dataclass(frozen=True)
class Link:
    name: str
    geometry: tuple[Geometry, ...] = ()
    inertial: Inertial | None = None


@dataclass(frozen=True)
class Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: Motor
    axis: tuple[float, float, float] | None = None
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    damping: float | None = None


@dataclass(frozen=True)
class Material:
    name: str
    color: tuple[float, float, float, float]  # rgba, 0-1


@dataclass(frozen=True)
class Robot:
    name: str
    base: str
    links: tuple[Link, ...]
    joints: tuple[Joint, ...]
    materials: tuple[Material, ...] = ()

    def link(self, name: str) -> Link:
        for lnk in self.links:
            if lnk.name == name:
                return lnk
        raise RobotError(f"no link named {name!r}")

    def joint(self, name: str) -> Joint:
        for j in self.joints:
            if j.name == name:
                return j
        raise RobotError(f"no joint named {name!r}")

    def child_links(self) -> dict[str, Joint]:
        """link name → 把它作为 child 的关节 (每个 link 至多一个父)。"""
        return {j.child: j for j in self.joints}

    def fk(self, q: dict[str, float]) -> dict[str, Motor]:
        """关节角度/位移 → 每个 link 的 world Motor。

        revolute/continuous: M_child = M_parent · M_origin · Rot(axis, q)
        prismatic:           M_child = M_parent · M_origin · Trans(axis·q)
        fixed:               M_child = M_parent · M_origin
        """
        missing = [j.name for j in self.joints if j.type in MOVABLE and j.name not in q]
        if missing:
            raise RobotError(f"fk 缺少关节角度: {missing}")
        world: dict[str, Motor] = {self.base: Motor.identity()}
        pending = list(self.joints)
        while pending:
            progressed = False
            for j in pending[:]:
                if j.parent not in world:
                    continue
                m = j.origin
                if j.type in (REVOLUTE, CONTINUOUS):
                    ax = j.axis
                    assert ax is not None  # 校验保证 movable 关节必有 axis
                    m = m.gp(Motor.rotor(ax, q[j.name]))
                elif j.type == PRISMATIC:
                    ax = j.axis
                    assert ax is not None
                    m = m.gp(
                        Motor.translator(
                            (ax[0] * q[j.name], ax[1] * q[j.name], ax[2] * q[j.name])
                        )
                    )
                world[j.child] = world[j.parent].gp(m)
                pending.remove(j)
                progressed = True
            if not progressed:
                raise RobotError("fk 图无法遍历 (有环或断链)")
        return world

    def fk_list(self, q: list[float]) -> dict[str, Motor]:
        """fk 的便捷形式: q 按 joints 顺序 (只含 movable 关节)。"""
        movable = [j for j in self.joints if j.type in MOVABLE]
        if len(q) != len(movable):
            raise RobotError(
                f"fk_list 需要 {len(movable)} 个值 (movable 关节数), got {len(q)}"
            )
        return self.fk({j.name: v for j, v in zip(movable, q, strict=True)})


# ── 解析 (dict → Robot, 含校验) ────────────────────────────────────


def _need(d: dict, key: str, where: str):
    if key not in d:
        raise RobotError(f"{where}: missing required key {key!r}")
    return d[key]


def _vec3(v, where: str) -> tuple[float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise RobotError(f"{where}: expected 3 numbers, got {v!r}")
    out = (float(v[0]), float(v[1]), float(v[2]))
    if not all(isfinite(x) for x in out):
        raise RobotError(f"{where}: non-finite value {v!r}")
    return out


def _motor(d: dict, where: str) -> Motor:
    """origin 两种写法 (xyz+rpy | motor) → Motor。"""
    if "motor" in d:
        if "xyz" in d or "rpy" in d:
            raise RobotError(f"{where}: origin 只允许 xyz/rpy 或 motor 一种写法")
        m = _need(d, "motor", where)
        axis = _vec3(_need(m, "axis", where), f"{where}.motor.axis")
        angle = float(_need(m, "angle", where))
        t = _vec3(m.get("t", [0.0, 0.0, 0.0]), f"{where}.motor.t")
        return Motor(axis, angle, t)
    xyz = _vec3(d.get("xyz", [0.0, 0.0, 0.0]), f"{where}.xyz")
    rpy = _vec3(d.get("rpy", [0.0, 0.0, 0.0]), f"{where}.rpy")
    # URDF rpy = 外旋 X-Y-Z: R = Rz(γ)·Ry(β)·Rx(α)
    return Motor.from_matrix(rpy_to_matrix(rpy), xyz)


def rpy_to_matrix(rpy: tuple[float, float, float]) -> list[list[float]]:
    """外旋 X-Y-Z (URDF 约定): R = Rz(γ)·Ry(β)·Rx(α)。"""
    import math

    a, b, c = rpy
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    # Rx(a) 后 Ry(b) 后 Rz(c):
    return [
        [cb * cc, sa * sb * cc - ca * sc, ca * sb * cc + sa * sc],
        [cb * sc, sa * sb * sc + ca * cc, ca * sb * sc - sa * cc],
        [-sb, sa * cb, ca * cb],
    ]


def matrix_to_rpy(R) -> tuple[float, float, float]:
    """旋转矩阵 → rpy (R = Rz·Ry·Rx), 逆函数; 万向节锁 (cosβ≈0) 用 γ=0 约定。"""
    import math

    (r00, r01, _), (r10, r11, _), (r20, r21, r22) = (R[0], R[1], R[2])
    cb = math.sqrt(r00 * r00 + r10 * r10)
    if cb > 1e-9:
        b = math.atan2(-r20, cb)
        a = math.atan2(r21, r22)
        c = math.atan2(r10, r00)
    else:  # 万向节锁: β = ±π/2, 取 γ = 0
        b = math.atan2(-r20, cb)
        a = math.atan2(-r01, r11)
        c = 0.0
    return (a, b, c)


def _inertial(d: dict, where: str) -> Inertial:
    mass = float(_need(d, "mass", where))
    if not isfinite(mass) or mass <= 0:
        raise RobotError(f"{where}: mass must be > 0, got {mass}")
    com = _vec3(d.get("com", [0.0, 0.0, 0.0]), f"{where}.com")
    i = _need(d, "inertia", where)
    ixx = float(i.get("ixx", 0.0))
    iyy = float(i.get("iyy", 0.0))
    izz = float(i.get("izz", 0.0))
    for k in ("ixx", "iyy", "izz"):
        v = float(i.get(k, 0.0))
        if not isfinite(v) or v < 0:
            raise RobotError(f"{where}.inertia.{k}: must be >= 0")
    off = [k for k in ("ixy", "ixz", "iyz") if float(i.get(k, 0.0)) != 0.0]
    if off:
        raise RobotError(
            f"{where}.inertia: 非对角项 {off} 非零 —— v1 只支持对角惯量张量"
        )
    return Inertial(mass=mass, com=com, ixx=ixx, iyy=iyy, izz=izz)


def _geometry(d: dict, where: str, materials: dict[str, Material]) -> Geometry:
    blade = _need(d, "blade", where)
    if blade not in BLADES:
        raise RobotError(
            f"{where}: unknown blade {blade!r} (支持 {BLADES}; mesh 不在 v1 范围)"
        )
    origin = _motor(d.get("origin", {}), f"{where}.origin")
    roles = tuple(d.get("role", list(ROLES)))
    if not roles or any(r not in ROLES for r in roles):
        raise RobotError(f"{where}: role 必须是 {ROLES} 的子集, got {roles!r}")
    material = d.get("material")
    if material is not None and material not in materials:
        raise RobotError(f"{where}: material {material!r} 未在 robot.materials 定义")
    g = Geometry(blade=blade, origin=origin, role=roles, material=material)
    if blade in ("cylinder", "sphere", "circle"):
        r = float(_need(d, "radius", where))
        if not isfinite(r) or r <= 0:
            raise RobotError(f"{where}: radius must be > 0, got {r}")
        g = _with(g, radius=r)
    if blade == "cylinder":
        ln = float(_need(d, "length", where))
        if not isfinite(ln) or ln <= 0:
            raise RobotError(f"{where}: length must be > 0, got {ln}")
        g = _with(g, length=ln)
    if blade == "box":
        size = _vec3(_need(d, "size", where), f"{where}.size")
        if any(s <= 0 for s in size):
            raise RobotError(f"{where}: box size 必须全 > 0, got {size}")
        g = _with(g, size=size)
    if blade == "plane":
        normal = _vec3(_need(d, "normal", where), f"{where}.normal")
        if all(x == 0.0 for x in normal):
            raise RobotError(f"{where}: plane normal 不能为零向量")
        g = _with(g, normal=normal, distance=float(d.get("distance", 0.0)))
    return g


def _with(g: Geometry, **kw) -> Geometry:
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
    )


def _joint(d: dict, where: str) -> Joint:
    jtype = _need(d, "type", where)
    if jtype not in MOVABLE + (FIXED,):
        raise RobotError(
            f"{where}: unknown joint type {jtype!r} (支持 {MOVABLE + (FIXED,)})"
        )
    parent = _need(d, "parent", where)
    child = _need(d, "child", where)
    if parent == child:
        raise RobotError(f"{where}: parent 不能等于 child ({parent!r})")
    origin = _motor(d.get("origin", {}), f"{where}.origin")
    axis: tuple[float, float, float] | None = None
    if jtype in MOVABLE:
        a = _vec3(_need(d, "axis", where), f"{where}.axis")
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
            raise RobotError(f"{where}: limit.lower ({lower}) 必须 < upper ({upper})")
    if jtype == CONTINUOUS and (lower is not None or upper is not None):
        raise RobotError(f"{where}: continuous 关节不能设 lower/upper limit")
    dyn = d.get("dynamics", {})
    return Joint(
        name=_need(d, "name", where),
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


def _link(d: dict, where: str, materials: dict[str, Material]) -> Link:
    geoms = []
    for i, gd in enumerate(d.get("geometry", [])):
        geoms.append(_geometry(gd, f"{where}.geometry[{i}]", materials))
    inertial = (
        _inertial(d["inertial"], f"{where}.inertial") if "inertial" in d else None
    )
    return Link(name=_need(d, "name", where), geometry=tuple(geoms), inertial=inertial)


def load_robot(source: str | Path) -> Robot:
    """解析 CRDF (YAML 字符串或文件路径) → Robot (含全部校验)。"""
    if isinstance(source, Path):
        text = source.read_text(encoding="utf-8")
        where = f"{source}"
    else:
        text = source
        where = "<crdf>"
    data = yaml.safe_load(text)
    if not isinstance(data, dict) or "robot" not in data:
        raise RobotError(f"{where}: 顶层必须是 {{robot: ...}} (CRDF)")
    r = data["robot"]
    if not isinstance(r, dict):
        raise RobotError(f"{where}: robot 必须是映射")
    name = _need(r, "name", where)
    base = _need(r, "base", where)

    mats: dict[str, Material] = {}
    for i, m in enumerate(r.get("materials", [])):
        mw = f"{where}.materials[{i}]"
        mname = _need(m, "name", mw)
        color = _vec4(_need(m, "color", mw), f"{mw}.color")
        mats[mname] = Material(mname, color)
    links = tuple(
        _link(d, f"{where}.links[{i}]", mats) for i, d in enumerate(r.get("links", []))
    )
    joints = tuple(
        _joint(d, f"{where}.joints[{i}]") for i, d in enumerate(r.get("joints", []))
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


def _vec4(v, where: str) -> tuple[float, float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 4:
        raise RobotError(f"{where}: expected 4 numbers (rgba), got {v!r}")
    out = (float(v[0]), float(v[1]), float(v[2]), float(v[3]))
    if not all(isfinite(x) for x in out):
        raise RobotError(f"{where}: non-finite {v!r}")
    return out


# ── 运动学: FK (Robot 方法, 见 Robot 类) ────────────────────────
