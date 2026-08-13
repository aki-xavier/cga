"""CRDF Robot (链接树 + FK)。"""

from __future__ import annotations

from dataclasses import dataclass

from cga.motors import Motor
from cga.robot.joint import Joint
from cga.robot.link import Link
from cga.robot.material import Material
from cga.robot.robot_error import RobotError


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
        missing = [
            j.name for j in self.joints if j.type in Joint.MOVABLE and j.name not in q
        ]
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
                if j.type in (Joint.REVOLUTE, Joint.CONTINUOUS):
                    ax = j.axis
                    assert ax is not None  # 校验保证 movable 关节必有 axis
                    m = m.gp(Motor.rotor(ax, q[j.name]))
                elif j.type == Joint.PRISMATIC:
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
        movable = [j for j in self.joints if j.type in Joint.MOVABLE]
        if len(q) != len(movable):
            raise RobotError(
                f"fk_list 需要 {len(movable)} 个值 (movable 关节数), got {len(q)}"
            )
        return self.fk({j.name: v for j, v in zip(movable, q, strict=True)})
