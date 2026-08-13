from cga.engine.color import Color
from cga.engine.mesh import Mesh


class Scene:
    """场景: 对象列表 + 灯光列表 + 背景色 (three.js Scene)。"""

    def __init__(self, background: Color | None = None):
        self.objects: list[Mesh] = []
        self.lights: list = []
        self.background = background if background is not None else Color(0x87CEEB)

    def add(self, *objs) -> None:
        for o in objs:
            if isinstance(o, Mesh):
                self.objects.append(o)
            else:
                self.lights.append(o)
