"""渲染图元数据类。"""

from typing import NamedTuple

from cga.multivector import Multivector


class RenderPrimitive(NamedTuple):
    """渲染图元 (鸭子类型自容, 不依赖 scenegraph)。"""

    kind: str  # "plane" / "sphere" / "cylinder"
    blade: Multivector  # 米制 Plane/Sphere/Cylinder blade
    region: int = 0  # 区域 id (掩码模式查表用)
    alpha: float = 1.0  # <1 = 半透明 (front-to-back 合成, 只取前表面)
