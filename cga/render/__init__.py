"""逆渲染: 把 CGA 图元场景渲染回 2D 图像 (每类一个文件)。

用途: 重建闭环验证 (模型 → 图像 round-trip) 与 novel-view 预览。
渲染器是 PrimitiveRenderer (静态方法集合); RenderPrimitive/RenderResult
是数据类。约定与限制见 PrimitiveRenderer 的文档。
"""

from cga.render.primitive_renderer import PrimitiveRenderer
from cga.render.render_primitive import RenderPrimitive
from cga.render.render_result import RenderResult

__all__ = [
    "PrimitiveRenderer",
    "RenderPrimitive",
    "RenderResult",
]
