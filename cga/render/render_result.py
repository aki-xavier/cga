"""渲染结果数据类。"""

from typing import NamedTuple

import mlx.core as mx


class RenderResult(NamedTuple):
    """渲染结果。"""

    depth: mx.array  # (H,W) float32 相机 Z (无命中 = 0)
    rgb: mx.array  # (H,W,3) uint8 可视化
