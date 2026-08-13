from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import mlx.core as mx

if TYPE_CHECKING:
    from cga.engine.ambient_light import AmbientLight


class Material:
    """材质基类: 着色走多态 shade (子类实现), 渲染循环零 isinstance。

    opacity: <1 = 半透明; ior: 折射率 (仅 opacity<1 时生效)。
    ior=1 → 无弯折无 Fresnel 反射, 退化为纯 alpha 混合;
    opacity=0 & ior>1 → 纯净玻璃 (只有 Fresnel 反射 + 折射)。
    absorption: Beer 吸收系数 σ (1/单位长度, 介质内行程衰减 exp(−σ·d))。
    另见 shade 的 vis 参数 (阴影可见性, Renderer 注入)。
    """

    opacity = 1.0
    ior = 1.5
    absorption = 0.0

    def shade(
        self,
        p: mx.array,
        n: mx.array,
        d: mx.array,
        lights: Sequence,
        ambient: AmbientLight | None = None,
        vis: Sequence | None = None,
    ) -> mx.array:
        raise NotImplementedError
