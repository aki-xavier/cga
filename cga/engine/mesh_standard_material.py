from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import mlx.core as mx

from cga.engine.color import Color
from cga.engine.material import Material

if TYPE_CHECKING:
    from cga.engine.ambient_light import AmbientLight


class MeshStandardMaterial(Material):
    """标准材质: 环境 + Lambert 漫反射 + Blinn-Phong 高光。

    metalness: 1 → 高光色 = 自身颜色 (金属), 0 → 高光白。
    roughness: 0..1 → 高光指数 200→4 (指数越低越糊)。
    v1 不做 PBR 能量守恒/IBL, 是廉价近似 (与 three.js 的差距如实标注)。
    """

    def __init__(
        self,
        color: Color | int = 0xFFFFFF,
        roughness: float = 0.5,
        metalness: float = 0.0,
        emissive: Color | int = 0x000000,
        opacity: float = 1.0,
        ior: float = 1.5,
        absorption: float = 0.0,
    ):
        self.color = Color(color) if isinstance(color, int) else color
        self.roughness = float(min(1.0, max(0.0, roughness)))
        self.metalness = float(min(1.0, max(0.0, metalness)))
        self.emissive = Color(emissive) if isinstance(emissive, int) else emissive
        self.opacity = float(min(1.0, max(0.0, opacity)))
        self.ior = float(max(1.0, ior))
        self.absorption = float(max(0.0, absorption))

    def shade(
        self,
        p: mx.array,
        n: mx.array,
        d: mx.array,
        lights: Sequence,
        ambient: AmbientLight | None = None,
        vis: Sequence | None = None,
    ) -> mx.array:
        """Blinn-Phong 着色 (p = 命中点, n = 相机空间法向, d = 射线方向)。

        L 与点光源位置均已变换进相机空间 (motor 共轭); 每灯光的方向/
        衰减由 light.direction_at 多态给出。vis = Renderer 注入的每灯光
        阴影可见性 (N,) (None = 无阴影, 全可见)。
        """
        v = -d
        k = 1.0 - self.roughness
        expo = 4.0 + 196.0 * k * k  # roughness 0→200, 1→4
        diff_c = mx.array(self.color.rgb(), dtype=mx.float32) * (1.0 - self.metalness)
        spec_c = mx.array(
            tuple(
                m * (1.0 - self.metalness) + c * self.metalness
                for m, c in zip((1.0, 1.0, 1.0), self.color.rgb())
            ),
            dtype=mx.float32,
        )
        out = mx.broadcast_to(mx.array(self.emissive.rgb(), dtype=mx.float32), p.shape)
        if ambient is not None:
            amb = mx.array(ambient.color.rgb(), dtype=mx.float32) * ambient.intensity
            out = out + mx.broadcast_to(amb, p.shape) * diff_c
        # N·V 作高光可见门 (掠射角高光消失, 廉价近似)
        ndv = mx.maximum(mx.sum(n * v, axis=-1, keepdims=True), 0.0)
        for i, light in enumerate(lights):
            lc = mx.array(light.color.rgb(), dtype=mx.float32)
            ld, atten = light.direction_at(p)
            nl = mx.maximum(mx.sum(n * ld, axis=-1, keepdims=True), 0.0)
            h = ld + v
            hn = mx.sqrt(mx.sum(h * h, axis=-1, keepdims=True))
            h = h / mx.maximum(hn, 1e-12)  # H=0 (光与视线反向) 时防 NaN
            spec = mx.pow(mx.maximum(mx.sum(n * h, axis=-1, keepdims=True), 0.0), expo)
            contrib = lc * atten * (diff_c * nl + spec_c * spec * ndv)
            if vis is not None:
                contrib = contrib * vis[i][:, None]  # 阴影可见性
            out = out + contrib
        return out
