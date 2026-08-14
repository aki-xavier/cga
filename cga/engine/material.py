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

    def shade_params(self) -> tuple[mx.array, mx.array, mx.array, float]:
        """批量着色所需的逐对象标量参数: (emissive, diff, spec, expo)。

        emissive/diff/spec 为 (3,) float32 线性色, expo 为标量高光指数。
        渲染器据此把所有对象合并成逐像素参数, 一次着色 —— 避免旧实现
        逐对象全帧 shade 再按最近对象挑选 (O(O×L×N) → O(L×N))。
        """
        raise NotImplementedError

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

    @staticmethod
    def shade_batched(
        emissive: mx.array,
        diff: mx.array,
        spec: mx.array,
        expo: mx.array,
        p: mx.array,
        n: mx.array,
        d: mx.array,
        lights: Sequence,
        ambient: AmbientLight | None = None,
        vis: Sequence | None = None,
    ) -> mx.array:
        """逐像素批量 Blinn-Phong 着色 (emissive/diff/spec 为 (N,3), expo 为
        (N,1))。p/n/d 为 (N,3) 相机空间命中点/法向/射线方向。

        L 与点光源位置均已变换进相机空间; 每灯光方向/衰减由
        light.direction_at 多态给出。vis = 每灯光阴影可见性 (N,) 列表。
        逐对象标量广播与逐像素 gather 在数学上逐位等价 (见 shade_params)。
        """
        v = -d
        out = emissive
        if ambient is not None:
            amb = mx.array(ambient.color.rgb(), dtype=mx.float32) * ambient.intensity
            out = out + mx.broadcast_to(amb, p.shape) * diff
        # N·V 作高光可见门 (掠射角高光消失, 廉价近似)
        ndv = mx.maximum(mx.sum(n * v, axis=-1, keepdims=True), 0.0)
        for i, light in enumerate(lights):
            lc = mx.array(light.color.rgb(), dtype=mx.float32)
            ld, atten = light.direction_at(p)
            nl = mx.maximum(mx.sum(n * ld, axis=-1, keepdims=True), 0.0)
            h = ld + v
            hn = mx.sqrt(mx.sum(h * h, axis=-1, keepdims=True))
            h = h / mx.maximum(hn, 1e-12)  # H=0 (光与视线反向) 时防 NaN
            spec_t = mx.pow(
                mx.maximum(mx.sum(n * h, axis=-1, keepdims=True), 0.0), expo
            )
            contrib = lc * atten * (diff * nl + spec * spec_t * ndv)
            if vis is not None:
                contrib = contrib * vis[i][:, None]  # 阴影可见性
            out = out + contrib
        return out
