from __future__ import annotations

from pathlib import Path
from typing import Literal

import mlx.core as mx
from PIL import Image

WrapMode = Literal["clamp", "repeat"]


class Texture:
    """Immutable linear RGBA texture sampled on the MLX device.

    Source images are decoded as sRGB and converted to linear RGB before they
    enter the renderer. Texture alpha is retained for future cutout support;
    base-color maps currently use RGB only.
    """

    def __init__(self, pixels: mx.array):
        if len(pixels.shape) != 3 or pixels.shape[-1] != 4:
            raise ValueError("texture pixels must have shape (height, width, 4)")
        if pixels.shape[0] < 1 or pixels.shape[1] < 1:
            raise ValueError("texture dimensions must be positive")
        self.pixels = pixels.astype(mx.float32)
        self.height = int(pixels.shape[0])
        self.width = int(pixels.shape[1])

    @classmethod
    def load(cls, path: str | Path) -> Texture:
        """Load an sRGB PNG/JPEG/WebP image and decode it to linear RGBA."""
        image = Image.open(path).convert("RGBA")
        rgba = (
            mx.array(bytearray(image.tobytes()), dtype=mx.uint8)
            .reshape(image.height, image.width, 4)
            .astype(mx.float32)
            / 255.0
        )
        rgb = rgba[..., :3]
        linear = mx.where(
            rgb <= 0.04045,
            rgb / 12.92,
            mx.power((rgb + 0.055) / 1.055, 2.4),
        )
        return cls(mx.concatenate([linear, rgba[..., 3:4]], axis=-1))

    @classmethod
    def from_rgba(cls, rgba: list[list[list[float]]]) -> Texture:
        """Build a texture from encoded sRGB RGBA floats in the [0, 1] range."""
        encoded = mx.array(rgba, dtype=mx.float32)
        if len(encoded.shape) != 3 or encoded.shape[-1] != 4:
            raise ValueError("rgba must have shape (height, width, 4)")
        rgb = encoded[..., :3]
        linear = mx.where(
            rgb <= 0.04045,
            rgb / 12.92,
            mx.power((rgb + 0.055) / 1.055, 2.4),
        )
        return cls(mx.concatenate([linear, encoded[..., 3:4]], axis=-1))

    @staticmethod
    def _wrap(value: mx.array, mode: WrapMode) -> mx.array:
        if mode == "repeat":
            return value - mx.floor(value)
        if mode == "clamp":
            return mx.clip(value, 0.0, 1.0)
        raise ValueError(f"unsupported texture wrap mode {mode!r}")

    def sample(
        self,
        uv: mx.array,
        wrap_s: WrapMode = "repeat",
        wrap_t: WrapMode = "repeat",
    ) -> mx.array:
        """Sample linearly interpolated RGBA texels for an ``(N, 2)`` UV array."""
        if len(uv.shape) != 2 or uv.shape[1] != 2:
            raise ValueError("uv must have shape (count, 2)")
        u = self._wrap(uv[:, 0], wrap_s)
        v = self._wrap(uv[:, 1], wrap_t)
        # Image rows grow downward, while UV v grows upward.
        x = u * self.width - 0.5
        y = (1.0 - v) * self.height - 0.5
        x0 = mx.floor(x).astype(mx.int32)
        y0 = mx.floor(y).astype(mx.int32)
        fx = (x - x0.astype(mx.float32))[:, None]
        fy = (y - y0.astype(mx.float32))[:, None]
        if wrap_s == "repeat":
            x0 = x0 % self.width
            x1 = (x0 + 1) % self.width
        else:
            x0 = mx.clip(x0, 0, self.width - 1)
            x1 = mx.clip(x0 + 1, 0, self.width - 1)
        if wrap_t == "repeat":
            y0 = y0 % self.height
            y1 = (y0 + 1) % self.height
        else:
            y0 = mx.clip(y0, 0, self.height - 1)
            y1 = mx.clip(y0 + 1, 0, self.height - 1)
        flat = self.pixels.reshape(-1, 4)
        c00 = mx.take(flat, y0 * self.width + x0, axis=0)
        c10 = mx.take(flat, y0 * self.width + x1, axis=0)
        c01 = mx.take(flat, y1 * self.width + x0, axis=0)
        c11 = mx.take(flat, y1 * self.width + x1, axis=0)
        return (c00 * (1.0 - fx) + c10 * fx) * (1.0 - fy) + (
            c01 * (1.0 - fx) + c11 * fx
        ) * fy
