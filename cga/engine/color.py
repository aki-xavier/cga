"""sRGB 颜色 (线性空间解码在构造侧, 编码在 Renderer 输出端)。"""


class Color:
    """sRGB 颜色: Color(0xRRGGBB) 或 Color(r, g, b) (0-1 sRGB 编码值)。

    rgb() 返回线性空间分量 (此处解码); 光照/着色全在线性空间进行,
    Renderer 输出端统一重新编码 → roundtrip 恒等 (自检覆盖)。
    """

    __slots__ = ("r", "g", "b")

    def __init__(self, r: float, g: float | None = None, b: float | None = None):
        if g is None and b is None:
            c = int(r)
            self.r, self.g, self.b = (
                ((c >> 16) & 0xFF) / 255.0,
                ((c >> 8) & 0xFF) / 255.0,
                (c & 0xFF) / 255.0,
            )
        else:
            self.r, self.g, self.b = float(r), float(g or 0.0), float(b or 0.0)

    @staticmethod
    def srgb_to_linear(c: float) -> float:
        """sRGB 编码值 → 线性 (IEC 61966-2-1 分段曲线)。"""
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4

    def rgb(self) -> tuple[float, float, float]:
        """线性空间分量 (着色用, 非编码值)。"""
        return (
            Color.srgb_to_linear(self.r),
            Color.srgb_to_linear(self.g),
            Color.srgb_to_linear(self.b),
        )
