"""示例纹理资产生成器: .venv/bin/python examples/assets/gen_textures.py

生成 brick.png (砖墙, 灰缝错缝) 与 treadplate.png (金属花纹板),
均为 256x256 可平铺图案, 供 .cgs 的 material(map="assets/*.png") 使用。
"""

from pathlib import Path

from PIL import Image, ImageDraw

OUT = Path(__file__).parent


def brick(size: int = 256) -> None:
    """错缝砖墙: 8 行 × 4 列砖块, 深色灰缝。"""
    img = Image.new("RGB", (size, size), (0x9A, 0x93, 0x88))  # 灰缝
    d = ImageDraw.Draw(img)
    rows, cols = 8, 4
    bh, bw = size // rows, size // cols
    mortar = 3
    for r in range(rows):
        offset = (bw // 2) if r % 2 else 0
        for c in range(-1, cols + 1):
            x0 = c * bw + offset + mortar // 2
            y0 = r * bh + mortar // 2
            x1 = x0 + bw - mortar
            y1 = y0 + bh - mortar
            # 砖色轻微交错 ( deterministic )
            shade = 0xB8 + ((r * 7 + c * 13) % 3 - 1) * 8
            d.rectangle([x0, y0, x1, y1], fill=(shade, 0x74, 0x62))
    img.save(OUT / "brick.png")


def treadplate(size: int = 256) -> None:
    """金属花纹板: 斜向菱形凸点阵列。"""
    img = Image.new("RGB", (size, size), (0x6E, 0x74, 0x7A))
    d = ImageDraw.Draw(img)
    step = 32
    for y in range(0, size, step):
        for x in range(0, size, step):
            cx, cy = x + step // 2, y + step // 2
            if (x // step + y // step) % 2:
                d.line([cx - 10, cy, cx + 10, cy], fill=(0x8A, 0x90, 0x96), width=5)
                d.line(
                    [cx - 10, cy + 2, cx + 10, cy + 2], fill=(0x54, 0x5A, 0x60), width=2
                )
            else:
                d.line([cx, cy - 10, cx, cy + 10], fill=(0x8A, 0x90, 0x96), width=5)
                d.line(
                    [cx + 2, cy - 10, cx + 2, cy + 10], fill=(0x54, 0x5A, 0x60), width=2
                )
    img.save(OUT / "treadplate.png")


if __name__ == "__main__":
    brick()
    treadplate()
    print(f"saved {OUT}/brick.png, {OUT}/treadplate.png")
