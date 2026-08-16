"""CGS 渲染入口: uv run python -m cga.scene_lang <file.cgs> [out.png] [宽 高 aa]"""

import sys
from pathlib import Path

from PIL import Image

from cga.engine import Renderer
from cga.scene_lang import SceneLoader


class CgsCli:
    """CGS 文件 → PNG (Renderer + PIL 桥)。"""

    @staticmethod
    def main() -> None:
        src = Path(sys.argv[1])
        out = Path(sys.argv[2]) if len(sys.argv) > 2 else src.with_suffix(".png")
        w = int(sys.argv[3]) if len(sys.argv) > 3 else 640
        h = int(sys.argv[4]) if len(sys.argv) > 4 else 480
        aa = int(sys.argv[5]) if len(sys.argv) > 5 else 2
        scene, camera = SceneLoader.load(
            src.read_text(encoding="utf-8"), asset_root=src.parent
        )
        camera.aspect = w / h
        img = Renderer(w, h, aa=aa).render(scene, camera)
        Image.frombytes("RGBA", (w, h), Renderer.frame_to_bytes(img)).save(out)
        print(f"saved {out}")


if __name__ == "__main__":
    CgsCli.main()
