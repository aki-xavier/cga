"""cga 渲染引擎 demo: three.js 风格场景 + 轨道动画 → PNG 帧 + GIF。

运行: uv run python demo_engine.py [帧数] [输出目录]
输出: <out>/frame_%03d.png + <out>/orbit.gif

场景: 地面 + 红/蓝球 + 金柱 + 绿盒 + 紫圆盘, 平行光 + 点光 + 环境光。
"""

import sys
from pathlib import Path

import numpy as np
from PIL import Image

from cga.engine import (
    AmbientLight,
    BoxGeometry,
    CircleGeometry,
    Color,
    CylinderGeometry,
    DirectionalLight,
    Mesh,
    MeshStandardMaterial,
    OrbitControls,
    PerspectiveCamera,
    PlaneGeometry,
    PointLight,
    Renderer,
    Scene,
    SphereGeometry,
)


def build_scene() -> Scene:
    scene = Scene()
    scene.add(
        Mesh(
            PlaneGeometry((0, 1, 0), 0.0),
            # 半光泽地面: roughness 0.7 → Blinn 指数 ~22, 掠射高光聚焦不整片过曝
            MeshStandardMaterial(Color(0xB0B0B0), roughness=0.7),
        ),
        # v1 无 envMap/IBL: 高 metalness 会黑死 (three.js 同样问题)。
        # 压低 metalness, 保留金属色高光, 让漫反射色可读。
        Mesh(
            SphereGeometry(1.0),
            MeshStandardMaterial(Color(0xC0392B), roughness=0.25, metalness=0.25),
            position=(0, 1, 0),
        ),
        Mesh(
            SphereGeometry(0.6),
            MeshStandardMaterial(Color(0x2980B9), roughness=0.15, metalness=0.35),
            position=(-2.2, 0.6, 0.5),
        ),
        Mesh(
            CylinderGeometry(0.7),
            MeshStandardMaterial(Color(0xD4AC0D), roughness=0.4, metalness=0.3),
            position=(2.2, 0.7, -0.5),
        ),
        Mesh(
            BoxGeometry(0.9, 0.9, 0.9),
            MeshStandardMaterial(Color(0x27AE60), roughness=0.6),
            position=(0.8, 0.45, 1.8),
        ),
        Mesh(
            CircleGeometry(0.9),
            MeshStandardMaterial(Color(0x8E44AD), roughness=0.3),
            position=(-2.4, 2.2, 0.8),
            rotation_axis=(1, 0, 0),
            rotation_angle=-0.4,
        ),
        DirectionalLight(intensity=0.38, direction=(0.4, 1.0, 0.35)),  # 主光
        PointLight(intensity=0.7, position=(0, 4, 3.5)),  # 顶面点缀
        DirectionalLight(intensity=0.18, direction=(0, 0.35, 0.9)),  # 正面补光
        AmbientLight(intensity=0.34),  # 环境底光: 金属暗面可读
    )
    return scene


def main() -> None:
    frames = int(sys.argv[1]) if len(sys.argv) > 1 else 90
    out_dir = Path(sys.argv[2] if len(sys.argv) > 2 else "artifacts")
    out_dir.mkdir(exist_ok=True)

    scene = build_scene()
    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(0, 2.4, 6.2), target=(0, 0.8, 0)
    )
    camera.look_at((0, 0.8, 0))
    controls = OrbitControls(camera, target=(0, 0.8, 0), radius=6.6, elevation=0.42)
    renderer = Renderer(360, 270)

    paths: list[Path] = []
    for i in range(frames):
        controls.azimuth = 2.0 * 3.141592653589793 * i / frames  # 绕一圈
        controls.elevation = 0.42 + 0.12 * np.sin(4.0 * 3.141592653589793 * i / frames)
        controls.update()
        img = renderer.render(scene, camera)
        arr = np.asarray(img.tolist(), dtype=np.uint8)
        p = out_dir / f"frame_{i:03d}.png"
        Image.fromarray(arr).save(p)
        paths.append(p)
        print(f"frame {i + 1}/{frames} saved {p}", end="\r")
    print()

    gif = out_dir / "orbit.gif"
    imgs = [Image.open(p) for p in paths]
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=50, loop=0)
    print(f"gif: {gif} ({len(paths)} frames, {gif.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
