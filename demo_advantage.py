"""渲染示例: 三个独立图像, 分别展示 README 提到的三大优势。

运行: uv run python demo_advantage.py [输出目录]
输出: <out>/advantage_a.png 无多边形 — 球/圆柱是解析 blade, 相机贴脸 (≈2.7×
      半径) 时轮廓仍是完美圆弧、高光无棱角; 网格渲染在同等距离已露三角面片。
     <out>/advantage_b.png 无限几何 — 无限平面延伸到地平线, 无限圆柱无端盖,
      无需裁剪; 红球作尺度参照 (底切正好在地面)。
     <out>/advantage_c.png 变换与几何同构 — 同一个 Motor 沿
      exp(s·log(M0⁻¹M1)) 插值 (SE(3) 螺旋), 驱动路径上每一个图元:
      小球轨迹 + 终点盒的旋转, 无矩阵分解。
"""

import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from cga.engine import (
    AmbientLight,
    BoxGeometry,
    Color,
    CylinderGeometry,
    DirectionalLight,
    Mesh,
    MeshStandardMaterial,
    PerspectiveCamera,
    PlaneGeometry,
    PointLight,
    Renderer,
    Scene,
    SphereGeometry,
    frame_to_bytes,
)
from cga.motors import Motor

W, H = 400, 300


def _render(
    scene: Scene,
    cam_pos: tuple[float, float, float],
    target: tuple[float, float, float],
) -> bytes:
    """静态视角渲染一帧 → RGBA bytes (PIL 输出桥, 无 numpy)。

    aa=2 超采样抗锯齿: 每像素 2×2 亚像素射线平均, 轮廓/高光边缘更平滑。
    """
    camera = PerspectiveCamera(fov=50, aspect=W / H, position=cam_pos, target=target)
    camera.look_at(target)
    img = Renderer(W, H, aa=2).render(scene, camera)
    return frame_to_bytes(img)


def panel_a_scene() -> Scene:
    """无多边形: 相机贴脸大球 + 无限圆柱, 光滑轮廓/高光弧。"""
    scene = Scene(background=Color(0x101418))
    scene.add(
        Mesh(
            SphereGeometry(1.2),
            MeshStandardMaterial(Color(0xC0392B), roughness=0.18, metalness=0.2),
            position=(0, 0.15, 0),
        ),
        Mesh(
            CylinderGeometry(0.4),
            MeshStandardMaterial(Color(0xD4AC0D), roughness=0.3, metalness=0.25),
            position=(1.55, 0.15, 0.1),
        ),
        DirectionalLight(intensity=0.5, direction=(0.3, 0.9, 0.4)),
        PointLight(intensity=0.9, position=(0.6, 2.0, 1.5)),
        AmbientLight(intensity=0.25),
    )
    return scene


def panel_b_scene() -> Scene:
    """无限几何: 地面延伸到地平线, 圆柱无端盖无限延伸。"""
    scene = Scene()
    scene.add(
        Mesh(
            PlaneGeometry((0, 1, 0), 0.0),  # 平面 y = 0, 直达地平线
            MeshStandardMaterial(Color(0xB0B0B0), roughness=0.75),
        ),
        Mesh(
            CylinderGeometry(0.5),  # 无限圆柱, 无顶盖/底盖
            MeshStandardMaterial(Color(0xD4AC0D), roughness=0.35, metalness=0.2),
            position=(1.3, 0, -0.5),
        ),
        Mesh(  # 红球放在地面作尺度参照 (底切 y=0)
            SphereGeometry(0.8),
            MeshStandardMaterial(Color(0xC0392B), roughness=0.3),
            position=(-1.4, 0.8, 0.6),
        ),
        DirectionalLight(intensity=0.42, direction=(0.4, 1.0, 0.35)),
        PointLight(intensity=0.5, position=(0, 3, 2.5)),
        AmbientLight(intensity=0.32),
    )
    return scene


def panel_c_scene() -> Scene:
    """变换与几何同构: 一个 motor 沿 exp(s·log) 插值, 驱动所有图元。"""
    scene = Scene(background=Color(0x101418))
    m0 = Motor((0, 1, 0), -0.5, (-2.2, 0.75, -0.6))  # 起点 pose
    m1 = Motor((0, 1, 0), 1.1, (2.0, 0.75, 1.0))  # 终点 pose
    # 路径轨迹: 同一个插值 motor 直接作用到小球 blade (m0.interpolate(m1, s))
    for i in range(6):
        m = m0.interpolate(m1, i / 5)
        scene.add(
            Mesh(
                SphereGeometry(0.16),
                MeshStandardMaterial(Color(0x95A5A6), roughness=0.6),
                motor=m,
            )
        )
    scene.add(
        Mesh(  # 终点盒: 旋转已沿路径平滑插值, 非纯平移
            BoxGeometry(0.65, 0.65, 0.65),
            MeshStandardMaterial(Color(0x27AE60), roughness=0.55),
            motor=m1,
        ),
        DirectionalLight(intensity=0.45, direction=(0.4, 1.0, 0.35)),
        PointLight(intensity=0.6, position=(-1.0, 2.5, 3.0)),
        AmbientLight(intensity=0.38),
    )
    return scene


def _label_font() -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    try:
        return ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", 15)
    except OSError:
        return ImageFont.load_default()


def _save_panel(
    name: str,
    scene: Scene,
    cam_pos: tuple[float, float, float],
    target: tuple[float, float, float],
    label: str,
    out_dir: Path,
) -> Path:
    """渲染一帧 + 底部说明文字 → 独立 PNG。"""
    frame = _render(scene, cam_pos, target)
    canvas = Image.new("RGB", (W, H + 26), (16, 18, 22))
    canvas.paste(Image.frombytes("RGBA", (W, H), frame).convert("RGB"), (0, 0))
    draw = ImageDraw.Draw(canvas)
    draw.text((8, H + 6), label, fill=(232, 232, 232), font=_label_font())
    p = out_dir / name
    canvas.save(p)
    print(f"saved {p}")
    return p


def main() -> None:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
    out_dir.mkdir(exist_ok=True)

    _save_panel(
        "advantage_a.png",
        panel_a_scene(),
        (0, 0.15, 3.2),
        (0, 0.15, 0),
        "implicit blades: close-up, no facets",
        out_dir,
    )
    _save_panel(
        "advantage_b.png",
        panel_b_scene(),
        (0, 1.6, 5.5),
        (0, 0.7, 0),
        "infinite plane + cylinder, no clipping",
        out_dir,
    )
    _save_panel(
        "advantage_c.png",
        panel_c_scene(),
        (0.2, 2.6, 5.8),
        (0, 0.75, 0.2),
        "motor exp(s·log) drives every primitive",
        out_dir,
    )


if __name__ == "__main__":
    main()
