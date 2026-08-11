"""接触 demo: 摆锤在重力下摆落, 圆柱碰撞体撞击地面并静止。

运行: uv run python demo_contact.py [输出目录]
输出: <out>/contact_pendulum.gif + 帧 PNG (README 嵌入 docs/contact_pendulum.gif)

物理: cga.contact.ContactModel (惩罚法 f_n = max(0, k·δ − b·v_n) +
库仑摩擦), 摆从 28.6° 释放, 落到接触角 146.4° (bob 底压 z=0 地面),
接触力平衡 m·g·(d_com/L)。渲染走 cga.engine (blade 求交)。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.contact import ContactModel
from cga.dynamics import DynamicsPlant
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
    frame_to_bytes,
)
from cga.motors import Motor
from cga.robot import load_robot

WORLD_UP = Motor.rotor((1.0, 0.0, 0.0), -1.5707963267948966)  # Z-up → Y-up

MODEL = Path(__file__).resolve().parent / "models" / "contact_pendulum.crdf.yaml"


def build_scene(robot, q: list[float]) -> Scene:
    """摆锤场景: 地面 + 铰座 + 摆杆圆柱 (经 WORLD_UP 转 Y-up)。"""
    world = robot.fk_list(q)
    scene = Scene()
    scene.add(
        Mesh(
            PlaneGeometry((0, 1, 0), 0.0),
            MeshStandardMaterial(Color(0x9AA0A6), roughness=0.8),
        ),
        DirectionalLight(intensity=0.6, direction=(0.4, 1.0, 0.35)),
        PointLight(intensity=0.5, position=(1.0, 2.0, 2.0)),
        AmbientLight(intensity=0.4),
    )
    rod = robot.link("rod")
    m_link = WORLD_UP.gp(world["rod"])
    for g in rod.geometry:
        if "visual" not in g.role:
            continue
        scene.add(
            Mesh(
                CylinderGeometry(g.radius, length=g.length),
                MeshStandardMaterial(Color(0xC0392B), roughness=0.4),
                motor=m_link.gp(g.origin),
            )
        )
    # 铰座 (固定, 视觉参照)
    scene.add(
        Mesh(
            BoxGeometry(0.08, 0.08, 0.08),
            MeshStandardMaterial(Color(0x2C3E50), roughness=0.6),
            motor=WORLD_UP.gp(world["base"]).gp(
                Motor.from_matrix(
                    [[1, 0, 0], [0, 1, 0], [0, 0, 1]], (0, 0, 0.5)
                )
            ),
        )
    )
    return scene


def main() -> None:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
    out_dir.mkdir(exist_ok=True)
    robot = load_robot(MODEL)
    plant = DynamicsPlant(robot)
    cm = ContactModel()

    q, qd = [0.5], [0.0]
    dt = 2e-3
    frames: list[list[float]] = []
    step = 0
    while step * dt <= 3.0:
        if step % 50 == 0:
            frames.append(list(q))
        q, qd = plant.integrate(q, qd, cm.generalized_forces(plant, q, qd), dt)
        step += 1
    frames.append(list(q))

    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(1.8, 0.35, 2.2), target=(0.1, 0.25, 0)
    )
    camera.look_at((0.1, 0.25, 0))
    renderer = Renderer(480, 360, aa=2)
    imgs = []
    for i, qq in enumerate(frames):
        img = renderer.render(build_scene(robot, qq), camera)
        imgs.append(Image.frombytes("RGBA", (480, 360), frame_to_bytes(img)))
        imgs[-1].save(out_dir / f"contact_pendulum_{i:03d}.png")
    gif = out_dir / "contact_pendulum.gif"
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=100, loop=0)
    print(f"saved {gif} ({len(imgs)} 帧)")


if __name__ == "__main__":
    main()
