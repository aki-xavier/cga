"""浮动基座 demo: 焊接 Z1 (整机刚体) 自由落体 + 地面接触反弹。

运行: uv run python demo_floating.py [输出目录]
输出: <out>/floating_z1.gif + 帧 PNG (README 嵌入 docs/floating_z1.gif)

物理: DynamicsPlant(floating_base=True, weld=全部关节) —— 浮动 6-DOF
(四元数位姿 + 速度状态) + 焊死关节 (Drake WeldFrames 语义) = 刚体;
隐式接触 (ContactModel.integrate_implicit, 速度级脉冲 + 位置修正)
处理撞击 —— 惩罚弹簧冲击失稳, 此法稳定。

场景: 折叠下垂位 (COM 最低) 从 0.5m 自由落体 —— 臂尖先着地,
机体前倾、基座落地, 短暂静止后 (头重脚轻亚稳态) 倾倒翻滚 ——
这是真实物理 (顶部沉重的机械臂无法自稳), 与摆锤接触 demo
(可静止) 形成对照。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.contact import ContactModel
from cga.dynamics import DynamicsPlant
from cga.engine import (
    AmbientLight,
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

MODEL = Path(__file__).resolve().parent / "models" / "z1_arm.crdf.yaml"
WELD = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def build_scene(robot, plant, q: list[float]) -> Scene:
    """焊接 Z1 场景: 地面 + 各 link 视觉几何 (基座 pose 来自 sim 状态)。"""
    world = plant.rigid_fk(q)
    scene = Scene()
    scene.add(
        Mesh(
            PlaneGeometry((0, 1, 0), 0.0),
            MeshStandardMaterial(Color(0x9AA0A6), roughness=0.8),
        ),
        DirectionalLight(intensity=0.6, direction=(0.4, 1.0, 0.35)),
        PointLight(intensity=0.5, position=(0, 3.5, 2.8)),
        AmbientLight(intensity=0.42),
    )
    mats = {m.name: m.color[:3] for m in robot.materials}
    for lnk in robot.links:
        if lnk.name not in world:
            continue
        R, t = world[lnk.name]
        m_link = Motor.from_matrix(R, t)
        for g in lnk.geometry:
            if "visual" not in g.role:
                continue
            if g.blade != "cylinder":
                continue
            rgba = mats.get(g.material or "", (0.7, 0.7, 0.7))
            scene.add(
                Mesh(
                    CylinderGeometry(g.radius, length=g.length),
                    MeshStandardMaterial(
                        Color(rgba[0], rgba[1], rgba[2]), roughness=0.45
                    ),
                    motor=m_link.gp(g.origin),
                )
            )
    return scene


def main() -> None:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
    out_dir.mkdir(exist_ok=True)
    robot = load_robot(MODEL)
    plant = DynamicsPlant(robot, floating_base=True, weld=WELD)
    contact = ContactModel()

    q = [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
    qd = [0.0] * 6
    dt = 2e-3
    frames: list[list[float]] = []
    step = 0
    while step * dt <= 1.6:
        if step % 50 == 0:
            frames.append(list(q))
        q, qd = contact.integrate_implicit(plant, q, qd, [0.0] * 6, dt)
        step += 1
    frames.append(list(q))

    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(0.8, 0.4, 2.4), target=(0, 0.2, 0)
    )
    camera.look_at((0, 0.2, 0))
    renderer = Renderer(480, 360, aa=2)
    imgs = []
    for i, qq in enumerate(frames):
        img = renderer.render(build_scene(robot, plant, qq), camera)
        imgs.append(Image.frombytes("RGBA", (480, 360), frame_to_bytes(img)))
        imgs[-1].save(out_dir / f"floating_z1_{i:03d}.png")
    gif = out_dir / "floating_z1.gif"
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=100, loop=0)
    print(f"saved {gif} ({len(imgs)} 帧)")


if __name__ == "__main__":
    main()
