"""积分器 demo: 焊接 Z1 自由自旋翻滚 (RK4 vs 半隐式欧拉 vs 自适应)。

运行: uv run python demo_integrator.py [输出目录]
输出: <out>/integrator_z1.gif + 帧 PNG (README 嵌入 docs/integrator_z1.gif)

物理: 浮动刚体 (weld 全关节) 以 ω=(0,3,0) 起步 —— 中惯量轴旋转的
Dzhanibekov 翻滚 (物理真实: 自由体旋转守恒下自发翻转)。控制台输出
三种积分器 4s 的能量漂移与自适应步数 (固定 2000 vs 自适应 ~500)。

演示点: RK4 能量漂移 ~0.4% ≪ 半隐式 ~2.2%; 自适应同精度少 4× 步数。
"""

import sys
import time
from pathlib import Path

from PIL import Image

from cga.dynamics import DynamicsPlant
from cga.engine import (
    AmbientLight,
    Color,
    CylinderGeometry,
    DirectionalLight,
    Mesh,
    MeshStandardMaterial,
    PerspectiveCamera,
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
    """焊接 Z1 场景 (基座 pose 来自 sim 状态), 无地面 (纯自由飞行)。"""
    world = plant.rigid_fk(q)
    scene = Scene()
    scene.add(
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
            if "visual" not in g.role or g.blade != "cylinder":
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
    q0 = [0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0]
    qd0 = [0.0, 0.0, 0.0, 0.0, 3.0, 0.0]

    def Etot(q, qd):
        return plant.kinetic_energy(q, qd) + plant.potential_energy(q)

    E0 = Etot(q0, qd0)
    t0 = time.perf_counter()
    q, qd = list(q0), list(qd0)
    for _ in range(2000):
        q, qd = plant.integrate(q, qd, [0.0] * 6, 2e-3)
    dt_euler = time.perf_counter() - t0
    t0 = time.perf_counter()
    qr, qdr = list(q0), list(qd0)
    for _ in range(2000):
        qr, qdr = plant.integrate_rk4(qr, qdr, [0.0] * 6, 2e-3)
    dt_rk4 = time.perf_counter() - t0
    t0 = time.perf_counter()
    qa, qda = plant.integrate_adaptive(list(q0), list(qd0), [0.0] * 6, 4.0)
    dt_adapt = time.perf_counter() - t0
    print("4s 自由自旋能量漂移:")
    print(f"  半隐式欧拉 {((Etot(q, qd) - E0) / E0) * 100:+.3f}%  ({dt_euler:.1f}s)")
    print(f"  RK4        {((Etot(qr, qdr) - E0) / E0) * 100:+.3f}%  ({dt_rk4:.1f}s)")
    print(f"  自适应     {((Etot(qa, qda) - E0) / E0) * 100:+.3f}%  ({dt_adapt:.1f}s)")

    # 渲染 (RK4 轨迹, 自适应步长下取等间隔帧)
    frames: list[list[float]] = []
    qf, qdf = list(q0), list(qd0)
    dt = 2e-3
    step = 0
    while step * dt <= 4.0:
        if step % 100 == 0:
            frames.append(list(qf))
        qf, qdf = plant.integrate_rk4(qf, qdf, [0.0] * 6, dt)
        step += 1
    frames.append(list(qf))

    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(0.8, 0.4, 2.4), target=(0, 0.2, 0)
    )
    camera.look_at((0, 0.2, 0))
    renderer = Renderer(480, 360, aa=2)
    imgs = []
    for i, qq in enumerate(frames):
        img = renderer.render(build_scene(robot, plant, qq), camera)
        imgs.append(Image.frombytes("RGBA", (480, 360), frame_to_bytes(img)))
        imgs[-1].save(out_dir / f"integrator_z1_{i:03d}.png")
    gif = out_dir / "integrator_z1.gif"
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=160, loop=0)
    print(f"saved {gif} ({len(imgs)} 帧)")


if __name__ == "__main__":
    main()
