"""刚体动力学 demo: Z1 从折叠姿态经计算力矩 PD 平滑升起到舒展姿态。

运行: uv run python demo_dynamics.py [输出目录]
输出: <out>/dynamics_z1.gif + 帧 PNG (README 嵌入 docs/dynamics_z1.gif)

API: cga.dynamics.DynamicsPlant —— 移植 Drake MultibodyPlant 的子集:
    mass_matrix / gravity_forces / coriolis_forces / inverse_dynamics /
    forward_dynamics / integrate (半隐式欧拉 + 限位 + CRDF 阻尼)。
控制: 计算力矩 (反馈线性化) τ = g(q) + M(q)·(Kp·e − Kd·q̇), 闭环
    ë + Kd·ė + Kp·e = 0 —— 增益与各关节惯量无关 (腕部轻关节不炸)。
CGA 角色: FK 走 motor 链 (robot.fk), link 空间速度 = twist 二重向量
    (plant.link_twists); 动力学热路径用 (R,t) 刚性链 (等价矩阵形式)。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.dynamics import DynamicsPlant
from cga.engine import (
    PerspectiveCamera,
    Renderer,
    frame_to_bytes,
)
from cga.robot import load_robot
from demo_robot import MODEL, build_scene

KP, KD = 50.0, 12.0  # 计算力矩增益 (闭环 ë + Kd·ė + Kp·e = 0)
DT = 2e-3
T_END = 4.0


def simulate(plant: DynamicsPlant) -> list[list[float]]:
    """计算力矩 PD: 折叠 → 舒展目标姿态, 返回每帧时刻的 q。"""
    q = [0.0, 0.3, -0.4, 0.2, 0.0, 0.0]
    q_des = [0.3, 2.0944, -2.0944, 0.4, 0.3, 0.5]
    qd = [0.0] * 6
    n = plant.nq
    frames = []
    step = 0
    while step * DT <= T_END:
        if step % 100 == 0:  # 每 0.2s 一帧
            frames.append(list(q))
        g = plant.gravity_forces(q)
        M = plant.mass_matrix(q)
        e = [q_des[i] - q[i] for i in range(n)]
        acc = [KP * e[i] - KD * qd[i] for i in range(n)]
        tau = [g[i] + sum(M[i][j] * acc[j] for j in range(n)) for i in range(n)]
        q, qd = plant.integrate(q, qd, tau, DT)
        step += 1
    frames.append(list(q))
    return frames


def main() -> None:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
    out_dir.mkdir(exist_ok=True)
    robot = load_robot(MODEL)
    plant = DynamicsPlant(robot)
    frames = simulate(plant)

    scene = None
    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(0.9, 0.3, 1.6), target=(0.15, 0.4, 0)
    )
    camera.look_at((0.15, 0.4, 0))
    renderer = Renderer(480, 360, aa=2)
    imgs = []
    for i, q in enumerate(frames):
        scene = build_scene(robot, q)
        img = renderer.render(scene, camera)
        imgs.append(Image.frombytes("RGBA", (480, 360), frame_to_bytes(img)))
        p = out_dir / f"dynamics_z1_{i:03d}.png"
        imgs[-1].save(p)
    gif = out_dir / "dynamics_z1.gif"
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=200, loop=0)
    print(f"saved {gif} ({len(imgs)} 帧)")


if __name__ == "__main__":
    main()
