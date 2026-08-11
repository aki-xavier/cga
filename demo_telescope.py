"""prismatic 关节 demo: 伸缩臂 (revolute 转台 + prismatic 吊臂) 计算力矩 PD。

运行: uv run python demo_telescope.py [输出目录]
输出: <out>/telescope.gif + 帧 PNG (README 嵌入 docs/telescope.gif)

物理: DynamicsPlant 的 prismatic 支持 —— FK 平移 (M·Trans(axis·q)),
雅可比平动列 (J_v = axis, J_ω = 0), RNEA 平动分支 (加速度含
q̈ + 2·ω×s·q̇ 运动轴 Coriolis), 反向力沿轴投影。吊臂随转台旋转
同时伸缩 (耦合: 旋转离心对滑动有耦合项)。
控制: 计算力矩 τ = g + M·(Kp·e − Kd·q̇), 闭环 ë + Kd·ė + Kp·e = 0。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.dynamics import DynamicsPlant
from cga.engine import PerspectiveCamera, Renderer, frame_to_bytes
from cga.robot import load_robot
from demo_robot import build_scene

MODEL = Path(__file__).resolve().parent / "models" / "telescope.crdf.yaml"
KP, KD = 40.0, 11.0
DT = 2e-3
T_END = 4.0


def main() -> None:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
    out_dir.mkdir(exist_ok=True)
    robot = load_robot(MODEL)
    plant = DynamicsPlant(robot)
    q = [0.0, 0.05]
    q_des = [2.4, 0.35]
    qd = [0.0] * 2
    n = plant.nq
    frames = []
    step = 0
    while step * DT <= T_END:
        if step % 100 == 0:
            frames.append(list(q))
        g = plant.gravity_forces(q)
        M = plant.mass_matrix(q)
        e = [q_des[i] - q[i] for i in range(n)]
        acc = [KP * e[i] - KD * qd[i] for i in range(n)]
        tau = [sum(M[i][j] * acc[j] for j in range(n)) - g[i] for i in range(n)]
        q, qd = plant.integrate(q, qd, tau, DT)
        step += 1
    frames.append(list(q))
    print(f"末态: yaw={q[0]:.3f} (目标 {q_des[0]}) slide={q[1]:.3f} (目标 {q_des[1]})")

    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(0.9, 0.7, 1.6), target=(0.1, 0.2, 0)
    )
    camera.look_at((0.1, 0.2, 0))
    renderer = Renderer(480, 360, aa=2)
    imgs = []
    for i, qq in enumerate(frames):
        img = renderer.render(build_scene(robot, qq), camera)
        imgs.append(Image.frombytes("RGBA", (480, 360), frame_to_bytes(img)))
        imgs[-1].save(out_dir / f"telescope_{i:03d}.png")
    gif = out_dir / "telescope.gif"
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=100, loop=0)
    print(f"saved {gif} ({len(imgs)} 帧)")


if __name__ == "__main__":
    main()
