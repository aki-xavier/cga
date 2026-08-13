"""刚体动力学 demo (Drake 物理后端): Z1 计算力矩 PD 折叠 → 舒展。

运行: uv run python demo_dynamics.py [输出目录]
输出: <out>/dynamics_z1.gif + 帧 PNG (README 嵌入 docs/dynamics_z1.gif)

物理: cga.drake.DrakePlant —— pydrake MultibodyPlant (连续查询 +
离散推进)。控制: 计算力矩 τ = M·(Kp·e − Kd·q̇) − Q (Q = 重力广义力,
对齐 EOM M·q̈ + C·v − Q = τ)。渲染: cga engine (robot.fk_list 的
motor 链, 与 Drake FK 交叉校验一致)。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.drake import DrakePlant
from cga.engine import PerspectiveCamera, Renderer
from cga.robot import RobotLoader
from demo_robot import RobotDemo


class DynamicsDemo:
    """计算力矩 PD 控制 (折叠 → 舒展) → GIF。"""

    KP, KD = 50.0, 12.0
    DT = 2e-3
    T_END = 4.0

    @staticmethod
    def main() -> None:
        out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
        out_dir.mkdir(exist_ok=True)
        robot = RobotLoader.load(RobotDemo.MODEL)
        plant = DrakePlant(robot, dt=DynamicsDemo.DT)
        n = plant.plant.num_positions()
        q = [0.0, 0.3, -0.4, 0.2, 0.0, 0.0]
        q_des = [0.3, 2.0944, -2.0944, 0.4, 0.3, 0.5]
        qd = [0.0] * n
        plant.set_joint_state(q, qd)

        camera = PerspectiveCamera(
            fov=50, aspect=4 / 3, position=(0.9, 0.3, 1.6), target=(0.15, 0.4, 0)
        )
        camera.look_at((0.15, 0.4, 0))
        renderer = Renderer(480, 360, aa=2)
        imgs = []
        step = 0
        kp, kd = DynamicsDemo.KP, DynamicsDemo.KD
        while plant.time() <= DynamicsDemo.T_END + 1e-9:
            if step % 100 == 0:
                img = renderer.render(RobotDemo.build_scene(robot, q), camera)
                imgs.append(
                    Image.frombytes("RGBA", (480, 360), Renderer.frame_to_bytes(img))
                )
                imgs[-1].save(out_dir / f"dynamics_z1_{len(imgs) - 1:03d}.png")
            q, qd = plant.joint_state()
            g = plant.gravity_forces(q)
            M = plant.mass_matrix(q)
            e = [q_des[i] - q[i] for i in range(n)]
            acc = [kp * e[i] - kd * qd[i] for i in range(n)]
            tau = [sum(M[i][j] * acc[j] for j in range(n)) - g[i] for i in range(n)]
            plant.step(tau)
            step += 1
        q, _ = plant.joint_state()
        print(f"末态 q = {[round(x, 3) for x in q]}")
        print(f"目标 q = {q_des}")
        gif = out_dir / "dynamics_z1.gif"
        imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=100, loop=0)
        print(f"saved {gif} ({len(imgs)} 帧)")


if __name__ == "__main__":
    DynamicsDemo.main()
