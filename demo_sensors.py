"""传感器 demo: Z1 腕部挂载 0.8kg 负载, 计算力矩 PD 摆动 + 力/力矩传感器。

运行: uv run python demo_sensors.py [输出目录]
输出: <out>/sensors_z1.gif + 帧 PNG (README 嵌入 docs/sensors_z1.gif)

API: cga.sensors —— 移植 Drake 传感器/驱动器语义:
- ForceTorqueSensor(plant, link, origin): 6 轴 F/T, 读数 = RNEA 反推
  的子树支撑力传播到传感器帧 (世界 → 传感器局部坐标)。
- JointActuator(plant, joint, effort_limit): 力矩饱和。
- JointStateSensor: 关节 (q, q̇, τ) 读数。

场景: 基座 F/T (测整臂反应力) + 腕部 F/T (测负载+腕部), 摆臂过程中
实时读数 —— 静止时 = 重力 (43N 整臂 / 负载+腕部 ~11N), 摆动时叠加
惯性力。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.dynamics import DynamicsPlant
from cga.engine import PerspectiveCamera, Renderer, frame_to_bytes
from cga.robot import Geometry, Inertial, Joint, Link, Robot, load_robot
from cga.sensors import ForceTorqueSensor
from demo_robot import MODEL, build_scene

KP, KD = 30.0, 9.0
DT = 2e-3
T_END = 3.0
PAYLOAD_MASS = 0.8  # kg (腕部抓取物)


def with_payload(robot: Robot) -> Robot:
    """在腕部 (link06) 挂一个 0.8kg 圆柱负载。"""
    flange = Motor_translator(0.0, 0.0, 0.0)
    payload = Link(
        name="payload",
        inertial=Inertial(
            mass=PAYLOAD_MASS,
            com=(0.0, 0.0, 0.04),
            ixx=0.0004,
            iyy=0.0004,
            izz=0.0002,
        ),
        geometry=(
            Geometry(
                blade="cylinder",
                radius=0.03,
                length=0.08,
                origin=flange,
                role=("visual", "collision"),
                material="dark",
            ),
        ),
    )
    attach = Joint(
        name="grip", type="fixed", parent="link06", child="payload", origin=flange
    )
    return Robot(
        robot.name,
        robot.base,
        robot.links + (payload,),
        robot.joints + (attach,),
        robot.materials,
    )


def Motor_translator(x, y, z):
    from cga.motors import Motor

    return Motor.translator((x, y, z))


def main() -> None:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
    out_dir.mkdir(exist_ok=True)
    robot = with_payload(load_robot(MODEL))
    plant = DynamicsPlant(robot)
    n = plant.nq
    # 基座 F/T (link00 原点) + 腕部 F/T (link05 原点, 传感器沿 link05)
    fts_base = ForceTorqueSensor(plant, "link00", name="base_fts")
    fts_wrist = ForceTorqueSensor(plant, "link05", name="wrist_fts")

    q = [0.0, 0.3, -0.4, 0.2, 0.0, 0.0]
    q_des = [0.5, 1.8, -1.6, 0.5, 0.4, 0.6]
    qd = [0.0] * n
    frames = []
    step = 0
    print(f"{'t':>5} | 基座F/T z | 腕部F/T |F| | 负载重量 {PAYLOAD_MASS*9.81:.1f}N")
    while step * DT <= T_END:
        if step % 100 == 0:
            frames.append(list(q))
            g = plant.gravity_forces(q)
            Mm = plant.mass_matrix(q)
            e = [q_des[i] - q[i] for i in range(n)]
            acc = [KP * e[i] - KD * qd[i] for i in range(n)]
            tau = [g[i] + sum(Mm[i][j] * acc[j] for j in range(n)) for i in range(n)]
            qdd = plant.forward_dynamics(q, qd, tau)
            f_b, _ = fts_base.read(q, qd, qdd)
            f_w, _ = fts_wrist.read(q, qd, qdd)
            fw_mag = (f_w[0] ** 2 + f_w[1] ** 2 + f_w[2] ** 2) ** 0.5
            print(f"{step*DT:5.2f} |   {f_b[2]:6.1f} |  {fw_mag:6.2f}")
        else:
            g = plant.gravity_forces(q)
            Mm = plant.mass_matrix(q)
            e = [q_des[i] - q[i] for i in range(n)]
            acc = [KP * e[i] - KD * qd[i] for i in range(n)]
            tau = [g[i] + sum(Mm[i][j] * acc[j] for j in range(n)) for i in range(n)]
        q, qd = plant.integrate(q, qd, tau, DT)
        step += 1
    frames.append(list(q))

    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(0.9, 0.3, 1.6), target=(0.15, 0.4, 0)
    )
    camera.look_at((0.15, 0.4, 0))
    renderer = Renderer(480, 360, aa=2)
    imgs = []
    for i, qq in enumerate(frames):
        img = renderer.render(build_scene(robot, qq), camera)
        imgs.append(Image.frombytes("RGBA", (480, 360), frame_to_bytes(img)))
        imgs[-1].save(out_dir / f"sensors_z1_{i:03d}.png")
    gif = out_dir / "sensors_z1.gif"
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=100, loop=0)
    print(f"saved {gif} ({len(imgs)} 帧)")


if __name__ == "__main__":
    main()
