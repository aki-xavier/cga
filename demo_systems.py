"""systems 框架 demo: Z1 + 腕部负载, 轨迹→PD→plant→F/T 图组合仿真。

运行: uv run python demo_systems.py [输出目录]
输出: <out>/systems_z1.gif + 帧 PNG (README 嵌入 docs/systems_z1.gif)

API: cga.systems —— 移植 Drake Diagram/ports 语义:
    System (端口+状态+step) / Diagram (连线+拓扑序推进) /
    Simulator (驱动循环 + 端口记录 tracer)。
组合: TrajectorySource(5次多项式目标) → PidController(计算力矩, 一拍
延迟读 plant 状态) → DynamicsSystem(plant) → FtsSystem(基座+腕部 F/T)。
反馈环 (plant→controller) 走一拍延迟 —— 离散控制标准语义, 非代数环。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.dynamics import DynamicsPlant
from cga.engine import PerspectiveCamera, Renderer, frame_to_bytes
from cga.robot import load_robot
from cga.sensors import ForceTorqueSensor
from cga.systems import (
    Diagram,
    DynamicsSystem,
    FtsSystem,
    PidController,
    Simulator,
    TrajectorySource,
)
from demo_robot import MODEL, build_scene
from demo_sensors import with_payload

DT = 2e-3
T_END = 3.0


def main() -> None:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
    out_dir.mkdir(exist_ok=True)
    robot = with_payload(load_robot(MODEL))
    plant = DynamicsPlant(robot)
    n = plant.nq

    diag = Diagram()
    i_traj = diag.add(TrajectorySource([0.0, 0.3, -0.4, 0.2, 0.0, 0.0],
                                       [0.5, 1.8, -1.6, 0.5, 0.4, 0.6], 1.5))
    i_pid = diag.add(PidController(plant, kp=30.0, kd=9.0))
    i_plant = diag.add(DynamicsSystem(plant, q=[0.0, 0.3, -0.4, 0.2, 0.0, 0.0],
                                      qd=[0.0] * n))
    i_fts_b = diag.add(FtsSystem(ForceTorqueSensor(plant, "link00")))
    i_fts_w = diag.add(FtsSystem(ForceTorqueSensor(plant, "link05")))
    diag.connect(i_traj, "q_des", i_pid, "q_des")
    diag.connect(i_plant, "state", i_pid, "state")
    diag.connect(i_pid, "tau", i_plant, "tau")
    diag.connect(i_plant, "state", i_fts_b, "state")
    diag.connect(i_plant, "qdd", i_fts_b, "qdd")
    diag.connect(i_plant, "state", i_fts_w, "state")
    diag.connect(i_plant, "qdd", i_fts_w, "qdd")

    sim = Simulator(
        diag, dt=DT,
        trace_ports=[(i_plant, "state"), (i_fts_b, "fts"), (i_fts_w, "fts")],
    )
    traces = sim.advance_to(T_END)
    print("图组合仿真完成:")
    fb = traces[f"{i_fts_b}:fts"][-1][0]
    fw = traces[f"{i_fts_w}:fts"][-1][0]
    print(f"  基座 F/T z = {fb[2]:.1f}N (期望 整臂+负载 51.2N)")
    print(f"  腕部 F/T |F| = {(fw[0]**2+fw[1]**2+fw[2]**2)**0.5:.1f}N (期望 ~14.2N)")

    # 渲染 (从 plant 的逐帧状态)
    frames = [s[0] for s in traces[f"{i_plant}:state"][::50]]
    frames.append(traces[f"{i_plant}:state"][-1][0])
    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(0.9, 0.3, 1.6), target=(0.15, 0.4, 0)
    )
    camera.look_at((0.15, 0.4, 0))
    renderer = Renderer(480, 360, aa=2)
    imgs = []
    for i, qq in enumerate(frames):
        img = renderer.render(build_scene(robot, qq), camera)
        imgs.append(Image.frombytes("RGBA", (480, 360), frame_to_bytes(img)))
        imgs[-1].save(out_dir / f"systems_z1_{i:03d}.png")
    gif = out_dir / "systems_z1.gif"
    imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=100, loop=0)
    print(f"saved {gif} ({len(imgs)} 帧)")


if __name__ == "__main__":
    main()
