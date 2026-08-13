"""回弹小球 demo (Drake 物理后端): 球从 1m 高处掉落, 触地静止。

运行: uv run python demo_bounce.py [输出目录]
输出: <out>/bounce_ball.gif + 帧 PNG (README 嵌入 docs/bounce_ball.gif)

物理: cga.drake.DrakePlant —— 直接接入 pydrake MultibodyPlant (离散
plant, 内置接触求解器): 浮动基座自由落体 → 点接触 (默认耗散, 真实
物理: 球触地静止在 z=0.06)。渲染: cga engine (WORLD_UP 变换)。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.drake import DrakePlant
from cga.engine import (
    AmbientLight,
    Color,
    DirectionalLight,
    Mesh,
    MeshStandardMaterial,
    PerspectiveCamera,
    PlaneGeometry,
    PointLight,
    Renderer,
    Scene,
    SphereGeometry,
)
from cga.motors import Motor
from cga.robot import RobotLoader
from demo_robot import RobotDemo


class BounceDemo:
    """自由落体小球 + 接触回弹 (离散 plant) → GIF。"""

    MODEL = Path(__file__).resolve().parent / "models" / "bounce_ball.crdf.yaml"
    DT = 2e-3
    T_END = 2.0

    @staticmethod
    def build_scene(robot, plant: DrakePlant) -> Scene:
        """球 + 地面 (球位姿来自 Drake 状态, WORLD_UP 转渲染系)。"""
        scene = Scene()
        scene.add(
            Mesh(
                PlaneGeometry((0, 1, 0), 0.0),
                MeshStandardMaterial(Color(0x9AA0A6), roughness=0.8),
            ),
            DirectionalLight(intensity=0.65, direction=(0.4, 1.0, 0.35)),
            PointLight(intensity=0.6, position=(0, 3.5, 2.8)),
            AmbientLight(intensity=0.52),
        )
        R, t = plant.poses()["body"]
        m_link = RobotDemo.WORLD_UP.gp(Motor.from_matrix(R, t))
        for g in robot.link("body").geometry:
            if "visual" not in g.role:
                continue
            scene.add(
                Mesh(
                    SphereGeometry(g.radius),
                    MeshStandardMaterial(Color(0.95, 0.35, 0.1), roughness=0.35),
                    motor=m_link.gp(g.origin),
                )
            )
        return scene

    @staticmethod
    def main() -> None:
        out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
        out_dir.mkdir(exist_ok=True)
        robot = RobotLoader.load(BounceDemo.MODEL)
        plant = DrakePlant(robot, floating_base=True, dt=BounceDemo.DT)
        plant.set_base_pose((0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0))
        plant.set_base_twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

        camera = PerspectiveCamera(
            fov=50, aspect=4 / 3, position=(0.9, 0.6, 1.8), target=(0, 0.35, 0)
        )
        camera.look_at((0, 0.35, 0))
        renderer = Renderer(480, 360, aa=2)
        imgs = []
        step = 0
        while plant.time() <= BounceDemo.T_END + 1e-9:
            if step % 50 == 0:
                img = renderer.render(BounceDemo.build_scene(robot, plant), camera)
                imgs.append(
                    Image.frombytes("RGBA", (480, 360), Renderer.frame_to_bytes(img))
                )
                imgs[-1].save(out_dir / f"bounce_ball_{len(imgs) - 1:03d}.png")
            plant.step()
            step += 1
        gif = out_dir / "bounce_ball.gif"
        imgs[0].save(gif, save_all=True, append_images=imgs[1:], duration=80, loop=0)
        z = plant.poses()["body"][1][2]
        print(f"saved {gif} ({len(imgs)} 帧), 末态球心 z = {z:.4f}")


if __name__ == "__main__":
    BounceDemo.main()
