"""CRDF 渲染 demo: YAML 机器人描述 → FK → CGA 引擎直接渲染。

运行: uv run python demo_robot.py [输出目录] [模型] [关节角...]
输出: <out>/robot_z1.png (默认模型, README 嵌入用 docs/robot_z1.png)

展示: models/z1_arm.crdf.yaml (宇树 Z1 官方 URDF 导入) 里的 blade 几何,
经 Motor FK 链 (robot.fk_list) 直接进 cga.engine —— 无网格、无中间表示。
两个坐标约定转换都在场景构建层, 数据文件保持 URDF 语义 (Z-up):
  1. 根级: URDF Z-up → 引擎 Y-up (WORLD_UP = Rot(X, -π/2))。
  2. 几何局部轴: engine 圆柱/圆盘局部轴 = +Z, 与 URDF 一致, 无需转换。
"""

import sys
from pathlib import Path

from PIL import Image

from cga.engine import (
    AmbientLight,
    BoxGeometry,
    CircleGeometry,
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
from cga.robot import Geometry, Robot, load_robot

WORLD_UP = Motor.rotor((1.0, 0.0, 0.0), -1.5707963267948966)  # Z-up → Y-up

MODEL = Path(__file__).resolve().parent / "models" / "z1_arm.crdf.yaml"


def _to_geometry(g: Geometry):
    """CRDF blade → engine 几何 (局部轴约定一致, 尺寸即参数)。

    cylinder 带 length → 有限圆柱 (端盖); None → 无限 (demo 面板 B 用)。
    """
    if g.blade == "cylinder":
        return CylinderGeometry(g.radius, length=g.length)
    if g.blade == "box":
        assert g.size is not None
        return BoxGeometry(*g.size)
    if g.blade == "sphere":
        return SphereGeometry(g.radius)
    if g.blade == "plane":
        assert g.normal is not None
        return PlaneGeometry(g.normal, g.distance)
    if g.blade == "circle":
        return CircleGeometry(g.radius)
    if g.blade == "mesh":
        raise ValueError(
            f"mesh 引用不渲染 ({g.file}); 重新导入时用 mesh_policy='skip' "
            "(默认, 忽略 mesh)"
        )
    raise ValueError(f"unknown blade {g.blade!r}")


def build_scene(robot: Robot, q: list[float]) -> Scene:
    """FK(q) → 每个 link 的 world motor → 视觉几何 mesh 进场景。"""
    world = robot.fk_list(q)
    colors = {m.name: m.color for m in robot.materials}
    scene = Scene()
    scene.add(
        Mesh(
            PlaneGeometry((0, 1, 0), 0.0),  # 地面 y=0, 臂底恰在地面
            MeshStandardMaterial(Color(0x9AA0A6), roughness=0.8),
        ),
        DirectionalLight(intensity=0.65, direction=(0.4, 1.0, 0.35)),
        PointLight(intensity=0.6, position=(0, 3.5, 2.8)),
        AmbientLight(intensity=0.52),
    )
    for link in robot.links:
        m = WORLD_UP.gp(world[link.name])
        for g in link.geometry:
            if "visual" not in g.role:
                continue
            rgba = colors.get(g.material or "", (0.7, 0.7, 0.7, 1.0))
            mat = MeshStandardMaterial(
                Color(*rgba[:3]), roughness=0.45, metalness=0.1
            )
            scene.add(Mesh(_to_geometry(g), mat, motor=m.gp(g.origin)))
    return scene


def main() -> None:
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else "artifacts")
    out_dir.mkdir(exist_ok=True)
    model = Path(sys.argv[2]) if len(sys.argv) > 2 else MODEL
    robot = load_robot(model)
    # 默认: Z1 舒展姿态 (肩偏航 0.3 + 上臂前倾 120° + 前臂前伸)
    default_q = [0.3, 2.0944, -2.0944, 0.4, 0.3, 0.5]
    q = [float(v) for v in sys.argv[3:]] or default_q
    scene = build_scene(robot, q)
    camera = PerspectiveCamera(
        fov=50, aspect=4 / 3, position=(0.45, 0.22, 0.75), target=(0.25, 0.30, 0.08)
    )
    camera.look_at((0.25, 0.30, 0.08))
    img = Renderer(640, 480, aa=2).render(scene, camera)
    fname = "robot_z1.png" if model == MODEL else f"robot_{robot.name}.png"
    p = out_dir / fname
    Image.frombytes("RGBA", (img.shape[1], img.shape[0]), frame_to_bytes(img)).save(p)
    print(f"saved {p}")


if __name__ == "__main__":
    main()
