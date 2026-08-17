"""运动学 demo —— Motor (SE(3) versor) 的直接应用, 无矩阵/四元数换算层。

三个机构同场景:
  1. 齿轮副: 齿数 16:8 → 角速度比精确 −1:2 (节圆相切, 相位错半齿);
  2. 曲柄滑块: 连杆 pose 每帧由两端点解算 (轴角转子, 无矩阵分解);
  3. 螺旋轨迹: M(s) = M₀·exp(s·log(M₀⁻¹M₁)) 插值同一 Motor 驱动小球
     (变换与几何同构 —— 轨迹本身是 motor 的解析函数)。

输出: artifacts/kinematics.gif (默认 72 帧)
用法: .venv/bin/python demo_kinematics.py [帧数]
"""

import math
import sys

from PIL import Image

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
    SphereGeometry,
)
from cga.motors import Motor

# ── 构件 ──────────────────────────────────────────────────────────


def gear_meshes(r_hub, r_tooth, n_teeth, thick, color):
    """齿轮 (局部, 轴 +Y): 轮毂圆柱 + 径向齿阵列 (Motor 复合定位)。"""
    mat = MeshStandardMaterial(color, roughness=0.35, metalness=0.7)
    out = [
        Mesh(
            CylinderGeometry(r_hub, thick),
            mat,
            rotation_axis=(1, 0, 0),
            rotation_angle=-math.pi / 2,  # 局部 +Z → +Y
        )
    ]
    r_mid = r_hub + (r_tooth - r_hub) / 2
    for i in range(n_teeth):
        a = i * 2 * math.pi / n_teeth
        # 齿: 先平移到节圆, 再绕 +Y 转 (T·R 顺序: 右先作用)
        m = Motor.rotor((0, 1, 0), a).gp(Motor.translator((r_mid, 0, 0)))
        out.append(Mesh(BoxGeometry(r_tooth - r_hub + 0.06, thick, 0.16), mat, motor=m))
    return out


def frame_motor(axis, angle, point=(0.0, 0.0, 0.0)):
    """绕任意过点轴的帧变换: M = T(p)·R(axis, angle)·T(−p)。"""
    return (
        Motor.translator(point)
        .gp(Motor.rotor(axis, angle))
        .gp(Motor.translator(tuple(-v for v in point)))
    )


def rod_motor(p, q):
    """连杆 (局部 +Z 圆柱, 居中) → 从 p 到 q 的刚体 pose 与长度。"""
    d = [q[i] - p[i] for i in range(3)]
    length = math.sqrt(sum(v * v for v in d))
    u = [v / length for v in d]
    # 转子: +Z → u (轴角, 无矩阵分解)
    ax = (-u[1], u[0])  # ẑ×u = (−uy, ux, 0)
    an = math.sqrt(ax[0] ** 2 + ax[1] ** 2)
    if an < 1e-12:
        rot = Motor.rotor((1, 0, 0), 0.0 if u[2] > 0 else math.pi)
    else:
        angle = math.acos(max(-1.0, min(1.0, u[2])))
        rot = Motor.rotor((ax[0] / an, ax[1] / an, 0.0), angle)
    mid = tuple((p[i] + q[i]) / 2 for i in range(3))
    return Motor.translator(mid).gp(rot), length


# ── 场景 ──────────────────────────────────────────────────────────

STEEL = MeshStandardMaterial(Color(0x9BA1A6), roughness=0.35, metalness=0.75)
BRASS = MeshStandardMaterial(Color(0xC8A24A), roughness=0.3, metalness=0.8)
DARK = MeshStandardMaterial(Color(0x4A4F54), roughness=0.5, metalness=0.6)
CRANK_C = (-1.6, 0.6, 2.6)  # 曲柄中心
ROD_LEN = 1.35


def build_scene():
    scene = Scene()
    scene.add(
        Mesh(
            PlaneGeometry((0, 1, 0), -0.05),
            MeshStandardMaterial(Color(0x3A4046), roughness=0.9),
        ),
        DirectionalLight(intensity=0.5, direction=(0.4, 1.0, 0.5)),
        PointLight(intensity=0.5, position=(-4, 6, 4)),
        AmbientLight(intensity=0.4),
    )
    # 齿轮副 (节圆相切于 x 轴): 大轮 16 齿在原点, 小轮 8 齿在 x=2.4
    big = gear_meshes(1.28, 1.6, 16, 0.4, Color(0xC8A24A))
    small = gear_meshes(0.64, 0.8, 8, 0.4, Color(0x9BA1A6))
    for m in small:
        m.motor_override = Motor.translator((2.4, 0, 0)).gp(m.motor())
    for m in big + small:
        scene.add(m)
    # 曲柄滑块 (后方 z=2.6 平面): 曲柄盘 + 连杆 + 滑块 + 导轨
    scene.add(
        Mesh(
            CylinderGeometry(0.55, 0.25),
            DARK,
            position=CRANK_C,
            rotation_axis=(1, 0, 0),
            rotation_angle=-math.pi / 2,
        )
    )
    rod = Mesh(CylinderGeometry(0.09, 1.0), BRASS)  # 每帧重设 pose/长度
    slider = Mesh(BoxGeometry(0.5, 0.4, 0.35), STEEL)
    scene.add(rod, slider)
    scene.add(Mesh(BoxGeometry(2.6, 0.08, 0.5), DARK, position=(0.0, 0.4, 2.6)))
    # 螺旋轨迹小球 + 轨道参照珠 (motor exp/log 解析轨迹, 避开机构上方)
    ball = Mesh(
        SphereGeometry(0.18), MeshStandardMaterial(Color(0xC0392B), roughness=0.3)
    )
    scene.add(ball)
    # 轨迹端点: M0 在左侧低位, M1 在右侧高位并绕 +Y 转 3π/2
    m0 = Motor.translator((-3.2, 0.5, 2.2))
    m1 = Motor.translator((3.6, 1.8, 3.4)).gp(Motor.rotor((0, 1, 0), 1.5 * math.pi))
    twist = m0.inverse().gp(m1).log()
    for k in range(9):
        scene.add(
            Mesh(
                SphereGeometry(0.05),
                MeshStandardMaterial(Color(0x7F8C8D), roughness=0.6),
                motor=m0.gp(Motor.exp(twist, k / 8)),
            )
        )
    return scene, big, small, rod, slider, ball, (m0, twist)


def main():
    n_frames = int(sys.argv[1]) if len(sys.argv) > 1 else 72
    scene, big, small, rod, slider, ball, (m0, twist) = build_scene()
    big_local = [m.motor() for m in big]  # 局部 motor (每帧左乘机构变换)
    small_local = [m.motor() for m in small]
    cam = PerspectiveCamera(
        fov=48, aspect=4 / 3, position=(0.4, 5.6, 8.8), target=(0.4, 0.2, 1.2)
    )
    cam.look_at((0.4, 0.2, 1.2))
    renderer = Renderer(480, 360, aa=2)
    frames = []
    for f in range(n_frames):
        s = f / n_frames
        # 齿轮: 大轮 ω = 2π/圈, 小轮 −2ω (齿数比 16:8) + 半齿相位
        g1 = frame_motor((0, 1, 0), 2 * math.pi * s)
        for m, lm in zip(big, big_local):
            m.motor_override = g1.gp(lm)
        g2 = frame_motor((0, 1, 0), -4 * math.pi * s + math.pi / 8, point=(2.4, 0, 0))
        for m, lm in zip(small, small_local):
            m.motor_override = g2.gp(lm)
        # 曲柄滑块: 销 P 在半径 0.35 圆上, 滑块约束在 x 方向导轨
        th = 2 * math.pi * s
        p = (
            CRANK_C[0] + 0.35 * math.cos(th),
            CRANK_C[1],
            CRANK_C[2] + 0.35 * math.sin(th),
        )
        xs = p[0] + math.sqrt(ROD_LEN**2 - (CRANK_C[2] - p[2]) ** 2)
        q = (xs, CRANK_C[1], CRANK_C[2])
        rm, rl = rod_motor(p, q)
        rod.motor_override = rm
        rod.geometry = CylinderGeometry(0.09, rl)
        slider.position = q
        slider.motor_override = None
        # 螺旋小球: M(s) = M0·exp(s·log(M0⁻¹M1))
        ball.motor_override = m0.gp(Motor.exp(twist, s))
        img = renderer.render(scene, cam)
        frames.append(
            Image.frombytes("RGBA", (480, 360), Renderer.frame_to_bytes(img)).convert(
                "P"
            )
        )
        if (f + 1) % 12 == 0:
            print(f"frame {f + 1}/{n_frames}")
    frames[0].save(
        "artifacts/kinematics.gif",
        save_all=True,
        append_images=frames[1:],
        duration=1000 // 24,
        loop=0,
    )
    print("saved artifacts/kinematics.gif")


if __name__ == "__main__":
    main()
