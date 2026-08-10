"""three.js 风格的三维渲染引擎: CGA 建模核心 + MLX/Metal 批量光线追踪。

API 对齐 three.js: Scene / PerspectiveCamera / Mesh / *Geometry /
MeshStandardMaterial / MeshBasicMaterial / AmbientLight /
DirectionalLight / PointLight / Renderer.render(scene, camera) /
OrbitControls。

CGA 核心 (与 three.js 的三角网格光栅化不同):
  - 场景对象 = CGA blade (球/平面/圆柱/圆/盒), 尺寸全在 geometry 参数。
  - 变换 = Motor (versor 共轭), 相机也是 Motor —— 每帧把每个 blade
    共轭进相机空间 (X_cam = M·X·M̃), 再对 (H×W) 射线批量解析求交。
  - 方向向量 (光方向/轴/法向) 共轭后 e1..e3 部分只由 rotor 决定:
    translator 只向 e∞ 槽写杂散项 (t·u), 方向语义不受影响 (无穷远点
    语义, 自检 "direction vector part rotor-only" 覆盖)。

相机空间约定 (与 cga.render 一致): X 右 / Y 下 / Z 前, 针孔
col = fx·X/Z + cx, row = fy·Y/Z + cy, 相机在原点。

抗锯齿: Renderer(aa=N) 每像素 N×N 条分层亚像素射线, 渲染后平均
(超采样 SSAA; 射线一次批量, 代价 = aa² × 像素数)。

范围声明 (v1 与 three.js 的差距, 如实标注):
  - 无阴影/纹理/后处理/tonemap; 颜色线性 I/O, 不做 sRGB 编码。
  - Object3D 无 scale: motor 是刚体变换, 尺寸全走 geometry 参数。
  - 无限平面/圆柱: 无面片裁剪; 相机在柱内等退化情形按内核处理。
  - float32: blade 共轭在 float32 下进行, 场景坐标宜控制在 ±20 内
    (远原点 conformal 抵消是本包已知限制)。
  - 每帧 Python 层循环图元 (~10 个), 像素级全在 MLX GPU 上批量。
"""

import math
from collections.abc import Sequence

import mlx.core as mx

from cga.algebra import E1, E2, E3, Circle, Cylinder, Plane, Point, Sphere
from cga.motors import Motor
from cga.multivector import Multivector

# ── 工具 ───────────────────────────────────────────────────────────


def _dir3(a: Multivector) -> tuple[float, float, float]:
    """grade-1 向量 → 欧氏三元组 (只读 e1..e3 槽)。

    方向向量共轭后 e∞ 槽会混入 (t·u) 杂散项 (translator 写入), 方向
    语义只看向量部分 —— 这里必须忽略 e∞ 槽。"""
    return (
        float(a.values[1]),
        float(a.values[2]),
        float(a.values[3]),
    )


def _unit(a: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


# ── 颜色 ───────────────────────────────────────────────────────────


class Color:
    """sRGB 颜色: Color(0xRRGGBB) 或 Color(r, g, b) (0-1 浮点)。"""

    __slots__ = ("r", "g", "b")

    def __init__(self, r: float, g: float | None = None, b: float | None = None):
        if g is None and b is None:
            c = int(r)
            self.r, self.g, self.b = (
                ((c >> 16) & 0xFF) / 255.0,
                ((c >> 8) & 0xFF) / 255.0,
                (c & 0xFF) / 255.0,
            )
        else:
            self.r, self.g, self.b = float(r), float(g or 0.0), float(b or 0.0)

    def rgb(self) -> tuple[float, float, float]:
        return (self.r, self.g, self.b)


# ── 场景图 ─────────────────────────────────────────────────────────


class Object3D:
    """场景节点: 局部 pose = Motor (先旋转后平移: M = T(pos)·R(axis, angle))。

    无 scale —— motor 是刚体变换; 尺寸全走 geometry 构造参数。
    """

    def __init__(
        self,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rotation_axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
        rotation_angle: float = 0.0,
        motor: Motor | None = None,
    ):
        # Full-pose mode: an arbitrary motor (e.g. a URDF link's world pose,
        # a compound rotation not expressible as a single axis).  Takes
        # precedence over position/rotation_axis/rotation_angle.
        self._motor = motor
        if motor is not None:
            mtx = motor.to_matrix()
            self.position = (float(mtx[0][3]), float(mtx[1][3]), float(mtx[2][3]))
            self.rotation_axis = (0.0, 0.0, 1.0)
            self.rotation_angle = 0.0
        else:
            self.position = position
            self.rotation_axis = rotation_axis
            self.rotation_angle = rotation_angle

    def motor(self) -> Motor:
        """局部 pose motor: full motor if given, else T·R."""
        if self._motor is not None:
            return self._motor
        return Motor(self.rotation_axis, self.rotation_angle, self.position)


class Scene:
    """场景: 对象列表 + 灯光列表 + 背景色 (three.js Scene)。"""

    def __init__(self, background: Color | None = None):
        self.objects: list[Mesh] = []
        self.lights: list = []
        self.background = background if background is not None else Color(0x87CEEB)

    def add(self, *objs) -> None:
        for o in objs:
            if isinstance(o, Mesh):
                self.objects.append(o)
            else:
                self.lights.append(o)


class Mesh(Object3D):
    """几何 + 材质 + pose (three.js Mesh)。"""

    def __init__(
        self,
        geometry: _Geometry,
        material: Material | None = None,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rotation_axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
        rotation_angle: float = 0.0,
        motor: Motor | None = None,
    ):
        super().__init__(position, rotation_axis, rotation_angle, motor)
        self.geometry = geometry
        self.material = material if material is not None else MeshStandardMaterial()


# ── 几何 (CGA blade, 局部系) ──────────────────────────────────────


class _Geometry:
    """几何基类: CGA blade + 每帧共轭进相机空间 + 批量求交。

    to_camera(motor) -> params:  CPU, 每帧一次, versor 共轭出相机空间参数。
    intersect(params, o, d) -> (t, n, mask):  GPU, mlx 批量 (N,) 数组。
      t    最近命中距离 (inf = 未命中)
      n    (N,3) 单位法向 (指向相机侧)
      mask (N,) 命中掩码 (1/0 浮点)
    """

    def to_camera(self, motor: Motor) -> tuple:
        raise NotImplementedError

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        raise NotImplementedError


class SphereGeometry(_Geometry):
    """球 (CGA Sphere blade)。构造参数只有半径, 尺寸建模的核心。"""

    def __init__(self, radius: float):
        if radius <= 0:
            raise ValueError(f"sphere radius must be > 0, got {radius}")
        self.blade = Sphere((0.0, 0.0, 0.0), radius)  # 局部系原点

    def to_camera(self, motor: Motor) -> tuple:
        s = motor.apply(self.blade)  # 对偶球 blade 共轭
        w = float(s.values[4])
        v1, v2, v3 = (float(s.values[1]), float(s.values[2]), float(s.values[3]))
        f = float(s.values[5])
        cx, cy, cz = v1 / w, v2 / w, v3 / w
        r2 = (v1 * v1 + v2 * v2 + v3 * v3) / (w * w) - 2.0 * f / w
        return ((cx, cy, cz), math.sqrt(max(0.0, r2)))

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        c = mx.array(params[0], dtype=mx.float32)
        r = params[1]
        oc = o - c
        b = 2.0 * mx.sum(oc * d, axis=-1)
        cq = mx.sum(oc * oc, axis=-1) - r * r
        disc = b * b - 4.0 * cq
        valid = disc > 1e-12
        sq = mx.sqrt(mx.maximum(disc, 0.0))
        t1 = (-b - sq) / 2.0
        t2 = (-b + sq) / 2.0
        t = mx.where(mx.logical_and(valid, t1 > 1e-6), t1, t2)
        mask = mx.logical_and(valid, t > 1e-6)
        p = o + t[:, None] * d
        n = (p - c) / r
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        # 相机在球内: t1<0 取 t2, 法向翻向相机
        inside = mx.logical_and(mask, t1 <= 1e-6)
        n = mx.where(inside[:, None], -n, n)
        return t, n, mask


class PlaneGeometry(_Geometry):
    """无限平面 (CGA Plane blade, 对偶形式 n + d·e∞)。"""

    def __init__(
        self,
        normal: tuple[float, float, float] = (0.0, 0.0, 1.0),
        distance: float = 0.0,
    ):
        self.blade = Plane(normal, distance)

    def to_camera(self, motor: Motor) -> tuple:
        pi = motor.apply(self.blade)
        n = _unit(_dir3(pi))
        d = float(pi.values[5])
        return (n, d)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        n = mx.array(params[0], dtype=mx.float32)
        dist = params[1]
        denom = mx.sum(n * d, axis=-1)
        t = (dist - mx.sum(n * o, axis=-1)) / denom
        mask = mx.logical_and(mx.abs(denom) > 1e-9, t > 1e-6)
        n_rep = mx.broadcast_to(n, o.shape)
        return t, mx.where(mask[:, None], n_rep, mx.zeros_like(n_rep)), mask


class CylinderGeometry(_Geometry):
    """无限圆柱 (CGA Cylinder = 轴 Line blade + 半径; 解析槽手动变换)。

    轴点走完整 motor (Point 共轭), 轴方向是 e∞ 系数为 0 的方向向量
    (translator 不变), 半径在刚体运动下不变。CGA Cylinder 是无限长。
    """

    def __init__(self, radius: float):
        if radius <= 0:
            raise ValueError(f"cylinder radius must be > 0, got {radius}")
        self.blade = Cylinder((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), radius)  # 局部轴 = +Z
        self._radius = float(radius)

    def to_camera(self, motor: Motor) -> tuple:
        q = motor.apply(Point(0.0, 0.0, 0.0)).coords()
        u = _unit(_dir3(motor.apply(E3)))  # 方向只吃旋转, translator 天然不变
        return (q, u, self._radius)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        q = mx.array(params[0], dtype=mx.float32)
        u = mx.array(params[1], dtype=mx.float32)
        r = params[2]
        oc = o - q
        d_par = mx.sum(d * u, axis=-1, keepdims=True)
        o_par = mx.sum(oc * u, axis=-1, keepdims=True)
        d_p = d - d_par * u
        o_p = oc - o_par * u
        a = mx.sum(d_p * d_p, axis=-1)
        b = 2.0 * mx.sum(o_p * d_p, axis=-1)
        cq = mx.sum(o_p * o_p, axis=-1) - r * r
        disc = b * b - 4.0 * a * cq
        valid = mx.logical_and(a > 1e-12, disc > 1e-12)
        sq = mx.sqrt(mx.maximum(disc, 0.0))
        t1 = (-b - sq) / (2.0 * a)
        t2 = (-b + sq) / (2.0 * a)
        t = mx.where(mx.logical_and(valid, t1 > 1e-6), t1, t2)
        mask = mx.logical_and(valid, t > 1e-6)
        hit = o_p + t[:, None] * d_p
        n = hit / r
        inside = mx.logical_and(mask, t1 <= 1e-6)
        n = mx.where(inside[:, None], -n, n)
        n = mx.where(mask[:, None], n, mx.zeros_like(n))
        return t, n, mask


class BoxGeometry(_Geometry):
    """轴对齐盒 (3 对平面 slab, 解析存储, 局部系中心在原点)。"""

    def __init__(self, width: float, height: float, depth: float):
        if min(width, height, depth) <= 0:
            raise ValueError(
                f"box dimensions must be > 0, got {(width, height, depth)}"
            )
        self._half = (width / 2.0, height / 2.0, depth / 2.0)

    def to_camera(self, motor: Motor) -> tuple:
        c = motor.apply(Point(0.0, 0.0, 0.0)).coords()
        # 局部轴方向经 motor 旋转 (translator 不变), 半尺寸不变
        axes = []
        for ax in (E1, E2, E3):
            axes.append(_unit(_dir3(motor.apply(ax))))
        return (c, axes, self._half)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        c = mx.array(params[0], dtype=mx.float32)
        axes = params[1]
        half = params[2]
        # 把射线变换进盒局部系: o' = Rᵀ(o−c), d' = Rᵀ·d
        oc = o - c
        op = mx.stack(
            [
                mx.sum(oc * mx.array(axes[i], dtype=mx.float32), axis=-1)
                for i in range(3)
            ],
            axis=-1,
        )
        dp = mx.stack(
            [
                mx.sum(d * mx.array(axes[i], dtype=mx.float32), axis=-1)
                for i in range(3)
            ],
            axis=-1,
        )
        inv = 1.0 / dp
        t0 = -inv * (op + mx.array(half, dtype=mx.float32))
        t1 = -inv * (op - mx.array(half, dtype=mx.float32))
        tmin = mx.minimum(t0, t1)
        tmax = mx.maximum(t0, t1)
        t_entry = mx.max(tmin, axis=-1)
        t_exit = mx.min(tmax, axis=-1)
        valid = mx.logical_and(t_entry < t_exit, t_exit > 1e-6)
        # 外命中: 可见面 = tmin 最大的轴 (进入面); 相机在内: 可见面 = 出口面
        i_entry = mx.argmax(tmin, axis=-1)
        i_exit = mx.argmin(tmax, axis=-1)
        inside_hit = mx.logical_and(valid, t_entry <= 1e-6)
        t = mx.where(mx.logical_and(valid, ~inside_hit), t_entry, t_exit)
        idx = mx.where(inside_hit, i_exit, i_entry)
        # 法向 = −sign(d'_{idx})·e_{idx} (两种命中同式, 恒指向相机侧)
        e = mx.eye(3, dtype=mx.float32)
        n = (
            mx.take(e, idx, axis=0)
            * (-mx.sign(mx.take_along_axis(dp, idx[:, None], axis=-1).squeeze(-1)))[
                :, None
            ]
        )
        n = mx.where(valid[:, None], n, mx.zeros_like(n))
        return t, n, valid


class CircleGeometry(_Geometry):
    """圆盘 (CGA Circle blade = 对偶球∧对偶平面; 解析存储中心/法向/半径)。"""

    def __init__(self, radius: float):
        if radius <= 0:
            raise ValueError(f"circle radius must be > 0, got {radius}")
        self.blade = Circle((0.0, 0.0, 0.0), radius, (0.0, 0.0, 1.0))  # 局部法向 +Z
        self._radius = float(radius)

    def to_camera(self, motor: Motor) -> tuple:
        c = motor.apply(Point(0.0, 0.0, 0.0)).coords()
        n = _unit(_dir3(motor.apply(E3)))
        return (c, n, self._radius)

    def intersect(
        self, params: tuple, o: mx.array, d: mx.array
    ) -> tuple[mx.array, mx.array, mx.array]:
        c = mx.array(params[0], dtype=mx.float32)
        n = mx.array(params[1], dtype=mx.float32)
        r = params[2]
        denom = mx.sum(n * d, axis=-1)
        t = mx.sum(n * (c - o), axis=-1) / denom
        front = denom < 0.0  # 法向面向相机的一侧
        p = o + t[:, None] * d
        in_disc = mx.sum((p - c) * (p - c), axis=-1) <= r * r
        mask = mx.logical_and(mx.logical_and(mx.abs(denom) > 1e-9, t > 1e-6), in_disc)
        n = mx.where(front[:, None], n, -n)  # 背面可见时翻向相机
        return t, mx.where(mask[:, None], n, mx.zeros_like(n)), mask


# ── 材质 ───────────────────────────────────────────────────────────


class Material:
    pass


class MeshBasicMaterial(Material):
    """不接光照, 直接输出颜色 (three.js MeshBasicMaterial)。"""

    def __init__(self, color: Color | int = 0xFFFFFF):
        self.color = Color(color) if isinstance(color, int) else color


class MeshStandardMaterial(Material):
    """标准材质: 环境 + Lambert 漫反射 + Blinn-Phong 高光。

    metalness: 1 → 高光色 = 自身颜色 (金属), 0 → 高光白。
    roughness: 0..1 → 高光指数 200→4 (指数越低越糊)。
    v1 不做 PBR 能量守恒/IBL, 是廉价近似 (与 three.js 的差距如实标注)。
    """

    def __init__(
        self,
        color: Color | int = 0xFFFFFF,
        roughness: float = 0.5,
        metalness: float = 0.0,
        emissive: Color | int = 0x000000,
    ):
        self.color = Color(color) if isinstance(color, int) else color
        self.roughness = float(min(1.0, max(0.0, roughness)))
        self.metalness = float(min(1.0, max(0.0, metalness)))
        self.emissive = Color(emissive) if isinstance(emissive, int) else emissive


# ── 灯光 ───────────────────────────────────────────────────────────


class AmbientLight:
    def __init__(self, color: Color | int = 0xFFFFFF, intensity: float = 0.3):
        self.color = Color(color) if isinstance(color, int) else color
        self.intensity = float(intensity)


class DirectionalLight:
    """平行光。direction 是"光来的方向" (指向光源), 世界系单位向量。

    direction 以 e∞ 系数为 0 的 grade-1 向量存, motor 共轭只吃旋转、
    translator 天然不变 —— 相机平移不会改变平行光着色 (自检覆盖)。
    """

    def __init__(
        self,
        color: Color | int = 0xFFFFFF,
        intensity: float = 1.0,
        direction: tuple[float, float, float] = (0.0, -1.0, 0.0),
    ):
        self.color = Color(color) if isinstance(color, int) else color
        self.intensity = float(intensity)
        self.direction = _unit(direction)


class PointLight:
    """点光源: 位置 + 强度, 距离衰减 1/(1 + d²/8) (软衰减, v1 声明)。"""

    def __init__(
        self,
        color: Color | int = 0xFFFFFF,
        intensity: float = 1.0,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ):
        self.color = Color(color) if isinstance(color, int) else color
        self.intensity = float(intensity)
        self.position = position


# ── 相机 ───────────────────────────────────────────────────────────


class PerspectiveCamera:
    """透视相机 (three.js PerspectiveCamera)。

    世界→相机 motor M_cam 由 position/target/up 构建 (from_matrix,
    roundtrip 自检钉死符号): 相机空间 = X 右 / Y 下 / Z 前 (render.py 约定)。
    """

    def __init__(
        self,
        fov: float = 50.0,
        aspect: float = 16.0 / 9.0,
        near: float = 0.1,
        far: float = 100.0,
        position: tuple[float, float, float] = (0.0, 0.0, 5.0),
        target: tuple[float, float, float] = (0.0, 0.0, 0.0),
        up: tuple[float, float, float] = (0.0, 1.0, 0.0),
    ):
        if fov <= 0 or fov >= 180:
            raise ValueError(f"fov must be in (0, 180), got {fov}")
        self.fov = float(fov)
        self.aspect = float(aspect)
        self.near = float(near)
        self.far = float(far)
        self.position = position
        self.target = target
        self.up = _unit(up)
        self.motor = Motor.identity()  # 世界→相机, look_at 时重建

    def look_at(
        self,
        target: tuple[float, float, float],
        up: tuple[float, float, float] | None = None,
    ) -> None:
        """构建世界→相机 motor (相机基 = {right, -up, forward})。"""
        self.target = target
        if up is not None:
            self.up = _unit(up)
        f = _unit(tuple(t - p for t, p in zip(target, self.position, strict=True)))
        r = _unit(_cross(f, self.up))
        u = _cross(r, f)  # 相机"上": cross(r,f) 保证 r×u = f 且 u 指向上方
        R = [
            [r[0], r[1], r[2]],
            [-u[0], -u[1], -u[2]],  # Y 向下
            [f[0], f[1], f[2]],
        ]
        t = tuple(-_dot(R[i], self.position) for i in range(3))
        self.motor = Motor.from_matrix(R, t)


# ── 轨道控制 ───────────────────────────────────────────────────────


class OrbitControls:
    """球面轨道 (three.js OrbitControls 的静态版): 改属性后 update()。

    azimuth/elevation 弧度, radius 距离, target 注视点。
    """

    def __init__(
        self,
        camera: PerspectiveCamera,
        target: tuple[float, float, float] = (0.0, 0.0, 0.0),
        azimuth: float = 0.0,
        elevation: float = 0.4,
        radius: float = 8.0,
    ):
        self.camera = camera
        self.target = target
        self.azimuth = float(azimuth)
        self.elevation = float(elevation)
        self.radius = float(radius)

    def update(self) -> None:
        ce = math.cos(self.elevation)
        x = self.radius * ce * math.sin(self.azimuth)
        y = self.radius * math.sin(self.elevation)
        z = self.radius * ce * math.cos(self.azimuth)
        self.camera.position = (
            self.target[0] + x,
            self.target[1] + y,
            self.target[2] + z,
        )
        self.camera.look_at(self.target)


# ── 渲染器 ─────────────────────────────────────────────────────────


class Renderer:
    """离屏光线追踪渲染器 (three.js WebGLRenderer 的 MLX 对应物)。

    render(scene, camera) -> (H, W, 4) uint8 RGBA (GPU→CPU 每帧回传)。
    每帧: 全部图元共轭进相机空间 (CPU, 图元数 ~10 可忽略) → 批量求交
    (MLX GPU, 全像素向量化) → Blinn-Phong 着色 → sRGB 近似输出。
    """

    def __init__(self, width: int = 640, height: int = 480, aa: int = 1):
        """超采样抗锯齿: aa=1 每像素 1 条射线 (像素中心, 原行为);
        aa=N → 每像素 N×N 条分层亚像素射线取平均 (SSAA, 射线一次批量)。"""
        if aa < 1:
            raise ValueError(f"aa must be >= 1, got {aa}")
        self.width = int(width)
        self.height = int(height)
        self.aa = int(aa)
        self._cam = None
        self._rays = None  # (aa²·N, 3) 单位方向, N = H·W, 按相机内参惰性构建

    def _build_rays(self) -> mx.array:
        """像素网格 → 相机空间单位射线方向 (相机在原点, 无透视偏移)。

        aa>1: 每像素 k×k (k=aa) 分层亚像素采样, 射线数 = aa²·H·W,
        渲染后按样本平均 (框滤波超采样 = SSAA)。aa=1 与原行为逐位一致。
        """
        H, W = self.height, self.width
        fy = H / (2.0 * math.tan(math.radians(self._cam.fov) / 2.0))
        fx = fy * self._cam.aspect
        cx, cy = (W - 1) / 2.0, (H - 1) / 2.0
        u0 = (mx.arange(W, dtype=mx.float32) - cx) / fx  # (W,) 像素中心
        v0 = (mx.arange(H, dtype=mx.float32) - cy) / fy  # (H,)
        z = mx.ones((H, W), dtype=mx.float32)
        dirs = []
        k = self.aa
        for j in range(k):
            for i in range(k):
                du = ((i + 0.5) / k - 0.5) / fx  # 亚像素偏移 (射线单位, 像素内分层)
                dv = ((j + 0.5) / k - 0.5) / fy
                u = mx.broadcast_to((u0 + du)[None, :], (H, W))
                v = mx.broadcast_to((v0 + dv)[:, None], (H, W))
                dirs.append(mx.stack([u, v, z], axis=-1))
        rays = mx.concatenate(dirs, axis=0).reshape(-1, 3)  # (aa²·N, 3)
        n = mx.sqrt(mx.sum(rays * rays, axis=-1, keepdims=True))
        return rays / n

    def _shade(
        self,
        p: mx.array,
        n: mx.array,
        mat: MeshStandardMaterial,
        lights: Sequence,
        d: mx.array,
        ambient: AmbientLight | None = None,
    ) -> mx.array:
        """Blinn-Phong 着色 (p = 命中点, n = 相机空间法向, d = 射线方向)。

        L 与点光源位置均已变换进相机空间 (motor 共轭)。
        """
        v = -d
        k = 1.0 - mat.roughness
        expo = 4.0 + 196.0 * k * k  # roughness 0→200, 1→4
        diff_c = mx.array(mat.color.rgb(), dtype=mx.float32) * (1.0 - mat.metalness)
        spec_c = mx.array(
            tuple(
                m * (1.0 - mat.metalness) + c * mat.metalness
                for m, c in zip((1.0, 1.0, 1.0), mat.color.rgb())
            ),
            dtype=mx.float32,
        )
        out = mx.broadcast_to(mx.array(mat.emissive.rgb(), dtype=mx.float32), p.shape)
        if ambient is not None:
            amb = mx.array(ambient.color.rgb(), dtype=mx.float32) * ambient.intensity
            out = out + mx.broadcast_to(amb, p.shape) * diff_c
        # N·V 作高光可见门 (掠射角高光消失, 廉价近似)
        ndv = mx.maximum(mx.sum(n * v, axis=-1, keepdims=True), 0.0)
        for light in lights:
            lc = mx.array(light.color.rgb(), dtype=mx.float32)
            if isinstance(light, DirectionalLight):
                ld = mx.broadcast_to(
                    mx.array(light.direction, dtype=mx.float32), p.shape
                )
                atten = light.intensity
            elif isinstance(light, PointLight):
                lv = (
                    mx.broadcast_to(mx.array(light.position, dtype=mx.float32), p.shape)
                    - p
                )
                dist2 = mx.sum(lv * lv, axis=-1, keepdims=True)
                ld = lv / mx.sqrt(dist2)
                atten = light.intensity / (1.0 + dist2 / 8.0)
            else:
                continue
            nl = mx.maximum(mx.sum(n * ld, axis=-1, keepdims=True), 0.0)
            h = ld + v
            hn = mx.sqrt(mx.sum(h * h, axis=-1, keepdims=True))
            h = h / mx.maximum(hn, 1e-12)  # H=0 (光与视线反向) 时防 NaN
            spec = mx.pow(mx.maximum(mx.sum(n * h, axis=-1, keepdims=True), 0.0), expo)
            contrib = lc * atten * (diff_c * nl + spec_c * spec * ndv)
            out = out + contrib
        return out

    def render(self, scene: Scene, camera: PerspectiveCamera) -> mx.array:
        """渲染一帧 → (H, W, 4) uint8 RGBA。"""
        self._cam = camera
        if self._rays is None:
            self._rays = self._build_rays()
        o = mx.zeros_like(self._rays)  # 相机在原点 (N,3)
        N = o.shape[0]
        bg = mx.broadcast_to(mx.array(scene.background.rgb(), dtype=mx.float32), (N, 3))
        # 灯光进相机空间 (点光位置吃平移, 平行光方向不吃平移); 拷贝不原地改
        lit = []
        ambient = None
        for light in scene.lights:
            if isinstance(light, DirectionalLight):
                d_world = Multivector.vector(*light.direction)
                lit.append(
                    DirectionalLight(
                        light.color, light.intensity, _dir3(camera.motor.apply(d_world))
                    )
                )
            elif isinstance(light, PointLight):
                pos_cam = camera.motor.apply(Point(*light.position)).coords()
                lit.append(PointLight(light.color, light.intensity, pos_cam))
            elif isinstance(light, AmbientLight):
                ambient = light
            else:
                lit.append(light)
        acc = mx.broadcast_to(mx.zeros(3, dtype=mx.float32), (N, 3))
        miss = mx.full((N,), float("inf"), dtype=mx.float32)
        for obj in scene.objects:
            wm = camera.motor.compose(obj.motor())  # 世界→相机 · 局部→世界
            params = obj.geometry.to_camera(wm)
            t, n, mask = obj.geometry.intersect(params, o, self._rays)
            hit = mx.logical_and(mask, t < miss)
            if isinstance(obj.material, MeshStandardMaterial):
                p = o + t[:, None] * self._rays
                col = self._shade(p, n, obj.material, lit, self._rays, ambient)
            elif isinstance(obj.material, MeshBasicMaterial):
                col = mx.broadcast_to(
                    mx.array(obj.material.color.rgb(), dtype=mx.float32), (N, 3)
                )
            else:
                raise TypeError(f"unknown material {type(obj.material).__name__}")
            acc = mx.where(hit[:, None], col, acc)
            miss = mx.where(hit, t, miss)
        rgb = mx.where(mx.isfinite(miss)[:, None], acc, bg)
        S = self.aa * self.aa
        if S > 1:
            # 超采样平均: (aa²·N, 3) → 每像素 aa² 条样本取均值
            rgb = mx.mean(mx.reshape(rgb, (S, N // S, 3)), axis=0)
        # v1 线性 I/O: 颜色当线性值直接输出, 不做 sRGB 编码 (hex 颜色
        # 直接是 sRGB 编码, 若做 sqrt 输出会双重提亮; 精确管线留后)
        rgba = mx.concatenate([rgb, mx.ones((N // S, 1), dtype=mx.float32)], axis=-1)
        rgba = mx.clip(rgba * 255.0, 0.0, 255.0).astype(mx.uint8)
        return mx.reshape(rgba, (self.height, self.width, 4))


def render_frame(
    scene: Scene,
    camera: PerspectiveCamera,
    width: int = 640,
    height: int = 480,
    aa: int = 1,
) -> mx.array:
    """单帧快捷入口 (demo 用)。"""
    return Renderer(width, height, aa=aa).render(scene, camera)
