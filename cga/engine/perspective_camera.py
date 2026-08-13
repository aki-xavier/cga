from cga.engine.vec3 import Vec3
from cga.motors import Motor


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
        self.up = Vec3.unit(up)
        self.motor = Motor.identity()  # 世界→相机, look_at 时重建

    def look_at(
        self,
        target: tuple[float, float, float],
        up: tuple[float, float, float] | None = None,
    ) -> None:
        """构建世界→相机 motor (相机基 = {right, -up, forward})。"""
        self.target = target
        if up is not None:
            self.up = Vec3.unit(up)
        f = Vec3.unit(tuple(t - p for t, p in zip(target, self.position, strict=True)))
        r = Vec3.unit(Vec3.cross(f, self.up))
        u = Vec3.cross(r, f)  # 相机"上": cross(r,f) 保证 r×u = f 且 u 指向上方
        R = [
            [r[0], r[1], r[2]],
            [-u[0], -u[1], -u[2]],  # Y 向下
            [f[0], f[1], f[2]],
        ]
        t = tuple(-Vec3.dot(R[i], self.position) for i in range(3))
        self.motor = Motor.from_matrix(R, t)
