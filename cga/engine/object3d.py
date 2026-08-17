from cga.motors import Motor

_Mat3 = tuple[tuple[float, ...], ...]

_IDENTITY3: _Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


class Object3D:
    """场景节点: 局部 pose = Motor (先旋转后平移: M = T(pos)·R(axis, angle))。

    可选 linear (3x3): 非 versor 可达的线性块 (scale/mirror/shear),
    世界变换 = M · linear —— motor 保持刚体语义, linear 由
    AffineGeometry 以射线逆变换实现 (见 cga/engine/affine_geometry.py)。
    默认恒等 (纯刚体, v1 行为)。
    """

    def __init__(
        self,
        position: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rotation_axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
        rotation_angle: float = 0.0,
        motor: Motor | None = None,
        linear: _Mat3 | None = None,
    ):
        # Full-pose mode: an arbitrary motor (e.g. a URDF link's world pose,
        # a compound rotation not expressible as a single axis).  Takes
        # precedence over position/rotation_axis/rotation_angle.
        self.motor_override = motor
        if motor is not None:
            mtx = motor.to_matrix()
            self.position = (float(mtx[0][3]), float(mtx[1][3]), float(mtx[2][3]))
            self.rotation_axis = (0.0, 0.0, 1.0)
            self.rotation_angle = 0.0
        else:
            self.position = position
            self.rotation_axis = rotation_axis
            self.rotation_angle = rotation_angle
        self.linear: _Mat3 = (
            _IDENTITY3
            if linear is None
            else tuple(tuple(float(v) for v in row) for row in linear)
        )

    def motor(self) -> Motor:
        """局部 pose motor: full motor if given, else T·R."""
        if self.motor_override is not None:
            return self.motor_override
        return Motor(self.rotation_axis, self.rotation_angle, self.position)
