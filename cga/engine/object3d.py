from cga.motors import Motor


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

    def motor(self) -> Motor:
        """局部 pose motor: full motor if given, else T·R."""
        if self.motor_override is not None:
            return self.motor_override
        return Motor(self.rotation_axis, self.rotation_angle, self.position)
