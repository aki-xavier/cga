import mlx.core as mx

from cga.motors import Motor


class GeometryBase:
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
