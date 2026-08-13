"""点对 (直接形式, grade 2)。"""

from cga.multivector import Multivector


class PointPair(Multivector):
    """点对 / 0-球 (grade 2, 直接形式): Pp = p1 ∧ p2。"""

    __slots__ = ()

    def __init__(self, p1: Multivector, p2: Multivector):
        """由两个共形点构造: Pp = p1 ∧ p2。"""
        super().__init__(p1.op(p2).values)
