"""线 (直接形式, grade 3)。"""

from cga.multivector import Multivector


class Line(Multivector):
    """线 (grade 3, 直接形式): L = p1 ∧ p2 ∧ e∞。"""

    __slots__ = ()

    def __init__(self, p1: Multivector, p2: Multivector):
        """由两个共形点构造: L = p1 ∧ p2 ∧ e∞。"""
        super().__init__(p1.op(p2).op(Multivector.EINF).values)
