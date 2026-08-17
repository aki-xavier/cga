"""Reference output for the C algebra self-test (float32, matching ccga)."""
import math

import cga.multivector as M
from cga.algebra import Circle, Plane, Point, Sphere
from cga.motors import Motor

mv = M.Multivector


def p32(label, v):
    vals = [float(x) for x in v.values.tolist()]
    print(f"{label}: " + " ".join(f"{x:.5f}" for x in vals))


e1 = mv.vector(1, 0, 0, 0, 0)
e12 = mv.bivector([1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
p32("gp_e1_e12", e1.gp(e12))
p32("gp_e12_e1", e12.gp(e1))

p = Point(1, 2, 3)
print(f"point_null: {p.gp(p).scalar_part():.5f}")

r = Motor.rotor((0, 0, 1), math.pi / 2)
p1 = r.apply(Point(1, 0, 0))
print(f"rotor_z90_point: {p1.coords()[0]:.5f} {p1.coords()[1]:.5f} {p1.coords()[2]:.5f}")

tr = Motor.translator((1, 2, 3))
p0 = tr.apply(Point(0, 0, 0))
print(f"translator_origin: {p0.coords()[0]:.5f} {p0.coords()[1]:.5f} {p0.coords()[2]:.5f}")

m = Motor.wrap(tr.gp(r))
flat = [float(x) for row in m.to_matrix() for x in row]
print("motor_matrix: " + " ".join(f"{x:.5f}" for x in flat))

s = Sphere((1, 2, 3), 2)
(c, rr) = Sphere.from_dual(s)
print(f"sphere_roundtrip: {c[0]:.5f} {c[1]:.5f} {c[2]:.5f} {rr:.5f}")

pl = Plane((0, 1, 0), 0)
print(f"plane_dist: {pl.dist(Point(0, 5, 0)):.5f}")

cir = Circle((0, 0, 0), 2, (0, 0, 1))
p32("circle", cir)

B = Motor.velocity_bivector((0, 0, 1), (0.1, 0.2, 0.3))
M2 = Motor.exp(B, 1.0)
L = M2.log()
p32("exp_log", L)
p32("exp_log_orig", B)

print(f"reverse_e12_c6: {e12.reverse().values.tolist()[6]:.5f}")
print(f"dual_scalar_c31: {mv.scalar(1.0).dual().values.tolist()[31]:.5f}")
