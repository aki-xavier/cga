"""扩展建模能力测试: float64 精度 / 仿射 / 新图元 / 网格与互操作 / CGS v3。

P1: set_precision float64 远原点共轭
P2: AffineGeometry scale/mirror
P4: cone/torus/ellipsoid
P5: 耳切/挤出/放样/网格成员测试/OBJ/GLB roundtrip
P6: CGS scale/mirror/CSG/新图元/precision 语句
守门: vecmat 全精度 (mx.matmul GPU 降精度回归)
"""

import mlx.core as mx
import pytest

from cga.engine import (
    AffineGeometry,
    ConeGeometry,
    EllipsoidGeometry,
    MeshGeometry,
    SphereGeometry,
    TorusGeometry,
)
from cga.engine.geometry_base import vecmat
from cga.motors import Motor

RAY_O = mx.array([[0.0, 0.0, 5.0]])
RAY_D = mx.array([[0.0, 0.0, -1.0]])


class TestPrecision:
    def test_float64_far_conjugation(self):
        from cga.algebra import Sphere
        from cga.multivector import set_precision

        try:
            set_precision("float64")
            c, r = Sphere.from_dual(
                Motor(None, 0.0, (500.0, 0.0, 0.0)).apply(Sphere((0, 0, 0), 1.0))
            )
            assert c == pytest.approx((500.0, 0.0, 0.0), abs=1e-9)
            assert r == pytest.approx(1.0, abs=1e-9)
        finally:
            set_precision("float32")

    def test_precision_rejects_bad_mode(self):
        from cga.multivector import set_precision

        with pytest.raises(ValueError, match="precision"):
            set_precision("float128")


class TestAffine:
    def test_scaled_sphere_interval(self):
        g = AffineGeometry(SphereGeometry(1.0), ((2, 0, 0), (0, 1, 0), (0, 0, 1)))
        p = g.to_camera(Motor.identity())
        t, n, m = g.intersect(p, RAY_O, RAY_D)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(4.0, abs=1e-5)
        assert n.tolist()[0] == pytest.approx([0.0, 0.0, 1.0], abs=1e-4)
        # x=3 处射线超出 x 半宽 2 → miss
        o2 = mx.array([[3.0, 0.0, 5.0]])
        _t, _n, m2 = g.intersect(p, o2, RAY_D)
        assert not m2.tolist()[0]

    def test_singular_linear_rejected(self):
        with pytest.raises(ValueError, match="singular"):
            AffineGeometry(SphereGeometry(1.0), ((0, 0, 0), (0, 1, 0), (0, 0, 1)))

    def test_vecmat_full_precision(self):
        # mx.matmul GPU float32 会把 1.0001 截成 1.0 (bfloat16 级) —— 守门
        v = mx.array([[1.0, 1.0, 1.0001], [2.0, 2.0, 2.0]], dtype=mx.float32)
        out = vecmat(v, mx.eye(3, dtype=mx.float32))
        assert out.tolist()[0][2] == pytest.approx(1.0001, abs=1e-7)

    def test_decompose_rigid_roundtrip(self):
        from cga.engine.affine_geometry import decompose_rigid

        m = Motor((0, 1, 0), 0.7, (1, 2, 3))
        motor, lin = decompose_rigid(tuple(tuple(r) for r in m.to_matrix()))
        err = max(
            abs(lin[i][j] - (1.0 if i == j else 0.0))
            for i in range(3)
            for j in range(3)
        )
        assert err < 1e-4
        t = motor.to_matrix()
        assert (t[0][3], t[1][3], t[2][3]) == pytest.approx((1, 2, 3), abs=1e-4)


class TestNewPrimitives:
    def test_cone_side_ray(self):
        # r=1 h=2 (k=0.5): z=−0.5 (s=−1.5) 处半径 0.75 → t=4.25
        g = ConeGeometry(1.0, 2.0)
        p = g.to_camera(Motor.identity())
        o = mx.array([[5.0, 0.0, -0.5]])
        d = mx.array([[-1.0, 0.0, 0.0]])
        t, _n, m = g.intersect(p, o, d)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(4.25, abs=1e-4)

    def test_cone_contains(self):
        g = ConeGeometry(1.0, 2.0)
        p = g.to_camera(Motor.identity())
        inside = g.contains(p, mx.array([[0.0, 0.0, 0.0], [0.6, 0.0, 0.0]]))
        assert inside.tolist() == [True, False]

    def test_torus_rays(self):
        g = TorusGeometry(1.0, 0.3)
        p = g.to_camera(Motor.identity())
        # 中心孔射线: miss; x=1 管截面: t = 5−0.3 = 4.7
        _t, _n, m = g.intersect(p, RAY_O, RAY_D)
        assert not m.tolist()[0]
        o2 = mx.array([[1.0, 0.0, 5.0]])
        t2, n2, m2 = g.intersect(p, o2, RAY_D)
        assert m2.tolist()[0]
        assert t2.tolist()[0] == pytest.approx(4.7, abs=1e-3)
        assert n2.tolist()[0] == pytest.approx([0.0, 0.0, 1.0], abs=1e-2)

    def test_torus_contains(self):
        g = TorusGeometry(1.0, 0.3)
        p = g.to_camera(Motor.identity())
        assert g.contains(p, mx.array([[1.0, 0.0, 0.1], [0.0, 0.0, 0.0]])).tolist() == [
            True,
            False,
        ]

    def test_ellipsoid_is_scaled_sphere(self):
        g = EllipsoidGeometry(2.0, 1.0, 1.0)
        p = g.to_camera(Motor.identity())
        t, _n, m = g.intersect(p, RAY_O, RAY_D)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(4.0, abs=1e-4)  # z 半轴 1


class TestModelingBuilders:
    def test_earclip_l_shape(self):
        from cga.modeling import triangulate

        tris = triangulate([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)])
        assert len(tris) == 4  # L 形 6 点 → 4 三角形

    def test_earclip_rejects_degenerate(self):
        from cga.modeling import triangulate

        with pytest.raises(ValueError):
            triangulate([(0, 0), (1, 1), (2, 2)])  # 共线

    def test_extrude_hit_and_contains(self):
        from cga.modeling import extrude

        g = MeshGeometry(
            *extrude([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)], 1.5)
        )
        p = g.to_camera(Motor.identity())
        o = mx.array([[1.0, 1.0, 5.0]])
        t, _n, m = g.intersect(p, o, RAY_D)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(3.5, abs=1e-5)  # 顶盖 z=1.5
        # 凹角外 miss
        o2 = mx.array([[3.0, 3.0, 5.0]])
        _t, _n, m2 = g.intersect(p, o2, RAY_D)
        assert not m2.tolist()[0]
        # contains (避开面片对角线的退化点)
        c = g.contains(p, mx.array([[1.0, 0.9, 0.6], [3.0, 3.0, 0.75]]))
        assert c.tolist() == [True, False]

    def test_loft_between_squares(self):
        from cga.modeling import loft

        verts, faces = loft(
            [
                [(0, 0), (2, 0), (2, 2), (0, 2)],
                [(0.4, 0.4), (1.6, 0.4), (1.6, 1.6), (0.4, 1.6)],
            ],
            [0.0, 1.0],
        )
        assert len(verts) == 8
        g = MeshGeometry(verts, faces)
        p = g.to_camera(Motor.identity())
        o = mx.array([[1.0, 1.0, 5.0]])
        t, _n, m = g.intersect(p, o, RAY_D)
        assert m.tolist()[0]
        assert t.tolist()[0] == pytest.approx(4.0, abs=1e-4)  # 顶盖 z=1


class TestMeshIO:
    def test_obj_roundtrip(self, tmp_path):
        from cga.mesh_io import load_obj, save_obj
        from cga.modeling import extrude

        verts, faces = extrude([(0, 0), (2, 0), (2, 2), (0, 2)], 1.0)
        path = tmp_path / "b.obj"
        save_obj(path, [(verts, faces, None)])
        v2, f2 = load_obj(path)
        assert v2 == [tuple(map(float, v)) for v in verts]
        assert f2 == faces

    def test_glb_roundtrip_with_transform(self, tmp_path):
        from cga.mesh_io import load_gltf, save_glb
        from cga.modeling import extrude

        verts, faces = extrude([(0, 0), (2, 0), (2, 2), (0, 2)], 1.0)
        t4 = ((1, 0, 0, 1), (0, 1, 0, 2), (0, 0, 1, 3), (0, 0, 0, 1))
        path = tmp_path / "b.glb"
        save_glb(path, [(verts, faces, t4, (0.8, 0.2, 0.2))])
        loaded = load_gltf(path)
        assert len(loaded) == 1
        v2, f2, m4 = loaded[0]
        assert f2 == faces
        assert (m4[0][3], m4[1][3], m4[2][3]) == pytest.approx((1, 2, 3), abs=1e-6)
        assert len(v2) == len(verts)


class TestCgsV3:
    def test_modifier_ordering(self):
        from cga.scene_lang import SceneLoader

        sc, _ = SceneLoader.load("translate([10,0,0]) scale(2) sphere(r=1);")
        assert sc.objects[0].position == pytest.approx((10.0, 0.0, 0.0))
        sc2, _ = SceneLoader.load(
            "mirror(axis=[1,0,0]) translate([2.5,0,0]) sphere(r=1);"
        )
        assert sc2.objects[0].position == pytest.approx((-2.5, 0.0, 0.0))

    def test_csg_block_and_new_primitives(self):
        from cga.engine.csg import CsgGeometry
        from cga.scene_lang import SceneLoader

        sc, _ = SceneLoader.load(
            """
            difference() { box(s=[2,2,2]); cylinder(r=0.5, h=4); }
            cone(r=1, h=2);
            torus(R=1, r=0.3);
            ellipsoid(radii=[1,2,3]);
            extrude(profile=[[0,0],[1,0],[1,1],[0,1]], h=0.5);
            p1 = [[0,0],[1,0],[1,1],[0,1]];
            p2 = [[0.2,0.2],[0.8,0.2],[0.8,0.8],[0.2,0.8]];
            loft(profiles=[p1, p2], zs=[0, 0.5]);
            """
        )
        from cga.engine import (
            ConeGeometry,
            EllipsoidGeometry,
            MeshGeometry,
            TorusGeometry,
        )

        types = [type(o.geometry).__name__ for o in sc.objects]
        assert types[0] == "CsgGeometry"
        assert isinstance(sc.objects[0].geometry, CsgGeometry)
        assert isinstance(sc.objects[1].geometry, ConeGeometry)
        assert isinstance(sc.objects[2].geometry, TorusGeometry)
        assert isinstance(sc.objects[3].geometry, EllipsoidGeometry)
        assert isinstance(sc.objects[4].geometry, MeshGeometry)
        assert isinstance(sc.objects[5].geometry, MeshGeometry)

    def test_precision_statement(self):
        from cga.multivector import Multivector
        from cga.scene_lang import SceneLoader

        try:
            SceneLoader.load('precision("float64"); sphere(r=1);')
            assert Multivector.DTYPE == mx.float64
        finally:
            from cga.multivector import set_precision

            set_precision("float32")

    def test_mesh_requires_asset_root(self):
        from cga.scene_lang import SceneLoader

        with pytest.raises(ValueError, match="asset_root"):
            SceneLoader.load('mesh(file="x.obj");')
