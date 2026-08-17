"""three.js 风格的三维渲染引擎: CGA 建模核心 + MLX/Metal 批量光线追踪。

API 对齐 three.js: Scene / PerspectiveCamera / Mesh / *Geometry /
MeshStandardMaterial / MeshBasicMaterial / AmbientLight /
DirectionalLight / PointLight / Renderer.render(scene, camera) /
OrbitControls。每类一个文件。

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
  - 线性空间光照 + 输出端 sRGB 编码 (roundtrip 恒等); >1 高光硬截断
    (无 tonemap); 无纹理/后处理。
  - 阴影: 每光源一条遮挡射线 (硬阴影); 透明遮挡物按 1−opacity 透光
    (忽略 Fresnel); 无软阴影 (面光源留作升级路径)。
  - 透明面: 批量 Whitted 递归 (精确非偏振 Fresnel 分裂反射/折射,
    Beer 吸收, max_depth 截断); 介质追踪假设透明体互不重叠。
  - Object3D 可选 linear (3x3): scale/mirror/shear 经 AffineGeometry
    射线逆变换实现 (非 versor 可达, 如实标注为渲染层仿射扩展)。
  - CSG: CsgGeometry(union/intersection/difference) 递归布尔,
    叶子 = 实体图元 (sphere/box/cylinder/cone/torus/ellipsoid/plane
    半空间), 见 cga/engine/csg.py 文档头。
  - cone/torus/ellipsoid 非 CGA blade, 经射线逆变换解析求交。
  - 无限平面/圆柱: v1 默认语义; 有限圆柱 (带端盖) 经 CylinderGeometry
    length 参数支持; 相机在柱内等退化情形按内核处理。
  - 精度: 默认 float32 (blade 共轭场景坐标宜 ±20); set_precision
    ("float64") 后代数核心走 CPU float64, 坐标可远超 ±20 (渲染内核
    仍 float32, 参数进相机空间后 near-origin)。
  - 每帧 Python 层循环图元 (~10 个), 像素级全在 MLX GPU 上批量。
"""

from cga.engine.affine_geometry import AffineGeometry
from cga.engine.ambient_light import AmbientLight
from cga.engine.box_geometry import BoxGeometry
from cga.engine.circle_geometry import CircleGeometry
from cga.engine.color import Color
from cga.engine.cone_geometry import ConeGeometry
from cga.engine.csg import CsgGeometry
from cga.engine.cylinder_geometry import CylinderGeometry
from cga.engine.directional_light import DirectionalLight
from cga.engine.ellipsoid_geometry import EllipsoidGeometry
from cga.engine.geometry_base import GeometryBase
from cga.engine.light_base import LightBase
from cga.engine.material import Material
from cga.engine.mesh import Mesh
from cga.engine.mesh_basic_material import MeshBasicMaterial
from cga.engine.mesh_standard_material import MeshStandardMaterial
from cga.engine.object3d import Object3D
from cga.engine.orbit_controls import OrbitControls
from cga.engine.perspective_camera import PerspectiveCamera
from cga.engine.plane_geometry import PlaneGeometry
from cga.engine.point_light import PointLight
from cga.engine.renderer import Renderer
from cga.engine.scene import Scene
from cga.engine.sphere_geometry import SphereGeometry
from cga.engine.texture import Texture
from cga.engine.torus_geometry import TorusGeometry
from cga.engine.trimesh_geometry import MeshGeometry
from cga.engine.vec3 import Vec3

__all__ = [
    "AffineGeometry",
    "AmbientLight",
    "BoxGeometry",
    "CircleGeometry",
    "Color",
    "ConeGeometry",
    "CsgGeometry",
    "CylinderGeometry",
    "DirectionalLight",
    "EllipsoidGeometry",
    "GeometryBase",
    "LightBase",
    "Material",
    "Mesh",
    "MeshBasicMaterial",
    "MeshGeometry",
    "MeshStandardMaterial",
    "Object3D",
    "OrbitControls",
    "PerspectiveCamera",
    "PlaneGeometry",
    "PointLight",
    "Renderer",
    "Scene",
    "SphereGeometry",
    "Texture",
    "TorusGeometry",
    "Vec3",
]
