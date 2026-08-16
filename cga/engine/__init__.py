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
  - Object3D 无 scale: motor 是刚体变换, 尺寸全走 geometry 参数。
  - 无限平面/圆柱: v1 默认语义; 有限圆柱 (带端盖) 经 CylinderGeometry
    length 参数支持; 相机在柱内等退化情形按内核处理。
  - float32: blade 共轭在 float32 下进行, 场景坐标宜控制在 ±20 内
    (远原点 conformal 抵消是本包已知限制)。
  - 每帧 Python 层循环图元 (~10 个), 像素级全在 MLX GPU 上批量。
"""

from cga.engine.ambient_light import AmbientLight
from cga.engine.box_geometry import BoxGeometry
from cga.engine.circle_geometry import CircleGeometry
from cga.engine.color import Color
from cga.engine.cylinder_geometry import CylinderGeometry
from cga.engine.directional_light import DirectionalLight
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
from cga.engine.vec3 import Vec3

__all__ = [
    "AmbientLight",
    "BoxGeometry",
    "CircleGeometry",
    "Color",
    "CylinderGeometry",
    "DirectionalLight",
    "GeometryBase",
    "LightBase",
    "Material",
    "Mesh",
    "MeshBasicMaterial",
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
    "Vec3",
]
