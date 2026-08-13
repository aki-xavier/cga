"""CRDF — CGA Robot Description Format: 机器人描述 (YAML, 非 XML), 每类一个文件。

URDF 的 CGA 版: 链接树 + 关节 + 几何 (blade) + 惯量, 用 YAML 表达。

与 URDF 的对应关系 (帧语义完全一致, 见 README "CRDF" 节):
  - joint.origin: 父 link frame → joint frame 的刚体变换, 两种写法等价:
      {xyz: [..], rpy: [..]}   (URDF 兼容, rpy = 外旋 X-Y-Z: R = Rz·Ry·Rx)
      {motor: {axis, angle, t}} (CGA 签名, Motor(axis, angle, t) 原样)
    载入时统一归一化为 Motor。
  - joint.axis: 表达在 joint frame (与 URDF 相同)。
  - link.geometry[].origin: 表达在 link frame。
  - link.inertial.com: 质心, 表达在 link frame。
  - 单位: 米/千克/弧度 (SI)。

CGA 特色 (URDF 没有的):
  - 几何是 blade: cylinder/box/sphere/plane/circle, 隐式精确, 无网格。
  - 同一几何可多角色复用: role: [visual, collision] (URDF 要写两遍)。
  - FK 走 Motor 链: M_child = M_parent · M_origin · Rot(axis, q) ——
    长链不积累矩阵正交性漂移 (versor 连乘保真)。

范围声明 (v1):
  - mesh 引用: v1 不支持, 转换时按策略跳过 (urdf_to_crdf(mesh_policy=...))。
  - 惯量: 全 6 分量张量 (ixx/iyy/izz/ixy/ixz/iyz)。
  - 无 SRDF/transmission/gazebo 语义 (纯运动学描述)。

关节类型常量是 Joint 的类属性 (Joint.REVOLUTE / Joint.MOVABLE / ...);
几何类型常量是 Geometry 的类属性 (Geometry.BLADES / Geometry.ROLES)。
解析入口: RobotLoader.load; 旋转换算: Rotation。
"""

from cga.robot.geometry import Geometry
from cga.robot.inertial import Inertial
from cga.robot.joint import Joint
from cga.robot.link import Link
from cga.robot.material import Material
from cga.robot.robot import Robot
from cga.robot.robot_error import RobotError
from cga.robot.robot_loader import RobotLoader
from cga.robot.rotation import Rotation

__all__ = [
    "Geometry",
    "Inertial",
    "Joint",
    "Link",
    "Material",
    "Robot",
    "RobotError",
    "RobotLoader",
    "Rotation",
]
