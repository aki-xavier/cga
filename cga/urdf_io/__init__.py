"""URDF ⇄ CRDF 转换 (XML ↔ YAML), 每类一个文件。

- UrdfConverter.urdf_to_crdf(xml): 导入 URDF → CRDF YAML 文本。rpy
  (外旋 X-Y-Z) 经 Motor.from_matrix 归一化为 motor; 圆柱/盒/球直接映射
  (局部轴约定一致); mesh 不在 v1 范围 → 按 mesh_policy 处理。
- UrdfConverter.crdf_to_urdf(robot): 导出 → URDF XML 文本。Motor →
  to_matrix → xyz + rpy (matrix_to_rpy, 万向节锁取 γ=0); 每 role 成员
  生成一个 visual/collision 块 (URDF 的视觉/碰撞要分写, CRDF 里一份
  几何多角色复用)。

帧语义与 URDF 完全一致 (见 cga/robot 文档头); 单位 SI (米/千克/弧度)。
"""

from cga.urdf_io.urdf_converter import UrdfConverter

__all__ = ["UrdfConverter"]
