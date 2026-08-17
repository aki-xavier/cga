"""建模构建器 —— 2D 轮廓 → 三角网格 (挤出/放样) 的纯数据变换。

产出 (vertices, faces) 契约, 供 cga.engine.MeshGeometry 消费。
不依赖 MLX/引擎, 可独立测试。

约定:
  - 轮廓 = [(x, y), ...] 逆时针 (CCW) 点列 (顺时针自动反转);
  - extrude 沿 +Z 从 0 到 h (OpenSCAD linear_extrude 约定);
  - loft 给若干等点数截面与其 z 高度, 相邻截面间生成侧壁;
  - 端盖三角化用耳切法 (支持凹轮廓; 不支持带孔轮廓 —— 孔洞用
    CSG difference 实现, 见 cga/engine/csg.py)。
"""

from cga.modeling.builders import extrude, loft
from cga.modeling.earclip import triangulate

__all__ = ["extrude", "loft", "triangulate"]
