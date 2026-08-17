"""CGS (CGA Scene) — OpenSCAD 风格的 CGA 场景描述语言。

设计: 声明式语句 + 前置修饰符 (OpenSCAD 同构), 单遍解析即求值,
产物 = cga.engine 的 (Scene, PerspectiveCamera)。

语法:

    // 行注释
    语句        := 修饰符* (调用 ';' | '{' 语句* '}') | 控制流 | 赋值
    修饰符      := translate([x,y,z]) | rotate(axis=[x,y,z], angle=弧度)
                  | scale(标量 或 [sx,sy,sz]) | mirror(axis=[x,y,z])
                  | material(color=0xRRGGBB, roughness=.., metalness=..,
                             emissive=.., opacity=.., ior=.., absorption=..,
                             unlit=true|false, map="贴图相对路径")
    调用        := 图元 | 灯光 | background(color=..) | camera(...)
                  | precision("float32"|"float64") | module 调用
    控制流      := for (i = 列表或range) 语句 | if (条件) 语句 [else 语句]
                  | union() 语句 (分组别名, 无布尔) | echo(表达式, ...);
    CSG         := difference() 语句块 | intersection() 语句块
                  (真布尔: 收集块内全部几何, 首子减余 / 全交; 单材质)
    赋值        := 名字 = 表达式;
    module 定义 := module 名(形参[=默认], ...) { 语句* }
    range       := [初:止] | [初:步:止]   (OpenSCAD 顺序, 止包含)

    图元:  sphere(r=1)  plane(n=[0,1,0], d=0)  cylinder(r=0.5[, h=2])
           box(s=[w,h,d])  circle(r=0.9)              // 圆盘, 局部法向 +Z
           cone(r, h)  torus(R, r)  ellipsoid(radii=[rx,ry,rz])
           cyclide(a=.., b=.., d=..)                  // Dupin cyclide (环型/尖型)
           extrude(profile=[[x,y],...], h=..)          // 沿 +Z 0..h
           loft(profiles=[...], zs=[...])              // 等点数多截面
           mesh(file="x.obj"|"x.glb"|"x.gltf")         // asset_root 相对
    灯光:  directional_light(direction=[..], intensity=.., color=..)
           point_light(position=[..], intensity=.., color=..)
           ambient_light(intensity=.., color=..)
    camera(fov=50, aspect=1.33, position=[..], target=[..])  // 可省略

表达式: 数字 / 0x 色值 / 向量 [..] / 字符串 ".." / 变量 / true false / pi;
  运算符 || && == != < <= > >= + - * / % 一元 - ! (括号分组);
  向量逐元素运算, 标量自动广播 ([1,2,3]*2);
  内建函数: abs sign sin cos tan asin acos atan atan2 sqrt exp ln log
           floor ceil round pow min max len norm cross。

作用域: 全局单层; {} 块不新开作用域 (块内赋值泄漏到外层); for 循环
变量写在包含作用域; module 体 = 全局 + 形参 (词法, 看不到调用点局部
变量)。变换/材质上下文在 module 调用点正常继承。

语义:
  - 修饰符对紧随的一条语句或 {} 块生效, 可嵌套; 右乘复合
    (translate(...) rotate(...) 物体 = 先旋后移, 与 OpenSCAD 一致)。
    上下文为全 4x4 仿射, 几何落点极分解为 motor·linear
    (rotate 与 scale/mirror 任意嵌套顺序均正确)。
  - scale/mirror 是渲染层仿射扩展 (非 versor 可达): 经
    AffineGeometry 射线逆变换实现, 见 cga/engine/affine_geometry.py。
  - material 缺省 = MeshStandardMaterial(); unlit=true → MeshBasicMaterial。
    材质按字段合并, 内层覆盖外层, 块外不泄漏。
  - 修饰符只作用于几何; 灯光/background/camera 出现在块内不吃变换。
  - CSG: 子节点须为实体 (sphere/box/cylinder/cone/torus/ellipsoid/
    cyclide/extrude/loft/mesh/plane 半空间; circle 非实体)。子节点材质
    被丢弃 —— 整个 CSG 节点共享当前材质 (单材质, 如实标注)。
  - cone/torus/cylinder 局部轴 = +Z; 竖直放置需
    rotate(axis=[1,0,0], angle=-pi/2)。
  - cyclide 是四次曲面 (非 blade), 经 Durand-Kerner 四次求根解析求交;
    环型 (c<d<a) 是光滑亏格-1 曲面, 尖型自交 (CSG 成员性语义退化)。
  - precision("float64") 切换代数核心到 CPU float64 (大坐标场景,
    请在几何创建前调用); 渲染内核仍 float32。
  - v3 范围 (如实标注): 无字符串变量/include/children, 无 $fn
    (解析图元无限光滑), CSG 相切/共面退化配置依赖 δ=1e-4 采样
    (见 cga/engine/csg.py), mesh 的 glTF 暂限单 primitive。

示例见 examples/orbit.cgs / grid.cgs (for/module 阵列) /
building.cgs (CSG 开窗 + 纹理) / mechanical.cgs (CSG 钻孔装配);
渲染: uv run python -m cga.scene_lang <file.cgs>
"""

from cga.scene_lang.scene_loader import SceneLoader

__all__ = ["SceneLoader"]
