"""CGS (CGA Scene) — OpenSCAD 风格的 CGA 场景描述语言。

设计: 声明式语句 + 前置修饰符 (OpenSCAD 同构), 单遍解析即求值,
产物 = cga.engine 的 (Scene, PerspectiveCamera)。

语法:

    // 行注释
    语句        := 修饰符* (调用 ';' | '{' 语句* '}') | 控制流 | 赋值
    修饰符      := translate([x,y,z]) | rotate(axis=[x,y,z], angle=弧度)
                  | material(color=0xRRGGBB, roughness=.., metalness=..,
                             emissive=.., opacity=.., ior=.., absorption=..,
                             unlit=true|false)
    调用        := 图元 | 灯光 | background(color=..) | camera(...) | module 调用
    控制流      := for (i = 列表或range) 语句 | if (条件) 语句 [else 语句]
                  | union() 语句 (分组别名, 无布尔) | echo(表达式, ...);
    赋值        := 名字 = 表达式;
    module 定义 := module 名(形参[=默认], ...) { 语句* }
    range       := [初:止] | [初:步:止]   (OpenSCAD 顺序, 止包含)

    图元:  sphere(r=1)  plane(n=[0,1,0], d=0)  cylinder(r=0.5[, h=2])
           box(s=[w,h,d])  circle(r=0.9)              // 圆盘, 局部法向 +Z
    灯光:  directional_light(direction=[..], intensity=.., color=..)
           point_light(position=[..], intensity=.., color=..)
           ambient_light(intensity=.., color=..)
    camera(fov=50, aspect=1.33, position=[..], target=[..])  // 可省略

表达式: 数字 / 0x 色值 / 向量 [..] / 变量 / true false / pi;
  运算符 || && == != < <= > >= + - * / % 一元 - ! (括号分组);
  向量逐元素运算, 标量自动广播 ([1,2,3]*2);
  内建函数: abs sign sin cos tan asin acos atan atan2 sqrt exp ln log
           floor ceil round pow min max len norm cross。

作用域: 全局单层; {} 块不新开作用域 (块内赋值泄漏到外层); for 循环
变量写在包含作用域; module 体 = 全局 + 形参 (词法, 看不到调用点局部
变量)。motor/材质上下文在 module 调用点正常继承。

语义:
  - 修饰符对紧随的一条语句或 {} 块生效, 可嵌套; 变换左乘复合
    (translate 在外先平移语义: translate(...) rotate(...) 物体 = 先旋后移,
    与 Motor T·R 一致)。材质按字段合并, 内层覆盖外层, 块外不泄漏。
  - material 缺省 = MeshStandardMaterial(); unlit=true → MeshBasicMaterial。
  - 修饰符只作用于几何; 灯光/background/camera 出现在块内不吃变换。
  - v2 范围 (如实标注): 无 CSG 布尔 (difference/intersection 需渲染器
    射线区间改造, 留升级路径), 无 scale/mirror (非刚体变换, 非 versor
    可达), 无字符串/include/children, 无 $fn (解析图元无限光滑)。

语义:
  - 修饰符对紧随的一条语句或 {} 块生效, 可嵌套; 变换左乘复合
    (translate 在外先平移语义: translate(...) rotate(...) 物体 = 先旋后移,
    与 Motor T·R 一致)。材质按字段合并, 内层覆盖外层, 块外不泄漏。
  - material 缺省 = MeshStandardMaterial(); unlit=true → MeshBasicMaterial。
  - 修饰符只作用于几何; 灯光/background/camera 出现在块内不吃变换。

示例见 examples/orbit.cgs 与 examples/grid.cgs (for/module 阵列);
渲染: uv run python -m cga.scene_lang <file.cgs>
"""

from cga.scene_lang.scene_loader import SceneLoader

__all__ = ["SceneLoader"]
