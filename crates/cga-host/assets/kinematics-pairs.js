// 运动副（kinematic pairs）——React 组件封装（图模型版，docs/kinematics-graph.md）。
//
// 这些组件把"副类型"写进组件名。函数组件对协调器透明：它们最终落成
// {t:'pair' | 'gear' | 'cam', p, c} 节点，交给 Rust 侧图求解（solve_graph）
// 处理——代数核心与求解保持单一来源（`scene_build/kinematics.rs`）。
//
// 图模型语义：pair 是两体之间的**无向约束**（a/b 对称），不是容器——
// 组件不再接收 children（约束没有子内容；几何写在自己的 <link> 里）。
//
// 用法：
//   <link name="base" /><anchor link="base" />
//   <link name="arm"><cylinder r={0.05} h={1} /></link>
//   <Revolute name="elbow" a="base" b="arm" axis={[0,1,0]} at={[0,0,1]} limit={[-1.5,1.5]} />
//   <Gear a="drive" b="driven" ratio={-0.5} />
//   <Cam a="cam_a" b="cam_b" aProfile={…} bProfile={…} />
//
// 加载顺序：与 scene-prelude.js 同层注入，且在其之后（`h` 已就绪）。

// 低副：kind 写进组件名，props 原样转发（kind 优先，防止作者覆盖）。
function __pairOf(kind) {
  return function PairComponent(props) {
    const rest = {};
    for (const k in props) {
      if (k !== 'children') rest[k] = props[k];
    }
    rest.kind = kind;
    return h('pair', rest);
  };
}

const Revolute = __pairOf('revolute');
const Continuous = __pairOf('continuous');
const Prismatic = __pairOf('prismatic');
const Helical = __pairOf('helical');
const Cylindrical = __pairOf('cylindrical');
const Spherical = __pairOf('spherical');
const Planar = __pairOf('planar');
const Fixed = __pairOf('fixed');

// 高副：齿轮副 / 凸轮副。纯转发（类型即 tag），使 API 与低副组件一致。
function __hostPair(tag) {
  return function HostPair(props) {
    const rest = {};
    for (const k in props) {
      if (k !== 'children') rest[k] = props[k];
    }
    return h(tag, rest);
  };
}

const Gear = __hostPair('gear');
const Cam = __hostPair('cam');
