// 运动副（kinematic pairs）——React 组件封装。
//
// 这些组件把“副类型”写进组件名。函数组件对协调器透明：它们最终仍落成
// {t:'joint' | 'gear' | 'cam', p, c} 节点，交给 Rust 侧 `joint_motion` / 齿轮关系 /
// `cam_solve` 处理——代数核心与求解保持单一来源（`scene_build/kinematics.rs`）。
// 因此报告、URDF 导出、pose 覆盖全都不受影响，只是作者侧多了一种写法。
//
// 用法与 `<joint>`/`<gear>`/`<cam>` 完全等价：
//   <Revolute name="a" axis={[0,0,1]} q={0.4}><sphere r={0.1} /></Revolute>
//   <Prismatic name="b" axis={[0,0,1]} limit={[-1,1]} q={0.2}><box /></Prismatic>
//   <Gear driver="a" driven="b" ratio={-0.5} />
//   <Cam driver="a" driven="b" driverProfile={…} drivenProfile={…} />
//
// 加载顺序：与 scene-prelude.js 同层注入，且在其之后（`h` 已就绪）。

// 关节副：注入 type，转发其余 props 与 children。children 以可变参数转发，
// 与 JSX 编译 `<joint>{a}{b}</joint>` 的形态一致（避免数组 child 的 key 警告）。
function __jointPair(type) {
  return function JointPair(props) {
    const rest = {};
    for (const k in props) {
      if (k !== 'children') rest[k] = props[k];
    }
    rest.type = type;
    const kids = props.children;
    if (kids === undefined || kids === null || kids === false) return h('joint', rest);
    if (Array.isArray(kids)) return h('joint', rest, ...kids);
    return h('joint', rest, kids);
  };
}

const Revolute = __jointPair('revolute');
const Continuous = __jointPair('continuous');
const Prismatic = __jointPair('prismatic');
const Helical = __jointPair('helical');
const Cylindrical = __jointPair('cylindrical');
const Spherical = __jointPair('spherical');
const Planar = __jointPair('planar');
const Fixed = __jointPair('fixed');

// 高副：齿轮副 / 凸轮副。纯转发（类型即 tag），使 API 与低副组件一致。
function __hostPair(tag) {
  return function HostPair(props) {
    const rest = {};
    for (const k in props) {
      if (k !== 'children') rest[k] = props[k];
    }
    const kids = props.children;
    if (kids === undefined || kids === null || kids === false) return h(tag, rest);
    if (Array.isArray(kids)) return h(tag, rest, ...kids);
    return h(tag, rest, kids);
  };
}

const Gear = __hostPair('gear');
const Cam = __hostPair('cam');
