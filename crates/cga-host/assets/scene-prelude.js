
const h = React.createElement;
const Fragment = React.Fragment;

const Sphere='sphere',Plane='plane',Cylinder='cylinder',Box='box',Circle='circle',Cone='cone',
Torus='torus',Cyclide='cyclide',Ellipsoid='ellipsoid',Translate='translate',Rotate='rotate',Scale='scale',Mirror='mirror',
Material='material',Union='union',Difference='difference',Intersection='intersection',
AmbientLight='ambient_light',DirectionalLight='directional_light',PointLight='point_light',
Camera='camera',Background='background',Scene='scene',Joint='joint',
Tag='tag';
const Drill='drill',Instances='instances',When='when',Group='group';
const __isQ = (v) => v && typeof v === 'object' && !Array.isArray(v) && v.__q;
const vadd = (a, b) => (__isQ(a) || __isQ(b)) ? { __q: 'vadd', a, b } : a.map((x, i) => x + b[i]);
const vsub = (a, b) => (__isQ(a) || __isQ(b)) ? { __q: 'vsub', a, b } : a.map((x, i) => x - b[i]);
const vscale = (a, s) => __isQ(a) ? { __q: 'vscale', a, s } : a.map((x) => x * s);
const face = (of, key) => ({ __q: 'face', of, key });
const fnrm = (of, key) => ({ __q: 'fnrm', of, key });
const center = (of) => ({ __q: 'center', of });
const lo = (of) => ({ __q: 'lo', of });
const hi = (of) => ({ __q: 'hi', of });
const size = (of) => ({ __q: 'size', of });
const xdir = (of) => ({ __q: 'xdir', of });
const ydir = (of) => ({ __q: 'ydir', of });
const zdir = (of) => ({ __q: 'zdir', of });
const polar = (r, a) => [r * Math.cos(a), r * Math.sin(a), 0];
const instances = (name) => h('instances', { of: name });
const eq = (a, b) => a - b;
const le = (a, b) => Math.max(0, a - b);
const ge = (a, b) => Math.max(0, b - a);
