// assembly 场景的 JSX + CSS 版（与 examples/jsx/assembly.jsx 同构：
// solve 约束、派生钻孔、面引用、tag 实例、集合计数门控）
import './assembly.css';

// 编译期约束: off = 0.8（等式残差 + 不等式 hinge）
const [off] = solve([0.5], [v => eq(2 * v[0], 1.6), v => le(v[0], 1.9)]);

// 几何值绑定: 板; 派生钻孔 through= 取目标包围盒
const plate = (
  <translate t={[0, 0.2, 0]}>
    <box s={[3.2, 0.4, 1.6]} />
  </translate>
);

export default (
  <scene>
    <camera fov={45} position={[4.2, 3.2, 6.0]} target={[0, 0.6, 0]} />
    <directional_light direction={[0.4, 1.0, 0.35]} intensity={0.5} />
    <ambient_light intensity={0.35} />
    <plane n={[0, 1, 0]} d={0} class="ground" />

    <difference class="plate">
      {plate}
      {[-1, 1].map((s) => (
        <translate t={[s * off, 0, 0]}>
          <drill r={0.16} through={plate} axis={1} />
        </translate>
      ))}
    </difference>

    {[-1, 1].map((s) => (
      <tag name="post">
        <translate t={vadd(face(plate, '+y'), [s * (off + 0.55), 0.5, 0])}>
          <rotate axis={[1, 0, 0]} angle={-Math.PI / 2}>
            <cylinder r={0.12} h={1.0} class="post" />
          </rotate>
        </translate>
      </tag>
    ))}

    <when of="post" count={2}>
      <translate t={vadd(center('post'), [0, 0.62, 0])}>
        <box s={[3.4, 0.24, 0.5]} class="beam" />
      </translate>
    </when>
  </scene>
);
