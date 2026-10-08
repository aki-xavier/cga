// orbit demo 场景的 JSX + CSS 版（与 examples/jsx/orbit.jsx 同构；
// 测试断言两者渲染逐字节相等）
import './orbit.css';

const Glass = ({ r }) => <sphere r={r} class="glass" />;

export default (
  <scene>
    <camera fov={50} aspect={4 / 3} position={[0, 2.4, 6.2]} target={[0, 0.8, 0]} />

    <plane n={[0, 1, 0]} d={0} class="ground" />

    <sphere r={1} class="red" t={[0, 1, 0]} />
    <sphere r={0.6} class="blue" t={[-2.2, 0.6, 0.5]} />
    <cylinder r={0.7} class="gold" t={[2.2, 0.7, -0.5]} />
    <box s={[0.9, 0.9, 0.9]} class="green" t={[0.8, 0.45, 1.8]} />
    <circle r={0.9} class="purple" rotate={[1, 0, 0, -0.4]} t={[-2.4, 2.2, 0.8]} />
    <group t={[0.4, 1.5, 2.6]}>
      <Glass r={0.8} />
    </group>

    <directional_light direction={[0.4, 1.0, 0.35]} intensity={0.38} />
    <point_light position={[0, 4, 3.5]} intensity={0.7} />
    <directional_light direction={[0, 0.35, 0.9]} intensity={0.18} />
    <ambient_light intensity={0.34} />
  </scene>
);
