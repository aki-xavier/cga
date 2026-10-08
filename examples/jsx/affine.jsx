// 仿射扩展的 JSX 版（与 examples/jsx/affine.jsx 同构）
import './affine.css';

const Body = () => (
  <difference class="red">
    <box s={[1.0, 1.0, 1.0]} t={[2.3, 0.5, -2.2]} />
    <cylinder r={0.26} h={1.6} t={[2.58, 0.55, -2.2]} />
  </difference>
);

const Marker = () => (
  <sphere r={0.16} class="gold" t={[2.72, 1.12, -1.85]} />
);

export default (
  <scene>
    <camera fov={45} position={[0, 3.8, 10.6]} target={[0, 0.3, -0.2]} />
    <directional_light direction={[0.4, 1.0, 0.35]} intensity={0.5} />
    <ambient_light intensity={0.35} />
    <plane n={[0, 1, 0]} d={0} class="ground" />

    <box s={[0.9, 0.9, 0.9]} class="gray" t={[-3.2, 0.45, 1.5]} />
    <box s={[0.9, 0.9, 0.9]} class="green" scale={[0.6, 1.6, 0.6]} t={[-1.4, 0.72, 1.5]} />
    <group rotate={[0, 1, 0, 0.6]} t={[1.1, 0.21, 1.5]}>
      <box s={[0.9, 0.9, 0.9]} class="blue" scale={[1.5, 0.45, 0.8]} />
    </group>

    <Body />
    <Marker />
    <group mirror={[1, 0, 0]}>
      <Body />
    </group>
    <group mirror={[1, 0, 0]}>
      <Marker />
    </group>
  </scene>
);
