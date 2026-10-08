// 图元全家福的 JSX 版（与 examples/jsx/primitives.jsx 同构）
import './primitives.css';

export default (
  <scene>
    <camera fov={50} position={[0, 4.4, 9.6]} target={[0, 0.2, -0.3]} />
    <directional_light direction={[0.4, 1.0, 0.35]} intensity={0.5} />
    <ambient_light intensity={0.35} />
    <plane n={[0, 1, 0]} d={0} class="ground" />

    <sphere r={0.62} class="red" t={[-3.3, 0.62, 1.0]} />
    <box s={[1.0, 1.0, 1.0]} class="green" rotate={[0, 1, 0, 0.5]} t={[-1.1, 0.5, 1.0]} />
    <cylinder r={0.44} h={1.2} class="gold" rotate={[1, 0, 0, -1.5707963]} t={[1.1, 0.6, 1.0]} />
    <cone r={0.55} h={1.2} class="blue" rotate={[1, 0, 0, -1.5707963]} t={[3.3, 0.6, 1.0]} />

    <torus R={0.55} r={0.24} class="purple" t={[-3.3, 0.24, -1.9]} />
    <ellipsoid radii={[0.7, 0.45, 0.55]} class="teal" t={[-1.1, 0.45, -1.9]} />
    <torus R={0.55} r={0.24} arc={4.2} class="pink" t={[-0.25, 0.72, -1.9]} />
    <cyclide a={0.5} b={0.49} d={0.45} class="orange" t={[3.3, 0.52, -2.1]} />
    <circle r={0.65} class="crimson" t={[1.4, 0.7, -1.9]} />
  </scene>
);
