// 3×3 球阵列（与 examples/jsx/grid.jsx 同构）
import './grid.css';

const Bead = ({ x, z, r = 0.35 }) => (
  <translate t={[x, r, z]}>
    <sphere r={r} class="bead" />
  </translate>
);

export default (
  <scene>
    <camera fov={50} position={[3, 4, 7]} target={[1.2, 0.4, 1.2]} />
    <plane n={[0, 1, 0]} d={0} class="ground" />
    {[0, 1, 2].map((i) => [0, 1, 2].map((j) => <Bead x={i * 1.2} z={j * 1.2} />))}
    <directional_light direction={[0.4, 1.0, 0.35]} intensity={0.5} />
    <ambient_light intensity={0.35} />
  </scene>
);
