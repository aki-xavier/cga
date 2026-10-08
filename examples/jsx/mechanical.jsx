// 机械装配体的 JSX 版（与 examples/jsx/mechanical.jsx 同构）
import './mechanical.css';

const Post = ({ r, h, class: cls }) => (
  <cylinder r={r} h={h} class={cls} rotate={[1, 0, 0, -Math.PI / 2]} />
);

const Flange = () => (
  <difference class="steel">
    <Post r={2.6} h={0.5} />
    {Array.from({ length: 8 }, (_, i) => (
      <group rotate={[0, 1, 0, (i * 2 * Math.PI) / 8]}>
        <group t={[2.0, 0, 0]}>
          <Post r={0.16} h={2.0} />
        </group>
      </group>
    ))}
    <Post r={0.58} h={2.0} />
  </difference>
);

const BoltCircle = ({ n, radius, y, boltR, boltH }) =>
  Array.from({ length: n }, (_, i) => (
    <group rotate={[0, 1, 0, (i * 2 * Math.PI) / n]}>
      <group t={[radius, y, 0]}>
        <Post r={boltR} h={boltH} class="bolt" />
        <box s={[boltR * 3.2, 0.18, boltR * 3.2]} class="bolt" t={[0, boltH / 2 + 0.09, 0]} />
      </group>
    </group>
  ));

const GearPart = ({ y, rHub, rTeeth, nTeeth, thick }) => (
  <scene>
    <group t={[0, y, 0]}>
      <Post r={rHub} h={thick} class="brass" />
    </group>
    {Array.from({ length: nTeeth }, (_, i) => (
      <group rotate={[0, 1, 0, (i * 2 * Math.PI) / nTeeth]}>
        <box s={[rTeeth - rHub + 0.12, thick, 0.22]} class="brass" t={[rHub + (rTeeth - rHub) / 2, y, 0]} />
      </group>
    ))}
  </scene>
);

const BasePlate = () => (
  <difference class="tread">
    <box s={[7.0, 0.3, 7.0]} />
    {Array.from({ length: 4 }, (_, i) => (
      <group rotate={[0, 1, 0, (i * Math.PI) / 2 + Math.PI / 4]}>
        <cone r={0.32} h={0.35} rotate={[1, 0, 0, Math.PI / 2]} t={[2.9, 0, 0]} />
      </group>
    ))}
  </difference>
);

export default (
  <scene>
    <camera fov={45} position={[7.5, 6.5, 9.5]} target={[0, 1.8, 0]} />
    <group t={[0, 0.15, 0]}>
      <BasePlate />
    </group>
    <group t={[0, 0.55, 0]}>
      <Flange />
    </group>
    <BoltCircle n={8} radius={2.0} y={0.55} boltR={0.13} boltH={0.7} />
    <group t={[0, 2.9, 0]}>
      <Post r={0.55} h={4.2} class="shaft" />
    </group>
    <group t={[0, 1.25, 0]}>
      <Post r={0.85} h={0.9} class="shaft" />
    </group>
    <GearPart y={2.6} rHub={1.15} rTeeth={1.75} nTeeth={16} thick={0.45} />
    <torus R={0.72} r={0.16} class="washer" rotate={[1, 0, 0, -Math.PI / 2]} t={[0, 4.45, 0]} />
    <box s={[0.95, 0.5, 0.95]} class="bolt" t={[0, 4.85, 0]} />
    <box s={[0.18, 0.9, 0.18]} class="key" t={[0.62, 3.6, 0]} />
    <plane n={[0, 1, 0]} d={0} class="ground" />
    <directional_light direction={[0.4, 1.0, 0.5]} intensity={0.5} />
    <point_light position={[-5, 6, 4]} intensity={0.55} />
    <ambient_light intensity={0.38} />
  </scene>
);
