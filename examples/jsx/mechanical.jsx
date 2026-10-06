// 机械装配体的 JSX 版（与 examples/jsx/mechanical.jsx 同构）
import './mechanical.css';

const Post = ({ r, h, class: cls }) => (
  <rotate axis={[1, 0, 0]} angle={-Math.PI / 2}>
    <cylinder r={r} h={h} class={cls} />
  </rotate>
);

const Flange = () => (
  <difference class="steel">
    <Post r={2.6} h={0.5} />
    {Array.from({ length: 8 }, (_, i) => (
      <rotate axis={[0, 1, 0]} angle={(i * 2 * Math.PI) / 8}>
        <translate t={[2.0, 0, 0]}>
          <Post r={0.16} h={2.0} />
        </translate>
      </rotate>
    ))}
    <Post r={0.58} h={2.0} />
  </difference>
);

const BoltCircle = ({ n, radius, y, boltR, boltH }) =>
  Array.from({ length: n }, (_, i) => (
    <rotate axis={[0, 1, 0]} angle={(i * 2 * Math.PI) / n}>
      <translate t={[radius, y, 0]}>
        <Post r={boltR} h={boltH} class="bolt" />
        <translate t={[0, boltH / 2 + 0.09, 0]}>
          <box s={[boltR * 3.2, 0.18, boltR * 3.2]} class="bolt" />
        </translate>
      </translate>
    </rotate>
  ));

const GearPart = ({ y, rHub, rTeeth, nTeeth, thick }) => (
  <scene>
    <translate t={[0, y, 0]}>
      <Post r={rHub} h={thick} class="brass" />
    </translate>
    {Array.from({ length: nTeeth }, (_, i) => (
      <rotate axis={[0, 1, 0]} angle={(i * 2 * Math.PI) / nTeeth}>
        <translate t={[rHub + (rTeeth - rHub) / 2, y, 0]}>
          <box s={[rTeeth - rHub + 0.12, thick, 0.22]} class="brass" />
        </translate>
      </rotate>
    ))}
  </scene>
);

const BasePlate = () => (
  <difference class="tread">
    <box s={[7.0, 0.3, 7.0]} />
    {Array.from({ length: 4 }, (_, i) => (
      <rotate axis={[0, 1, 0]} angle={(i * Math.PI) / 2 + Math.PI / 4}>
        <translate t={[2.9, 0, 0]}>
          <rotate axis={[1, 0, 0]} angle={Math.PI / 2}>
            <cone r={0.32} h={0.35} />
          </rotate>
        </translate>
      </rotate>
    ))}
  </difference>
);

export default (
  <scene>
    <camera fov={45} position={[7.5, 6.5, 9.5]} target={[0, 1.8, 0]} />
    <translate t={[0, 0.15, 0]}>
      <BasePlate />
    </translate>
    <translate t={[0, 0.55, 0]}>
      <Flange />
    </translate>
    <BoltCircle n={8} radius={2.0} y={0.55} boltR={0.13} boltH={0.7} />
    <translate t={[0, 2.9, 0]}>
      <Post r={0.55} h={4.2} class="shaft" />
    </translate>
    <translate t={[0, 1.25, 0]}>
      <Post r={0.85} h={0.9} class="shaft" />
    </translate>
    <GearPart y={2.6} rHub={1.15} rTeeth={1.75} nTeeth={16} thick={0.45} />
    <translate t={[0, 4.45, 0]}>
      <rotate axis={[1, 0, 0]} angle={-Math.PI / 2}>
        <torus R={0.72} r={0.16} class="washer" />
      </rotate>
    </translate>
    <translate t={[0, 4.85, 0]}>
      <box s={[0.95, 0.5, 0.95]} class="bolt" />
    </translate>
    <translate t={[0.62, 3.6, 0]}>
      <box s={[0.18, 0.9, 0.18]} class="key" />
    </translate>
    <plane n={[0, 1, 0]} d={0} class="ground" />
    <directional_light direction={[0.4, 1.0, 0.5]} intensity={0.5} />
    <point_light position={[-5, 6, 4]} intensity={0.55} />
    <ambient_light intensity={0.38} />
  </scene>
);
