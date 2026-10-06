// 自由曲面的 JSX 版（与 examples/cgs/freeform.cgs 同构）
import './freeform.css';

const PTS = [
  [-1.5, -1.5, 0.175], [-1.5, -0.5, 0.275], [-1.5, 0.5, 0.275], [-1.5, 1.5, 0.175],
  [-0.5, -1.5, 0.275], [-0.5, -0.5, 0.375], [-0.5, 0.5, 0.375], [-0.5, 1.5, 0.275],
  [0.5, -1.5, 0.275], [0.5, -0.5, 0.375], [0.5, 0.5, 0.375], [0.5, 1.5, 0.275],
  [1.5, -1.5, 0.175], [1.5, -0.5, 0.275], [1.5, 0.5, 0.275], [1.5, 1.5, 0.175],
];

const hood = <bezier points={PTS} thickness={0.15} div={4} />;

export default (
  <scene>
    <camera fov={50} position={[0, 2.8, 5.6]} target={[0, 0.6, 0]} />
    <plane n={[0, 1, 0]} d={0} class="ground" />
    <translate t={[0, 1.35, 0]}>
      <rotate axis={[1, 0, 0]} angle={-1.5707963}>
        <material color={0xC0392B} roughness={0.3} metalness={0.2}>
          {hood}
        </material>
      </rotate>
    </translate>
    <difference>
      <translate t={[0, 0.35, 0]}>
        <box s={[3.2, 0.7, 3.2]} />
      </translate>
      <translate t={[0, 1.2, 0]}>
        <rotate axis={[1, 0, 0]} angle={-1.5707963}>
          <bezier points={PTS} thickness={2.0} div={2} />
        </rotate>
      </translate>
    </difference>
    <directional_light direction={[0.4, 1.0, 0.35]} intensity={0.5} />
    <ambient_light intensity={0.35} />
  </scene>
);
