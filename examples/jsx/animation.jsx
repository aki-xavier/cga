// 逐帧动画场景：宿主输入 `IN.t ∈ [0,1)` 驱动（React context），hooks 保有跨帧状态。
// 渲染：cargo run --release -p cga-examples --bin render_frames -- examples/jsx/animation.jsx /tmp/anim 320 240 2 6
import './animation.css';

const { useContext, useState, useEffect } = React;

function Orbiter({ phase, radius, cls, lift }) {
  const IN = useContext(HostInput);
  const t = IN.t === undefined ? 0 : IN.t;
  const a = phase + t * Math.PI * 2;
  return (
    <translate t={[Math.cos(a) * 1.9, lift, Math.sin(a) * 1.9]}>
      <sphere r={radius} class={cls} />
    </translate>
  );
}

// 输入每变一次（每帧）计数 +1：证明 hook 状态跨帧保留、副作用在 drain 中执行。
function FrameCounter() {
  const IN = useContext(HostInput);
  const [frames, setFrames] = useState(0);
  useEffect(() => {
    setFrames((n) => n + 1);
  }, [IN.t]);
  return (
    <translate t={[-2.6, 1.2, 0]}>
      <box s={[0.3 + frames * 0.25, 0.25, 0.25]} class="counter" />
    </translate>
  );
}

export default (
  <scene>
    <camera fov={50} position={[0, 3.4, 7.2]} target={[0, 0.9, 0]} />
    <directional_light direction={[0.4, 1.0, 0.3]} intensity={0.6} />
    <ambient_light intensity={0.35} />
    <plane n={[0, 1, 0]} d={0} class="ground" />
    <translate t={[0, 0.55, 0]}>
      <sphere r={0.6} class="sun" />
    </translate>
    <Orbiter phase={0} radius={0.3} cls="moon" lift={0.5} />
    <Orbiter phase={2.1} radius={0.22} cls="moon2" lift={0.85} />
    <FrameCounter />
  </scene>
);
