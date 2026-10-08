// 人脸场景：解析图元 + CSG 组合建模（无网格）。
// 头 = 椭球；眼窝 = difference 挖坑；虹膜/瞳孔 = intersection 球冠；
// 眉/耳 = torus arc<2π 弧管；唇/鼻/翼 = 椭球叠加；发 = 大椭球 − 头 − 面部盒。
import './face.css';

const eye = (s) => {
  const ex = s * 0.24;
  const ey = 1.66;
  const ez = 0.43;
  return [
    // 巩膜（眼球）
    <sphere key={`sclera${s}`} r={0.13} t={[ex, ey, ez]} class="sclera" />,
    // 虹膜：球冠（r=0.132 的球 ∩ 半空间盒，切面 z≥0.5775 → 冠半径 0.060）
    // CSG 块的材质取自身元素的 class，不下探子元素。
    <intersection key={`iris${s}`} class="iris">
      <sphere r={0.132} t={[ex, ey, ez]} />
      <box s={[0.5, 0.5, 0.5]} t={[ex, ey, ez + 0.3675]} />
    </intersection>,
    // 瞳孔：更小的前冠（切面 z≥0.5917 → 冠半径 0.025），叠在虹膜前 0.002
    <intersection key={`pupil${s}`} class="pupil">
      <sphere r={0.134} t={[ex, ey, ez]} />
      <box s={[0.5, 0.5, 0.5]} t={[ex, ey, ez + 0.3817]} />
    </intersection>,
  ];
};

const brow = (s) => (
  // 弧管眉
  <group key={`brow${s}`} t={[s * 0.23, 1.685, 0.6]}>
    <group rotate={[0, 1, 0, s * 0.35]}>
      <torus R={0.16} r={0.028} arc={2.4} class="brow" rotate={[0, 0, 1, 0.37]} />
    </group>
  </group>
);

const ear = (s) => (
  <group key={`ear${s}`} t={[s * 0.645, 1.5, 0.04]}>
    <group rotate={[0, 1, 0, (s * Math.PI) / 2]}>
      <torus R={0.135} r={0.035} arc={4.6} class="skin" rotate={[0, 0, 1, s > 0 ? Math.PI : 0]} />
    </group>
  </group>
);

export default (
  <scene>
    <camera fov={40} position={[0, 1.6, 3.4]} target={[0, 1.45, 0]} />
    <directional_light direction={[0.4, 1.0, 0.35]} intensity={0.5} />
    <point_light position={[-1.8, 2.2, 2.4]} intensity={0.25} />
    <ambient_light intensity={0.35} />
    <plane n={[0, 1, 0]} d={0} class="ground" />

    {/* 头：椭球挖双眼窝 */}
    <difference class="skin">
      <ellipsoid radii={[0.62, 0.78, 0.66]} t={[0, 1.62, 0]} />
      <sphere r={0.16} t={[-0.24, 1.66, 0.5]} />
      <sphere r={0.16} t={[0.24, 1.66, 0.5]} />
    </difference>

    {[-1, 1].flatMap(eye)}
    {[-1, 1].map(brow)}
    {[-1, 1].map(ear)}

    {/* 鼻：桥+尖+翼 椭球叠加，鼻孔暗球 */}
    <ellipsoid radii={[0.1, 0.24, 0.14]} t={[0, 1.52, 0.56]} rotate={[1, 0, 0, -0.3]} class="skin" />
    <sphere r={0.075} t={[0, 1.3, 0.645]} class="skin" />
    <sphere r={0.07} t={[-0.07, 1.27, 0.6]} class="skin" />
    <sphere r={0.07} t={[0.07, 1.27, 0.6]} class="skin" />
    <sphere r={0.033} t={[-0.05, 1.235, 0.635]} class="nostril" />
    <sphere r={0.033} t={[0.05, 1.235, 0.635]} class="nostril" />

    {/* 口：上唇/下唇/口缝 椭球 */}
    <ellipsoid radii={[0.16, 0.035, 0.06]} t={[0, 1.125, 0.475]} class="lips" />
    <ellipsoid radii={[0.15, 0.04, 0.065]} t={[0, 1.06, 0.43]} class="lips" />
    <ellipsoid radii={[0.13, 0.012, 0.055]} t={[0, 1.092, 0.485]} class="mouthline" />

    {/* 发：大椭球 − 头椭球 → 贴头薄壳；− 面部盒（露脸+发际线）
        − 低位盒（短发）− 耳袋球（露出耳朵） */}
    <difference class="hair">
      <ellipsoid radii={[0.655, 0.815, 0.695]} t={[0, 1.64, -0.015]} />
      <ellipsoid radii={[0.62, 0.78, 0.66]} t={[0, 1.62, 0]} />
      <box s={[2, 2, 2]} t={[0, 0.9, 1.2]} />
      <box s={[4, 4, 6]} t={[0, -0.5, -2]} />
      <sphere r={0.17} t={[-0.65, 1.5, 0.04]} />
      <sphere r={0.17} t={[0.65, 1.5, 0.04]} />
    </difference>

    {/* 颈与肩：t 在内层 rotate 外（world = T·R·geo） */}
    <cylinder r={0.24} h={1.0} class="skin" rotate={[1, 0, 0, -Math.PI / 2]} t={[0, 0.7, -0.06]} />
    <ellipsoid radii={[0.95, 0.35, 0.42]} t={[0, 0.35, -0.05]} class="shirt" />
  </scene>
);
