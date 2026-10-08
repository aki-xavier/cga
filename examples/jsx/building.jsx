// 参数化板式办公楼的 JSX 版（与 examples/jsx/building.jsx 同构）
import './building.css';

const Storey = ({ y, width, depth, floorH, cols, bay }) => {
  const slabT = 0.35;
  const winW = bay - 1.1;
  const winH = floorH - 1.3;
  const js = Array.from({ length: cols }, (_, j) => j);
  return (
    <scene>
      <box s={[width, slabT, depth]} class="slab" t={[0, y, 0]} />
      {[-1, 1].map((side) => {
        const z = (side * depth) / 2;
        return (
          <scene>
            <difference class="brick" t={[0, y + floorH / 2, z]}>
              <box s={[width, floorH, 0.24]} />
              {js.map((j) => (
                <box s={[winW, winH, 0.6]} t={[-width / 2 + (j + 0.5) * bay, 0.15, 0]} />
              ))}
            </difference>
            {js.map((j) => (
              <box
                s={[winW - 0.1, winH - 0.1, 0.06]}
                class="glass"
                t={[-width / 2 + (j + 0.5) * bay, y + floorH / 2 + 0.15, z]}
              />
            ))}
          </scene>
        );
      })}
      {Array.from({ length: cols + 1 }, (_, j) => -width / 2 + j * bay).map((x) => (
        <scene>
          <box s={[0.45, floorH, 0.45]} class="slab" t={[x, y + floorH / 2, depth / 2]} />
          <box s={[0.45, floorH, 0.45]} class="slab" t={[x, y + floorH / 2, -depth / 2]} />
        </scene>
      ))}
    </scene>
  );
};

const Parapet = ({ top, width, depth }) => (
  <scene>
    <box s={[width, 0.6, 0.3]} class="slab" t={[0, top + 0.3, depth / 2 - 0.15]} />
    <box s={[width, 0.6, 0.3]} class="slab" t={[0, top + 0.3, -depth / 2 + 0.15]} />
    <box s={[0.3, 0.6, depth]} class="slab" t={[width / 2 - 0.15, top + 0.3, 0]} />
    <box s={[0.3, 0.6, depth]} class="slab" t={[-width / 2 + 0.15, top + 0.3, 0]} />
  </scene>
);

const Tree = ({ x, z, s = 1.0 }) => (
  <scene>
    <cylinder r={0.12 * s} h={1.8 * s} class="trunk" rotate={[1, 0, 0, -Math.PI / 2]} t={[x, 0.9 * s, z]} />
    <sphere r={0.9 * s} class="crown" t={[x, 2.1 * s, z]} />
  </scene>
);

const floors = 4;
const floorH = 3.0;
const cols = 5;
const bay = 2.6;
const depth = 8.0;
const width = cols * bay;

export default (
  <scene>
    <camera fov={55} position={[15, 11, 20]} target={[0, 5.5, 0]} />
    {Array.from({ length: floors }, (_, i) => (
      <Storey y={i * floorH} width={width} depth={depth} floorH={floorH} cols={cols} bay={bay} />
    ))}
    <Parapet top={floors * floorH} width={width} depth={depth} />
    <box s={[2.2, 1.8, 3.0]} class="penthouse" t={[width / 4, floors * floorH + 0.9, 0]} />
    <box s={[5.0, 0.15, 2.0]} class="canopy" t={[0, floorH + 0.1, depth / 2 + 1.0]} />
    <box s={[3.2, floorH - 0.6, 0.8]} class="entrance" t={[0, floorH / 2, depth / 2 + 0.4]} />
    <plane n={[0, 1, 0]} d={0} class="ground" />
    <Tree x={9.5} z={4.0} />
    <Tree x={9.5} z={-2.0} s={0.85} />
    <Tree x={-8.5} z={5.0} s={1.1} />
    <directional_light direction={[0.5, 1.0, 0.45]} intensity={0.55} />
    <ambient_light intensity={0.42} />
  </scene>
);
