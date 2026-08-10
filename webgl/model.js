// CRDF YAML → 渲染模型 (镜像 cga/robot.py 的载入语义)。
import { Motor } from "./cga.js";

function originToMotor(o) {
  if (!o) return Motor.identity();
  if (o.motor) {
    return Motor.fromAxisAngle(o.motor.axis, o.motor.angle, o.motor.t || [0, 0, 0]);
  }
  return Motor.fromRPY(o.rpy || [0, 0, 0], o.xyz || [0, 0, 0]);
}

function geomParams(g) {
  switch (g.blade) {
    case "cylinder":
      return { radius: g.radius, length: g.length };
    case "box":
      return { size: g.size };
    case "sphere":
      return { radius: g.radius };
    case "plane":
      return { normal: g.normal, distance: g.distance };
    case "circle":
      return { radius: g.radius };
    case "mesh":
      return { file: g.file, scale: g.scale };
    default:
      throw new Error("unknown blade " + g.blade);
  }
}

export function buildModel(doc) {
  const robot = doc.robot;
  const links = [];
  for (const l of robot.links || []) {
    const geometry = (l.geometry || [])
      .filter((g) => (g.role || []).includes("visual"))
      .map((g) => ({
        blade: g.blade,
        origin: originToMotor(g.origin),
        material: g.material ?? null,
        params: geomParams(g),
      }));
    links.push({ name: l.name, geometry });
  }
  const joints = (robot.joints || []).map((j) => ({
    name: j.name,
    type: j.type,
    parent: j.parent,
    child: j.child,
    origin: originToMotor(j.origin),
    axis: j.axis ?? null,
    lower: j.lower ?? null,
    upper: j.upper ?? null,
  }));
  return {
    name: robot.name,
    base: robot.base,
    materials: (robot.materials || []).map((m) => ({ name: m.name, color: m.color })),
    links,
    joints,
  };
}
