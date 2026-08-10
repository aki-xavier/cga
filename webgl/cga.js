// CGA (5D 共形几何代数) 浏览器端核心 —— 原生实现, 无导出表。
//
// 基向量: 0=e1, 1=e2, 2=e3, 3=e0, 4=e∞; 度量:
//   e1²=e2²=e3²=1, e0²=e∞²=0, m(e0,e∞)=m(e∞,e0)=−1。
// 基 blade = bitmask; 多向量 = {mask: value} 稀疏对象。
// 几何积递归: 向量·blade = 左收缩(a⌋B) + 楔积(a∧B);
//   (a∧W)·B = a·(W·B) − (a⌋W)·B。
// 已对照 Python 端 32×32 基乘积表逐项验证 (0/1024 不匹配)。

const N5 = 5;

function popcount(x) {
  let c = 0;
  while (x) {
    c += x & 1;
    x >>= 1;
  }
  return c;
}
function bitsOf(mask) {
  const o = [];
  for (let i = 0; i < N5; i++) if (mask & (1 << i)) o.push(i);
  return o;
}
function metric(u, v) {
  if (u === v) return u < 3 ? 1 : 0;
  if ((u === 3 && v === 4) || (u === 4 && v === 3)) return -1;
  return 0;
}
// reverse 符号: grade g → (−1)^{g(g−1)/2}
function reverseSign(mask) {
  const g = popcount(mask);
  return (g * (g - 1) / 2) % 2 ? -1 : 1;
}

// ── 基 blade 乘积 (缓存: 纯函数, 至多 32×32) ─────────────────────
const _prodCache = new Map();
function basisProduct(a, b) {
  const key = a * 32 + b;
  const hit = _prodCache.get(key);
  if (hit) return hit;
  const res = gpBlade(a, b);
  _prodCache.set(key, res);
  return res;
}

// vector·blade: a⌋B (左收缩, (−1)^{j−1}) + a∧B (楔积)
function vecBlade(a, B) {
  const res = [];
  const bits = bitsOf(B);
  for (let j = 0; j < bits.length; j++) {
    const bj = bits[j];
    const mu = metric(a, bj);
    if (mu !== 0) res.push({ mask: B & ~(1 << bj), value: (j % 2 ? -1 : 1) * mu });
  }
  if (!(B & (1 << a))) {
    const swaps = popcount(B & ((1 << a) - 1)); // B 中索引 < a
    res.push({ mask: B | (1 << a), value: swaps % 2 ? -1 : 1 });
  }
  return res;
}
// blade·blade 递归: (a∧W)·B = a·(W·B) − (a⌋W)·B
function gpBlade(A, B) {
  if (A === 0) return [{ mask: B, value: 1 }];
  const a = bitsOf(A)[0];
  const W = A & ~(1 << a);
  const acc = {};
  for (const wb of gpBlade(W, B)) {
    for (const e of vecBlade(a, wb.mask)) {
      acc[e.mask] = (acc[e.mask] || 0) + e.value * wb.value;
    }
  }
  const bits = bitsOf(W);
  for (let j = 0; j < bits.length; j++) {
    const mu = metric(a, bits[j]);
    if (mu !== 0) {
      const Wj = W & ~(1 << bits[j]);
      const sign = (j % 2 ? -1 : 1) * mu;
      for (const r of gpBlade(Wj, B)) acc[r.mask] = (acc[r.mask] || 0) - sign * r.value;
    }
  }
  return Object.entries(acc)
    .filter(([, v]) => v !== 0)
    .map(([m, v]) => ({ mask: parseInt(m), value: v }));
}

// ── 多向量运算 (稀疏 {mask: value}) ───────────────────────────────

export function gp(a, b) {
  const out = {};
  for (const [ma, va] of Object.entries(a)) {
    for (const [mb, vb] of Object.entries(b)) {
      for (const t of basisProduct(parseInt(ma), parseInt(mb))) {
        out[t.mask] = (out[t.mask] || 0) + va * vb * t.value;
      }
    }
  }
  for (const k of Object.keys(out)) if (out[k] === 0) delete out[k];
  return out;
}
export function reverse(a) {
  const out = {};
  for (const [m, v] of Object.entries(a)) out[m] = reverseSign(parseInt(m)) * v;
  return out;
}
export function apply(M, X) {
  return gp(gp(M, X), reverse(M)); // M·X·M̃
}

// ── Motor (versor, 稀疏多向量) ───────────────────────────────────

export class Motor {
  constructor(m) {
    this.m = m; // {mask: value}
  }
  static identity() {
    return new Motor({ 0: 1 });
  }
  // 镜像 motors.py: R = cos(θ/2) − sin(θ/2)(nx·e23 + ny·e31 + nz·e12)
  // e12=mask3, e13=mask5 (e31=−e13), e23=mask6
  static rotor(axis, angle) {
    const l = Math.hypot(axis[0], axis[1], axis[2]);
    if (l < 1e-12) return Motor.identity();
    const ax = axis[0] / l, ay = axis[1] / l, az = axis[2] / l;
    const s = Math.cos(angle / 2), sf = Math.sin(angle / 2);
    return new Motor({ 0: s, 3: -sf * az, 5: sf * ay, 6: -sf * ax });
  }
  // T = 1 − ½(t∧e∞): e1e∞=mask17, e2e∞=mask18, e3e∞=mask20
  static translator(t) {
    return new Motor({ 0: 1, 17: -0.5 * t[0], 18: -0.5 * t[1], 20: -0.5 * t[2] });
  }
  // M = T(t)·R(rpy), rpy = 外旋 X-Y-Z (URDF 约定)
  static fromRPY(rpy, xyz) {
    const [a, b, c] = rpy;
    const ca = Math.cos(a), sa = Math.sin(a);
    const cb = Math.cos(b), sb = Math.sin(b);
    const cc = Math.cos(c), sc = Math.sin(c);
    const R = [
      [cb * cc, sa * sb * cc - ca * sc, ca * sb * cc + sa * sc],
      [cb * sc, sa * sb * sc + ca * cc, ca * sb * sc - sa * cc],
      [-sb, sa * cb, ca * cb],
    ];
    return Motor.translator(xyz).compose(Motor.fromMatrix(R));
  }
  static fromAxisAngle(axis, angle, t) {
    return Motor.translator(t).compose(Motor.rotor(axis, angle));
  }
  // 旋转矩阵 → rotor (Shepperd, 镜像 motors.py _matrix_to_quaternion)
  static fromMatrix(R) {
    const tr = R[0][0] + R[1][1] + R[2][2];
    let q;
    if (tr > 0) {
      const s = Math.sqrt(tr + 1) * 2;
      q = [0.25 * s, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s];
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
      const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2;
      q = [(R[2][1] - R[1][2]) / s, 0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s];
    } else if (R[1][1] > R[2][2]) {
      const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2;
      q = [(R[0][2] - R[2][0]) / s, (R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s];
    } else {
      const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2;
      q = [(R[1][0] - R[0][1]) / s, (R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s];
    }
    const [w, x, y, z] = q;
    const n = Math.hypot(w, x, y, z);
    if (n < 1e-12) return Motor.identity();
    const angle = 2 * Math.atan2(Math.hypot(x, y, z) / n, w / n);
    if (Math.abs(angle) < 1e-9) return Motor.identity();
    return Motor.rotor([x, y, z], angle);
  }
  compose(o) {
    return new Motor(gp(this.m, o.m)); // this·o
  }
  reverse() {
    return new Motor(reverse(this.m));
  }
  apply(X) {
    return apply(this.m, X); // X: {mask: value} 或数字 mask
  }
  // versor → (R, t): R 列 = apply(e_i) 欧氏部分; t = apply(e0) 归一化坐标
  toRT() {
    const R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) {
      const r = this.apply({ [1 << i]: 1 });
      R[0][i] = r[1] || 0;
      R[1][i] = r[2] || 0;
      R[2][i] = r[4] || 0;
    }
    const p = this.apply({ 8: 1 }); // e0 = mask 8
    const w = p[8] || 1;
    return { R, t: [(p[1] || 0) / w, (p[2] || 0) / w, (p[4] || 0) / w] };
  }
}

// ── 刚体变换工具 ─────────────────────────────────────────────────

export function rigid(R, t, v) {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2] + t[0],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2] + t[1],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2] + t[2],
  ];
}
export function rotate(R, v) {
  return rigid(R, [0, 0, 0], v);
}
export function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// ── FK: 关节角 → 每个 link 的 world Motor (versor 链, 镜像 robot.fk) ──

export function fk(robot, q) {
  const world = { [robot.base]: Motor.identity() };
  const pending = [...robot.joints];
  while (pending.length > 0) {
    let progressed = false;
    for (let k = 0; k < pending.length; k++) {
      const j = pending[k];
      if (!world[j.parent]) continue;
      let m = j.origin;
      const v = q[j.name] ?? 0;
      if (j.type === "revolute" || j.type === "continuous") {
        m = m.compose(Motor.rotor(j.axis, v));
      } else if (j.type === "prismatic") {
        m = m.compose(Motor.translator(j.axis.map((a) => a * v)));
      }
      world[j.child] = world[j.parent].compose(m);
      pending.splice(k, 1);
      k--;
      progressed = true;
    }
    if (!progressed) throw new Error("FK 循环/断链");
  }
  return world;
}
