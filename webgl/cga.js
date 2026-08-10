// CGA (5D 共形几何代数) 浏览器端核心 —— 与 Python 包同一张乘法表。
//
// 表驱动 gp: 32×32 基 blade 乘积表 (basis_table) 与 reverse 符号
// (reverse_sign) 由 cga.webgl 从 Python 端导出 —— 唯一的代数真值源。
// Motor = 32 分量 versor; FK 走 versor 乘积; 几何共轭走 sandwich。

export function makeCGA(basisTable, reverseSign) {
  const N = 32;

  // a·b: 稀疏表驱动几何积 (motor 只有少数槽非零, 稀疏跳过; 每项可多分量)
  function gp(a, b) {
    const out = new Float32Array(N);
    for (let i = 0; i < N; i++) {
      const ai = a[i];
      if (ai === 0) continue;
      for (let j = 0; j < N; j++) {
        const bj = b[j];
        if (bj === 0) continue;
        const terms = basisTable[i * N + j];
        if (!terms) continue;
        for (let k = 0; k < terms.length; k++) {
          out[terms[k][0]] += ai * bj * terms[k][1];
        }
      }
    }
    return out;
  }

  // 逆向 (grade k → (−1)^(k(k−1)/2), 表导出)
  function reverse(a) {
    const out = new Float32Array(N);
    for (let i = 0; i < N; i++) out[i] = reverseSign[i] * a[i];
    return out;
  }

  // M·X·M̃ (sandwich, versor 共轭)
  function apply(M, X) {
    return gp(gp(M, X), reverse(M));
  }

  class Motor {
    constructor(v) {
      this.v = v; // Float32Array(32)
    }
    static fromArray(v) {
      return new Motor(v);
    }
    static identity() {
      const v = new Float32Array(N);
      v[0] = 1;
      return new Motor(v);
    }
    // 镜像 motors.py: R = cos(θ/2) − sin(θ/2)(nx·e23 + ny·e31 + nz·e12)
    // 槽位: e12=6, e13=7, e23=10 (e31 = −e13)
    static rotor(axis, angle) {
      const l = Math.hypot(axis[0], axis[1], axis[2]);
      if (l < 1e-12) return Motor.identity();
      const ax = axis[0] / l, ay = axis[1] / l, az = axis[2] / l;
      const s = Math.cos(angle / 2), sf = Math.sin(angle / 2);
      const v = new Float32Array(N);
      v[0] = s;
      v[6] = -sf * az;
      v[7] = sf * ay;
      v[10] = -sf * ax;
      return new Motor(v);
    }
    // T = 1 − ½(t∧e∞): e1e∞=9, e2e∞=12, e3e∞=14
    static translator(t) {
      const v = new Float32Array(N);
      v[0] = 1;
      v[9] = -0.5 * t[0];
      v[12] = -0.5 * t[1];
      v[14] = -0.5 * t[2];
      return new Motor(v);
    }
    compose(o) {
      return new Motor(gp(this.v, o.v)); // this·o (先 o 后 this)
    }
    apply(X) {
      return apply(this.v, X);
    }
    reverse() {
      return new Motor(reverse(this.v));
    }
    // versor → (R, t): R 列 = apply(e_i) 欧氏部分; t = apply(e0) 归一化坐标
    toRT() {
      const R = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
      ];
      for (let i = 0; i < 3; i++) {
        const Ei = new Float32Array(N);
        Ei[1 + i] = 1; // e1,e2,e3 槽 1..3
        const r = this.apply(Ei);
        R[0][i] = r[1];
        R[1][i] = r[2];
        R[2][i] = r[3];
      }
      const E0 = new Float32Array(N);
      E0[4] = 1; // e0 槽 4
      const p = this.apply(E0);
      const w = p[4] || 1;
      return { R, t: [p[1] / w, p[2] / w, p[3] / w] };
    }
  }

  return { gp, reverse, apply, Motor };
}

// 刚体变换工具 (R,t 作用于欧氏向量/点)
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

// FK: 关节角 → 每个 link 的 world Motor (versor 链, 镜像 robot.fk)
export function fk(cga, robot, q) {
  const Motor = cga.Motor;
  const world = { [robot.base]: Motor.identity() };
  const pending = [...robot.joints];
  while (pending.length > 0) {
    let progressed = false;
    for (let k = 0; k < pending.length; k++) {
      const j = pending[k];
      if (!world[j.parent]) continue;
      let m = Motor.fromArray(j.origin);
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
