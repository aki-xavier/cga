// CGA WebGL 渲染器: 隐式 blade 解析求交 (与 cga.engine 同族数学) + Blinn-Phong。
//
// 每帧: 关节角 → FK (versor 链) → link motor · 几何 origin → (R,t) →
// 世界→相机刚体变换 → 图元参数进 uniform 数组 → 片段着色器逐像素求交。
// 相机 = OrbitControls (azimuth/elevation/radius), 世界→相机 (R,t)。

import { makeCGA, fk, rigid, rotate, dot } from "./cga.js";

const MAX_PRIMS = 32;

const VERT = `#version 300 es
layout(location=0) in vec2 a_pos;
void main(){ gl_Position = vec4(a_pos, 0.0, 1.0); }`;

const FRAG = `#version 300 es
precision highp float;
uniform vec2 u_res;
uniform float u_fx, u_fy, u_cx, u_cy;
uniform int u_count;
uniform int u_type[${MAX_PRIMS}];
uniform vec4 u_p[${MAX_PRIMS * 3}];   // 每图元 12 floats (3×vec4)
uniform vec3 u_color[${MAX_PRIMS}];
uniform vec3 u_bg, u_lightDir, u_lightPos, u_ambient;
uniform float u_lightInt, u_pointInt;
out vec4 fragColor;

// 球: 二次求交 (与引擎同公式)
float raySphere(vec3 o, vec3 d, vec3 c, float r, out vec3 n) {
  vec3 oc = o - c;
  float b = 2.0 * dot(oc, d);
  float cq = dot(oc, oc) - r * r;
  float disc = b * b - 4.0 * cq;
  if (disc < 1e-12) return -1.0;
  float sq = sqrt(disc);
  float t1 = (-b - sq) / 2.0, t2 = (-b + sq) / 2.0;
  float t = (t1 > 1e-6) ? t1 : t2;
  if (t < 1e-6) return -1.0;
  vec3 p = o + t * d;
  n = (p - c) / r;
  if (t1 <= 1e-6) n = -n;  // 相机在球内
  return t;
}

float rayPlane(vec3 o, vec3 d, vec3 n, float dist, out vec3 nn) {
  float denom = dot(n, d);
  if (abs(denom) < 1e-9) return -1.0;
  float t = (dist - dot(n, o)) / denom;
  if (t < 1e-6) return -1.0;
  nn = n;
  return t;
}

// 有限圆柱 (轴 u 过 p, 半径 r, 长度 len): 侧面 + 两个端盖圆盘
float rayCylinder(vec3 o, vec3 d, vec3 p, vec3 u, float r, float len, out vec3 n) {
  float h = len / 2.0;
  vec3 oc = o - p;
  float dpar = dot(d, u), opar = dot(oc, u);
  vec3 dp = d - dpar * u, op = oc - opar * u;
  float a = dot(dp, dp), b = 2.0 * dot(op, dp), cq = dot(op, op) - r * r;
  float disc = b * b - 4.0 * a * cq;
  float t = -1.0; n = vec3(0);
  if (a > 1e-12 && disc > 1e-12) {
    float sq = sqrt(disc);
    float t1 = (-b - sq) / (2.0 * a), t2 = (-b + sq) / (2.0 * a);
    float ts = (t1 > 1e-6) ? t1 : t2;
    if (ts > 1e-6) {
      float s = opar + ts * dpar;
      if (abs(s) <= h) {
        t = ts;
        vec3 hit = op + t * dp;
        n = hit / r;
        if (t1 <= 1e-6) n = -n;
      }
    }
  }
  // 端盖: 圆心 p ± h·u, 法向 ±u; 出射法向 −sign(d·u)·u
  for (int e = 0; e < 2; e++) {
    float sgn = (e == 0) ? 1.0 : -1.0;
    float denom = dot(d, u);
    if (abs(denom) < 1e-9) continue;
    float tc = (dot(p + sgn * h * u, u) - dot(o, u)) / denom;
    if (tc < 1e-6) continue;
    vec3 pc = o + tc * d;
    vec3 lat = pc - (p + sgn * h * u);
    lat = lat - dot(lat, u) * u;
    if (dot(lat, lat) <= r * r) {
      if (t < 0.0 || tc < t) { t = tc; n = -sign(denom) * u; }
    }
  }
  return t;
}

// 盒 (中心 c, 轴 ax/ay/az 单位正交, 半尺寸 hx/hy/hz): slab 法
float rayBox(vec3 o, vec3 d, vec3 c, vec3 ax, vec3 ay, vec3 az, vec3 hh, out vec3 n) {
  vec3 oc = o - c;
  vec3 p = vec3(dot(oc, ax), dot(oc, ay), dot(oc, az));
  vec3 dd = vec3(dot(d, ax), dot(d, ay), dot(d, az));
  vec3 t0 = (-hh - p) / dd, t1 = (hh - p) / dd;
  vec3 tminv = min(t0, t1), tmaxv = max(t0, t1);
  float tmin = max(max(tminv.x, tminv.y), tminv.z);
  float tmax = min(min(tmaxv.x, tmaxv.y), tmaxv.z);
  if (tmax < tmin || tmax < 0.0) return -1.0;
  float t = (tmin > 1e-6) ? tmin : tmax;
  if (t < 1e-6) return -1.0;
  vec3 ph = p + t * dd;  // 盒局部命中点
  float mx = max(max(abs(ph.x) / hh.x, abs(ph.y) / hh.y), abs(ph.z) / hh.z);
  if (abs(mx - abs(ph.x) / hh.x) < 1e-4) n = sign(ph.x) * ax;
  else if (abs(mx - abs(ph.y) / hh.y) < 1e-4) n = sign(ph.y) * ay;
  else n = sign(ph.z) * az;
  return t;
}

float rayCircle(vec3 o, vec3 d, vec3 c, vec3 n, float r, out vec3 nn) {
  float denom = dot(n, d);
  if (abs(denom) < 1e-9) return -1.0;
  float t = dot(c - o, n) / denom;
  if (t < 1e-6) return -1.0;
  vec3 pc = o + t * d - c;
  if (dot(pc, pc) > r * r) return -1.0;
  nn = (denom < 0.0) ? n : -n;
  return t;
}

void main() {
  vec2 uv = gl_FragCoord.xy;
  // gl_FragCoord.y 从底部起算, 而相机空间 Y 向下、图像行从顶部起算 →
  // y 分量用 u_res.y - uv.y 翻转 (否则上下颠倒)
  vec3 d = normalize(vec3((uv.x - u_cx) / u_fx, (u_res.y - uv.y - u_cy) / u_fy, 1.0));
  vec3 o = vec3(0.0);
  float tmin = 1e30;
  vec3 n = vec3(0.0), col = u_bg;
  for (int i = 0; i < ${MAX_PRIMS}; i++) {
    if (i >= u_count) break;
    vec4 a = u_p[i * 3], b = u_p[i * 3 + 1], cc = u_p[i * 3 + 2];
    float tn = -1.0;
    vec3 nn = vec3(0.0);
    if (u_type[i] == 0) tn = raySphere(o, d, a.xyz, a.w, nn);
    else if (u_type[i] == 1) tn = rayPlane(o, d, a.xyz, a.w, nn);
    else if (u_type[i] == 2) tn = rayCylinder(o, d, a.xyz, b.xyz, a.w, b.w, nn);
    else if (u_type[i] == 3) {
      vec3 az = cross(b.xyz, cc.xyz);
      tn = rayBox(o, d, a.xyz, b.xyz, cc.xyz, az, vec3(a.w, b.w, cc.w), nn);
    }
    else if (u_type[i] == 4) tn = rayCircle(o, d, a.xyz, b.xyz, a.w, nn);
    if (tn > 0.0 && tn < tmin) { tmin = tn; n = nn; col = u_color[i]; }
  }
  if (tmin < 1e29) {
    vec3 v = -d;
    vec3 p = o + tmin * d;
    vec3 diff = col * 0.9;           // roughness 0.45 → k=0.55, 漫反射 0.9
    vec3 spec = vec3(0.1) + col * 0.1;
    float ndv = max(dot(n, v), 0.0);
    float expo = 64.0;
    vec3 outC = u_ambient * diff;
    // 平行光
    float nl = max(dot(n, u_lightDir), 0.0);
    vec3 h = normalize(u_lightDir + v);
    float sp = pow(max(dot(n, h), 0.0), expo);
    outC += u_lightInt * (diff * nl + spec * sp * ndv);
    // 点光
    vec3 lv = u_lightPos - p;
    float dist2 = dot(lv, lv);
    vec3 ld = lv / sqrt(dist2);
    float atten = u_pointInt / (1.0 + dist2 / 8.0);
    float nl2 = max(dot(n, ld), 0.0);
    vec3 h2 = normalize(ld + v);
    float sp2 = pow(max(dot(n, h2), 0.0), expo);
    outC += atten * (diff * nl2 + spec * sp2 * ndv);
    col = outC;
  }
  fragColor = vec4(col, 1.0);
}`;

export class CgaRenderer {
  constructor(canvas, model) {
    this.canvas = canvas;
    this.model = model;
    this.gl = canvas.getContext("webgl2", { antialias: false, preserveDrawingBuffer: true });
    if (!this.gl) throw new Error("需要 WebGL2");
    const gl = this.gl;
    this.prog = this._build(gl);
    this.vao = gl.createVertexArray();
    gl.bindVertexArray(this.vao);
    const buf = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 3, -1, -1, 3]), gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);

    // CGA 核心 (表来自模型 JSON)
    this.cga = makeCGA(model.basis_table, model.reverse_sign);
    this.Motor = this.cga.Motor;

    // 轨道 (相机 = OrbitControls; 世界 Y-up, 模型 Z-up → 根级 Rot(X,-π/2))
    this.azimuth = 0.7;
    this.elevation = 0.35;
    this.radius = 0.95;
    this.target = [0.24, 0.25, 0];
    this.worldUp = this.Motor.rotor([1, 0, 0], -Math.PI / 2);

    // 关节滑块状态
    this.q = {};
    for (const j of model.robot.joints) {
      if (j.type === "revolute" || j.type === "continuous" || j.type === "prismatic") {
        this.q[j.name] = 0;
      }
    }
  }

  _build(gl) {
    const mk = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
        throw new Error(gl.getShaderInfoLog(s));
      }
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, mk(gl.VERTEX_SHADER, VERT));
    gl.attachShader(p, mk(gl.FRAGMENT_SHADER, FRAG));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) {
      throw new Error(gl.getProgramInfoLog(p));
    }
    return p;
  }

  // 世界 → 相机 (orbit 相机, (R,t) = 相机 motor 的矩阵形式)
  _cameraRT() {
    const ce = Math.cos(this.elevation);
    const pos = [
      this.target[0] + this.radius * ce * Math.sin(this.azimuth),
      this.target[1] + this.radius * Math.sin(this.elevation),
      this.target[2] + this.radius * ce * Math.cos(this.azimuth),
    ];
    const f = norm(sub(this.target, pos));
    const r = norm(cross(f, [0, 1, 0]));
    const u = cross(r, f);
    // 相机基 X 右 / Y 下 / Z 前 → 世界→相机行 = [r; −u; f]
    const R = [r, [-u[0], -u[1], -u[2]], f];
    const t = [-dot(R[0], pos), -dot(R[1], pos), -dot(R[2], pos)];
    return { R, t };
  }

  // 收集相机空间图元 → uniform 数组
  _primitives(camRT) {
    const type = new Int32Array(MAX_PRIMS);
    const p = new Float32Array(MAX_PRIMS * 12);
    const color = new Float32Array(MAX_PRIMS * 3);
    const mats = new Map(this.model.robot.materials.map((m) => [m.name, m.color.slice(0, 3)]));
    const world = fk(this.cga, this.model.robot, this.q);
    let n = 0;
    for (const link of this.model.robot.links) {
      const lm = this.worldUp.compose(world[link.name]);
      if (!lm) continue;
      for (const g of link.geometry) {
        if (n >= MAX_PRIMS) break;
        const m = lm.compose(this.Motor.fromArray(g.origin));
        const { R, t } = m.toRT();
        const c = mats.get(g.material) || [0.7, 0.7, 0.7];
        const wp = worldParams(g, R, t);
        const cp = toCamera(wp, camRT);
        const base = n * 12;
        type[n] = cp.type;
        for (let k = 0; k < 12; k++) p[base + k] = cp.data[k];
        color[n * 3] = c[0]; color[n * 3 + 1] = c[1]; color[n * 3 + 2] = c[2];
        n++;
      }
    }
    return { count: n, type, p, color };
  }

  render() {
    const gl = this.gl;
    const W = this.canvas.width, H = this.canvas.height;
    gl.viewport(0, 0, W, H);
    gl.useProgram(this.prog);
    gl.bindVertexArray(this.vao);
    const u = (name) => gl.getUniformLocation(this.prog, name);
    gl.uniform2f(u("u_res"), W, H);
    const fy = H / (2.0 * Math.tan((50.0 * Math.PI) / 180.0 / 2.0));
    const fx = fy * (W / H);
    gl.uniform1f(u("u_fx"), fx);
    gl.uniform1f(u("u_fy"), fy);
    gl.uniform1f(u("u_cx"), (W - 1) / 2);
    gl.uniform1f(u("u_cy"), (H - 1) / 2);
    gl.uniform3f(u("u_bg"), 0.53, 0.81, 0.92);
    gl.uniform3f(u("u_lightDir"), 0.29, 0.72, 0.25);
    gl.uniform1f(u("u_lightInt"), 0.65);
    gl.uniform3f(u("u_lightPos"), 0, 3.5, 2.8);
    gl.uniform1f(u("u_pointInt"), 0.6);
    gl.uniform3f(u("u_ambient"), 0.52, 0.52, 0.52);

    const camRT = this._cameraRT();
    const prims = this._primitives(camRT);
    gl.uniform1i(u("u_count"), prims.count);
    gl.uniform1iv(u("u_type"), prims.type);
    gl.uniform4fv(u("u_p"), prims.p);
    gl.uniform3fv(u("u_color"), prims.color);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
  }

  resize(w, h) {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.floor(w * dpr);
    this.canvas.height = Math.floor(h * dpr);
    this.canvas.style.width = w + "px";
    this.canvas.style.height = h + "px";
  }
}

// ── 几何参数: 局部 → 世界 → 相机 (与 engine.to_camera 同族) ──────

function worldParams(g, R, t) {
  // 局部轴约定: 圆柱/圆 = +Z (与 URDF/engine 一致)
  switch (g.blade) {
    case "sphere":
      return { type: 0, data: [t[0], t[1], t[2], g.params.radius] };
    case "plane": {
      const n = rotate(R, g.params.normal);
      const d = g.params.distance + dot(n, t);
      return { type: 1, data: [n[0], n[1], n[2], d] };
    }
    case "cylinder": {
      const u = rotate(R, [0, 0, 1]);
      // 打包 [p3, r, u3, len]: a.xyz=点,a.w=r; b.xyz=轴,b.w=len
      return {
        type: 2,
        data: [
          t[0], t[1], t[2], g.params.radius,
          u[0], u[1], u[2], g.params.length,
        ],
      };
    }
    case "box": {
      const h = g.params.size.map((s) => s / 2);
      const ax = [R[0][0], R[1][0], R[2][0]];
      const ay = [R[0][1], R[1][1], R[2][1]];
      // 打包: [c3, hx, ax3, hy, ay3, hz] → a.xyz=c,a.w=hx; b.xyz=ax,b.w=hy; cc.xyz=ay,cc.w=hz
      return {
        type: 3,
        data: [
          t[0], t[1], t[2], h[0],
          ax[0], ax[1], ax[2], h[1],
          ay[0], ay[1], ay[2], h[2],
        ],
      };
    }
    case "circle": {
      const n = rotate(R, [0, 0, 1]);
      // 打包 [c3, r, n3]: a.xyz=圆心,a.w=r; b.xyz=法向
      return { type: 4, data: [t[0], t[1], t[2], g.params.radius, n[0], n[1], n[2]] };
    }
    default:
      throw new Error("unknown blade " + g.blade);
  }
}

function toCamera(wp, camRT) {
  // 世界 → 相机: 点 c' = R·c + t; 方向 n' = R·n; 平面 d' = d + n'·t
  const { R, t } = camRT;
  const d = wp.data;
  if (wp.type === 0) {
    const c = rigid(R, t, [d[0], d[1], d[2]]);
    return { type: 0, data: [c[0], c[1], c[2], d[3]] };
  }
  if (wp.type === 1) {
    const n = rotate(R, [d[0], d[1], d[2]]);
    return { type: 1, data: [n[0], n[1], n[2], d[3] + dot(n, t)] };
  }
  if (wp.type === 2) {
    const c = rigid(R, t, [d[0], d[1], d[2]]);
    const u = rotate(R, [d[4], d[5], d[6]]);
    return { type: 2, data: [c[0], c[1], c[2], d[3], u[0], u[1], u[2], d[7]] };
  }
  if (wp.type === 3) {
    const c = rigid(R, t, [d[0], d[1], d[2]]);
    const ax = rotate(R, [d[4], d[5], d[6]]);
    const ay = rotate(R, [d[8], d[9], d[10]]);
    return {
      type: 3,
      data: [c[0], c[1], c[2], d[3], ax[0], ax[1], ax[2], d[7], ay[0], ay[1], ay[2], d[11]],
    };
  }
  if (wp.type === 4) {
    const c = rigid(R, t, [d[0], d[1], d[2]]);
    const n = rotate(R, [d[4], d[5], d[6]]);
    return { type: 4, data: [c[0], c[1], c[2], d[3], n[0], n[1], n[2]] };
  }
  throw new Error("bad type");
}

function norm(v) {
  const l = Math.hypot(v[0], v[1], v[2]);
  return [v[0] / l, v[1] / l, v[2] / l];
}
function sub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}
function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}
