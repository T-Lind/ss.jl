// The assembly hangar: a small WebGL renderer for the stack being built.
//
// The page owns the model and the 2D overlay (attach nodes, stage brackets);
// this module owns geometry, light, picking and the camera. Geometry comes
// from the layout rows, so the vehicle on screen is the stack the simulator
// is handed. Units are metres, y up, the vehicle axis on x = z = 0.
//
// No imports, on purpose: the page passes a canvas and a scene.

const TAU = Math.PI * 2;
const FOV = 35 * Math.PI / 180;
const hex = s => [parseInt(s.slice(1, 3), 16) / 255, parseInt(s.slice(3, 5), 16) / 255, parseInt(s.slice(5, 7), 16) / 255];
// Colours are sRGB; the shader linearises them. Propellant colours mirror
// PROP_COLORS in static/vab_parts.js.
const C = {
  tank: [0.9, 0.9, 0.9], seam: [0.8, 0.8, 0.81], dark: [0.12, 0.13, 0.15], gun: [0.3, 0.32, 0.36],
  copper: [0.82, 0.55, 0.3], copperDark: [0.3, 0.16, 0.08], throat: [0.05, 0.035, 0.03],
  amber: [0.94, 0.65, 0.0], gold: [0.8, 0.65, 0.3], goldDark: [0.42, 0.32, 0.13],
  panel: [0.1, 0.22, 0.42], metal: [0.62, 0.65, 0.7], shield: [0.16, 0.13, 0.12], white: [0.9, 0.92, 0.95],
};
const PROPC = {
  kerolox: hex('#e8a33c'), hydrolox: hex('#6fb7ff'), methalox: hex('#b58cff'),
  hypergolic: hex('#7fd67f'), hydrazine: hex('#99aadd'), solid: hex('#d7745a'),
};
const MATERIAL = {                                    // metallic amount, shininess
  tank: [0.55, 60], engine: [0.8, 40], decoupler: [0.3, 30], capsule: [0.35, 50],
  bus: [0.85, 40], lander: [0.85, 40], fairing: [0, 30], boosters: [0.55, 60], adapter: [0.5, 40],
};

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const mix3 = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];

// ------------------------------------------------------------ mesh builder --
// Interleaved vertices: position, normal, colour (9 floats). Triangles are
// not culled, and the shader flips normals on back faces, so winding does not
// matter and the fairing can be seen from inside.
class Build {
  constructor() { this.v = []; this.i = []; this.cyl = []; }
  get count() { return this.v.length / 9; }
  vert(p, n, c) {
    this.v.push(p[0], p[1], p[2], n[0], n[1], n[2], c[0], c[1], c[2]);
    return this.count - 1;
  }
  quad(a, b, c, d) { this.i.push(a, b, c, a, c, d); }
  tri(a, b, c) { this.i.push(a, b, c); }
}

// A surface of revolution. prof: [{r, y, c}] from the bottom up. Normals are
// averaged across smooth joins; a repeated point (same r, y) with a different
// colour makes a hard edge, which is how bands and seams are drawn.
function lathe(B, prof, seg, ox = 0, oz = 0) {
  const P = prof.length, base = B.count;
  const segN = i => {
    const a = prof[i], b = prof[i + 1], dr = b.r - a.r, dy = b.y - a.y, L = Math.hypot(dr, dy);
    return L < 1e-9 ? null : [dy / L, -dr / L];
  };
  for (let k = 0; k < P; k++) {
    const n1 = k > 0 ? segN(k - 1) : null, n2 = k < P - 1 ? segN(k) : null;
    let nr = (n1 ? n1[0] : 0) + (n2 ? n2[0] : 0), ny = (n1 ? n1[1] : 0) + (n2 ? n2[1] : 0);
    const L = Math.hypot(nr, ny) || 1;
    nr /= L; ny /= L;
    const p = prof[k];
    for (let j = 0; j <= seg; j++) {
      const t = TAU * j / seg, c = Math.cos(t), s = Math.sin(t);
      B.vert([ox + p.r * c, p.y, oz + p.r * s], [nr * c, ny, nr * s], p.c);
    }
  }
  for (let k = 0; k < P - 1; k++) for (let j = 0; j < seg; j++) {
    const a = base + k * (seg + 1) + j, d = a + seg + 1;
    B.quad(a, a + 1, d + 1, d);
  }
}

// A polygonal prism with flat faces and flat caps.
function prism(B, sides, R, y0, y1, rot, col, ox = 0, oz = 0, capCol = col) {
  const pt = (a, y) => [ox + R * Math.cos(a), y, oz + R * Math.sin(a)];
  for (let i = 0; i < sides; i++) {
    const a0 = rot + TAU * i / sides, a1 = rot + TAU * (i + 1) / sides, am = (a0 + a1) / 2;
    const n = [Math.cos(am), 0, Math.sin(am)];
    const v0 = B.vert(pt(a0, y0), n, col), v1 = B.vert(pt(a1, y0), n, col);
    const v2 = B.vert(pt(a1, y1), n, col), v3 = B.vert(pt(a0, y1), n, col);
    B.quad(v0, v1, v2, v3);
  }
  for (const [y, ny] of [[y1, 1], [y0, -1]]) {
    const c = B.vert([ox, y, oz], [0, ny, 0], capCol);
    for (let i = 0; i <= sides; i++) B.vert(pt(rot + TAU * i / sides, y), [0, ny, 0], capCol);
    for (let i = 0; i < sides; i++) B.tri(c, c + 1 + i, c + 2 + i);
  }
}

// An axis-aligned box with flat faces.
function box(B, lo, hi, col) {
  const [x0, y0, z0] = lo, [x1, y1, z1] = hi;
  const face = (n, ps) => {
    const a = B.vert(ps[0], n, col), b = B.vert(ps[1], n, col), c = B.vert(ps[2], n, col), d = B.vert(ps[3], n, col);
    B.quad(a, b, c, d);
  };
  face([1, 0, 0], [[x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]]);
  face([-1, 0, 0], [[x0, y0, z1], [x0, y1, z1], [x0, y1, z0], [x0, y0, z0]]);
  face([0, 1, 0], [[x0, y1, z0], [x0, y1, z1], [x1, y1, z1], [x1, y1, z0]]);
  face([0, -1, 0], [[x0, y0, z1], [x0, y0, z0], [x1, y0, z0], [x1, y0, z1]]);
  face([0, 0, 1], [[x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]);
  face([0, 0, -1], [[x1, y0, z0], [x0, y0, z0], [x0, y1, z0], [x1, y1, z0]]);
}

// A round strut between two points.
function tube(B, a, b, r, col, sides = 6) {
  const d = norm(sub(b, a));
  const h = Math.abs(d[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0];
  const u = norm(cross(d, h)), w = cross(d, u);
  const base = B.count;
  for (const end of [a, b]) for (let i = 0; i <= sides; i++) {
    const t = TAU * i / sides, c = Math.cos(t), s = Math.sin(t);
    const n = [u[0] * c + w[0] * s, u[1] * c + w[1] * s, u[2] * c + w[2] * s];
    B.vert([end[0] + r * n[0], end[1] + r * n[1], end[2] + r * n[2]], n, col);
  }
  for (let i = 0; i < sides; i++) {
    const p = base + i, q = base + sides + 1 + i;
    B.quad(p, p + 1, q + 1, q);
  }
}

// ------------------------------------------------------------ part builders --
// Tank: a cylinder with rounded ends, faint panel seams every ~2 m, and a
// thin ring of propellant colour near the top.
function buildTank(B, y0, y1, d, prop, ox, oz) {
  const R = d / 2, h = y1 - y0, dome = Math.min(0.12 * R, 0.1 * h), yTop = y1 - dome;
  const pc = PROPC[prop] || PROPC.kerolox, pb = yTop - Math.min(0.5, 0.12 * h);
  const P = [];
  const add = (r, y, c) => P.push({ r, y, c });
  add(0, y0, C.tank); add(0.9 * R, y0, C.tank); add(R, y0 + dome, C.tank);
  for (let s = Math.ceil((y0 + dome + 0.05) / 2) * 2; s < pb - 0.1; s += 2) {
    add(R, s - 0.025, C.tank); add(R, s - 0.025, C.seam); add(R, s + 0.025, C.seam); add(R, s + 0.025, C.tank);
  }
  if (pb > y0 + dome + 0.2) { add(R, pb, C.tank); add(R, pb, pc); add(R, yTop, pc); }
  else add(R, yTop, C.tank);
  add(0.9 * R, y1, C.tank); add(0, y1, C.tank);
  lathe(B, P, 48, ox, oz);
}

// Bell positions for n engines inside a stage of diameter d: one bell for
// one engine; up to eight on a ring; more gets a centre bell plus rings.
function bellLayout(n, d) {
  if (n <= 1) return [{ x: 0, z: 0, bw: 0.5 * d }];
  const sizes = [];
  let rem = n - (n > 8 ? 1 : 0), cap = 8;
  while (rem > 0) { const s = Math.min(rem, cap); sizes.push(s); rem -= s; cap += 8; }
  const centre = n > 8, K = sizes.length;
  for (let bw = 0.5 * d; bw > 0.01 * d; bw *= 0.96) {
    const Rout = 0.5 * d - 0.72 * bw;
    const radii = sizes.map((_, k) => Rout - (K - 1 - k) * 1.7 * bw);
    const fits = radii.every((R, k) => R > 0 && (2 * Math.PI * R) / sizes[k] >= 1.15 * bw)
      && (!centre || radii[0] - 0.72 * bw >= 0.72 * 1.25 * bw + 0.05 * bw);
    if (!fits) continue;
    const out = centre ? [{ x: 0, z: 0, bw: 1.25 * bw }] : [];
    sizes.forEach((s, k) => {
      for (let i = 0; i < s; i++) {
        const t = TAU * i / s + k * 0.4;
        out.push({ x: radii[k] * Math.cos(t), z: radii[k] * Math.sin(t), bw });
      }
    });
    return out;
  }
  return [{ x: 0, z: 0, bw: 0.3 * d }];
}

// Engine cluster: a dark thrust plate on top, flared bells hanging below it.
function buildEngine(B, y0, y1, d, n, ox, oz) {
  const h = y1 - y0, plateH = Math.min(0.6, Math.max(0.25, 0.2 * h)), top = y1 - plateH, Rp = 0.49 * d;
  lathe(B, [{ r: 0, y: top, c: C.dark }, { r: Rp, y: top, c: C.dark },
            { r: Rp, y: y1, c: C.dark }, { r: 0, y: y1, c: C.dark }], 40, ox, oz);
  const bh = top - y0;
  for (const b of bellLayout(n, d)) {
    const rBot = 0.72 * b.bw, rTop = 0.5 * b.bw, P = [];
    for (let k = 0; k <= 8; k++) {
      const t = k / 8;
      const r = rBot + (rTop - rBot) * Math.pow(t, 1.8), y = y0 + t * bh;
      P.push({ r, y, c: t > 0.9 ? C.throat : mix3(C.copperDark, C.copper, Math.min(1, t * 2.2)) });
    }
    lathe(B, P, 28, ox + b.x, oz + b.z);
  }
}

function buildDecoupler(B, y0, y1, d, ox, oz) {
  const R = d / 2, h = y1 - y0, D = C.dark, A = C.amber;
  lathe(B, [{ r: 0, y: y0, c: D }, { r: R, y: y0, c: D }, { r: R, y: y0 + 0.35 * h, c: D },
            { r: R, y: y0 + 0.35 * h, c: A }, { r: R, y: y0 + 0.65 * h, c: A },
            { r: R, y: y0 + 0.65 * h, c: D }, { r: R, y: y1, c: D }, { r: 0, y: y1, c: D }], 48, ox, oz);
}

// Capsule: a cone with a dark heat-shield lip and a rounded nose.
function buildCapsule(B, y0, y1, d, ox, oz) {
  const R = d / 2, h = y1 - y0, Hc = 0.72 * h, yc = y0 + Hc, R1 = 0.27 * d, sh = y0 + 0.14 * h;
  const rAt = y => R + (R1 - R) * ((y - y0) / Hc);
  const P = [{ r: 0, y: y0, c: C.shield }, { r: 1.02 * R, y: y0, c: C.shield },
             { r: 1.02 * R, y: sh, c: C.shield }, { r: rAt(sh), y: sh, c: C.white }, { r: R1, y: yc, c: C.white }];
  for (let k = 1; k <= 8; k++) {
    const a = (k / 8) * Math.PI / 2;
    P.push({ r: R1 * Math.cos(a), y: yc + 0.28 * h * Math.sin(a), c: C.white });
  }
  lathe(B, P, 48, ox, oz);
}

// Probe core: an octagonal gold-foil prism with two flat solar panels.
function buildBus(B, y0, y1, d, ox, oz) {
  const R = d / 2, ym = (y0 + y1) / 2, ph = 0.7 * (y1 - y0), pw = 1.25 * d, t = 0.04 * d;
  prism(B, 8, R, y0, y1, Math.PI / 8, C.gold, ox, oz, C.goldDark);
  box(B, [ox + R, ym - ph / 2, oz - t / 2], [ox + R + pw, ym + ph / 2, oz + t / 2], C.panel);
  box(B, [ox - R - pw, ym - ph / 2, oz - t / 2], [ox - R, ym + ph / 2, oz + t / 2], C.panel);
}

// Lunar lander: octagonal gold body on four splayed legs with footpads.
function buildLander(B, y0, y1, d, ox, oz) {
  const R = d / 2, h = y1 - y0;
  prism(B, 8, R, y0, y1, Math.PI / 8, C.gold, ox, oz, C.goldDark);
  for (let i = 0; i < 4; i++) {
    const a = Math.PI / 4 + i * Math.PI / 2, c = Math.cos(a), s = Math.sin(a);
    const top = [ox + c * R * 0.7, y0 + 0.1 * h, oz + s * R * 0.7];
    const foot = [ox + c * R * 1.3, y0 - 0.55 * d, oz + s * R * 1.3];
    tube(B, top, foot, 0.035 * d + 0.03, C.gun, 6);
    box(B, [foot[0] - 0.2 * d, foot[1] - 0.015 * d, foot[2] - 0.2 * d],
            [foot[0] + 0.2 * d, foot[1], foot[2] + 0.2 * d], C.gun);
  }
}

// Fairing: translucent ogive, drawn last with depth writes off.
function buildFairing(B, y0, y1, d) {
  const R = d / 2, H = y1 - y0, P = [];
  for (let k = 0; k <= 24; k++) {
    const u = k / 24;
    P.push({ r: R * Math.sqrt(Math.max(0, 1 - u * u)), y: y0 + u * H, c: C.white });
  }
  lathe(B, P, 48);
}

// Conical interstage where the diameter changes.
function buildAdapter(B, d0, d1, y0, y1, ox, oz) {
  lathe(B, [{ r: d0 / 2, y: y0, c: C.metal }, { r: d1 / 2, y: y1, c: C.metal }], 48, ox, oz);
}

// The radial set: count boosters evenly around stage 1, bottom-aligned with it.
function buildBoosters(B, cfg) {
  const rb = cfg.bd / 2, R = cfg.core / 2 + rb + 0.05;
  const bellH = Math.max(1.2, 0.9 * cfg.bd), y1 = bellH + cfg.L, nh = 0.7 * cfg.bd;
  for (let i = 0; i < cfg.count; i++) {
    const t = TAU * i / cfg.count, ox = R * Math.cos(t), oz = R * Math.sin(t);
    buildTank(B, bellH, y1, cfg.bd, cfg.prop, ox, oz);
    const P = [];
    for (let k = 0; k <= 10; k++) { const u = k / 10; P.push({ r: rb * Math.sqrt(1 - u * u), y: y1 + nh * u, c: C.white }); }
    lathe(B, P, 40, ox, oz);
    buildEngine(B, 0, bellH, cfg.bd, cfg.n, ox, oz);
    B.cyl.push({ cx: ox, cz: oz, r: rb, y0: 0, y1: y1 + nh });
  }
}

// ------------------------------------------------------------ scene --------
// One mesh per layout row, so picking and hover work per part. An interstage
// cone belongs to the stage above it. Fairing and ghost are drawn translucent.
function buildStack(scene) {
  const lay = scene.lay, asm = scene.asm, an = lay.analysis, rows = lay.rows;
  const propOf = new Map();
  an.stages.forEach(s => s.tanks.forEach(t => propOf.set(t.id, s.prop)));
  const meshes = [];
  rows.forEach((r, j) => {
    const p = r.part, B = new Build(), R = r.d / 2;
    if (p.type === 'tank') buildTank(B, r.y0, r.y1, r.d, propOf.get(p.id) || 'kerolox', 0, 0);
    else if (p.type === 'engine') buildEngine(B, r.y0, r.y1, r.d, Math.max(1, Math.min(33, p.count || 1)), 0, 0);
    else if (p.type === 'decoupler') buildDecoupler(B, r.y0, r.y1, r.d, 0, 0);
    else if (p.type === 'payload') {
      if (p.kind === 'capsule') buildCapsule(B, r.y0, r.y1, r.d, 0, 0);
      else if (p.kind === 'bus') buildBus(B, r.y0, r.y1, r.d, 0, 0);
      else buildLander(B, r.y0, r.y1, r.d, 0, 0);
    }
    if (j > 0 && rows[j - 1].d > 0 && r.d > 0 && Math.abs(r.d - rows[j - 1].d) > 0.05) {
      const adH = Math.min(1.2, 0.25 * Math.max(rows[j - 1].d, r.d) + 0.1);
      buildAdapter(B, rows[j - 1].d, r.d, r.y0, r.y0 + adH, 0, 0);
    }
    const wide = p.type === 'payload' && p.kind === 'bus' ? 2.4 : 1;
    B.cyl.push({ cx: 0, cz: 0, r: R * wide, y0: r.y0, y1: r.y1 });
    const mat = MATERIAL[p.type === 'payload' ? p.kind : p.type] || MATERIAL.tank;
    meshes.push({ id: p.id, B, alpha: false, metal: mat[0], shin: mat[1] });
  });
  if (lay.fairing) {
    const f = lay.fairing, B = new Build();
    buildFairing(B, f.y0, f.y1, f.d);
    B.cyl.push({ cx: 0, cz: 0, r: f.d / 2, y0: f.y0, y1: f.y1 });
    meshes.push({ id: f.part.id, B, alpha: true, metal: 0, shin: 30 });
  }
  const g = an.stages.length && asm.boosters ? asm.boosters : null;
  if (g) {
    const B = new Build();
    buildBoosters(B, {
      count: g.count, bd: g.diameter, L: g.length, n: an.boosters ? an.boosters.n : (g.engines || 1),
      prop: an.boosters ? an.boosters.prop : 'kerolox', core: an.stages[0].diameter,
    });
    meshes.push({ id: scene.boostersId || 'boosters', B, alpha: false, metal: 0.55, shin: 60 });
  }
  return meshes;
}

function buildGhostMesh(gh) {
  const B = new Build(), p = gh.part, d = gh.d, h = gh.h;
  if (gh.radial) buildBoosters(B, { count: p.count, bd: p.diameter, L: p.length, n: gh.n, prop: gh.prop, core: gh.core });
  else if (p.type === 'tank') buildTank(B, 0, h, d, gh.prop, 0, 0);
  else if (p.type === 'engine') buildEngine(B, 0, h, d, gh.n, 0, 0);
  else if (p.type === 'decoupler') buildDecoupler(B, 0, h, d, 0, 0);
  else if (p.type === 'fairing') buildFairing(B, 0, h, d);
  else if (p.type === 'payload') {
    if (p.kind === 'capsule') buildCapsule(B, 0, h, d, 0, 0);
    else if (p.kind === 'bus') buildBus(B, 0, h, d, 0, 0);
    else buildLander(B, 0, h, d, 0, 0);
  }
  return { B, alpha: true, metal: 0.2, shin: 30 };
}

// ------------------------------------------------------------ GL ----------
const LIT_VS = `
attribute vec3 aPos; attribute vec3 aNrm; attribute vec3 aCol;
uniform mat4 uVP; uniform vec3 uOff;
varying vec3 vN; varying vec3 vW; varying vec3 vC;
void main() { vec3 w = aPos + uOff; vW = w; vN = aNrm; vC = aCol; gl_Position = uVP * vec4(w, 1.0); }`;
const LIT_FS = `
precision highp float;
varying vec3 vN; varying vec3 vW; varying vec3 vC;
uniform vec3 uEye; uniform vec3 uKey; uniform float uMetal; uniform float uShin;
uniform vec3 uTint; uniform float uTintK; uniform float uAlpha; uniform float uGhost;
void main() {
  vec3 N = normalize(vN);
  if (!gl_FrontFacing) N = -N;
  vec3 V = normalize(uEye - vW);
  vec3 base = pow(vC, vec3(2.2));
  vec3 L = normalize(uKey);
  float ndl = max(dot(N, L), 0.0);
  float spec = pow(max(dot(N, normalize(L + V)), 0.0), uShin) * (0.15 + 0.85 * uMetal);
  vec3 keyCol = vec3(1.0, 0.92, 0.80) * 1.1;
  float fill = max(dot(N, normalize(vec3(0.8, 0.25, -0.5))), 0.0) * 0.18;
  vec3 sky = vec3(0.50, 0.58, 0.72), gnd = vec3(0.28, 0.18, 0.10);
  vec3 amb = mix(gnd, sky, N.y * 0.5 + 0.5) * 0.42;
  vec3 col = base * (amb + keyCol * ndl + vec3(0.55, 0.7, 1.0) * fill);
  vec3 R = reflect(-V, N);
  vec3 env = mix(vec3(0.05, 0.06, 0.08), vec3(0.85, 0.9, 1.0), smoothstep(-0.25, 0.85, R.y));
  // two soft-box reflections give metal its streaks
  float strip = exp(-pow((R.x + 0.45) * 3.5, 2.0)) * 0.9 + exp(-pow((R.x - 0.6) * 5.0, 2.0)) * 0.35;
  col += (env + vec3(strip)) * base * uMetal * 0.6;
  col += keyCol * spec * 0.8;
  col += vec3(0.35, 0.55, 0.95) * pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.22;
  col += uTint * uTintK;
  if (uGhost > 0.5) col = mix(col, vec3(0.25, 0.95, 0.45), 0.55);
  gl_FragColor = vec4(pow(max(col, 0.0), vec3(1.0 / 2.2)), uAlpha);
}`;
const BG_VS = `attribute vec2 aPos; void main() { gl_Position = vec4(aPos, 0.0, 1.0); }`;
const BG_FS = `
precision highp float; uniform vec2 uRes;
void main() {
  vec2 uv = gl_FragCoord.xy / uRes;
  vec3 col = mix(vec3(0.020, 0.027, 0.039), vec3(0.043, 0.063, 0.094), uv.y);
  vec2 q = (uv - vec2(0.5, 0.58)) * vec2(1.0, 1.25);
  col += vec3(0.20, 0.27, 0.38) * 0.07 * exp(-dot(q, q) * 5.0);
  gl_FragColor = vec4(col, 1.0);
}`;
const FLOOR_VS = `
attribute vec3 aPos; uniform mat4 uVP; varying vec3 vW;
void main() { vW = aPos; gl_Position = uVP * vec4(aPos, 1.0); }`;
const FLOOR_FS = `
precision highp float;
varying vec3 vW; uniform vec3 uEye; uniform float uPx;
float grid(float x, float w) { float d = abs(x - floor(x + 0.5)); return 1.0 - smoothstep(0.0, w, d); }
void main() {
  float w = length(uEye - vW) * uPx * 1.1;
  float minor = max(grid(vW.x, w), grid(vW.z, w));
  float major = max(grid(vW.x * 0.1, w * 0.1), grid(vW.z * 0.1, w * 0.1));
  float fade = exp(-length(vW.xz) / 90.0) * 0.95;
  vec3 col = vec3(0.04, 0.052, 0.068) + vec3(0.25, 0.36, 0.52) * (minor * 0.16 + major * 0.26);
  gl_FragColor = vec4(col, fade);
}`;
const BLOB_VS = `
attribute vec3 aPos; uniform mat4 uVP; uniform float uR; varying vec2 vUv;
void main() { vUv = aPos.xz / uR; gl_Position = uVP * vec4(aPos, 1.0); }`;
const BLOB_FS = `
precision mediump float; varying vec2 vUv;
void main() { float a = clamp(1.0 - length(vUv), 0.0, 1.0); gl_FragColor = vec4(0.0, 0.0, 0.0, a * a * 0.55); }`;

// ------------------------------------------------------------ math --------
function persp(fy, asp, n, f) {
  const t = 1 / Math.tan(fy / 2);
  return new Float32Array([t / asp, 0, 0, 0, 0, t, 0, 0, 0, 0, (f + n) / (n - f), -1, 0, 0, 2 * f * n / (n - f), 0]);
}
function lookAt(e, c, u) {
  const z = norm(sub(e, c)), x = norm(cross(u, z)), y = cross(z, x);
  const d = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  return new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0,
                           -d(x, e), -d(y, e), -d(z, e), 1]);
}
function mul(a, b) {
  const o = new Float32Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
    o[c * 4 + r] = s;
  }
  return o;
}
function mulv(m, v) {
  const o = [0, 0, 0, 0];
  for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++) o[r] += m[c * 4 + r] * v[c];
  return o;
}

// ------------------------------------------------------------ renderer -----
export function createVabRenderer(canvas) {
  const gl = canvas.getContext('webgl', { antialias: true, preserveDrawingBuffer: true })
          || canvas.getContext('experimental-webgl', { antialias: true, preserveDrawingBuffer: true });
  if (!gl) return null;
  const extU32 = gl.getExtension('OES_element_index_uint');

  const cam = { yaw: 25 * Math.PI / 180, pitch: 12 * Math.PI / 180, dist: 90, target: [0, 20, 0] };
  const KEY = norm([-0.5, 0.8, 0.5]);
  const state = { key: null, meshes: [], gkey: null, ghost: null, blob: 0, halfW: 4, height: 1, scene: null };

  // Shader programs with attribute slots fixed, so one buffer layout serves all.
  const program = (vs, fs) => {
    const sh = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, sh(gl.VERTEX_SHADER, vs));
    gl.attachShader(p, sh(gl.FRAGMENT_SHADER, fs));
    gl.bindAttribLocation(p, 0, 'aPos'); gl.bindAttribLocation(p, 1, 'aNrm'); gl.bindAttribLocation(p, 2, 'aCol');
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
    const u = {};
    const n = gl.getProgramParameter(p, gl.ACTIVE_UNIFORMS);
    for (let i = 0; i < n; i++) { const info = gl.getActiveUniform(p, i); u[info.name] = gl.getUniformLocation(p, info.name); }
    return { p, u };
  };
  const lit = program(LIT_VS, LIT_FS), bg = program(BG_VS, BG_FS);
  const floor = program(FLOOR_VS, FLOOR_FS), blobP = program(BLOB_VS, BLOB_FS);

  const bgBuf = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, bgBuf);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 3, -1, -1, 3]), gl.STATIC_DRAW);
  const floorBuf = gl.createBuffer();
  const F = 150;
  gl.bindBuffer(gl.ARRAY_BUFFER, floorBuf);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-F, 0, -F, F, 0, -F, F, 0, F, -F, 0, -F, F, 0, F, -F, 0, F]), gl.STATIC_DRAW);
  const blobBuf = gl.createBuffer();

  function upload(B) {
    const vb = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vb);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(B.v), gl.STATIC_DRAW);
    const wide = !!extU32;
    if (!wide && B.count > 65535) throw new Error('assembly mesh too large for 16-bit indices');
    const ib = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, wide ? new Uint32Array(B.i) : new Uint16Array(B.i), gl.STATIC_DRAW);
    return { vb, ib, n: B.i.length, idxType: wide ? gl.UNSIGNED_INT : gl.UNSIGNED_SHORT };
  }
  const freeMesh = m => { if (m.gpu) { gl.deleteBuffer(m.gpu.vb); gl.deleteBuffer(m.gpu.ib); } };

  function rebuild(scene) {
    state.meshes.forEach(freeMesh);
    state.meshes = buildStack(scene).map(m => ({ ...m, gpu: upload(m.B), cyl: m.B.cyl }));
    let halfW = 1, height = 1;
    for (const m of state.meshes) for (const c of m.cyl) {
      halfW = Math.max(halfW, Math.abs(c.cx) + c.r); height = Math.max(height, c.y1);
    }
    state.halfW = halfW; state.height = height;
    const R = halfW * 1.6 + 1;
    const q = [-R, 0.01, -R, R, 0.01, -R, R, 0.01, R, -R, 0.01, -R, R, 0.01, R, -R, 0.01, R];
    gl.bindBuffer(gl.ARRAY_BUFFER, blobBuf);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(q), gl.STATIC_DRAW);
    state.blob = R;
  }

  function sync(scene) {
    const key = JSON.stringify([scene.asm.parts, scene.asm.boosters]);
    if (key !== state.key) { rebuild(scene); state.key = key; }
    const gh = scene.ghost;
    const gkey = gh ? JSON.stringify([gh.part, gh.d, gh.h, gh.radial, gh.core, gh.prop, gh.n]) : '';
    if (gkey !== state.gkey) {
      if (state.ghost) freeMesh(state.ghost);
      state.ghost = null;
      if (gh) { const m = buildGhostMesh(gh); state.ghost = { ...m, gpu: upload(m.B) }; }
      state.gkey = gkey;
    }
    state.scene = scene;
  }

  function camState() {
    const cp = Math.cos(cam.pitch);
    const eye = [cam.target[0] + cam.dist * Math.sin(cam.yaw) * cp,
                 cam.target[1] + cam.dist * Math.sin(cam.pitch),
                 cam.target[2] + cam.dist * Math.cos(cam.yaw) * cp];
    const w = canvas.clientWidth || 1, h = canvas.clientHeight || 1, asp = w / h;
    const near = Math.max(0.05, cam.dist * 0.01), far = cam.dist * 50 + 4000;
    const vp = mul(persp(FOV, asp, near, far), lookAt(eye, cam.target, [0, 1, 0]));
    return { vp, eye, w, h, asp };
  }

  function drawMesh(m, o) {
    gl.bindBuffer(gl.ARRAY_BUFFER, m.gpu.vb);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, m.gpu.ib);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 36, 0);
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 36, 12);
    gl.vertexAttribPointer(2, 3, gl.FLOAT, false, 36, 24);
    const u = lit.u;
    gl.uniform3fv(u.uOff, o.off);
    gl.uniform1f(u.uAlpha, o.alpha);
    gl.uniform3fv(u.uTint, o.tint);
    gl.uniform1f(u.uTintK, o.tintK);
    gl.uniform1f(u.uGhost, o.ghost ? 1 : 0);
    gl.uniform1f(u.uMetal, m.metal);
    gl.uniform1f(u.uShin, m.shin);
    gl.drawElements(gl.TRIANGLES, m.gpu.n, m.gpu.idxType, 0);
  }

  function draw(scene) {
    if (scene) sync(scene);
    const s = state.scene;
    gl.viewport(0, 0, canvas.width, canvas.height);
    gl.clearColor(0.02, 0.027, 0.039, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.enableVertexAttribArray(0); gl.enableVertexAttribArray(1); gl.enableVertexAttribArray(2);
    const cs = camState();
    const pxPerM = 2 * Math.tan(FOV / 2) / Math.max(1, canvas.clientHeight);

    // hangar backdrop, floor grid, contact shadow
    gl.disable(gl.DEPTH_TEST); gl.depthMask(false); gl.disable(gl.BLEND);
    gl.useProgram(bg.p);
    gl.bindBuffer(gl.ARRAY_BUFFER, bgBuf);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 8, 0);
    gl.uniform2f(bg.u.uRes, canvas.width, canvas.height);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    gl.enable(gl.DEPTH_TEST); gl.depthMask(true); gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.useProgram(floor.p);
    gl.uniformMatrix4fv(floor.u.uVP, false, cs.vp);
    gl.uniform3fv(floor.u.uEye, cs.eye);
    gl.uniform1f(floor.u.uPx, pxPerM);
    gl.bindBuffer(gl.ARRAY_BUFFER, floorBuf);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 12, 0);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
    gl.depthMask(false);
    gl.useProgram(blobP.p);
    gl.uniformMatrix4fv(blobP.u.uVP, false, cs.vp);
    gl.uniform1f(blobP.u.uR, state.blob);
    gl.bindBuffer(gl.ARRAY_BUFFER, blobBuf);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 12, 0);
    gl.drawArrays(gl.TRIANGLES, 0, 6);

    // the vehicle: opaque parts first, then the translucent fairing and ghost
    gl.depthMask(true);
    gl.useProgram(lit.p);
    gl.uniformMatrix4fv(lit.u.uVP, false, cs.vp);
    gl.uniform3fv(lit.u.uEye, cs.eye);
    gl.uniform3fv(lit.u.uKey, KEY);
    const tintFor = id => {
      if (s && s.selectedId && id === s.selectedId) return { tint: [0.94, 0.65, 0.0], tintK: 0.2 };
      if (s && s.hoverId && id === s.hoverId) return { tint: [1, 1, 1], tintK: 0.14 };
      return { tint: [0, 0, 0], tintK: 0 };
    };
    const off = [0, 0, 0];
    for (const m of state.meshes) if (!m.alpha) {
      const t = tintFor(m.id);
      drawMesh(m, { off, alpha: 1, ...t, ghost: false });
    }
    gl.depthMask(false);
    for (const m of state.meshes) if (m.alpha) {
      const t = tintFor(m.id);
      drawMesh(m, { off, alpha: 0.26, ...t, ghost: false });
    }
    if (state.ghost && s && s.ghost) {
      drawMesh(state.ghost, { off: [0, s.ghost.y, 0], alpha: 0.45, tint: [0, 0, 0], tintK: 0, ghost: true });
    }
    gl.depthMask(true);
  }

  // Ray through a canvas pixel: origin and unit direction.
  function ray(sx, sy) {
    const s = camState();
    const f = norm(sub(cam.target, s.eye));
    const r = norm(cross(f, [0, 1, 0])), u = cross(r, f);
    const tanH = Math.tan(FOV / 2), nx = (sx / s.w) * 2 - 1, ny = 1 - (sy / s.h) * 2;
    const d = norm([0, 1, 2].map(k => f[k] + r[k] * nx * tanH * s.asp + u[k] * ny * tanH));
    return { o: s.eye, d };
  }

  function hitCyl(o, d, c) {
    const ox = o[0] - c.cx, oz = o[2] - c.cz;
    const a = d[0] * d[0] + d[2] * d[2];
    if (a < 1e-12) return null;
    const b = 2 * (ox * d[0] + oz * d[2]), cc = ox * ox + oz * oz - c.r * c.r;
    const disc = b * b - 4 * a * cc;
    if (disc < 0) return null;
    const sq = Math.sqrt(disc);
    for (const t of [(-b - sq) / (2 * a), (-b + sq) / (2 * a)]) {
      if (t <= 0) continue;
      const y = o[1] + t * d[1];
      if (y >= c.y0 && y <= c.y1) return t;
    }
    return null;
  }

  return {
    draw,
    camera: cam,
    project(p) {
      const s = camState();
      const v = mulv(s.vp, [p[0], p[1], p[2], 1]), w = v[3];
      if (w <= 0) return { x: 0, y: 0, depth: w, visible: false };
      const nx = v[0] / w, ny = v[1] / w;
      return { x: (nx * 0.5 + 0.5) * s.w, y: (1 - (ny * 0.5 + 0.5)) * s.h, depth: w,
               visible: Math.abs(nx) <= 1.2 && Math.abs(ny) <= 1.2 };
    },
    pickPart(sx, sy) {
      if (!state.scene) return null;
      const { o, d } = ray(sx, sy);
      let best = null, bt = Infinity;
      for (const m of state.meshes) for (const c of m.cyl) {
        const t = hitCyl(o, d, c);
        if (t !== null && t < bt) { bt = t; best = m.id; }
      }
      return best;
    },
    // Height where the view ray passes closest to the vehicle axis, or null.
    heightAt(sx, sy) {
      const { o, d } = ray(sx, sy);
      const a = d[0] * d[0] + d[2] * d[2];
      if (a < 1e-12) return null;
      const t = -(o[0] * d[0] + o[2] * d[2]) / a;
      return t > 0 ? o[1] + t * d[1] : null;
    },
    fit(height, halfW = 6) {
      const asp = (canvas.clientWidth || 1) / (canvas.clientHeight || 1), t = Math.tan(FOV / 2);
      cam.target = [0, Math.max(height * 0.5, 1), 0];
      cam.dist = Math.max(height * 0.56 / t, halfW * 1.5 / (t * asp), 6);
    },
    resize() {
      const k = Math.min(3, window.devicePixelRatio || 1);
      canvas.width = Math.max(1, Math.round((canvas.clientWidth || 1) * k));
      canvas.height = Math.max(1, Math.round((canvas.clientHeight || 1) * k));
    },
  };
}
