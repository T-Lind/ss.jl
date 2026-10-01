// Port of src/mesh.jl: the procedural launch-vehicle geometry. Meshes are
// triangle soups — `tris` is a list of `[v1, v2, v3]` faces, each vertex an
// `[x, y, z]` array — with the same winding and ordering as the Julia.
import { engineCluster, stageBells } from '../static/engine_layout.js';
import { deg2rad_ } from './constants.js';
import { vadd, vsub, vcross, vdot, vnorm, vunit } from './vec3.js';
import { PROPELLANTS, bulk_density } from './engines.js';
import { barrel_length, stage_diameter } from './propulsion.js';
import { pod_radius } from './mission.js';

// ------------------------------------------------------------- primitives --

const triMesh = tris => ({ tris });
const _eqv = (a, b) => a[0] === b[0] && a[1] === b[1] && a[2] === b[2];

const face_normal2 = t => vcross(vsub(t[1], t[0]), vsub(t[2], t[0]));

export const mesh_area = m =>
  0.5 * m.tris.reduce((s, t) => s + vnorm(face_normal2(t)), 0.0);

export const mesh_volume = m =>
  m.tris.reduce((s, t) => s + vdot(t[0], vcross(t[1], t[2])), 0.0) / 6.0;

// Flip winding if the signed volume is negative (normals then point outward).
function ensure_outward(m) {
  return mesh_volume(m) >= 0 ? m : triMesh(m.tris.map(t => [t[0], t[2], t[1]]));
}

// Surface of revolution about the x-axis. `profile` is [x, radius] from nose
// to tail; radii of 0 at the ends close the body.
export function lathe_mesh(profile, { nseg = 48 } = {}) {
  const ring = (x, r) => {
    const out = [];
    for (let k = 0; k < nseg; k++) {
      const a = 2 * Math.PI * k / nseg;
      out.push([x, r * Math.cos(a), r * Math.sin(a)]);
    }
    return out;
  };
  const tris = [];
  let prev = null;
  for (const [x, r] of profile) {
    const cur = r > 1e-12 ? ring(x, r)
      : Array.from({ length: nseg }, () => [x, 0.0, 0.0]);
    if (prev !== null) {
      for (let k = 0; k < nseg; k++) {
        const k2 = (k + 1) % nseg;
        const a = prev[k], b = prev[k2], c = cur[k], d = cur[k2];
        if (!_eqv(a, b)) tris.push([a, c, b]);
        if (!_eqv(c, d)) tris.push([b, c, d]);
      }
    }
    prev = cur;
  }
  return ensure_outward(triMesh(tris));
}

// Axis-aligned closed box between corners lo and hi (12 triangles).
export function box_mesh(lo, hi) {
  const [x0, y0, z0] = lo, [x1, y1, z1] = hi;
  const p = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
             [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
  const f = [[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7], [0, 1, 5], [0, 5, 4],
             [1, 2, 6], [1, 6, 5], [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]];
  return ensure_outward(triMesh(f.map(([a, b, c]) => [p[a], p[b], p[c]])));
}

const merge_meshes = (...ms) => triMesh(ms.flatMap(m => m.tris));
const _translate_mesh = (m, d) =>
  triMesh(m.tris.map(t => [vadd(t[0], d), vadd(t[1], d), vadd(t[2], d)]));

// Closed engine bell: exit plane at xexit, throat len above it, offset to (y, z).
function _bell_mesh(xexit, len, rex, rt, y, z, { nseg = 24 } = {}) {
  const prof = [[xexit, 0.0], [xexit, rex]];
  for (const f of lin(0.0, 1.0, 7).slice(1))
    prof.push([xexit + len * f, rt + (rex - rt) * (1 - f) ** 1.6]);
  prof.push([xexit + len, 0.0]);
  return _translate_mesh(lathe_mesh(prof, { nseg }), [0.0, y, z]);
}

// Lay out n engine bells inside radius rmax, packed so they do not overlap.
const _cluster = engineCluster;

// Inclusive linspace, matching Julia's `range(a, b; length=n)`.
function lin(a, b, n) {
  return Array.from({ length: n }, (_, i) => a + (b - a) * i / (n - 1));
}

// Rotate about +z by pitch, then about +x by roll, then translate.
function _place_mesh(m, { roll = 0.0, pitch = 0.0, shift = [0.0, 0.0, 0.0] } = {}) {
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const f = v => {
    const [x, y, z] = v;
    const x1 = cp * x - sp * y, y1 = sp * x + cp * y;
    const y2 = cr * y1 - sr * z, z2 = sr * y1 + cr * z;
    return [x1 + shift[0], y2 + shift[1], z2 + shift[2]];
  };
  return triMesh(m.tris.map(t => [f(t[0]), f(t[1]), f(t[2])]));
}

const _negv = v => [-v[0], -v[1], -v[2]];

// Emit a quad as two triangles wound so the face normal follows `outward`.
function _quad(tris, p, q, r, s, outward) {
  if (vdot(vcross(vsub(q, p), vsub(r, p)), outward) >= 0) {
    tris.push([p, q, r]); tris.push([p, r, s]);
  } else {
    tris.push([p, r, q]); tris.push([p, s, r]);
  }
}

// True if angle a lies in the arc from a0 counter-clockwise to a1.
const _ang_in = (a, a0, a1) =>
  mod(a - a0, 2 * Math.PI) <= mod(a1 - a0, 2 * Math.PI);
const mod = (x, m) => x - m * Math.floor(x / m);

const TAPER_HALFANGLE = deg2rad_(17.0);

// Axial length of the cone taking radius r to the joint radius rj.
function _taper_len(r, rj, D) {
  const dr = Math.abs(rj - r);
  if (dr < 0.02 * r) return 0.0;
  return Math.min(6.0 * D, Math.max(0.10 * D, dr / Math.tan(TAPER_HALFANGLE)));
}

// Length of the transition cone joining a stage to the one above it.
export function interstage_length(d_below, d_above) {
  return _taper_len(d_below / 2, d_above / 2, d_below);
}

// Closed solid spanning x0..x1 axially, r0..r1 radially and the angular
// sector a0..a1 about +x.
function _arc_slab(x0, x1, r0, r1, a0, a1, { nseg = 10 } = {}) {
  const na = Math.max(2, nseg);
  const th = Array.from({ length: na + 1 }, (_, k) => a0 + (a1 - a0) * k / na);
  const P = (x, r, k) => [x, r * Math.cos(th[k]), r * Math.sin(th[k])];
  const rad = k => [0.0, Math.cos(th[k]), Math.sin(th[k])];
  const tng = k => [0.0, -Math.sin(th[k]), Math.cos(th[k])];
  const tris = [];
  for (let k = 0; k < na; k++) {
    _quad(tris, P(x0, r1, k), P(x1, r1, k), P(x1, r1, k + 1), P(x0, r1, k + 1), rad(k));
    _quad(tris, P(x0, r0, k), P(x1, r0, k), P(x1, r0, k + 1), P(x0, r0, k + 1), _negv(rad(k)));
    _quad(tris, P(x0, r0, k), P(x0, r1, k), P(x0, r1, k + 1), P(x0, r0, k + 1), [-1.0, 0.0, 0.0]);
    _quad(tris, P(x1, r0, k), P(x1, r1, k), P(x1, r1, k + 1), P(x1, r0, k + 1), [1.0, 0.0, 0.0]);
  }
  _quad(tris, P(x0, r0, 0), P(x1, r0, 0), P(x1, r1, 0), P(x0, r1, 0), _negv(tng(0)));
  _quad(tris, P(x0, r0, na), P(x1, r0, na), P(x1, r1, na), P(x0, r1, na), tng(na));
  return ensure_outward(triMesh(tris));
}

// Watertight shell of revolution. `prof` is the outer [x, radius] contour; the
// inner surface is offset by `thick` along the local surface normal.
function _shell_mesh(prof, thick, { nseg = 24, holes = [] } = {}) {
  const n = prof.length;
  if (n < 2) throw new Error('shell profile needs at least two points');
  const m = n - 1;
  const inw = new Array(n);
  for (let i = 1; i <= n; i++) {
    const i0 = Math.max(1, i - 1), i1 = Math.min(n, i + 1);
    let tx = prof[i1 - 1][0] - prof[i0 - 1][0];
    let tr = prof[i1 - 1][1] - prof[i0 - 1][1];
    let L = Math.hypot(tx, tr);
    if (L < 1e-12) { tx = 1.0; tr = 0.0; L = 1.0; }
    inw[i - 1] = [tr / L, -tx / L];
  }
  const th = Array.from({ length: nseg }, (_, k) => 2 * Math.PI * k / nseg);
  const ct = k => 2 * Math.PI * (k + 0.5) / nseg;
  const O = (i, k) => [prof[i - 1][0], prof[i - 1][1] * Math.cos(th[mod(k, nseg)]),
                       prof[i - 1][1] * Math.sin(th[mod(k, nseg)])];
  const Q = (i, k) => {
    const x = prof[i - 1][0] + thick * inw[i - 1][0];
    const r = Math.max(prof[i - 1][1] + thick * inw[i - 1][1], 1e-4);
    return [x, r * Math.cos(th[mod(k, nseg)]), r * Math.sin(th[mod(k, nseg)])];
  };
  const cut = (i, k) => holes.some(h =>
    i >= h[0] && i <= h[1] && _ang_in(ct(k), h[2], h[3]));
  const solid = (i, k) => 1 <= i && i <= m && !cut(i, mod(k, nseg));
  const sg = thick >= 0 ? 1.0 : -1.0;

  const tris = [];
  for (let i = 1; i <= m; i++) {
    for (let k = 0; k < nseg; k++) {
      if (!solid(i, k)) continue;
      const ca = Math.cos(ct(k)), sa = Math.sin(ct(k));
      const ox = -(inw[i - 1][0] + inw[i][0]);
      const orr = -(inw[i - 1][1] + inw[i][1]);
      const ref = [sg * ox, sg * orr * ca, sg * orr * sa];
      _quad(tris, O(i, k), O(i, k + 1), O(i + 1, k + 1), O(i + 1, k), ref);
      _quad(tris, Q(i, k), Q(i, k + 1), Q(i + 1, k + 1), Q(i + 1, k), _negv(ref));
    }
  }
  for (let i = 1; i <= m + 1; i++) {
    for (let k = 0; k < nseg; k++) {
      const a = solid(i - 1, k), b = solid(i, k);
      if (a === b) continue;
      const j = Math.min(n, Math.max(1, i));
      const ca = Math.cos(ct(k)), sa = Math.sin(ct(k));
      const t3 = [-inw[j - 1][1], inw[j - 1][1] * ca, inw[j - 1][1] * sa];
      _quad(tris, O(j, k), O(j, k + 1), Q(j, k + 1), Q(j, k), a ? t3 : _negv(t3));
    }
  }
  for (let i = 1; i <= m; i++) {
    for (let k = 0; k < nseg; k++) {
      const a = solid(i, k - 1), b = solid(i, k);
      if (a === b) continue;
      const ca = Math.cos(th[mod(k, nseg)]), sa = Math.sin(th[mod(k, nseg)]);
      const tg = [0.0, -sa, ca];
      _quad(tris, O(i, k), O(i + 1, k), Q(i + 1, k), Q(i, k), a ? tg : _negv(tg));
    }
  }
  return ensure_outward(triMesh(tris));
}

const POD_R_COEFF = 0.086;

// How many couches abreast a capsule of base radius rp can take.
function pod_crew(rp) {
  if (rp < 0.55) return 0;
  if (rp < 1.10) return 1;
  if (rp < 1.30) return 2;
  const R = 0.85 * 0.89 * rp;
  const hw = Math.min(0.14 * rp, 0.30);
  const P = Math.min(0.44 * rp, 0.86);
  const nwide = 1 + Math.floor(2 * (R - hw) / P);
  const nface = 1 + Math.floor(2 * (0.44 * rp) / P);
  return Math.min(6, Math.max(3, Math.min(nwide, nface)));
}

// Apollo-proportioned crew capsule: heat shield, conical pressure shell with
// glazed apertures, RCS quads, docking tunnel, and the cabin behind it all.
function pod_mesh({ radius = 0.75, nseg = 24, ncrew = 0 } = {}) {
  const rp = radius;
  const nc = Math.max(16, nseg);
  const Rs = 2.4 * rp;
  const xsh = Rs - Math.sqrt(Rs * Rs - rp * rp);
  const ts = 0.055 * rp;
  const tw = 0.050 * rp;
  const ta = Math.tan(deg2rad_(32.5));
  const rf = 0.26 * rp;
  const Lc = (rp - rf) / ta;
  const xb0 = xsh + 0.05 * rp;
  const xtop = xb0 + Lc;
  const hgt = xtop + 0.34 * rp;
  const crew = ncrew > 0 ? ncrew : pod_crew(rp);
  const ext = [], glass = [], cab = [];

  const shield = [];
  for (const f of lin(0.0, 1.0, 9)) {
    const u = rp * f;
    shield.push([Rs - Math.sqrt(Math.max(Rs * Rs - u * u, 0.0)), u]);
  }
  shield.push([xsh + 0.06 * rp, rp]);
  for (const f of lin(1.0, 0.0, 9)) {
    const u = 0.985 * rp * f;
    shield.push([ts + Rs - Math.sqrt(Math.max(Rs * Rs - u * u, 0.0)), u]);
  }
  ext.push(lathe_mesh(shield, { nseg: nc }));

  const NB = 12;
  const cone = [[xb0, rp]];
  for (let i = 1; i <= NB; i++) {
    const f = i / NB;
    cone.push([xb0 + f * Lc, rp + (rf - rp) * f]);
  }
  const rcone = x => rp - ta * (x - xb0);
  const wins = [[0.0, deg2rad_(22.0)], [deg2rad_(68.0), deg2rad_(15.0)],
                [deg2rad_(-68.0), deg2rad_(15.0)]];
  const wi0 = 5, wi1 = 7;
  const hatch = [Math.PI, deg2rad_(42.0)];
  const holes = wins.map(w => [wi0, wi1, w[0] - w[1], w[0] + w[1]]);
  ext.push(_shell_mesh(cone, tw, { nseg: nc, holes }));

  const csn = Math.sin(deg2rad_(32.5)), ccs = Math.cos(deg2rad_(32.5));
  const sink = (prof, d) => prof.map(p => [p[0] - d * csn, p[1] - d * ccs]);
  const bite = 0.010 * rp;

  for (const w of wins) {
    const fr = sink(cone.slice(wi0 - 2, wi1 + 2), bite);
    const ncell = fr.length - 1;
    ext.push(_shell_mesh(fr, -(0.030 * rp + bite), { nseg: nc, holes: [
      [1.0, ncell, w[0] + w[1] + 0.10, w[0] - w[1] - 0.10],
      [2.0, ncell - 1, w[0] - w[1], w[0] + w[1]]] }));
    const pn = sink(cone.slice(wi0 - 1, wi1 + 1), tw);
    glass.push(_shell_mesh(pn, 0.012 * rp, { nseg: nc, holes: [
      [1.0, pn.length - 1, w[0] + w[1], w[0] - w[1]]] }));
  }
  const hh = sink(cone.slice(3, 9), bite);
  const nhc = hh.length - 1;
  ext.push(_shell_mesh(hh, -(0.026 * rp + bite), { nseg: nc, holes: [
    [1.0, nhc, hatch[0] + hatch[1], hatch[0] - hatch[1]]] }));

  const xq = xb0 + 0.80 * Lc;
  const rq = rcone(xq);
  for (let i = 0; i <= 3; i++) {
    const a = 0.25 * Math.PI + i * 0.5 * Math.PI;
    ext.push(_place_mesh(box_mesh([xq - 0.10 * rp, rq - 0.02 * rp, -0.07 * rp],
                                  [xq + 0.10 * rp, rq + 0.05 * rp, 0.07 * rp]),
                         { roll: a }));
    for (const [sg, pit] of [[-1.0, 0.0], [1.0, Math.PI]]) {
      const noz = _place_mesh(_bell_mesh(0.0, 0.055 * rp, 0.030 * rp, 0.014 * rp,
                                         0.0, 0.0, { nseg: 10 }),
                              { pitch: pit, shift: [xq + sg * 0.105 * rp, rq + 0.02 * rp, 0.0] });
      ext.push(_place_mesh(noz, { roll: a }));
    }
  }
  ext.push(lathe_mesh([[xtop - 0.02 * rp, 0.0], [xtop - 0.02 * rp, rf],
    [xtop + 0.10 * rp, 0.245 * rp], [xtop + 0.16 * rp, 0.225 * rp],
    [xtop + 0.16 * rp, 0.0]], { nseg: nc }));
  ext.push(lathe_mesh([[xtop + 0.14 * rp, 0.0], [xtop + 0.14 * rp, 0.215 * rp],
    [hgt - 0.05 * rp, 0.200 * rp], [hgt - 0.05 * rp, 0.240 * rp],
    [hgt, 0.240 * rp], [hgt, 0.0]], { nseg: nc }));

  const rin = x => rcone(x) - tw;
  const inface = (x, fout, fin, dcap) =>
    Math.max(fin * rin(x), fout * rin(x) - dcap);
  const xfl = xb0 + 0.04 * rp;
  cab.push(lathe_mesh([[xfl, 0.0], [xfl, 0.96 * rin(xfl + 0.045 * rp)],
    [xfl + 0.045 * rp, 0.96 * rin(xfl + 0.045 * rp)], [xfl + 0.045 * rp, 0.0]],
    { nseg: nc }));
  const xa = xfl + 0.045 * rp;
  const zs = crew <= 0 ? [] : crew === 1 ? [0.0] : crew === 2 ? [-0.30 * rp, 0.30 * rp]
    : Array.from({ length: crew }, (_, i) => (i - (crew - 1) / 2) * Math.min(0.44 * rp, 0.86));
  const hw = Math.min(crew >= 3 ? 0.14 * rp : 0.17 * rp, 0.30);
  const ln = Math.min(0.60 * rin(xa + 0.07 * rp), 1.14);
  for (const zc of zs) {
    cab.push(box_mesh([xa + 0.02 * rp, -ln, zc - hw], [xa + 0.07 * rp, 0.36 * ln, zc + hw]));
    cab.push(box_mesh([xa + 0.055 * rp, 0.19 * ln, zc - 0.78 * hw],
                      [xa + 0.15 * rp, 0.41 * ln, zc + 0.78 * hw]));
    cab.push(_place_mesh(box_mesh([-0.025 * rp, -0.20 * ln, -0.88 * hw],
                                  [0.025 * rp, 0.20 * ln, 0.88 * hw]),
                         { pitch: deg2rad_(-50.0), shift: [xa + 0.10 * rp, -1.06 * ln, zc] }));
    for (const sg of [-1.0, 1.0])
      cab.push(box_mesh([xa + 0.05 * rp, -0.96 * ln, zc + sg * hw - 0.022 * rp],
                        [xa + 0.13 * rp, 0.31 * ln, zc + sg * hw + 0.022 * rp]));
    for (const [sy, sg] of [[-0.84, -1.0], [-0.84, 1.0], [0.24, -1.0], [0.24, 1.0]])
      cab.push(box_mesh([xfl + 0.030 * rp, sy * ln - 0.022 * rp, zc + sg * hw * 0.8 - 0.022 * rp],
                        [xa + 0.035 * rp, sy * ln + 0.022 * rp, zc + sg * hw * 0.8 + 0.022 * rp]));
  }

  const xcon = xb0 + 0.62 * Lc;
  const rcin = rin(xcon);
  const xf = xcon;
  const xfb = xf + 0.012 * rp;
  cab.push(lathe_mesh([[xcon, 0.20 * rcin], [xcon, 0.92 * rcin],
    [xcon + 0.05 * rp, 0.92 * rcin], [xcon + 0.05 * rp, 0.20 * rcin],
    [xcon, 0.20 * rcin]], { nseg: nc }));

  for (const [a0, a1] of [[deg2rad_(-46.0), deg2rad_(-18.0)],
                          [deg2rad_(-15.0), deg2rad_(15.0)],
                          [deg2rad_(18.0), deg2rad_(46.0)]]) {
    const b0 = a0 + 0.030, b1 = a1 - 0.030;
    cab.push(_arc_slab(xf - 0.055 * rp, xfb, 0.44 * rcin, 0.48 * rcin, a0, a1, { nseg: 7 }));
    cab.push(_arc_slab(xf - 0.055 * rp, xfb, 0.76 * rcin, 0.80 * rcin, a0, a1, { nseg: 7 }));
    cab.push(_arc_slab(xf - 0.059 * rp, xfb + 0.004 * rp, 0.435 * rcin, 0.805 * rcin,
                       a0 - 0.004, b0, { nseg: 2 }));
    cab.push(_arc_slab(xf - 0.059 * rp, xfb + 0.004 * rp, 0.435 * rcin, 0.805 * rcin,
                       b1, a1 + 0.004, { nseg: 2 }));
    cab.push(_arc_slab(xf - 0.030 * rp, xf - 0.022 * rp, 0.465 * rcin, 0.775 * rcin,
                       a0 + 0.020, a1 - 0.020, { nseg: 7 }));
    for (let i = 0; i <= 3; i++) for (let j = 0; j <= 1; j++) {
      const c0 = a0 + (a1 - a0) * (0.10 + 0.26 * i);
      const c1 = c0 + (a1 - a0) * 0.17;
      const r0 = (0.24 + 0.09 * j) * rcin;
      cab.push(_arc_slab(xf - 0.042 * rp, xfb, r0, r0 + 0.062 * rcin, c0, c1, { nseg: 3 }));
    }
    for (let i = 0; i <= 2; i++) {
      const c0 = a0 + (a1 - a0) * (0.12 + 0.32 * i);
      cab.push(_arc_slab(xf - 0.036 * rp, xfb, 0.83 * rcin, 0.905 * rcin,
                         c0, c0 + (a1 - a0) * 0.20, { nseg: 3 }));
    }
  }
  for (const sg of [-1.0, 1.0]) for (let row = 0; row <= 2; row++) {
    for (let i = 0; i <= 4; i++) {
      const a0 = sg * deg2rad_(58.0 + 13.0 * i);
      const r0 = (0.30 + 0.19 * row) * rcin;
      cab.push(_arc_slab(xf - 0.028 * rp, xfb, r0, r0 + 0.115 * rcin,
                         Math.min(a0, a0 + sg * deg2rad_(9.0)),
                         Math.max(a0, a0 + sg * deg2rad_(9.0)), { nseg: 3 }));
    }
  }
  for (const [a0, a1] of [[deg2rad_(100.0), deg2rad_(136.0)],
                          [deg2rad_(224.0), deg2rad_(260.0)]]) {
    const x0r = xa - 0.022 * rp, x1r = xa + 0.26 * rp;
    cab.push(_arc_slab(x0r, x1r, inface(x1r, 0.97, 0.72, 0.36), 0.97 * rin(x1r),
                       a0, a1, { nseg: 8 }));
    const x2r = x1r + 0.04 * rp, x3r = x1r + 0.30 * rp;
    cab.push(_arc_slab(x2r, x3r, inface(x3r, 0.96, 0.68, 0.40), 0.96 * rin(x3r),
                       a0 + 0.08, a1 - 0.08, { nseg: 8 }));
  }

  const xw0 = xb0 + 0.32 * Lc, xw1 = xb0 + 0.60 * Lc;
  for (const [xa_, xb_] of [[xfl + 0.05 * rp, xw0], [xw1, xtop - 0.10 * rp]]) {
    const ro = 0.985 * rin(xb_), ri = 0.945 * rin(xb_);
    cab.push(lathe_mesh([[xa_, ro], [xa_, ri], [xb_, ri], [xb_, ro], [xa_, ro]],
                        { nseg: nc }));
  }
  for (const a of [deg2rad_(34.0), deg2rad_(90.0), deg2rad_(270.0), deg2rad_(326.0)]) {
    for (const [x0h, x1h] of [[xa + 0.085 * rp, xw0 - 0.03 * rp],
                              [xw1 + 0.03 * rp, xtop - 0.14 * rp]]) {
      const rr = Math.max(0.82 * rin(x1h), 0.96 * rin(x1h) - 0.20);
      cab.push(_place_mesh(box_mesh([x0h, rr - 0.018 * rp, -0.018 * rp],
                                    [x1h, rr + 0.018 * rp, 0.018 * rp]), { roll: a }));
      for (const xs of [x0h, x1h])
        cab.push(_place_mesh(box_mesh([xs - 0.014 * rp, rr, -0.012 * rp],
                                      [xs + 0.014 * rp, 0.96 * rin(xs + 0.02 * rp), 0.012 * rp]),
                             { roll: a }));
    }
  }
  for (const a0 of [deg2rad_(6.0), deg2rad_(78.0), deg2rad_(150.0),
                    deg2rad_(222.0), deg2rad_(294.0)]) {
    const a1 = a0 + deg2rad_(56.0);
    const xl0 = xtop - 0.36 * rp, xl1 = xtop - 0.13 * rp;
    const lo = 0.96 * rin(xl1), li = inface(xl1, 0.96, 0.70, 0.38);
    cab.push(_arc_slab(xl0, xl1, li, lo, a0, a1, { nseg: 7 }));
    cab.push(_arc_slab(xl0 - 0.018 * rp, xl0 + 0.012 * rp,
                       li + 0.15 * (lo - li), lo - 0.15 * (lo - li),
                       a0 + 0.05, a1 - 0.05, { nseg: 6 }));
  }
  {
    const ah = Math.PI, dh = deg2rad_(38.0);
    const xh0 = xb0 + 0.26 * Lc, xh1 = xb0 + 0.72 * Lc;
    cab.push(_arc_slab(xh0, xh1, 0.93 * rin(xh1), 0.985 * rin(xh1),
                       ah - dh, ah - dh + 0.09, { nseg: 3 }));
    cab.push(_arc_slab(xh0, xh1, 0.93 * rin(xh1), 0.985 * rin(xh1),
                       ah + dh - 0.09, ah + dh, { nseg: 3 }));
  }
  for (const a of [0.0, 0.5 * Math.PI, 1.0 * Math.PI, 1.5 * Math.PI])
    cab.push(_place_mesh(box_mesh([xtop - 0.10 * rp, 0.105 * rp, -0.016 * rp],
                                  [xtop + 0.01 * rp, 0.180 * rp, 0.016 * rp]), { roll: a }));

  for (const [xa_, xb_] of [[xfl + 0.05 * rp, xw0], [xw1, xtop - 0.10 * rp]]) {
    for (const a of lin(0.0, 2 * Math.PI, 17).slice(0, -1)) {
      const rr = 0.945 * rin(xb_);
      cab.push(_place_mesh(box_mesh([xa_ + 0.01 * rp, rr - 0.010 * rp, -0.009 * rp],
                                    [xb_ - 0.01 * rp, rr + 0.003 * rp, 0.009 * rp]),
                           { roll: a }));
    }
  }
  for (const a of [deg2rad_(158.0), deg2rad_(202.0)]) {
    cab.push(_place_mesh(box_mesh([xa + 0.065 * rp, 0.90 * rin(xcon) - 0.030 * rp, -0.026 * rp],
                                  [xcon - 0.02 * rp, 0.90 * rin(xcon), 0.026 * rp]),
                         { roll: a }));
    for (const xs of [xa + 0.14 * rp, xa + 0.34 * rp, xa + 0.54 * rp]) {
      if (!(xs < xcon - 0.06 * rp)) continue;
      cab.push(_place_mesh(box_mesh([xs, 0.90 * rin(xcon) - 0.034 * rp, -0.034 * rp],
                                    [xs + 0.020 * rp, 0.97 * rin(xs + 0.02 * rp), 0.034 * rp]),
                           { roll: a }));
    }
  }
  for (const a of lin(0.0, 2 * Math.PI, 7).slice(0, -1)) {
    const xl = xcon - 0.10 * rp;
    const rl = rin(xl + 0.075 * rp);
    cab.push(_place_mesh(box_mesh([xl, 0.930 * rl, -0.060 * rp],
                                  [xl + 0.075 * rp, 0.985 * rl, 0.060 * rp]),
                         { roll: a + deg2rad_(26.0) }));
  }
  for (const zc of zs) for (const sg of [-1.0, 1.0])
    cab.push(box_mesh([xa - 0.005 * rp, 0.62 * ln, zc + sg * 0.62 * hw - 0.05 * rp],
                      [xa + 0.030 * rp, 0.62 * ln + 0.11 * rp, zc + sg * 0.62 * hw + 0.05 * rp]));
  for (const [a0, a1] of [[deg2rad_(146.0), deg2rad_(172.0)],
                          [deg2rad_(188.0), deg2rad_(214.0)]]) {
    const x0s = xw1 + 0.03 * rp, x1s = xtop - 0.20 * rp;
    if (!(x1s > x0s + 0.05 * rp)) continue;
    const so = 0.95 * rin(x1s), si = inface(x1s, 0.95, 0.70, 0.36);
    cab.push(_arc_slab(x0s, x1s, si, so, a0, a1, { nseg: 6 }));
    cab.push(_place_mesh(box_mesh(
      [0.5 * (x0s + x1s) - 0.02 * rp, si - 0.06 * (so - si), -0.018 * rp],
      [0.5 * (x0s + x1s) + 0.02 * rp, si + 0.12 * (so - si), 0.018 * rp]),
      { roll: 0.5 * (a0 + a1) }));
  }

  {
    const aw = crew >= 3 ? 0.028 * rp : crew === 2 ? 0.038 * rp : 0.055 * rp;
    const gw = Math.min(0.040 * rp, 0.9 * aw), pw = Math.min(0.020 * rp, 0.6 * aw);
    const zmax = 0.92 * rin(xa + 0.37 * rp);
    for (const zc of zs) for (const sg of [-1.0, 1.0]) {
      const za = zc + sg * (hw + aw + 0.012 * rp);
      if (!(Math.hypot(Math.abs(za) + aw, 0.34 * ln) < zmax)) continue;
      cab.push(box_mesh([xa + 0.035 * rp, -0.34 * ln, za - aw],
                        [xa + 0.150 * rp, 0.14 * ln, za + aw]));
      cab.push(box_mesh([xa + 0.115 * rp, -0.15 * ln, za - pw],
                        [xa + 0.205 * rp, -0.03 * ln, za + pw]));
      cab.push(box_mesh([xa + 0.190 * rp, -0.16 * ln, za - gw],
                        [xa + 0.240 * rp, 0.00 * ln, za + gw]));
      cab.push(box_mesh([xa + 0.060 * rp, 0.20 * ln, za - 0.9 * aw],
                        [xa + 0.125 * rp, 0.34 * ln, za + 0.9 * aw]));
    }
  }
  for (const sg of [-1.0, 1.0]) {
    const b0 = sg > 0 ? deg2rad_(49.0) : -deg2rad_(57.0);
    const b1 = sg > 0 ? deg2rad_(57.0) : -deg2rad_(49.0);
    cab.push(_arc_slab(xf - 0.026 * rp, xfb, 0.205 * rcin, 0.44 * rcin, b0, b1, { nseg: 3 }));
    for (let j = 0; j <= 2; j++) {
      const r0 = (0.225 + 0.070 * j) * rcin;
      cab.push(_arc_slab(xf - 0.040 * rp, xf - 0.019 * rp, r0, r0 + 0.052 * rcin,
                         b0 + 0.020, b1 - 0.020, { nseg: 3 }));
    }
  }
  for (const [a0, a1] of [[deg2rad_(100.0), deg2rad_(136.0)],
                          [deg2rad_(224.0), deg2rad_(260.0)]]) {
    const x2r = xa + 0.30 * rp, x3r = xa + 0.56 * rp;
    const rd = rin(x3r);
    const rri = inface(x3r, 0.96, 0.68, 0.40), rro = 0.96 * rd;
    const dd = rro - rri;
    for (let k = 0; k <= 2; k++) {
      const xd0 = x2r + 0.025 * rp + k * 0.078 * rp;
      const xd1 = xd0 + 0.060 * rp;
      cab.push(_arc_slab(xd0, xd1, rri + 0.036 * dd, rri + 0.286 * dd,
                         a0 + 0.12, a1 - 0.12, { nseg: 6 }));
      cab.push(_place_mesh(box_mesh(
        [0.5 * (xd0 + xd1) - 0.011 * rp, rri - 0.029 * dd, -0.055 * rp],
        [0.5 * (xd0 + xd1) + 0.011 * rp, rri + 0.129 * dd, 0.055 * rp]),
        { roll: 0.5 * (a0 + a1) }));
    }
  }
  for (const a0 of [deg2rad_(6.0), deg2rad_(78.0), deg2rad_(150.0),
                    deg2rad_(222.0), deg2rad_(294.0)]) {
    const xl0 = xtop - 0.36 * rp;
    const rl = rin(xtop - 0.13 * rp);
    cab.push(_place_mesh(box_mesh([xl0 - 0.014 * rp, 0.712 * rl, -0.050 * rp],
                                  [xl0 + 0.004 * rp, 0.762 * rl, 0.050 * rp]),
                         { roll: a0 + deg2rad_(28.0) }));
  }
  {
    const xh = xb0 + 0.50 * Lc, rh = 0.90 * rin(xb0 + 0.55 * Lc);
    cab.push(_place_mesh(box_mesh([xh - 0.05 * rp, rh - 0.045 * rp, -0.070 * rp],
                                  [xh + 0.05 * rp, rh + 0.020 * rp, 0.070 * rp]),
                         { roll: Math.PI }));
    cab.push(_place_mesh(box_mesh([xh - 0.016 * rp, rh - 0.130 * rp, -0.020 * rp],
                                  [xh + 0.016 * rp, rh - 0.030 * rp, 0.020 * rp]),
                         { roll: Math.PI }));
  }
  for (let k = 0; k <= 3; k++) {
    const yr = (-0.72 + 0.48 * k) * ln;
    cab.push(box_mesh([xa - 0.032 * rp, yr - 0.016 * rp, -0.86 * 0.96 * rin(xa)],
                      [xa + 0.018 * rp, yr + 0.016 * rp, 0.86 * 0.96 * rin(xa)]));
  }
  for (const [ac, awd] of [[0.0, deg2rad_(22.0)], [deg2rad_(68.0), deg2rad_(15.0)],
                           [deg2rad_(-68.0), deg2rad_(15.0)]]) {
    const cuts = [0.322, 0.352, 0.420, 0.500, 0.572, 0.605];
    const gp = 0.0022;
    for (let k = 1; k < cuts.length; k++) {
      const xk0 = xb0 + (cuts[k - 1] + (k === 1 ? 0.0 : gp)) * Lc;
      const xk1 = xb0 + (cuts[k] - (k === cuts.length - 1 ? 0.0 : gp)) * Lc;
      const rk = rin(xk1);
      if (k === 1 || k === cuts.length - 1) {
        cab.push(_arc_slab(xk0, xk1, 0.880 * rk, 0.985 * rk,
                           ac - awd - 0.09, ac + awd + 0.09, { nseg: 7 }));
      } else {
        for (const sg of [-1.0, 1.0])
          cab.push(_arc_slab(xk0, xk1, 0.872 * rk, 0.991 * rk,
                             Math.min(ac + sg * awd, ac + sg * (awd + 0.09)),
                             Math.max(ac + sg * awd, ac + sg * (awd + 0.09)), { nseg: 3 }));
      }
    }
  }
  if (crew <= 0) return [ext, glass, [], hgt];
  return [ext, glass, cab, hgt];
}

// An uncrewed payload: a spacecraft bus rather than a capsule.
function probe_mesh({ radius = 0.75, nseg = 24 } = {}) {
  const r = radius;
  const ext = [];
  const hb = 1.35 * r;
  ext.push(lathe_mesh([[0.0, 0.0], [0.0, 0.96 * r], [0.06 * r, r],
                       [hb - 0.06 * r, r], [hb, 0.90 * r], [hb, 0.0]], { nseg }));
  for (let k = 0; k <= 3; k++) {
    const a = deg2rad_(35.0 + 90.0 * k);
    ext.push(_place_mesh(box_mesh([0.28 * hb, 0.86 * r, -0.20 * r],
                                  [0.78 * hb, 1.06 * r, 0.20 * r]), { roll: a }));
  }
  ext.push(box_mesh([-0.30 * r, -0.05 * r, -0.05 * r], [0.0, 0.05 * r, 0.05 * r]));
  ext.push(lathe_mesh([[-0.30 * r, 0.0], [-0.30 * r, 0.62 * r],
                       [-0.10 * r, 0.30 * r], [-0.12 * r, 0.0]], { nseg }));
  for (const sg of [-1.0, 1.0]) for (let j = 0; j <= 1; j++)
    ext.push(box_mesh([0.20 * hb + j * 0.42 * hb, sg * 1.02 * r - sg * 0.02 * r, -0.62 * r],
                      [0.58 * hb + j * 0.42 * hb, sg * 1.02 * r + sg * 0.02 * r, 0.62 * r]));
  for (let k = 0; k <= 3; k++) {
    const a = deg2rad_(90.0 * k);
    ext.push(_place_mesh(lathe_mesh([[0.02 * r, 0.0], [0.02 * r, 0.055 * r],
                                     [0.14 * r, 0.10 * r]], { nseg: 8 }),
                         { shift: [0.0, 0.90 * r, 0.0], roll: a }));
  }
  return [ext, [], [], hb];
}

// Compact two-stage lunar lander for the launch-stack payload bay.
function _lander_payload_mesh(diameter, {nseg=32}={}) {
  const s=diameter/4.2, v=(x,y,z)=>[s*y,-s*x,s*z],parts=[];
  const box=(c,size)=>parts.push(box_mesh(v(c[0]+size[0]/2,c[1]-size[1]/2,c[2]-size[2]/2),
    v(c[0]-size[0]/2,c[1]+size[1]/2,c[2]+size[2]/2)));
  const rod=(a,b,radius)=>{
    const p=v(...a),q=v(...b),ax=vunit(vsub(q,p));
    const side=vunit(vcross(ax,Math.abs(ax[1])>.9?[1,0,0]:[0,1,0])),up=vcross(ax,side);
    const len=vnorm(vsub(q,p)),r=radius*s;
    const m=lathe_mesh([[0,0],[0,r],[len,r],[len,0]],{nseg:10});
    const f=t=>vadd(p,vadd(ax.map(x=>x*t[0]),vadd(side.map(x=>x*t[1]),up.map(x=>x*t[2]))));
    parts.push(triMesh(m.tris.map(t=>t.map(f))));
  };
  parts.push(lathe_mesh([[.82*s,0],[.82*s,1.28*s],[1.62*s,1.34*s],[1.74*s,1.34*s],[1.74*s,0]],{nseg:8}));
  parts.push(_bell_mesh(.34*s,.64*s,.43*s,.16*s,0,0,{nseg:24}));
  const yz=[[1.83,-.91],[1.97,-1.05],[3.86,-1.05],[4.14,-.77],[4.14,.77],[3.86,1.05],[1.97,1.05],[1.83,.91]],tris=[];
  for(let i=0;i<8;i++) {
    const j=(i+1)%8,a=v(-.98,...yz[i]),b=v(-.98,...yz[j]),c=v(1.06,...yz[j]),d=v(1.06,...yz[i]);
    tris.push([a,b,c],[a,c,d],[v(-.98,2.98,0),b,a],[v(1.06,2.98,0),d,c]);
  }
  parts.push(ensure_outward(triMesh(tris)));
  box([-1.11,2.92,0],[.26,1.65,1.5]);
  for(const z of [-.53,.53]) box([1.087,3.49,z],[.045,.725,.805]);
  box([1.11,2.22,0],[.06,.72,.63]); box([1.36,1.80,0],[.56,.07,.66]);
  for(let k=0;k<4;k++) {
    const a=Math.PI/4+k*Math.PI/2,c=Math.cos(a),z=Math.sin(a);
    const hip=[c*1.13,1.52,z*1.13],ankle=[c*1.84,.17,z*1.84],knee=hip.map((v,i)=>v+.47*(ankle[i]-v));
    rod(hip,ankle,.065); rod(ankle,[ankle[0],.105,ankle[2]],.065);
    for(const sign of [-1,1]) {
      const aa=a+sign*.33; rod([Math.cos(aa)*.91,.88,Math.sin(aa)*.91],knee,.034);
    }
    const pad=lathe_mesh([[0,0],[0,.26*s],[.105*s,.20*s],[.105*s,0]],{nseg:20});
    parts.push(_translate_mesh(pad,[0,-s*ankle[0],s*ankle[2]]));
  }
  for(const z of [-.27,.27]) rod([1.33,1.80,z],[1.78,.15,z],.025);
  for(let i=0;i<=8;i++) { const f=i/8; rod([1.33+.45*f,1.80-1.65*f,-.27],[1.33+.45*f,1.80-1.65*f,.27],.02); }
  for(const sign of [-1,1]) rod([-.51,2.07,sign*1.22],[-.51,3.38,sign*1.22],.20);
  parts.push(lathe_mesh([[4.14*s,0],[4.14*s,.33*s],[4.37*s,.33*s],[4.37*s,.40*s],[4.42*s,.40*s],[4.42*s,0]],{nseg:24}));
  rod([-.66,4.06,.53],[-.66,4.72,.53],.025);
  return [parts,4.72*s];
}

// Procedural launch-vehicle geometry with real detailing.
export function rocket_mesh(a, b = {}) {
  let o;
  if (a && a.stages) {
    const lv = a;
    const diameter = b.diameter ?? 2 * Math.sqrt(lv.sref / Math.PI);
    o = {
      diameter,
      payload_mass: lv.payload_mass,
      fairing: lv.fairing_mass > 0,
      prop_masses: lv.stages.map(s => s.mprop),
      densities: lv.stages.map(s => bulk_density(s.prop)),
      n_engines: lv.stages.map(s => s.n_engines),
      diameters: lv.stages.map(s => stage_diameter(s, diameter)),
      boosters: lv.boosters.map(x => ({
        count: x.count, diameter: stage_diameter(x.stage, diameter),
        prop_mass: x.stage.mprop, density: bulk_density(x.stage.prop),
        n_engines: x.stage.n_engines })),
      ...b,
    };
  } else {
    o = a;
  }
  const {
    diameter = 1.8,
    prop_masses = [42000.0, 9500.0, 950.0],
    n_engines = prop_masses.map(() => 1),
    boosters = [],
    payload_mass = 350.0,
    pod_diameter = 0.0,
    crewed = true,
    payload_kind = 'capsule',
    payload_diameter = 0.0,
    nseg = 48,
  } = o;
  const densities = o.densities ??
    prop_masses.map(() => bulk_density(PROPELLANTS.kerolox));
  const diameters = o.diameters ?? prop_masses.map(() => diameter);
  const fairing = o.fairing ?? true;
  if (prop_masses.length !== densities.length)
    throw new Error('prop_masses and densities length mismatch');
  if (n_engines.length !== prop_masses.length)
    throw new Error('n_engines and prop_masses length mismatch');
  if (diameters.length !== prop_masses.length)
    throw new Error('diameters and prop_masses length mismatch');
  if (!diameters.every(d => d > 0)) throw new Error('stage diameters must be positive');
  const fairing_len = o.fairing_len ?? 2.2 * diameters[diameters.length - 1];
  const K = prop_masses.length;
  let D = diameters[0];
  let r = D / 2;
  const nb = Math.max(16, Math.floor(nseg / 2));
  const parts = [];
  const sections = [];
  let tcount = 0;
  const finish = (name, x0, x1, ms) => {
    const n = ms.reduce((s, m) => s + m.tris.length, 0);
    for (const m of ms) parts.push(m);
    sections.push({ name, x0, x1, t0: tcount + 1, t1: tcount + n });
    tcount += n;
  };
  const raceway = (xa, xb, rr, DD) =>
    box_mesh([xa, 0.955 * rr, -0.030 * DD], [xb, rr + 0.048 * DD, 0.030 * DD]);

  let x = 0.28 * D;
  let rtop_prev = D / 2;
  for (let k = 1; k <= K; k++) {
    D = diameters[k - 1];
    r = D / 2;
    const mp = prop_masses[k - 1], rho = densities[k - 1], ne = n_engines[k - 1];
    const len = barrel_length(mp, rho, D);
    const rj = k < K ? diameters[k] / 2 : r;
    const ltap = _taper_len(r, rj, D);
    const xtop = x + len + ltap;
    const ms = [];
    if (k === 1) {
      const prof1 = [[0.0, 0.0], [0.0, 0.80 * r], [0.28 * D, r], [x + len, r]];
      if (ltap > 0) prof1.push([xtop, rj]);
      prof1.push([xtop, 0.0]);
      ms.push(lathe_mesh(prof1, { nseg }));
      const bells = stageBells(ne,D,0,K);
      for (const [by,bz,bs] of bells.pts)
        ms.push(_bell_mesh(bells.exitX,bells.length,bs,bells.throatRadius,by,bz,{nseg:nb}));
      ms.push(raceway(0.30 * D, x + len - 0.02 * D, r, D));
      finish('stage1', Math.min(0,bells.exitX), xtop, ms);
    } else {
      const rbase = Math.min(0.945 * r, 0.98 * rtop_prev);
      const profk = [[x, 0.0], [x, rbase], [x + 0.10 * D, rbase],
                     [x + 0.13 * D, r], [x + len, r]];
      if (ltap > 0) profk.push([xtop, rj]);
      profk.push([xtop, 0.0]);
      ms.push(lathe_mesh(profk, { nseg }));
      const [bex, blen, thr] = k === K ? [0.085 * D, 0.22 * D, 0.53]
                                       : [0.155 * D, 0.36 * D, 0.29];
      for (const [by, bz, bs] of _cluster(ne, 0.80 * r, bex)) {
        const L = blen * bs / bex;
        ms.push(_bell_mesh(x + 0.04 * D - L, L, bs, thr * bs, by, bz, { nseg: nb }));
      }
      if (k < K) ms.push(raceway(x + 0.12 * D, x + len - 0.02 * D, r, D));
      if (k === K) {
        const xm = x + 0.5 * len;
        const rc = 0.955 * r + 0.024 * D;
        for (const [py, pz] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
          const hy = py === 0 ? 0.033 * D : 0.024 * D;
          const hz = pz === 0 ? 0.033 * D : 0.024 * D;
          ms.push(box_mesh([xm - 0.05 * D, py * rc - hy, pz * rc - hz],
                           [xm + 0.05 * D, py * rc + hy, pz * rc + hz]));
        }
        ms.push(lathe_mesh([[x + len, 0.0], [x + len, 0.90 * r],
                            [x + len + 0.13 * D, 0.44 * r], [x + len + 0.13 * D, 0.0]],
                           { nseg }));
      }
      finish('stage' + k, x, xtop + (k === K ? 0.13 * D : 0.0), ms);
    }
    x += len + ltap;
    rtop_prev = ltap > 0 ? rj : r;
  }
  D = diameters[K - 1];
  r = D / 2;
  const rp = payload_kind === 'lander'
    ? (payload_diameter > 0 ? payload_diameter / 2 : 0.45 * D)
    : (pod_diameter > 0 ? pod_diameter / 2
       : Math.min(pod_radius(payload_mass), 1.25 * Math.max(...diameters) / 2));
  const xb = x + 0.13 * D + 0.02;
  let lp;
  if (payload_kind === 'lander') {
    const [lparts, lpp] = _lander_payload_mesh(2 * rp, { nseg: Math.max(20, Math.floor(nseg / 2)) });
    lp = lpp;
    const shifted = lparts.map(m => _place_mesh(m, { shift: [xb, 0.0, 0.0] }));
    finish('lander', x, xb + lp, shifted);
  } else {
    const [phull, pglass, pcab, lpp] = crewed
      ? pod_mesh({ radius: rp, nseg: Math.max(20, Math.floor(nseg / 2)) })
      : probe_mesh({ radius: rp, nseg: Math.max(20, Math.floor(nseg / 2)) });
    lp = lpp;
    const shift = [xb, 0.0, 0.0];
    const place = (nm, ms) => finish(nm, x, xb + lp,
      ms.map(m => _place_mesh(m, { shift })));
    place('pod', phull);
    if (pglass.length > 0) place('glass', pglass);
    if (pcab.length > 0) place('cabin', pcab);
  }
  if (fairing) {
    const x0f = x;
    const rF = Math.max(r, rp / 0.90);
    const flen = Math.max(fairing_len * rF / r, (xb + lp - x0f) + 0.55 * rF);
    const xsh = x0f + 0.12 * flen;
    const prof = [[x0f, 0.0], [x0f, rF], [xsh, rF]];
    for (const f of lin(0.0, 1.0, 11).slice(1, -1))
      prof.push([xsh + f * 0.88 * flen, rF * (1 - f * f) ** 0.60]);
    prof.push([x0f + 0.985 * flen, 0.055 * rF]);
    prof.push([x0f + flen, 0.0]);
    finish('fairing', x0f, x0f + flen, [lathe_mesh(prof, { nseg })]);
  }

  const r1 = diameters[0] / 2;
  for (let bi = 0; bi < boosters.length; bi++) {
    const bset = boosters[bi];
    const db = bset.diameter;
    const rb = db / 2;
    const lb = bset.prop_mass / (bset.density * Math.PI * rb * rb) * 1.15 + 0.9 * db;
    const xn = 0.28 * db + lb;
    const bprof = [[0.0, 0.0], [0.0, 0.80 * rb], [0.28 * db, rb], [xn, rb]];
    for (const f of lin(0.0, 1.0, 8).slice(1, -1))
      bprof.push([xn + f * 1.75 * rb, rb * (1 - f * f) ** 0.55]);
    bprof.push([xn + 1.75 * rb, 0.0]);
    const one = [lathe_mesh(bprof, { nseg: Math.max(16, Math.floor(nseg / 2)) })];
    const bells = _cluster(bset.n_engines,0.78*rb,0.105*db);
    for (const [by,bz,bs] of bells)
      one.push(_bell_mesh(.08*db-3.6*bs,3.6*bs,bs,.46*bs,by,bz,{nseg:nb}));
    const R = r1 + rb;
    const set = [];
    for (let j = 0; j < bset.count; j++) {
      const ang = 2 * Math.PI * j / bset.count;
      const sh = [0.0, R * Math.cos(ang), R * Math.sin(ang)];
      for (const m of one) set.push(_place_mesh(m, { roll: ang, shift: sh }));
    }
    finish('booster' + (bi+1), Math.min(0,.08*db-3.6*bells[0][2]), xn + 1.75 * rb, set);
  }
  return [merge_meshes(...parts), sections];
}
