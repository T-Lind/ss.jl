// Port of src/terrain.jl: the deterministic procedural lunar surface and the
// surface model the descent flies over. The 32-bit hash is reproduced exactly
// with Math.imul.
import { R_MOON } from './constants.js';
import { vadd, vsub, vscale, vdot, vcross, vunit } from './vec3.js';
import { moonfixed } from './moon.js';

const u32 = x => x >>> 0;
function _hash32(a, b, c, d) {
  let h = Math.imul(a >>> 0, 0x9E3779B1);
  h = Math.imul(h ^ (b >>> 0), 0x85EBCA77);
  h = Math.imul(h ^ (c >>> 0), 0xC2B2AE3D);
  h = Math.imul(h ^ (d >>> 0), 0x27D4EB2F);
  h ^= h >>> 15;
  h = Math.imul(h, 0x2545F491);
  h ^= h >>> 13;
  h = Math.imul(h, 0x9E3779B1);
  h ^= h >>> 16;
  return h >>> 0;
}
const _cell_hash = (i, j, k, salt) => _hash32(u32(i), u32(j), u32(k), u32(salt));
const _cell_rand = (h, n) => _hash32(h, 0x000000FF, u32(n), 0x165667B1) / 4.294967296e9;

function _dominant_face(u) {
  const ax = Math.abs(u[0]), ay = Math.abs(u[1]), az = Math.abs(u[2]);
  if (ax >= ay && ax >= az) return u[0] >= 0 ? 0 : 1;
  if (ay >= az) return u[1] >= 0 ? 2 : 3;
  return u[2] >= 0 ? 4 : 5;
}

function _face_project(u, f) {
  const sg = f % 2 === 0 ? 1.0 : -1.0;
  let w;
  if (f < 2) { w = sg * u[0]; if (w <= 0.35) return [false, 0, 0]; return [true, sg * u[1] / w, u[2] / w]; }
  if (f < 4) { w = sg * u[1]; if (w <= 0.35) return [false, 0, 0]; return [true, sg * u[2] / w, u[0] / w]; }
  w = sg * u[2]; if (w <= 0.35) return [false, 0, 0]; return [true, sg * u[0] / w, u[1] / w];
}

function _face_unit(f, s, t) {
  const sg = f % 2 === 0 ? 1.0 : -1.0;
  if (f < 2) return vunit([sg, sg * s, t]);
  if (f < 4) return vunit([t, sg, sg * s]);
  return vunit([sg * s, t, sg]);
}

export const lunarTerrain = (o = {}) => ({
  seed: o.seed ?? 0x00C0FFEE,
  relief: o.relief ?? 1800.0,
  d_max: o.d_max ?? 40.0e3,
  classes: o.classes ?? 8,
  ratio: o.ratio ?? 2.6,
  density: o.density ?? 0.18,
  rough: o.rough ?? 0.35,
});
export const highland_terrain = ({ seed = 0x00BADBED } = {}) =>
  lunarTerrain({ seed, relief: 3000.0, d_max: 70.0e3, classes: 9, ratio: 2.4, density: 0.35, rough: 1.2 });
export const mare_terrain = ({ seed = 0x000A11CE } = {}) =>
  lunarTerrain({ seed, relief: 700.0, d_max: 18.0e3, classes: 7, ratio: 2.6, density: 0.10, rough: 0.18 });

function _relief(tr, u) {
  const p1 = _cell_rand(tr.seed, 1) * 6.2831853;
  const p2 = _cell_rand(tr.seed, 2) * 6.2831853;
  const p3 = _cell_rand(tr.seed, 3) * 6.2831853;
  const x = u[0], y = u[1], z = u[2];
  let s = 0.55 * Math.sin(1.7 * x + 2.1 * y + p1) * Math.cos(1.3 * z - 0.9 * y + p2);
  s += 0.28 * Math.sin(3.9 * y - 2.7 * z + p2) * Math.cos(3.1 * x + p3);
  s += 0.17 * Math.sin(7.3 * z + 5.1 * x + p3) * Math.sin(6.1 * y + p1);
  return tr.relief * s;
}

function _crater_profile(s) {
  if (s <= 1.0) return -1.0 + 1.15 * s * s * (3.0 - 2.0 * s);
  if (s < 2.2) { const e = (2.2 - s) / 1.2; return 0.15 * e * e * (3.0 - 2.0 * e); }
  return 0.0;
}

const _crater_depth = d => Math.min(0.2 * d, 1044.0 * (d / 1000.0) ** 0.301);

function _craters_class(tr, u, cls) {
  const d0 = tr.d_max / tr.ratio ** (cls - 1);
  const reach = 1.15 * d0 / R_MOON;
  let n = Math.max(2, Math.ceil(1.0 / Math.max(reach, 1e-9)));
  if (n > 1e6) n = 1e6;
  const cell = 2.0 / n;
  const salt = (Math.imul(0x51ED270B, cls) ^ tr.seed) >>> 0;
  let h = 0.0;
  for (let f = 0; f < 6; f++) {
    const [ok, s, t] = _face_project(u, f);
    if (!ok) continue;
    if (Math.abs(s) > 1.0 + 2 * cell || Math.abs(t) > 1.0 + 2 * cell) continue;
    const i0 = Math.floor((s + 1.0) / cell), j0 = Math.floor((t + 1.0) / cell);
    for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
      const i = i0 + di, j = j0 + dj;
      const ch = _cell_hash(i, j, f, salt);
      if (!(_cell_rand(ch, 0) < tr.density)) continue;
      const cs = -1.0 + (i + _cell_rand(ch, 1)) * cell;
      const ct = -1.0 + (j + _cell_rand(ch, 2)) * cell;
      if (Math.abs(cs) > 1.0 || Math.abs(ct) > 1.0) continue;
      const cu = _face_unit(f, cs, ct);
      if (_dominant_face(cu) !== f) continue;
      const dia = d0 * (0.55 + 0.9 * _cell_rand(ch, 3));
      const rad = 0.5 * dia;
      const ang = Math.acos(Math.min(1, Math.max(-1, vdot(u, cu))));
      const dist = R_MOON * ang;
      if (dist > 2.2 * rad) continue;
      h += _crater_depth(dia) * _crater_profile(dist / rad);
    }
  }
  return h;
}

const _smooth = x => x * x * (3.0 - 2.0 * x);

function _value_noise(u, wave, salt) {
  const p = vscale(u, R_MOON / wave);
  const i0 = Math.floor(p[0]), j0 = Math.floor(p[1]), k0 = Math.floor(p[2]);
  const fx = _smooth(p[0] - i0), fy = _smooth(p[1] - j0), fz = _smooth(p[2] - k0);
  let acc = 0.0;
  for (let dk = 0; dk <= 1; dk++) {
    const wz = dk === 0 ? 1.0 - fz : fz;
    for (let dj = 0; dj <= 1; dj++) {
      const wy = dj === 0 ? 1.0 - fy : fy;
      for (let di = 0; di <= 1; di++) {
        const wx = di === 0 ? 1.0 - fx : fx;
        const g = _cell_hash(i0 + di, j0 + dj, k0 + dk, salt) / 4.294967296e9;
        acc += wx * wy * wz * (2.0 * g - 1.0);
      }
    }
  }
  return acc;
}

export function terrain_height(tr, u) {
  let h = _relief(tr, u);
  for (let c = 1; c <= tr.classes; c++) h += _craters_class(tr, u, c);
  h += tr.rough * _value_noise(u, 40.0, (tr.seed ^ 0x7A5C1E39) >>> 0);
  h += 0.45 * tr.rough * _value_noise(u, 15.0, (tr.seed ^ 0x1B873593) >>> 0);
  return h;
}

export const terrain_radius = (tr, u) => R_MOON + terrain_height(tr, u);

function _tangents(u) {
  const a = Math.abs(u[2]) < 0.9 ? [0.0, 0.0, 1.0] : [1.0, 0.0, 0.0];
  const e = vunit(vcross(a, u));
  return [e, vcross(u, e)];
}

export function terrain_normal(tr, u, { baseline = 8.0 } = {}) {
  const [e1, e2] = _tangents(u);
  const d = baseline / R_MOON;
  const hp1 = terrain_radius(tr, vunit(vadd(u, vscale(e1, d))));
  const hm1 = terrain_radius(tr, vunit(vsub(u, vscale(e1, d))));
  const hp2 = terrain_radius(tr, vunit(vadd(u, vscale(e2, d))));
  const hm2 = terrain_radius(tr, vunit(vsub(u, vscale(e2, d))));
  const g1 = (hp1 - hm1) / (2 * baseline);
  const g2 = (hp2 - hm2) / (2 * baseline);
  return vunit(vsub(u, vadd(vscale(e1, g1), vscale(e2, g2))));
}

export const terrain_slope = (tr, u, { baseline = 8.0 } = {}) =>
  Math.acos(Math.min(1, Math.max(-1, vdot(terrain_normal(tr, u, { baseline }), u))));

export const surface_offset = (u, e1, e2, d1, d2) =>
  vunit(vadd(u, vadd(vscale(e1, d1 / R_MOON), vscale(e2, d2 / R_MOON))));

export function site_hazard(tr, u, { radius = 15.0, baseline = 8.0, samples = 8 } = {}) {
  const [e1, e2] = _tangents(u);
  const h0 = terrain_radius(tr, u);
  let hmin = h0, hmax = h0;
  let worst = terrain_slope(tr, u, { baseline });
  for (let k = 0; k < samples; k++) {
    const a = 2 * Math.PI * k / samples;
    for (const fr of [0.55, 1.0]) {
      const p = surface_offset(u, e1, e2, fr * radius * Math.cos(a), fr * radius * Math.sin(a));
      const hh = terrain_radius(tr, p);
      if (hh < hmin) hmin = hh;
      if (hh > hmax) hmax = hh;
      const sl = terrain_slope(tr, p, { baseline });
      if (sl > worst) worst = sl;
    }
  }
  const spread = hmax - hmin;
  return { score: Math.max(worst, Math.atan(spread / radius)), slope: worst, relief: spread };
}

export function safe_site(tr, u0, e_down, e_cross,
                          { reach = 900.0, step = 120.0, cross_reach = 360.0, radius = 15.0 } = {}) {
  let best_u = u0;
  let best = site_hazard(tr, u0, { radius }).score;
  let best_d = 0.0, best_c = 0.0;
  const penalty = 1.0e-6;
  const nd = Math.floor(reach / step), nc = Math.floor(cross_reach / step);
  for (let i = -nd; i <= nd; i++) for (let j = -nc; j <= nc; j++) {
    if (i === 0 && j === 0) continue;
    const d = i * step, c = j * step;
    const u = surface_offset(u0, e_down, e_cross, d, c);
    const sc = site_hazard(tr, u, { radius }).score + penalty * Math.hypot(d, c);
    if (sc < best) { best = sc; best_u = u; best_d = d; best_c = c; }
  }
  return [best_u, best_d, best_c, best];
}

// --------------------------------------------------------- surface model ---
export const surfaceModel = (terrain, eph) => ({ terrain, eph });

export const surface_radius = (sm, r, t) =>
  sm == null ? R_MOON : terrain_radius(sm.terrain, vunit(moonfixed(r, t, sm.eph)));
export const surface_altitude = (sm, r, t) =>
  Math.sqrt(vdot(r, r)) - surface_radius(sm, r, t);
export const ground_elevation = (sm, r, t) =>
  sm == null ? 0.0 : surface_radius(sm, r, t) - R_MOON;

export function terrain_profile(sm, r0, v0, t, { span = 20.0e3, n = 200 } = {}) {
  const u0 = vunit(moonfixed(r0, t, sm.eph));
  const vf = moonfixed(v0, t, sm.eph);
  const e = vunit(vsub(vf, vscale(u0, vdot(vf, u0))));
  const arc = [], el = [];
  for (let k = 1; k <= n; k++) {
    const d = -span / 2 + span * (k - 1) / (n - 1);
    arc.push(d);
    el.push(terrain_height(sm.terrain, vunit(vadd(u0, vscale(e, d / R_MOON)))));
  }
  return [arc, el];
}
