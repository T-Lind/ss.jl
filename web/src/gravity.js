// Port of src/gravity.jl. Models are tagged objects; `gravity_accel` dispatches
// on `kind`, which matches how the Julia structs dispatch.
import { MU_EARTH, RE_EQ, J2_EARTH } from './constants.js';
import { vnorm, vscale, vsub, vadd } from './vec3.js';

export const pointMassGravity = (mu = MU_EARTH) => ({ kind: 'point', mu });
export const j2Gravity = (mu = MU_EARTH, re = RE_EQ, j2 = J2_EARTH) =>
  ({ kind: 'j2', mu, re, j2 });
export const thirdBodyGravity = (mu, ephemeris) =>
  ({ kind: 'third', mu, ephemeris });
export const compositeGravity = (...models) => ({ kind: 'composite', models });

function pointAccel(g, r) {
  const rn = vnorm(r);
  return vscale(r, -g.mu / (rn * rn * rn));
}

function j2Accel(g, r) {
  const [x, y, z] = r;
  const rn = vnorm(r);
  const r2 = rn * rn;
  const zr2 = (z * z) / r2;
  const k = 1.5 * g.j2 * (g.re * g.re) / r2;
  const c = -g.mu / (rn * r2);
  return [
    c * x * (1 + k * (1 - 5 * zr2)),
    c * y * (1 + k * (1 - 5 * zr2)),
    c * z * (1 + k * (3 - 5 * zr2)),
  ];
}

function thirdAccel(g, r, t) {
  const s = g.ephemeris(t);
  const d = vsub(s, r);
  const dn = vnorm(d);
  const sn = vnorm(s);
  return vsub(vscale(d, g.mu / (dn * dn * dn)),
              vscale(s, g.mu / (sn * sn * sn)));
}

export function gravity_accel(g, r, t) {
  switch (g.kind) {
    case 'point': return pointAccel(g, r);
    case 'j2': return j2Accel(g, r);
    case 'third': return thirdAccel(g, r, t);
    case 'composite': {
      let a = [0.0, 0.0, 0.0];
      for (const m of g.models) a = vadd(a, gravity_accel(m, r, t));
      return a;
    }
    default: throw new Error(`unknown gravity model ${g.kind}`);
  }
}
