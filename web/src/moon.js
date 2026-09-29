// Port of src/moon.jl plus the Moon-fixed frame helpers from src/terrain.jl.
import { A_MOON, N_MOON, MU_MOON, R_MOON, deg2rad_ } from './constants.js';
import { vadd, vsub, vscale, vdot, vcross, vnorm, vunit } from './vec3.js';

export const circularMoonEphemeris = ({ p, q, a, n, phase0 }) => ({ p, q, a, n, phase0 });

export function moon_position(e, t) {
  const th = e.phase0 + e.n * t;
  const c = Math.cos(th), s = Math.sin(th);
  return [e.a * (c * e.p[0] + s * e.q[0]),
          e.a * (c * e.p[1] + s * e.q[1]),
          e.a * (c * e.p[2] + s * e.q[2])];
}

export function moon_velocity(e, t) {
  const th = e.phase0 + e.n * t;
  const c = Math.cos(th), s = Math.sin(th);
  const an = e.a * e.n;
  return [an * (-s * e.p[0] + c * e.q[0]),
          an * (-s * e.p[1] + c * e.q[1]),
          an * (-s * e.p[2] + c * e.q[2])];
}

export function coplanar_moon(r, v, { phase0 = 0.0, a = A_MOON } = {}) {
  const p = vunit(r);
  const h = vcross(r, v);
  const q = vunit(vcross(h, r));
  const c = Math.cos(phase0), s = Math.sin(phase0);
  const pp = vadd(vscale(p, c), vscale(q, s));
  const qq = vadd(vscale(p, -s), vscale(q, c));
  return circularMoonEphemeris({ p: pp, q: qq, a, n: N_MOON, phase0: 0.0 });
}

export const moon_distance = (e, r, t) => vnorm(vsub(moon_position(e, t), r));
export const moon_altitude = (e, r, t) => moon_distance(e, r, t) - R_MOON;

export const mascon = (name, lat, lon, depth, dmu) => ({ name, lat, lon, depth, dmu });

export const default_mascons = () => [
  mascon('imbrium', deg2rad_(33.0), deg2rad_(-16.0), 140.0e3, 4.9e7),
  mascon('serenitatis', deg2rad_(28.0), deg2rad_(18.0), 140.0e3, 4.2e7),
  mascon('crisium', deg2rad_(17.0), deg2rad_(59.0), 140.0e3, 3.4e7),
  mascon('nectaris', deg2rad_(-15.0), deg2rad_(34.0), 140.0e3, 2.3e7),
  mascon('humorum', deg2rad_(-24.0), deg2rad_(-39.0), 140.0e3, 2.1e7),
];

export const lunarGravity = ({ j2 = 2.0323e-4, mascons = default_mascons() } = {}) =>
  ({ j2, mascons });

const _mascon_dir = m => [
  Math.cos(m.lat) * Math.cos(m.lon),
  Math.cos(m.lat) * Math.sin(m.lon),
  Math.sin(m.lat),
];

export function moonfixed_basis(eph, t) {
  const s = moon_position(eph, t);
  const xhat = vunit(vscale(s, -1.0));
  const zhat = vunit(vcross(s, moon_velocity(eph, t)));
  return [xhat, vcross(zhat, xhat), zhat];
}

export function moonfixed(r, t, eph) {
  const [x, y, z] = moonfixed_basis(eph, t);
  return [vdot(r, x), vdot(r, y), vdot(r, z)];
}

export function moonfixed_inv(rf, t, eph) {
  const [x, y, z] = moonfixed_basis(eph, t);
  return vadd(vadd(vscale(x, rf[0]), vscale(y, rf[1])), vscale(z, rf[2]));
}

export function lunar_gravity(r, field, t, eph) {
  const rn = vnorm(r);
  let a = vscale(r, -MU_MOON / (rn * rn * rn));
  if (field == null) return a;
  const [xh, yh, zh] = moonfixed_basis(eph, t);
  if (field.j2 !== 0.0) {
    const z = vdot(r, zh);
    const k = -1.5 * field.j2 * MU_MOON * R_MOON ** 2 / (rn ** 5);
    const zr2 = (z / rn) ** 2;
    a = vadd(a, vscale(vadd(vscale(r, 1.0 - 5.0 * zr2), vscale(zh, 2.0 * z)), k));
  }
  for (const m of field.mascons) {
    const d = _mascon_dir(m);
    const p = vscale(vadd(vadd(vscale(xh, d[0]), vscale(yh, d[1])), vscale(zh, d[2])),
                     R_MOON - m.depth);
    const s = vsub(r, p);
    const sn = vnorm(s);
    if (sn < 1.0) continue;
    a = vadd(a, vscale(s, -m.dmu / (sn * sn * sn)));
    a = vadd(a, vscale(r, m.dmu / (rn * rn * rn)));
  }
  return a;
}

export function gravity_anomaly(field, lat, lon, h, eph, t) {
  const d = [Math.cos(lat) * Math.cos(lon), Math.cos(lat) * Math.sin(lon), Math.sin(lat)];
  const r = vscale(moonfixed_inv(d, t, eph), R_MOON + h);
  const u = vunit(r);
  return -vdot(vsub(lunar_gravity(r, field, t, eph), lunar_gravity(r, null, t, eph)), u);
}
