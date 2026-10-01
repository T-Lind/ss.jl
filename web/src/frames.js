// Port of src/frames.jl: ECI <-> ECEF <-> geodetic WGS-84, ENU, and osculating
// classical elements.
import { MU_EARTH, RE_EQ, RE_POL, E2_WGS84, RE_MEAN, OMEGA_EARTH } from './constants.js';
import { vnorm, vcross, vsub, vscale, vdot } from './vec3.js';

export const earth_rotation_angle = (theta_g0, t) => theta_g0 + OMEGA_EARTH * t;

// -> [lat, lon, h]
export function geodetic_from_ecef(r) {
  const [x, y, z] = r;
  const lon = Math.atan2(y, x);
  const p = Math.hypot(x, y);
  if (p < 1e-9) {
    const lat = (z < 0 ? -1 : 1) * (Math.PI / 2);
    return [lat, lon, Math.abs(z) - RE_POL];
  }
  const ep2 = E2_WGS84 / (1 - E2_WGS84);
  const b = RE_POL;
  const theta = Math.atan2(z * RE_EQ, p * b);
  const st = Math.sin(theta), ct = Math.cos(theta);
  const lat = Math.atan2(z + ep2 * b * st * st * st,
                         p - E2_WGS84 * RE_EQ * ct * ct * ct);
  const sl = Math.sin(lat);
  const N = RE_EQ / Math.sqrt(1 - E2_WGS84 * sl * sl);
  const h = p / Math.cos(lat) - N;
  return [lat, lon, h];
}

export function ecef_from_geodetic(lat, lon, h) {
  const sl = Math.sin(lat), cl = Math.cos(lat);
  const N = RE_EQ / Math.sqrt(1 - E2_WGS84 * sl * sl);
  return [(N + h) * cl * Math.cos(lon),
          (N + h) * cl * Math.sin(lon),
          (N * (1 - E2_WGS84) + h) * sl];
}

// -> [e_east, e_north, e_up]
export function enu_basis(lat, lon) {
  const sl = Math.sin(lat), cl = Math.cos(lat);
  const so = Math.sin(lon), co = Math.cos(lon);
  return [
    [-so, co, 0.0],
    [-sl * co, -sl * so, cl],
    [cl * co, cl * so, sl],
  ];
}

export function haversine(lat1, lon1, lat2, lon2) {
  const dlat = lat2 - lat1;
  const dlon = lon2 - lon1;
  const a = Math.sin(dlat / 2) ** 2 +
            Math.cos(lat1) * Math.cos(lat2) * Math.sin(dlon / 2) ** 2;
  return 2 * RE_MEAN * Math.asin(Math.min(1.0, Math.sqrt(a)));
}

export function elements_from_state(r, v, mu = MU_EARTH) {
  const rn = vnorm(r), vn = vnorm(v);
  const h = vcross(r, v);
  const hn = vnorm(h);
  const energy = 0.5 * vn * vn - mu / rn;
  const a = -mu / (2 * energy);
  const ev = vsub(vscale(vcross(v, h), 1 / mu), vscale(r, 1 / rn));
  const e = vnorm(ev);
  const i = Math.acos(Math.min(1, Math.max(-1, h[2] / hn)));
  const nvec = vcross([0.0, 0.0, 1.0], h);
  const nn = vnorm(nvec);
  const inclined = nn > 1e-12 * hn;
  const wrap = angle => (angle + 2 * Math.PI) % (2 * Math.PI);
  const orientation = h[2] < 0 ? -1 : 1;
  const raan = inclined ? Math.atan2(nvec[1], nvec[0]) : 0.0;
  const argp = (inclined && e > 1e-12)
    ? (() => { const w = Math.acos(Math.min(1, Math.max(-1, vdot(nvec, ev) / (nn * e)))); return ev[2] < 0 ? 2 * Math.PI - w : w; })()
    : e > 1e-12 ? wrap(Math.atan2(orientation * ev[1], ev[0])) : 0.0;
  const nu = e > 1e-12
    ? (() => { const f = Math.acos(Math.min(1, Math.max(-1, vdot(ev, r) / (e * rn)))); return vdot(r, v) < 0 ? 2 * Math.PI - f : f; })()
    : inclined ? wrap(Math.atan2(vdot(vcross(nvec, r), h) / hn, vdot(nvec, r)))
      : wrap(Math.atan2(orientation * r[1], r[0]));
  const p = hn * hn / mu;
  const rp = p / (1 + e);
  const ra = e < 1 ? p / (1 - e) : Infinity;
  return { a, e, i, raan, argp, nu, rp, ra, energy };
}

// -> [r, v]
export function state_from_elements(a, e, i, raan, argp, nu, mu = MU_EARTH) {
  const p = a * (1 - e * e);
  const r = p / (1 + e * Math.cos(nu));
  const rp = [r * Math.cos(nu), r * Math.sin(nu), 0.0];
  const vf = Math.sqrt(mu / p);
  const vp = [-vf * Math.sin(nu), vf * (e + Math.cos(nu)), 0.0];
  const co = Math.cos(raan), so = Math.sin(raan);
  const cw = Math.cos(argp), sw = Math.sin(argp);
  const ci = Math.cos(i), si = Math.sin(i);
  const R11 = co * cw - so * sw * ci, R12 = -co * sw - so * cw * ci, R13 = so * si;
  const R21 = so * cw + co * sw * ci, R22 = -so * sw + co * cw * ci, R23 = -co * si;
  const R31 = sw * si, R32 = cw * si, R33 = ci;
  const rot = pv => [
    R11 * pv[0] + R12 * pv[1] + R13 * pv[2],
    R21 * pv[0] + R22 * pv[1] + R23 * pv[2],
    R31 * pv[0] + R32 * pv[1] + R33 * pv[2],
  ];
  return [rot(rp), rot(vp)];
}
