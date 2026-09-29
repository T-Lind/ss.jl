// Port of src/landingnav.jl: the descent navigation filter, the landing radar,
// and high-gate hazard redesignation.
import { R_MOON, deg2rad_ } from './constants.js';
import { vadd, vsub, vscale, vdot, vcross, vnorm, vunit } from './vec3.js';
import { lunar_gravity, moonfixed } from './moon.js';
import { surface_altitude, surface_radius, site_hazard, safe_site, surface_offset } from './terrain.js';
import { _hash32 } from './terrain.js';

export const landingRadar = (o = {}) => ({
  h_lock: o.h_lock ?? 12.0e3,
  h_lock_vel: o.h_lock_vel ?? 7.0e3,
  dt_update: o.dt_update ?? 0.25,
  sigma_h: o.sigma_h ?? 2.5,
  scale_h: o.scale_h ?? 0.012,
  sigma_v: o.sigma_v ?? 0.30,
  beam_tilt: o.beam_tilt ?? deg2rad_(22.0),
  gain_h: o.gain_h ?? 0.22,
  gain_v: o.gain_v ?? 0.16,
});

export const descentNav = (o = {}) => ({
  dr_down: o.dr_down ?? 900.0,
  dr_radial: o.dr_radial ?? 250.0,
  dr_cross: o.dr_cross ?? 300.0,
  dv_down: o.dv_down ?? 0.6,
  dv_radial: o.dv_radial ?? 0.3,
  dv_cross: o.dv_cross ?? 0.4,
  site_elev: o.site_elev ?? NaN,
  radar: o.radar === undefined ? landingRadar() : o.radar,
  seed: o.seed ?? 0x5EED1A11,
});

export const perfect_nav = () => descentNav({
  dr_down: 0.0, dr_radial: 0.0, dr_cross: 0.0,
  dv_down: 0.0, dv_radial: 0.0, dv_cross: 0.0, radar: null,
});

const u32 = x => x >>> 0;
export const _nrand = (seed, k) =>
  (_hash32(seed, u32(k), 0x9E3779B9, 0x85EBCA6B) + 0.5) / 4.294967296e9;

export function _ngauss(seed, k) {
  const u1 = _nrand(seed, 2 * k);
  const u2 = _nrand(seed, 2 * k + 1);
  return Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(6.283185307179586 * u2);
}

export const navState = (r, v, r_ref, locked_h, locked_v, t_next, n_update, seed) =>
  ({ r, v, r_ref, locked_h, locked_v, t_next, n_update, seed });

export function init_nav(cfg, r, v, hhat) {
  const ur = vunit(r);
  const ut = vcross(hhat, ur);
  const dr = vadd(vadd(vscale(ut, cfg.dr_down * _ngauss(cfg.seed, 1)),
                       vscale(ur, cfg.dr_radial * _ngauss(cfg.seed, 2))),
                  vscale(hhat, cfg.dr_cross * _ngauss(cfg.seed, 3)));
  const dv = vadd(vadd(vscale(ut, cfg.dv_down * _ngauss(cfg.seed, 4)),
                       vscale(ur, cfg.dv_radial * _ngauss(cfg.seed, 5))),
                  vscale(hhat, cfg.dv_cross * _ngauss(cfg.seed, 6)));
  const r_ref = Number.isNaN(cfg.site_elev) ? R_MOON : R_MOON + cfg.site_elev;
  return navState(vadd(r, dr), vadd(v, dv), r_ref, false, false, 0.0, 0,
                  (cfg.seed ^ 0x2C1B3C6D) >>> 0);
}

export const nav_altitude = n => vnorm(n.r) - n.r_ref;

export function nav_propagate(n, a_thrust, dt) {
  const a = vadd(lunar_gravity(n.r, null, 0.0, 0.0), a_thrust);
  n.r = vadd(n.r, vadd(vscale(n.v, dt), vscale(a, 0.5 * dt * dt)));
  n.v = vadd(n.v, vscale(a, dt));
}

export function radar_update(n, radar, r, v, t, surf, t_abs, hhat) {
  if (t < n.t_next) return false;
  n.t_next = t + radar.dt_update;
  const ur = vunit(r);
  const ut = vcross(hhat, ur);
  const h_true = surface_altitude(surf, r, t_abs);
  if (h_true <= 0.0) return false;
  let updated = false;

  if (h_true <= radar.h_lock) {
    n.locked_h = true;
    const lead = h_true * Math.tan(radar.beam_tilt);
    const rb = vadd(r, vscale(ut, lead));
    const h_beam = vnorm(r) - surface_radius(surf, rb, t_abs);
    const k = n.n_update;
    const noise = (radar.sigma_h + radar.scale_h * h_true) * _ngauss(n.seed, 1000 + k);
    const h_meas = h_beam + noise;
    n.r = vadd(n.r, vscale(vunit(n.r), radar.gain_h * (h_meas - nav_altitude(n))));
    updated = true;
  }
  if (h_true <= radar.h_lock_vel) {
    n.locked_v = true;
    const k = n.n_update;
    const dv = [radar.sigma_v * _ngauss(n.seed, 2000 + 3 * k),
                radar.sigma_v * _ngauss(n.seed, 2000 + 3 * k + 1),
                radar.sigma_v * _ngauss(n.seed, 2000 + 3 * k + 2)];
    const v_meas = vadd(v, dv);
    n.v = vadd(n.v, vscale(vsub(v_meas, n.v), radar.gain_v));
    updated = true;
  }
  n.n_update += 1;
  return updated;
}

export const nav_error = (n, r, v, surf, t_abs) =>
  [vnorm(vsub(n.r, r)), vnorm(vsub(n.v, v)),
   nav_altitude(n) - surface_altitude(surf, r, t_abs)];

export const hazardScan = (o = {}) => ({
  reach: o.reach ?? 900.0,
  cross_reach: o.cross_reach ?? 300.0,
  step: o.step ?? 100.0,
  radius: o.radius ?? 15.0,
  arrival: o.arrival ?? 30.0,
  tau: o.tau ?? 10.0,
});

export function redesignate(scan, surf, r, v, t_abs, hhat, lead) {
  const eph = surf.eph;
  const rf = moonfixed(r, t_abs, eph);
  const hf = moonfixed(hhat, t_abs, eph);
  const u0 = vunit(rf);
  const e_down = vunit(vcross(hf, u0));
  const e_cross = vcross(u0, e_down);
  const u_nom = surface_offset(u0, e_down, e_cross, lead, 0.0);
  const score0 = site_hazard(surf.terrain, u_nom, { radius: scan.radius }).score;
  const [u, d, c, sc] = safe_site(surf.terrain, u_nom, e_down, e_cross,
    { reach: scan.reach, cross_reach: scan.cross_reach, step: scan.step, radius: scan.radius });
  return [u, sc, score0, Math.hypot(d, c)];
}
