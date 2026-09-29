// Port of src/landing.jl: lunar arrival and powered descent — lunar-orbit
// insertion at perilune, descent-orbit initiation, and a guided powered
// descent to touchdown.
//
// Frames. Everything from perilune on is integrated in a Moon-centred frame
// that falls freely with the Moon, so the residual Earth term is the tidal
// difference and the Earth is dropped. How much world to fly through is the
// caller's choice, made with a `descentConfig`: left empty the Moon is a point
// mass and the surface a sphere; filled in, the descent flies procedural
// terrain, oblate mascon-lumped gravity, a navigation state corrected by
// landing radar, and high-gate hazard redesignation.
import { MU_MOON, R_MOON, A_MOON, N_MOON, G0, deg2rad_ } from './constants.js';
import { vadd, vsub, vscale, vdot, vcross, vnorm, vunit } from './vec3.js';
import { moon_position, moon_velocity, moon_distance, moonfixed_basis,
         moonfixed, moonfixed_inv, lunar_gravity, lunarGravity } from './moon.js';
import { terrain_height, terrain_slope, ground_elevation, surface_radius,
         surfaceModel, lunarTerrain } from './terrain.js';
import { init_nav, nav_altitude, nav_propagate, radar_update, nav_error,
         redesignate, descentNav, hazardScan, _ngauss } from './landingnav.js';
import { CIS_ETA, cislunarLog, cislunarResult, cis_push, _cis_dt, _cis_step,
         tli_burn } from './translunar.js';
import { default_moon_rocket } from './propulsion.js';
import { translunar_design } from './mission.js';

// ------------------------------------------------------------- vehicles ---

// Lander(name, mdry, mprop, thrust, isp, throttle_min, diameter): a single
// stage that puts itself on the surface.
export const lander = (o = {}) => ({
  name: o.name ?? 'lander',
  mdry: o.mdry ?? 3500.0,
  mprop: o.mprop ?? 9000.0,
  thrust: o.thrust ?? 45.0e3,
  isp: o.isp ?? 311.0,
  throttle_min: o.throttle_min ?? 0.10,
  diameter: o.diameter ?? 4.2,
});

// Wet mass of the lander [kg] — what the launcher has to throw.
export const lander_mass = l => l.mdry + l.mprop;

// Full-throttle mass flow [kg/s].
export const lander_mdot = l => l.thrust / (G0 * l.isp);

// Apollo-LM-class single-stage lander: 12.5 t wet, 9 t of hypergolic
// propellant, one 45 kN engine throttleable to 10%.
export const default_lander = ({ payload = 0.0 } = {}) =>
  lander({ mdry: 3500.0 + payload, mprop: 9000.0, thrust: 45.0e3,
           isp: 311.0, throttle_min: 0.10, diameter: 4.2 });

// Ideal vacuum delta-v the lander carries [m/s].
export const lander_dv = l => G0 * l.isp * Math.log(lander_mass(l) / l.mdry);

// The part of the lander that comes back.
export const ascentStage = (o = {}) => ({
  name: o.name ?? 'ascent',
  mdry: o.mdry ?? 1300.0,
  mprop: o.mprop ?? 1500.0,
  thrust: o.thrust ?? 15.6e3,
  isp: o.isp ?? 311.0,
});

export const ascent_mass = a => a.mdry + a.mprop;
export const ascent_dv = a => G0 * a.isp * Math.log(ascent_mass(a) / a.mdry);
export const _ascent_mdot = a => a.thrust / (G0 * a.isp);

// What waits in lunar orbit.
export const orbiter = (o = {}) => ({
  name: o.name ?? 'orbiter',
  mdry: o.mdry ?? 5200.0,
  mprop: o.mprop ?? 4800.0,
  thrust: o.thrust ?? 45.0e3,
  isp: o.isp ?? 314.0,
});

export const orbiter_mass = o => o.mdry + o.mprop;
export const orbiter_dv = o => G0 * o.isp * Math.log(orbiter_mass(o) / o.mdry);

// ------------------------------------------------------------------ logs ---

// Powered-descent log. `downrange` is surface arc from the point under the
// vehicle at ignition; `h` is height above the ground, not the mean sphere.
export const descentLog = () => ({
  t: [], h: [], downrange: [], v: [], vh: [], vv: [], m: [], throttle: [],
  pitch: [], x: [], y: [], z: [], elev: [], nav_dh: [], nav_dr: [], cross: [],
});

// Coast log in the lunar parking / descent orbit, Moon-centred inertial.
export const lunarOrbitLog = () => ({
  t: [], x: [], y: [], z: [], h: [], phase: [],
});

// Products of a powered descent: the log, the touchdown state, and the
// numbers that decide whether the vehicle survived it.
export const descentResult = (o = {}) => ({
  log: o.log ?? descentLog(),
  outcome: o.outcome ?? 'timeout',       // touchdown | timeout | crash | tipped | propellant | diverged
  t_touchdown: o.t_touchdown ?? 0.0,
  t_gate: o.t_gate ?? 0.0,
  v_vertical: o.v_vertical ?? 0.0,
  v_horizontal: o.v_horizontal ?? 0.0,
  downrange: o.downrange ?? 0.0,
  dv_braking: o.dv_braking ?? 0.0,
  dv_terminal: o.dv_terminal ?? 0.0,
  prop_used: o.prop_used ?? 0.0,
  prop_left: o.prop_left ?? 0.0,
  hover_s: o.hover_s ?? 0.0,
  min_throttle: o.min_throttle ?? 0.0,
  pitch0: o.pitch0 ?? 0.0,
  pitch_rate: o.pitch_rate ?? 0.0,
  r: o.r, v: o.v, m: o.m ?? 0.0,
  slope: o.slope ?? 0.0,
  elev: o.elev ?? 0.0,
  site_score: o.site_score ?? NaN,
  site_score_nominal: o.site_score_nominal ?? NaN,
  redesignated: o.redesignated ?? 0.0,
  nav_err: o.nav_err ?? 0.0,
  nav_dh: o.nav_dh ?? 0.0,
  radar_locked: o.radar_locked ?? false,
});

// The whole landing mission.
export const landingResult = (o = {}) => ({
  lv: o.lv, lander: o.lander, guid: o.guid, ascent: o.ascent, eph: o.eph,
  cislunar: o.cislunar, orbit: o.orbit, descent: o.descent,
  dv_loi: o.dv_loi ?? 0.0, dv_doi: o.dv_doi ?? 0.0,
  t_loi: o.t_loi ?? 0.0, t_doi: o.t_doi ?? 0.0,
  t_pdi: o.t_pdi ?? 0.0, t_touchdown: o.t_touchdown ?? 0.0,
  h_park_moon: o.h_park_moon ?? 0.0, h_pdi: o.h_pdi ?? 0.0,
  n_rev: o.n_rev ?? 0,
  lat_land: o.lat_land ?? 0.0, lon_land: o.lon_land ?? 0.0,
  prop_margin: o.prop_margin ?? 0.0,
  orbiter: o.orbiter ?? null, m_orbiter: o.m_orbiter ?? 0.0,
  r_orbiter: o.r_orbiter, v_orbiter: o.v_orbiter,
});

// ------------------------------------------------------- Moon-frame basics ---

// Lunar gravity in the Moon-centred frame: the point mass the whole
// trans-lunar chain uses, or the oblate and lumpy field.
export const _moon_accel = (r, t = 0.0, field = null, eph = null) =>
  field == null || eph == null ? vscale(r, -MU_MOON / vnorm(r) ** 3)
                               : lunar_gravity(r, field, t, eph);

// RK4 step of a ballistic Moon-centred coast.
export function _moon_step(r, v, dt, { t = 0.0, field = null, eph = null } = {}) {
  const acc = (rr, tt) => _moon_accel(rr, tt, field, eph);
  const k1v = acc(r, t),                             k1r = v;
  const r2 = vadd(r, vscale(k1r, dt / 2)), v2 = vadd(v, vscale(k1v, dt / 2));
  const k2v = acc(r2, t + dt / 2),                   k2r = v2;
  const r3 = vadd(r, vscale(k2r, dt / 2)), v3 = vadd(v, vscale(k2v, dt / 2));
  const k3v = acc(r3, t + dt / 2),                   k3r = v3;
  const r4 = vadd(r, vscale(k3r, dt)),   v4 = vadd(v, vscale(k3v, dt));
  const k4v = acc(r4, t + dt),                       k4r = v4;
  return [vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), dt / 6)),
          vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), dt / 6))];
}

// Moon-centred state [m, m/s] of an ECI state at time `t`.
export const mci_state = (r, v, t, eph) =>
  [vsub(r, moon_position(eph, t)), vsub(v, moon_velocity(eph, t))];

// Latitude and longitude of a Moon-centred position, in the tidally-locked
// Moon-fixed frame: longitude 0 is the sub-Earth meridian.
export function selenographic(r_m, t, eph) {
  const [xhat, yhat, zhat] = moonfixed_basis(eph, t);
  const u = vunit(r_m);
  const lat = Math.asin(clamp(vdot(u, zhat), -1.0, 1.0));
  const lon = Math.atan2(vdot(u, yhat), vdot(u, xhat));
  return [lat, lon];
}

// ------------------------------------------------ coast to perilune (ECI) ---

// The outbound half of the trans-lunar flight: parking coast, finite TLI
// burn, then coast until closest approach to the Moon, located by bisection
// on the range rate. Same dynamics and log as `fly_cislunar`; stops where the
// landing mission stops caring about the free return.
export function fly_to_perilune(r0, v0, t0, eph, opts) {
  const { t_ign, dv, stage, m_stack, prop_avail, theta_g0 = 0.0,
          eta = CIS_ETA, t_max = 12.0 * 86400.0, log_every = 4 } = opts;
  const L = cislunarLog();
  let r = r0, v = v0, t = t0;
  let kount = 0;
  while (t < t_ign) {
    const dtp = Math.min(_cis_dt(r, t, eph, { eta, dt_max: 30.0 }), t_ign - t);
    if (kount % log_every === 0) cis_push(L, t, r, v, eph, theta_g0, 0);
    [r, v] = _cis_step(r, v, t, dtp, eph);
    t += dtp;
    kount += 1;
  }

  const burn = tli_burn(r, v, t, m_stack, stage, dv, eph, prop_avail);
  r = burn.r; v = burn.v; t = burn.t;
  const m = burn.m, dv_del = burn.dv_delivered, tburn = burn.duration;
  for (let i = 0; i < burn.ts.length; i++)
    cis_push(L, burn.ts[i], burn.rs[i], burn.vs[i], eph, theta_g0, 1);

  // range rate to the Moon; perilune is where it changes sign
  const rate = (rr, vv, tt) => vdot(vsub(rr, moon_position(eph, tt)),
                                    vsub(vv, moon_velocity(eph, tt)));
  let outcome = 'timeout';
  let peri_alt = Infinity, t_peri = NaN;
  const t_end = t0 + t_max;
  kount = 0;
  while (t < t_end) {
    const dtc = _cis_dt(r, t, eph, { eta });
    if (kount % log_every === 0) cis_push(L, t, r, v, eph, theta_g0, 2);
    kount += 1;
    const [rn_, vn_] = _cis_step(r, v, t, dtc, eph);
    const tn = t + dtc;
    if (moon_distance(eph, rn_, tn) - R_MOON <= 0.0) {
      r = rn_; v = vn_; t = tn;
      outcome = 'lunar_impact';
      break;
    }
    if (rate(r, v, t) < 0.0 && rate(rn_, vn_, tn) >= 0.0) {
      // bisect the step onto closest approach
      let lo = 0.0, hi = dtc;
      for (let _ = 0; _ < 60; _++) {
        const mid = 0.5 * (lo + hi);
        const [rm, vm] = _cis_step(r, v, t, mid, eph);
        if (rate(rm, vm, t + mid) < 0.0) lo = mid; else hi = mid;
        if (hi - lo < 1e-6) break;
      }
      [r, v] = _cis_step(r, v, t, 0.5 * (lo + hi), eph);
      t += 0.5 * (lo + hi);
      peri_alt = moon_distance(eph, r, t) - R_MOON;
      t_peri = t;
      outcome = 'perilune';
      cis_push(L, t, r, v, eph, theta_g0, 2);
      break;
    }
    r = rn_; v = vn_; t = tn;
    if (vnorm(r) > 2.0 * A_MOON) {
      outcome = 'escape';
      break;
    }
  }
  return cislunarResult(L, outcome, r, v, t, m, dv_del, t_ign, tburn,
                        peri_alt, t_peri, NaN, NaN, 0, NaN);
}

// ------------------------------------------------------- impulsive burns ---

const clamp = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

// Mass after an impulsive burn of `dv` [m/s] on a lander.
export const _burn_mass = (l, m, dv) => m * Math.exp(-dv / (G0 * l.isp));

// Lunar-orbit insertion at perilune. At closest approach the relative
// velocity is perpendicular to the relative position, so the cheapest
// circularisation is purely retrograde and its magnitude is the speed excess
// over circular.
export function loi_burn(r_m, v_m) {
  const rn = vnorm(r_m);
  const v_circ = Math.sqrt(MU_MOON / rn);
  const vhat = vunit(v_m);
  const dv = vnorm(v_m) - v_circ;
  return [dv, vscale(vhat, v_circ)];
}

// Descent-orbit initiation: a retrograde burn from the circular parking orbit
// onto an ellipse whose periapsis is `h_pdi` above the surface, half a
// revolution downrange.
export function doi_burn(r_m, v_m, h_pdi) {
  const ra = vnorm(r_m);
  const rp = R_MOON + h_pdi;
  if (!(rp < ra)) throw new Error('descent periapsis must be below the parking orbit');
  const a = 0.5 * (ra + rp);
  const v_apo = Math.sqrt(MU_MOON * (2 / ra - 1 / a));
  const dv = vnorm(v_m) - v_apo;
  return [dv, vscale(vunit(v_m), v_apo)];
}

// Ballistic Moon-centred coast of `dt_total` seconds, logging as it goes.
export function coast_moon(L, r, v, t, dt_total, opts = {}) {
  const { phase = 0, dt = 5.0, log_every = 4, field = null, eph = null } = opts;
  const n = Math.max(1, Math.ceil(dt_total / dt));
  const step = dt_total / n;
  for (let k = 1; k <= n; k++) {
    if ((k - 1) % log_every === 0) {
      L.t.push(t); L.x.push(r[0]); L.y.push(r[1]); L.z.push(r[2]);
      L.h.push(vnorm(r) - R_MOON); L.phase.push(phase);
    }
    [r, v] = _moon_step(r, v, step, { t, field, eph });
    t += step;
  }
  L.t.push(t); L.x.push(r[0]); L.y.push(r[1]); L.z.push(r[2]);
  L.h.push(vnorm(r) - R_MOON); L.phase.push(phase);
  return [r, v];
}

// ------------------------------------------------------- powered descent ---

// Everything the descent knows about the world it is descending into. Every
// field is optional, and with all of them left out the descent is flown as it
// was before any of this existed: a point-mass Moon, a spherical surface, and
// guidance that reads the integrator's own state vector.
export const descentConfig = (o = {}) => ({
  surface: o.surface ?? null,
  field: o.field ?? null,
  nav: o.nav ?? null,
  hazard: o.hazard ?? null,
  eph: o.eph ?? null,
  t0: o.t0 ?? 0.0,
});

// The world as the *designer* sees it: sphere, point mass, perfect knowledge.
// The braking pitch program is shot against this and then flown against the
// real one.
export const nominal = cfg => descentConfig({ eph: cfg.eph, t0: cfg.t0 });

// Ground radius under a Moon-centred position, at descent time `t`.
const _ground = (cfg, r, t) => surface_radius(cfg.surface, r, cfg.t0 + t);

// Height above the ground directly below [m].
const _alt = (cfg, r, t) => vnorm(r) - _ground(cfg, r, t);

// Velocity of the ground itself at a Moon-centred position, in the inertial
// frame. The Moon turns once a month, 4.6 m/s at the equator — small against
// a 1.7 km/s orbit and enormous against a lander's lateral touchdown limit.
const _surface_vel = (cfg, r, t) => {
  if (cfg.eph === null) return [0.0, 0.0, 0.0];
  const [, , zh] = moonfixed_basis(cfg.eph, cfg.t0 + t);
  return vcross(vscale(zh, N_MOON), r);
};

// Gravity at a Moon-centred position, at descent time `t`.
const _grav = (cfg, r, t) =>
  cfg.field === null || cfg.eph === null ? _moon_accel(r)
                                         : lunar_gravity(r, cfg.field, cfg.t0 + t, cfg.eph);

// In-plane frame at a Moon-centred position: radial-out and along-track, the
// latter fixed by the orbit normal `hhat` captured at ignition rather than by
// the instantaneous velocity.
const _descent_frame = (r, hhat) => {
  const ur = vunit(r);
  return [ur, vcross(hhat, ur)];
};

// Orbit normal of a Moon-centred state — the descent plane, fixed at ignition.
const _descent_normal = (r, v) => vunit(vcross(r, v));

// Braking phase: full thrust, thrust elevation above the local horizontal
// following `theta(t) = pitch0 + pitch_rate * t`, integrated until the
// along-track speed falls through `vh_gate` — high gate. It also stops early
// if the tank runs dry, the vehicle reaches the surface, it climbs away, or
// the clock runs out.
export function _descent_leg(l, r0, v0, m0, pitch0, pitch_rate, opts = {}) {
  const { vh_gate = 150.0, dt = 0.5, t_max = 1200.0, log = null,
          r_ref = r0, log_every = 4, cfg = descentConfig(), nav = null } = opts;
  let r = r0, v = v0, m = m0, t = 0.0;
  const mdot = lander_mdot(l);
  const m_dry = m0 - l.mprop;
  const h0 = _alt(cfg, r0, 0.0);
  const hhat = _descent_normal(r0, v0);
  // :gate is a *result*, set only by the two gate-crossing breaks below; the
  // initial value is the failure the loop can fall out of.
  let outcome = 'timeout';
  let kount = 0;
  const radar = nav === null || cfg.nav === null ? null : cfg.nav.radar;

  const thrust_dir = (rr, tt) => {
    const [ur, ut] = _descent_frame(rr, hhat);
    const th = clamp(pitch0 + pitch_rate * tt, -deg2rad_(60.0), deg2rad_(89.0));
    return [vadd(vscale(ur, Math.sin(th)), vscale(ut, -Math.cos(th))), th];
  };

  const vhof = (rr, vv, tt) => {
    const ur = vunit(rr);
    return vdot(vsub(vv, _surface_vel(cfg, rr, tt)), vcross(hhat, ur));
  };
  // the number the *vehicle* has: its own estimate when it is navigating
  const guide_vh = () => (nav === null ? vhof(r, v, t) : vhof(nav.r, nav.v, t));

  while (t < t_max) {
    const h = _alt(cfg, r, t);
    if (log !== null && kount % log_every === 0) {
      const [, th] = thrust_dir(r, t);
      _log_descent(log, t, r, v, m, 1.0, th, r_ref, hhat, cfg, nav);
    }
    kount += 1;
    if (guide_vh() <= vh_gate) { outcome = 'gate'; break; }
    if (h <= 0.0) { outcome = 'surface'; break; }
    if (m <= m_dry + 1e-9) { outcome = 'propellant'; break; }
    if (h > h0 + 20.0e3) { outcome = 'climbing'; break; }
    const step = Math.min(dt, (m - m_dry) / mdot);
    // RK4 on (r, v) with mass drawn linearly across the step
    const acc = (rr, mm, tt) => {
      const [d] = thrust_dir(rr, tt);
      return vadd(_grav(cfg, rr, tt), vscale(d, l.thrust / mm));
    };
    const k1r = v,                            k1v = acc(r, m, t);
    const r2 = vadd(r, vscale(k1r, step / 2)), v2 = vadd(v, vscale(k1v, step / 2)), m2 = m - mdot * step / 2;
    const k2r = v2,                           k2v = acc(r2, m2, t + step / 2);
    const r3 = vadd(r, vscale(k2r, step / 2)), v3 = vadd(v, vscale(k2v, step / 2));
    const k3r = v3,                           k3v = acc(r3, m2, t + step / 2);
    const r4 = vadd(r, vscale(k3r, step)),   v4 = vadd(v, vscale(k3v, step)), m4 = m - mdot * step;
    const k4r = v4,                           k4v = acc(r4, m4, t + step);
    let rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step / 6));
    let vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step / 6));
    // land exactly on the gate rather than stepping past it; only worth doing
    // when the gate is called on truth
    if (nav === null && vhof(rn, vn, t + step) < vh_gate) {
      let lo = 0.0, hi = 1.0;
      for (let _ = 0; _ < 40; _++) {
        const f = 0.5 * (lo + hi);
        const rm = vadd(r, vscale(vsub(rn, r), f));
        const vm = vadd(v, vscale(vsub(vn, v), f));
        if (vhof(rm, vm, t + step * f) > vh_gate) lo = f; else hi = f;
      }
      const f = 0.5 * (lo + hi);
      r = vadd(r, vscale(vsub(rn, r), f));
      v = vadd(v, vscale(vsub(vn, v), f));
      m -= mdot * step * f;
      t += step * f;
      outcome = 'gate';
      break;
    }
    if (nav !== null) {
      const [d] = thrust_dir(r, t);
      nav_propagate(nav, vscale(d, l.thrust / m), step);
      if (radar !== null)
        radar_update(nav, radar, rn, vn, t + step, cfg.surface, cfg.t0 + t + step, hhat);
    }
    r = rn; v = vn; m = m - mdot * step; t = t + step;
  }
  const [ur, ut] = _descent_frame(r, hhat);
  const vr = vsub(v, _surface_vel(cfg, r, t));
  return { r, v, m, t, outcome, h: _alt(cfg, r, t), vv: vdot(vr, ur), vh: vdot(vr, ut) };
}

// Push one sample onto a descent log.
export function _log_descent(L, t, r, v, m, throttle, pitch, r_ref, hhat,
                             cfg = descentConfig(), nav = null, target = null) {
  const [ur, ut] = _descent_frame(r, hhat);
  // velocities are logged relative to the ground, which is what a landing is
  // measured against
  const vr = vsub(v, _surface_vel(cfg, r, t));
  L.t.push(t); L.h.push(_alt(cfg, r, t));
  L.downrange.push(R_MOON * Math.acos(clamp(vdot(vunit(r), vunit(r_ref)), -1.0, 1.0)));
  L.v.push(vnorm(vr)); L.vh.push(vdot(vr, ut)); L.vv.push(vdot(vr, ur));
  L.m.push(m); L.throttle.push(throttle); L.pitch.push(pitch);
  L.x.push(r[0]); L.y.push(r[1]); L.z.push(r[2]);
  L.elev.push(ground_elevation(cfg.surface, r, cfg.t0 + t));
  if (nav === null) {
    L.nav_dh.push(0.0); L.nav_dr.push(0.0);
  } else {
    const [dr, , dh] = nav_error(nav, r, v, cfg.surface, cfg.t0 + t);
    L.nav_dh.push(dh); L.nav_dr.push(dr);
  }
  L.cross.push(target === null || cfg.eph === null ? 0.0 :
    vdot(vscale(vsub(moonfixed_inv(target, cfg.t0 + t, cfg.eph), vunit(r)), R_MOON), hhat));
}

// Shoot the braking phase. Two parameters (initial pitch, pitch rate) against
// two targets — the altitude and sink rate at high gate — by damped Newton
// with finite differences. A coarse grid seeds the Newton; failed legs return
// a signed penalty that pushes the search back toward the feasible region.
export function tune_braking(l, r0, v0, m0, opts = {}) {
  const { h_gate = 2300.0, vv_gate = -45.0, vh_gate = 150.0, max_iter = 25,
          cfg = descentConfig(), verbose = false } = opts;
  const ncfg = nominal(cfg);
  const resid = (p0, pr) => {
    const leg = _descent_leg(l, r0, v0, m0, p0, pr, { vh_gate, cfg: ncfg });
    if (leg.outcome === 'gate') {
      return [(leg.h - h_gate) / 1000.0, (leg.vv - vv_gate) / 100.0, leg];
    } else if (leg.outcome === 'climbing') {
      // too much lift: the whole program has to come down
      return [leg.h / 1000.0, 10.0, leg];
    } else if (leg.outcome === 'surface') {
      // arrived at the ground still flying: a continuous extension of the
      // residual rather than a cliff
      return [-h_gate / 1000.0 - leg.vh / 500.0, (leg.vv - vv_gate) / 100.0, leg];
    }
    // dry, or out of clock, still above the gate and still fast
    return [(leg.h - h_gate) / 1000.0,
            (leg.vv - vv_gate) / 100.0 - leg.vh / 500.0, leg];
  };
  const score = (p0, pr) => { const f = resid(p0, pr); return Math.hypot(f[0], f[1]); };

  // coarse grid, then Newton, then — if it did not converge — a local grid
  // around the best point seen and another Newton
  let best = [Infinity, 0.0, 0.0];
  for (let p0i = -6.0; p0i <= 24.0 + 1e-9; p0i += 3.0) {
    const p0 = deg2rad_(p0i);
    for (let pri = 0; pri <= 10; pri++) {
      const pr = pri * 1.5e-4;
      const sc = score(p0, pr);
      if (sc < best[0]) best = [sc, p0, pr];
    }
  }

  const newton = (p0, pr) => {
    let bp = [Infinity, p0, pr];
    for (let it = 1; it <= max_iter; it++) {
      const [f1, f2, leg] = resid(p0, pr);
      const sc = Math.hypot(f1, f2);
      if (sc < bp[0]) bp = [sc, p0, pr];
      if (Math.abs(f1) < 0.1 && Math.abs(f2) < 0.1 && leg.outcome === 'gate')
        return [true, p0, pr];
      const d1 = deg2rad_(0.4), d2 = 4.0e-5;
      const [f1a, f2a] = resid(p0 + d1, pr);
      const [f1b, f2b] = resid(p0, pr + d2);
      const j11 = (f1a - f1) / d1, j21 = (f2a - f2) / d1;
      const j12 = (f1b - f1) / d2, j22 = (f2b - f2) / d2;
      const det = j11 * j22 - j12 * j21;
      if (Math.abs(det) < 1e-14) break;
      const dp0 = -(j22 * f1 - j12 * f2) / det;
      const dpr = -(-j21 * f1 + j11 * f2) / det;
      p0 += clamp(0.7 * dp0, -deg2rad_(4.0), deg2rad_(4.0));
      pr += clamp(0.7 * dpr, -2.0e-4, 2.0e-4);
      p0 = clamp(p0, -deg2rad_(30.0), deg2rad_(60.0));
      pr = clamp(pr, -1.0e-3, 4.0e-3);
    }
    return [false, bp[1], bp[2]];
  };

  let [ok, p0, pr] = newton(best[1], best[2]);
  if (!ok) {
    // refine locally around the best point and try once more
    let bl = [Infinity, p0, pr];
    for (let dpi = -4; dpi <= 4; dpi++) {
      const dp = deg2rad_(dpi * 0.75);
      for (let dri = -4; dri <= 4; dri++) {
        const dr = dri * 7.5e-5;
        const sc = score(p0 + dp, pr + dr);
        if (sc < bl[0]) bl = [sc, p0 + dp, pr + dr];
      }
    }
    [ok, p0, pr] = newton(bl[1], bl[2]);
  }
  if (ok) return [p0, pr, true];

  const [f1, f2, leg] = resid(p0, pr);
  // a loose finish is still a flyable descent
  return [p0, pr, Math.abs(f1) < 1.0 && Math.abs(f2) < 0.5 && leg.outcome === 'gate'];
}

// Closed-loop descent from high gate to the surface. The guidance holds a
// commanded sink rate that tapers with altitude and flies the horizontal
// channels to null, or to a designated landing point if it has one.
export function terminal_descent(l, r0, v0, m0, opts = {}) {
  const { m_dry, v_touch = 0.8, k_profile = 0.85, v_cap = 25.0,
          tau_h = 18.0, tau_v = 5.0, dt = 0.1, t_max = 900.0, log = null,
          t0 = 0.0, r_ref = r0, log_every = 10, cfg = descentConfig(), nav = null,
          target = null, arrival = 30.0, vh_cap = 180.0 } = opts;
  let r = r0, v = v0, m = m0, t = 0.0;
  const mdot_full = lander_mdot(l);
  const hhat = _descent_normal(r0, v0);
  let min_thr = 1.0;
  // As in `_descent_leg`: :touchdown is set only by the ground-contact breaks.
  let outcome = 'timeout';
  let kount = 0;
  const radar = nav === null || cfg.nav === null ? null : cfg.nav.radar;

  // position error to the designated site, in the along-track / crossrange
  // pair, from whatever position the vehicle believes it holds
  const offsets = (rr, ta) => {
    if (target === null) return [0.0, 0.0];
    const ui = moonfixed_inv(target, cfg.t0 + ta, cfg.eph);
    const w = vscale(vsub(ui, vunit(rr)), R_MOON);
    const [, ut] = _descent_frame(rr, hhat);
    return [vdot(w, ut), vdot(w, hhat)];
  };

  const command = (rr, vv, mm, hh, ta) => {
    const [ur, ut] = _descent_frame(rr, hhat);
    // horizontal channels fly relative to the ground
    const vr = vsub(vv, _surface_vel(cfg, rr, ta));
    const vv_now = vdot(vv, ur); const vh_now = vdot(vr, ut); const vc_now = vdot(vr, hhat);
    const [d_rem, c_rem] = offsets(rr, ta);
    // Do not descend faster than the approach can converge.
    const off = Math.hypot(d_rem, c_rem);
    const t_go = off > 25.0 ? arrival * Math.log(off / 25.0) : 0.0;
    const v_allow = t_go > 0.1 ? hh / t_go : Infinity;
    const v_cmd = -Math.min(v_cap, v_touch + k_profile * Math.sqrt(Math.max(hh, 0.0)), v_allow);
    // Feed-forward on the profile itself.
    const dv_dh = v_cmd <= -v_cap ? 0.0 : -0.5 * k_profile / Math.sqrt(Math.max(hh, 1.0));
    const a_r = (v_cmd - vv_now) / tau_v + dv_dh * vv_now;
    const vh_des = clamp(d_rem / arrival, -vh_cap, vh_cap);
    const vc_des = clamp(c_rem / arrival, -0.2 * vh_cap, 0.2 * vh_cap);
    let a_t = (vh_des - vh_now) / tau_h;
    let a_c = (vc_des - vc_now) / tau_h;
    // cancel gravity and the centrifugal relief of whatever speed remains
    const g_eff = MU_MOON / vnorm(rr) ** 2 - vh_now ** 2 / vnorm(rr);
    const ar_tot = a_r + g_eff;
    // Thrust is finite, and the vertical demand is served first.
    const a_max = l.thrust / mm;
    if (Math.abs(ar_tot) > a_max) {
      a_t = 0.0; a_c = 0.0;
    } else {
      const lim = Math.sqrt(Math.max(a_max ** 2 - ar_tot ** 2, 0.0));
      const ah = Math.hypot(a_t, a_c);
      if (ah > lim && ah > 0.0) { a_t *= lim / ah; a_c *= lim / ah; }
    }
    const a_des = vadd(vadd(vscale(ur, ar_tot), vscale(ut, a_t)), vscale(hhat, a_c));
    const an = vnorm(a_des);
    const thr = clamp(mm * an / l.thrust, l.throttle_min, 1.0);
    const dir = an > 1e-9 ? vscale(a_des, 1 / an) : ur;
    return [dir, thr];
  };

  // what the vehicle flies on: its own estimate, or the truth
  const guide = ta => nav === null ? [r, v, _alt(cfg, r, ta)]
                                   : [nav.r, nav.v, nav_altitude(nav)];

  while (t < t_max) {
    const ta = t0 + t;
    const h = _alt(cfg, r, ta);
    const [rg, vg, hg] = guide(ta);
    const [dir, thr] = command(rg, vg, m, hg, ta);
    min_thr = Math.min(min_thr, thr);
    if (log !== null && kount % log_every === 0) {
      const [ur] = _descent_frame(r, hhat);
      _log_descent(log, ta, r, v, m, thr,
                   Math.asin(clamp(vdot(dir, ur), -1.0, 1.0)), r_ref, hhat,
                   cfg, nav, target);
    }
    kount += 1;
    if (h <= 0.0) { outcome = 'touchdown'; break; }
    if (m <= m_dry + 1e-9) { outcome = 'propellant'; break; }
    const step = Math.min(dt, (m - m_dry) / (thr * mdot_full));
    // zero-order hold on the command across the control cycle
    const a_th = vscale(dir, thr * l.thrust);
    const acc = (rr, mm, tt) => vadd(_grav(cfg, rr, tt), vscale(a_th, 1 / mm));
    const k1r = v,                            k1v = acc(r, m, ta);
    const r2 = vadd(r, vscale(k1r, step / 2)), v2 = vadd(v, vscale(k1v, step / 2));
    const k2r = v2,                           k2v = acc(r2, m - thr * mdot_full * step / 2, ta + step / 2);
    const r3 = vadd(r, vscale(k2r, step / 2)), v3 = vadd(v, vscale(k2v, step / 2));
    const k3r = v3,                           k3v = acc(r3, m - thr * mdot_full * step / 2, ta + step / 2);
    const r4 = vadd(r, vscale(k3r, step)),   v4 = vadd(v, vscale(k3v, step));
    const k4r = v4,                           k4v = acc(r4, m - thr * mdot_full * step, ta + step);
    let rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step / 6));
    let vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step / 6));
    const hn = _alt(cfg, rn, ta + step);
    if (hn <= 0.0 && h > 0.0) {
      const f = h / Math.max(h - hn, 1e-9);   // linear touchdown interpolation
      rn = vadd(r, vscale(vsub(rn, r), f));
      vn = vadd(v, vscale(vsub(vn, v), f));
      m -= thr * mdot_full * step * f;
      t += step * f;
      r = rn; v = vn;
      outcome = 'touchdown';
      break;
    }
    if (nav !== null) {
      nav_propagate(nav, vscale(a_th, 1 / m), step);
      if (radar !== null)
        radar_update(nav, radar, rn, vn, ta + step, cfg.surface, cfg.t0 + ta + step, hhat);
    }
    r = rn; v = vn; m = m - thr * mdot_full * step; t = t + step;
  }
  const [ur, ut] = _descent_frame(r, hhat);
  const vrel = vsub(v, _surface_vel(cfg, r, t0 + t));
  return { r, v, m, t, outcome, min_throttle: min_thr,
           v_vertical: -vdot(v, ur),
           v_horizontal: Math.hypot(vdot(vrel, ut), vdot(vrel, hhat)) };
}

// Braking phase (shot open-loop against the nominal world, flown against the
// real one) followed by the closed-loop terminal phase, logged as one
// continuous descent. If `cfg` carries a hazard scan, the landing point is
// chosen at high gate, between the two.
export function powered_descent(l, r0, v0, m0, opts = {}) {
  const { h_gate = 2300.0, vh_gate = 150.0, vv_gate = -45.0, h_ref = 0.0,
          cfg = descentConfig(), verbose = false } = opts;
  const m_dry = m0 - l.mprop;
  // The pitch program is shot on the sphere, so a gate over high ground has
  // to be aimed higher over the sphere; `h_ref` is that landing-site radius.
  const [p0, pr] = tune_braking(l, r0, v0, m0, { h_gate: h_gate + h_ref,
                                                vh_gate, vv_gate, cfg, verbose });
  const hhat = _descent_normal(r0, v0);
  const nav = cfg.nav === null ? null : init_nav(cfg.nav, r0, v0, hhat);
  const L = descentLog();
  const leg = _descent_leg(l, r0, v0, m0, p0, pr, { vh_gate, log: L,
                                                    r_ref: r0, cfg, nav });
  const dv_brake = G0 * l.isp * Math.log(m0 / leg.m);
  const [nav_dr0, , nav_dh0] =
    nav === null ? [0.0, 0.0, 0.0]
                 : nav_error(nav, leg.r, leg.v, cfg.surface, cfg.t0 + leg.t);
  // A braking phase that reached high gate is worth flying out even if the
  // shooter finished loose. Only a leg that never reached the gate is
  // unflyable.
  if (leg.outcome !== 'gate') {
    return descentResult({
      log: L, outcome: leg.outcome,
      t_touchdown: leg.t, t_gate: leg.t,
      v_vertical: -leg.vv, v_horizontal: leg.vh,
      downrange: L.downrange.length === 0 ? 0.0 : L.downrange[L.downrange.length - 1],
      dv_braking: dv_brake, dv_terminal: 0.0,
      prop_used: m0 - leg.m, prop_left: leg.m - m_dry,
      hover_s: 0.0, min_throttle: 1.0,
      pitch0: p0, pitch_rate: pr, r: leg.r, v: leg.v, m: leg.m,
      slope: 0.0, elev: ground_elevation(cfg.surface, leg.r, cfg.t0 + leg.t),
      site_score: NaN, site_score_nominal: NaN, redesignated: 0.0,
      nav_err: nav_dr0, nav_dh: nav_dh0,
      radar_locked: nav !== null && nav.locked_h,
    });
  }

  // --- landing-point designation ----------------------------------------
  const [, ut_gate] = _descent_frame(leg.r, hhat);
  let target = null;
  let score = NaN, score0 = NaN, moved = 0.0;
  if (cfg.hazard !== null && cfg.surface !== null && cfg.eph !== null) {
    // where the vehicle would arrive if it simply flew its forward speed out
    // on the approach time constant: the aim point it is redesignating away
    // from
    const rg = nav === null ? leg.r : nav.r;
    const vg = nav === null ? leg.v : nav.v;
    const [, ut] = _descent_frame(rg, hhat);
    const lead = vdot(vg, ut) * cfg.hazard.arrival;
    // The scan looks at the ground from where the vehicle actually is, but
    // the site it picks then has to be flown to using the same filter, so the
    // answer is expressed as an offset from the *estimated* position.
    const [tgt, sc, sc0, mv] =
      redesignate(cfg.hazard, cfg.surface, leg.r, leg.v, cfg.t0 + leg.t, hhat, lead);
    target = tgt; score = sc; score0 = sc0; moved = mv;
    if (nav !== null) {
      const eph = cfg.surface.eph;
      const u_true = vunit(moonfixed(leg.r, cfg.t0 + leg.t, eph));
      const u_nav = vunit(moonfixed(nav.r, cfg.t0 + leg.t, eph));
      target = vunit(vadd(target, vsub(u_nav, u_true)));
    }
  }

  const n_before = L.t.length;
  const term = terminal_descent(l, leg.r, leg.v, leg.m, {
    m_dry, log: L, t0: leg.t, r_ref: r0, cfg, nav, target,
    arrival: cfg.hazard === null ? 30.0 : cfg.hazard.arrival,
    // chasing a designated point is a position loop, and it needs a tighter
    // inner constant than the pure drift-nulling one does
    tau_h: target === null || cfg.hazard === null ? 18.0 : cfg.hazard.tau,
    vh_cap: Math.max(60.0, 1.2 * Math.abs(vdot(leg.v, ut_gate))),
  });
  const dv_term = G0 * l.isp * Math.log(leg.m / term.m);
  // the touchdown sample carries the attitude it landed in, not a zero
  _log_descent(L, leg.t + term.t, term.r, term.v, term.m, term.min_throttle,
               L.pitch.length > n_before ? L.pitch[L.pitch.length - 1] : deg2rad_(90.0),
               r0, hhat, cfg, nav, target);
  const prop_left = term.m - m_dry;
  // what the residual is actually worth: seconds of hover at touchdown mass
  const hover = prop_left / (term.m * MU_MOON / vnorm(term.r) ** 2 / (G0 * l.isp));

  // the ground it actually arrived on
  const t_td = cfg.t0 + leg.t + term.t;
  let slope = 0.0;
  if (cfg.surface !== null) {
    const u_td = vunit(moonfixed(term.r, t_td, cfg.surface.eph));
    slope = terrain_slope(cfg.surface.terrain, u_td);
  }
  const elev = ground_elevation(cfg.surface, term.r, t_td);
  const [nav_dr, , nav_dh] = nav === null ? [0.0, 0.0, 0.0]
    : nav_error(nav, term.r, term.v, cfg.surface, t_td);

  // Touchdown limits are the lander's, not the trajectory's: 3 m/s of sink
  // and about 1.2 m/s of lateral drift before a leg digs in and the vehicle
  // tips, and ground no steeper than about 12°.
  let outcome;
  if (term.outcome !== 'touchdown') outcome = term.outcome;
  else if (term.v_vertical > 3.0 || Math.abs(term.v_horizontal) > 1.5) outcome = 'crash';
  else if (slope > deg2rad_(12.0)) outcome = 'tipped';
  else outcome = 'touchdown';
  return descentResult({
    log: L, outcome, t_touchdown: leg.t + term.t, t_gate: leg.t,
    v_vertical: term.v_vertical, v_horizontal: term.v_horizontal,
    downrange: L.downrange[L.downrange.length - 1],
    dv_braking: dv_brake, dv_terminal: dv_term,
    prop_used: m0 - term.m, prop_left, hover_s: hover,
    min_throttle: term.min_throttle, pitch0: p0, pitch_rate: pr,
    r: term.r, v: term.v, m: term.m, slope, elev,
    site_score: score, site_score_nominal: score0, redesignated: moved,
    nav_err: nav_dr, nav_dh,
    radar_locked: nav !== null && nav.locked_h,
  });
}

// ------------------------------------------------------- mission assembly ---

// Design and fly the whole landing mission: pad to lunar surface.
export function moonlanding(opts = {}) {
  const { lander: landerIn = default_lander(),
          h_park = 200.0e3, h_moon_park = 100.0e3, h_pdi = 15.0e3, n_rev = 1,
          inclination = deg2rad_(28.5), lv: lvIn = null, hp_return = 50.0e3,
          h_gate = 2000.0, terrain = null, field = null, nav: navIn = null,
          hazard = null, survey_error = 60.0, orbiter: orbiterIn = null,
          kick_angle = deg2rad_(8.0), optimize_kick = false, cis_eta = CIS_ETA,
          perigee_tol = 5.0e3, verbose = false } = opts;
  let nav = navIn;
  const m_payload = lander_mass(landerIn) +
    (orbiterIn === null ? 0.0 : orbiter_mass(orbiterIn));
  let lv = lvIn;
  if (lv === null) lv = default_moon_rocket({ payload: m_payload });
  if (!(Math.abs(lv.payload_mass - m_payload) < 1.0))
    throw new Error(`launch vehicle payload (${Math.round(lv.payload_mass)} kg) is not the ` +
      `mass being sent to the Moon (${Math.round(m_payload)} kg: a ` +
      `${Math.round(lander_mass(landerIn))} kg lander` +
      (orbiterIn === null ? '' : ` and a ${Math.round(orbiter_mass(orbiterIn))} kg orbiter`) + ')');

  // The free return is an *abort* path here, not an entry corridor.
  const des = translunar_design(lv, { h_park, hp_moon: h_moon_park, hp_return,
    inclination, kick_angle, optimize_kick, cis_eta, perigee_tol,
    tol_perigee_km: 30.0, verbose });
  const asc = des.ascent, eph = des.eph;

  const cis = fly_to_perilune(asc.r, asc.v, asc.t, eph, {
    t_ign: des.t_ign, dv: des.dv, stage: des.kick, m_stack: des.m_stack,
    prop_avail: asc.prop_left[asc.prop_left.length - 1], eta: cis_eta });
  if (cis.outcome !== 'perilune')
    throw new Error(`trans-lunar leg did not reach perilune (outcome: ${cis.outcome})`);

  // --- insertion ---------------------------------------------------------
  const t_loi = cis.t;
  const [r_m, v_m] = mci_state(cis.r, cis.v, t_loi, eph);
  const [dv_loi, v_after] = loi_burn(r_m, v_m);
  let m = _burn_mass(landerIn, lander_mass(landerIn), dv_loi);
  if (m <= landerIn.mdry)
    throw new Error(`lunar-orbit insertion alone empties the lander ` +
      `(${Math.round(dv_loi)} m/s needed, ${Math.round(lander_dv(landerIn))} m/s carried)`);
  // the orbiter arrives on the same trajectory and pays the same delta-v
  let m_orb = 0.0;
  if (orbiterIn !== null) {
    m_orb = orbiter_mass(orbiterIn) * Math.exp(-dv_loi / (G0 * orbiterIn.isp));
    if (m_orb <= orbiterIn.mdry)
      throw new Error(`lunar-orbit insertion alone empties the orbiter ` +
        `(${Math.round(dv_loi)} m/s needed, ${Math.round(orbiter_dv(orbiterIn))} m/s carried)`);
  }

  // --- parking orbit, DOI, coast to the descent periapsis ----------------
  const OL = lunarOrbitLog();
  let r_park = r_m, v_park = v_after;
  const T_park = 2 * Math.PI * Math.sqrt(vnorm(r_park) ** 3 / MU_MOON);
  [r_park, v_park] = coast_moon(OL, r_park, v_park, t_loi, n_rev * T_park,
    { phase: 0, dt: 5.0, log_every: 8, field, eph });
  const t_doi = t_loi + n_rev * T_park;
  const [dv_doi, v_doi] = doi_burn(r_park, v_park, h_pdi);
  m = _burn_mass(landerIn, m, dv_doi);
  const a_desc = 0.5 * (vnorm(r_park) + R_MOON + h_pdi);
  const t_transfer = Math.PI * Math.sqrt(a_desc ** 3 / MU_MOON);
  const [r_pdi, v_pdi] = coast_moon(OL, r_park, v_doi, t_doi, t_transfer,
    { phase: 1, dt: 2.0, log_every: 8, field, eph });
  const t_pdi = t_doi + t_transfer;

  // --- powered descent ---------------------------------------------------
  // the lander's remaining propellant is what it flies the descent on
  const flying = lander({ name: landerIn.name, mdry: landerIn.mdry,
    mprop: m - landerIn.mdry, thrust: landerIn.thrust, isp: landerIn.isp,
    throttle_min: landerIn.throttle_min, diameter: landerIn.diameter });
  const surf = terrain === null ? null : surfaceModel(terrain, eph);

  // Survey the site the way a real mission does: fly the descent once over a
  // smooth sphere to find out where it is going to end up, then look up what
  // the ground there actually does.
  let h_ref = 0.0;
  if (surf !== null && !Number.isNaN(survey_error)) {
    const dry = powered_descent(flying, r_pdi, v_pdi, m,
      { h_gate, cfg: descentConfig({ eph, t0: t_pdi }) });
    const u_aim = vunit(moonfixed(dry.r, t_pdi + dry.t_touchdown, eph));
    const seed = nav === null ? 0x00537EE1 : (nav.seed ^ 0x00537EE1) >>> 0;
    h_ref = terrain_height(terrain, u_aim) + survey_error * _ngauss(seed, 11);
    if (nav !== null && Number.isNaN(nav.site_elev)) {
      nav = descentNav({ dr_down: nav.dr_down, dr_radial: nav.dr_radial,
        dr_cross: nav.dr_cross, dv_down: nav.dv_down, dv_radial: nav.dv_radial,
        dv_cross: nav.dv_cross, site_elev: h_ref, radar: nav.radar, seed: nav.seed });
    }
  }

  const cfg = descentConfig({ surface: surf, field, nav, hazard, eph, t0: t_pdi });
  const desc = powered_descent(flying, r_pdi, v_pdi, m,
    { h_gate, h_ref, cfg, verbose });
  const t_td = t_pdi + desc.t_touchdown;
  const [lat, lon] = selenographic(desc.r, t_td, eph);

  return landingResult({
    lv, lander: landerIn, guid: des.guid, ascent: asc, eph, cislunar: cis,
    orbit: OL, descent: desc, dv_loi, dv_doi, t_loi, t_doi, t_pdi,
    t_touchdown: t_td, h_park_moon: h_moon_park, h_pdi, n_rev,
    lat_land: lat, lon_land: lon, prop_margin: desc.prop_left,
    orbiter: orbiterIn, m_orbiter: m_orb, r_orbiter: r_m, v_orbiter: v_after,
  });
}

// The landing mission with everything switched on.
export const apollo_landing = ({ terrain = lunarTerrain(), ...kwargs } = {}) =>
  moonlanding({ terrain, field: lunarGravity(), nav: descentNav(),
                hazard: hazardScan(), ...kwargs });
