// Port of src/launch.jl: 3-DOF + mass ascent over the rotating Earth, the
// guidance sequence, the 2x2 pitch shooter and the kick scan.
import { MU_EARTH, RE_MEAN, OMEGA_EARTH, G0, deg2rad_, rad2deg_ } from './constants.js';
import { vadd, vsub, vscale, vdot, vcross, vnorm, vunit, rot_z } from './vec3.js';
import { USSA76, atmosphere_state } from './atmosphere.js';
import { j2Gravity, gravity_accel } from './gravity.js';
import { earth_rotation_angle, ecef_from_geodetic, geodetic_from_ecef, enu_basis,
         elements_from_state } from './frames.js';
import { interp1 } from './aerodynamics.js';
import { stage_thrust, stage_mdot, booster_thrust, booster_mdot,
         pad_thrust, liftoff_mass, frontal_area, core_diameter } from './propulsion.js';

const OMEGA = [0.0, 0.0, OMEGA_EARTH];
const PITCH_CMD_MIN = -25.0 * Math.PI / 180;
const PITCH_CMD_MAX = 85.0 * Math.PI / 180;
const SECO_HP_MIN = 100.0e3;
const MAX_ASCENT_ACCEL = 2.0 * G0;
const MIN_CORE_THROTTLE = 0.20;

export const ascentGuidance = (o = {}) => ({
  site_lat: o.site_lat ?? deg2rad_(28.5),
  site_lon: o.site_lon ?? deg2rad_(-80.6),
  azimuth: o.azimuth ?? deg2rad_(90.0),
  v_pitchover: o.v_pitchover ?? 40.0,
  kick_angle: o.kick_angle ?? deg2rad_(8.0),
  kick_duration: o.kick_duration ?? 14.0,
  pitch0: o.pitch0 ?? deg2rad_(26.0),
  pitch_rate: o.pitch_rate ?? 1.45e-3,
  h_target: o.h_target ?? 200.0e3,
  fairing_alt: o.fairing_alt ?? 120.0e3,
  stage_gap: o.stage_gap ?? 4.0,
  cutoff: o.cutoff ?? 'energy',
  apogee_target: o.apogee_target ?? NaN,
  range_target: o.range_target ?? NaN,
  pitch_hold: o.pitch_hold ?? NaN,
});

const reguid = (g, kw) => ascentGuidance({ ...g, ...kw });

export const launch_azimuth = (inc, lat) =>
  Math.asin(Math.min(1, Math.max(-1, Math.cos(inc) / Math.cos(lat))));

export function site_geocentric_lat(lat, lon) {
  const r = ecef_from_geodetic(lat, lon, 0.0);
  return Math.atan2(r[2], Math.hypot(r[0], r[1]));
}

export function launch_window(guid, inc, raan, { theta_g0 = 0.0, after = 0.0 } = {}) {
  const out = [];
  if (Math.abs(Math.sin(inc)) < 1e-12) return out;
  const phi = site_geocentric_lat(guid.site_lat, guid.site_lon);
  const k = -Math.tan(phi) / Math.tan(inc);
  if (Math.abs(k) > 1.0) return out;
  const base = Math.asin(k);
  const sidereal = 2 * Math.PI / OMEGA_EARTH;
  const az0 = launch_azimuth(inc, guid.site_lat);
  const mod = (x, m) => x - m * Math.floor(x / m);
  for (const [node, dO] of [['ascending', base], ['descending', Math.PI - base]]) {
    const alpha = raan - dO;
    let t = (alpha - guid.site_lon - theta_g0) / OMEGA_EARTH;
    t = after + mod(t - after, sidereal);
    out.push({ t, node, azimuth: node === 'ascending' ? az0 : Math.PI - az0 });
  }
  out.sort((a, b) => a.t - b.t);
  return out;
}

export function next_launch_window(guid, inc, raan, { node = 'ascending', ...rest } = {}) {
  const ws = launch_window(guid, inc, raan, rest);
  const w = ws.find(w => w.node === node);
  return w ?? null;
}

export const ascentCtx = (stage, phase, t_ign, t_kick0, t_loop0, fairing_on,
                          burning, nb = 0) => ({
  stage, phase, t_stage_ign: t_ign, t_kick0, t_loop0, fairing_on, burning,
  burned: 0.0, bstate: new Array(nb).fill('waiting'),
  b_tign: new Array(nb).fill(NaN), b_tspent: new Array(nb).fill(NaN),
});

export const booster_attached = ctx => ctx.bstate.map(s => s !== 'gone');

export function core_throttle(lv, ctx, mass = Infinity, pamb = 0.0) {
  let thr = 1.0;
  if (ctx.stage === 1) {
    for (let i = 0; i < lv.boosters.length; i++)
      if (ctx.bstate[i] === 'burning') thr = Math.min(thr, lv.boosters[i].core_throttle);
  }
  if (Number.isFinite(mass) && ctx.stage >= 1 && lv.boosters.length === 0 &&
      pad_thrust(lv) / liftoff_mass(lv) > MAX_ASCENT_ACCEL) {
    let fixed = 0;
    for (let i = 0; i < lv.boosters.length; i++)
      if (ctx.bstate[i] === 'burning') fixed += booster_thrust(lv.boosters[i], pamb);
    const core = stage_thrust(lv.stages[ctx.stage - 1], pamb);
    const limit = (MAX_ASCENT_ACCEL * mass - fixed) / Math.max(core, 1.0);
    thr = Math.min(thr, Math.min(1.0, Math.max(MIN_CORE_THROTTLE, limit)));
  }
  return thr;
}

export function _perigee_radius(r, v) {
  const rr = vnorm(r);
  const eps = 0.5 * vdot(v, v) - MU_EARTH / rr;
  if (eps >= 0.0) return Infinity;
  const hv = vcross(r, v);
  const a = -MU_EARTH / (2 * eps);
  const e = Math.sqrt(Math.max(0.0, 1 + 2 * eps * vdot(hv, hv) / MU_EARTH ** 2));
  return a * (1 - e);
}

export function _apogee_radius(r, v) {
  const rr = vnorm(r);
  const eps = 0.5 * vdot(v, v) - MU_EARTH / rr;
  if (eps >= 0.0) return Infinity;
  const hv = vcross(r, v);
  const a = -MU_EARTH / (2 * eps);
  const e = Math.sqrt(Math.max(0.0, 1 + 2 * eps * vdot(hv, hv) / MU_EARTH ** 2));
  return a * (1 + e);
}

export function _ballistic_range(r, v) {
  const el = elements_from_state(r, v);
  if (el.e >= 1.0 || el.a <= 0.0) return Infinity;
  if (el.rp >= RE_MEAN) return Infinity;
  const p = el.a * (1 - el.e * el.e);
  const cnu = Math.min(1, Math.max(-1, (p / RE_MEAN - 1) / el.e));
  const nu_i = Math.acos(cnu);
  const psi = (2 * Math.PI - nu_i) - el.nu;
  if (psi <= 0.0) return 0.0;
  return RE_MEAN * psi;
}

export function _suborbital_cut(guid, r, v, dr) {
  if (guid.cutoff === 'apogee') {
    if (Number.isNaN(guid.apogee_target)) return false;
    return _apogee_radius(r, v) >= RE_MEAN + guid.apogee_target;
  } else if (guid.cutoff === 'range') {
    if (Number.isNaN(guid.range_target)) return false;
    return dr + _ballistic_range(r, v) >= guid.range_target;
  }
  return false;
}

export function _steer(guid, ctx, r, v, t, theta_g0) {
  const rhat = vunit(r);
  if (ctx.phase === 'vertical') return rhat;
  const vrel = vsub(v, vcross(OMEGA, r));
  if (ctx.phase === 'kick') {
    const theta = earth_rotation_angle(theta_g0, t);
    const [lat, lon] = geodetic_from_ecef(rot_z(r, theta));
    const [eE, eN] = enu_basis(lat, lon);
    const az_ecef = vadd(vscale(eN, Math.cos(guid.azimuth)), vscale(eE, Math.sin(guid.azimuth)));
    const az_eci = rot_z(az_ecef, -theta);
    const az_h = vunit(vsub(az_eci, vscale(rhat, vdot(az_eci, rhat))));
    const sk = Math.sin(guid.kick_angle), ck = Math.cos(guid.kick_angle);
    return vadd(vscale(rhat, ck), vscale(az_h, sk));
  } else if (ctx.phase === 'gravity_turn') {
    return vunit(vrel);
  } else {
    const that = vunit(vsub(v, vscale(rhat, vdot(v, rhat))));
    if (!Number.isNaN(guid.pitch_hold)) {
      const th = Math.min(0.5 * Math.PI, Math.max(-0.5 * Math.PI, guid.pitch_hold));
      const sh = Math.sin(th), ch = Math.cos(th);
      if (ch < 1e-6) return rhat;
      const vh = vsub(v, vscale(rhat, vdot(v, rhat)));
      if (vnorm(vh) < 1.0) return rhat;
      return vadd(vscale(vunit(vh), ch), vscale(rhat, sh));
    }
    const tt = Math.tan(guid.pitch0) - guid.pitch_rate * (t - ctx.t_loop0);
    const th = Math.min(PITCH_CMD_MAX, Math.max(PITCH_CMD_MIN, Math.atan(tt)));
    return vadd(vscale(that, Math.cos(th)), vscale(rhat, Math.sin(th)));
  }
}

export function _ascent_deriv(dx, x, lv, guid, ctx, atm, grav, theta_g0, t) {
  const r = [x[0], x[1], x[2]];
  const v = [x[3], x[4], x[5]];
  const m = x[6];

  let a = gravity_accel(grav, r, t);
  const theta = earth_rotation_angle(theta_g0, t);
  const h = geodetic_from_ecef(rot_z(r, theta))[2];

  let dm = 0.0, thrust_mag = 0.0, pamb = 0.0, rho = 0.0, asnd = 300.0;
  if (h < 150.0e3) [rho, , pamb, asnd] = atmosphere_state(atm, Math.max(h, 0.0));

  if (ctx.burning && ctx.stage >= 1) {
    const st = lv.stages[ctx.stage - 1];
    const thr = core_throttle(lv, ctx, m, pamb);
    thrust_mag = thr * stage_thrust(st, pamb);
    dm = -thr * stage_mdot(st);
  }
  for (let i = 0; i < lv.boosters.length; i++) {
    if (ctx.bstate[i] !== 'burning') continue;
    thrust_mag += booster_thrust(lv.boosters[i], pamb);
    dm -= booster_mdot(lv.boosters[i]);
  }
  if (thrust_mag > 0.0) {
    const dhat = _steer(guid, ctx, r, v, t, theta_g0);
    a = vadd(a, vscale(dhat, thrust_mag / m));
  }

  if (rho > 0) {
    const vrel = vsub(v, vcross(OMEGA, r));
    const Vr = vnorm(vrel);
    if (Vr > 1.0) {
      const M = Vr / asnd;
      const sref = lv.boosters.length === 0 ? lv.sref
        : frontal_area(lv, booster_attached(ctx));
      const D = 0.5 * rho * Vr * Vr * sref * interp1(lv.cd, M);
      a = vadd(a, vscale(vrel, -D / (m * Vr)));
    }
  }

  dx[0] = v[0]; dx[1] = v[1]; dx[2] = v[2];
  dx[3] = a[0]; dx[4] = a[1]; dx[5] = a[2];
  dx[6] = dm;
}

function rk4_ascent(xo, x, t, dt, w, lv, guid, ctx, atm, grav, th0) {
  const n = x.length;
  _ascent_deriv(w.k1, x, lv, guid, ctx, atm, grav, th0, t);
  for (let i = 0; i < n; i++) w.xt[i] = x[i] + 0.5 * dt * w.k1[i];
  _ascent_deriv(w.k2, w.xt, lv, guid, ctx, atm, grav, th0, t + 0.5 * dt);
  for (let i = 0; i < n; i++) w.xt[i] = x[i] + 0.5 * dt * w.k2[i];
  _ascent_deriv(w.k3, w.xt, lv, guid, ctx, atm, grav, th0, t + 0.5 * dt);
  for (let i = 0; i < n; i++) w.xt[i] = x[i] + dt * w.k3[i];
  _ascent_deriv(w.k4, w.xt, lv, guid, ctx, atm, grav, th0, t + dt);
  for (let i = 0; i < n; i++)
    xo[i] = x[i] + (dt / 6) * (w.k1[i] + 2 * w.k2[i] + 2 * w.k3[i] + w.k4[i]);
}

export function _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atm = USSA76) {
  const r = [x[0], x[1], x[2]];
  const v = [x[3], x[4], x[5]];
  const theta = earth_rotation_angle(theta_g0, t);
  const [lat, lon, h] = geodetic_from_ecef(rot_z(r, theta));
  const vrel = vsub(v, vcross(OMEGA, r));
  const Vr = vnorm(vrel);
  let rho = 0.0, asnd = 300.0, pamb = 0.0;
  if (h < 150e3) [rho, , pamb, asnd] = atmosphere_state(atm, Math.max(h, 0.0));
  const qbar = 0.5 * rho * Vr * Vr;
  const rhat = vunit(r);
  const vin = vnorm(v);
  const gamma = vin > 1 ? Math.asin(Math.min(1, Math.max(-1, vdot(rhat, vscale(v, 1 / vin))))) : Math.PI / 2;
  const gamma_rel = Vr > 1 ? Math.asin(Math.min(1, Math.max(-1, vdot(rhat, vscale(vrel, 1 / Vr))))) : Math.PI / 2;
  let thrust = ctx.burning && ctx.stage >= 1
    ? core_throttle(lv, ctx, x[6], pamb) * stage_thrust(lv.stages[ctx.stage - 1], pamb)
    : 0.0;
  for (let i = 0; i < lv.boosters.length; i++)
    if (ctx.bstate[i] === 'burning') thrust += booster_thrust(lv.boosters[i], pamb);
  const dr = RE_MEAN * Math.acos(Math.min(1, Math.max(-1, vdot(vunit(r_site0), rhat))));
  return { h, vrel: Vr, vin, gamma, gamma_rel, mach: Vr / asnd, qbar, pamb,
           lat, lon, thrust, downrange: dr };
}

export function simulate_ascent(lv, guid, { atmosphere = USSA76, gravity = j2Gravity(),
    theta_g0 = 0.0, dt = 0.10, log_dt = 1.0, t_max = 2.0e3 } = {}) {
  const r_ecef = ecef_from_geodetic(guid.site_lat, guid.site_lon, 0.0);
  const r0 = rot_z(r_ecef, -theta_g0);
  const v0 = vcross(OMEGA, r0);
  const x = Float64Array.from([r0[0], r0[1], r0[2], v0[0], v0[1], v0[2], liftoff_mass(lv)]);
  const xnew = new Float64Array(7);
  const w = { k1: new Float64Array(7), k2: new Float64Array(7), k3: new Float64Array(7),
              k4: new Float64Array(7), xt: new Float64Array(7) };

  const nst = lv.stages.length;
  const prop_left = lv.stages.map(s => s.mprop);
  const ctx = ascentCtx(1, 'vertical', 0.0, NaN, NaN, lv.fairing_mass > 0, true, lv.boosters.length);
  const events = [];
  const L = { t: [], rx: [], ry: [], rz: [], h: [], vrel: [], vin: [], gamma: [],
              mach: [], qbar: [], m: [], thrust: [], lat: [], lon: [], downrange: [] };
  const r_site0 = r0;

  const a_t = RE_MEAN + guid.h_target;
  const e_target = -MU_EARTH / (2 * a_t);

  let t = 0.0, next_log = 0.0, reached = false, h_cut = NaN, gamma_cut = NaN;

  const ev = name => {
    const d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere);
    events.push({ name, t, h: d.h, vrel: d.vrel, m: x[6] });
    return d;
  };
  const logrec = d => {
    L.t.push(t); L.rx.push(x[0]); L.ry.push(x[1]); L.rz.push(x[2]);
    L.h.push(d.h); L.vrel.push(d.vrel); L.vin.push(d.vin); L.gamma.push(d.gamma);
    L.mach.push(d.mach); L.qbar.push(d.qbar); L.m.push(x[6]); L.thrust.push(d.thrust);
    L.lat.push(d.lat); L.lon.push(d.lon); L.downrange.push(d.downrange);
  };
  ev('liftoff');

  while (t < t_max) {
    let d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere);
    if (ctx.phase === 'vertical' && d.vrel >= guid.v_pitchover) {
      ctx.phase = 'kick'; ctx.t_kick0 = t; ev('pitchover');
    } else if (ctx.phase === 'kick' && t - ctx.t_kick0 >= guid.kick_duration) {
      ctx.phase = 'gravity_turn'; ev('gravity_turn');
    }
    if (ctx.phase === 'gravity_turn' && !Number.isNaN(guid.pitch_hold) &&
        d.gamma_rel <= guid.pitch_hold + 1e-9) {
      ctx.phase = 'closed_loop';
      if (Number.isNaN(ctx.t_loop0)) ctx.t_loop0 = t;
      ev('pitch_hold');
    }
    if (ctx.fairing_on && d.h >= guid.fairing_alt) {
      ctx.fairing_on = false;
      x[6] -= lv.fairing_mass;
      ev('fairing_jettison');
    }

    if (ctx.burning && lv.boosters.length === 0 && guid.cutoff === 'energy' &&
        t > 60.0 && d.gamma < deg2rad_(-10.0)) {
      const st = lv.stages[ctx.stage - 1];
      prop_left[ctx.stage - 1] = st.mprop - ctx.burned;
      ctx.burning = false; ctx.phase = 'coast';
      reached = false; h_cut = d.h; gamma_cut = d.gamma;
      ev('powered_descent_abort');
      break;
    }
    if (t > 0.5 && d.h <= 0.0) {
      const theta = earth_rotation_angle(theta_g0, t);
      const rs = rot_z(ecef_from_geodetic(d.lat, d.lon, 0.0), -theta);
      x[0] = rs[0]; x[1] = rs[1]; x[2] = rs[2];
      for (let k = 0; k < 4; k++) {
        d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere);
        if (Math.abs(d.h) < 1e-6) break;
        const rh = vunit([x[0], x[1], x[2]]);
        x[0] -= d.h * rh[0]; x[1] -= d.h * rh[1]; x[2] -= d.h * rh[2];
      }
      d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere);
      h_cut = 0.0; gamma_cut = d.gamma;
      ctx.burning = false; ctx.phase = 'coast';
      reached = false;
      ev('ground_impact');
      break;
    }

    for (let i = 0; i < lv.boosters.length; i++) {
      const b = lv.boosters[i];
      if (ctx.bstate[i] === 'waiting' && t >= b.ignition_delay) {
        ctx.bstate[i] = 'burning'; ctx.b_tign[i] = t; ev(`ignition_${b.stage.name}`);
      }
      if (ctx.bstate[i] === 'burning' && t - ctx.b_tign[i] >= stage_burn_time_of(b.stage) - 1e-9) {
        ctx.bstate[i] = 'spent'; ctx.b_tspent[i] = t; ev(`burnout_${b.stage.name}`);
      }
      if (ctx.bstate[i] === 'spent' && t - ctx.b_tspent[i] >= b.sep_delay) {
        ctx.bstate[i] = 'gone';
        x[6] -= b.count * b.stage.mdry;
        ev(`sep_${b.stage.name}`);
      }
    }

    if (ctx.burning) {
      const st = lv.stages[ctx.stage - 1];
      if (ctx.burned >= st.mprop - 1e-9) {
        prop_left[ctx.stage - 1] = 0.0;
        x[6] -= st.mdry;
        ev(`sep_${st.name}`);
        if (ctx.stage < nst) {
          ctx.stage += 1; ctx.burning = false; ctx.t_stage_ign = t + guid.stage_gap;
        } else {
          ctx.stage = 0; ctx.burning = false; ctx.phase = 'coast';
          h_cut = d.h; gamma_cut = d.gamma; ev('propellant_depletion');
          break;
        }
      }
      if (ctx.burning && guid.cutoff !== 'energy' && d.h > 200.0 &&
          _suborbital_cut(guid, [x[0], x[1], x[2]], [x[3], x[4], x[5]], d.downrange)) {
        const s2 = lv.stages[ctx.stage - 1];
        prop_left[ctx.stage - 1] = s2.mprop - ctx.burned;
        ctx.burning = false; ctx.phase = 'coast'; reached = false;
        h_cut = d.h; gamma_cut = d.gamma; ev('seco');
        break;
      }
      if (ctx.burning && ctx.phase === 'closed_loop' && guid.cutoff === 'energy') {
        const eps_now = 0.5 * d.vin ** 2 - MU_EARTH / vnorm([x[0], x[1], x[2]]);
        if (eps_now >= e_target) {
          const s2 = lv.stages[ctx.stage - 1];
          prop_left[ctx.stage - 1] = s2.mprop - ctx.burned;
          ctx.burning = false; ctx.phase = 'coast';
          const rp = _perigee_radius([x[0], x[1], x[2]], [x[3], x[4], x[5]]);
          reached = rp >= RE_MEAN + SECO_HP_MIN;
          h_cut = d.h; gamma_cut = d.gamma;
          ev('seco');
          if (!reached) ev('insertion_below_surface');
          break;
        }
      }
    } else if (ctx.stage >= 1 && ctx.stage <= nst && t >= ctx.t_stage_ign) {
      ctx.burning = true; ctx.burned = 0.0; ctx.phase = 'closed_loop';
      if (Number.isNaN(ctx.t_loop0)) ctx.t_loop0 = t;
      ev(`ignition_${lv.stages[ctx.stage - 1].name}`);
    }

    if (t >= next_log) { logrec(d); next_log += log_dt; }

    const thr_now = ctx.burning && ctx.stage >= 1 ? core_throttle(lv, ctx, x[6], d.pamb) : 0.0;
    let dts = dt;
    if (thr_now > 0) {
      const mdot_core = thr_now * stage_mdot(lv.stages[ctx.stage - 1]);
      dts = Math.min(dts, (lv.stages[ctx.stage - 1].mprop - ctx.burned) / mdot_core);
    }
    for (let i = 0; i < lv.boosters.length; i++) {
      if (ctx.bstate[i] !== 'burning') continue;
      dts = Math.min(dts, stage_burn_time_of(lv.boosters[i].stage) - (t - ctx.b_tign[i]));
    }
    dts = Math.min(dt, Math.max(1e-6, dts));
    const m_before = x[6];
    rk4_ascent(xnew, x, t, dts, w, lv, guid, ctx, atmosphere, gravity, theta_g0);
    x.set(xnew); t += dts;
    if (ctx.burning && ctx.stage >= 1) {
      ctx.burned += lv.boosters.length === 0
        ? (m_before - x[6])
        : thr_now * stage_mdot(lv.stages[ctx.stage - 1]) * dts;
    }
  }

  const d = _ascent_data(x, lv, ctx, theta_g0, t, r_site0, atmosphere);
  logrec(d);
  const r = [x[0], x[1], x[2]], v = [x[3], x[4], x[5]];
  const el = elements_from_state(r, v);
  return { log: L, events, r, v, m: x[6], t, elements: el, prop_left,
           reached_orbit: reached, h_cut, gamma_cut };
}

// stage.mprop / stage_mdot, kept local to avoid a name clash with the exported
// burn-time helper carrying a different signature elsewhere.
const stage_burn_time_of = st => st.mprop / (st.thrust_vac / (G0 * st.isp_vac));

export function tune_ascent(lv, guid, { tol_h = 1.0e3, tol_gamma = deg2rad_(0.05),
    max_iter = 30, verbose = false, optimize_kick = false, recover_kick = true, onProgress = null,
    ...kwargs } = {}) {
  if (optimize_kick)
    return _tune_with_kick(lv, guid, { tol_h, tol_gamma, max_iter, verbose, onProgress, ...kwargs });

  let p1 = guid.pitch0, p2 = guid.pitch_rate, res;
  const resid = g => {
    const r = simulate_ascent(lv, g, kwargs);
    return [r.h_cut - g.h_target, r.gamma_cut, r];
  };
  const rebuild = (a, b) => reguid(guid, { pitch0: a, pitch_rate: b });
  for (let it = 0; it < max_iter; it++) {
    if (onProgress) onProgress({ ok: true, stage: 'ascent',
      detail: `solving ascent guidance · iteration ${it + 1} of ${max_iter}`, current: 1, total: 6 });
    const g = rebuild(p1, p2);
    const [f1, f2, r] = resid(g);
    res = r;
    if (Math.abs(f1) < tol_h && Math.abs(f2) < tol_gamma && r.reached_orbit)
      return [g, r];
    const d1 = deg2rad_(0.4), d2 = 5.0e-5;
    const [f1a, f2a] = resid(rebuild(p1 + d1, p2));
    const [f1b, f2b] = resid(rebuild(p1, p2 + d2));
    const j11 = (f1a - f1) / d1, j21 = (f2a - f2) / d1;
    const j12 = (f1b - f1) / d2, j22 = (f2b - f2) / d2;
    const det = j11 * j22 - j12 * j21;
    if (Math.abs(det) < 1e-12) break;
    const dp1 = -(j22 * f1 - j12 * f2) / det;
    const dp2 = -(-j21 * f1 + j11 * f2) / det;
    const lim1 = deg2rad_(6.0), lim2 = 6.0e-4;
    p1 += Math.min(lim1, Math.max(-lim1, 0.8 * dp1));
    p2 += Math.min(lim2, Math.max(-lim2, 0.8 * dp2));
  }
  const g = rebuild(p1, p2);
  const [, , r] = resid(g);
  res = r;
  if (recover_kick && guid.cutoff === 'energy' && !r.reached_orbit &&
      pad_thrust(lv) > 1.05 * liftoff_mass(lv) * G0)
    return _tune_with_kick(lv, guid, { tol_h, tol_gamma, max_iter, verbose, onProgress, ...kwargs });
  return [g, r];
}

const _with_kick = (g, ka) => reguid(g, { kick_angle: ka });

function _tune_with_kick(lv, guid, { tol_h, tol_gamma, max_iter, verbose, onProgress, ...kwargs }) {
  let candidates = 0;
  const scan = angles => {
    const out = [];
    for (const ka of angles) {
      if (ka <= 0) continue;
      candidates++;
      if (onProgress) onProgress({ ok: true, stage: 'ascent',
        detail: `optimizing pitch kick · candidate ${candidates}, ${(ka * 180 / Math.PI).toFixed(2)}°`,
        current: 1, total: 6 });
      const [g, r] = tune_ascent(lv, _with_kick(guid, ka),
        { tol_h, tol_gamma, max_iter, optimize_kick: false, recover_kick: false, ...kwargs });
      const ok = r.reached_orbit && Math.abs(r.h_cut - g.h_target) < tol_h &&
                 Math.abs(r.gamma_cut) < tol_gamma;
      if (ok) out.push([g, r]);
    }
    return out;
  };
  const pick = cands => {
    if (!cands.length) return null;
    let bi = 0;
    for (let i = 1; i < cands.length; i++) if (cands[i][1].m > cands[bi][1].m) bi = i;
    return cands[bi];
  };
  const ladder = [0.6, 1.0, 1.5, 2.2, 3.0, 4.0, 5.5, 7.5, 10.0, 13.5, 18.0].map(deg2rad_);
  let best = pick(scan(ladder));
  if (best !== null) {
    const ka = best[0].kick_angle;
    let i = 0;
    for (let j = 1; j < ladder.length; j++) if (Math.abs(ladder[j] - ka) < Math.abs(ladder[i] - ka)) i = j;
    const lo = i > 0 ? Math.sqrt(ladder[i - 1] * ka) : ka * 0.8;
    const hi = i < ladder.length - 1 ? Math.sqrt(ladder[i + 1] * ka) : ka * 1.25;
    best = pick([best, ...scan([lo, hi])]);
  }
  return best === null
    ? tune_ascent(lv, guid, { tol_h, tol_gamma, max_iter, optimize_kick: false, recover_kick: false, onProgress, ...kwargs })
    : best;
}
