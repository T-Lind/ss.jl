// Port of src/earthorbit.jl: Earth-orbit missions — launch, transfer burns on
// the kick stage, the target orbit, and an optional deorbit + entry.
import { MU_EARTH, RE_MEAN, G0, deg2rad_ } from './constants.js';
import { vadd, vsub, vscale, vdot, vcross, vnorm, vunit } from './vec3.js';
import { elements_from_state } from './frames.js';
import { coplanar_moon } from './moon.js';
import { default_moon_rocket, stage_mdot } from './propulsion.js';
import { launch_azimuth, ascentGuidance, tune_ascent } from './launch.js';
import { cislunarLog, cis_push, _cis_dt, _cis_step, _cis_accel } from './translunar.js';
import { default_reentry_pod } from './vehicle.js';
import { scenario } from './dynamics.js';
import { simulate } from './simulation.js';

export const orbitTarget = (o = {}) => ({
  name: o.name,
  perigee_alt: o.perigee_alt ?? 200.0e3,
  apogee_alt: o.apogee_alt ?? 200.0e3,
  inclination: o.inclination ?? deg2rad_(28.5),
  note: o.note ?? '',
});

export const ORBITS = {
  leo: orbitTarget({ name: 'leo', perigee_alt: 200.0e3, apogee_alt: 200.0e3,
                     inclination: deg2rad_(28.5), note: 'direct ascent, no transfer' }),
  polar: orbitTarget({ name: 'polar', perigee_alt: 500.0e3, apogee_alt: 500.0e3,
                       inclination: deg2rad_(90.0),
                       note: 'polar; the ascent flies the azimuth, two burns raise the circle' }),
  geo: orbitTarget({ name: 'geo', perigee_alt: 35786.0e3, apogee_alt: 35786.0e3,
                     inclination: 0.0,
                     note: 'raise at the node, combined circularise + plane change at apogee' }),
  molniya: orbitTarget({ name: 'molniya', perigee_alt: 600.0e3, apogee_alt: 39400.0e3,
                         inclination: deg2rad_(63.4),
                         note: 'critical inclination; argp is reported, not commanded' }),
};

export const earthOrbitResult = (o = {}) => ({
  lv: o.lv, guid: o.guid, ascent: o.ascent, eph: o.eph, log: o.log,
  burns: o.burns ?? [], elements: o.elements, target: o.target,
  on_target: o.on_target ?? false, outcome: o.outcome ?? 'ascent_failed',
  r: o.r, v: o.v, t: o.t ?? 0.0, m: o.m ?? 0.0,
  entry_scn: o.entry_scn ?? null, entry: o.entry ?? null,
});

// ------------------------------------------------------------------ coasts --

// Coast for a fixed duration, logging with the given phase tag.
function _eo_coast_time(L, r, v, t, dt_total, eph,
                        { phase, theta_g0 = 0.0, log_every = 4 } = {}) {
  const tend = t + dt_total;
  let kount = 0;
  while (t < tend) {
    const dtp = Math.min(_cis_dt(r, t, eph, { dt_max: 120.0 }), tend - t);
    [r, v] = _cis_step(r, v, t, dtp, eph);
    t += dtp;
    if ((++kount % log_every) === 0) cis_push(L, t, r, v, eph, theta_g0, phase);
  }
  return [r, v, t];
}

// Coast until f(r, v) crosses zero on a falling edge (dir = -1), a rising edge
// (dir = +1), or either (dir = 0); bisects the final step onto the crossing.
function _eo_coast_until(L, r, v, t, f, eph,
                         { dir = 0, phase, theta_g0 = 0.0,
                           t_max = 3.0 * 86400.0, log_every = 4 } = {}) {
  const tend = t + t_max;
  let s_prev = f(r, v);
  let armed = dir === 0 ? s_prev !== 0.0 : s_prev * dir < 0.0;
  let kount = 0;
  while (t < tend) {
    const dtp = _cis_dt(r, t, eph, { dt_max: 120.0 });
    const rp_ = r, vp_ = v, tp_ = t;
    [r, v] = _cis_step(r, v, t, dtp, eph);
    t += dtp;
    const s = f(r, v);
    const crossed = armed && s_prev * s < 0.0 && (dir === 0 || s * dir > 0.0);
    if (crossed) {
      let lo_r = rp_, lo_v = vp_, lo_t = tp_;
      let width = dtp;
      for (let k = 0; k < 12; k++) {
        width /= 2;
        const [rm, vm] = _cis_step(lo_r, lo_v, lo_t, width, eph);
        if (f(rm, vm) * s_prev > 0.0) { lo_r = rm; lo_v = vm; lo_t += width; }
      }
      cis_push(L, lo_t, lo_r, lo_v, eph, theta_g0, phase);
      return [lo_r, lo_v, lo_t];
    }
    armed = armed || (dir === 0 ? s !== 0.0 : s * dir < 0.0);
    s_prev = s;
    if ((++kount % log_every) === 0) cis_push(L, t, r, v, eph, theta_g0, phase);
  }
  return [r, v, t];
}

// ------------------------------------------------------------------- burns --

// Finite steered burn: thrust along dirfn(r, v) until stopfn(r, v) is
// satisfied, the stage runs dry, or t_burn_max elapses.
function _eo_burn(L, r, v, t, m, stage, dirfn, stopfn, eph, prop_avail,
                  { theta_g0 = 0.0, dt = 0.5, log_every = 2, t_burn_max = 2400.0 } = {}) {
  const vex = G0 * stage.isp_vac;
  const md = stage_mdot(stage);
  const m_dry = m - prop_avail;
  const m0 = m, t0 = t;
  let kount = 0, dry = false;
  const onestep = (r, v, t, m, step) => {
    const uhat = dirfn(r, v);
    const acc = (rr, vv, mm, tt) => vadd(_cis_accel(rr, tt, eph),
                                          vscale(uhat, stage.thrust_vac / mm));
    const k1r = v, k1v = acc(r, v, m, t);
    const r2 = vadd(r, vscale(k1r, step / 2)), v2 = vadd(v, vscale(k1v, step / 2));
    const m2 = m - md * step / 2;
    const k2r = v2, k2v = acc(r2, v2, m2, t + step / 2);
    const r3 = vadd(r, vscale(k2r, step / 2)), v3 = vadd(v, vscale(k2v, step / 2));
    const k3r = v3, k3v = acc(r3, v3, m2, t + step / 2);
    const r4 = vadd(r, vscale(k3r, step)), v4 = vadd(v, vscale(k3v, step));
    const k4r = v4, k4v = acc(r4, v4, m - md * step, t + step);
    return [vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step / 6)),
            vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step / 6)),
            t + step, m - md * step];
  };
  while (!stopfn(r, v)) {
    if (m <= m_dry + 1e-9) { dry = true; break; }
    if (t - t0 > t_burn_max) { dry = true; break; }
    let step = Math.min(dt, (m - m_dry) / md);
    let [rn, vn, tn, mn] = onestep(r, v, t, m, step);
    if (stopfn(rn, vn)) {
      for (let k = 0; k < 6; k++) {
        step /= 2;
        const [rh, vh, th, mh] = onestep(r, v, t, m, step);
        if (!stopfn(rh, vh)) { r = rh; v = vh; t = th; m = mh; }
      }
      [rn, vn, tn, mn] = onestep(r, v, t, m, step);
    }
    r = rn; v = vn; t = tn; m = mn;
    if ((++kount % log_every) === 0) cis_push(L, t, r, v, eph, theta_g0, 1);
  }
  return [r, v, t, m, vex * Math.log(m0 / m), t - t0, dry];
}

// Finite velocity-to-gain burn for the shape manoeuvre.
function _eo_burn_vg(L, r, v, t, m, stage, vdesfn, eph, prop_avail,
                     { theta_g0 = 0.0, dt = 0.5, log_every = 2, t_burn_max = 2400.0 } = {}) {
  const u0 = vunit(vsub(vdesfn(r, v), v));
  const dirfn = (rr, vv) => {
    const togo = vsub(vdesfn(rr, vv), vv);
    const n = vnorm(togo);
    return n > 1e-9 ? vscale(togo, 1.0 / n) : u0;
  };
  const stopfn = (rr, vv) => {
    const togo = vsub(vdesfn(rr, vv), vv);
    return vnorm(togo) < 0.5 || vdot(togo, u0) <= 0.0;
  };
  return _eo_burn(L, r, v, t, m, stage, dirfn, stopfn, eph, prop_avail,
                  { theta_g0, dt, log_every, t_burn_max });
}

// Unit thrust direction in the local horizontal, lying in the plane of
// inclination inc through r (or the current plane when inc is NaN).
function _eo_plane_dir(r, v_now, inc) {
  const rhat = vunit(r);
  let n;
  if (Number.isNaN(inc)) {
    n = vunit(vcross(r, v_now));
  } else {
    const h_now = vunit(vcross(r, v_now));
    const zhat = [0.0, 0.0, 1.0];
    const zr = vdot(zhat, rhat);
    const u = vunit(vsub(zhat, vscale(rhat, zr)));
    const w = vunit(vcross(rhat, zhat));
    const alpha = Math.min(1, Math.max(-1, Math.cos(inc) / Math.max(u[2], 1e-9)));
    const beta = Math.sqrt(Math.max(1.0 - alpha * alpha, 0.0));
    const n1 = vadd(vscale(u, alpha), vscale(w, beta));
    const n2 = vadd(vscale(u, alpha), vscale(w, -beta));
    n = vdot(n1, h_now) >= vdot(n2, h_now) ? n1 : n2;
  }
  const d = vunit(vcross(n, rhat));
  return vdot(d, v_now) < 0.0 ? vscale(d, -1.0) : d;
}

// ----------------------------------------------------------------- mission --

export function earthorbit(opts = {}) {
  const { target = 'leo', lv: lvIn = null, pod_mass: pod_mass0 = 350.0,
          h_park = 200.0e3, perigee_alt = NaN, apogee_alt = NaN,
          inclination = NaN, n_orbits = 2.0, deorbit = false,
          hp_entry = 25.0e3, kick_angle = deg2rad_(8.0), optimize_kick = false,
          theta_g0 = 0.0, strict = true, verbose = false } = opts;
  if (!(target in ORBITS) && target !== 'custom')
    throw new Error(`unknown orbit target ${target}; have custom, ` +
                    `${Object.keys(ORBITS).sort().join(', ')}`);
  const tgt0 = target === 'custom'
    ? orbitTarget({ name: 'custom', perigee_alt: 200.0e3, apogee_alt: 200.0e3,
                    inclination: deg2rad_(28.5),
                    note: 'operator-defined perigee, apogee, and inclination' })
    : ORBITS[target];
  const hp = Number.isNaN(perigee_alt) ? tgt0.perigee_alt : perigee_alt;
  const ha = Number.isNaN(apogee_alt) ? tgt0.apogee_alt : apogee_alt;
  const inc = Number.isNaN(inclination) ? tgt0.inclination : inclination;
  if (hp < 100.0e3) throw new Error('target perigee must be at least 100 km');
  if (ha < hp) throw new Error('target apogee must be at or above perigee');
  if (!(0.0 <= inc && inc <= Math.PI))
    throw new Error('inclination must be between 0 and 180 degrees');
  const tgt = orbitTarget({ name: tgt0.name, perigee_alt: hp, apogee_alt: ha,
                            inclination: inc, note: tgt0.note });
  let lv = lvIn;
  if (lv === null) lv = default_moon_rocket({ payload: pod_mass0 });
  const pod_mass = lv.payload_mass;

  const site_lat = deg2rad_(28.5);
  const az = launch_azimuth(tgt.inclination, site_lat);
  const guid0 = ascentGuidance({ azimuth: az, h_target: h_park, kick_angle });
  let [guid, asc] = tune_ascent(lv, guid0, { optimize_kick, theta_g0, verbose });
  if (asc.reached_orbit && tgt.inclination >= site_lat) {
    const miss = tgt.inclination - asc.elements.i;
    if (Math.abs(miss) > deg2rad_(1.0)) {
      const az2 = launch_azimuth(tgt.inclination + miss, site_lat);
      const guid2 = ascentGuidance({ azimuth: az2, h_target: h_park, kick_angle });
      const [g2, a2] = tune_ascent(lv, guid2, { optimize_kick, theta_g0, verbose });
      if (a2.reached_orbit &&
          Math.abs(a2.elements.i - tgt.inclination) < Math.abs(miss)) {
        guid = g2; asc = a2;
      }
    }
  }
  const eph_seed = coplanar_moon([RE_MEAN + h_park, 0.0, 0.0], [0.0, 7.8e3, 0.0]);
  if (!(asc.reached_orbit &&
        asc.elements.rp > RE_MEAN + 0.5 * h_park &&
        Math.abs(asc.gamma_cut) < deg2rad_(1.0))) {
    if (strict) throw new Error(`ascent failed to reach orbit (h_cut=${asc.h_cut / 1e3} km, gamma=${asc.gamma_cut * 180 / Math.PI}°)`);
    return earthOrbitResult({ lv, guid, ascent: asc, eph: eph_seed, log: cislunarLog(),
      burns: [], elements: asc.elements, target: tgt, on_target: false,
      outcome: 'ascent_failed', r: asc.r, v: asc.v, t: asc.t, m: asc.m,
      entry_scn: null, entry: null });
  }

  let m_stack = asc.m;
  for (let k = 0; k < lv.stages.length - 1; k++)
    if (asc.prop_left[k] > 0) m_stack -= lv.stages[k].mdry + asc.prop_left[k];
  const kick = lv.stages[lv.stages.length - 1];
  let prop = asc.prop_left[asc.prop_left.length - 1];

  const eph = coplanar_moon(asc.r, asc.v);
  const L = cislunarLog();
  let r = asc.r, v = asc.v, t = asc.t, m = m_stack;
  cis_push(L, t, r, v, eph, theta_g0, 0);

  const rp_t = RE_MEAN + tgt.perigee_alt;
  const ra_t = RE_MEAN + tgt.apogee_alt;
  const el0 = elements_from_state(r, v);
  const Tpark = 2 * Math.PI * Math.sqrt(el0.a ** 3 / MU_EARTH);
  const need_size = Math.abs(el0.ra - ra_t) > 10.0e3 || Math.abs(el0.rp - rp_t) > 10.0e3;
  const need_plane = Math.abs(el0.i - tgt.inclination) > deg2rad_(0.5);

  const burns = [];
  let dry = false;
  const rdot = (rr, vv) => vdot(rr, vv);
  const prograde = (rr, vv) => vunit(vv);
  const retro = (rr, vv) => vscale(vunit(vv), -1.0);
  const vcirc = rn => Math.sqrt(MU_EARTH / rn);
  const vis = (rn, a) => Math.sqrt(MU_EARTH * Math.max(2.0 / rn - 1.0 / a, 1.0e-12));

  if (need_size || need_plane) {
    if (need_plane) {
      [r, v, t] = _eo_coast_until(L, r, v, t, (rr, vv) => rr[2], eph,
                                  { dir: 0, phase: 0, theta_g0, t_max: 2.0 * Tpark });
    } else {
      [r, v, t] = _eo_coast_time(L, r, v, t, 0.3 * Tpark, eph,
                                 { phase: 0, theta_g0 });
    }
    const rn1 = vnorm(r);
    const plan1 = vis(rn1, 0.5 * (rn1 + ra_t)) - vnorm(v);
    let m_pre = m;
    let dv1, dur1, dry1;
    [r, v, t, m, dv1, dur1, dry1] =
      _eo_burn(L, r, v, t, m, kick, prograde,
               (rr, vv) => elements_from_state(rr, vv).ra >= ra_t,
               eph, prop, { theta_g0 });
    prop = Math.max(prop - (m_pre - m), 0.0);
    burns.push({ name: 'raise', t_ign: t - dur1, duration: dur1, dv_plan: plan1, dv: dv1 });
    dry = dry || dry1;
    if (!dry) {
      [r, v, t] = _eo_coast_until(L, r, v, t, rdot, eph,
        { dir: -1, phase: 2, theta_g0,
          t_max: 1.5 * 2 * Math.PI * Math.sqrt((0.5 * (vnorm(r) + ra_t)) ** 3 / MU_EARTH) });
      const rn2 = vnorm(r);
      const di = need_plane ? Math.abs(el0.i - tgt.inclination) : 0.0;
      const v1 = vnorm(v);
      const v2 = vis(rn2, 0.5 * (rp_t + ra_t));
      const plan2 = Math.sqrt(Math.max(v1 * v1 + v2 * v2 - 2 * v1 * v2 * Math.cos(di), 0.0));
      const a_f = 0.5 * (rp_t + ra_t);
      const e_f = (ra_t - rp_t) / (ra_t + rp_t);
      const h_f = Math.sqrt(MU_EARTH * a_f * (1.0 - e_f * e_f));
      const vdesfn = (rr, vv) => {
        const rn = vnorm(rr);
        const vp = Math.min(h_f / rn, vis(rn, a_f));
        return vscale(_eo_plane_dir(rr, vv, need_plane ? tgt.inclination : NaN), vp);
      };
      m_pre = m;
      let dv2, dur2, dry2;
      [r, v, t, m, dv2, dur2, dry2] =
        _eo_burn_vg(L, r, v, t, m, kick, vdesfn, eph, prop, { theta_g0 });
      prop = Math.max(prop - (m_pre - m), 0.0);
      burns.push({ name: 'shape', t_ign: t - dur2, duration: dur2, dv_plan: plan2, dv: dv2 });
      dry = dry || dry2;
    }
  }

  const el1 = elements_from_state(r, v);
  const tol_r = Math.max(10.0e3, 0.005 * ra_t);
  const on_target = !dry &&
    Math.abs(el1.rp - rp_t) <= tol_r && Math.abs(el1.ra - ra_t) <= tol_r &&
    Math.abs(el1.i - tgt.inclination) <= deg2rad_(1.0);
  const Tfin = el1.a > 0 ? 2 * Math.PI * Math.sqrt(el1.a ** 3 / MU_EARTH) : Tpark;
  [r, v, t] = _eo_coast_time(L, r, v, t, Math.max(n_orbits, 0.25) * Tfin, eph,
                             { phase: 3, theta_g0 });

  let entry_scn = null, entry = null;
  if (deorbit && !dry) {
    const eln = elements_from_state(r, v);
    if (eln.e > 0.01) {
      [r, v, t] = _eo_coast_until(L, r, v, t, rdot, eph,
                                  { dir: -1, phase: 3, theta_g0, t_max: 1.5 * Tfin });
    }
    const rnd = vnorm(r);
    const pland = vnorm(v) - vis(rnd, 0.5 * (rnd + RE_MEAN + hp_entry));
    let dvd, durd, dryd;
    [r, v, t, m, dvd, durd, dryd] =
      _eo_burn(L, r, v, t, m, kick, retro,
               (rr, vv) => elements_from_state(rr, vv).rp <= RE_MEAN + hp_entry,
               eph, prop, { theta_g0 });
    burns.push({ name: 'deorbit', t_ign: t - durd, duration: durd, dv_plan: pland, dv: dvd });
    dry = dry || dryd;
    if (!dryd) {
      const pod = default_reentry_pod({ mass: pod_mass });
      entry_scn = scenario({ vehicle: pod, r0: r, v0: v, t0: t,
                             t_max: t + 3.0e4, theta_g0, alpha0: deg2rad_(5.0) });
      entry = simulate(entry_scn);
    }
  }

  const elf = elements_from_state(r, v);
  const outcome = entry !== null ? 'splashdown' :
                  dry ? 'prop_depleted' : 'on_orbit';
  return earthOrbitResult({ lv, guid, ascent: asc, eph, log: L, burns, elements: elf,
    target: tgt, on_target, outcome, r, v, t, m, entry_scn, entry });
}
