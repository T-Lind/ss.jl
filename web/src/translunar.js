// Port of src/translunar.jl: finite-burn TLI, the cislunar coast (Earth point
// mass + differential lunar gravity), free-return design and mid-course
// correction.
import { MU_EARTH, MU_MOON, R_MOON, A_MOON, N_MOON, G0, RE_MEAN } from './constants.js';
import { vadd, vsub, vscale, vdot, vcross, vnorm, vunit, rot_z } from './vec3.js';
import { earth_rotation_angle, geodetic_from_ecef, elements_from_state } from './frames.js';
import { moon_position, moon_velocity, moon_distance } from './moon.js';
import { stage_mdot } from './propulsion.js';

export const CIS_ETA = 0.002;
export const PERIGEE_TOL = 250.0;
export const CIS_LOG_NEAR = 5.0e7;

export const cislunarLog = () => ({
  t: [], rx: [], ry: [], rz: [], vx: [], vy: [], vz: [], h: [], d_moon: [],
  mx: [], my: [], mz: [], phase: [],
});

export function cis_push(L, t, r, v, eph, theta_g0, phase) {
  const theta = earth_rotation_angle(theta_g0, t);
  const h = geodetic_from_ecef(rot_z(r, theta))[2];
  const m = moon_position(eph, t);
  L.t.push(t);
  L.rx.push(r[0]); L.ry.push(r[1]); L.rz.push(r[2]);
  L.vx.push(v[0]); L.vy.push(v[1]); L.vz.push(v[2]);
  L.h.push(h); L.d_moon.push(vnorm(vsub(m, r)));
  L.mx.push(m[0]); L.my.push(m[1]); L.mz.push(m[2]);
  L.phase.push(phase);
}

export const cislunarResult = (L, outcome, r, v, t, m, dv_tli, t_tli, burn_duration,
                        perilune_alt, t_perilune, vac_perigee_alt, gamma_end,
                        miss_passes, first_perigee_alt) => ({
  log: L, outcome, r, v, t, m, dv_tli, t_tli, burn_duration, perilune_alt,
  t_perilune, vac_perigee_alt, gamma_end, miss_passes, first_perigee_alt,
});

export function _cis_accel(r, t, eph) {
  const rn = vnorm(r);
  const a = vscale(r, -MU_EARTH / (rn * rn * rn));
  const s = moon_position(eph, t);
  const d = vsub(s, r);
  const dn = vnorm(d), sn = vnorm(s);
  return vadd(a, vsub(vscale(d, MU_MOON / (dn * dn * dn)),
                      vscale(s, MU_MOON / (sn * sn * sn))));
}

export function _cis_dt(r, t, eph, { eta = CIS_ETA, dt_max = 240.0 } = {}) {
  const re = vnorm(r);
  const dm = moon_distance(eph, r, t);
  const te = 2 * Math.PI * Math.sqrt(re ** 3 / MU_EARTH);
  const tm = 2 * Math.PI * Math.sqrt(dm ** 3 / MU_MOON);
  return Math.min(dt_max, eta * Math.min(te, tm));
}

export function _cis_step(r, v, t, dt, eph) {
  const k1v = _cis_accel(r, t, eph), k1r = v;
  const r2 = vadd(r, vscale(k1r, dt / 2)), v2 = vadd(v, vscale(k1v, dt / 2));
  const k2v = _cis_accel(r2, t + dt / 2, eph), k2r = v2;
  const r3 = vadd(r, vscale(k2r, dt / 2)), v3 = vadd(v, vscale(k2v, dt / 2));
  const k3v = _cis_accel(r3, t + dt / 2, eph), k3r = v3;
  const r4 = vadd(r, vscale(k3r, dt)), v4 = vadd(v, vscale(k3v, dt));
  const k4v = _cis_accel(r4, t + dt, eph), k4r = v4;
  const rn = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), dt / 6));
  const vn = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), dt / 6));
  return [rn, vn];
}

export function tli_burn(r, v, t, m0, stage, dv_target, eph, prop_avail,
                         { dt = 0.5, dv_scale = 1.0, point_err = 0.0 } = {}) {
  const vex = G0 * stage.isp_vac;
  const m_cut = m0 * Math.exp(-dv_target * dv_scale / vex);
  const m_dry_limit = m0 - prop_avail;
  let m = m0;
  const md = stage_mdot(stage);
  const ts = [], rs = [], vs = [];
  const t0 = t;
  const sp = Math.sin(point_err), cp = Math.cos(point_err);
  while (m > m_cut && m > m_dry_limit + 1e-9) {
    const step = Math.min(dt, (m - Math.max(m_cut, m_dry_limit)) / md);
    const acc = (rr, vv, mm, tt) => {
      let vhat = vunit(vv);
      if (point_err !== 0.0) {
        const hhat = vunit(vcross(rr, vv));
        vhat = vadd(vscale(vhat, cp), vscale(vcross(hhat, vhat), sp));
      }
      return vadd(_cis_accel(rr, tt, eph), vscale(vhat, stage.thrust_vac / mm));
    };
    const k1r = v, k1v = acc(r, v, m, t);
    const r2 = vadd(r, vscale(k1r, step / 2)), v2 = vadd(v, vscale(k1v, step / 2)), m2 = m - md * step / 2;
    const k2r = v2, k2v = acc(r2, v2, m2, t + step / 2);
    const r3 = vadd(r, vscale(k2r, step / 2)), v3 = vadd(v, vscale(k2v, step / 2));
    const k3r = v3, k3v = acc(r3, v3, m2, t + step / 2);
    const r4 = vadd(r, vscale(k3r, step)), v4 = vadd(v, vscale(k3v, step)), m4 = m - md * step;
    const k4r = v4, k4v = acc(r4, v4, m4, t + step);
    r = vadd(r, vscale(vadd(vadd(k1r, vscale(vadd(k2r, k3r), 2.0)), k4r), step / 6));
    v = vadd(v, vscale(vadd(vadd(k1v, vscale(vadd(k2v, k3v), 2.0)), k4v), step / 6));
    m -= md * step;
    t += step;
    ts.push(t); rs.push(r); vs.push(v);
  }
  const dv_delivered = vex * Math.log(m0 / m);
  return { r, v, t, m, dv_delivered, duration: t - t0, ts, rs, vs };
}

export function fly_cislunar(r0, v0, t0, eph, opts) {
  const { t_ign, dv, stage, m_stack, prop_avail, theta_g0 = 0.0,
          h_stop = 140.0e3, t_max = 30.0 * 86400.0, stop_after_flyby = false,
          dv_scale = 1.0, point_err = 0.0, eta = CIS_ETA, log_every = 4 } = opts;
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

  const burn = tli_burn(r, v, t, m_stack, stage, dv, eph, prop_avail, { dv_scale, point_err });
  r = burn.r; v = burn.v; t = burn.t;
  const m = burn.m, dv_del = burn.dv_delivered, tburn = burn.duration;
  for (let i = 0; i < burn.ts.length; i++)
    cis_push(L, burn.ts[i], burn.rs[i], burn.vs[i], eph, theta_g0, 1);

  const leg = _coast_leg(L, r, v, t, eph, {
    theta_g0, h_stop, t_end: t0 + t_max, stop_after_flyby, log_every, eta,
  });

  return cislunarResult(L, leg.outcome, leg.r, leg.v, leg.t, m, dv_del, t_ign,
                        tburn, leg.peri_alt, leg.t_peri, leg.vac_perigee,
                        leg.gamma_end, leg.miss_passes, leg.first_perigee_alt);
}

export function _coast_leg(L, r, v, t, eph, opts) {
  const { theta_g0, h_stop, t_end, stop_after_flyby, log_every,
          t_stop = Infinity, outbound0 = true,
          peri_alt0 = Infinity, t_peri0 = NaN, vac_perigee0 = NaN,
          miss_passes0 = 0, first_perigee_alt0 = NaN, eta = CIS_ETA } = opts;
  let outbound = outbound0;
  let peri_alt = peri_alt0, t_peri = t_peri0, vac_perigee = vac_perigee0;
  let miss_passes = miss_passes0, first_perigee_alt = first_perigee_alt0;
  let outcome = 'timeout';
  let d_prev = moon_distance(eph, r, t);
  let kount = 0;
  let gamma_end = NaN;
  let rdot_prev = vdot(r, v);
  let a2 = Infinity, a1 = Infinity, tt2 = NaN, tt1 = NaN;

  while (t < t_end) {
    const dtc = Math.min(_cis_dt(r, t, eph, { eta }), Math.max(t_stop - t, 1.0e-3));
    if (kount % log_every === 0 || d_prev < CIS_LOG_NEAR)
      cis_push(L, t, r, v, eph, theta_g0, outbound ? 2 : 3);
    kount += 1;
    const [rn_, vn_] = _cis_step(r, v, t, dtc, eph);
    const tn = t + dtc;

    const dm = moon_distance(eph, rn_, tn);
    const alt_m = dm - R_MOON;
    if (alt_m < peri_alt) { peri_alt = alt_m; t_peri = tn; }
    if (Number.isFinite(a2) && a1 < a2 && a1 <= alt_m) {
      const u = tt2 - tt1, w = tn - tt1;
      const den = u * w * (u - w);
      if (Math.abs(den) > 1e-12) {
        const qa = ((a2 - a1) * w - (alt_m - a1) * u) / den;
        const qb = ((alt_m - a1) * u * u - (a2 - a1) * w * w) / den;
        if (qa > 0.0) {
          const dts = -qb / (2 * qa);
          if (u <= dts && dts <= w) {
            const av = a1 - qb * qb / (4 * qa);
            if (av < peri_alt) { peri_alt = av; t_peri = tt1 + dts; }
          }
        }
      }
    }
    a2 = a1; tt2 = tt1; a1 = alt_m; tt1 = tn;
    if (alt_m <= 0.0) {
      r = rn_; v = vn_; t = tn; outcome = 'lunar_impact';
      cis_push(L, t, r, v, eph, theta_g0, 2);
      break;
    }
    if (outbound && dm > d_prev && dm < 0.35 * A_MOON) outbound = false;
    d_prev = dm;

    if (!outbound && dm > 5.0e7) {
      const el_ = elements_from_state(rn_, vn_);
      vac_perigee = el_.rp - RE_MEAN;
    }
    if (stop_after_flyby && !outbound && dm > 1.2e8) {
      r = rn_; v = vn_; t = tn; outcome = 'flyby_complete';
      cis_push(L, t, r, v, eph, theta_g0, 3);
      break;
    }
    if (tn >= t_stop - 1e-6) {
      r = rn_; v = vn_; t = tn; outcome = 't_stop';
      cis_push(L, t, r, v, eph, theta_g0, outbound ? 2 : 3);
      break;
    }

    const theta = earth_rotation_angle(theta_g0, tn);
    const h = geodetic_from_ecef(rot_z(rn_, theta))[2];
    if (h <= h_stop && vdot(rn_, vn_) < 0) {
      let lo = 0.0, hi = dtc;
      for (let k = 0; k < 40; k++) {
        const mid = 0.5 * (lo + hi);
        const [rm, vm] = _cis_step(r, v, t, mid, eph);
        const th = earth_rotation_angle(theta_g0, t + mid);
        const hm = geodetic_from_ecef(rot_z(rm, th))[2];
        if (hm > h_stop) lo = mid; else hi = mid;
        if (hi - lo < 1e-4) break;
      }
      [r, v] = _cis_step(r, v, t, hi, eph);
      t += hi;
      const el = elements_from_state(r, v);
      vac_perigee = el.rp - RE_MEAN;
      const rhat = vunit(r), vin = vnorm(v);
      gamma_end = Math.asin(Math.min(1, Math.max(-1, vdot(rhat, vscale(v, 1 / vin)))));
      outcome = 'entry_interface';
      cis_push(L, t, r, v, eph, theta_g0, 3);
      break;
    }

    const rdot_now = vdot(rn_, vn_);
    if (!outbound && rdot_prev < 0.0 && rdot_now >= 0.0 && vnorm(rn_) < 0.5 * A_MOON) {
      if (Number.isNaN(first_perigee_alt)) first_perigee_alt = h;
      miss_passes += 1;
    }
    rdot_prev = rdot_now;

    const rn2 = vnorm(rn_);
    if (rn2 > 2.0 * A_MOON) {
      r = rn_; v = vn_; t = tn; outcome = 'escape';
      break;
    }
    r = rn_; v = vn_; t = tn;
  }
  return { r, v, t, outcome, peri_alt, t_peri, vac_perigee, gamma_end, outbound,
           miss_passes, first_perigee_alt };
}

export function seed_free_return(r0, v0, { ra_offset = 60_000.0e3 } = {}) {
  const rp = vnorm(r0);
  const ra = A_MOON + ra_offset;
  const at = 0.5 * (rp + ra);
  const e = (ra - rp) / (ra + rp);
  const p = at * (1 - e * e);
  const nux = Math.acos(Math.min(1, Math.max(-1, (p / A_MOON - 1) / e)));
  let Ex = 2 * Math.atan(Math.sqrt((1 - e) / (1 + e)) * Math.tan(nux / 2));
  if (Ex < 0) Ex += 2 * Math.PI;
  const tf = Math.sqrt(at ** 3 / MU_EARTH) * (Ex - e * Math.sin(Ex));
  const lead = nux - N_MOON * tf;
  const vperi = Math.sqrt(MU_EARTH * (2 / rp - 1 / at));
  const dv = vperi - vnorm(v0);
  return [lead, tf, dv];
}

export function tli_alignment_time(r0, v0, t0, eph, lead) {
  const p = vunit(r0);
  const h = vcross(r0, v0);
  const q = vunit(vcross(h, r0));
  const m = moon_position(eph, t0);
  const phi0 = Math.atan2(vdot(m, q), vdot(m, p));
  const n_sc = Math.sqrt(MU_EARTH / vnorm(r0) ** 3);
  const mod = (x, mm) => x - mm * Math.floor(x / mm);
  let dphi = mod(phi0 - lead, 2 * Math.PI);
  let dt = dphi / (n_sc - N_MOON);
  if (dt < 600.0) dt += 2 * Math.PI / (n_sc - N_MOON);
  return t0 + dt;
}

export function design_free_return(r0, v0, t0, eph, opts) {
  const { stage, m_stack, prop_avail, theta_g0 = 0.0,
          hp_moon_target = 2000.0e3, hp_return_target = 35.0e3,
          perigee_tol = PERIGEE_TOL,
          tol_perilune_km = Math.min(25.0, Math.max(0.05, 0.02 * hp_moon_target / 1e3)),
          perilune_only = hp_moon_target < 10.0e3,
          tol_perigee_km = 2.0, outer_iter = 12, eta = CIS_ETA,
          max_iter = 15, verbose = false } = opts;

  const fly = (tig, dvv) => fly_cislunar(r0, v0, t0, eph, {
    t_ign: tig, dv: dvv, stage, m_stack, prop_avail, theta_g0,
    stop_after_flyby: true, eta, t_max: 10.0 * 86400.0,
  });

  let proxy_target = hp_return_target;
  const resid = (tig_, dvv_) => {
    const res_ = fly(tig_, dvv_);
    const r1 = Number.isFinite(res_.perilune_alt) ? (res_.perilune_alt - hp_moon_target) / 1e3 : 1.0e5;
    const r2 = Number.isNaN(res_.vac_perigee_alt) ? 1.0e5 : (res_.vac_perigee_alt - proxy_target) / 1e3;
    return [r1, r2, res_];
  };

  const [lead, tf_seed, dv_seed] = seed_free_return(r0, v0);
  const t_align = tli_alignment_time(r0, v0, t0, eph, lead);

  const peri_of = tig => fly(tig, dv_seed).perilune_alt;
  const ts = [];
  for (let d = -1500.0; d <= 1500.0 + 1e-9; d += 60.0) ts.push(t_align + d);
  const ps = ts.map(peri_of);
  let ifirst = null;
  for (let i = 1; i < ps.length - 1; i++)
    if (ps[i] < 50_000.0e3 && ps[i] <= ps[i - 1] && ps[i] <= ps[i + 1]) { ifirst = i; break; }
  let imin = 0;
  if (ifirst !== null) imin = ifirst;
  else { let bv = Infinity; for (let i = 0; i < ps.length; i++) if (ps[i] < bv) { bv = ps[i]; imin = i; } }
  const tmin = ts[imin];

  let best = [Infinity, tmin, dv_seed];
  for (let dts = -240.0; dts <= 300.0 + 1e-9; dts += 15.0) {
    const tig = tmin + dts;
    const [f1, f2] = resid(tig, dv_seed);
    const score = Math.hypot(f1, Math.min(Math.abs(f2), 5.0e4));
    if (score < best[0]) best = [score, tig, dv_seed];
  }
  let tig = best[1], dvv = best[2];

  const newton = () => {
    let f1, f2, res;
    for (let it = 1; it <= max_iter; it++) {
      [f1, f2, res] = resid(tig, dvv);
      if (Math.abs(f1) < tol_perilune_km && Math.abs(f2) < tol_perigee_km) return true;
      const d1 = 5.0, d2 = 0.5;
      const [f1a, f2a] = resid(tig + d1, dvv);
      const [f1b, f2b] = resid(tig, dvv + d2);
      const j11 = (f1a - f1) / d1, j21 = (f2a - f2) / d1;
      const j12 = (f1b - f1) / d2, j22 = (f2b - f2) / d2;
      const det = j11 * j22 - j12 * j21;
      if (Math.abs(det) < 1e-14) { tig += 30.0; continue; }
      const dt_ = -(j22 * f1 - j12 * f2) / det;
      const dd_ = -(-j21 * f1 + j11 * f2) / det;
      tig += Math.min(120.0, Math.max(-120.0, 0.7 * dt_));
      dvv += Math.min(10.0, Math.max(-10.0, 0.7 * dd_));
    }
    [f1, f2] = resid(tig, dvv);
    return Math.abs(f1) < tol_perilune_km && Math.abs(f2) < tol_perigee_km;
  };

  const verify = (tig_, dvv_) => fly_cislunar(r0, v0, t0, eph, {
    t_ign: tig_, dv: dvv_, stage, m_stack, prop_avail, theta_g0, eta,
  });

  if (perilune_only) {
    const pa = tg => fly(tg, dvv).perilune_alt;
    let lo = tmin;
    let alo = pa(lo);
    if (alo > hp_moon_target) {
      const full = verify(lo, dvv);
      return [lo, dvv, full, 'unreachable'];
    }
    let hi = lo, ahi = alo;
    for (let k = 1; k <= 14; k++) {
      hi = lo + 30.0 * k;
      ahi = pa(hi);
      if (ahi > hp_moon_target) break;
    }
    let reached = true;
    if (ahi <= hp_moon_target) reached = false;
    else {
      for (let k = 0; k < 22; k++) {
        const mid = 0.5 * (lo + hi);
        const am = pa(mid);
        if (am < hp_moon_target) { lo = mid; alo = am; }
        else { hi = mid; ahi = am; }
        if (Math.abs(ahi - hp_moon_target) < tol_perilune_km * 1e3) break;
      }
    }
    tig = hi;
    const f1 = (ahi - hp_moon_target) / 1e3;
    const full = verify(tig, dvv);
    return [tig, dvv, full,
            !reached ? 'unreachable'
            : Math.abs(f1) < tol_perilune_km ? 'converged' : 'outside_tolerance'];
  }

  let full;
  let stalls = 0;
  best = [Infinity, tig, dvv, null];
  for (let outer = 0; outer < outer_iter; outer++) {
    let converged = newton();
    if (!converged && stalls === 0) { stalls += 1; converged = newton(); }
    full = verify(tig, dvv);
    if (converged && full.outcome === 'entry_interface') {
      stalls = 0;
      const err = full.miss_passes > 0 ? full.first_perigee_alt - hp_return_target
                                       : full.vac_perigee_alt - hp_return_target;
      if (full.miss_passes === 0 && Math.abs(err) < best[0])
        best = [Math.abs(err), tig, dvv, full];
      if (Math.abs(err) < perigee_tol && full.miss_passes === 0)
        return [tig, dvv, full, 'converged'];
      proxy_target -= err;
    } else {
      break;
    }
  }
  if (best[3] !== null) return [best[1], best[2], best[3], 'outside_tolerance'];
  return [tig, dvv, full, 'stalled'];
}

export function design_tcm(r, v, t, eph, opts) {
  const { r_ref, t_ref, hp_moon_target, hp_perigee_proxy, theta_g0 = 0.0,
          verbose = false } = opts;
  const vhat = vunit(v);
  const rhat = vunit(r);
  const nhat = vunit(vsub(rhat, vscale(vhat, vdot(rhat, vhat))));
  const ctrl = (da, dr) => vadd(v, vadd(vscale(vhat, da), vscale(nhat, dr)));

  const pos_err = (da, dr) => {
    const L = cislunarLog();
    const leg = _coast_leg(L, r, ctrl(da, dr), t, eph, {
      theta_g0, h_stop: 0.0, t_end: t_ref + 1.0, stop_after_flyby: false,
      log_every: 1_000_000, t_stop: t_ref,
    });
    const d = vsub(leg.r, r_ref);
    return [vdot(d, vhat) / 1e3, vdot(d, nhat) / 1e3];
  };
  let da = 0.0, dr = 0.0;
  for (let it = 0; it < 8; it++) {
    const [e1, e2] = pos_err(da, dr);
    if (Math.hypot(e1, e2) < 20.0) break;
    const d = 0.5;
    const [e1a, e2a] = pos_err(da + d, dr);
    const [e1b, e2b] = pos_err(da, dr + d);
    const j11 = (e1a - e1) / d, j21 = (e2a - e2) / d;
    const j12 = (e1b - e1) / d, j22 = (e2b - e2) / d;
    const det = j11 * j22 - j12 * j21;
    if (Math.abs(det) < 1e-14) break;
    da -= (j22 * e1 - j12 * e2) / det;
    dr -= (-j21 * e1 + j11 * e2) / det;
  }

  const resid = (da_, dr_) => {
    const L = cislunarLog();
    const leg = _coast_leg(L, r, ctrl(da_, dr_), t, eph, {
      theta_g0, h_stop: 140.0e3, t_end: t + 25.0 * 86400.0,
      stop_after_flyby: true, log_every: 1_000_000,
    });
    const r1 = Number.isFinite(leg.peri_alt) ? (leg.peri_alt - hp_moon_target) / 1e3 : 1.0e5;
    const r2 = Number.isNaN(leg.vac_perigee) ? 1.0e5 : (leg.vac_perigee - hp_perigee_proxy) / 1e3;
    return [r1, r2];
  };
  let f1, f2;
  for (let it = 0; it < 10; it++) {
    [f1, f2] = resid(da, dr);
    if (Math.abs(f1) < 25.0 && Math.abs(f2) < 2.0) break;
    const d = 0.2;
    const [f1a, f2a] = resid(da + d, dr);
    const [f1b, f2b] = resid(da, dr + d);
    const j11 = (f1a - f1) / d, j21 = (f2a - f2) / d;
    const j12 = (f1b - f1) / d, j22 = (f2b - f2) / d;
    const det = j11 * j22 - j12 * j21;
    if (Math.abs(det) < 1e-14) break;
    da += Math.min(10.0, Math.max(-10.0, 0.8 * (-(j22 * f1 - j12 * f2) / det)));
    dr += Math.min(10.0, Math.max(-10.0, 0.8 * (-(-j21 * f1 + j11 * f2) / det)));
  }
  const dv_vec = vadd(vscale(vhat, da), vscale(nhat, dr));
  return [dv_vec, f1, f2];
}

export function fly_cislunar_tcm(r0, v0, t0, eph, opts) {
  const { t_ign, dv, stage, m_stack, prop_avail, r_ref, t_ref,
          dv_scale = 1.0, point_err = 0.0, tcm_delay = 86400.0,
          hp_moon_target = 2000.0e3, hp_perigee_proxy = 35.0e3,
          theta_g0 = 0.0, h_stop = 140.0e3, t_max = 30.0 * 86400.0,
          eta = CIS_ETA, verbose = false } = opts;
  const L = cislunarLog();
  let r = r0, v = v0, t = t0;

  let kount = 0;
  while (t < t_ign) {
    const dtp = Math.min(_cis_dt(r, t, eph, { eta, dt_max: 30.0 }), t_ign - t);
    if (kount % 4 === 0) cis_push(L, t, r, v, eph, theta_g0, 0);
    [r, v] = _cis_step(r, v, t, dtp, eph);
    t += dtp;
    kount += 1;
  }

  const burn = tli_burn(r, v, t, m_stack, stage, dv, eph, prop_avail, { dv_scale, point_err });
  r = burn.r; v = burn.v; t = burn.t;
  const m = burn.m, dv_del = burn.dv_delivered, tburn = burn.duration;
  for (let i = 0; i < burn.ts.length; i++)
    cis_push(L, burn.ts[i], burn.rs[i], burn.vs[i], eph, theta_g0, 1);

  const leg1 = _coast_leg(L, r, v, t, eph, {
    theta_g0, h_stop, t_end: t0 + t_max, stop_after_flyby: false, log_every: 4,
    t_stop: t_ign + tcm_delay,
  });
  if (leg1.outcome !== 't_stop')
    return [cislunarResult(L, leg1.outcome, leg1.r, leg1.v, leg1.t, m, dv_del,
                           t_ign, tburn, leg1.peri_alt, leg1.t_peri,
                           leg1.vac_perigee, leg1.gamma_end, leg1.miss_passes,
                           leg1.first_perigee_alt), NaN];

  const [dv_vec] = design_tcm(leg1.r, leg1.v, leg1.t, eph, {
    r_ref, t_ref, hp_moon_target, hp_perigee_proxy, theta_g0, verbose,
  });
  const tcm_dv = vnorm(dv_vec);
  const v_corr = vadd(leg1.v, dv_vec);
  const m_after = m * Math.exp(-tcm_dv / (G0 * stage.isp_vac));

  const leg2 = _coast_leg(L, leg1.r, v_corr, leg1.t, eph, {
    theta_g0, h_stop, t_end: t0 + t_max, stop_after_flyby: false, log_every: 4,
    outbound0: leg1.outbound, peri_alt0: leg1.peri_alt, t_peri0: leg1.t_peri,
    vac_perigee0: leg1.vac_perigee, miss_passes0: leg1.miss_passes,
    first_perigee_alt0: leg1.first_perigee_alt,
  });

  return [cislunarResult(L, leg2.outcome, leg2.r, leg2.v, leg2.t, m_after,
                         dv_del, t_ign, tburn, leg2.peri_alt, leg2.t_peri,
                         leg2.vac_perigee, leg2.gamma_end, leg2.miss_passes,
                         leg2.first_perigee_alt), tcm_dv];
}
