// Port of src/mission.jl: the full circumlunar mission.
import { MU_EARTH, N_MOON, G0, deg2rad_, rad2deg_ } from './constants.js';
import { vsub } from './vec3.js';
import { default_moon_rocket, stage_diameter, core_diameter, cruise_inertia } from './propulsion.js';
import { ascentGuidance, launch_azimuth, tune_ascent } from './launch.js';
import { coplanar_moon } from './moon.js';
import { seed_free_return, design_free_return, fly_cislunar, fly_cislunar_tcm } from './translunar.js';
import { default_reentry_pod } from './vehicle.js';
import { scenario } from './dynamics.js';
import { simulate } from './simulation.js';
import { sized_kick_rcs, cruise_rcs_budget } from './rcs.js';

const POD_R_COEFF = 0.086;
export const pod_radius = payload_mass =>
  Math.min(3.0, Math.max(0.30, POD_R_COEFF * Math.cbrt(Math.max(Number(payload_mass), 1.0))));

export function translunar_design(lv, opts = {}) {
  const { h_park = 200.0e3, hp_moon = 2000.0e3, hp_return = 50.0e3,
          inclination = deg2rad_(28.5), kick_angle = deg2rad_(8.0),
          optimize_kick = false, cis_eta = 0.002, perigee_tol = 250.0,
          tol_perigee_km = 2.0, theta_g0 = 0.0,
          site_lat = deg2rad_(28.5), site_lon = deg2rad_(-80.6),
          onProgress = null, strict = true, verbose = false } = opts;
  const az = launch_azimuth(inclination, site_lat);
  const guid0 = ascentGuidance({ azimuth: az, h_target: h_park, kick_angle,
                                 site_lat, site_lon });
  const [guid, asc] = tune_ascent(lv, guid0, { optimize_kick, theta_g0, verbose, onProgress });
  if (onProgress) onProgress({ ok: true, stage: 'ascent',
    detail: 'ascent to the parking orbit complete', current: 2, total: 6 });
  const partial = { guid, ascent: asc, eph: null, t_ign: NaN, dv: NaN, cis: null,
                    m_stack: NaN, kick: lv.stages[lv.stages.length - 1],
                    design_status: 'no_design' };
  if (!asc.reached_orbit) {
    if (strict) throw new Error(`ascent failed to reach orbit (h_cut=${asc.h_cut / 1e3} km)`);
    return partial;
  }
  const el0 = asc.elements;
  if (!(el0.rp > 6.3710088e6 + 0.5 * h_park && Math.abs(asc.gamma_cut) < deg2rad_(1.0))) {
    if (strict) throw new Error('ascent reached orbital energy but not the orbit');
    return partial;
  }

  let m_stack = asc.m;
  for (let k = 0; k < lv.stages.length - 1; k++)
    if (asc.prop_left[k] > 0) m_stack -= lv.stages[k].mdry + asc.prop_left[k];

  const el = asc.elements;
  const Tpark = 2 * Math.PI * Math.sqrt(el.a ** 3 / MU_EARTH);
  const n_sc = 2 * Math.PI / Tpark;
  const [lead] = seed_free_return(asc.r, asc.v);
  const t_des = 0.55 * Tpark;
  const phase_at_insertion = lead + (n_sc - N_MOON) * t_des;
  const eph = coplanar_moon(asc.r, asc.v, { phase0: phase_at_insertion - N_MOON * asc.t });

  const kick = lv.stages[lv.stages.length - 1];
  const [t_ign, dv, cis, dstatus] = design_free_return(asc.r, asc.v, asc.t, eph, {
    eta: cis_eta, perigee_tol, theta_g0, stage: kick, m_stack,
    prop_avail: asc.prop_left[asc.prop_left.length - 1],
    hp_moon_target: hp_moon, hp_return_target: hp_return, tol_perigee_km, verbose,
  });
  if (onProgress) onProgress({ ok: true, stage: 'trajectory',
    detail: 'trans-lunar trajectory designed', current: 3, total: 6 });
  return { guid, ascent: asc, eph, t_ign, dv, cis, m_stack, kick, design_status: dstatus };
}

export function moonshot(opts = {}) {
  const { pod_mass: pod_mass0 = 350.0, h_park = 200.0e3, hp_moon = 2000.0e3,
          hp_return = 50.0e3, inclination = deg2rad_(28.5), lv: lvIn = null,
          tli_mag_err = 0.0, tli_point_err = 0.0, tcm_delay = 86400.0,
          kick_angle = deg2rad_(8.0), optimize_kick = false, theta_g0 = 0.0,
          cis_eta = 0.002, perigee_tol = 250.0, strict = true,
          site_lat = deg2rad_(28.5), site_lon = deg2rad_(-80.6),
          pod_diameter = NaN, onProgress = null, verbose = false } = opts;
  let lv = lvIn;
  if (lv === null) lv = default_moon_rocket({ payload: pod_mass0 });
  const pod_mass = lv.payload_mass;
  const pod_d = Number.isFinite(pod_diameter) && pod_diameter > 0
    ? pod_diameter : 2 * pod_radius(pod_mass);

  const des = translunar_design(lv, { h_park, hp_moon, hp_return, inclination,
    kick_angle, optimize_kick, cis_eta, perigee_tol, theta_g0,
    site_lat, site_lon, onProgress, strict, verbose });
  const { guid, ascent: asc, eph } = des;
  const dstatus = des.design_status;
  const result = { lv, guid, ascent: asc, eph, cislunar: null, entry_scn: null,
                   entry: null, cruise: null, design_status: dstatus };
  if (des.cis === null) return result;
  let cis = des.cis;
  const m_stack = des.m_stack, kick = des.kick;
  if (cis.outcome !== 'entry_interface') {
    if (strict) throw new Error(`free-return design did not come home (outcome: ${cis.outcome})`);
    return result;
  }
  if (dstatus === 'stalled' || dstatus === 'unreachable') {
    if (strict) throw new Error(`free-return targeting did not converge (status: ${dstatus})`);
    return result;
  }

  let cruise = null;
  if (tli_mag_err !== 0.0 || tli_point_err !== 0.0) {
    const { t_ign, dv } = des;
    const nomfly = fly_cislunar(asc.r, asc.v, asc.t, eph, {
      eta: cis_eta, theta_g0, t_ign, dv, stage: kick, m_stack,
      prop_avail: asc.prop_left[asc.prop_left.length - 1],
      stop_after_flyby: true, t_max: 10.0 * 86400.0 });
    const proxy = nomfly.vac_perigee_alt;
    const refleg = fly_cislunar(asc.r, asc.v, asc.t, eph, {
      eta: cis_eta, theta_g0, t_ign, dv, stage: kick, m_stack,
      prop_avail: asc.prop_left[asc.prop_left.length - 1],
      t_max: nomfly.t_perilune - asc.t });
    const [cis_d, tcm_dv] = fly_cislunar_tcm(asc.r, asc.v, asc.t, eph, {
      eta: cis_eta, theta_g0, t_ign, dv, stage: kick, m_stack,
      prop_avail: asc.prop_left[asc.prop_left.length - 1],
      r_ref: refleg.r, t_ref: refleg.t,
      dv_scale: 1.0 + tli_mag_err, point_err: tli_point_err, tcm_delay,
      hp_moon_target: hp_moon, hp_perigee_proxy: proxy, verbose });
    if (cis_d.outcome !== 'entry_interface') {
      if (strict) throw new Error(`dispersed cruise did not come home (outcome: ${cis_d.outcome})`);
      return result;
    }
    const tcm_prop = cis_d.m * (Math.exp(tcm_dv / (G0 * kick.isp_vac)) - 1);
    const [, I_t] = cruise_inertia(lv, cis_d.m, pod_d, { payload_mass: pod_mass });
    const rcs_sys = sized_kick_rcs(I_t, stage_diameter(kick, core_diameter(lv)) / 2, cis_d.m);
    const rcs_budget = cruise_rcs_budget(rcs_sys, I_t, {
      duration: cis_d.t - cis_d.t_tli, t_tli: cis_d.t_tli,
      t_events: [cis_d.t_tli + tcm_delay] });
    cruise = { tli_mag_err, tli_point_err, tcm_dv, tcm_time: cis_d.t_tli + tcm_delay,
               tcm_prop, rcs: rcs_budget };
    cis = cis_d;
  }

  const pod = default_reentry_pod({ mass: pod_mass, diameter: pod_d });
  const scn = scenario({ vehicle: pod, r0: cis.r, v0: cis.v, t0: cis.t,
                         t_max: cis.t + 3.0e4, theta_g0, alpha0: deg2rad_(5.0) });
  if (onProgress) onProgress({ ok: true, stage: 'entry',
    detail: 'propagating atmospheric entry', current: 5, total: 6 });
  const entry = simulate(scn);
  return { lv, guid, ascent: asc, eph, cislunar: cis, entry_scn: scn, entry, cruise,
           design_status: dstatus };
}
