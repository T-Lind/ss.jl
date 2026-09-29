// Port of src/dynamics.jl: the 4-DOF entry equations, the Scenario/FlightContext
// containers they read, and flight_data for logging.
import { OMEGA_EARTH, G0, deg2rad_ } from './constants.js';
import { vadd, vsub, vscale, vdot, vcross, vnorm, vunit } from './vec3.js';
import { USSA76, atmosphere_state } from './atmosphere.js';
import { j2Gravity, gravity_accel } from './gravity.js';
import { earth_rotation_angle, geodetic_from_ecef, enu_basis } from './frames.js';
import { rot_z } from './vec3.js';
import { cd_coeff, cl_coeff, cm_coeff } from './aerodynamics.js';
import { chute_fill } from './vehicle.js';
import { heating_convective, heating_radiative, wall_temperature } from './heating.js';

export const COSGAMMA_MIN = 0.05;
const OMEGA = [0.0, 0.0, OMEGA_EARTH];

export const flightContext = nchutes => ({
  chute_deploy_t: new Array(nchutes).fill(NaN),
  entered: false,
});
export const any_chute_deployed = ctx => ctx.chute_deploy_t.some(t => !Number.isNaN(t));

export function scenario(o) {
  return {
    vehicle: o.vehicle,
    atmosphere: o.atmosphere ?? USSA76,
    gravity: o.gravity ?? j2Gravity(),
    r0: o.r0,
    v0: o.v0,
    alpha0: o.alpha0 ?? deg2rad_(5.0),
    t0: o.t0 ?? 0.0,
    theta_g0: o.theta_g0 ?? 0.0,
    h_ei: o.h_ei ?? 120.0e3,
    bank: o.bank ?? 0.0,
    extra_accel: o.extra_accel ?? (() => [0.0, 0.0, 0.0]),
    target_lat: o.target_lat ?? NaN,
    target_lon: o.target_lon ?? NaN,
    dt_orbit: o.dt_orbit ?? 1.0,
    dt_entry: o.dt_entry ?? 0.05,
    dt_descent: o.dt_descent ?? 0.2,
    t_max: o.t_max ?? 3.0e4,
  };
}

export const initial_state = s =>
  [s.r0[0], s.r0[1], s.r0[2], s.v0[0], s.v0[1], s.v0[2], s.alpha0, 0.0, 0.0];

const bank_command = (b, t, h, v, gl) =>
  typeof b === 'function' ? Number(b(t, h, v, gl)) : Number(b);

export function gload_bank(g_target, { bank_max = deg2rad_(150.0), kp = 1.0 } = {}) {
  return (t, h, v, gl) =>
    bank_max * Math.min(1.0, Math.max(0.0, kp * (g_target - gl) / g_target));
}

export function dynamics(dx, x, scn, ctx, t) {
  const r = [x[0], x[1], x[2]];
  const v = [x[3], x[4], x[5]];
  const alpha = x[6];
  const qrate = x[7];
  const veh = scn.vehicle;
  const theta = earth_rotation_angle(scn.theta_g0, t);
  const [, , h] = geodetic_from_ecef(rot_z(r, theta));

  let a = gravity_accel(scn.gravity, r, t);
  a = vadd(a, scn.extra_accel(r, v, t));

  let dalpha = 0.0, dq = 0.0, qdot_heat = 0.0;

  if (h < scn.h_ei) {
    const [rho, , , asnd] = atmosphere_state(scn.atmosphere, h);
    const vrel = vsub(v, vcross(OMEGA, r));
    const Vr = vnorm(vrel);
    if (rho > 0 && Vr > 1.0) {
      const qbar = 0.5 * rho * Vr * Vr;
      const M = Vr / asnd;
      const vhat = vscale(vrel, 1 / Vr);
      const rhat = vunit(r);
      const singam = Math.min(1, Math.max(-1, vdot(rhat, vhat)));
      const cosgam = Math.sqrt(Math.max(0.0, 1 - singam * singam));

      let cda_chutes = 0.0;
      for (let i = 0; i < veh.chutes.length; i++) {
        const td = ctx.chute_deploy_t[i];
        if (!Number.isNaN(td)) cda_chutes += veh.chutes[i].cda * chute_fill(veh.chutes[i], t - td);
      }
      const chutes_out = any_chute_deployed(ctx);

      const CD = cd_coeff(veh.aero, M, alpha);
      const D = qbar * (veh.sref * CD + cda_chutes);
      let L = chutes_out ? 0.0 : qbar * veh.sref * cl_coeff(veh.aero, M, alpha);
      let f_aero = vscale(vhat, -D);
      if (L !== 0.0 && cosgam > COSGAMMA_MIN) {
        const uhat = vunit(vsub(rhat, vscale(vhat, singam)));
        const shat = vcross(vhat, uhat);
        const gl_now = Math.sqrt(D * D + L * L) / (veh.mass * G0);
        const bk = bank_command(scn.bank, t, h, Vr, gl_now);
        const lhat = vadd(vscale(uhat, Math.cos(bk)), vscale(shat, Math.sin(bk)));
        f_aero = vadd(f_aero, vscale(lhat, L));
      }
      a = vadd(a, vscale(f_aero, 1 / veh.mass));

      const arel = vsub(a, vcross(OMEGA, v));
      const rn_ = vnorm(r);
      const drhat = vscale(vsub(v, vscale(rhat, vdot(v, rhat))), 1 / rn_);
      const dvhat = vscale(vsub(arel, vscale(vhat, vdot(arel, vhat))), 1 / Vr);
      const gamdot = (vdot(drhat, vhat) + vdot(rhat, dvhat)) /
                     Math.max(cosgam, COSGAMMA_MIN);

      if (!chutes_out && cosgam > COSGAMMA_MIN) {
        const qhat = qrate * veh.lref / (2 * Vr);
        const Cm = cm_coeff(veh.aero, M, alpha, qhat);
        dq = qbar * veh.sref * veh.lref * Cm / veh.iyy;
        dalpha = qrate - gamdot;
      } else {
        dalpha = -0.2 * alpha;
        dq = -0.5 * qrate;
      }

      qdot_heat = heating_convective(rho, Vr, veh.rn) +
                  heating_radiative(rho, Vr, veh.rn);
    }
  }

  dx[0] = v[0]; dx[1] = v[1]; dx[2] = v[2];
  dx[3] = a[0]; dx[4] = a[1]; dx[5] = a[2];
  dx[6] = dalpha;
  dx[7] = dq;
  dx[8] = qdot_heat;
}

export function flight_data(x, scn, ctx, t) {
  const r = [x[0], x[1], x[2]];
  const v = [x[3], x[4], x[5]];
  const veh = scn.vehicle;
  const theta = earth_rotation_angle(scn.theta_g0, t);
  const [lat, lon, h] = geodetic_from_ecef(rot_z(r, theta));

  const vrel = vsub(v, vcross(OMEGA, r));
  const Vr = vnorm(vrel);
  const [rho, T, , asnd] = h < 600e3
    ? atmosphere_state(scn.atmosphere, h)
    : [0.0, 0.0, 0.0, 300.0];
  const M = Vr / asnd;
  const qbar = 0.5 * rho * Vr * Vr;

  const rhat = vunit(r);
  const vhat = Vr > 1 ? vscale(vrel, 1 / Vr) : [1.0, 0.0, 0.0];
  const gamma = Math.asin(Math.min(1, Math.max(-1, vdot(rhat, vhat))));

  const vrel_ecef = rot_z(vrel, theta);
  const [eE, eN] = enu_basis(lat, lon);
  const psi = Math.atan2(vdot(vrel_ecef, eE), vdot(vrel_ecef, eN));

  let gload = 0.0, qdot_c = 0.0, qdot_r = 0.0;
  if (h < scn.h_ei && rho > 0 && Vr > 1) {
    let cda_chutes = 0.0;
    for (let i = 0; i < veh.chutes.length; i++) {
      const td = ctx.chute_deploy_t[i];
      if (!Number.isNaN(td)) cda_chutes += veh.chutes[i].cda * chute_fill(veh.chutes[i], t - td);
    }
    const CD = cd_coeff(veh.aero, M, x[6]);
    const D = qbar * (veh.sref * CD + cda_chutes);
    const L = any_chute_deployed(ctx) ? 0.0 : qbar * veh.sref * cl_coeff(veh.aero, M, x[6]);
    gload = Math.hypot(D, L) / (veh.mass * G0);
    qdot_c = heating_convective(rho, Vr, veh.rn);
    qdot_r = heating_radiative(rho, Vr, veh.rn);
  }

  return {
    t, h, lat, lon, vin: vnorm(v), vrel: Vr, gamma, psi, mach: M, qbar, gload,
    alpha: x[6], qrate: x[7], rho, T,
    qdot_conv: qdot_c, qdot_rad: qdot_r, qload: x[8],
    twall: wall_temperature(qdot_c + qdot_r, veh.emissivity),
  };
}
