// Port of src/rcs.jl: thruster hardware, couples, and the analytic budgets.
import { G0, MU_EARTH, RE_MEAN, deg2rad_ } from './constants.js';
import { vcross, vscale } from './vec3.js';
import { atmosphere_state } from './atmosphere.js';

const thruster = (pos, dir, thrust) => ({ pos, dir, thrust });
export const rcSystem = ({ thrusters, isp = 220.0, prop = 5.0, mib = 0.02 }) =>
  ({ thrusters, isp, prop, mib });

const thruster_torque = t => vcross(t.pos, vscale(t.dir, t.thrust));

export function torque_authority(sys) {
  let ax = 0, ay = 0, az = 0;
  for (const t of sys.thrusters) {
    const m = thruster_torque(t);
    ax += Math.max(m[0], 0.0); ay += Math.max(m[1], 0.0); az += Math.max(m[2], 0.0);
  }
  return [ax, ay, az];
}

export const rcs_mdot = (sys, n = 1) => n * sys.thrusters[0].thrust / (G0 * sys.isp);

export function limit_cycle_prop(I, torque, isp, t_mib, theta_db, duration,
                                 { nthr = 2, thrust = torque } = {}) {
  const dw = torque * t_mib / I;
  const w0 = dw / 2;
  const t_coast = 2 * theta_db / w0;
  const cycles = duration / t_coast;
  const mdot = nthr * thrust / (G0 * isp);
  return cycles * mdot * t_mib;
}

export function slew_prop(I, torque, isp, angle, { nthr = 2, thrust = torque } = {}) {
  const t_half = Math.sqrt(angle * I / torque);
  const mdot = nthr * thrust / (G0 * isp);
  return [mdot * 2 * t_half, 2 * t_half];
}

function couple_pair(axis, arm, F) {
  if (axis === 1)
    return [thruster([0.0, arm, 0.0], [0.0, 0.0, 1.0], F),
            thruster([0.0, -arm, 0.0], [0.0, 0.0, -1.0], F)];
  if (axis === 2)
    return [thruster([arm, 0.0, 0.0], [0.0, 0.0, 1.0], F),
            thruster([-arm, 0.0, 0.0], [0.0, 0.0, -1.0], F)];
  return [thruster([arm, 0.0, 0.0], [0.0, 1.0, 0.0], F),
          thruster([-arm, 0.0, 0.0], [0.0, -1.0, 0.0], F)];
}

const flip_pair = p => [thruster(p[0].pos, vscale(p[0].dir, -1.0), p[0].thrust),
                        thruster(p[1].pos, vscale(p[1].dir, -1.0), p[1].thrust)];

export function three_axis_set(arm, F) {
  const ths = [];
  for (let ax = 1; ax <= 3; ax++) {
    const p = couple_pair(ax, arm, F);
    ths.push(p[0], p[1]);
    const m = flip_pair(p);
    ths.push(m[0], m[1]);
  }
  return ths;
}

export const default_pod_rcs = () => rcSystem({ thrusters: three_axis_set(0.65, 20.0), isp: 220.0, prop: 6.0, mib: 0.02 });
export const default_kick_rcs = () => rcSystem({ thrusters: three_axis_set(1.4, 10.0), isp: 220.0, prop: 12.0, mib: 0.01 });

export const SLEW_180_S = 120.0;

export function sized_kick_rcs(I_t, radius, m_stack) {
  const t_half = SLEW_180_S / 2;
  const T = Math.PI * Math.max(I_t, 1.0) / t_half ** 2;
  const arm = Math.max(radius, 0.3);
  const F = Math.max(T / (2 * arm), 5.0);
  return rcSystem({ thrusters: three_axis_set(arm, F), isp: 220.0,
                    prop: Math.max(0.01 * m_stack, 4.0), mib: 0.01 });
}

export const gravity_gradient_torque = (mu, r, dI) => 1.5 * mu * Math.abs(dI) / r ** 3;
export const aero_torque = (rho, v, cd, area, arm) => 0.5 * rho * v * v * cd * area * arm;

export const momentum_dump_prop = (sys, torque, t_dist, duration) =>
  rcs_mdot(sys, 2) * Math.min(1.0, Math.max(0.0, t_dist / Math.max(torque, 1e-9))) * duration;

// IEEE-754 next float toward -inf, to mirror Julia's prevfloat.
function prevfloat(x) {
  if (!Number.isFinite(x) || x === -Infinity) return x;
  if (x === 0) return -Number.MIN_VALUE;
  const buf = new ArrayBuffer(8);
  const f = new Float64Array(buf), u = new BigUint64Array(buf);
  f[0] = x;
  u[0] = x > 0 ? u[0] - 1n : u[0] + 1n;
  return f[0];
}

export function rcs_budget(sys, I_t, opts) {
  const { duration, t0 = 0.0, t_burns = [], slew_angle = Math.PI,
          theta_db = deg2rad_(5.0), settling_s = 10.0, disturbance_torque = 0.0,
          capacity = sys.prop } = opts;
  const dur = Math.max(duration, 0.0);
  const T = Math.max(torque_authority(sys)[2], 1e-9);
  const F = sys.thrusters[0].thrust;
  const mdot = rcs_mdot(sys, 2);

  const lc = 2 * limit_cycle_prop(I_t, T, sys.isp, sys.mib, theta_db, dur, { nthr: 2, thrust: F });
  const dump = momentum_dump_prop(sys, T, disturbance_torque, dur);
  const hold = Math.max(lc, dump);
  const hold_rate = dur > 0 ? hold / dur : 0.0;

  const burns = t_burns.filter(t => t0 <= t && t <= t0 + dur).map(Number).sort((a, b) => a - b);
  const [p_slew, t_slew] = slew_prop(I_t, T, sys.isp, slew_angle, { nthr: 2, thrust: F });
  const p_settle = mdot * settling_s;
  const steps = [];
  steps.push([t0, p_slew, 'post-burn turnaround']);
  for (const tb of burns) {
    steps.push([Math.max(tb - t_slew - settling_s, t0), p_slew, 'slew to burn attitude']);
    steps.push([Math.max(tb - settling_s, t0), p_settle, 'ullage settling']);
  }
  steps.push([t0 + dur, p_slew, 'attitude for the next event']);
  steps.sort((a, b) => a[0] - b[0]);

  const sl = p_slew * (burns.length + 2);
  const settle = p_settle * burns.length;
  const total = hold + sl + settle;

  const grid = [];
  const N = 121;
  for (let i = 0; i < N; i++) grid.push(t0 + (t0 + dur - t0) * i / (N - 1));
  const ts = [];
  for (const [te] of steps) { ts.push(prevfloat(te), te); }
  const times = [...new Set([...grid, ...ts])].sort((a, b) => a - b);
  const used = times.map(t => {
    let u = hold_rate * (t - t0);
    for (const [te, kg] of steps) if (te <= t) u += kg;
    return u;
  });

  return { limit_cycle: lc, dump, hold, slews: sl, settling: settle, total,
           margin: capacity - total, capacity, n_slews: burns.length + 2,
           slew_time: t_slew, disturbance_torque, t: times, used,
           events: steps.map(([t, kg, what]) => ({ t, kg, what })) };
}

export const cruise_rcs_budget = (sys, I_t, opts) => {
  const { duration, t_tli = 0.0, t_events = [], ...rest } = opts;
  return rcs_budget(sys, I_t, { duration, t0: t_tli, t_burns: t_events, ...rest });
};

export function orbit_rcs_budget(sys, I_t, I_roll, opts) {
  const { duration, alt, v, area, body_length, cp_offset = 0.08,
          atmosphere = null, slew_angle = 0.5 * Math.PI, ...rest } = opts;
  const rho = atmosphere === null ? 0.0 : atmosphere_state(atmosphere, alt)[0];
  const r = RE_MEAN + Math.max(alt, 0.0);
  const t_gg = gravity_gradient_torque(MU_EARTH, r, I_t - I_roll);
  const t_aero = aero_torque(rho, v, 2.2, area, cp_offset * body_length);
  return rcs_budget(sys, I_t, { duration, slew_angle, disturbance_torque: t_gg + t_aero, ...rest });
}
