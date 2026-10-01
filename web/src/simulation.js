// Port of src/simulation.jl: RK4 driver, event detection and logging.
import { dynamics, flight_data, initial_state, any_chute_deployed } from './dynamics.js';
import { earth_rotation_angle, geodetic_from_ecef, haversine } from './frames.js';
import { rot_z } from './vec3.js';

const rk4Work = n => ({
  k1: new Float64Array(n), k2: new Float64Array(n),
  k3: new Float64Array(n), k4: new Float64Array(n), xt: new Float64Array(n),
});

export function rk4Step(xout, x, t, dt, w, scn, ctx) {
  const n = x.length;
  dynamics(w.k1, x, scn, ctx, t);
  for (let i = 0; i < n; i++) w.xt[i] = x[i] + 0.5 * dt * w.k1[i];
  dynamics(w.k2, w.xt, scn, ctx, t + 0.5 * dt);
  for (let i = 0; i < n; i++) w.xt[i] = x[i] + 0.5 * dt * w.k2[i];
  dynamics(w.k3, w.xt, scn, ctx, t + 0.5 * dt);
  for (let i = 0; i < n; i++) w.xt[i] = x[i] + dt * w.k3[i];
  dynamics(w.k4, w.xt, scn, ctx, t + dt);
  for (let i = 0; i < n; i++)
    xout[i] = x[i] + (dt / 6) * (w.k1[i] + 2 * w.k2[i] + 2 * w.k3[i] + w.k4[i]);
}

const newLog = () => {
  const o = {};
  for (const k of ['t', 'h', 'lat', 'lon', 'vin', 'vrel', 'gamma', 'psi',
    'mach', 'qbar', 'gload', 'alpha', 'qrate', 'rho', 'qdot_conv',
    'qdot_rad', 'qload', 'twall']) o[k] = [];
  return o;
};

function pushLog(L, d) {
  L.t.push(d.t); L.h.push(d.h); L.lat.push(d.lat); L.lon.push(d.lon);
  L.vin.push(d.vin); L.vrel.push(d.vrel); L.gamma.push(d.gamma); L.psi.push(d.psi);
  L.mach.push(d.mach); L.qbar.push(d.qbar); L.gload.push(d.gload); L.alpha.push(d.alpha);
  L.qrate.push(d.qrate); L.rho.push(d.rho);
  L.qdot_conv.push(d.qdot_conv); L.qdot_rad.push(d.qdot_rad);
  L.qload.push(d.qload); L.twall.push(d.twall);
}

function altitude(x, scn, t) {
  const theta = earth_rotation_angle(scn.theta_g0, t);
  return geodetic_from_ecef(rot_z([x[0], x[1], x[2]], theta))[2];
}

export function simulate(scn, { log_dt_orbit = 5.0, log_dt_entry = 0.5 } = {}) {
  if (![...scn.r0, ...scn.v0, scn.t0, scn.t_max, scn.h_ei, scn.alpha0, scn.theta_g0].every(Number.isFinite)
      || Math.hypot(...scn.r0) === 0 || scn.t_max < scn.t0)
    throw new RangeError('simulation needs finite state and times, with t_max >= t0');
  if (![scn.dt_orbit, scn.dt_entry, scn.dt_descent].every(dt => Number.isFinite(dt) && dt > 0))
    throw new RangeError('integration steps must be finite and positive');
  if (![log_dt_orbit, log_dt_entry].every(dt => dt > 0))
    throw new RangeError('logging intervals must be positive');
  const x = Float64Array.from(initial_state(scn));
  const xnew = new Float64Array(x.length);
  const w = rk4Work(x.length);
  const ctx = { chute_deploy_t: new Array(scn.vehicle.chutes.length).fill(NaN), entered: false };
  const events = [];
  const L = newLog();

  let t = scn.t0;
  let h = altitude(x, scn, t);
  ctx.entered = h < scn.h_ei;
  let next_log = t;

  let peak_g = 0.0, peak_qdot = 0.0, peak_qbar = 0.0;
  let terminated = 'timeout';
  let t_sp = NaN, lat_sp = NaN, lon_sp = NaN, v_sp = NaN;

  const record = tnow => { const d = flight_data(x, scn, ctx, tnow); pushLog(L, d); return d; };
  const event = (name, tnow) => {
    const d = flight_data(x, scn, ctx, tnow);
    events.push({ name, t: tnow, h: d.h, mach: d.mach, vrel: d.vrel, lat: d.lat, lon: d.lon });
    return d;
  };

  const initial = record(t);
  peak_g = initial.gload;
  peak_qdot = initial.qdot_conv + initial.qdot_rad;
  peak_qbar = ctx.entered ? initial.qbar : 0;

  while (t < scn.t_max) {
    const chutes_out = any_chute_deployed(ctx);
    const dt = Math.min(!ctx.entered ? scn.dt_orbit : (chutes_out ? scn.dt_descent : scn.dt_entry), scn.t_max - t);
    if (!(t + dt > t)) throw new RangeError('integration step cannot advance time at this epoch');

    rk4Step(xnew, x, t, dt, w, scn, ctx);
    if (!xnew.every(Number.isFinite)) throw new Error(`non-finite reentry state at t=${t + dt}`);
    const hnew = altitude(xnew, scn, t + dt);

    if (!ctx.entered && hnew < scn.h_ei) {
      let lo = 0.0, hi = dt;
      for (let k = 0; k < 30; k++) {
        const mid = 0.5 * (lo + hi);
        rk4Step(xnew, x, t, mid, w, scn, ctx);
        if (altitude(xnew, scn, t + mid) > scn.h_ei) lo = mid; else hi = mid;
        if (hi - lo < 1e-4) break;
      }
      rk4Step(xnew, x, t, hi, w, scn, ctx);
      x.set(xnew); t += hi;
      ctx.entered = true;
      event('entry_interface', t);
      record(t); next_log = t + log_dt_entry;
      continue;
    }

    if (hnew <= 0.0) {
      let lo = 0.0, hi = dt;
      for (let k = 0; k < 40; k++) {
        const mid = 0.5 * (lo + hi);
        rk4Step(xnew, x, t, mid, w, scn, ctx);
        if (altitude(xnew, scn, t + mid) > 0.0) lo = mid; else hi = mid;
        if (hi - lo < 1e-5) break;
      }
      rk4Step(xnew, x, t, hi, w, scn, ctx);
      x.set(xnew); t += hi;
      const d = event('splashdown', t);
      record(t);
      terminated = 'splashdown';
      t_sp = t; lat_sp = d.lat; lon_sp = d.lon; v_sp = d.vrel;
      break;
    }

    x.set(xnew); t += dt;

    if (ctx.entered) {
      const d = flight_data(x, scn, ctx, t);
      peak_g = Math.max(peak_g, d.gload);
      peak_qdot = Math.max(peak_qdot, d.qdot_conv + d.qdot_rad);
      peak_qbar = Math.max(peak_qbar, d.qbar);
      for (let i = 0; i < scn.vehicle.chutes.length; i++) {
        const c = scn.vehicle.chutes[i];
        if (Number.isNaN(ctx.chute_deploy_t[i]) && d.mach < c.mach_max && d.h < c.alt_max) {
          ctx.chute_deploy_t[i] = t;
          events.push({ name: `deploy_${c.name}`, t, h: d.h, mach: d.mach, vrel: d.vrel, lat: d.lat, lon: d.lon });
        }
      }
      if (t >= next_log) { pushLog(L, d); next_log += log_dt_entry; }
    } else if (t >= next_log) {
      record(t);
      next_log += log_dt_orbit;
    }
  }

  if (terminated === 'timeout' && L.t.at(-1) !== t) record(t);

  const miss = (!Number.isNaN(scn.target_lat) && terminated === 'splashdown')
    ? haversine(lat_sp, lon_sp, scn.target_lat, scn.target_lon) / 1000
    : NaN;

  return {
    log: L, events, t_splash: t_sp, lat_splash: lat_sp, lon_splash: lon_sp,
    v_splash: v_sp, miss_km: miss, peak_gload: peak_g, peak_qdot,
    peak_qbar, heat_load: x[8], terminated,
  };
}
