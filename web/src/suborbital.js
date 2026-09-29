// Port of src/suborbital.jl: suborbital hops and ballistic shots, flown as an
// ascent with a target it can close on, a ballistic arc, and the same entry.
import { RE_MEAN, deg2rad_ } from './constants.js';
import { default_moon_rocket } from './propulsion.js';
import { ascentGuidance, simulate_ascent } from './launch.js';
import { default_reentry_pod } from './vehicle.js';
import { scenario } from './dynamics.js';
import { simulate } from './simulation.js';

export const suborbitalResult = (o = {}) => ({
  lv: o.lv, guid: o.guid, ascent: o.ascent, profile: o.profile,
  target_apogee: o.target_apogee ?? 0.0, target_range: o.target_range ?? 0.0,
  apogee: o.apogee ?? 0.0, range: o.range ?? 0.0, t_apogee: o.t_apogee ?? 0.0,
  outcome: o.outcome ?? 'short', entry_scn: o.entry_scn ?? null,
  entry: o.entry ?? null,
});

// Apogee altitude [m] and the time it happens, from a flown entry log.
function _arc_apogee(res) {
  if (res.log.t.length === 0) return [NaN, NaN];
  let i = 0;
  for (let k = 1; k < res.log.h.length; k++) if (res.log.h[k] > res.log.h[i]) i = k;
  return [res.log.h[i], res.log.t[i]];
}

// Great-circle ground range [m] from the launch site to the splashdown point.
function _ground_range(lat0, lon0, lat1, lon1) {
  if (Number.isNaN(lat1) || Number.isNaN(lon1)) return NaN;
  const d = Math.sin(lat0) * Math.sin(lat1) +
            Math.cos(lat0) * Math.cos(lat1) * Math.cos(lon1 - lon0);
  return RE_MEAN * Math.acos(Math.min(1, Math.max(-1, d)));
}

// Fly one suborbital attempt at a given commanded target, without correction.
function _subfly(lv, guid, pod_mass, theta_g0) {
  const asc = simulate_ascent(lv, guid, { t_max: 2.0e3 });
  const cut = asc.events.some(e => e.name === 'seco');
  const pod = default_reentry_pod({ mass: pod_mass });
  const scn = scenario({ vehicle: pod, r0: asc.r, v0: asc.v, t0: asc.t,
                         t_max: asc.t + 6.0e3, theta_g0, alpha0: deg2rad_(2.0) });
  const arc = simulate(scn);
  const [hap, tap] = _arc_apogee(arc);
  const rng = _ground_range(guid.site_lat, guid.site_lon, arc.lat_splash, arc.lon_splash);
  return { asc, scn, arc, apogee: hap, t_apogee: tap, range: rng, cut };
}

export function suborbital(opts = {}) {
  const { profile = 'hop', lv: lvIn = null, pod_mass: pod_mass0 = 350.0,
          apogee = 100.0e3, downrange = 250.0e3, loft = deg2rad_(40.0),
          azimuth = deg2rad_(90.0), kick_angle = deg2rad_(8.0),
          theta_g0 = 0.0, iterations = 3, strict = true, verbose = false } = opts;
  if (!(profile === 'hop' || profile === 'downrange'))
    throw new Error(`unknown suborbital profile ${profile}; have hop, downrange`);
  if (!(apogee > 0)) throw new Error('apogee must be positive');
  if (profile === 'downrange' && !(downrange > 0))
    throw new Error('downrange must be positive');

  let lv = lvIn;
  if (lv === null) lv = default_moon_rocket({ payload: pod_mass0 });
  const pod_mass = lv.payload_mass;

  const base = profile === 'hop'
    ? ascentGuidance({ azimuth, kick_angle: 0.0, kick_duration: 0.0,
                       pitch_hold: 0.5 * Math.PI, cutoff: 'apogee',
                       apogee_target: apogee,
                       fairing_alt: Math.min(60.0e3, 0.55 * apogee) })
    : ascentGuidance({ azimuth, kick_angle, pitch_hold: loft, cutoff: 'range',
                       range_target: downrange, fairing_alt: 60.0e3 });

  const goal = profile === 'hop' ? apogee : downrange;
  const got = f => profile === 'hop' ? f.apogee : f.range;
  let cmd = goal;
  let best = null;
  let prev_cmd = NaN, prev_err = NaN;
  for (let it = 1; it <= Math.max(iterations, 1); it++) {
    const g = profile === 'hop'
      ? ascentGuidance({ ...base, apogee_target: cmd })
      : ascentGuidance({ ...base, range_target: cmd });
    const f = _subfly(lv, g, pod_mass, theta_g0);
    const err = Number.isFinite(got(f)) ? got(f) - goal : NaN;
    if (best === null ||
        (Number.isFinite(err) && (!Number.isFinite(best.err) || Math.abs(err) < Math.abs(best.err))))
      best = { g, f, err };
    if (!f.cut || !Number.isFinite(err)) break;
    if (Math.abs(err) < 0.004 * goal) break;
    const cmd_new = (Number.isFinite(prev_err) && Math.abs(err - prev_err) > 1e-6)
      ? cmd - err * (cmd - prev_cmd) / (err - prev_err)
      : cmd - err;
    prev_cmd = cmd; prev_err = err;
    cmd = Math.min(4.0 * goal, Math.max(0.25 * goal, cmd_new));
  }

  const g = best.g, f = best.f;
  const outcome = !f.cut ? 'short' : Number.isNaN(f.range) ? 'timeout' : 'splashdown';
  if (outcome === 'short' && strict)
    throw new Error('the vehicle could not reach its suborbital target ' +
      `(profile ${profile}, apogee ${f.apogee / 1e3} km, range ${f.range / 1e3} km)`);
  return suborbitalResult({ lv, guid: g, ascent: f.asc, profile, target_apogee: apogee,
    target_range: profile === 'hop' ? NaN : downrange, apogee: f.apogee,
    range: f.range, t_apogee: f.t_apogee, outcome, entry_scn: f.scn, entry: f.arc });
}
