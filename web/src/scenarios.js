// Port of src/scenarios.jl: the reference LEO reentry mission and its targeting.
import { RE_MEAN, deg2rad_ } from './constants.js';
import { state_from_elements } from './frames.js';
import { scenario } from './dynamics.js';
import { USSA76 } from './atmosphere.js';
import { default_reentry_pod } from './vehicle.js';
import { simulate } from './simulation.js';

export const WEST_COAST_TARGET_LAT = deg2rad_(32.5);
export const WEST_COAST_TARGET_LON = deg2rad_(-121.5);

export const deorbitElements = (o = {}) => ({
  apoapsis_alt: o.apoapsis_alt ?? 400.0e3,
  periapsis_alt: o.periapsis_alt ?? 25.0e3,
  inclination: o.inclination ?? deg2rad_(51.6),
  raan: o.raan ?? 0.0,
  argp: o.argp ?? 0.0,
  nu0: o.nu0 ?? Math.PI,
});

const rem2pi = x => x - 2 * Math.PI * Math.round(x / (2 * Math.PI));
const clamp1 = x => Math.min(1, Math.max(-1, x));

export function scenario_from_elements(el, veh,
    { target_lat = WEST_COAST_TARGET_LAT, target_lon = WEST_COAST_TARGET_LON,
      atmosphere = USSA76, ...kwargs } = {}) {
  const ra = RE_MEAN + el.apoapsis_alt;
  const rp = RE_MEAN + el.periapsis_alt;
  const a = 0.5 * (ra + rp);
  const e = (ra - rp) / (ra + rp);
  const [r0, v0] = state_from_elements(a, e, el.inclination, el.raan, el.argp, el.nu0);
  return scenario({ vehicle: veh, atmosphere, r0, v0, target_lat, target_lon, ...kwargs });
}

export function target_deorbit(el, veh,
    { max_iter = 8, tol_deg = 0.1, ...kwargs } = {}) {
  let raan = el.raan, argp = el.argp, res;
  for (let it = 0; it < max_iter; it++) {
    const eli = deorbitElements({ ...el, raan, argp });
    const scn = scenario_from_elements(eli, veh, kwargs);
    res = simulate(scn);
    if (res.terminated !== 'splashdown')
      throw new Error(`targeting run did not reach splashdown (terminated: ${res.terminated})`);

    const dlat = scn.target_lat - res.lat_splash;
    const dlon = rem2pi(scn.target_lon - res.lon_splash);
    if (Math.abs(dlat) < deg2rad_(tol_deg) && Math.abs(dlon) < deg2rad_(tol_deg))
      return [deorbitElements({ ...el, raan, argp }), res];

    const sini = Math.sin(el.inclination);
    const u_now = Math.asin(clamp1(Math.sin(res.lat_splash) / sini));
    const u_tgt = Math.asin(clamp1(Math.sin(scn.target_lat) / sini));
    argp += u_tgt - u_now;
    raan += dlon;
  }
  return [deorbitElements({ ...el, raan, argp }), res];
}

export function west_coast_scenario({ mass = 350.0, ...kw } = {}) {
  const veh = default_reentry_pod({ mass });
  const [el, res] = target_deorbit(deorbitElements(), veh, kw);
  const scn = scenario_from_elements(el, veh);
  return { scn, el, res };
}
