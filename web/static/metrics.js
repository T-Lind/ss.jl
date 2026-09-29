// What the numbers are called, and where they stop being comfortable.
//
// Mission control renders the metric groups; the analysis page names the same
// quantities in its sweep and solve pickers. Two copies of this vocabulary
// would drift into two different names for one number, which is worse than an
// ugly name — it makes the two pages look like they are measuring different
// things.

import { fmt } from './fmt.js';

/** Mission fields a sweep or a solve may vary. */
export const PARAM_LABEL = {
  pod_mass: 'pod mass', h_park_km: 'parking alt', hp_moon_km: 'perilune',
  hp_return_km: 'return perigee', incl_deg: 'inclination',
  diameter: 'diameter', fairing: 'fairing mass',
};

/** Per-stage fields, suffixed onto `stage N `. */
export const SFIELD_LABEL = {
  prop: 'prop', dry: 'dry', isp: 'Isp', thrust_kn: 'thrust',
  engines: 'engine count',
};

/** Suborbital-only parameters. */
export const SPARAM_LABEL = {
  sub_apogee_km: 'target apogee', sub_range_km: 'target range',
  sub_loft_deg: 'loft angle', sub_azimuth_deg: 'launch azimuth',
};

/** Lander and descent parameters. */
export const LPARAM_LABEL = {
  l_dry: 'lander dry mass', l_prop: 'lander propellant',
  l_thrust_kn: 'lander thrust', l_isp: 'lander Isp',
  l_throttle_min: 'lander min throttle', h_moon_park_km: 'lunar orbit altitude',
  h_pdi_km: 'descent-orbit altitude', n_rev: 'revs before DOI',
  plain_moon: 'smooth sphere, perfect navigation',
};

/** Every metric the server will chart or root-find against, with its unit. */
export const METRIC_LABEL = {
  prop_margin_kg: 'launcher prop margin [kg]',
  perilune_km: 'perilune [km]', vac_perigee_km: 'return perigee [km]',
  peak_g: 'peak entry g', peak_q_wcm2: 'peak q̇ [W/cm²]',
  t_days: 'mission days', liftoff_t: 'liftoff mass [t]',
  park_apogee_km: 'parking apogee [km]', v_splash: 'splashdown speed [m/s]',
  tli_dv: 'TLI Δv [m/s]', heat_mj: 'heat load [MJ/m²]',
  prop_left_kg: 'lander propellant left [kg]', hover_s: 'hover margin [s]',
  descent_dv: 'descent Δv [m/s]', loi_dv: 'insertion Δv [m/s]',
  touchdown_v: 'touchdown sink [m/s]', downrange_km: 'descent downrange [km]',
  min_throttle_pct: 'deepest throttle [%]',
  apogee_km: 'apogee [km]', range_km: 'ground range [km]',
  cutoff_h_km: 'cutoff altitude [km]', t_apogee_s: 'coast to apogee [s]',
};

/** A parameter's display name, including the `sN_field` stage forms that are
 *  generated rather than listed. */
export function paramLabel(key) {
  if (PARAM_LABEL[key]) return PARAM_LABEL[key];
  if (LPARAM_LABEL[key]) return LPARAM_LABEL[key];
  if (SPARAM_LABEL[key]) return SPARAM_LABEL[key];
  const m = /^(?:s(\d+)|b)_(.+)$/.exec(key);
  if (m) {
    const who = m[1] ? `stage ${m[1]}` : 'booster';
    return `${who} ${SFIELD_LABEL[m[2]] || m[2].replace(/_/g, ' ')}`;
  }
  return key.replace(/_/g, ' ');
}

export const metricLabel = key => METRIC_LABEL[key] || String(key).replace(/_/g, ' ');

// Where a number stops being comfortable.
//
// Deliberately short, and only for quantities with a defensible limit rather
// than a taste. A threshold invented for a metric nobody has a real limit for
// would be worse than no colour at all: it teaches the reader that the colour
// does not mean anything.
//
//   prop_margin_kg < 0   the launcher did not have the propellant. Not
//                        marginal — the mission as configured cannot fly.
//   peak_g > 10          Apollo entries ran about 6.5 g; sustained double
//                        digits is injury territory for a crew.
//   touchdown_v > 3      typical landing-gear design limit.
//   touchdown_vh > 1.5   lateral rate is what tips a lander over.
//   v_splash > 12        the parachutes have not done their job.
export function limit(key, x) {
  if (x == null || !isFinite(x)) return '';
  switch (key) {
    case 'prop_margin_kg': return x < 0 ? 'is-failed' : x < 50 ? 'is-marginal' : '';
    case 'peak_g':         return x > 12 ? 'is-failed' : x > 10 ? 'is-marginal' : '';
    case 'touchdown_v':    return x > 4 ? 'is-failed' : x > 3 ? 'is-marginal' : '';
    case 'touchdown_vh':   return x > 2 ? 'is-failed' : x > 1.5 ? 'is-marginal' : '';
    case 'v_splash':       return x > 12 ? 'is-marginal' : '';
    case 'hover_s':        return x < 0 ? 'is-failed' : x < 10 ? 'is-marginal' : '';
    default: return '';
  }
}

// A measured quantity and its unit, kept as separate spans so the figures
// align down a column and the unit — which never changes — stops competing
// with the number, which does.
//
// A missing value carries no unit: "— m/s" reads as a measurement that came
// out blank, when what happened is that this leg never flew. The em-dash
// alone says that, and it keeps a failed run's column from looking like a
// table of empty readings.
export function V(x, d, unit) {
  const s = fmt(x, d);
  return s === '—' || !unit ? s : s + `<span class="u-unit">${unit}</span>`;
}
