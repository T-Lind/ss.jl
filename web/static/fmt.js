// Number formatting, shared by every page.
//
// This module exists because the guard below was written three times and got
// it right twice. The null check comes FIRST and stays first: isFinite
// coerces, and Number(null) is 0, so isFinite(null) is true. The panel server
// serialises NaN and Inf as JSON null, so "the value is missing" arrives here
// as exactly the case the naive guard waves through.
//
// The builder's private copy lacked the check and rendered a null diameter as
// "0.0 m" — a confident, measured-looking zero for a number the server had
// explicitly declined to give. One home for these means the next such fix
// lands everywhere at once.

/** True only for a real, finite number.
 *
 *  Rejects null and undefined, which isFinite alone accepts (Number(null) is
 *  0) or mishandles quietly. Use this anywhere a metric may be absent. */
export const fin = v => v !== null && v !== undefined && isFinite(v);

/** Fixed-point, or an em-dash when there is no number to show.
 *
 *  Rounding also manufactures a sign: toFixed carries the minus off a value
 *  too small to display, so a nav error of -4 cm printed "-0 m" and a landing
 *  site on the prime meridian printed "-0.0°" — readings that look measured
 *  and are only an artefact. Anything that rounds to zero IS zero. */
export function fmt(v, d = 1) {
  if (!fin(v)) return '—';
  const s = (+v).toFixed(d);
  return /^-0(\.0*)?$/.test(s) ? s.slice(1) : s;
}

/** Megametres (1000 km) to readable km, switching to "k km" once the digits
 *  stop carrying information at cislunar distances. */
export function fmtKm(mm) {
  if (!fin(mm)) return '—';
  const km = mm * 1000;
  return km >= 99500 ? (km / 1000).toFixed(0) + 'k km' : km.toFixed(0) + ' km';
}

/** Kilograms, switching to tonnes once the number stops being readable. */
export function fmtMass(m) {
  if (!fin(m)) return '—';
  return m >= 1000 ? (m / 1000).toFixed(1) + ' t' : Math.round(m) + ' kg';
}
