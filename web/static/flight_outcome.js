// What happened to a flight, in words — and what is likely to happen before it.
//
// A run that ends early is not a crash of the viewer: the simulator flew what
// the vehicle could do and stopped there. The launch view used to fall silent
// at that point ("END OF MISSION · MAX Q"), which reads as a bug. These two
// functions turn the run's own outcome, metrics and events — and the vehicle's
// geometry — into a sentence and some advice, so the builder can warn before
// launch and the launch view can explain after.
import { landingSummary, successfulTouchdown } from './flight_state.js';

const fin = v => typeof v === 'number' && Number.isFinite(v);
const km = v => `${Math.round(v).toLocaleString('en-US')} km`;

/** Design checks before launch, from a geometry payload (api.geometry) and
 *  the mission mode. Returns [{level: 'error'|'warn', msg}]; an error means
 *  the flight cannot work and LAUNCH should stay disabled. */
export function preflight(geo, mode) {
  const out = [];
  if (!geo || !Array.isArray(geo.stages) || !geo.stages.length) return out;
  const twr = +geo.liftoff_twr;
  if (fin(twr) && twr < 1.0)
    out.push({ level: 'error', msg: `lift-off thrust-to-weight is ${twr.toFixed(2)} — heavier than its thrust, it will not leave the pad. Add engines or strap-ons, or lighten the upper stages.` });
  else if (fin(twr) && twr < 1.15)
    out.push({ level: 'warn', msg: `lift-off thrust-to-weight ${twr.toFixed(2)} is sluggish: most of the first minute goes into fighting gravity. 1.2–1.6 is the usual range.` });
  const orbital = mode !== 'suborbital';
  const S = geo.stages, total = S.reduce((a, s) => a + (+s.dv_mps || 0), 0);
  if (orbital && S.length >= 2 && total > 0) {
    const share = (+S[0].dv_mps || 0) / total;
    if (share > 0.6)
      out.push({ level: 'warn', msg: `stage 1 carries ${(100 * share).toFixed(0)} % of the Δv. The ascent guidance flies stage 1 as a gravity turn and leaves the horizontal speed to the upper stages, so a stack this bottom-heavy tends to loft or dive instead of reaching orbit. Move Δv up: a larger upper stage, or a smaller first stage.` });
    const top = S[S.length - 1];
    const upper = S.slice(1);
    const longest = Math.max(...upper.map(s => +s.burn_s || 0));
    if (longest > 0 && longest < 60)
      out.push({ level: 'warn', msg: `the upper stages burn for ${longest.toFixed(0)} s at most — too short for the guidance to steer an insertion. Upper stages usually burn for minutes: fewer engines or more propellant.` });
    else if (fin(+top.twr) && +top.twr > 4)
      out.push({ level: 'warn', msg: `the last stage's thrust-to-weight is ${(+top.twr).toFixed(1)}; insertion burns want gentle acceleration. One smaller engine flies it more accurately.` });
  }
  return out;
}

/** Explain a flown run. `geo` is the vehicle geometry (may be null).
 *  Returns {ok, state, why, details: [], advice: []}. */
export function explainOutcome(run, geo) {
  const m = run?.metrics || {}, ev = run?.events || [], out = run?.outcome;
  const has = n => ev.some(e => e.name === n);
  const at = n => (ev.find(e => e.name === n) || {}).t;
  const details = [], advice = [];
  if (out === 'nominal' && m.on_target === false) {
    let why = 'The flight finished outside the requested target.';
    if (run.mode === 'suborbital') {
      const shot = run.suborbital?.profile === 'downrange';
      const actual = shot ? m.range_km : m.apogee_km, target = shot ? m.target_range_km : m.target_apogee_km;
      if (fin(actual) && fin(target))
        why = `${shot ? 'Splashdown range' : 'Apogee'} was ${actual.toFixed(2)} km against a target of ${target.toFixed(2)} km.`;
    } else if (run.mode === 'orbit') {
      why = 'The vehicle did not reach the requested orbital elements.';
      const o = run.orbit;
      if (fin(o?.target_rp_km) && fin(o?.target_ra_km) && fin(o?.target_incl_deg))
        details.push(`Target orbit ${km(o.target_rp_km)} × ${km(o.target_ra_km)} at ${o.target_incl_deg.toFixed(1)}°.`);
    } else if (run.mode === 'landing') {
      if (!successfulTouchdown(run))
        return { ok: false, state: 'DESCENT FAILED', why: landingSummary(run), details, advice };
      why = fin(m.target_miss_km) ? `The lander touched down ${km(m.target_miss_km)} from its target.` : 'The lander touched down outside the target area.';
    } else if (run.mode === 'flyby') {
      why = 'The trajectory missed its lunar flyby or Earth-return target.';
      if (fin(m.perilune_km)) details.push(`Closest approach to the Moon was ${km(m.perilune_km)} above the surface.`);
    }
    return { ok: false, state: 'OFF TARGET', why, details, advice };
  }
  if (!out || out === 'nominal') return { ok: true, state: 'NOMINAL', why: '', details, advice };

  if (run.mode === 'suborbital' && out === 'short')
    return { ok: false, state: 'TARGET OUT OF REACH', why: 'The ascent ended before the commanded ballistic cutoff.',
      details, advice: ['reduce the target distance or add usable ascent Δv'] };

  if (out === 'ascent_failed') {
    const twr = +geo?.liftoff_twr;
    if (fin(twr) && twr < 1.0) {
      return { ok: false, state: 'DID NOT LEAVE THE PAD',
        why: `lift-off thrust-to-weight is ${twr.toFixed(2)}: the engines push with less than the vehicle's weight, so it never climbed.`,
        details, advice: ['add engines or strap-on boosters to stage 1', 'or lighten what sits above it — upper stages and payload'] };
    }
    let why = 'the ascent never reached orbit';
    if (has('powered_descent_abort')) {
      const t = at('powered_descent_abort');
      why = `the ascent guidance could not find a pitch program that reaches orbit with this stack — the trajectory turned downward${fin(t) ? ` at T+${Math.round(t)} s` : ''} and the burn was cut.`;
    } else if (fin(m.orbit_rp_km) && m.orbit_rp_km < 0) {
      why = 'the engines stopped short of orbital speed: the path that remained comes back down.';
    }
    if (fin(m.orbit_ra_km) && fin(m.orbit_rp_km))
      details.push(`highest point ${km(Math.max(0, m.orbit_ra_km))}, lowest point of that path ${km(m.orbit_rp_km)}${m.orbit_rp_km < 0 ? ' (below the surface)' : ''}`);
    if (fin(m.prop_margin_kg) && m.prop_margin_kg > 0)
      details.push(`${(m.prop_margin_kg / 1000).toFixed(1)} t of propellant was still on board`);
    for (const w of preflight(geo, run?.mode)) advice.push(w.msg);
    if (!advice.length) advice.push('try a higher thrust-to-weight on stage 1, or more Δv in the upper stages');
    return { ok: false, state: 'DID NOT REACH ORBIT', why, details, advice };
  }
  if (out === 'design_failed')
    return { ok: false, state: 'NO TRAJECTORY',
      why: 'targeting did not converge — no trajectory reaches the requested target, so nothing flew past the parking orbit.',
      details, advice: ['the kick stage may be too small for the trans-lunar burn — check its Δv'] };
  if (out === 'entry_timeout')
    return { ok: false, state: 'DID NOT COME HOME',
      why: 'the capsule skipped back out of the atmosphere instead of decelerating.', details,
      advice: ['a steeper return perigee, or a capsule with more frontal area for its mass'] };
  if (/propellant|depleted/.test(String(out)))
    return { ok: false, state: 'PROPELLANT DEPLETED',
      why: 'a stage ran dry before its burn was complete.', details,
      advice: ['more propellant in the stage doing that burn, or a lighter payload'] };
  return { ok: false, state: 'MISSION ENDED EARLY', why: `the simulator stopped with "${String(out).replace(/_/g, ' ')}".`, details, advice };
}
