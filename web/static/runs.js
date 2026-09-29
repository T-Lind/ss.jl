// The run history, client side.
//
// A "run" is a mission that was actually flown. The server keeps the last
// dozen of them (see `panelapp.jl`, RUNS) and hands out an id for each; every
// page that displays a trajectory takes `?run=<id>` and looks it up rather
// than flying its own. That is what makes the launch view show the flight you
// were just looking at instead of a fresh one that merely resembles it.
//
// Labels live here rather than in the pages because three places render the
// same list — mission control's history strip, and the empty state on both
// /launch and /analysis — and a history entry that reads differently
// depending on which page you are on is not a history.

import { fin, fmt } from '/static/fmt.js';
import { runsList, runsGet } from '/static/api.js';

/** Every run this session has flown, newest first. Never throws: an empty
 *  history and an unreachable server both mean "nothing to offer". */
export async function list() {
  try {
    return (runsList() || {}).runs || [];
  } catch (e) {
    return [];
  }
}

/** One stored run, whole. Throws with the store's own wording on a miss —
 *  an id CAN legitimately expire, and the page needs to say so. */
export async function get(id) {
  const j = runsGet(id);
  if (!j || !j.ok) throw new Error((j && j.error) || `no run "${id}"`);
  return j;
}

export const MODE_LABEL = {
  flyby: 'circumlunar free return',
  landing: 'lunar landing',
  orbit: 'Earth orbit',
  suborbital: 'suborbital',
};

/** What kind of mission, in words. */
export function modeLabel(mode) { return MODE_LABEL[mode] || mode || 'mission'; }

/**
 * A one-line description of a run: the vehicle, then the two or three numbers
 * that distinguish this flight from the last one of the same shape.
 *
 * Which numbers depends on the mission — a suborbital hop has no perilune and
 * a landing has no splashdown — so this asks the metrics what happened rather
 * than assuming a flyby, which is what the old fixed "peri · g · days" chip
 * did to every run regardless of what it was.
 */
export function describe(d) {
  const m = d.metrics || {}, bits = [];
  if (fin(m.liftoff_t)) bits.push(`${fmt(m.liftoff_t, 1)} t`);
  if (d.mode === 'suborbital') {
    if (fin(m.apogee_km)) bits.push(`apogee ${fmt(m.apogee_km, 0)} km`);
    if (fin(m.range_km) && m.range_km > 1) bits.push(`${fmt(m.range_km, 0)} km downrange`);
  } else if (d.mode === 'orbit') {
    if (fin(m.h_park_km)) bits.push(`${fmt(m.h_park_km, 0)} km orbit`);
    if (fin(m.peak_g)) bits.push(`${fmt(m.peak_g, 1)} g`);
  } else if (d.mode === 'landing') {
    if (fin(m.touchdown_v)) bits.push(`touchdown ${fmt(m.touchdown_v, 1)} m/s`);
    if (fin(m.prop_margin_kg)) bits.push(`${fmt(m.prop_margin_kg, 0)} kg margin`);
  } else {
    if (fin(m.perilune_km)) bits.push(`peri ${fmt(m.perilune_km, 0)} km`);
    if (fin(m.peak_g)) bits.push(`${fmt(m.peak_g, 1)} g`);
  }
  if (fin(m.t_days)) bits.push(m.t_days < 0.5 ? `${fmt(m.t_days * 1440, 0)} min`
                                              : `${fmt(m.t_days, 1)} d`);
  return bits.join(' · ');
}

/** Did it do what it was asked to? Drives the colour, so it is one function. */
export function verdict(d) {
  if (d.outcome && d.outcome !== 'nominal') return 'failed';
  const m = d.metrics || {};
  if (m.on_target === false) return 'off';
  return 'nominal';
}

/** "just now" / "4 min ago" — history is a session, so hours never appear. */
export function ago(at, now = Date.now() / 1000) {
  const s = Math.max(0, now - (at || 0));
  if (s < 45) return 'just now';
  if (s < 5400) return `${Math.round(s / 60)} min ago`;
  return `${Math.round(s / 3600)} h ago`;
}
