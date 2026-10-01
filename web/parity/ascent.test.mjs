import { juliaGolden } from './julia_golden.mjs';
// Parity for the ascent.
//
// Unlike the reentry (dissipative, matched to 1e-8 over 3800 samples), a
// gravity-turn ascent is unstable: a 1-ULP difference between JS Math.* and
// Julia's libm at t~1 s amplifies to ~30 m by cutoff. So the trajectory is
// compared within physical tolerances, while the discrete behaviour that
// matters — staging order and times, cutoff, orbital elements, outcome — is
// asserted tightly.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { default_moon_rocket } from '../src/propulsion.js';
import { ascentGuidance, tune_ascent, simulate_ascent } from '../src/launch.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_ascent.jl');

const golden = () => juliaGolden(script);

function close(got, want, what, [rtol, atol]) {
  if (!Number.isFinite(want)) { assert.ok(!Number.isFinite(got), `${what}: ${got} != ${want}`); return; }
  const tol = atol + rtol * Math.abs(want);
  assert.ok(Math.abs(got - want) <= tol,
            `${what}: ${got} != ${want} (d=${got - want}, tol=${tol})`);
}

// [rtol, atol] per logged field
const LOG_TOL = {
  t: [1e-9, 1e-3], rx: [1e-8, 100], ry: [1e-8, 100], rz: [1e-8, 100],
  h: [1e-8, 100], vrel: [1e-8, 0.5], vin: [1e-8, 0.5], gamma: [1e-9, 5e-4],
  mach: [1e-3, 1e-2], qbar: [1e-3, 100], m: [1e-9, 2.0], thrust: [1e-3, 100],
  lat: [1e-9, 2e-5], lon: [1e-9, 2e-5], downrange: [1e-8, 150],
};

const g = golden();

test('ascent matches Julia', t => {
  if (!g) return t.skip('julia not available');
  const lv = default_moon_rocket();

  // 1) The simulator, flown on Julia's own tuned guidance.
  const jg = ascentGuidance({ pitch0: g.guid.pitch0, pitch_rate: g.guid.pitch_rate,
                              kick_angle: g.guid.kick_angle, h_target: g.guid.h_target });
  const res = simulate_ascent(lv, jg);

  assert.equal(res.reached_orbit, g.summary.reached_orbit);
  close(res.h_cut, g.summary.h_cut, 'h_cut', [1e-9, 100]);
  close(res.gamma_cut, g.summary.gamma_cut, 'gamma_cut', [1e-9, 1e-4]);
  close(res.m, g.summary.m, 'm', [1e-9, 2.0]);
  for (let i = 0; i < g.summary.prop_left.length; i++)
    close(res.prop_left[i], g.summary.prop_left[i], `prop_left[${i}]`, [1e-9, 2.0]);
  for (const k of ['a', 'rp', 'ra']) close(res.elements[k], g.summary.elements[k], `elements.${k}`, [1e-9, 300]);
  for (const k of ['e', 'i']) close(res.elements[k], g.summary.elements[k], `elements.${k}`, [1e-9, 1e-4]);

  // staging and events: same names, same order, same times
  assert.equal(res.events.length, g.events.length, 'event count');
  for (let i = 0; i < g.events.length; i++) {
    assert.equal(res.events[i].name, g.events[i].name, `event[${i}].name`);
    close(res.events[i].t, g.events[i].t, `event[${i}].t`, [1e-9, 0.05]);
  }

  for (const [k, tol] of Object.entries(LOG_TOL)) {
    assert.equal(res.log[k].length, g.log[k].length, `log.${k} length`);
    for (let i = 0; i < g.log[k].length; i++)
      close(res.log[k][i], g.log[k][i], `log.${k}[${i}]`, tol);
  }

  // 2) The 2x2 shooter must reach the same orbit: same acceptance test Julia
  //    uses, and the same mass delivered to insertion.
  const [guid, tuned] = tune_ascent(lv, ascentGuidance());
  assert.ok(tuned.reached_orbit, 'tune_ascent reached orbit');
  assert.ok(Math.abs(tuned.h_cut - guid.h_target) < 1e3, 'cutoff altitude within tol');
  assert.ok(Math.abs(tuned.gamma_cut) < 0.05 * Math.PI / 180, 'cutoff gamma within tol');
  close(tuned.m, g.summary.m, 'tuned mass to orbit', [1e-9, 5.0]);
});
