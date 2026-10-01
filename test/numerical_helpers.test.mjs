// Independent invariants: parity alone would preserve bugs shared by both engines.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { state_from_elements, elements_from_state } from '../web/src/frames.js';
import { find_root } from '../web/src/solve.js';
import { scenario } from '../web/src/dynamics.js';
import { simulate } from '../web/src/simulation.js';
import { default_reentry_pod } from '../web/src/vehicle.js';
import { RE_EQ } from '../web/src/constants.js';
import { deorbitElements, target_deorbit, scenario_from_elements } from '../web/src/scenarios.js';

const distance = (a, b) => Math.hypot(...a.map((x, i) => x - b[i]));

test('circular and equatorial element conversions preserve orbital phase', () => {
  for (const e of [0, 0.3]) for (const i of [0, 1e-15, 0.5, Math.PI])
    for (const nu of [0.3, 1.3, 4.7]) {
      const [r, v] = state_from_elements(8e6, e, i, 0.7, 0.6, nu);
      const el = elements_from_state(r, v);
      const [rr, vv] = state_from_elements(el.a, el.e, el.i, el.raan, el.argp, el.nu);
      assert.ok(distance(r, rr) < 1e-4, `position: e=${e}, i=${i}, nu=${nu}`);
      assert.ok(distance(v, vv) < 1e-7);
    }
});

test('solver budgets include failed endpoints and results are actual evaluations', () => {
  for (let budget = 1; budget <= 12; budget++) {
    const r = find_root(() => NaN, 0, 1, { max_iter: budget });
    assert.equal(r.iterations, r.history.length);
    assert.ok(r.iterations <= budget);
    assert.notEqual(r.status, 'converged');
  }
  const r = find_root(x => Math.exp(x) - 5, 0, 40, { max_iter: 3 });
  assert.equal(r.status, 'max_iter');
  assert.equal(r.value, Math.exp(r.x) - 5);
  assert.ok(r.history.some(([x, v]) => x === r.x && v === r.value));
  assert.equal(find_root(x => 1e200 * (x - 0.5), 0, 1).x, 0.5);
  assert.equal(find_root(x => x, 1, 1, { target: 1 }).status, 'converged');
  assert.equal(find_root(x => x, 1, 1).status, 'no_bracket');
  assert.throws(() => find_root(x => x, NaN, 1), RangeError);
  assert.throws(() => find_root(x => x, 0, 1, { max_iter: 0 }), RangeError);
});

test('failed interior cannot fabricate a root or a non-finite best answer', () => {
  const r = find_root(x => x > 0.4 && x < 0.6 ? NaN : x - 0.5, 0, 1, { xtol: 0.2 });
  assert.equal(r.status, 'infeasible');
  assert.ok(Number.isFinite(r.value));
  assert.equal(r.value, r.x - 0.5);
  assert.equal(find_root(() => null, 0, 1).status, 'infeasible');
  const clipped = find_root(x => x > 2.5 ? NaN : 1 - x, 0, 10);
  assert.equal(clipped.status, 'converged');
  assert.ok(Math.abs(clipped.x - 1) < 1e-3);
});

test('reentry timeout is exact, logged, and rejects non-advancing steps', () => {
  const [r0, v0] = state_from_elements(RE_EQ + 400e3, 0, 0.5, 0, 0, 0.3);
  const base = { vehicle: default_reentry_pod(), r0, v0, t0: 10, t_max: 10.25, dt_orbit: 2 };
  const res = simulate(scenario(base), { log_dt_orbit: 1e9 });
  assert.equal(res.terminated, 'timeout');
  assert.equal(res.log.t.at(-1), base.t_max);
  assert.ok(res.log.t.every(t => t <= base.t_max));
  for (const dt_orbit of [0, -1, Infinity, NaN])
    assert.throws(() => simulate(scenario({ ...base, dt_orbit })), RangeError);
  assert.throws(() => simulate(scenario(base), { log_dt_entry: 0 }), RangeError);
  const hot = simulate(scenario({ vehicle: base.vehicle,
    r0: [RE_EQ + 50e3, 0, 0], v0: [0, 7500, 0], t_max: 0 }));
  assert.equal(hot.peak_gload, hot.log.gload[0]);
  assert.ok(hot.peak_gload > 0);
  assert.equal(hot.peak_qdot, hot.log.qdot_conv[0] + hot.log.qdot_rad[0]);
});

test('exhausted deorbit targeting returns elements from its actual flight', () => {
  const veh = default_reentry_pod();
  const [el, res] = target_deorbit(deorbitElements(), veh, { max_iter: 1 });
  const rerun = simulate(scenario_from_elements(el, veh));
  assert.equal(res.lat_splash, rerun.lat_splash);
  assert.equal(res.lon_splash, rerun.lon_splash);
  assert.equal(res.t_splash, rerun.t_splash);
  assert.throws(() => target_deorbit(el, veh, { max_iter: 0 }), RangeError);
});
