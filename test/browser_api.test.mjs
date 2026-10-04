import { test } from 'node:test';
import assert from 'node:assert/strict';
import { panelSweep, panelSolve, panelGeometry, panelCatalogue } from '../web/src/panel.js';

async function withApi(initial, quota, action) {
  const previous = { Worker: globalThis.Worker, sessionStorage: globalThis.sessionStorage };
  let stored = initial;
  globalThis.sessionStorage = {
    getItem: () => stored,
    setItem: (_, value) => {
      if (value.length > quota) throw new Error('quota exceeded');
      stored = value;
    },
  };
  globalThis.Worker = class {
    terminate() {}
    postMessage({ id }) {
      queueMicrotask(() => this.onmessage({ data: { id, result: {
        ok: true, outcome: 'nominal', metrics: {}, samples: 'x'.repeat(500),
      } } }));
    }
  };
  try {
    const api = await import(`../web/static/api.js?case=${Math.random()}`);
    await action(api, () => JSON.parse(stored));
  } finally {
    for (const [key, value] of Object.entries(previous)) {
      if (value === undefined) delete globalThis[key];
      else globalThis[key] = value;
    }
  }
}

test('history persists the newest flight when older trajectories exceed storage quota', async () => {
  await withApi(null, 1800, async (api, stored) => {
    for (let i = 0; i < 5; i++) await api.run('mode=orbit&vname=Test');
    assert.equal(api.runsList().runs.length, 5, 'current page retains its in-memory history');
    const saved = stored();
    assert.equal(saved.order[0], 'r5');
    assert.equal(saved.runs[0].id, 'r5');
    assert.ok(saved.order.length < 5 && saved.order.length > 0);
    assert.equal(saved.seq, 5);
  });
});

test('restored history filters invalid entries and never reuses a flight id', async () => {
  const old = JSON.stringify({ runs: [null, { ok: true, id: 'r9', params: {} }],
    order: ['r9', 'missing', 'r9'], seq: 2 });
  await withApi(old, Infinity, async api => {
    assert.deepEqual(api.runsList().runs.map(r => r.id), ['r9']);
    assert.equal((await api.run('mode=orbit')).id, 'r10');
  });
});

test('a single flight over quota replaces stale history without recycling its id', async () => {
  await withApi(JSON.stringify({ runs: [], order: [], seq: 9 }), 100, async (api, stored) => {
    const run = await api.run('mode=orbit');
    assert.equal(run.id, 'r10');
    assert.equal(api.runsGet(run.id), run);
    assert.deepEqual(stored(), { runs: [], order: [], seq: 10 });
  });
});

test('cancelling a browser computation stops it without saving a run and permits the next flight', async () => {
  await withApi(null, Infinity, async api => {
    const controller = new AbortController();
    const flight = api.run('mode=orbit', { signal: controller.signal });
    controller.abort();
    await assert.rejects(flight, { name: 'AbortError', message: 'operation cancelled' });
    assert.equal(api.runsList().runs.length, 0);
    assert.equal((await api.run('mode=orbit')).id, 'r1');
  });
});

test('non-finite panel inputs fail before geometry or simulation, while blank fields use defaults', () => {
  for (const value of ['Infinity', '-Infinity', 'NaN', '1e999'])
    assert.throws(() => panelGeometry({ diameter: value }), /finite number/);
  assert.equal(panelGeometry({ diameter: '  ' }).diameter, panelGeometry({}).diameter);
});

test('browser study entry points report evaluated missions and completion', () => {
  const sweepProgress = [], solveProgress = [];
  const base = { mode: 'suborbital', sub_apogee_km: '90' };
  const sweep = panelSweep({ ...base, sweep_param: 'pod_mass', sweep_min: '340',
    sweep_max: '350', sweep_n: '2' }, p => sweepProgress.push(p));
  assert.equal(sweep.ok, true);
  assert.equal(sweep.runs.length, 2);
  assert.ok(sweepProgress.some(p => p.current === 1 && p.total === 2));
  assert.equal(sweepProgress.at(-1).done, true);
  const solve = panelSolve({ ...base, solve_param: 'pod_mass', solve_metric: 'range_km',
    solve_min: '340', solve_max: '350', solve_target: '1000', solve_iters: '4' },
    p => solveProgress.push(p));
  assert.equal(solve.ok, true);
  assert.ok(solveProgress.some(p => p.stage === 'solve' && p.current > 0));
  assert.equal(solveProgress.at(-1).done, true);
});

test('orbit studies offer achieved orbit metrics and reject unrelated lunar metrics', () => {
  const catalogue = panelCatalogue();
  assert.ok(catalogue.orbit_metrics.includes('orbit_ra_km'));
  assert.ok(!catalogue.orbit_metrics.includes('tli_dv'));
  const rejected = panelSolve({ mode: 'orbit', solve_metric: 'tli_dv' });
  assert.equal(rejected.ok, false);
  const solved = panelSolve({ mode: 'orbit', solve_param: 'pod_mass',
    solve_metric: 'liftoff_t', solve_min: '340', solve_max: '360',
    solve_target: '57.79', solve_iters: '4' });
  assert.equal(solved.ok, true);
  assert.equal(solved.status, 'converged');
  assert.ok(Math.abs(solved.value - 57.79) < .01);
});
