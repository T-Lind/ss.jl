import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createContext, runInContext } from 'node:vm';
import { describe } from '../web/static/runs.js';

function loadFunction(context, page, name, async = false) {
  const source = readFileSync(new URL(`../web/${page}_page.html`, import.meta.url), 'utf8');
  const start = source.indexOf(`${async ? 'async ' : ''}function ${name}(`);
  const end = source.indexOf('\n}', start);
  assert.ok(start >= 0 && end > start, name);
  runInContext(source.slice(start, end + 2), context);
}

test('a running mission keeps its submitted configuration, rejects duplicate starts, and marks newer edits stale', async () => {
  const elements = { run: {}, status: {} };
  let configuration = 'mode=orbit&pod_mass=350', finish, requests = 0, stale = 0;
  const context = createContext({ URLSearchParams, Object, performance, AbortController,
    $: id => elements[id], params: () => new URLSearchParams(configuration),
    api: { run: async () => { requests++; return new Promise(resolve => finish = resolve); } },
    busy: () => Object.assign(() => {}, { progress() {} }),
    showVerdict() {}, clearFailures() {}, clearStale() {},
    markStale: () => stale++, renderAll() {}, setVerdict() {}, loadHistory() {},
    fail: err => { throw err; }, run: null,
  });
  loadFunction(context, 'panel', 'doRun', true);
  const work = context.doRun();
  await context.doRun();
  configuration = 'mode=orbit&pod_mass=420';
  finish({ ok: true, id: 'r1', outcome: 'nominal' });
  await work;
  assert.equal(requests, 1);
  assert.equal(context.run._params.pod_mass, '350');
  assert.equal(stale, 1);
  assert.equal(elements.run.disabled, false);
});

test('restoring history immediately updates selection and navigation to that exact flight', async () => {
  const links = Object.fromEntries(['nav_analysis', 'analysislink', 'nav_launch', 'nav_build', 'status']
    .map(key => [key, { classList: { toggle() {} } }]));
  let selected;
  const context = createContext({ URLSearchParams, run: null, history: [{ id: 'r1' }, { id: 'r2' }],
    $: id => links[id], runs: { get: async id => ({ ok: true, id, params: { mode: 'orbit' } }) },
    params: () => new URLSearchParams('mode=orbit'),
    busy: () => () => {}, clearFailures() {}, applyParams() {}, clearStale() {},
    renderAll() {}, setVerdict() {}, fetchGeo() {},
    renderHistory: () => selected = context.run.id,
    fail: err => { throw err; },
  });
  loadFunction(context, 'panel', 'runQuery');
  loadFunction(context, 'panel', 'syncLinks');
  loadFunction(context, 'panel', 'restore', true);
  await context.restore('r1');
  await context.restore('r2');
  assert.equal(selected, 'r2');
  assert.equal(links.nav_launch.href, 'launch_page.html?run=r2');
  assert.equal(links.nav_analysis.href, 'analysis_page.html?run=r2');
});

test('orbit history labels use achieved orbit data', () => {
  assert.match(describe({ mode: 'orbit', metrics: { orbit_rp_km: 202, orbit_ra_km: 206 } }),
    /202×206 km orbit/);
});
