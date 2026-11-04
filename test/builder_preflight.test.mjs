import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createContext, runInContext } from 'node:vm';
import { preflight, explainOutcome } from '../web/static/flight_outcome.js';
import { MISSIONS } from '../web/static/vab_parts.js';

test('off-target flight reports identify the target for the flown mission', () => {
  const why = explainOutcome({ mode: 'suborbital', outcome: 'nominal', suborbital: { profile: 'downrange' },
    metrics: { on_target: false, range_km: 375.25, target_range_km: 400 } });
  assert.equal(why.state, 'OFF TARGET');
  assert.match(why.why, /375\.25 km against a target of 400\.00 km/);
  const landed = explainOutcome({ mode: 'landing', outcome: 'nominal',
    metrics: { outcome: 'touchdown', on_target: false, target_miss_km: 42 } });
  assert.match(landed.why, /touched down 42 km from its target/);
});

for (const tree of ['scripts', 'web']) {
  const source = readFileSync(new URL(`../${tree}/vab_page.html`, import.meta.url), 'utf8');
  function load(c, name, async = false) {
    const start = source.indexOf(`${async ? 'async ' : ''}function ${name}(`);
    assert.ok(start >= 0, name);
    runInContext(source.slice(start, source.indexOf('\n}', start) + 2), c);
  }
  const geometry = twr => ({ liftoff_twr: twr, total_dv_mps: 12000, stages: [{ dv_mps: 6000 }, { dv_mps: 6000, burn_s: 200 }] });
  function setup() {
    const fields = new Map(), $ = id => {
      if (!fields.has(id)) fields.set(id, {});
      return fields.get(id);
    };
    const asm = { name: 'Test' };
    const el = Object.fromEntries(['undo', 'redo', 'launch', 'name', 'staging', 'report', 'issues', 'issueCount',
      'mDesc', 'lLandWrap', 'dvCheck', 'lGo', 'lErr'].map(k => [k, {}]));
    el.dlg = { open: true, close() { this.open = false; }, showModal() { this.open = true; } };
    el.lSite = { value: 'cape' }; el.lTime = { value: 3.5 }; el.lKick = { checked: true };
    const c = createContext({ $, el, asm, an: { ok: true, issues: [] }, geo: geometry(0.8),
      geoKey: JSON.stringify(asm), geoErr: null, geoSeq: 0, catalogue: {}, undoS: [], redoS: [],
      mission: 'orbit', MISSIONS: [{ id: 'orbit', mode: 'orbit', desc: 'Orbit', dv: 9, params: { mode: 'orbit' } }],
      preflight, preMode: () => 'orbit', document: {}, esc: String, stagingHTML: () => '', reportHTML: () => '',
      toParams: () => new URLSearchParams(), URLSearchParams, AbortController,
      vehicle: { save() {}, markDirty() {} }, location: {}, scheduleEval() {}, requestDraw() {},
      busy: () => Object.assign(() => {}, { progress() {} }), resetDlg() {},
      explainOutcome: () => ({ ok: true }), fail: err => { throw err; },
      clamp: (v, a, b) => Math.max(a, Math.min(b, v)),
    });
    for (const name of ['liveGeo', 'issuesHTML', 'updateChrome', 'renderLaunchSide', 'renderSide', 'showLErr']) load(c, name);
    load(c, 'evaluate', true); load(c, 'launch', true);
    return c;
  }

  test(`${tree}: custom orbit uses the edited targets and blocks invalid apsides before running`, async () => {
    const m = MISSIONS.find(x => x.id === 'custom'), c = setup();
    const draft = { orbit_perigee_km: '200', orbit_apogee_km: '100', orbit_incl_deg: '55', n_orbits: '0.5', deorbit: true };
    Object.assign(c, { MISSIONS: [m], mission: 'custom', targetDrafts: new Map([['custom', draft]]), geo: geometry(1.4) });
    load(c, 'targetIssue'); load(c, 'applyTargets');
    c.renderLaunchSide();
    assert.equal(c.el.lGo.disabled, true);
    assert.match(c.$('#lTargetError').textContent, /at or above perigee/);
    let params;
    c.api = { run: async p => { params = p; return { id: 'orbit1' }; } };
    await c.launch();
    assert.equal(params, undefined);
    draft.orbit_apogee_km = '800';
    c.renderLaunchSide();
    assert.equal(c.el.lGo.disabled, false);
    await c.launch();
    assert.equal(params.get('orbit'), 'custom');
    for (const [key, value] of Object.entries(draft)) assert.equal(params.get(key), key === 'deorbit' ? '1' : value);
  });

  test(`${tree}: geometry results refresh both launch guards and issue counts`, async () => {
    const c = setup();
    c.geo = null; c.geoKey = null;
    c.api = { geometry: async () => geometry(0.8) };
    await c.evaluate();
    assert.equal(c.el.launch.disabled, true);
    assert.equal(c.el.lGo.disabled, true);
    assert.match(c.el.issueCount.textContent, /1 error/);
    c.api.geometry = async () => geometry(1.4);
    await c.evaluate();
    assert.equal(c.el.launch.disabled, false, 'fixing the vehicle re-enables launch');
    assert.equal(c.el.lGo.disabled, false);
    assert.equal(c.el.issueCount.textContent, 'clear');
  });

  test(`${tree}: unfinished assembly survives refresh until mission control replaces the vehicle`, () => {
    const store = new Map(), draft = { name: 'Work in progress', parts: [{ type: 'tank' }] };
    let published = 'vname=original';
    const c = createContext({ STORE_KEY: 'draft', asm: draft, URLSearchParams, location: { search: '' },
      catalogue: {}, missionFromQuery: () => null, withId: p => p, PRESETS: {}, sameVehicle: () => false,
      localStorage: { setItem: (k,v) => store.set(k,v), getItem: k => store.get(k) },
      vehicle: { own: () => false, load: () => published }, fromParams: v => ({ name: v }) });
    load(c, 'startAssembly'); load(c, 'persist');
    c.persist();
    assert.equal(c.startAssembly().name, 'Work in progress');
    published = 'vname=replacement';
    assert.equal(c.startAssembly().name, published);
    store.set('draft', JSON.stringify(draft)); published = '';
    assert.equal(c.startAssembly().name, 'Work in progress', 'legacy hangar save');
  });

  test(`${tree}: stale or failed geometry cannot enable launch`, () => {
    const c = setup();
    c.geoKey = 'previous vehicle'; c.geoErr = 'geometry request failed';
    c.updateChrome(); c.renderLaunchSide();
    assert.equal(c.el.launch.disabled, true);
    assert.equal(c.el.lGo.disabled, true);
    assert.match(c.el.dvCheck.innerHTML, /Waiting/);
  });

  test(`${tree}: final launch action rejects low thrust and pending checks`, async () => {
    const c = setup();
    let requests = 0;
    c.api = { run: async () => { requests++; return { id: 'r1' }; } };
    await c.launch();
    assert.equal(requests, 0);
    assert.match(c.el.lErr.textContent, /will not leave the pad/);
    c.geoKey = 'old';
    await c.launch();
    assert.equal(requests, 0);
    c.geoKey = JSON.stringify(c.asm); c.geo = geometry(1.4);
    await c.launch();
    assert.equal(requests, 1, 'a checked vehicle can fly');
  });
}
