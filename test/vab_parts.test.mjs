import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { runInNewContext } from 'node:vm';
import { panelCatalogue, panelGeometry } from '../web/src/panel.js';
import { PRESETS, fromParams, toParams, analyse, withId, stageMass, editStage, layout,
  LAUNCH_SITES, LANDING_SITES, launchHourFor } from '../web/static/vab_parts.js';

const catalogue = panelCatalogue();
const close = (a, b) => assert.ok(Math.abs(a - b) <= 1e-9 * Math.max(1, Math.abs(b)), `${a} != ${b}`);
const roundTrip = p => Object.fromEntries(toParams(fromParams(p, catalogue), catalogue));

for (const [name, preset] of Object.entries(PRESETS)) {
  test(`assembly preserves ${name} mass, delta-v and thrust-to-weight`, () => {
    const mode = preset.l_dry ? 'landing' : 'flyby';
    const before = panelGeometry({ ...preset, mode });
    const after = panelGeometry({ ...roundTrip(preset), mode });
    for (const key of ['liftoff_mass_kg', 'total_dv_mps', 'liftoff_twr']) close(after[key], before[key]);
    before.stages.forEach((s, i) => {
      for (const key of ['dry_kg', 'prop_kg', 'thrust_kn', 'isp_s']) close(after.stages[i][key], s[key]);
    });
  });
}

test('booster packing checks the actual core diameter and retains the chosen count in flight', () => {
  const asm = fromParams(PRESETS['Sable Heavy (4 strap-ons)'], catalogue);
  asm.boosters.count = 4;
  assert.ok(!analyse(asm, catalogue).issues.some(i => i.msg.includes('overlap')));
  asm.boosters.count = 8;
  assert.ok(analyse(asm, catalogue).issues.some(i => i.level === 'warn' && i.msg.includes('overlap')));
  assert.equal(toParams(asm, catalogue).get('nboost'), '8');
  asm.boosters.diameter = 0.8;
  assert.ok(!analyse(asm, catalogue).issues.some(i => i.msg.includes('overlap')));
});

test('stage moves preserve the parts and put their performance in the new firing order', () => {
  const asm = fromParams(PRESETS['Sable Heavy (4 strap-ons)'], catalogue);
  const before = structuredClone(asm), g = panelGeometry(Object.fromEntries(toParams(asm, catalogue)));
  assert.equal(editStage(asm, catalogue, 1, 'earlier'), false);
  assert.equal(editStage(asm, catalogue, 1, 'later'), true);
  const moved = panelGeometry(Object.fromEntries(toParams(asm, catalogue)));
  close(moved.stages[0].prop_kg, g.stages[1].prop_kg);
  close(moved.stages[1].thrust_kn, g.stages[0].thrust_kn);
  assert.deepEqual(asm.boosters, before.boosters);
  assert.equal(editStage(asm, catalogue, 2, 'earlier'), true);
  assert.deepEqual(asm, before);
  assert.equal(editStage(asm, catalogue, 2, 'copy'), true);
  assert.equal(analyse(asm, catalogue).ok, true);
  assert.equal(new Set(asm.parts.map(p => p.id)).size, asm.parts.length);
  const stages = analyse(asm, catalogue).stages;
  assert.equal(stages[2].mprop, stages[1].mprop);
  stages[2].tanks[0].length += 1;
  assert.notEqual(stages[2].tanks[0].length, stages[1].tanks[0].length);
  assert.equal(editStage(asm, catalogue, 1, 'copy'), true);
  assert.equal(editStage(asm, catalogue, 1, 'copy'), false);
});

test('exploded layout separates stages and keeps the fairing with its payload', () => {
  const asm = fromParams(PRESETS.Sable, catalogue), params = toParams(asm, catalogue).toString();
  const normal = layout(asm, catalogue), exploded = layout(asm, catalogue, true);
  assert.ok(exploded.height > normal.height);
  for (const [i, r] of exploded.rows.entries()) {
    close(r.y1 - r.y0, normal.rows[i].y1 - normal.rows[i].y0);
    if (r.part.type === 'decoupler') assert.ok(r.y0 > exploded.rows[i-1].y1);
  }
  close(exploded.fairing.y0, exploded.rows.at(-1).y0);
  assert.equal(toParams(asm, catalogue).toString(), params);
});

test('custom manual core and booster performance survives import and subsequent edits', () => {
  const p = { ...PRESETS['Sable Heavy (4 strap-ons)'], s1_thrust_kn: 812.345, s1_isp: 299.5,
    s2_thrust_kn: 94, s2_isp: 344, b_thrust_kn: 537.123, b_isp: 302.75, b_propellant: 'methalox' };
  const asm = fromParams(p, catalogue);
  const result = Object.fromEntries(toParams(asm, catalogue));
  for (const key of ['s1_thrust_kn', 's1_isp', 's2_thrust_kn', 's2_isp', 'b_thrust_kn', 'b_isp']) close(+result[key], p[key]);
  assert.equal(result.b_propellant, 'methalox');
  const first = analyse(asm, catalogue).stages[0].engine;
  first.count *= 2;
  close(+toParams(asm, catalogue).get('s1_thrust_kn'), p.s1_thrust_kn * 2);
});

test('lunar mission queries retain the lander when capsule fields are also present', () => {
  const p = { ...PRESETS['Saturn V (Apollo)'], mode: 'landing', pod_mass: 5560, payload_kind: 'capsule' };
  const asm = fromParams(p, catalogue);
  assert.equal(analyse(asm, catalogue).payload.kind, 'lander');
  close(+toParams(asm, catalogue).get('l_prop'), p.l_prop);
  const capsule = fromParams({ ...p, mode: 'orbit' }, catalogue);
  assert.equal(analyse(capsule, catalogue).payload.kind, 'capsule');
});

test('a tankless solid motor estimates its casing mass from the propellant it flies', () => {
  const asm = fromParams(PRESETS.Sable, catalogue);
  const bottom = analyse(asm, catalogue).stages[0];
  asm.parts = asm.parts.filter(p => !bottom.seg.includes(p));
  asm.parts.push(withId({ type: 'engine', engine: 'granite_srm', count: 1, dry: null }));
  const s = analyse(asm, catalogue).stages[0];
  assert.ok(s.mprop > 0);
  close(s.dry, stageMass(s.e, s.n, s.mprop, s.diameter));
  close(panelGeometry(Object.fromEntries(toParams(asm, catalogue))).stages[0].prop_kg, s.mprop);
});

test('unknown engines are actionable assembly errors', () => {
  const asm = fromParams({ ...PRESETS.Sable, s1_engine: 'missing' }, catalogue);
  assert.equal(analyse(asm, catalogue).ok, false);
  assert.throws(() => toParams(asm, catalogue), /unknown engine/);
});

test('both frontends ship the same vehicle translation', () => {
  assert.equal(readFileSync(new URL('../scripts/static/vab_parts.js', import.meta.url), 'utf8'),
    readFileSync(new URL('../web/static/vab_parts.js', import.meta.url), 'utf8'));
});

test('launch dialog retains the requested launch site, epoch and lunar landing site', () => {
  for (const tree of ['web', 'scripts']) {
    const source = readFileSync(new URL(`../${tree}/vab_page.html`, import.meta.url), 'utf8');
    const buildSites = source.slice(source.indexOf('function buildSites()'), source.indexOf('// ================================================================ evaluation'));
    const field = value => ({ value, addEventListener() {} });
    const el = { lSite: field('cape'), lLand: field('auto'), lTime: field('3.5') };
    const context = { el, URLSearchParams, LAUNCH_SITES, LANDING_SITES, launchHourFor,
      location: { search: '?site=mahia&target=aitken&launch_h=21.25' }, esc: String,
      clamp: (v, a, b) => Math.min(b, Math.max(a, v)) };
    runInNewContext(buildSites + '\nbuildSites();', context);
    assert.equal(el.lSite.value, 'mahia');
    assert.equal(el.lLand.value, 'aitken');
    assert.equal(+el.lTime.value, 21.25);
  }
});
