import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { panelCatalogue, panelGeometry } from '../web/src/panel.js';
import { PRESETS, fromParams, toParams, analyse, withId, stageMass } from '../web/static/vab_parts.js';

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
