import { juliaGolden } from './julia_golden.mjs';
// Parity for the panel's payload layer: the browser builds the same objects
// scripts/panelapp.jl serialised. Four missions (flyby, orbit, suborbital,
// landing) and two vehicles' rocket_geometry are compared structurally.
//
// The payloads carry decimated telemetry, so long arrays are checked for
// length and their first/last few samples; every scalar metric and every short
// array (events, metrics, sites, orbit.burns, achievements, sections) is
// compared in full. Tolerances are loose enough for a port that reorders a few
// floating-point operations, tight enough that a wrong constant shows.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { panelRun, panelGeometry } from '../src/panel.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_panel.jl');

const MISSIONS = {
  flyby: { mode: 'flyby', payload_kind: 'bus', bus_mass: '200', cargo_mass: '150' },
  orbit: { mode: 'orbit', orbit: 'leo' },
  suborbital: { mode: 'suborbital', sub_profile: 'hop', sub_apogee_km: '110', pod_mass: '350' },
  // The reference Sable cannot lift the 12.5 t lander, so the panel's landing
  // page selects a Starship-class stack; these are that preset's fields.
  landing: {
    mode: 'landing', nstages: '2', nboost: '0', diameter: '9', fairing_on: '0',
    kick_deg: '5', opt_kick: '1',
    s1_engine: 'raptor_2', s1_engines: '33', s1_prop: '3400000', s1_dry_auto: '1', s1_diameter: '9',
    s2_engine: 'raptor_2', s2_engines: '6', s2_prop: '1200000', s2_dry_auto: '1', s2_diameter: '9',
  },
};

const GEOMETRY = {
  sable: { nstages: '3', nboost: '0', diameter: '1.8' },
  falcon: {
    nstages: '3', nboost: '0', diameter: '3.7', payload_kind: 'bus', bus_mass: '200',
    s1_engine: 'merlin_1d', s1_engines: '9', s1_prop: '411000', s1_dry_auto: '1', s1_diameter: '3.7',
    s2_engine: 'merlin_1d_vac', s2_engines: '1', s2_prop: '108000', s2_dry_auto: '1', s2_diameter: '3.7',
    s3_engine: 'aj10', s3_engines: '1', s3_prop: '2200', s3_dry_auto: '1', s3_diameter: '2.4',
  },
};

const golden = () => juliaGolden(script);

const RTOL = 1e-5;
const ATOL = 1e-3;
const LONG = 40;

function assertClose(got, want, path) {
  if (want === null) {
    assert.ok(got === null || got === undefined || Number.isNaN(got),
              `${path}: expected null/NaN, got ${got}`);
    return;
  }
  if (typeof want === 'number') {
    assert.ok(Number.isFinite(got), `${path}: expected a finite number, got ${got}`);
    const tol = ATOL + RTOL * Math.abs(want);
    assert.ok(Math.abs(got - want) <= tol,
              `${path}: ${got} != ${want} (d=${got - want}, tol=${tol})`);
    return;
  }
  if (typeof want === 'string' || typeof want === 'boolean') {
    assert.strictEqual(got, want, path);
    return;
  }
  if (Array.isArray(want)) {
    assert.ok(Array.isArray(got), `${path}: expected an array`);
    assert.strictEqual(got.length, want.length, `${path}.length`);
    const idxs = want.length <= LONG
      ? Array.from({ length: want.length }, (_, i) => i)
      : [0, 1, 2, want.length - 3, want.length - 2, want.length - 1];
    for (const i of idxs) assertClose(got[i], want[i], `${path}[${i}]`);
    return;
  }
  if (typeof want === 'object') {
    assert.deepStrictEqual(Object.keys(got).sort(), Object.keys(want).sort(),
                           `${path}: key sets differ`);
    for (const k of Object.keys(want)) assertClose(got[k], want[k], `${path}.${k}`);
    return;
  }
  assert.fail(`${path}: unhandled golden type ${typeof want}`);
}

const g = golden();

test('panel mission payloads match Julia', t => {
  if (!g) return t.skip('julia not available');
  for (const [name, params] of Object.entries(MISSIONS))
    assertClose(panelRun(params), g.missions[name], name);
});

test('panel rocket_geometry matches Julia', t => {
  if (!g) return t.skip('julia not available');
  for (const [name, params] of Object.entries(GEOMETRY))
    assertClose(panelGeometry(params), g.geometry[name], name);
});
