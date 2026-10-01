import { test } from 'node:test';
import assert from 'node:assert/strict';
import { juliaGolden } from './julia_golden.mjs';
import { find_root } from '../src/solve.js';
import { fileURLToPath } from 'node:url';

const g = juliaGolden(fileURLToPath(new URL('./emit_numerics.jl', import.meta.url)));
const cases = {
  root: [x => x * x - 2, 0, 3, {}],
  target: [x => x ** 3, -1, 4, { target: 8 }],
  budget: [x => Math.exp(x) - 5, 0, 40, { max_iter: 3 }],
  failed: [x => NaN, 0, 1, { max_iter: 2 }],
  gap: [x => x > 0.4 && x < 0.6 ? NaN : x - 0.5, 0, 1, {}],
  clipped: [x => x > 2.5 ? NaN : 1 - x, 0, 10, {}],
  overflow: [x => 1e200 * (x - 0.5), 0, 1, {}],
};

function compare(got, want, path) {
  if (want === null) return assert.ok(!Number.isFinite(got), path);
  if (typeof want === 'number')
    return assert.ok(Math.abs(got - want) <= 1e-10 * Math.max(1, Math.abs(want)), `${path}: ${got} != ${want}`);
  if (typeof want === 'string') return assert.equal(got, want, path);
  assert.deepEqual(Object.keys(got).sort(), Object.keys(want).sort(), path);
  for (const k of Object.keys(want)) compare(got[k], want[k], `${path}.${k}`);
}

test('scalar solver values, budgets, bounds and failure histories match Julia', t => {
  if (!g) return t.skip('julia not available');
  for (const [name, args] of Object.entries(cases)) compare(find_root(...args), g[name], name);
});
