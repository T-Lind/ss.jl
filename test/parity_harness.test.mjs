import { test } from 'node:test';
import assert from 'node:assert/strict';
import { juliaGolden } from '../web/parity/julia_golden.mjs';

test('missing Julia is optional locally and mandatory in strict mode', () => {
  const missing = '/ssjl-no-such-runtime/julia';
  assert.equal(juliaGolden('unused.jl', { julia: missing, required: false }), null);
  assert.throws(() => juliaGolden('unused.jl', { julia: missing, required: true }), /Julia reference failed/);
});

test('an installed but failing reference never silently skips', () => {
  assert.throws(() => juliaGolden('unused.jl', { julia: process.execPath, required: false }), /Julia reference failed/);
});
