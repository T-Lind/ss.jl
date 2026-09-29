// Parity: the browser port must reproduce the Julia core bit-for-bit to
// floating-point tolerance. Julia generates the reference on the fly so the
// two can never drift apart unnoticed. If Julia is not installed the parity
// tests skip rather than fail — the port itself has no Julia dependency.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { atmosphere_state, USSA76 } from '../src/atmosphere.js';
import { gravity_accel, j2Gravity } from '../src/gravity.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_parity.jl');

function golden() {
  const dir = mkdtempSync(join(tmpdir(), 'ssjl-parity-'));
  try {
    const out = join(dir, 'golden.json');
    const r = spawnSync('julia', ['--project=' + root, script, out],
                        { encoding: 'utf8' });
    if (r.error || r.status !== 0) return null;
    return JSON.parse(readFileSync(out, 'utf8'));
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
}

function close(got, want, what, rtol = 1e-9) {
  if (!Number.isFinite(want)) { assert.ok(!Number.isFinite(got), `${what}: expected ${want}, got ${got}`); return; }
  const tol = rtol * Math.max(Math.abs(want), 1e-300);
  assert.ok(Math.abs(got - want) <= tol,
            `${what}: ${got} != ${want} (rel ${Math.abs(got - want) / Math.abs(want)})`);
}

const g = golden();

test('USSA76 atmosphere matches Julia', t => {
  if (!g) return t.skip('julia not available');
  for (const e of g.atmosphere) {
    const [rho, T, p, a] = atmosphere_state(USSA76, e.h);
    close(rho, e.rho, `rho(h=${e.h})`);
    close(T, e.T, `T(h=${e.h})`);
    close(p, e.p, `p(h=${e.h})`);
    close(a, e.a, `a(h=${e.h})`);
  }
});

test('J2 gravity matches Julia', t => {
  if (!g) return t.skip('julia not available');
  for (const e of g.gravity) {
    const a = gravity_accel(j2Gravity(), e.r, 0.0);
    close(a[0], e.a[0], `ax(r=${e.r})`);
    close(a[1], e.a[1], `ay(r=${e.r})`);
    close(a[2], e.a[2], `az(r=${e.r})`);
  }
});
