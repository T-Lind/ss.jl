// Parity for the cislunar chain: seed, a fixed propagation, and the converged
// free-return design.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { MU_EARTH, RE_MEAN } from '../src/constants.js';
import { coplanar_moon } from '../src/moon.js';
import { default_moon_rocket } from '../src/propulsion.js';
import { seed_free_return, tli_alignment_time, fly_cislunar,
         design_free_return } from '../src/translunar.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_cislunar.jl');

function golden() {
  const dir = mkdtempSync(join(tmpdir(), 'ssjl-cis-'));
  try {
    const out = join(dir, 'golden.json');
    const r = spawnSync('julia', ['--project=' + root, script, out], { encoding: 'utf8', maxBuffer: 64 * 1024 * 1024 });
    if (r.error || r.status !== 0) { if (r.stderr) process.stderr.write(r.stderr); return null; }
    return JSON.parse(readFileSync(out, 'utf8'));
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
}
const close = (got, want, what, rtol = 1e-9, atol = 1e-6) => {
  const tol = atol + rtol * Math.abs(want);
  assert.ok(Math.abs(got - want) <= tol, `${what}: ${got} != ${want} (d=${got - want}, tol=${tol})`);
};

const g = golden();

test('cislunar seed, propagation and free-return design match Julia', t => {
  if (!g) return t.skip('julia not available');
  const lv = default_moon_rocket();
  const kick = lv.stages[lv.stages.length - 1];
  const r0 = [RE_MEAN + 200.0e3, 0.0, 0.0];
  const v0 = [0.0, Math.sqrt(MU_EARTH / (RE_MEAN + 200.0e3)), 0.0];
  const t0 = 0.0;
  const eph = coplanar_moon(r0, v0);
  const m_stack = 1400.0, prop_avail = 900.0;

  const [lead, tf, dvSeed] = seed_free_return(r0, v0);
  close(lead, g.seed[0], 'lead', 1e-8, 1e-10);
  close(tf, g.seed[1], 'tf', 1e-9, 1e-3);
  close(dvSeed, g.seed[2], 'dv_seed', 1e-9, 1e-6);
  const tAlign = tli_alignment_time(r0, v0, t0, eph, lead);
  close(tAlign, g.t_align, 't_align', 1e-9, 1e-3);

  // fixed propagation: the multi-day three-body coast is sensitive, so compare
  // the discrete outcome tightly and the geometry within physical tolerance
  const fixed = fly_cislunar(r0, v0, t0, eph, { t_ign: tAlign, dv: dvSeed,
    stage: kick, m_stack, prop_avail });
  assert.equal(fixed.outcome, g.fixed.outcome, 'fixed outcome');
  assert.equal(fixed.log.t.length, g.fixed.n, 'fixed log length');
  close(fixed.perilune_alt, g.fixed.perilune_alt, 'fixed perilune', 1e-4, 5e3);
  close(fixed.t, g.fixed.t, 'fixed t', 1e-6, 60);
  const dmMin = Math.min(...fixed.log.d_moon);
  close(dmMin, g.fixed.d_moon_min, 'fixed min moon distance', 1e-6, 50e3);

  // the design must reach the same mission
  const [tIgn, dv, full, status] = design_free_return(r0, v0, t0, eph, {
    stage: kick, m_stack, prop_avail });
  assert.equal(status, g.design.status, 'design status');
  assert.equal(full.outcome, g.design.outcome, 'design outcome');
  close(tIgn, g.design.t_ign, 'design t_ign', 1e-6, 5.0);
  close(dv, g.design.dv, 'design dv', 1e-4, 0.5);
  close(full.perilune_alt, g.design.perilune_alt, 1e-4, 5e3);
});
