// End-to-end parity for the original LEO reentry mission: the JS port must
// reproduce Julia's tuned deorbit elements, trajectory log, events and summary.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { default_reentry_pod } from '../src/vehicle.js';
import { deorbitElements, target_deorbit, scenario_from_elements } from '../src/scenarios.js';
import { simulate } from '../src/simulation.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_reentry.jl');

function golden() {
  const dir = mkdtempSync(join(tmpdir(), 'ssjl-reentry-'));
  try {
    const out = join(dir, 'golden.json');
    const r = spawnSync('julia', ['--project=' + root, script, out],
                        { encoding: 'utf8' });
    if (r.error || r.status !== 0) {
      if (r.stderr) process.stderr.write(r.stderr);
      return null;
    }
    return JSON.parse(readFileSync(out, 'utf8'));
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
}

// atol is one micron/metre-class: the splashdown bisection terminates on the
// first step below 1e-5 m, so the final altitude sample is ULP-sensitive and
// can differ by tens of nanometres even when every step before it is identical.
function close(got, want, what, rtol = 1e-7, atol = 1e-6) {
  if (!Number.isFinite(want)) { assert.ok(!Number.isFinite(got), `${what}: ${got} != ${want}`); return; }
  const tol = atol + rtol * Math.abs(want);
  assert.ok(Math.abs(got - want) <= tol,
            `${what}: ${got} != ${want} (d=${got - want}, tol=${tol})`);
}

const g = golden();

test('reentry mission matches Julia', t => {
  if (!g) return t.skip('julia not available');

  const veh = default_reentry_pod();
  const [el] = target_deorbit(deorbitElements(), veh);
  close(el.raan, g.el.raan, 'el.raan');
  close(el.argp, g.el.argp, 'el.argp');

  const scn = scenario_from_elements(el, veh);
  const res = simulate(scn);

  assert.equal(res.terminated, 'splashdown');
  assert.equal(res.events.length, g.events.length, 'event count');
  for (const k of ['t', 'h', 'lat', 'lon', 'vin', 'vrel', 'gamma', 'psi',
                   'mach', 'qbar', 'gload', 'alpha', 'qrate', 'rho',
                   'qdot_conv', 'qdot_rad', 'qload', 'twall']) {
    assert.equal(res.log[k].length, g.log[k].length, `log.${k} length`);
    for (let i = 0; i < g.log[k].length; i++)
      close(res.log[k][i], g.log[k][i], `log.${k}[${i}]`);
  }
  for (const [k, v] of Object.entries(g.summary))
    if (k !== 'terminated') close(res[k], v, `summary.${k}`);
  assert.equal(res.terminated, g.summary.terminated);
});
