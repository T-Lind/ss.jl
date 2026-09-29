// Parity for the Moon ephemeris, Moon-fixed frame and non-spherical gravity.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { MU_EARTH, RE_MEAN, deg2rad_ } from '../src/constants.js';
import { coplanar_moon, moon_position, moon_velocity, lunar_gravity,
         lunarGravity, gravity_anomaly } from '../src/moon.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_moon.jl');

function golden() {
  const dir = mkdtempSync(join(tmpdir(), 'ssjl-moon-'));
  try {
    const out = join(dir, 'golden.json');
    const r = spawnSync('julia', ['--project=' + root, script, out], { encoding: 'utf8' });
    if (r.error || r.status !== 0) { if (r.stderr) process.stderr.write(r.stderr); return null; }
    return JSON.parse(readFileSync(out, 'utf8'));
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
}

function close(got, want, what, rtol = 1e-9, atol = 1e-6) {
  const tol = atol + rtol * Math.abs(want);
  assert.ok(Math.abs(got - want) <= tol, `${what}: ${got} != ${want} (d=${got - want}, tol=${tol})`);
}

const g = golden();

test('moon ephemeris, frame and gravity match Julia', t => {
  if (!g) return t.skip('julia not available');
  const r0 = [RE_MEAN + 200.0e3, 0.0, 0.0];
  const v0 = [0.0, Math.sqrt(MU_EARTH / (RE_MEAN + 200.0e3)), 0.0];
  const eph = coplanar_moon(r0, v0);
  const fd = lunarGravity();

  for (let i = 0; i < g.ts.length; i++) {
    const p = moon_position(eph, g.ts[i]), v = moon_velocity(eph, g.ts[i]);
    for (let j = 0; j < 3; j++) {
      close(p[j], g.pos[i][j], `pos[${i}][${j}]`);
      close(v[j], g.vel[i][j], `vel[${i}][${j}]`);
    }
  }
  for (let i = 0; i < g.grav.length; i++) {
    const a = lunar_gravity(g.grav[i].r, fd, g.grav[i].t, eph);
    for (let j = 0; j < 3; j++) close(a[j], g.grav[i].a[j], `grav[${i}][${j}]`, 1e-8, 1e-15);
  }
  for (let i = 0; i < g.grav0.length; i++) {
    const a = lunar_gravity(g.grav0[i].r, null, g.grav0[i].t, eph);
    for (let j = 0; j < 3; j++) close(a[j], g.grav0[i].a[j], `grav0[${i}][${j}]`, 1e-8, 1e-15);
  }
  const anom = [
    gravity_anomaly(fd, deg2rad_(33.0), deg2rad_(-16.0), 100.0e3, eph, 0.0),
    gravity_anomaly(fd, deg2rad_(0.0), deg2rad_(0.0), 100.0e3, eph, 0.0),
  ];
  for (let i = 0; i < anom.length; i++) close(anom[i], g.anom[i], `anomaly[${i}]`, 1e-8, 1e-15);
});
