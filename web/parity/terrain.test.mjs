// Parity for the procedural terrain: heights across three terrains, a hazard
// score, a safe-site search, slope, and the surface model.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { MU_EARTH, RE_MEAN, R_MOON, deg2rad_ } from '../src/constants.js';
import { vunit, vscale } from '../src/vec3.js';
import { lunarTerrain, highland_terrain, mare_terrain, terrain_height,
         terrain_slope, site_hazard, safe_site, surfaceModel, surface_radius,
         surface_altitude } from '../src/terrain.js';
import { coplanar_moon, moonfixed_inv } from '../src/moon.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_terrain.jl');

function golden() {
  const dir = mkdtempSync(join(tmpdir(), 'ssjl-terrain-'));
  try {
    const out = join(dir, 'golden.json');
    const r = spawnSync('julia', ['--project=' + root, script, out], { encoding: 'utf8' });
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

test('procedural terrain matches Julia', t => {
  if (!g) return t.skip('julia not available');
  const tr = lunarTerrain(), trh = highland_terrain(), trm = mare_terrain();

  for (let i = 0; i < g.us.length; i++) {
    close(terrain_height(tr, g.us[i]), g.hd[i], `height_default[${i}]`, 1e-7, 1e-3);
    close(terrain_height(trh, g.us[i]), g.hh[i], `height_highland[${i}]`, 1e-7, 1e-3);
    close(terrain_height(trm, g.us[i]), g.hm[i], `height_mare[${i}]`, 1e-7, 1e-3);
  }

  const sh = site_hazard(tr, g.u0);
  close(sh.score, g.sh.score, 'site_hazard.score', 1e-7, 1e-6);
  close(sh.slope, g.sh.slope, 'site_hazard.slope', 1e-7, 1e-6);
  close(sh.relief, g.sh.relief, 'site_hazard.relief', 1e-7, 1e-3);

  // recompute tangents exactly as terrain.js does
  const tang = (() => {
    const u = g.u0;
    const a = Math.abs(u[2]) < 0.9 ? [0.0, 0.0, 1.0] : [1.0, 0.0, 0.0];
    const cross = (p, q) => [p[1]*q[2]-p[2]*q[1], p[2]*q[0]-p[0]*q[2], p[0]*q[1]-p[1]*q[0]];
    const e = vunit(cross(a, u));
    return [e, cross(u, e)];
  })();
  const [su, sd, sc] = safe_site(tr, g.u0, tang[0], tang[1]);
  close(sd, g.safe.d, 'safe_site.d', 1e-6, 1e-3);
  close(sc, g.safe.c, 'safe_site.c', 1e-6, 1e-3);
  for (let j = 0; j < 3; j++) close(su[j], g.safe.u[j], `safe_site.u[${j}]`, 1e-7, 1e-6);

  close(terrain_slope(tr, g.u0), g.slope, 'terrain_slope', 1e-7, 1e-6);

  const r0 = [RE_MEAN + 200.0e3, 0.0, 0.0];
  const v0 = [0.0, Math.sqrt(MU_EARTH / (RE_MEAN + 200.0e3)), 0.0];
  const eph = coplanar_moon(r0, v0);
  const sm = surfaceModel(tr, eph);
  const rr = vscale(moonfixed_inv(vunit([0.3, 0.4, 0.5]), 0.0, eph), R_MOON + 2500.0);
  close(surface_radius(sm, rr, 0.0), g.sr, 'surface_radius', 1e-7, 1e-3);
  close(surface_altitude(sm, rr, 0.0), g.sa, 'surface_altitude', 1e-7, 1e-3);
});
