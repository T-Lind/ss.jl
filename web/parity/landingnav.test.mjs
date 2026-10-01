import { juliaGolden } from './julia_golden.mjs';
// Parity for the descent nav RNG, initial nav state, and hazard redesignation.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { MU_EARTH, RE_MEAN, R_MOON } from '../src/constants.js';
import { vunit, vcross, vscale } from '../src/vec3.js';
import { coplanar_moon } from '../src/moon.js';
import { lunarTerrain, surfaceModel } from '../src/terrain.js';
import { descentNav, hazardScan, init_nav, _nrand, _ngauss,
         redesignate } from '../src/landingnav.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_landingnav.jl');

const golden = () => juliaGolden(script);
const close = (got, want, what, rtol = 1e-9, atol = 1e-6) => {
  const tol = atol + rtol * Math.abs(want);
  assert.ok(Math.abs(got - want) <= tol, `${what}: ${got} != ${want} (d=${got - want}, tol=${tol})`);
};

const g = golden();

test('descent nav and redesignation match Julia', t => {
  if (!g) return t.skip('julia not available');
  const seed = 0x5EED1A11;
  for (let k = 1; k <= 8; k++) {
    close(_nrand(seed, k), g.nrand[k - 1], `nrand[${k}]`, 1e-9, 1e-12);
    close(_ngauss(seed, k), g.ngauss[k - 1], `ngauss[${k}]`, 1e-9, 1e-9);
  }

  const r = vscale(vunit([0.3, 0.4, 0.5]), R_MOON + 3000.0);
  const v = [10.0, -5.0, 3.0];
  const hhat = vunit(vcross(r, v));
  const nav = init_nav(descentNav(), r, v, hhat);
  for (let j = 0; j < 3; j++) {
    close(nav.r[j], g.nav.r[j], `nav.r[${j}]`, 1e-9, 1e-6);
    close(nav.v[j], g.nav.v[j], `nav.v[${j}]`, 1e-9, 1e-9);
  }
  close(nav.r_ref, g.nav.r_ref, 'nav.r_ref');

  const rr0 = [RE_MEAN + 200.0e3, 0.0, 0.0];
  const vv0 = [0.0, Math.sqrt(MU_EARTH / (RE_MEAN + 200.0e3)), 0.0];
  const eph = coplanar_moon(rr0, vv0);
  const surf = surfaceModel(lunarTerrain(), eph);
  const [u, sc, sc0, off] = redesignate(hazardScan(), surf, r, v, 0.0, hhat, 500.0);
  for (let j = 0; j < 3; j++) close(u[j], g.redesignate.u[j], `redesignate.u[${j}]`, 1e-7, 1e-6);
  close(sc, g.redesignate.sc, 'redesignate.sc', 1e-7, 1e-6);
  close(sc0, g.redesignate.sc0, 'redesignate.sc0', 1e-7, 1e-6);
  close(off, g.redesignate.off, 'redesignate.off', 1e-7, 1e-3);
});
