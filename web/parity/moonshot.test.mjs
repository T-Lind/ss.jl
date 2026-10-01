import { juliaGolden } from './julia_golden.mjs';
// Parity for the full circumlunar free-return mission. The cislunar coast over
// ~6.5 days is sensitive, so the design targets (perilune, return perigee) and
// the entry outcome are compared within physical tolerances; the entry itself is
// a dissipative reentry that starts from a close state.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { deg2rad_ } from '../src/constants.js';
import { moonshot } from '../src/mission.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_moonshot.jl');

const golden = () => juliaGolden(script);
const close = (got, want, what, rtol = 1e-9, atol = 1e-6) => {
  const tol = atol + rtol * Math.abs(want);
  assert.ok(Math.abs(got - want) <= tol, `${what}: ${got} != ${want} (d=${got - want}, tol=${tol})`);
};

const g = golden();

test('circumlunar moonshot matches Julia', t => {
  if (!g) return t.skip('julia not available');
  const ms = moonshot();
  const asc = ms.ascent, cis = ms.cislunar, ent = ms.entry;

  assert.equal(ms.design_status, g.design_status, 'design_status');
  close(asc.elements.rp, g.ascent.rp, 'ascent.rp', 1e-6, 2e3);
  close(asc.elements.ra, g.ascent.ra, 'ascent.ra', 1e-6, 2e3);
  close(asc.elements.i, g.ascent.i, 'ascent.i', 1e-6, 1e-5);
  close(asc.m, g.ascent.m, 'ascent.m', 1e-6, 5.0);
  close(asc.prop_left[asc.prop_left.length - 1], g.ascent.prop_left_end, 'prop_left_end', 1e-6, 2.0);

  assert.equal(cis.outcome, g.cis.outcome, 'cis outcome');
  close(cis.t_tli, g.cis.t_tli, 't_tli', 1e-6, 10.0);
  close(cis.dv_tli, g.cis.dv_tli, 'dv_tli', 1e-4, 1.0);
  close(cis.perilune_alt, g.cis.perilune_alt, 'perilune_alt', 1e-6, 1e4);
  close(cis.vac_perigee_alt, g.cis.vac_perigee_alt, 'vac_perigee_alt', 1e-6, 1e4);
  close(cis.t, g.cis.t, 'cis.t', 1e-6, 120.0);

  assert.equal(ent.terminated, g.entry.terminated, 'entry terminated');
  if (ent.terminated === 'splashdown') {
    close(ent.t_splash, g.entry.t_splash, 't_splash', 1e-6, 120.0);
    close(ent.lat_splash, g.entry.lat_splash, 'lat_splash', 1e-6, deg2rad_(0.5));
    close(ent.lon_splash, g.entry.lon_splash, 'lon_splash', 1e-6, deg2rad_(0.5));
    close(ent.v_splash, g.entry.v_splash, 'v_splash', 1e-3, 3.0);
  }
  close(ent.peak_gload, g.entry.peak_gload, 'peak_gload', 0.15, 0.5);
  close(ent.peak_qbar, g.entry.peak_qbar, 'peak_qbar', 0.15, 500.0);
  close(ent.heat_load, g.entry.heat_load, 'heat_load', 0.15, 1e6);
});
