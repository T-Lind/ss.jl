import { juliaGolden } from './julia_golden.mjs';
// Parity for lunar arrival and powered descent: the LM-class burns, the
// closed-loop descent from a descent-orbit periapsis, and the whole
// pad-to-surface mission.
//
// A powered descent is closed-loop and dissipative — the braking phase is shot
// against a nominal sphere and then flown, and the terminal phase chases a
// sink-rate profile — so unlike the unstable ascent it tracks the Julia
// reference to near machine precision. Discrete outcomes are asserted exactly;
// trajectory-derived summaries are compared within physical tolerances, since
// the braking shooter can land on either side of a loose residual.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';
import { MU_MOON, R_MOON, deg2rad_ } from '../src/constants.js';
import { vnorm } from '../src/vec3.js';
import { default_lander, lander_mass, lander as L, powered_descent,
         moonlanding, _moon_step, loi_burn, doi_burn, selenographic } from '../src/landing.js';
import { starship_expendable } from '../src/propulsion.js';

const root = join(dirname(fileURLToPath(import.meta.url)), '..', '..');
const script = join(root, 'web', 'parity', 'emit_landing.jl');

const golden = () => juliaGolden(script);

function close(got, want, what, [rtol, atol]) {
  if (!Number.isFinite(want)) { assert.ok(!Number.isFinite(got), `${what}: ${got} != ${want}`); return; }
  const tol = atol + rtol * Math.abs(want);
  assert.ok(Math.abs(got - want) <= tol,
            `${what}: ${got} != ${want} (d=${got - want}, tol=${tol})`);
}

const g = golden();

test('impulsive burns and Moon-frame step match closed form', () => {
  // circularising at periapsis of a 100 x 15 km ellipse is exactly the speed
  // difference, and the DOI is the same statement run backwards
  const rp = R_MOON + 100e3;
  const v_circ = Math.sqrt(MU_MOON / rp);
  const r0 = [rp, 0.0, 0.0];
  const [dv, vafter] = loi_burn(r0, [0.0, 1.15 * v_circ, 0.0]);
  close(dv, 0.15 * v_circ, 'loi dv', [1e-12, 1e-9]);
  close(vnorm(vafter), v_circ, 'loi v_after', [1e-12, 1e-9]);

  const [dv2, v2] = doi_burn(r0, [0.0, v_circ, 0.0], 15e3);
  const a_t = 0.5 * (rp + R_MOON + 15e3);
  close(dv2, v_circ - Math.sqrt(MU_MOON * (2 / rp - 1 / a_t)), 'doi dv', [1e-12, 1e-9]);
  assert.ok(dv2 > 15.0 && dv2 < 30.0, 'DOI is a couple of dozen m/s');
  close(vnorm(v2), Math.sqrt(MU_MOON * (2 / rp - 1 / a_t)), 'doi v_after', [1e-12, 1e-9]);
  assert.throws(() => doi_burn(r0, [0.0, v_circ, 0.0], 150e3));

  // tide-free ballistic step conserves the two-body energy
  const a = R_MOON + 100e3;
  const v = Math.sqrt(MU_MOON / a);
  const eps = 0.5 * v * v - MU_MOON / a;
  const [rn, vn] = _moon_step([a, 0.0, 0.0], [0.0, v, 0.0], 60.0);
  const eps2 = 0.5 * vn[0] ** 2 + 0.5 * vn[1] ** 2 + 0.5 * vn[2] ** 2 - MU_MOON / vnorm(rn);
  close(eps2, eps, 'energy', [1e-9, 1e-3]);
});

test('powered descent from a descent-orbit periapsis matches Julia', t => {
  if (!g) return t.skip('julia not available');
  const lander = L({ mdry: 3500.0, mprop: 5700.0, thrust: 45e3, isp: 311.0, throttle_min: 0.10 });
  const rpdi = R_MOON + 15e3;
  const a_d = 0.5 * (rpdi + R_MOON + 100e3);
  const vpdi = Math.sqrt(MU_MOON * (2 / rpdi - 1 / a_d));

  const p = g.powered;
  close(rpdi, p.rpdi, 'rpdi', [0, 1e-6]);
  close(vpdi, p.vpdi, 'vpdi', [1e-15, 1e-9]);

  const d = powered_descent(lander, [rpdi, 0.0, 0.0], [0.0, vpdi, 0.0], 9200.0);
  assert.equal(d.outcome, p.outcome, 'powered outcome');
  close(d.v_vertical, p.v_vertical, 'v_vertical', [1e-4, 2e-2]);
  close(d.v_horizontal, p.v_horizontal, 'v_horizontal', [1e-4, 2e-2]);
  close(d.prop_used, p.prop_used, 'prop_used', [1e-6, 0.5]);
  close(d.prop_left, p.prop_left, 'prop_left', [1e-6, 0.5]);
  close(d.hover_s, p.hover_s, 'hover_s', [1e-5, 0.5]);
  close(d.min_throttle, p.min_throttle, 'min_throttle', [1e-4, 1e-3]);
  close(d.dv_braking, p.dv_braking, 'dv_braking', [1e-5, 0.5]);
  close(d.dv_terminal, p.dv_terminal, 'dv_terminal', [1e-4, 0.5]);
  close(d.t_touchdown, p.t_touchdown, 't_touchdown', [1e-6, 1.0]);
  close(d.t_gate, p.t_gate, 't_gate', [1e-6, 1.0]);
  close(d.downrange, p.downrange, 'downrange', [1e-5, 10.0]);
  close(d.pitch0, p.pitch0, 'pitch0', [1e-4, 1e-4]);
  close(d.pitch_rate, p.pitch_rate, 'pitch_rate', [1e-3, 1e-6]);

  // physical acceptance, independent of the reference
  assert.equal(d.outcome, 'touchdown', 'lands');
  assert.ok(d.v_vertical < 3.0, 'survivable sink rate');
  assert.ok(d.v_horizontal < 2.0, 'limited lateral drift');
  close(vnorm(d.r), R_MOON, 'on the surface', [0, 5.0]);
  assert.ok(d.prop_left > 0.0, 'propellant left');
  assert.ok(d.min_throttle >= lander.throttle_min - 1e-9, 'respects throttle floor');
  close(d.prop_used + d.prop_left, lander.mprop, 'propellant books', [1e-6, 1.0]);
});

test('a lander that cannot throttle cannot land', () => {
  const stiff = L({ mdry: 3500.0, mprop: 5700.0, thrust: 45e3, isp: 311.0, throttle_min: 1.0 });
  const rpdi = R_MOON + 15e3;
  const a_d = 0.5 * (rpdi + R_MOON + 100e3);
  const vpdi = Math.sqrt(MU_MOON * (2 / rpdi - 1 / a_d));
  const d2 = powered_descent(stiff, [rpdi, 0.0, 0.0], [0.0, vpdi, 0.0], 9200.0);
  assert.notEqual(d2.outcome, 'touchdown');
});

test('full landing mission matches Julia', t => {
  if (!g) return t.skip('julia not available');
  const lnd = default_lander();
  const lv = starship_expendable({ payload: lander_mass(lnd) });
  const ls = moonlanding({ lander: lnd, lv, kick_angle: deg2rad_(5.0) });
  const m = g.moonlanding, dm = m.descent;

  assert.equal(ls.cislunar.outcome, 'perilune', 'reached perilune');
  close(ls.cislunar.perilune_alt, m.perilune_alt, 'perilune_alt', [1e-6, 25e3]);
  close(ls.dv_loi, m.dv_loi, 'dv_loi', [1e-6, 0.05]);
  close(ls.dv_doi, m.dv_doi, 'dv_doi', [1e-6, 0.05]);
  close(ls.t_loi, m.t_loi, 't_loi', [1e-6, 1.0]);
  close(ls.t_doi, m.t_doi, 't_doi', [1e-6, 1.0]);
  close(ls.t_pdi, m.t_pdi, 't_pdi', [1e-6, 1.0]);
  close(ls.t_touchdown, m.t_touchdown, 't_touchdown', [1e-6, 1.0]);
  close(ls.lat_land, m.lat_land, 'lat_land', [1e-6, 1e-6]);
  close(ls.lon_land, m.lon_land, 'lon_land', [1e-6, 1e-6]);
  close(ls.prop_margin, m.prop_margin, 'prop_margin', [1e-6, 0.5]);

  assert.equal(ls.descent.outcome, dm.outcome, 'descent outcome');
  close(ls.descent.v_vertical, dm.v_vertical, 'v_vertical', [1e-4, 2e-2]);
  close(ls.descent.v_horizontal, dm.v_horizontal, 'v_horizontal', [1e-4, 2e-2]);
  close(ls.descent.prop_used, dm.prop_used, 'prop_used', [1e-6, 0.5]);
  close(ls.descent.hover_s, dm.hover_s, 'hover_s', [1e-5, 0.5]);
  close(ls.descent.min_throttle, dm.min_throttle, 'min_throttle', [1e-4, 1e-3]);
  close(ls.descent.dv_braking, dm.dv_braking, 'dv_braking', [1e-5, 0.5]);
  close(ls.descent.dv_terminal, dm.dv_terminal, 'dv_terminal', [1e-4, 0.5]);
  close(ls.descent.downrange, dm.downrange, 'descent downrange', [1e-5, 10.0]);
  close(ls.descent.pitch0, dm.pitch0, 'descent pitch0', [1e-4, 1e-4]);
  close(ls.descent.pitch_rate, dm.pitch_rate, 'descent pitch_rate', [1e-3, 1e-6]);

  // the mission's own acceptance criteria
  assert.equal(ls.descent.outcome, 'touchdown', 'lands');
  assert.ok(ls.prop_margin > 0.0, 'positive propellant margin');
  assert.ok(ls.descent.hover_s > 30.0, 'a real hover margin');
  assert.ok(ls.t_touchdown / 86400 < 6.0, 'free return still comes home in time');
  assert.ok(ls.dv_loi > 750.0 && ls.dv_loi < 1100.0, 'LOI from a free return');
  assert.ok(ls.dv_doi > 10.0 && ls.dv_doi < 40.0, 'DOI is a small nudge');
  assert.ok(ls.lon_land >= -Math.PI && ls.lon_land <= Math.PI, 'longitude wraps');

  // touchdown is on the sphere and the reported site is consistent with it
  const [lat2, lon2] = selenographic(ls.descent.r, ls.t_touchdown, ls.eph);
  close(lat2, ls.lat_land, 'lat_land consistent', [1e-9, 1e-12]);
  close(lon2, ls.lon_land, 'lon_land consistent', [1e-9, 1e-12]);
});
