import { find_root } from './solve.js';
export { find_root } from './solve.js';
// Port of the payload builders in scripts/panelapp.jl. The HTTP server,
// routing and static-file serving are not needed in the browser: these
// functions produce the same payload objects the pages used to receive as
// JSON, and `panelRun` is the in-process replacement for POST /api/run.
import { MU_EARTH, RE_MEAN, R_MOON, A_MOON, G0, P0_SEA, OMEGA_EARTH,
         deg2rad_, rad2deg_ } from './constants.js';
import { vsub, vcross, vdot, vnorm, vunit } from './vec3.js';
import { ecef_from_geodetic, geodetic_from_ecef, earth_rotation_angle } from './frames.js';
import { rot_z } from './vec3.js';
import { USSA76 } from './atmosphere.js';
import { PROPELLANTS, ENGINES, lookup_engine, propellant, bulk_density,
         sized_stage, stage_mass } from './engines.js';
import { stage, stage_diameter, stage_length, stage_volume, stage_burn_time,
         stage_dv, stage_thrust, pad_thrust, liftoff_mass, booster_mass,
         stack_mass_above, stack_sref, bare_payload_cd, LV_CD_TABLE,
         core_diameter, launchVehicle, boosterSet, cruise_inertia } from './propulsion.js';
import { ascentGuidance, tune_ascent } from './launch.js';
import { moonshot, pod_radius } from './mission.js';
import { lander, lander_mass, lander_dv, moonlanding, selenographic } from './landing.js';
import { lunarTerrain } from './terrain.js';
import { descentNav, hazardScan } from './landingnav.js';
import { lunarGravity, moonfixed } from './moon.js';
import { sized_kick_rcs, cruise_rcs_budget, orbit_rcs_budget } from './rcs.js';
import { earthorbit, ORBITS } from './earthorbit.js';
import { suborbital } from './suborbital.js';
import { rocket_mesh, interstage_length } from './mesh.js';

// ---------------------------------------------------------------- helpers --

// Panel string parameter with a default (blank and whitespace count as absent).
export function gets(p, k, def) {
  const v = p[k] ?? '';
  const s = String(v).trim();
  return s === '' ? def : s;
}

function getf(p, k, def) {
  if (k in p && String(p[k]).trim() !== '') {
    const n = Number(p[k]);
    if (!Number.isFinite(n)) throw new Error(`${k} must be a finite number`);
    return n;
  }
  return def;
}

// A form checkbox: present and truthy, absent and defaulted.
function getb(p, k, def) {
  return (k in p) ? ['1', 'true', 'on'].includes(gets(p, k, '0')) : def;
}

// ------------------------------------------------------------ mission api --

const STAGE_ROLE = {
  booster: { dry: 3800.0, prop: 42000.0, thrust: 950.0, isp: 305.0, ae: 0.80, ptype: 'kerolox', ne: 5 },
  upper:   { dry: 900.0, prop: 9500.0, thrust: 95.0, isp: 345.0, ae: 0.0, ptype: 'kerolox', ne: 1 },
  kick:    { dry: 140.0, prop: 950.0, thrust: 15.0, isp: 315.0, ae: 0.0, ptype: 'hypergolic', ne: 1 },
};
const stage_role = (k, nst) => k === 1 ? 'booster' : k === nst ? 'kick' : 'upper';
const stage_scale = (k, nst) => stage_role(k, nst) === 'upper' ? 0.25 ** (k - 2) : 1.0;

const n_stages = p => Math.min(5, Math.max(2, Math.round(getf(p, 'nstages', 3.0))));
const n_boosters = p => Math.min(8, Math.max(0, Math.round(getf(p, 'nboost', 0.0))));

const BOOSTER_ROLE = { dry: 900.0, prop: 12000.0, thrust: 380.0, isp: 285.0, ae: 0.32, ptype: 'kerolox', ne: 2 };

// Build one stage from the panel's s<k>_* fields.
function stage_from_params(p, pre, name, dia,
                           { dry, prop, thrust_kn, isp, ae, ptype = 'kerolox', nedef = 1 } = {}) {
  const ne = Math.min(33, Math.max(1, Math.round(getf(p, pre + 'engines', nedef))));
  const eng = gets(p, pre + 'engine', 'manual');
  const dst = Math.max(0.3, getf(p, pre + 'diameter', dia));
  const mdry_in = getf(p, pre + 'dry', dry);
  let pr;
  if (eng !== 'manual') {
    const epr = lookup_engine(eng).prop;
    const typed = gets(p, pre + 'propellant', '');
    if (typed !== '' && typed !== epr.name)
      throw new Error(`${eng} burns ${epr.name}; it cannot run on ${typed}`);
    pr = epr;
  } else {
    pr = propellant(gets(p, pre + 'propellant', ptype));
  }
  const mprop = gets(p, pre + 'size_by', 'prop') === 'length'
    ? Math.max(1.0, (Math.max(getf(p, pre + 'len', 10.0), 0.95 * dst) - 0.9 * dst) *
                     bulk_density(pr) * Math.PI * (dst / 2) ** 2 / 1.15)
    : getf(p, pre + 'prop', prop);
  if (eng !== 'manual') {
    const auto = ['1', 'true', 'on'].includes(gets(p, pre + 'dry_auto', '0'));
    return sized_stage(name, { engine: eng, n_engines: ne, prop_mass: mprop,
                               diameter: dst, dry_mass: auto ? null : mdry_in });
  }
  return stage(name, mdry_in, mprop, getf(p, pre + 'thrust_kn', thrust_kn) * 1e3,
               getf(p, pre + 'isp', isp), ae, pr, ne, dst);
}

// What a stage would weigh dry if built the way `stage_mass` says stages are.
function dry_estimate(st, vehicle_d) {
  const n = Math.max(st.n_engines, 1);
  const per = st.thrust_vac / n;
  let hit = null;
  for (const key of Object.keys(ENGINES)) {
    const e = ENGINES[key];
    if (e.prop.name === st.prop.name &&
        Math.abs(e.thrust_vac - per) <= 0.02 * Math.max(per, 1.0) &&
        Math.abs(e.isp_vac - st.isp_vac) <= 0.02 * Math.max(st.isp_vac, 1.0)) {
      hit = e; break;
    }
  }
  const eng = hit !== null
    ? hit
    : { name: 'estimate', prop: st.prop, thrust_vac: per, isp_vac: st.isp_vac,
        isp_sl: st.isp_vac, ae: st.ae / n, mass: per / (G0 * 100.0), throttle_min: 0.4 };
  return stage_mass(eng, n, st.mprop, stage_diameter(st, vehicle_d)).dry;
}

// Build the strap-on booster sets from the panel's b_* fields.
function boosters_from_params(p, dia) {
  const nb = n_boosters(p);
  if (nb === 0) return [];
  const d = BOOSTER_ROLE;
  const st = stage_from_params(p, 'b_', 'strap', dia * 0.85,
    { dry: d.dry, prop: d.prop, thrust_kn: d.thrust, isp: d.isp, ae: d.ae,
      ptype: d.ptype, nedef: d.ne });
  return [boosterSet({ stage: st, count: nb,
    ignition_delay: Math.max(0.0, getf(p, 'b_ign_delay', 0.0)),
    sep_delay: Math.max(0.0, getf(p, 'b_sep_delay', 0.0)),
    core_throttle: Math.min(1.0, Math.max(0.2, getf(p, 'b_throttle', 100.0) / 100)) })];
}

// The capsule's diameter [m]: whatever the form states, or the mass fit.
const pod_diameter = (p, spacecraft_kg) => {
  const d = getf(p, 'pod_dia', 0.0);
  return d > 0 ? d : 2 * pod_radius(spacecraft_kg);
};

const payload_is_bus = p => gets(p, 'payload_kind', 'capsule') === 'bus';
const BUS_MASS_DEFAULT = 200.0;
const spacecraft_mass = p => Math.max(payload_is_bus(p)
  ? getf(p, 'bus_mass', BUS_MASS_DEFAULT) : getf(p, 'pod_mass', 350.0), 0.0);
const cargo_mass = p => Math.max(getf(p, 'cargo_mass', 0.0), 0.0);
const payload_total = p => spacecraft_mass(p) + cargo_mass(p);

// The stem a stage is named after, taken from the vehicle's own name.
export function stage_slug(vname) {
  const base = vname.split('(')[0];
  let s = base.trim().toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '');
  return s === '' ? 'sable' : s.slice(0, 14);
}

// Build a LaunchVehicle from panel parameters.
export function lv_from_params(p) {
  const dia = getf(p, 'diameter', 1.8);
  const nst = n_stages(p);
  const slug = stage_slug(gets(p, 'vname', 'Sable (panel)'));
  const stages = [];
  for (let k = 1; k <= nst; k++) {
    const d = STAGE_ROLE[stage_role(k, nst)];
    const f = stage_scale(k, nst);
    const nm = k === nst ? slug + 'k' : slug + k;
    stages.push(stage_from_params(p, `s${k}_`, nm, dia,
      { dry: d.dry * f, prop: d.prop * f, thrust_kn: d.thrust * f, isp: d.isp,
        ae: d.ae * f, ptype: d.ptype, nedef: d.ne }));
  }
  const fair = getb(p, 'fairing_on', true);
  const pod = payload_total(p);
  const pod_d = pod_diameter(p, spacecraft_mass(p));
  const sref = stack_sref(stages.map(s => stage_diameter(s, dia)), pod_d, fair);
  return launchVehicle({
    name: gets(p, 'vname', 'Sable (panel)'),
    stages,
    fairing_mass: fair ? getf(p, 'fairing', 150.0) : 0.0,
    payload_mass: pod,
    sref,
    cd: fair ? LV_CD_TABLE : bare_payload_cd(pod_d, sref),
    boosters: boosters_from_params(p, dia),
  });
}

const opt_kick = p => ['1', 'true', 'on'].includes(gets(p, 'opt_kick', '0'));
const kick_rad = p => deg2rad_(Math.min(30.0, Math.max(0.5, getf(p, 'kick_deg', 8.0))));

// Which mission the panel is flying.
export function mission_mode(p) {
  const m = gets(p, 'mode', 'flyby');
  return m === 'landing' ? 'landing' : m === 'orbit' ? 'orbit'
       : m === 'suborbital' ? 'suborbital' : 'flyby';
}

const lander_from_params = p => lander({
  name: 'lander',
  mdry: Math.max(100.0, getf(p, 'l_dry', 3500.0)),
  mprop: Math.max(10.0, getf(p, 'l_prop', 9000.0)),
  thrust: Math.max(1.0e3, getf(p, 'l_thrust_kn', 45.0) * 1e3),
  isp: Math.min(500.0, Math.max(100.0, getf(p, 'l_isp', 311.0))),
  throttle_min: Math.min(1.0, Math.max(0.02, getf(p, 'l_throttle_min', 10.0) / 100)),
  diameter: Math.max(0.5, getf(p, 'l_diameter', 4.2)),
});

function landing_vehicle_from_params(p, landerIn = lander_from_params(p)) {
  const lv0 = lv_from_params(p);
  const dia = getf(p, 'diameter', 1.8);
  const fair = lv0.fairing_mass > 0.0;
  const sref = stack_sref(lv0.stages.map(s => stage_diameter(s, dia)),
                          landerIn.diameter, fair);
  return launchVehicle({
    name: lv0.name, stages: lv0.stages, fairing_mass: lv0.fairing_mass,
    payload_mass: lander_mass(landerIn), sref,
    cd: fair ? LV_CD_TABLE : bare_payload_cd(landerIn.diameter, sref),
    boosters: lv0.boosters,
  });
}

// ---------------------------------------------------------------- decimate --

// Decimate a vector to at most n points (keeping ends).
function deci(v, n) {
  if (v.length <= n) return v.map(Number);
  return deci_idx(v.length, n).map(i => v[i]);
}

function deci_idx(len, n) {
  if (len <= n) return Array.from({ length: len }, (_, i) => i);
  const out = [];
  for (let i = 0; i < n; i++) {
    const r = Math.round(1 + (len - 1) * i / (n - 1)) - 1;
    if (out.length === 0 || out[out.length - 1] !== r) out.push(r);
  }
  return out;
}

const sub = (v, idx) => idx.map(i => v[i]);

// Wire indices for a cislunar track, keeping the flyby at full log resolution.
function flyby_idx(L, n, { near = 2.0e7 } = {}) {
  const m = L.t.length;
  if (m <= n) return Array.from({ length: m }, (_, i) => i);
  const nearidx = [], faridx = [];
  for (let i = 0; i < m; i++) (L.d_moon[i] < near ? nearidx : faridx).push(i);
  if (nearidx.length === 0) return deci_idx(m, n);
  const bnear = Math.min(nearidx.length, Math.max(1, Math.floor((2 * n) / 3)));
  const bfar = Math.max(2, n - bnear);
  const sel = [...sub(nearidx, deci_idx(nearidx.length, bnear)),
               ...(faridx.length === 0 ? [] : sub(faridx, deci_idx(faridx.length, bfar)))];
  sel.sort((a, b) => a - b);
  return [...new Set(sel)];
}

// Shared 3D-scene payload: the pad-to-wherever track in true ECI geometry.
function scene_payload(asc, cis) {
  let cisd = null;
  if (cis !== null && cis !== undefined) {
    const L = cis.log;
    const k = Math.max(2, Math.floor(L.t.length / 3));
    const nrm = vunit(vcross([L.mx[0], L.my[0], L.mz[0]],
                             [L.mx[k - 1], L.my[k - 1], L.mz[k - 1]]));
    const idx = flyby_idx(L, 1600);
    const px = [], py = [], pz = [], vx = [], vy = [], vz = [], dm = [];
    const mx = [], my = [], mz = [], tt = [], pp = [], lat = [], lon = [];
    for (const i of idx) {
      px.push(L.rx[i] / 1e6); py.push(L.ry[i] / 1e6); pz.push(L.rz[i] / 1e6);
      vx.push(L.vx[i] / 1e3); vy.push(L.vy[i] / 1e3); vz.push(L.vz[i] / 1e3);
      dm.push(L.d_moon[i] / 1e6);
      mx.push(L.mx[i] / 1e6); my.push(L.my[i] / 1e6); mz.push(L.mz[i] / 1e6);
      tt.push(L.t[i]); pp.push(L.phase[i]);
      const [la, lo] = geodetic_from_ecef(
        rot_z([L.rx[i], L.ry[i], L.rz[i]], earth_rotation_angle(0.0, L.t[i])));
      lat.push(rad2deg_(la)); lon.push(rad2deg_(lo));
    }
    cisd = { t: tt, x: px, y: py, z: pz, vx, vy, vz, dm, mx, my, mz, ph: pp,
             lat, lon, n: [nrm[0], nrm[1], nrm[2]] };
  }
  const AL = asc.log;
  const aidx = deci_idx(AL.t.length, 400);
  const asc3d = { t: sub(AL.t, aidx), x: sub(AL.rx, aidx).map(x => x / 1e6),
                  y: sub(AL.ry, aidx).map(x => x / 1e6),
                  z: sub(AL.rz, aidx).map(x => x / 1e6) };
  const ascent = {
    t: deci(sub(AL.t, aidx), 400),
    h: deci(sub(AL.h, aidx).map(x => x / 1e3), 400),
    v: deci(sub(AL.vrel, aidx), 400),
    qbar: deci(sub(AL.qbar, aidx).map(x => x / 1e3), 400),
    gamma: deci(sub(AL.gamma, aidx), 400),
    mach: deci(sub(AL.mach, aidx), 400),
    thrust: deci(sub(AL.thrust, aidx), 400),
    m: deci(sub(AL.m, aidx), 400),
    dr: deci(sub(AL.downrange, aidx), 400),
    lat: deci(sub(AL.lat, aidx), 400).map(rad2deg_),
    lon: deci(sub(AL.lon, aidx), 400).map(rad2deg_),
    g: deci(sub(AL.thrust, aidx).map((x, i) => x / sub(AL.m, aidx)[i]).map(x => x / G0), 400),
  };
  return { cis: cisd, asc3d, ascent };
}

// Serialize an rcs_budget for the client.
function rcs_payload(b) {
  return {
    t: b.t,
    used: b.used,
    remaining: b.used.map(u => b.capacity - u),
    limit_cycle: b.limit_cycle, dump: b.dump, hold: b.hold, slews: b.slews,
    settling: b.settling, total: b.total, margin: b.margin, capacity: b.capacity,
    n_slews: b.n_slews, slew_s: b.slew_time, disturbance_nm: b.disturbance_torque,
    events: b.events.map(e => ({ t: e.t, kg: e.kg, what: e.what })),
    model: 'analytic attitude budget: continuous deadband hold (or ' +
           'disturbance-momentum dumping, whichever governs) plus discrete ' +
           'slews and ullage settling at their own epochs. Individual ' +
           'valve pulses are not resolved.',
  };
}

// Axial length [m] of the coasting stack — kick stage plus the payload on it.
const cruise_body_length = (lv, pod_d) =>
  stage_length(lv.stages[lv.stages.length - 1], core_diameter(lv)) + pod_d;

function cruise_budget(lv, cis, pod_m, pod_d, { t_events = [] } = {}) {
  const [, I_t] = cruise_inertia(lv, cis.m, pod_d, { payload_mass: pod_m });
  const sys = sized_kick_rcs(I_t,
    stage_diameter(lv.stages[lv.stages.length - 1], core_diameter(lv)) / 2, cis.m);
  return cruise_rcs_budget(sys, I_t, { duration: Math.max(cis.t - cis.t_tli, 0.0),
    t_tli: cis.t_tli, t_events });
}

function orbit_budget(lv, eo, asc, pod_m, pod_d) {
  const [I_r, I_t] = cruise_inertia(lv, eo.m, pod_d, { payload_mass: pod_m });
  const sys = sized_kick_rcs(I_t,
    stage_diameter(lv.stages[lv.stages.length - 1], core_diameter(lv)) / 2, eo.m);
  const el = eo.elements;
  const a = el.a > 0 ? el.a : (el.rp + el.ra) / 2;
  return orbit_rcs_budget(sys, I_t, I_r, {
    duration: Math.max(eo.t - asc.t, 0.0), t0: asc.t,
    t_burns: eo.burns.map(b => b.t_ign),
    alt: Math.max(a - RE_MEAN, 0.0), v: Math.sqrt(MU_EARTH / Math.max(a, 1.0)),
    area: lv.sref, body_length: cruise_body_length(lv, pod_d),
    atmosphere: USSA76,
  });
}

// The powered descent in a frame anchored at the touchdown point.
function descent_local(ls, idx = ls.descent.log.t.map((_, i) => i)) {
  const eph = ls.eph;
  const D = ls.descent.log;
  const t_td = ls.t_touchdown;
  const utd = vunit(moonfixed(ls.descent.r, t_td, eph));
  const r0 = [D.x[0], D.y[0], D.z[0]];
  const r1 = [D.x[1], D.y[1], D.z[1]];
  const hf = vunit(vcross(moonfixed(r0, ls.t_pdi, eph),
                          moonfixed(vsub(r1, r0), ls.t_pdi, eph)));
  const ed = vunit(vcross(hf, utd));
  const ec = vcross(ed, utd);
  const lx = [], ly = [], lz = [];
  for (const i of idx) {
    const r = [D.x[i], D.y[i], D.z[i]];
    const uf = vunit(moonfixed(r, ls.t_pdi + D.t[i], eph));
    const b = Math.asin(Math.min(1, Math.max(-1, vdot(uf, ec))));
    const a = Math.atan2(vdot(uf, ed), vdot(uf, utd));
    lx.push(R_MOON * a); lz.push(R_MOON * b); ly.push(vnorm(r) - R_MOON);
  }
  return { lx, ly, lz, u: utd, ed, ec,
           earth: [ed[0], utd[0], ec[0]], earth_d: A_MOON };
}

// Terrain parameters, so the viewer draws the ground the descent was flown over.
const terrain_payload = tr => tr === null || tr === undefined
  ? { seed: 0, relief: 0.0, d_max: 1.0, classes: 0, ratio: 2.6, density: 0.0, rough: 0.0 }
  : { seed: Math.trunc(tr.seed), relief: tr.relief, d_max: tr.d_max,
      classes: tr.classes, ratio: tr.ratio, density: tr.density, rough: tr.rough };

// Ascent events, named by the vehicle's own stages.
const ascent_events = asc => asc.events.map(e =>
  ({ phase: 'ascent', name: String(e.name), t: e.t }));

// Launch time of day. `theta_g0` is the Greenwich sidereal angle at liftoff,
// so one hour on the clock is OMEGA_EARTH * 3600 rad of Earth rotation. Zero
// is the epoch every mission was always flown at; 0-24 h walks the launch site
// once around the globe, which is exactly what shifts the ground track.
const launch_theta = p => OMEGA_EARTH * 3600.0 * getf(p, 'launch_h', 0.0);

// Launch sites. Latitude is what a mission can reach directly: a prograde
// orbit cannot have an inclination below the site latitude, so an equatorial
// target from Baikonur is an honest miss rather than a silent re-aim. The
// lunar missions pick the Moon plane to match the ascent, so they fly from
// anywhere; only the ground track and the trans-lunar phasing change.
export const LAUNCH_SITES = {
  cape:        { name: 'Cape Canaveral', lat: 28.5,  lon: -80.6 },
  vandenberg:  { name: 'Vandenberg',     lat: 34.6,  lon: -120.6 },
  wallops:     { name: 'Wallops',        lat: 37.9,  lon: -75.5 },
  baikonur:    { name: 'Baikonur',       lat: 45.9,  lon: 63.3 },
  kourou:      { name: 'Kourou',         lat: 5.2,   lon: -52.8 },
  tanegashima: { name: 'Tanegashima',    lat: 30.4,  lon: 131.0 },
  sriharikota: { name: 'Sriharikota',    lat: 13.7,  lon: 80.2 },
  wenchang:    { name: 'Wenchang',       lat: 19.6,  lon: 110.9 },
  mahia:       { name: 'Mahia',          lat: -39.3, lon: 177.9 },
};
const launch_site = p => LAUNCH_SITES[gets(p, 'site', 'cape')] || LAUNCH_SITES.cape;

// Lunar landing sites, by selenographic latitude and longitude in degrees —
// the real coordinates of the sites the missions reached. The coplanar Moon
// puts its equator in the transfer plane, so unlike the real Moon it hands the
// arriving vehicle no free inclination: reaching a site costs a real combined
// LOI/plane-change burn and a phasing wait, exactly as `target_parking` in
// landing.js computes. `auto` keeps the old free-return site.
export const TARGET_SITES = {
  auto:          { name: 'Free return (auto)',            lat: NaN,   lon: NaN },
  tranquillity:  { name: 'Mare Tranquillitatis · Apollo 11', lat: 0.674, lon: 23.473 },
  procellarum:   { name: 'Oceanus Procellarum · Apollo 12',  lat: -3.012, lon: -23.422 },
  framauro:      { name: 'Fra Mauro · Apollo 14',         lat: -3.646, lon: -17.472 },
  hadley:        { name: 'Hadley Rille · Apollo 15',      lat: 26.132, lon: 3.634 },
  descartes:     { name: 'Descartes · Apollo 16',         lat: -8.973, lon: 15.501 },
  taurus:        { name: 'Taurus–Littrow · Apollo 17',    lat: 20.191, lon: 30.772 },
  fecunditatis:  { name: 'Mare Fecunditatis · Luna 16',   lat: 0.68,  lon: 56.30 },
  procellarum9:  { name: 'Oceanus Procellarum · Luna 9',  lat: 7.13,  lon: -64.37 },
  aitken:        { name: 'South Pole–Aitken · Chang’e 4', lat: -45.5, lon: 177.6 },
};
const target_site = p => TARGET_SITES[gets(p, 'target', 'auto')] || TARGET_SITES.auto;

// Fly the lunar landing mission for the panel.
function panel_landing(p, onProgress) {
  const lnd = lander_from_params(p);
  const lv = landing_vehicle_from_params(p, lnd);
  const real_moon = !getb(p, 'plain_moon', false);
  const terr = real_moon ? lunarTerrain() : null;
  const ts = target_site(p);
  const aiming = Number.isFinite(ts.lat);
  const ls = moonlanding({
    lander: lnd, lv, terrain: terr,
    field: real_moon ? lunarGravity() : null,
    nav: real_moon ? descentNav() : null,
    hazard: real_moon ? hazardScan() : null,
    target_lat: aiming ? deg2rad_(ts.lat) : NaN,
    target_lon: aiming ? deg2rad_(ts.lon) : NaN,
    h_park: getf(p, 'h_park_km', 200.0) * 1e3,
    h_moon_park: getf(p, 'h_moon_park_km', 100.0) * 1e3,
    h_pdi: getf(p, 'h_pdi_km', 15.0) * 1e3,
    n_rev: Math.min(12, Math.max(0, Math.round(getf(p, 'n_rev', 1.0)))),
    inclination: deg2rad_(getf(p, 'incl_deg', 28.5)),
    hp_return: getf(p, 'hp_return_km', 50.0) * 1e3,
    kick_angle: kick_rad(p),
    optimize_kick: opt_kick(p),
    theta_g0: launch_theta(p),
    site_lat: deg2rad_(launch_site(p).lat),
    site_lon: deg2rad_(launch_site(p).lon),
    onProgress,
  });
  const asc = ls.ascent, cis = ls.cislunar, d = ls.descent;
  const target_miss = aiming ? R_MOON * Math.acos(Math.max(-1, Math.min(1,
    Math.sin(ls.lat_land) * Math.sin(deg2rad_(ts.lat)) +
    Math.cos(ls.lat_land) * Math.cos(deg2rad_(ts.lat)) * Math.cos(ls.lon_land - deg2rad_(ts.lon))))) : NaN;
  const el = asc.elements;
  const sc = scene_payload(asc, cis);

  const O = ls.orbit;
  const oidx = deci_idx(O.t.length, 900);
  const D = d.log;
  const didx = deci_idx(D.t.length, 700);

  // Selenographic sub-points for the Moon ground track: latitude and longitude
  // in the tidally-locked Moon-fixed frame, longitude 0 being the sub-Earth
  // meridian. The orbit log carries absolute mission time; the powered descent
  // is logged relative to PDI.
  const orbit_ll = oidx.map(i =>
    selenographic([O.x[i] * 1e3, O.y[i] * 1e3, O.z[i] * 1e3], O.t[i], ls.eph).map(rad2deg_));
  const desc_ll = didx.map(i =>
    selenographic([D.x[i], D.y[i], D.z[i]], ls.t_pdi + D.t[i], ls.eph).map(rad2deg_));

  const events = ascent_events(asc);
  events.push({ phase: 'cislunar', name: 'tli_ignition', t: cis.t_tli });
  events.push({ phase: 'cislunar', name: 'tli_cutoff', t: cis.t_tli + cis.burn_duration });
  events.push({ phase: 'lunar', name: 'loi', t: ls.t_loi });
  events.push({ phase: 'lunar', name: 'doi', t: ls.t_doi });
  events.push({ phase: 'lunar', name: 'pdi', t: ls.t_pdi });
  if (Number.isFinite(d.t_gate))
    events.push({ phase: 'lunar', name: 'high_gate', t: ls.t_pdi + d.t_gate });
  events.push({ phase: 'lunar', name: String(d.outcome), t: ls.t_touchdown });

  const prop_margin = cis.m - (ls.lv.stages[ls.lv.stages.length - 1].mdry + ls.lv.payload_mass);
  const rcs = rcs_payload(cruise_budget(ls.lv, cis, ls.lv.payload_mass,
    lander_from_params(p).diameter, { t_events: [ls.t_loi, ls.t_doi] }));

  return {
    ok: true, mode: 'landing',
    metrics: {
      on_target: d.outcome === 'touchdown' && (!aiming || target_miss <= 10e3),
      outcome: String(d.outcome),
      liftoff_t: liftoff_mass(ls.lv) / 1e3,
      park_perigee_km: (el.rp - RE_MEAN) / 1e3,
      park_apogee_km: (el.ra - RE_MEAN) / 1e3,
      incl_deg: rad2deg_(el.i),
      tli_dv: cis.dv_tli,
      tli_burn_s: cis.burn_duration,
      prop_margin_kg: prop_margin,
      rcs_used_kg: rcs.used[rcs.used.length - 1],
      rcs_margin_kg: rcs.remaining[rcs.remaining.length - 1],
      lander_wet_t: lander_mass(lnd) / 1e3,
      lander_dv: lander_dv(lnd),
      perilune_km: cis.perilune_alt / 1e3,
      t_perilune_d: cis.t_perilune / 86400,
      loi_dv: ls.dv_loi,
      doi_dv: ls.dv_doi,
      braking_dv: d.dv_braking,
      terminal_dv: d.dv_terminal,
      descent_dv: d.dv_braking + d.dv_terminal,
      descent_s: d.t_touchdown,
      gate_s: d.t_gate,
      downrange_km: d.downrange / 1e3,
      touchdown_v: d.v_vertical,
      touchdown_vh: d.v_horizontal,
      min_throttle_pct: 100 * d.min_throttle,
      prop_left_kg: d.prop_left,
      hover_s: d.hover_s,
      land_lat: rad2deg_(ls.lat_land),
      land_lon: rad2deg_(ls.lon_land),
      target_lat: aiming ? ts.lat : NaN,
      target_lon: aiming ? ts.lon : NaN,
      target_miss_km: target_miss / 1e3,
      ground_elev_m: d.elev,
      ground_slope_deg: rad2deg_(d.slope),
      site_score_deg: Number.isNaN(d.site_score) ? 0.0 : rad2deg_(d.site_score),
      site_was_deg: Number.isNaN(d.site_score_nominal) ? 0.0 : rad2deg_(d.site_score_nominal),
      redesignate_m: d.redesignated,
      nav_err_m: d.nav_err,
      nav_alt_err_m: d.nav_dh,
      t_pdi_d: ls.t_pdi / 86400,
      t_days: ls.t_touchdown / 86400,
    },
    cis: sc.cis, asc3d: sc.asc3d, ascent: sc.ascent,
    rcs,
    ent3d: { t: [], x: [], y: [], z: [] },
    sites: { launch_lat: rad2deg_(ls.guid.site_lat),
             launch_lon: rad2deg_(ls.guid.site_lon) },
    moon: {
      r_km: R_MOON / 1e3,
      orbit: { t: sub(O.t, oidx), x: sub(O.x, oidx).map(x => x / 1e3),
               y: sub(O.y, oidx).map(x => x / 1e3), z: sub(O.z, oidx).map(x => x / 1e3),
               ph: sub(O.phase, oidx),
               lat: orbit_ll.map(x => x[0]), lon: orbit_ll.map(x => x[1]) },
      descent: { x: sub(D.x, didx).map(x => x / 1e3),
                 y: sub(D.y, didx).map(x => x / 1e3),
                 z: sub(D.z, didx).map(x => x / 1e3),
                 lat: desc_ll.map(x => x[0]), lon: desc_ll.map(x => x[1]) },
    },
    descent: {
      t: sub(D.t, didx), h: sub(D.h, didx).map(x => x / 1e3),
      dr: sub(D.downrange, didx).map(x => x / 1e3), v: sub(D.v, didx),
      vh: sub(D.vh, didx), vv: sub(D.vv, didx),
      thr: sub(D.throttle, didx).map(x => 100 * x),
      pitch: sub(D.pitch, didx).map(rad2deg_), elev: sub(D.elev, didx),
      navdh: sub(D.nav_dh, didx), m: sub(D.m, didx),
    },
    site: Object.assign(descent_local(ls, didx), {
      terrain: terrain_payload(terr), diameter: ls.lander.diameter,
      t_pdi: ls.t_pdi, t_gate: ls.t_pdi + d.t_gate, t_td: ls.t_touchdown }),
    events,
  };
}

// The entry leg's payload blocks, shared by every mission that ends in one.
function entry_payload(out, metrics, events, ent) {
  const EL = ent.log;
  const eidx = deci_idx(EL.t.length, 500);
  const ex3 = [], ey3 = [], ez3 = [], et3 = [];
  for (const i of eidx) {
    const re_ = ecef_from_geodetic(EL.lat[i], EL.lon[i], EL.h[i]);
    const th = earth_rotation_angle(0.0, EL.t[i]);
    const reci = rot_z(re_, -th);
    ex3.push(reci[0] / 1e6); ey3.push(reci[1] / 1e6); ez3.push(reci[2] / 1e6);
    et3.push(EL.t[i]);
  }
  const ei = ent.events.findIndex(e => e.name === 'entry_interface');
  metrics.ei_v_ms = ei < 0 ? NaN : ent.events[ei].vrel;
  metrics.peak_g = ent.peak_gload;
  metrics.peak_q_wcm2 = ent.peak_qdot / 1e4;
  metrics.heat_mj = ent.heat_load / 1e6;
  metrics.splash_lat = rad2deg_(ent.lat_splash);
  metrics.splash_lon = rad2deg_(ent.lon_splash);
  metrics.v_splash = ent.v_splash;
  for (const e of ent.events)
    events.push({ phase: 'entry', name: String(e.name), t: e.t });
  out.ent3d = { t: et3, x: ex3, y: ey3, z: ez3 };
  out.entry = {
    t: deci(sub(EL.t, eidx).map((x, i) => x - EL.t[eidx[0]]), 500),
    h: deci(sub(EL.h, eidx).map(x => x / 1e3), 500),
    v: deci(sub(EL.vrel, eidx), 500),
    mach: deci(sub(EL.mach, eidx), 500),
    qbar: deci(sub(EL.qbar, eidx).map(x => x / 1e3), 500),
    g: deci(sub(EL.gload, eidx), 500),
    alpha: deci(sub(EL.alpha, eidx), 500).map(rad2deg_),
    qrate: deci(sub(EL.qrate, eidx), 500).map(rad2deg_),
    rho: deci(sub(EL.rho, eidx), 500),
    q: deci(sub(EL.qdot_conv, eidx).map((x, i) => (x + EL.qdot_rad[eidx[i]]) / 1e4), 500),
    heat: deci(sub(EL.qload, eidx).map(x => x / 1e6), 500),
    twall: deci(sub(EL.twall, eidx), 500),
    lat: deci(sub(EL.lat, eidx), 500).map(rad2deg_),
    lon: deci(sub(EL.lon, eidx), 500).map(rad2deg_),
  };
  out.sites.splash_lat = rad2deg_(ent.lat_splash);
  out.sites.splash_lon = rad2deg_(ent.lon_splash);
}

// Fly an Earth-orbit mission for the panel.
function panel_orbit(p, onProgress) {
  let tkey = gets(p, 'orbit', 'leo');
  if (!(tkey in ORBITS) && tkey !== 'custom') tkey = 'leo';
  const eo = earthorbit({
    target: tkey,
    lv: lv_from_params(p),
    pod_mass: payload_total(p),
    h_park: getf(p, 'h_park_km', 200.0) * 1e3,
    perigee_alt: tkey === 'custom'
      ? Math.min(100000.0, Math.max(100.0, getf(p, 'orbit_perigee_km', 200.0))) * 1e3 : NaN,
    apogee_alt: tkey === 'custom'
      ? Math.min(100000.0, Math.max(100.0, getf(p, 'orbit_apogee_km', 200.0))) * 1e3 : NaN,
    inclination: tkey === 'custom'
      ? deg2rad_(Math.min(180.0, Math.max(0.0, getf(p, 'orbit_incl_deg', 28.5)))) : NaN,
    n_orbits: Math.min(16.0, Math.max(0.25, getf(p, 'n_orbits', 2.0))),
    deorbit: getb(p, 'deorbit', false),
    hp_entry: getf(p, 'hp_entry_km', 25.0) * 1e3,
    kick_angle: kick_rad(p),
    optimize_kick: opt_kick(p),
    theta_g0: launch_theta(p),
    site_lat: deg2rad_(launch_site(p).lat),
    site_lon: deg2rad_(launch_site(p).lon),
    onProgress,
    strict: false,
  });
  const asc = eo.ascent, ent = eo.entry;
  const el = asc.elements;
  const haslog = eo.log.t.length > 2;
  const sc = scene_payload(asc, haslog ? { log: eo.log } : null);

  const events = ascent_events(asc);
  for (const b of eo.burns) {
    events.push({ phase: 'orbit', name: `${b.name}_ignition`, t: b.t_ign });
    events.push({ phase: 'orbit', name: `${b.name}_cutoff`, t: b.t_ign + b.duration });
  }
  if (ent !== null) events.push({ phase: 'orbit', name: 'entry_handoff', t: eo.entry_scn.t0 });

  const metrics = {
    liftoff_t: liftoff_mass(eo.lv) / 1e3,
    t_days: (ent !== null ? ent.t_splash : eo.t) / 86400,
    on_target: eo.on_target,
    prop_margin_kg: eo.m - (eo.lv.stages[eo.lv.stages.length - 1].mdry + eo.lv.payload_mass),
    orbit_rp_km: (eo.elements.rp - RE_MEAN) / 1e3,
    orbit_ra_km: (eo.elements.ra - RE_MEAN) / 1e3,
    orbit_incl_deg: rad2deg_(eo.elements.i),
    period_min: eo.elements.a > 0
      ? 2 * Math.PI * Math.sqrt(eo.elements.a ** 3 / MU_EARTH) / 60 : NaN,
    burn_dv_total: eo.burns.reduce((s, b) => s + b.dv, 0.0),
  };
  if (asc.reached_orbit) {
    metrics.park_perigee_km = (el.rp - RE_MEAN) / 1e3;
    metrics.park_apogee_km = (el.ra - RE_MEAN) / 1e3;
    metrics.incl_deg = rad2deg_(el.i);
  }
  const rcs = haslog
    ? rcs_payload(orbit_budget(eo.lv, eo, asc, eo.lv.payload_mass,
                               pod_diameter(p, spacecraft_mass(p))))
    : null;
  if (rcs !== null) {
    metrics.rcs_used_kg = rcs.used[rcs.used.length - 1];
    metrics.rcs_margin_kg = rcs.remaining[rcs.remaining.length - 1];
  }
  const out = {
    ok: true, mode: 'orbit',
    outcome: (eo.outcome === 'on_orbit' || eo.outcome === 'splashdown')
      ? 'nominal' : String(eo.outcome),
    metrics,
    asc3d: sc.asc3d, ascent: sc.ascent, events,
    achievements: { orbit: asc.reached_orbit && haslog, tli: false, flyby: false,
                    return: false, entry: ent !== null },
    sites: { launch_lat: rad2deg_(eo.guid.site_lat),
             launch_lon: rad2deg_(eo.guid.site_lon) },
    orbit: {
      target: String(eo.target.name),
      target_rp_km: eo.target.perigee_alt / 1e3,
      target_ra_km: eo.target.apogee_alt / 1e3,
      target_incl_deg: rad2deg_(eo.target.inclination),
      burns: eo.burns.map(b => ({ name: String(b.name), t_ign: b.t_ign,
        duration_s: b.duration, dv_plan: b.dv_plan, dv: b.dv })),
    },
  };
  if (haslog) { out.cis = sc.cis; out.rcs = rcs; }
  if (ent !== null) entry_payload(out, metrics, events, ent);
  return out;
}

// Fly a suborbital mission for the panel.
function panel_suborbital(p, onProgress) {
  const prof = gets(p, 'sub_profile', 'hop') === 'downrange' ? 'downrange' : 'hop';
  const sb = suborbital({
    profile: prof,
    lv: lv_from_params(p),
    pod_mass: payload_total(p),
    apogee: Math.min(3000.0, Math.max(5.0, getf(p, 'sub_apogee_km', 100.0))) * 1e3,
    downrange: Math.min(12000.0, Math.max(10.0, getf(p, 'sub_range_km', 400.0))) * 1e3,
    loft: deg2rad_(Math.min(85.0, Math.max(5.0, getf(p, 'sub_loft_deg', 40.0)))),
    azimuth: deg2rad_(Math.min(360.0, Math.max(0.0, getf(p, 'sub_azimuth_deg', 90.0)))),
    kick_angle: kick_rad(p),
    theta_g0: launch_theta(p),
    site_lat: deg2rad_(launch_site(p).lat),
    site_lon: deg2rad_(launch_site(p).lon),
    onProgress,
    strict: false,
  });
  const asc = sb.ascent, ent = sb.entry;
  const sc = scene_payload(asc, null);
  const events = ascent_events(asc);

  const metrics = {
    liftoff_t: liftoff_mass(sb.lv) / 1e3,
    apogee_km: sb.apogee / 1e3,
    target_apogee_km: sb.target_apogee / 1e3,
    range_km: sb.range / 1e3,
    target_range_km: sb.target_range / 1e3,
    t_apogee_s: sb.t_apogee - asc.t,
    cutoff_h_km: asc.h_cut / 1e3,
    cutoff_gamma_deg: rad2deg_(asc.gamma_cut),
    prop_margin_kg: asc.prop_left.reduce((s, x) => s + x, 0.0),
    apogee_err_km: (sb.apogee - sb.target_apogee) / 1e3,
    range_err_km: Number.isNaN(sb.target_range) ? NaN : (sb.range - sb.target_range) / 1e3,
    on_target: sb.outcome === 'splashdown' &&
      Math.abs(prof === 'hop' ? sb.apogee - sb.target_apogee : sb.range - sb.target_range) <
        0.004 * (prof === 'hop' ? sb.target_apogee : sb.target_range),
  };
  metrics.t_days = (ent !== null ? ent.t_splash : asc.t) / 86400;

  const out = {
    ok: true, mode: 'suborbital',
    outcome: sb.outcome === 'splashdown' ? 'nominal' : String(sb.outcome),
    metrics, asc3d: sc.asc3d, ascent: sc.ascent, events,
    achievements: { orbit: false, tli: false, flyby: false, return: false,
                    entry: ent !== null },
    sites: { launch_lat: rad2deg_(sb.guid.site_lat),
             launch_lon: rad2deg_(sb.guid.site_lon) },
    suborbital: { profile: String(sb.profile), apogee_km: sb.apogee / 1e3,
                  range_km: sb.range / 1e3 },
  };
  if (ent !== null) {
    entry_payload(out, metrics, events, ent);
    const EL = ent.log;
    let top = 0;
    for (let i = 1; i < EL.h.length; i++) if (EL.h[i] > EL.h[top]) top = i;
    let ei = -1;
    for (let i = top + 1; i < EL.h.length; i++) if (EL.h[i] <= 120.0e3) { ei = i; break; }
    if (!events.some(e => e.name === 'entry_interface'))
      events.push({ phase: 'entry', name: 'entry_interface', t: EL.t[ei < 0 ? top : ei] });
    events.push({ phase: 'entry', name: 'apogee', t: EL.t[top] });
  }
  return out;
}

// The mission the panel is configured for.
export function panel_mission(p, onProgress) {
  const mode = mission_mode(p);
  if (mode === 'landing') return panel_landing(p, onProgress);
  if (mode === 'orbit') return panel_orbit(p, onProgress);
  if (mode === 'suborbital') return panel_suborbital(p, onProgress);
  const ms = moonshot({
    pod_mass: payload_total(p),
    pod_diameter: pod_diameter(p, spacecraft_mass(p)),
    h_park: getf(p, 'h_park_km', 200.0) * 1e3,
    hp_moon: getf(p, 'hp_moon_km', 2000.0) * 1e3,
    hp_return: getf(p, 'hp_return_km', 50.0) * 1e3,
    inclination: deg2rad_(getf(p, 'incl_deg', 28.5)),
    lv: lv_from_params(p),
    tli_mag_err: getf(p, 'tli_mag_err_pct', 0.0) / 100,
    tli_point_err: deg2rad_(getf(p, 'tli_point_err_deg', 0.0)),
    kick_angle: kick_rad(p),
    optimize_kick: opt_kick(p),
    theta_g0: launch_theta(p),
    site_lat: deg2rad_(launch_site(p).lat),
    site_lon: deg2rad_(launch_site(p).lon),
    onProgress,
    strict: false,
  });
  const asc = ms.ascent, cis = ms.cislunar, ent = ms.entry;
  const el = asc.elements;
  const sc = scene_payload(asc, cis);

  const events = ascent_events(asc);
  const metrics = {
    liftoff_t: liftoff_mass(ms.lv) / 1e3,
    t_days: (ent !== null ? ent.t_splash : cis !== null ? cis.t : asc.t) / 86400,
    design_status: String(ms.design_status),
  };
  const out = {
    ok: true, mode: 'flyby',
    outcome: ent !== null
      ? (ent.terminated === 'splashdown' ? 'nominal' : 'entry_' + String(ent.terminated))
      : (ms.design_status === 'stalled' || ms.design_status === 'unreachable')
        ? 'design_failed'
        : cis !== null ? String(cis.outcome) : 'ascent_failed',
    metrics, asc3d: sc.asc3d, ascent: sc.ascent, events,
    achievements: {
      orbit: asc.reached_orbit,
      tli: cis !== null && Number.isFinite(cis.t_tli),
      flyby: cis !== null &&
             !(ms.design_status === 'stalled' || ms.design_status === 'unreachable') &&
             Number.isFinite(cis.t_perilune) && cis.perilune_alt > 0.0 &&
             cis.log.phase.some(x => x === 3),
      return: cis !== null &&
              !(ms.design_status === 'stalled' || ms.design_status === 'unreachable') &&
              cis.log.phase.some(x => x === 3),
      entry: ent !== null,
    },
    sites: { launch_lat: rad2deg_(ms.guid.site_lat),
             launch_lon: rad2deg_(ms.guid.site_lon) },
  };
  if (asc.reached_orbit) {
    metrics.park_perigee_km = (el.rp - RE_MEAN) / 1e3;
    metrics.park_apogee_km = (el.ra - RE_MEAN) / 1e3;
    metrics.incl_deg = rad2deg_(el.i);
  }

  if (cis !== null) {
    out.cis = sc.cis;
    const rcs = rcs_payload(ms.cruise !== null ? ms.cruise.rcs
      : cruise_budget(ms.lv, cis, ms.lv.payload_mass, pod_diameter(p, spacecraft_mass(p))));
    out.rcs = rcs;
    metrics.tli_dv = cis.dv_tli;
    metrics.tli_burn_s = cis.burn_duration;
    metrics.prop_margin_kg = cis.m - (ms.lv.stages[ms.lv.stages.length - 1].mdry + ms.lv.payload_mass);
    metrics.perilune_km = cis.perilune_alt / 1e3;
    metrics.t_perilune_d = cis.t_perilune / 86400;
    metrics.vac_perigee_km = cis.vac_perigee_alt / 1e3;
    metrics.tcm_dv = ms.cruise === null ? null : ms.cruise.tcm_dv;
    metrics.tcm_prop_kg = ms.cruise === null ? null : ms.cruise.tcm_prop;
    metrics.rcs_used_kg = rcs.used[rcs.used.length - 1];
    metrics.rcs_margin_kg = rcs.remaining[rcs.remaining.length - 1];
    const hp_moon = getf(p, 'hp_moon_km', 2000.0) * 1e3;
    const hp_ret = getf(p, 'hp_return_km', 50.0) * 1e3;
    const graze = hp_moon < 10e3;
    const peri_ok = Math.abs(cis.perilune_alt - hp_moon) <= Math.max(0.08 * hp_moon, 300.0);
    metrics.on_target = graze ? peri_ok
      : (ent !== null && peri_ok &&
         Math.abs(cis.vac_perigee_alt - hp_ret) <= 20e3);
    metrics.grazing = graze;
    events.push({ phase: 'cislunar', name: 'tli_ignition', t: cis.t_tli });
    events.push({ phase: 'cislunar', name: 'tli_cutoff', t: cis.t_tli + cis.burn_duration });
    if (out.achievements.flyby)
      events.push({ phase: 'cislunar', name: 'perilune', t: cis.t_perilune });
    if (ent !== null)
      events.push({ phase: 'cislunar', name: 'entry_handoff', t: cis.t });
  } else {
    metrics.on_target = false;
  }

  if (ent !== null) entry_payload(out, metrics, events, ent);
  return out;
}

// --------------------------------------------------------------- geometry --

export function rocket_geometry(p) {
  const d = getf(p, 'diameter', 1.8);
  const lnd = mission_mode(p) === 'landing' ? lander_from_params(p) : null;
  const lv = lnd === null ? lv_from_params(p) : landing_vehicle_from_params(p, lnd);
  const [mesh, secs] = rocket_mesh(lv, {
    diameter: d, nseg: 36,
    pod_diameter: getf(p, 'pod_dia', 0.0),
    crewed: getb(p, 'crewed', true),
    payload_kind: lnd === null ? 'capsule' : 'lander',
    payload_diameter: lnd === null ? 0.0 : lnd.diameter,
  });
  const nst = lv.stages.length;
  const payload_above = k => stack_mass_above(lv, k + 1, { fairing: k === 1 });
  const dv = k => stage_dv(lv.stages[k - 1], payload_above(k));
  const twr = k => {
    const s = lv.stages[k - 1];
    return stage_thrust(s, k === 1 ? P0_SEA : 0.0) /
           ((payload_above(k) + s.mdry + s.mprop) * G0);
  };
  const pad_twr = pad_thrust(lv) / (liftoff_mass(lv) * G0);
  const bdv = b => b.count * b.stage.mprop * G0 * b.stage.isp_vac / liftoff_mass(lv);
  return {
    ok: true,
    name: String(lv.name),
    length: Math.max(...secs.map(s => s.x1)),
    diameter: d,
    payload: {
      kind: lnd !== null ? 'lander' : payload_is_bus(p) ? 'bus' : 'capsule',
      mass_kg: lv.payload_mass,
      spacecraft_kg: lnd === null ? spacecraft_mass(p) : lander_mass(lnd),
      cargo_kg: lnd === null ? cargo_mass(p) : 0.0,
      crewed: lnd === null && !payload_is_bus(p) && getb(p, 'crewed', true),
      diameter_m: lnd === null ? pod_diameter(p, spacecraft_mass(p)) : lnd.diameter,
    },
    liftoff_mass_kg: liftoff_mass(lv),
    liftoff_twr: pad_twr,
    total_dv_mps: Array.from({ length: nst }, (_, i) => dv(i + 1)).reduce((s, x) => s + x, 0.0) +
                  lv.boosters.reduce((s, b) => s + bdv(b), 0.0),
    boosters: lv.boosters.map(b => ({
      name: String(b.stage.name), count: b.count,
      propellant: String(b.stage.prop.name), engines: b.stage.n_engines,
      dry_kg: b.stage.mdry, prop_kg: b.stage.mprop,
      thrust_kn: b.stage.thrust_vac / 1e3, isp_s: b.stage.isp_vac,
      diameter_m: stage_diameter(b.stage, d), length_m: stage_length(b.stage, d),
      burn_s: stage_burn_time(b.stage), set_mass_kg: booster_mass(b),
      dv_mps: bdv(b), core_throttle: b.core_throttle,
      ignition_delay_s: b.ignition_delay, sep_delay_s: b.sep_delay,
    })),
    stages: lv.stages.map((s, i) => {
      const k = i + 1;
      return {
        name: String(s.name), propellant: String(s.prop.name), engines: s.n_engines,
        dry_kg: s.mdry, prop_kg: s.mprop, dry_est_kg: dry_estimate(s, d),
        dry_frac: s.mdry / (s.mdry + s.mprop),
        thrust_kn: s.thrust_vac / 1e3, isp_s: s.isp_vac,
        diameter_m: stage_diameter(s, d), length_m: stage_length(s, d),
        volume_m3: stage_volume(s), dv_mps: dv(k), twr: twr(k),
        burn_s: stage_burn_time(s),
        interstage_m: k < nst
          ? interstage_length(stage_diameter(s, d), stage_diameter(lv.stages[k], d)) : 0.0,
      };
    }),
    sections: secs.map(s => ({ name: String(s.name), x0: s.x0, x1: s.x1, t0: s.t0, t1: s.t1 })),
    tris: mesh.tris.map(t => [t[0][0], t[0][1], t[0][2],
                              t[1][0], t[1][1], t[1][2],
                              t[2][0], t[2][1], t[2][2]]),
  };
}

// ------------------------------------------------------------- sweep/solve --

export const SOLVE_METRICS = ['prop_margin_kg', 'peak_g', 'peak_q_wcm2', 't_days',
  'liftoff_t', 'park_apogee_km', 'v_splash', 'tli_dv', 'heat_mj'];
export const COMMANDED_METRICS = { perilune_km: 'perilune [km]',
  vac_perigee_km: 'return perigee [km]' };
export const SWEEP_METRICS = [...SOLVE_METRICS, ...Object.keys(COMMANDED_METRICS)];
export const SUBORBITAL_METRICS = ['apogee_km', 'range_km', 'peak_g', 'peak_q_wcm2',
  'v_splash', 'heat_mj', 'prop_margin_kg', 'liftoff_t', 'cutoff_h_km', 't_apogee_s'];
export const LANDING_METRICS = ['prop_left_kg', 'hover_s', 'descent_dv', 'loi_dv',
  'touchdown_v', 'touchdown_vh', 'downrange_km', 'prop_margin_kg', 'min_throttle_pct',
  'liftoff_t', 'tli_dv', 't_days', 'ground_slope_deg', 'ground_elev_m',
  'redesignate_m', 'nav_alt_err_m'];
export const ORBIT_METRICS = ['prop_margin_kg', 'orbit_rp_km', 'orbit_ra_km',
  'orbit_incl_deg', 'period_min', 'burn_dv_total', 'park_apogee_km',
  'liftoff_t', 't_days', 'rcs_used_kg', 'rcs_margin_kg'];

const solve_metrics = p => mission_mode(p) === 'landing' ? LANDING_METRICS
  : mission_mode(p) === 'suborbital' ? SUBORBITAL_METRICS
  : mission_mode(p) === 'orbit' ? [...ORBIT_METRICS,
      ...(getb(p, 'deorbit', false) ? ['peak_g', 'peak_q_wcm2', 'v_splash', 'heat_mj'] : [])]
  : SOLVE_METRICS;

// Numeric parameters that may be swept or solved for, for this stack height.
export function sweepable(p) {
  const out = ['pod_mass', 'bus_mass', 'cargo_mass', 'h_park_km', 'hp_moon_km',
    'hp_return_km', 'incl_deg', 'diameter', 'fairing', 'kick_deg'];
  if (mission_mode(p) === 'landing')
    out.push('l_dry', 'l_prop', 'l_thrust_kn', 'l_isp', 'l_throttle_min',
             'h_moon_park_km', 'h_pdi_km', 'n_rev');
  if (mission_mode(p) === 'suborbital')
    out.push('sub_apogee_km', 'sub_range_km', 'sub_loft_deg', 'sub_azimuth_deg');
  for (let k = 1; k <= n_stages(p); k++)
    for (const f of ['prop', 'dry', 'isp', 'thrust_kn', 'engines'])
      out.push(`s${k}_${f}`);
  if (n_boosters(p) !== 0)
    out.push('nboost', 'b_prop', 'b_dry', 'b_isp', 'b_thrust_kn', 'b_engines',
             'b_throttle', 'b_diameter');
  return out;
}

export function run_sweep(p, onProgress) {
  const param = p.sweep_param ?? 'pod_mass';
  if (!sweepable(p).includes(param))
    return { ok: false, error: `unknown sweep parameter: ${param}` };
  const lo = getf(p, 'sweep_min', 250.0);
  const hi = getf(p, 'sweep_max', 450.0);
  const nv = Math.min(41, Math.max(2, Math.trunc(getf(p, 'sweep_n', 9.0))));
  const vals = Array.from({ length: nv }, (_, i) => lo + (hi - lo) * i / (nv - 1));
  if (onProgress)
    onProgress({ ok: true, stage: 'sweep', detail: `launching ${nv} mission evaluations…`,
                 current: 0, total: nv });
  const runs = vals.map((v, i) => {
    const q = { ...p, [param]: String(v) };
    let out;
    try {
      const r = panel_mission(q);
      out = { ok: true, metrics: r.metrics };
    } catch (err) {
      out = { ok: false, error: String(err) };
    }
    if (onProgress)
      onProgress({ ok: true, stage: 'sweep', detail: `completed mission ${i + 1} of ${nv}`,
                   current: i + 1, total: nv });
    return out;
  });
  if (onProgress)
    onProgress({ ok: true, stage: 'complete', detail: 'parameter sweep complete',
                 current: nv, total: nv, done: true });
  return { ok: true, param, values: vals, runs };
}

export function run_solve(p, onProgress) {
  const param = p.solve_param ?? 'pod_mass';
  const metric = p.solve_metric ?? 'prop_margin_kg';
  if (!sweepable(p).includes(param))
    return { ok: false, error: `cannot solve for: ${param}` };
  if (metric in COMMANDED_METRICS)
    return { ok: false, error: `${metric} is commanded, not achieved — the designer already ` +
      `drives it to the target, so it does not respond to ${param}. ` +
      `Set "${COMMANDED_METRICS[metric]}" directly instead.` };
  if (!solve_metrics(p).includes(metric))
    return { ok: false, error: `cannot target metric: ${metric}` };
  const lo = getf(p, 'solve_min', 200.0);
  const hi = getf(p, 'solve_max', 600.0);
  const target = getf(p, 'solve_target', 0.0);
  const budget = Math.min(30, Math.max(4, Math.round(getf(p, 'solve_iters', 14.0))));
  const ftol = 1e-4 * Math.max(Math.abs(target), 1.0);
  if (onProgress)
    onProgress({ ok: true, stage: 'solve', detail: 'starting bracket search…',
                 current: 0, total: budget });
  let evals = 0;
  const res = find_root(x => {
    evals += 1;
    if (onProgress)
      onProgress({ ok: true, stage: 'solve',
                   detail: `evaluation ${evals} of up to ${budget} · ${param} = ${Number(x).toPrecision(5)}`,
                   current: evals, total: budget });
    const q = { ...p, [param]: String(x) };
    const v = panel_mission(q).metrics[metric];
    return v === null || v === undefined ? NaN : Number(v);
  }, lo, hi, { target, max_iter: budget, ftol });
  if (onProgress)
    onProgress({ ok: true, stage: 'complete', detail: 'target search complete',
                 current: res.iterations, total: budget, done: true });
  return {
    ok: true, param, metric, target, x: res.x, value: res.value,
    status: String(res.status), iterations: res.iterations,
    lo: res.lo, hi: res.hi, asked_lo: Math.min(lo, hi), asked_hi: Math.max(lo, hi),
    history: res.history.map(h => ({ x: h[0], value: h[1] })),
  };
}

// -------------------------------------------------------------- catalogue --

export function catalogue_payload() {
  const byKey = (a, b) => a[0] < b[0] ? -1 : a[0] > b[0] ? 1 : 0;
  return {
    propellants: Object.entries(PROPELLANTS).sort(byKey)
      .map(([k, v]) => ({ name: String(k), bulk: bulk_density(v) })),
    engines: Object.entries(ENGINES).sort(byKey).map(([k, v]) => ({
      name: String(k), thrust_kn: v.thrust_vac / 1e3, isp_vac: v.isp_vac,
      isp_sl: v.isp_sl, mass_kg: v.mass, propellant: String(v.prop.name) })),
    orbits: Object.entries(ORBITS).sort(byKey).map(([k, v]) => ({
      name: String(v.name), perigee_km: v.perigee_alt / 1e3,
      apogee_km: v.apogee_alt / 1e3, incl_deg: rad2deg_(v.inclination), note: v.note })),
    features: ['fairing_on', 'crewed', 'pod_dia', 'l_diameter', 'grazing',
               'flyby_wire', 'payload_kind', 'bus_mass', 'cargo_mass'],
    solve_metrics: SOLVE_METRICS, sweep_metrics: SWEEP_METRICS,
    landing_metrics: LANDING_METRICS, suborbital_metrics: SUBORBITAL_METRICS,
    orbit_metrics: ORBIT_METRICS,
    max_stages: 5,
  };
}

// Clean entry points for the pages.
export function panelRun(p, mode, onProgress) {
  const q = mode === undefined ? p : { ...p, mode };
  return panel_mission(q, onProgress);
}
export const run = panelRun;
export function panelGeometry(p) { return rocket_geometry(p); }
export function panelCatalogue() { return catalogue_payload(); }
export function panelSweep(p, onProgress) { return run_sweep(p, onProgress); }
export function panelSolve(p, onProgress) { return run_solve(p, onProgress); }
