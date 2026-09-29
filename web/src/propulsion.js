// Port of src/propulsion.jl: stages, strap-on sets and the stacked vehicle.
import { G0, P0_SEA } from './constants.js';
import { table1d } from './aerodynamics.js';
import { PROPELLANTS, bulk_density, sized_stage } from './engines.js';
import { stack_inertia } from './rigidbody.js';

// Stage(name, mdry, mprop, thrust_vac, isp_vac, ae, prop?, n_engines?, diameter?)
export const stage = (name, mdry, mprop, thrust_vac, isp_vac, ae,
                      prop = PROPELLANTS.kerolox, n_engines = 1, diameter = 0.0) =>
  ({ name, mdry, mprop, thrust_vac, isp_vac, ae, prop, n_engines, diameter });

export const stage_diameter = (st, vehicle_d) =>
  st.diameter > 0 ? st.diameter : vehicle_d;

export const stage_mdot = st => st.thrust_vac / (G0 * st.isp_vac);
export const stage_thrust = (st, pamb) =>
  Math.max(st.thrust_vac - st.ae * pamb, 0.2 * st.thrust_vac);
export const stage_burn_time = st => st.mprop / stage_mdot(st);
export const stage_dv = (st, mpayload) =>
  G0 * st.isp_vac * Math.log((st.mdry + st.mprop + mpayload) / (st.mdry + mpayload));

export const boosterSet = ({ stage: st, count = 2, ignition_delay = 0.0,
                             sep_delay = 0.0, core_throttle = 1.0 }) =>
  ({ stage: st, count, ignition_delay, sep_delay, core_throttle });

export const booster_mass = b => b.count * (b.stage.mdry + b.stage.mprop);
export const booster_thrust = (b, pamb) => b.count * stage_thrust(b.stage, pamb);
export const booster_mdot = b => b.count * stage_mdot(b.stage);

export const launchVehicle = ({ name = 'launcher', stages, fairing_mass,
                                payload_mass, sref, cd, boosters = [] }) =>
  ({ name, stages, fairing_mass, payload_mass, sref, cd, boosters });

export const core_diameter = lv => 2 * Math.sqrt(lv.sref / Math.PI);

export function frontal_area(lv, attached) {
  let a = lv.sref;
  const dc = core_diameter(lv);
  for (let i = 0; i < lv.boosters.length; i++) {
    if (!attached[i]) continue;
    const b = lv.boosters[i];
    a += b.count * Math.PI * (stage_diameter(b.stage, dc) / 2) ** 2;
  }
  return a;
}

export const stage_volume = st => st.mprop / bulk_density(st.prop);

export const barrel_length = (mprop, density, diameter) =>
  mprop / (density * Math.PI * (diameter / 2) ** 2) * 1.15 + 0.9 * diameter;

export const stage_length = (st, dc) =>
  barrel_length(st.mprop, bulk_density(st.prop), stage_diameter(st, dc));

export function cruise_inertia(lv, m_stack, payload_diameter,
                               { payload_mass = lv.payload_mass } = {}) {
  const kick = lv.stages[lv.stages.length - 1];
  const dc = core_diameter(lv);
  const dk = stage_diameter(kick, dc);
  const Lk = barrel_length(kick.mprop, bulk_density(kick.prop), dk);
  const mk = Math.max(m_stack - payload_mass, kick.mdry);
  const dp = Math.max(payload_diameter, 0.1);
  const Lp = dp;
  return stack_inertia([[mk, dk / 2, Lk, Lk / 2],
                        [payload_mass, dp / 2, Lp, Lk + Lp / 2]]);
}

export const liftoff_mass = lv =>
  lv.stages.reduce((s, x) => s + x.mdry + x.mprop, 0) + lv.fairing_mass +
  lv.payload_mass + lv.boosters.reduce((s, b) => s + booster_mass(b), 0);

export const pad_thrust = lv =>
  stage_thrust(lv.stages[0], P0_SEA) +
  lv.boosters.reduce((s, b) => s + (b.ignition_delay <= 0.0 ? booster_thrust(b, P0_SEA) : 0), 0);

export function stack_mass_above(lv, k, { fairing = true } = {}) {
  let m = lv.payload_mass + (fairing ? lv.fairing_mass : 0.0);
  for (let i = k; i <= lv.stages.length; i++)
    m += lv.stages[i - 1].mdry + lv.stages[i - 1].mprop;
  return m;
}

export const LV_CD_TABLE = table1d(
  [0.0, 0.6, 0.9, 1.05, 1.2, 1.6, 2.5, 4.0, 6.0, 10.0, 25.0],
  [0.28, 0.30, 0.42, 0.58, 0.55, 0.46, 0.36, 0.30, 0.26, 0.24, 0.22]);

export const LV_CD_BARE_DELTA = table1d(
  [0.0, 0.6, 0.9, 1.05, 1.2, 1.6, 2.5, 4.0, 6.0, 10.0, 25.0],
  [0.06, 0.09, 0.22, 0.34, 0.32, 0.24, 0.16, 0.11, 0.09, 0.08, 0.07]);

export function bare_payload_cd(pod_diameter, sref, { base = LV_CD_TABLE } = {}) {
  const dref = 2 * Math.sqrt(Math.max(Number(sref), 1e-9) / Math.PI);
  const frac = Math.min(1.0, Math.max(0.0, (pod_diameter / dref) ** 2));
  return table1d(base.x, base.y.map((v, i) => v + frac * LV_CD_BARE_DELTA.y[i]));
}

export function stack_sref(stage_diameters, pod_diameter, fairing) {
  let d = Math.max(...stage_diameters);
  if (!fairing) d = Math.max(d, pod_diameter);
  return Math.PI * (d / 2) ** 2;
}

export function starship_expendable({ payload = 12500.0 } = {}) {
  const d = 9.0;
  return launchVehicle({
    name: 'Starship (expendable)',
    stages: [
      sized_stage('superheavy', { engine: 'raptor_2', n_engines: 33, prop_mass: 3400.0e3, diameter: d }),
      sized_stage('ship', { engine: 'raptor_2', n_engines: 6, prop_mass: 1200.0e3, diameter: d }),
    ],
    fairing_mass: 0.0,
    payload_mass: payload,
    sref: Math.PI * (d / 2) ** 2,
    cd: LV_CD_TABLE,
  });
}

export function default_moon_rocket({ payload = 350.0 } = {}) {
  const d = 1.8;
  return launchVehicle({
    name: 'Sable',
    stages: [
      stage('sable1', 3800.0, 42000.0, 950.0e3, 305.0, 0.80, PROPELLANTS.kerolox, 5),
      stage('sable2', 900.0, 9500.0, 95.0e3, 345.0, 0.0, PROPELLANTS.kerolox, 1),
      stage('sablek', 140.0, 950.0, 15.0e3, 315.0, 0.0, PROPELLANTS.hypergolic, 1),
    ],
    fairing_mass: 150.0,
    payload_mass: payload,
    sref: Math.PI * (d / 2) ** 2,
    cd: LV_CD_TABLE,
  });
}
