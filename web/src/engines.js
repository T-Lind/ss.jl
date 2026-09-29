// Port of src/engines.jl: propellants, the engine catalogue, and conceptual
// stage sizing.
import { P0_SEA } from './constants.js';

export const propellantDef = (name, rho_fuel, rho_ox, mr, cryo) =>
  ({ name, rho_fuel, rho_ox, mr, cryo });

export function bulk_density(p) {
  if (p.mr <= 0) return p.rho_fuel;
  return (1.0 + p.mr) / (1.0 / p.rho_fuel + p.mr / p.rho_ox);
}

export function propellant_volumes(p, mprop) {
  if (p.mr <= 0) return [mprop / p.rho_fuel, 0.0];
  const mf = mprop / (1.0 + p.mr);
  return [mf / p.rho_fuel, (mprop - mf) / p.rho_ox];
}

export const PROPELLANTS = {
  kerolox: propellantDef('kerolox', 810.0, 1141.0, 2.56, true),
  hydrolox: propellantDef('hydrolox', 70.8, 1141.0, 5.50, true),
  methalox: propellantDef('methalox', 422.6, 1141.0, 3.60, true),
  hypergolic: propellantDef('hypergolic', 880.0, 1443.0, 1.65, false),
  hydrazine: propellantDef('hydrazine', 1004.0, 0.0, 0.0, false),
  solid: propellantDef('solid', 1750.0, 0.0, 0.0, false),
};

export function propellant(name) {
  const p = PROPELLANTS[name];
  if (!p) throw new Error(`unknown propellant ${name}`);
  return p;
}

export function engine(name, { prop, thrust_vac_kn, isp_vac, isp_sl = 0.0,
                               mass = 0.0, throttle_min = 1.0 }) {
  const F = thrust_vac_kn * 1e3;
  const ae = isp_sl > 0 ? F * (1.0 - isp_sl / isp_vac) / P0_SEA : 0.0;
  return { name, prop: propellant(prop), thrust_vac: F, isp_vac, isp_sl, ae,
           mass, throttle_min };
}

export const ENGINES = {
  merlin_1d: engine('merlin_1d', { prop: 'kerolox', thrust_vac_kn: 981.0, isp_vac: 311.0, isp_sl: 282.0, mass: 470.0, throttle_min: 0.4 }),
  merlin_1d_vac: engine('merlin_1d_vac', { prop: 'kerolox', thrust_vac_kn: 981.0, isp_vac: 348.0, mass: 490.0, throttle_min: 0.4 }),
  rutherford: engine('rutherford', { prop: 'kerolox', thrust_vac_kn: 26.0, isp_vac: 343.0, isp_sl: 311.0, mass: 35.0, throttle_min: 0.5 }),
  rutherford_vac: engine('rutherford_vac', { prop: 'kerolox', thrust_vac_kn: 25.8, isp_vac: 343.0, mass: 38.0, throttle_min: 0.5 }),
  raptor_2: engine('raptor_2', { prop: 'methalox', thrust_vac_kn: 2300.0, isp_vac: 347.0, isp_sl: 327.0, mass: 1600.0, throttle_min: 0.4 }),
  rs25: engine('rs25', { prop: 'hydrolox', thrust_vac_kn: 2279.0, isp_vac: 452.3, isp_sl: 366.0, mass: 3177.0, throttle_min: 0.67 }),
  rl10b2: engine('rl10b2', { prop: 'hydrolox', thrust_vac_kn: 110.1, isp_vac: 465.5, mass: 301.0, throttle_min: 1.0 }),
  vinci: engine('vinci', { prop: 'hydrolox', thrust_vac_kn: 180.0, isp_vac: 457.0, mass: 550.0, throttle_min: 0.35 }),
  aj10: engine('aj10', { prop: 'hypergolic', thrust_vac_kn: 26.7, isp_vac: 316.0, mass: 118.0, throttle_min: 1.0 }),
  f1: engine('f1', { prop: 'kerolox', thrust_vac_kn: 7770.0, isp_vac: 304.0, isp_sl: 263.0, mass: 8400.0, throttle_min: 1.0 }),
  j2: engine('j2', { prop: 'hydrolox', thrust_vac_kn: 1033.1, isp_vac: 421.0, mass: 1788.0, throttle_min: 1.0 }),
};

export function lookup_engine(name) {
  if (typeof name === 'object') return name;
  const e = ENGINES[name];
  if (!e) throw new Error(`unknown engine ${name}`);
  return e;
}

export function stage_mass(eng, n, mprop, diameter, { systems_coeff = 0.55 } = {}) {
  if (n < 1) throw new Error('a stage needs at least one engine');
  const [vf, vox] = propellant_volumes(eng.prop, mprop);
  const vol = vf + vox;
  const k_f = eng.prop.name === 'hydrolox' ? 9.1 : 12.2;
  const tanks = k_f * vf + 12.2 * vox;
  const L = vol / (Math.PI * diameter * diameter / 4);
  const area = Math.PI * diameter * L + Math.PI * diameter * diameter / 2;
  const insulation = eng.prop.cryo ? 1.123 * area : 0.0;
  const engines_m = n * eng.mass;
  const thrust_structure = 2.55e-4 * n * eng.thrust_vac;
  const systems = systems_coeff * mprop ** 0.78;
  const total = tanks + insulation + engines_m + thrust_structure + systems;
  return {
    tanks, insulation, engines: engines_m, thrust_structure, systems,
    dry: total, volume: vol, length: L,
    dry_fraction: mprop > 0 ? total / mprop : NaN,
  };
}

export function sized_stage(name, { engine: engName, n_engines = 1, prop_mass,
                                    diameter = 1.8, dry_mass = null,
                                    systems_coeff = 0.55 }) {
  const eng = lookup_engine(engName);
  const md = dry_mass === null
    ? stage_mass(eng, n_engines, prop_mass, diameter, { systems_coeff }).dry
    : dry_mass;
  return { name, mdry: md, mprop: prop_mass, thrust_vac: n_engines * eng.thrust_vac,
           isp_vac: eng.isp_vac, ae: n_engines * eng.ae, prop: eng.prop,
           n_engines, diameter };
}
