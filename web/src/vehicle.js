// Port of src/vehicle.jl.
import { deg2rad_ } from './constants.js';
import { default_capsule_aero, cd_coeff } from './aerodynamics.js';

export const parachute = (name, cda, mach_max, alt_max, fill_time) =>
  ({ name, cda, mach_max, alt_max, fill_time });

export const chute_fill = (c, tau) =>
  Math.min(1.0, Math.max(0.0, tau / c.fill_time)) ** 2;

export function vehicle({ name = 'capsule', mass, sref, lref, rn, iyy,
                          emissivity = 0.85, aero, chutes = [] }) {
  return { name, mass, sref, lref, rn, iyy, emissivity, aero, chutes };
}

export function ballistic_coefficient(v, M = 25.0) {
  const alpha = v.aero.kind === 'capsule' ? v.aero.alpha_trim : 0.0;
  return v.mass / (cd_coeff(v.aero, M, alpha) * v.sref);
}

export function default_reentry_pod({ mass = 350.0, cl_trim_hyp = 0.45,
                                      diameter = 1.5,
                                      alpha_trim = cl_trim_hyp > 0 ? deg2rad_(25.0) : 0.0 } = {}) {
  const d = diameter;
  return vehicle({
    name: 'reentry-pod',
    mass,
    sref: Math.PI * (d / 2) ** 2,
    lref: d,
    rn: 1.2 * d,
    iyy: 0.35 * mass * (d / 2) ** 2 * 2.0,
    aero: default_capsule_aero({ cl_trim_hyp, alpha_trim }),
    chutes: [
      parachute('drogue', 12.0 * (d / 1.5) ** 2, 1.5, 9000.0, 2.0),
      parachute('main', 260.0 * (mass / 350.0), 0.5, 3000.0, 6.0),
    ],
  });
}

export const apollo_capsule = ({ mass = 5560.0, cl_trim_hyp = 0.45 } = {}) =>
  default_reentry_pod({ mass, cl_trim_hyp, diameter: 3.9 });
