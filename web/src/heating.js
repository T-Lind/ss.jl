// Port of src/heating.jl.
import { K_SUTTON_GRAVES, SIGMA_SB } from './constants.js';
import { table1d, interp1 } from './aerodynamics.js';

export function heating_convective(rho, vrel, rn) {
  if (rho <= 0) return 0.0;
  return K_SUTTON_GRAVES * Math.sqrt(rho / rn) * vrel ** 3;
}

const TAUBER_SUTTON_TABLE = table1d(
  [9000, 9250, 10000, 11000, 12000, 13000, 14000, 15000, 16000],
  [0.0, 1.5, 35.0, 151.0, 359.0, 660.0, 1065.0, 1550.0, 2040.0]);

export function heating_radiative(rho, vrel, rn) {
  if (vrel < 9000.0 || rho <= 0) return 0.0;
  const f = interp1(TAUBER_SUTTON_TABLE, vrel);
  const a = Math.min(1.0, Math.max(0.0,
    1.072e6 * vrel ** (-1.88) * rho ** (-0.325)));
  return 4.736e4 * rn ** a * rho ** 1.22 * f * 1.0e4;
}

export const wall_temperature = (q, emissivity) =>
  q <= 0 ? 0.0 : (q / (emissivity * SIGMA_SB)) ** 0.25;
