// Port of src/constants.jl. Physical and planetary constants (SI throughout).
export const MU_EARTH = 3.986004418e14;      // gravitational parameter [m^3/s^2]
export const RE_EQ = 6378137.0;              // WGS-84 equatorial radius [m]
export const F_WGS84 = 1.0 / 298.257223563;  // WGS-84 flattening
export const RE_POL = RE_EQ * (1 - F_WGS84);
export const E2_WGS84 = F_WGS84 * (2 - F_WGS84); // first eccentricity squared
export const RE_MEAN = 6371008.8;            // mean Earth radius [m]
export const J2_EARTH = 1.08262668e-3;       // second zonal harmonic
export const OMEGA_EARTH = 7.2921159e-5;     // Earth rotation rate [rad/s]
export const G0 = 9.80665;                   // standard gravity [m/s^2]
export const P0_SEA = 101325.0;              // sea-level standard pressure [Pa]
export const R_AIR = 287.0528;               // specific gas constant, air [J/(kg K)]
export const GAMMA_AIR = 1.4;                // ratio of specific heats
export const SIGMA_SB = 5.670374419e-8;      // Stefan-Boltzmann [W/(m^2 K^4)]

// Sutton-Graves stagnation-point convective heating constant for Earth air.
export const K_SUTTON_GRAVES = 1.74153e-4;

// Moon (mean values; circular-orbit ephemeris fidelity)
export const MU_MOON = 4.9048695e12;         // [m^3/s^2]
export const R_MOON = 1737.4e3;              // mean radius [m]
export const A_MOON = 384400.0e3;            // mean Earth-Moon distance [m]
export const T_SIDEREAL_MOON = 27.321661 * 86400.0; // sidereal month [s]
export const N_MOON = 2 * Math.PI / T_SIDEREAL_MOON; // mean motion [rad/s]

export const deg2rad_ = x => x * (Math.PI / 180);
export const rad2deg_ = x => x * (180 / Math.PI);
