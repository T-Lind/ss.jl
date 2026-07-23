# Physical and planetary constants (SI units throughout).

const MU_EARTH     = 3.986004418e14      # gravitational parameter [m^3/s^2]
const RE_EQ        = 6378137.0           # WGS-84 equatorial radius [m]
const F_WGS84      = 1.0 / 298.257223563 # WGS-84 flattening
const RE_POL       = RE_EQ * (1 - F_WGS84)
const E2_WGS84     = F_WGS84 * (2 - F_WGS84) # first eccentricity squared
const RE_MEAN      = 6371008.8           # mean Earth radius [m] (for great-circle distances)
const J2_EARTH     = 1.08262668e-3       # second zonal harmonic
const OMEGA_EARTH  = 7.2921159e-5        # Earth rotation rate [rad/s]
const G0           = 9.80665             # standard gravity [m/s^2]
const R_AIR        = 287.0528            # specific gas constant, air [J/(kg K)]
const GAMMA_AIR    = 1.4                 # ratio of specific heats
const SIGMA_SB     = 5.670374419e-8      # Stefan-Boltzmann [W/(m^2 K^4)]

# Sutton-Graves stagnation-point convective heating constant for Earth air,
# q_dot = K_SG * sqrt(rho / R_n) * V^3   [W/m^2]
const K_SUTTON_GRAVES = 1.74153e-4

# Moon (mean values; circular-orbit ephemeris fidelity)
const MU_MOON      = 4.9048695e12        # lunar gravitational parameter [m^3/s^2]
const R_MOON       = 1737.4e3            # mean lunar radius [m]
const A_MOON       = 384400.0e3          # mean Earth-Moon distance [m]
const T_SIDEREAL_MOON = 27.321661 * 86400.0  # sidereal month [s]
const N_MOON       = 2pi / T_SIDEREAL_MOON   # lunar mean motion [rad/s]

deg2rad_(x) = x * (pi / 180)
rad2deg_(x) = x * (180 / pi)
