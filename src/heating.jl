# Stagnation-point aerothermal heating.
#
#   * Convective: Sutton-Graves correlation  q = K sqrt(rho/Rn) V^3.
#   * Radiative (shock-layer): Tauber-Sutton correlation for Earth air,
#     q = C Rn^a rho^1.22 f(V). Negligible below ~9 km/s (returns 0), so it
#     contributes ~nothing for LEO entry but activates automatically for
#     lunar/planetary-return extensions.
#   * Radiative-equilibrium wall temperature from total stagnation heat flux.

"Sutton-Graves convective stagnation heating [W/m^2]."
@inline function heating_convective(rho::Float64, vrel::Float64, rn::Float64)
    rho <= 0 && return 0.0
    K_SUTTON_GRAVES * sqrt(rho / rn) * vrel^3
end

# Tauber-Sutton velocity function f(V) for Earth (tabulated), V in m/s.
const TAUBER_SUTTON_V = [9000.0, 9250.0, 10000.0, 11000.0, 12000.0, 13000.0, 14000.0, 15000.0, 16000.0]
const TAUBER_SUTTON_F = [0.0,    1.5,    35.0,    151.0,   359.0,   660.0,   1065.0,  1550.0,  2040.0]
const TAUBER_SUTTON_TABLE = Table1D(TAUBER_SUTTON_V, TAUBER_SUTTON_F)

"Tauber-Sutton radiative stagnation heating for Earth air [W/m^2]."
function heating_radiative(rho::Float64, vrel::Float64, rn::Float64)
    (vrel < 9000.0 || rho <= 0) && return 0.0
    f = interp1(TAUBER_SUTTON_TABLE, vrel)
    # exponent a per Tauber-Sutton, clamped to its stated validity range
    a = clamp(1.072e6 * vrel^(-1.88) * rho^(-0.325), 0.0, 1.0)
    # correlation yields W/cm^2 (C = 4.736e4); convert to W/m^2
    4.736e4 * rn^a * rho^1.22 * f * 1.0e4
end

"Radiative-equilibrium wall temperature [K] for total stagnation flux q [W/m^2]."
@inline wall_temperature(q::Float64, emissivity::Float64) =
    q <= 0 ? 0.0 : (q / (emissivity * SIGMA_SB))^0.25
