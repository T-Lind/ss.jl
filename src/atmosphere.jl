# Atmosphere models.
#
# `USSA76` implements the U.S. Standard Atmosphere 1976:
#   * exact layered analytic solution from 0-86 km geometric altitude
#     (7 geopotential layers with linear/isothermal temperature profiles),
#   * tabulated log-linear density / linear temperature interpolation from
#     86-500 km (values from the USSA76 upper-atmosphere tables),
#   * exponential extrapolation above 500 km.
#
# Extension point: subtype `AbstractAtmosphere` and implement
# `atmosphere_state(model, h)` to plug in e.g. NRLMSISE-00, GRAM dispersions,
# or Mars/Titan atmospheres. `ScaledAtmosphere` wraps any model with a density
# multiplier and is used by the Monte Carlo dispersions.

abstract type AbstractAtmosphere end

"""
    atmosphere_state(atm, h) -> (rho, T, p, a)

Density [kg/m^3], temperature [K], pressure [Pa] and speed of sound [m/s]
at geodetic altitude `h` [m]. Above ~86 km the continuum speed of sound is
only a bookkeeping quantity for Mach number (flow is free-molecular there).
"""
function atmosphere_state end

struct USSA76 <: AbstractAtmosphere end

# Lower atmosphere: geopotential base altitude [m], base temperature [K],
# base pressure [Pa], lapse rate [K/m].
const USSA76_LAYERS = (
    (0.0,     288.15, 101325.0,   -0.0065),
    (11000.0, 216.65, 22632.06,    0.0),
    (20000.0, 216.65, 5474.889,    0.0010),
    (32000.0, 228.65, 868.0187,    0.0028),
    (47000.0, 270.65, 110.9063,    0.0),
    (51000.0, 270.65, 66.93887,   -0.0028),
    (71000.0, 214.65, 3.956420,   -0.0020),
)
const USSA76_R0 = 6356766.0   # radius used for geometric<->geopotential conversion [m]

# Upper atmosphere table: geometric altitude [m], T [K], rho [kg/m^3] (USSA76).
const USSA76_UPPER = (
    (86.0e3,  186.87, 6.958e-6),
    (90.0e3,  186.87, 3.416e-6),
    (95.0e3,  188.42, 1.393e-6),
    (100.0e3, 195.08, 5.604e-7),
    (110.0e3, 240.00, 9.708e-8),
    (120.0e3, 360.00, 2.222e-8),
    (130.0e3, 469.27, 8.152e-9),
    (140.0e3, 559.63, 3.831e-9),
    (150.0e3, 634.39, 2.076e-9),
    (160.0e3, 696.29, 1.233e-9),
    (180.0e3, 790.07, 5.194e-10),
    (200.0e3, 854.56, 2.541e-10),
    (250.0e3, 941.33, 6.073e-11),
    (300.0e3, 976.01, 1.916e-11),
    (350.0e3, 990.06, 7.014e-12),
    (400.0e3, 995.83, 2.803e-12),
    (450.0e3, 998.22, 1.184e-12),
    (500.0e3, 999.24, 5.215e-13),
)

function atmosphere_state(::USSA76, h::Float64)
    h = max(h, -100.0)  # tolerate slightly sub-zero geodetic altitude at splashdown
    if h <= 86.0e3
        hp = USSA76_R0 * h / (USSA76_R0 + h)  # geopotential altitude
        k = length(USSA76_LAYERS)
        for i in eachindex(USSA76_LAYERS)
            if i == length(USSA76_LAYERS) || USSA76_LAYERS[i+1][1] > hp
                k = i
                break
            end
        end
        hb, Tb, pb, L = USSA76_LAYERS[k]
        dh = hp - hb
        if L == 0.0
            T = Tb
            p = pb * exp(-G0 * dh / (R_AIR * Tb))
        else
            T = Tb + L * dh
            p = pb * (Tb / T)^(G0 / (R_AIR * L))
        end
        rho = p / (R_AIR * T)
        return (rho, T, p, sqrt(GAMMA_AIR * R_AIR * T))
    else
        n = length(USSA76_UPPER)
        if h >= USSA76_UPPER[n][1]
            # exponential extrapolation using the last two table points
            h1, T1, r1 = USSA76_UPPER[n-1]
            h2, T2, r2 = USSA76_UPPER[n]
            Hs = (h2 - h1) / log(r1 / r2)   # scale height
            rho = r2 * exp(-(h - h2) / Hs)
            T = T2
            return (rho, T, rho * R_AIR * T, sqrt(GAMMA_AIR * R_AIR * T))
        end
        k = 1
        for i in 1:n-1
            if USSA76_UPPER[i+1][1] > h
                k = i
                break
            end
        end
        h1, T1, r1 = USSA76_UPPER[k]
        h2, T2, r2 = USSA76_UPPER[k+1]
        f = (h - h1) / (h2 - h1)
        T = T1 + f * (T2 - T1)
        rho = r1 * exp(f * log(r2 / r1))    # log-linear in density
        return (rho, T, rho * R_AIR * T, sqrt(GAMMA_AIR * R_AIR * T))
    end
end

"""
    ScaledAtmosphere(base, rho_mult)

Wraps another atmosphere and scales its density by `rho_mult`
(temperature and speed of sound unchanged). Used for Monte Carlo dispersions.
"""
struct ScaledAtmosphere{A<:AbstractAtmosphere} <: AbstractAtmosphere
    base::A
    rho_mult::Float64
end

function atmosphere_state(atm::ScaledAtmosphere, h::Float64)
    rho, T, p, a = atmosphere_state(atm.base, h)
    return (rho * atm.rho_mult, T, p * atm.rho_mult, a)
end
