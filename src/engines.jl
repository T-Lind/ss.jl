# Propellants, engines, and conceptual stage sizing.
#
# The flight model only needs a stage's mass, thrust, Isp and nozzle exit
# area — that is what `Stage` carries and what the integrator burns. This
# file is the *design* layer that sits above it: real propellant properties,
# a catalogue of real engines, and mass-estimating relations that turn
# "150 t of methalox and nine engines" into a `Stage`.
#
# Keeping the two apart matters. Mixture ratios and tank coefficients have
# no business inside an ODE right-hand side, but without them a stage's
# propellant volume is a guess — and the volume is exactly what sets how
# physically large the vehicle has to be.

"""
    Propellant(name, rho_fuel, rho_ox, mr, cryo)

A propellant combination. `rho_fuel`/`rho_ox` are storage densities
[kg/m^3], `mr` the oxidiser-to-fuel *mass* ratio (0 for a monopropellant or
solid), and `cryo` marks combinations needing insulated tanks.
"""
struct Propellant
    name::Symbol
    rho_fuel::Float64
    rho_ox::Float64
    mr::Float64
    cryo::Bool
end

"""
    bulk_density(p) -> kg/m^3

Density of the loaded combination: the mass of fuel + oxidiser divided by
the volume both occupy. This is what sets tank size, and it varies by more
than an order of magnitude across the catalogue — hydrolox is 343 kg/m^3
against kerolox's 1023, so the same propellant mass needs a three-times
longer barrel.
"""
function bulk_density(p::Propellant)
    p.mr <= 0 && return p.rho_fuel
    (1.0 + p.mr) / (1.0 / p.rho_fuel + p.mr / p.rho_ox)
end

"Fuel and oxidiser volumes [m^3] for `mprop` kg of loaded propellant."
function propellant_volumes(p::Propellant, mprop::Float64)
    p.mr <= 0 && return (mprop / p.rho_fuel, 0.0)
    mf = mprop / (1.0 + p.mr)
    (mf / p.rho_fuel, (mprop - mf) / p.rho_ox)
end

"""
Storage densities are the usual handbook values: RP-1 810, LH2 70.8,
LCH4 422.6, LOX 1141, MMH 880, NTO 1443, hydrazine 1004 kg/m^3; solid is a
representative HTPB/AP/Al grain. Mixture ratios are typical flight values
rather than optimum-Isp ones.
"""
const PROPELLANTS = Dict{Symbol,Propellant}(
    :kerolox    => Propellant(:kerolox,    810.0, 1141.0, 2.56, true),
    :hydrolox   => Propellant(:hydrolox,    70.8, 1141.0, 5.50, true),
    :methalox   => Propellant(:methalox,   422.6, 1141.0, 3.60, true),
    :hypergolic => Propellant(:hypergolic, 880.0, 1443.0, 1.65, false),
    :hydrazine  => Propellant(:hydrazine, 1004.0,    0.0, 0.0,  false),
    :solid      => Propellant(:solid,     1750.0,    0.0, 0.0,  false),
)

propellant(name::Symbol) = get(PROPELLANTS, name) do
    throw(ArgumentError("unknown propellant $name; have $(sort(collect(keys(PROPELLANTS))))"))
end
propellant(p::Propellant) = p

"""
    Engine(name, prop, thrust_vac, isp_vac, isp_sl, ae, mass, throttle_min)

One engine. `thrust_vac` [N], `isp_vac`/`isp_sl` [s] (`isp_sl` = 0 for a
vacuum-only nozzle), `ae` nozzle exit area [m^2], `mass` [kg].

`ae` is what couples the two Isp figures: with a fixed mass flow,
`F(p) = F_vac - Ae*p`, so `Isp_sl/Isp_vac = 1 - Ae*p0/F_vac`. Build engines
with [`engine`](@ref) and the exit area is derived from the two Isp values,
which keeps the pair consistent by construction.
"""
struct Engine
    name::Symbol
    prop::Propellant
    thrust_vac::Float64
    isp_vac::Float64
    isp_sl::Float64
    ae::Float64
    mass::Float64
    throttle_min::Float64
end

"""
    engine(name; prop, thrust_vac_kn, isp_vac, isp_sl=0, mass, throttle_min=1.0)

Define an engine, deriving the nozzle exit area from the sea-level/vacuum
Isp split so the pressure correction reproduces both figures.
"""
function engine(name::Symbol; prop, thrust_vac_kn::Float64, isp_vac::Float64,
                isp_sl::Float64 = 0.0, mass::Float64 = 0.0,
                throttle_min::Float64 = 1.0)
    F = thrust_vac_kn * 1e3
    ae = isp_sl > 0 ? F * (1.0 - isp_sl / isp_vac) / P0_SEA : 0.0
    Engine(name, propellant(prop), F, isp_vac, isp_sl, ae, mass, throttle_min)
end

"""
Representative flight engines, from published performance figures. These
are approximate — good enough to size a vehicle and to see how propellant
choice drives its shape, not a substitute for a manufacturer data sheet.
Vacuum-only entries (`isp_sl = 0`) get no pressure correction.
"""
const ENGINES = Dict{Symbol,Engine}(
    :merlin_1d     => engine(:merlin_1d;     prop = :kerolox,
                             thrust_vac_kn = 981.0, isp_vac = 311.0,
                             isp_sl = 282.0, mass = 470.0, throttle_min = 0.4),
    :merlin_1d_vac => engine(:merlin_1d_vac; prop = :kerolox,
                             thrust_vac_kn = 981.0, isp_vac = 348.0,
                             mass = 490.0, throttle_min = 0.4),
    :rutherford    => engine(:rutherford;    prop = :kerolox,
                             thrust_vac_kn = 26.0, isp_vac = 343.0,
                             isp_sl = 311.0, mass = 35.0, throttle_min = 0.5),
    :rutherford_vac=> engine(:rutherford_vac; prop = :kerolox,
                             thrust_vac_kn = 25.8, isp_vac = 343.0,
                             mass = 38.0, throttle_min = 0.5),
    :raptor_2      => engine(:raptor_2;      prop = :methalox,
                             thrust_vac_kn = 2300.0, isp_vac = 347.0,
                             isp_sl = 327.0, mass = 1600.0, throttle_min = 0.4),
    :rs25          => engine(:rs25;          prop = :hydrolox,
                             thrust_vac_kn = 2279.0, isp_vac = 452.3,
                             isp_sl = 366.0, mass = 3177.0, throttle_min = 0.67),
    :rl10b2        => engine(:rl10b2;        prop = :hydrolox,
                             thrust_vac_kn = 110.1, isp_vac = 465.5,
                             mass = 301.0, throttle_min = 1.0),
    :vinci         => engine(:vinci;         prop = :hydrolox,
                             thrust_vac_kn = 180.0, isp_vac = 457.0,
                             mass = 550.0, throttle_min = 0.35),
    :aj10          => engine(:aj10;          prop = :hypergolic,
                             thrust_vac_kn = 26.7, isp_vac = 316.0,
                             mass = 118.0, throttle_min = 1.0),
    # The Apollo pair. Neither throttles: the F-1 ran wide open and the S-IC
    # limited its acceleration by shutting the centre engine down early, and the
    # J-2 had two mixture-ratio settings rather than a throttle. `throttle_min =
    # 1` is the honest way to say a fixed engine, and it is what the aj10 and
    # the RL10B-2 already say.
    :f1            => engine(:f1;            prop = :kerolox,
                             thrust_vac_kn = 7770.0, isp_vac = 304.0,
                             isp_sl = 263.0, mass = 8400.0, throttle_min = 1.0),
    # Vacuum-only, by this catalogue's convention (isp_sl = 0 means no pressure
    # correction): the J-2 first lit above 60 km on the S-II and in orbit on the
    # S-IVB, and it never saw sea-level backpressure in flight.
    :j2            => engine(:j2;            prop = :hydrolox,
                             thrust_vac_kn = 1033.1, isp_vac = 421.0,
                             mass = 1788.0, throttle_min = 1.0),
)

lookup_engine(name::Symbol) = get(ENGINES, name) do
    throw(ArgumentError("unknown engine $name; have $(sort(collect(keys(ENGINES))))"))
end
lookup_engine(e::Engine) = e

# ------------------------------------------------------ conceptual sizing --

"""
    stage_mass(eng, n, mprop, diameter; systems_coeff=0.55) -> NamedTuple

Conceptual dry-mass breakdown for a stage: tanks sized from the real
propellant volumes, the engines' own mass, a thrust structure scaled with
liftoff thrust, insulation for cryogenic tanks, and one lumped `systems`
term for everything else (skirts, intertank, plumbing, pressurisation, TVC,
avionics, residuals).

Tank and thrust-structure coefficients follow the usual conceptual-design
rules of thumb (about 12.2 kg per m^3 of dense-propellant tankage, 9.1 for
liquid hydrogen, 1.12 kg/m^2 of cryogenic insulation, 2.55e-4 kg per newton
of thrust structure).

`systems` is the honest fudge — one calibration curve, not a derivation. It
goes as `systems_coeff * mprop^0.78` rather than a flat fraction because
mass fraction improves with size (square-cube): a 9.5 t upper stage is
about 9% dry, a 411 t booster about 6%. At the default coefficient this
lands within a few percent of both a Falcon-9-class first stage and the
reference Sable upper stage, and ~15% light on small dense boosters, which
are structurally heavier than the curve suggests. Anything needing a real
mass statement should pass `dry_mass` and skip the estimate entirely.
"""
function stage_mass(eng::Engine, n::Int, mprop::Float64, diameter::Float64;
                    systems_coeff::Float64 = 0.55)
    n >= 1 || throw(ArgumentError("a stage needs at least one engine"))
    vf, vox = propellant_volumes(eng.prop, mprop)
    vol = vf + vox
    # LH2 tanks are the light-but-bulky case and get their own coefficient
    k_f = eng.prop.name === :hydrolox ? 9.1 : 12.2
    tanks = k_f * vf + 12.2 * vox
    # wetted area of a cylinder of this diameter holding that volume
    L = vol / (pi * diameter^2 / 4)
    area = pi * diameter * L + pi * diameter^2 / 2
    insulation = eng.prop.cryo ? 1.123 * area : 0.0
    engines = n * eng.mass
    thrust_structure = 2.55e-4 * n * eng.thrust_vac
    systems = systems_coeff * mprop^0.78
    total = tanks + insulation + engines + thrust_structure + systems
    (tanks = tanks, insulation = insulation, engines = engines,
     thrust_structure = thrust_structure, systems = systems,
     dry = total, volume = vol, length = L,
     dry_fraction = mprop > 0 ? total / mprop : NaN)
end

"""
    sized_stage(name; engine, n_engines=1, prop_mass, diameter,
                dry_mass=nothing, systems_coeff=0.55) -> Stage

Build a [`Stage`](@ref) from an engine and a propellant load. Thrust, Isp
and exit area come from `n_engines` copies of the engine; dry mass comes
from [`stage_mass`](@ref) unless you pass a measured `dry_mass`.
"""
function sized_stage(name::Symbol; engine, n_engines::Int = 1,
                     prop_mass::Float64, diameter::Float64 = 1.8,
                     dry_mass::Union{Nothing,Float64} = nothing,
                     systems_coeff::Float64 = 0.55)
    eng = lookup_engine(engine)
    md = dry_mass === nothing ?
         stage_mass(eng, n_engines, prop_mass, diameter;
                    systems_coeff = systems_coeff).dry : dry_mass
    Stage(name, md, prop_mass, n_engines * eng.thrust_vac, eng.isp_vac,
          n_engines * eng.ae, eng.prop, n_engines, diameter)
end
