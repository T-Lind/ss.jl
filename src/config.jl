# Mission and vehicle definitions as data: a TOML file fully specifies a
# mission — targets, launch vehicle, dispersions — so new configurations are
# files, not code. `load_mission` parses a spec; `run_mission` flies it.
#
# The reference spec is missions/moonshot.toml. Any field can be omitted and
# falls back to the same defaults the Julia API uses.

"""
    MissionSpec

Parsed mission definition. `kwargs` feeds `moonshot` directly; `lv` is the
launch vehicle built from the `[vehicle]` block (or the default rocket).
"""
struct MissionSpec
    name::String
    lv::LaunchVehicle
    pod_mass::Float64
    h_park::Float64
    inclination::Float64
    hp_moon::Float64
    hp_return::Float64
    tli_mag_err::Float64
    tli_point_err::Float64
    tcm_delay::Float64
end

_getf(d, k, def) = Float64(get(d, k, def))

"""
A stage is spelled out either way round. Name an `engine` from the
catalogue and thrust, Isp, exit area and propellant all follow from it
(with `engines = n` for a cluster, and `dry_kg` optional — omit it and the
mass is estimated); or give the four performance numbers directly, with
`propellant` naming the combination so the tank still gets a real volume.
"""
function _stage_from(d::AbstractDict, diameter::Float64)
    name = Symbol(get(d, "name", "stage"))
    mprop = _getf(d, "prop_kg", 0.0)
    ne = Int(get(d, "engines", 1))
    if haskey(d, "engine")
        return sized_stage(name;
            engine = Symbol(d["engine"]), n_engines = ne, prop_mass = mprop,
            diameter = diameter,
            dry_mass = haskey(d, "dry_kg") ? _getf(d, "dry_kg", 0.0) : nothing,
            systems_coeff = _getf(d, "systems_coeff", 0.55))
    end
    Stage(name, _getf(d, "dry_kg", 0.0), mprop,
          _getf(d, "thrust_vac_kn", 0.0) * 1e3,
          _getf(d, "isp_vac_s", 300.0),
          _getf(d, "exit_area_m2", 0.0),
          propellant(Symbol(get(d, "propellant", "kerolox"))), ne,
          _getf(d, "diameter_m", 0.0))
end

"""
A `[[vehicle.booster]]` set is one strap-on spelled out exactly like a stage
— the same two ways round — plus `count` and how the set behaves in parallel
with the core (`core_throttle`, `ignition_delay_s`, `sep_delay_s`).
"""
function _booster_from(d::AbstractDict, diameter::Float64)
    BoosterSet(
        stage = _stage_from(d, _getf(d, "diameter_m", diameter * 0.85)),
        count = Int(get(d, "count", 2)),
        ignition_delay = _getf(d, "ignition_delay_s", 0.0),
        sep_delay = _getf(d, "sep_delay_s", 0.0),
        core_throttle = clamp(_getf(d, "core_throttle", 1.0), 0.2, 1.0))
end

"""
    load_mission(path) -> MissionSpec

Parse a TOML mission spec. Sections: `[mission]` (targets & pod),
`[vehicle]` with `[[vehicle.stage]]` entries bottom-up, any number of
`[[vehicle.booster]]` strap-on sets, and optional `[mission.dispersions]`.
"""
function load_mission(path::AbstractString)
    raw = TOML.parsefile(path)
    m = get(raw, "mission", Dict{String,Any}())
    vd = get(raw, "vehicle", nothing)
    pod = _getf(m, "pod_mass_kg", 350.0)

    lv = if vd === nothing
        default_moon_rocket(payload = pod)
    else
        d = _getf(vd, "diameter_m", 1.8)
        stages = [_stage_from(s, d) for s in get(vd, "stage", Any[])]
        isempty(stages) && error("[vehicle] block needs at least one [[vehicle.stage]]")
        # `fairing = false` flies the spacecraft in the open, the way a crewed
        # stack usually does: no shroud mass, a blunt nose in the wave drag, and
        # the capsule itself setting the reference area if it is the widest
        # thing on the vehicle. Defaults true, so an existing spec is unchanged.
        fair = Bool(get(vd, "fairing", true))
        pod_d = 2 * pod_radius(pod)
        sref = stack_sref((stage_diameter(s, d) for s in stages), pod_d, fair)
        LaunchVehicle(
            name = String(get(vd, "name", "vehicle")),
            stages = stages,
            fairing_mass = fair ? _getf(vd, "fairing_kg", 150.0) : 0.0,
            payload_mass = pod,
            # drag acts on the widest cross-section in the stack; strap-ons
            # add their own on top while they are attached
            sref = sref,
            cd = fair ? LV_CD_TABLE : bare_payload_cd(pod_d, sref),
            boosters = [_booster_from(b, d) for b in get(vd, "booster", Any[])],
        )
    end

    disp = get(m, "dispersions", Dict{String,Any}())
    MissionSpec(
        String(get(m, "name", basename(path))),
        lv, pod,
        _getf(m, "parking_altitude_km", 200.0) * 1e3,
        deg2rad_(_getf(m, "inclination_deg", 28.5)),
        _getf(m, "perilune_km", 2000.0) * 1e3,
        _getf(m, "return_perigee_km", 35.0) * 1e3,
        _getf(disp, "tli_mag_err_pct", 0.0) / 100,
        deg2rad_(_getf(disp, "tli_point_err_deg", 0.0)),
        _getf(disp, "tcm_delay_hr", 24.0) * 3600,
    )
end

"""
    run_mission(spec::MissionSpec; verbose=false) -> MoonshotResult
    run_mission(path::AbstractString; verbose=false)

Design and fly a mission from its spec.
"""
run_mission(spec::MissionSpec; verbose::Bool = false) =
    moonshot(lv = spec.lv,
             h_park = spec.h_park,
             inclination = spec.inclination,
             hp_moon = spec.hp_moon,
             hp_return = spec.hp_return,
             tli_mag_err = spec.tli_mag_err,
             tli_point_err = spec.tli_point_err,
             tcm_delay = spec.tcm_delay,
             # a stack with strap-ons needs its pitch kick re-found, or the
             # extra impulse goes into lofting it
             optimize_kick = !isempty(spec.lv.boosters),
             verbose = verbose)
run_mission(path::AbstractString; kwargs...) = run_mission(load_mission(path); kwargs...)
