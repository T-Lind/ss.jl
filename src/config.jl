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

function _stage_from(d::AbstractDict)
    Stage(Symbol(get(d, "name", "stage")),
          _getf(d, "dry_kg", 0.0),
          _getf(d, "prop_kg", 0.0),
          _getf(d, "thrust_vac_kn", 0.0) * 1e3,
          _getf(d, "isp_vac_s", 300.0),
          _getf(d, "exit_area_m2", 0.0))
end

"""
    load_mission(path) -> MissionSpec

Parse a TOML mission spec. Sections: `[mission]` (targets & pod),
`[vehicle]` with `[[vehicle.stage]]` entries bottom-up, and optional
`[mission.dispersions]`.
"""
function load_mission(path::AbstractString)
    raw = TOML.parsefile(path)
    m = get(raw, "mission", Dict{String,Any}())
    vd = get(raw, "vehicle", nothing)
    pod = _getf(m, "pod_mass_kg", 350.0)

    lv = if vd === nothing
        default_moon_rocket(payload = pod)
    else
        stages = [_stage_from(s) for s in get(vd, "stage", Any[])]
        isempty(stages) && error("[vehicle] block needs at least one [[vehicle.stage]]")
        d = _getf(vd, "diameter_m", 1.8)
        LaunchVehicle(
            name = String(get(vd, "name", "vehicle")),
            stages = stages,
            fairing_mass = _getf(vd, "fairing_kg", 150.0),
            payload_mass = pod,
            sref = pi * (d / 2)^2,
            cd = LV_CD_TABLE,
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
             verbose = verbose)
run_mission(path::AbstractString; kwargs...) = run_mission(load_mission(path); kwargs...)
