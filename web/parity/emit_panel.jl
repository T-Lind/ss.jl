# Emit the panel's own payload objects for the browser port to compare against.
# Loads the real PanelApp module (HTTP server included) and writes the same
# dicts `route` would serialise, so the Node test can diff them structurally.
#
#   julia --project=. web/parity/emit_panel.jl web/parity/panel_golden.json
include(joinpath(@__DIR__, "..", "..", "scripts", "panelapp.jl"))

const P = PanelApp
const form = P.parse_form

# A Starship-class stack, because the reference Sable cannot lift the 12.5 t
# lander to the Moon (the panel's own landing page selects a vehicle for this).
const STARSHIP = "mode=landing&nstages=2&nboost=0&diameter=9&fairing_on=0" *
    "&kick_deg=5&opt_kick=1" *
    "&s1_engine=raptor_2&s1_engines=33&s1_prop=3400000&s1_dry_auto=1&s1_diameter=9" *
    "&s2_engine=raptor_2&s2_engines=6&s2_prop=1200000&s2_dry_auto=1&s2_diameter=9"

const CASES = Dict{String,String}(
    "flyby"      => "mode=flyby&payload_kind=bus&bus_mass=200&cargo_mass=150",
    "orbit"      => "mode=orbit&orbit=leo",
    "suborbital" => "mode=suborbital&sub_profile=hop&sub_apogee_km=110&pod_mass=350",
    "landing"    => STARSHIP,
)

# Geometry for two vehicles: the reference crew capsule, and a Falcon-class
# stack carrying an uncrewed bus.
const GEOM = Dict{String,String}(
    "sable"  => "nstages=3&nboost=0&diameter=1.8",
    "falcon" => "nstages=3&nboost=0&diameter=3.7&payload_kind=bus&bus_mass=200" *
                "&s1_engine=merlin_1d&s1_engines=9&s1_prop=411000&s1_dry_auto=1" *
                "&s1_diameter=3.7&s2_engine=merlin_1d_vac&s2_engines=1&s2_prop=108000" *
                "&s2_dry_auto=1&s2_diameter=3.7&s3_engine=aj10&s3_engines=1" *
                "&s3_prop=2200&s3_dry_auto=1&s3_diameter=2.4",
)

missions = Dict{String,Any}(k => P.panel_mission(form(v)) for (k, v) in CASES)
geometry = Dict{String,Any}(k => P.rocket_geometry(form(v)) for (k, v) in GEOM)

open(ARGS[1], "w") do io
    print(io, P.json(Dict{String,Any}("missions" => missions, "geometry" => geometry)))
end
