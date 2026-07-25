# Local mission-control panel: configure, run, and explore missions in the
# browser. Pure stdlib (raw Sockets HTTP) — no package dependencies.
#
# This file defines the panel and starts nothing. `scripts/panel.jl` is the
# entry point that runs it, and `test/panel_http.jl` starts one on an
# ephemeral port — which is only possible because loading this file has no
# side effects.

module PanelApp

push!(LOAD_PATH, joinpath(@__DIR__, "..", "src"))
using SatelliteSim
using Sockets
using Printf

const PAGE_PATH = joinpath(@__DIR__, "panel_page.html")
const LAUNCH_PATH = joinpath(@__DIR__, "launch_page.html")

# ---------------------------------------------------------------- helpers --

"Minimal URL decoding (+, %xx)."
function urldecode(s::AbstractString)
    buf = IOBuffer()
    i = firstindex(s)
    while i <= lastindex(s)
        c = s[i]
        if c == '+'
            write(buf, ' ')
        elseif c == '%' && i + 2 <= lastindex(s)
            write(buf, parse(UInt8, s[i+1:i+2]; base = 16))
            i += 2
        else
            write(buf, c)
        end
        i = nextind(s, i)
    end
    String(take!(buf))
end

"Parse an application/x-www-form-urlencoded body into a Dict."
function parse_form(body::AbstractString)
    d = Dict{String,String}()
    for pair in split(body, '&'; keepempty = false)
        kv = split(pair, '='; limit = 2)
        d[urldecode(kv[1])] = length(kv) == 2 ? urldecode(kv[2]) : ""
    end
    d
end

getf(d, k, def) = haskey(d, k) && !isempty(d[k]) ? parse(Float64, d[k]) : def
"A form checkbox: present and truthy, absent and defaulted."
getb(d, k, def) = haskey(d, k) ? gets(d, k, "0") in ("1", "true", "on") : def

"Tiny JSON writer: handles Dict/Vector/String/Number/Bool/Nothing/Symbol."
function json(io::IO, x)
    if x isa AbstractDict
        print(io, '{')
        first_ = true
        for (k, v) in x
            first_ || print(io, ',')
            json(io, string(k)); print(io, ':'); json(io, v)
            first_ = false
        end
        print(io, '}')
    elseif x isa Union{AbstractVector,Tuple}
        print(io, '[')
        for (i, v) in enumerate(x)
            i > 1 && print(io, ',')
            json(io, v)
        end
        print(io, ']')
    elseif x isa AbstractString || x isa Symbol
        print(io, '"')
        for c in string(x)
            c == '"' ? print(io, "\\\"") :
            c == '\\' ? print(io, "\\\\") :
            c == '\n' ? print(io, "\\n") : print(io, c)
        end
        print(io, '"')
    elseif x isa Bool || x === nothing
        print(io, x === nothing ? "null" : x)
    elseif x isa AbstractFloat
        isfinite(x) ? print(io, round(x; sigdigits = 8)) : print(io, "null")
    else
        print(io, x)
    end
end
json(x) = sprint(json, x)

# ------------------------------------------------------------ mission api --

"Panel string parameter with a default."
gets(p, k, def) = (v = get(p, k, ""); isempty(strip(String(v))) ? def : strip(String(v)))

"""
Build one stage from the panel's `s<k>_*` fields.

Naming an engine from the catalogue derives thrust, Isp, exit area and
propellant from it (times the engine count), and estimates dry mass unless
one is given. Leaving the engine on "manual" keeps the explicit numbers and
only takes the propellant, which still sets the tank's physical size.
"""
function stage_from_params(p, pre::String, name::Symbol, dia::Float64;
                           dry, prop, thrust_kn, isp, ae, ptype = "kerolox",
                           nedef = 1)
    ne = clamp(round(Int, getf(p, pre * "engines", Float64(nedef))), 1, 33)
    eng = gets(p, pre * "engine", "manual")
    dst = max(0.3, getf(p, pre * "diameter", dia))     # this stage's own width
    mdry_in = getf(p, pre * "dry", dry)
    # the propellant has to be resolved first: sizing a stage by its length
    # needs the bulk density to say how much that volume actually holds
    pr = eng != "manual" ? lookup_engine(Symbol(eng)).prop :
                           propellant(Symbol(gets(p, pre * "propellant", ptype)))
    mprop = if gets(p, pre * "size_by", "prop") == "length"
        # invert the barrel-length rule: a wider stage of the same length
        # holds proportionally more
        L = max(getf(p, pre * "len", 10.0), 0.95 * dst)
        max(1.0, (L - 0.9dst) * bulk_density(pr) * pi * (dst/2)^2 / 1.15)
    else
        getf(p, pre * "prop", prop)
    end
    if eng != "manual"
        auto = gets(p, pre * "dry_auto", "0") in ("1", "true", "on")
        return sized_stage(name; engine = Symbol(eng), n_engines = ne,
                           prop_mass = mprop, diameter = dst,
                           dry_mass = auto ? nothing : mdry_in)
    end
    Stage(name, mdry_in, mprop, getf(p, pre * "thrust_kn", thrust_kn) * 1e3,
          getf(p, pre * "isp", isp), ae, pr, ne, dst)
end

"Barrel length a stage needs for its propellant load [m]."
stage_length(st::Stage, vehicle_d::Float64) =
    (d = stage_diameter(st, vehicle_d);
     stage_volume(st) / (pi * (d/2)^2) * 1.15 + 0.9d)

"""
Per-role stage defaults. Stage 1 is the booster, the last stage is the kick
stage that performs TLI, and anything between them is an upper stage — so
the form still has sensible starting numbers whatever the stack height.
"""
const STAGE_ROLE = Dict(
    :booster => (dry = 3800.0, prop = 42000.0, thrust = 950.0, isp = 305.0,
                 ae = 0.80, ptype = "kerolox", ne = 5),
    :upper   => (dry =  900.0, prop =  9500.0, thrust =  95.0, isp = 345.0,
                 ae = 0.0,  ptype = "kerolox", ne = 1),
    :kick    => (dry =  140.0, prop =   950.0, thrust =  15.0, isp = 315.0,
                 ae = 0.0,  ptype = "hypergolic", ne = 1),
)
stage_role(k, nst) = k == 1 ? :booster : k == nst ? :kick : :upper

"""
Default scale for an inserted upper stage. Stage 2 keeps the reference
figures; each stage above it starts a quarter the size, so raising the
stack height does not silently make the vehicle too heavy to fly.
"""
stage_scale(k, nst) = stage_role(k, nst) === :upper ? 0.25^(k - 2) : 1.0

"Number of stages the panel is configured for (2-5)."
n_stages(p) = clamp(round(Int, getf(p, "nstages", 3.0)), 2, 5)

"Number of strap-on boosters (0 = none, i.e. a plain serial stack)."
n_boosters(p) = clamp(round(Int, getf(p, "nboost", 0.0)), 0, 8)

"""
Default strap-on: a kerolox booster roughly a third of the reference first
stage, sized so a pair meaningfully changes the vehicle without swamping it.
"""
const BOOSTER_ROLE = (dry = 900.0, prop = 12000.0, thrust = 380.0, isp = 285.0,
                      ae = 0.32, ptype = "kerolox", ne = 2)

"""
Build the strap-on booster sets from the panel's `b_*` fields. `nboost` is
the number of boosters in the (single) set; zero means a plain serial stack
and returns nothing at all, so a vehicle without strap-ons is exactly the
vehicle it was before they existed.
"""
function boosters_from_params(p, dia::Float64)
    nb = n_boosters(p)
    nb == 0 && return BoosterSet[]
    d = BOOSTER_ROLE
    st = stage_from_params(p, "b_", :strap, dia * 0.85; dry = d.dry, prop = d.prop,
                           thrust_kn = d.thrust, isp = d.isp, ae = d.ae,
                           ptype = d.ptype, nedef = d.ne)
    [BoosterSet(stage = st, count = nb,
                ignition_delay = max(0.0, getf(p, "b_ign_delay", 0.0)),
                sep_delay = max(0.0, getf(p, "b_sep_delay", 0.0)),
                core_throttle = clamp(getf(p, "b_throttle", 100.0) / 100, 0.2, 1.0))]
end

"Build a LaunchVehicle from panel parameters."
function lv_from_params(p)
    dia = getf(p, "diameter", 1.8)
    nst = n_stages(p)
    stages = map(1:nst) do k
        d = STAGE_ROLE[stage_role(k, nst)]
        f = stage_scale(k, nst)
        nm = k == nst ? :sablek : Symbol(:sable, k)
        stage_from_params(p, "s$(k)_", nm, dia; dry = d.dry * f, prop = d.prop * f,
                          thrust_kn = d.thrust * f, isp = d.isp, ae = d.ae * f,
                          ptype = d.ptype, nedef = d.ne)
    end
    LaunchVehicle(
        # the page sends the name of whatever preset is loaded, so the livery
        # and the launch view say what you are actually flying
        name = gets(p, "vname", "Sable (panel)"),
        stages = stages,
        fairing_mass = getf(p, "fairing", 150.0),
        payload_mass = getf(p, "pod_mass", 350.0),
        # drag acts on the widest cross-section in the stack; strap-ons add
        # their own frontal area on top, but only while they are attached
        sref = pi * (maximum(stage_diameter(s, dia) for s in stages) / 2)^2,
        cd = SatelliteSim.LV_CD_TABLE,
        boosters = boosters_from_params(p, dia),
    )
end

"Should the ascent tuner also search for the best pitch-over kick?"
opt_kick(p) = gets(p, "opt_kick", "0") in ("1", "true", "on")

"Pitch-over kick angle [rad] — the one guidance number a big stack has to change."
kick_rad(p) = deg2rad_(clamp(getf(p, "kick_deg", 8.0), 0.5, 30.0))

"Which mission the panel is flying: the free-return flyby or a landing."
mission_mode(p) = gets(p, "mode", "flyby") == "landing" ? :landing : :flyby

"Build the lander from the panel's `l_*` fields."
lander_from_params(p) = Lander(
    name = :lander,
    mdry = max(100.0, getf(p, "l_dry", 3500.0)),
    mprop = max(10.0, getf(p, "l_prop", 9000.0)),
    thrust = max(1.0e3, getf(p, "l_thrust_kn", 45.0) * 1e3),
    isp = clamp(getf(p, "l_isp", 311.0), 100.0, 500.0),
    throttle_min = clamp(getf(p, "l_throttle_min", 10.0) / 100, 0.02, 1.0),
    diameter = max(0.5, getf(p, "l_diameter", 4.2)))

"Decimate a vector to at most n points (keeping ends)."
function deci(v, n)
    length(v) <= n && return collect(Float64, v)
    idx = unique(round.(Int, range(1, length(v); length = n)))
    Float64[v[i] for i in idx]
end
deci_idx(len, n) = len <= n ? collect(1:len) :
                   unique(round.(Int, range(1, len; length = n)))

"""
Shared 3D-scene payload: the pad-to-wherever track in true ECI geometry,
units of 1000 km, decimated for the wire. Both missions fly the same launch
and trans-lunar legs, so both scenes are built from the same code.
"""
function scene_payload(asc, cis)
    L = cis.log
    k = max(2, length(L.t) ÷ 3)
    nrm = SatelliteSim.vunit(SatelliteSim.vcross((L.mx[1], L.my[1], L.mz[1]),
                                                 (L.mx[k], L.my[k], L.mz[k])))
    idx = deci_idx(length(L.t), 1600)
    px = Float64[]; py = Float64[]; pz = Float64[]
    mx = Float64[]; my = Float64[]; mz = Float64[]
    tt = Float64[]; pp = Int[]
    for i in idx
        push!(px, L.rx[i] / 1e6); push!(py, L.ry[i] / 1e6); push!(pz, L.rz[i] / 1e6)
        push!(mx, L.mx[i] / 1e6); push!(my, L.my[i] / 1e6); push!(mz, L.mz[i] / 1e6)
        push!(tt, L.t[i]); push!(pp, L.phase[i])
    end
    AL = asc.log
    aidx = deci_idx(length(AL.t), 400)
    cisd = Dict("t" => tt, "x" => px, "y" => py, "z" => pz,
                "mx" => mx, "my" => my, "mz" => mz, "ph" => pp,
                "n" => [nrm[1], nrm[2], nrm[3]])
    asc3d = Dict("t" => [AL.t[i] for i in aidx],
                 "x" => [AL.rx[i] / 1e6 for i in aidx],
                 "y" => [AL.ry[i] / 1e6 for i in aidx],
                 "z" => [AL.rz[i] / 1e6 for i in aidx])
    ascent = Dict("t" => deci(AL.t[aidx], 400), "h" => deci(AL.h[aidx] ./ 1e3, 400),
                  "v" => deci(AL.vrel[aidx], 400), "qbar" => deci(AL.qbar[aidx] ./ 1e3, 400),
                  "gamma" => deci(AL.gamma[aidx], 400), "mach" => deci(AL.mach[aidx], 400),
                  "thrust" => deci(AL.thrust[aidx], 400), "m" => deci(AL.m[aidx], 400),
                  "dr" => deci(AL.downrange[aidx], 400))
    (cis = cisd, asc3d = asc3d, ascent = ascent)
end

"""
    descent_local(ls) -> Dict

The powered descent in a frame anchored at the touchdown point: `lx` metres of
surface arc along the direction of travel (negative before touchdown, zero at
it), `ly` metres above the mean sphere, `lz` metres of crossrange. Plus the
Moon-fixed basis at the site, so the viewer can rebuild the *same* terrain the
descent was flown over — the surface is a pure function of direction, and this
is the direction.

Anchoring at touchdown rather than at ignition is what makes the view work:
the interesting part of a descent is the last kilometre, and a frame pinned
250 km upstream puts it at the far end of a float.
"""
function descent_local(ls)
    S = SatelliteSim
    eph = ls.eph
    D = ls.descent.log
    t_td = ls.t_touchdown
    utd = S.vunit(moonfixed(ls.descent.r, t_td, eph))
    # the descent plane, from the state at ignition
    r0 = (D.x[1], D.y[1], D.z[1])
    r1 = (D.x[2], D.y[2], D.z[2])
    hf = S.vunit(S.vcross(moonfixed(r0, ls.t_pdi, eph),
                          moonfixed(S.vsub(r1, r0), ls.t_pdi, eph)))
    ed = S.vunit(S.vcross(hf, utd))          # direction of travel
    ec = S.vcross(ed, utd)                   # crossrange, right-handed with up
    lx = Float64[]; ly = Float64[]; lz = Float64[]
    for i in eachindex(D.t)
        r = (D.x[i], D.y[i], D.z[i])
        uf = S.vunit(moonfixed(r, ls.t_pdi + D.t[i], eph))
        b = asin(clamp(S.vdot(uf, ec), -1.0, 1.0))
        a = atan(S.vdot(uf, ed), S.vdot(uf, utd))
        push!(lx, R_MOON * a); push!(lz, R_MOON * b)
        push!(ly, S.vnorm(r) - R_MOON)
    end
    Dict("lx" => lx, "ly" => ly, "lz" => lz,
         "u" => collect(utd), "ed" => collect(ed), "ec" => collect(ec),
         # the Earth never moves in this sky: the Moon keeps one face to it, so
         # longitude zero is the sub-Earth point and everything else is fixed
         "earth" => [ed[1], utd[1], ec[1]], "earth_d" => A_MOON)
end

"Terrain parameters, so the viewer draws the ground the descent was flown over."
terrain_payload(tr::Union{Nothing,LunarTerrain}) =
    tr === nothing ?
    Dict("seed" => 0, "relief" => 0.0, "d_max" => 1.0, "classes" => 0,
         "ratio" => 2.6, "density" => 0.0, "rough" => 0.0) :
    Dict("seed" => Int(tr.seed), "relief" => tr.relief, "d_max" => tr.d_max,
         "classes" => tr.classes, "ratio" => tr.ratio, "density" => tr.density,
         "rough" => tr.rough)

"Ascent events, named by the vehicle's own stages."
ascent_events(asc) = Any[Dict("phase" => "ascent", "name" => string(e.name),
                              "t" => e.t) for e in asc.events]

"""
Fly the lunar landing mission for the panel: the same launch and trans-lunar
legs as the flyby, then insertion, the lunar-orbit coast and the powered
descent. The lunar phase is returned in Moon-centred coordinates (km),
because that is the only frame in which a 15 km descent is visible at all
next to a 384,000 km transfer.
"""
function panel_landing(p)::Dict{String,Any}
    lander = lander_from_params(p)
    lv0 = lv_from_params(p)
    # the launcher carries the lander, whatever the pod-mass field says
    lv = LaunchVehicle(name = lv0.name, stages = lv0.stages,
                       fairing_mass = lv0.fairing_mass,
                       payload_mass = lander_mass(lander),
                       sref = lv0.sref, cd = lv0.cd, boosters = lv0.boosters)
    # The real Moon by default: terrain under the vehicle, mascons around it,
    # navigation error corrected by landing radar, hazard avoidance choosing
    # the touchdown point. Untick it and the descent is flown onto a smooth
    # sphere by a vehicle that knows exactly where it is, which is what every
    # figure in this repo predating terrain was flown against.
    real_moon = !getb(p, "plain_moon", false)
    terr = real_moon ? LunarTerrain() : nothing
    ls = moonlanding(
        lander = lander, lv = lv,
        terrain = terr,
        field = real_moon ? LunarGravity() : nothing,
        nav = real_moon ? DescentNav() : nothing,
        hazard = real_moon ? HazardScan() : nothing,
        h_park = getf(p, "h_park_km", 200.0) * 1e3,
        h_moon_park = getf(p, "h_moon_park_km", 100.0) * 1e3,
        h_pdi = getf(p, "h_pdi_km", 15.0) * 1e3,
        n_rev = clamp(round(Int, getf(p, "n_rev", 1.0)), 0, 12),
        inclination = deg2rad_(getf(p, "incl_deg", 28.5)),
        hp_return = getf(p, "hp_return_km", 50.0) * 1e3,
        kick_angle = kick_rad(p),
        optimize_kick = opt_kick(p),
    )
    asc, cis, d = ls.ascent, ls.cislunar, ls.descent
    el = asc.elements
    sc = scene_payload(asc, cis)

    # Moon-centred tracks, in km: the parking orbit, the descent ellipse and
    # the powered descent itself
    O = ls.orbit
    oidx = deci_idx(length(O.t), 900)
    D = d.log
    didx = deci_idx(length(D.t), 700)

    events = ascent_events(asc)
    push!(events, Dict("phase" => "cislunar", "name" => "tli_ignition", "t" => cis.t_tli))
    push!(events, Dict("phase" => "cislunar", "name" => "tli_cutoff",
                       "t" => cis.t_tli + cis.burn_duration))
    push!(events, Dict("phase" => "lunar", "name" => "loi", "t" => ls.t_loi))
    push!(events, Dict("phase" => "lunar", "name" => "doi", "t" => ls.t_doi))
    push!(events, Dict("phase" => "lunar", "name" => "pdi", "t" => ls.t_pdi))
    push!(events, Dict("phase" => "lunar", "name" => "high_gate",
                       "t" => ls.t_pdi + d.t_gate))
    push!(events, Dict("phase" => "lunar", "name" => string(d.outcome),
                       "t" => ls.t_touchdown))

    prop_margin = cis.m - (ls.lv.stages[end].mdry + ls.lv.payload_mass)
    Dict{String,Any}(
        "ok" => true, "mode" => "landing",
        "metrics" => Dict(
            "on_target" => d.outcome === :touchdown,
            "outcome" => string(d.outcome),
            "liftoff_t" => liftoff_mass(ls.lv) / 1e3,
            "park_perigee_km" => (el.rp - RE_MEAN) / 1e3,
            "park_apogee_km" => (el.ra - RE_MEAN) / 1e3,
            "incl_deg" => rad2deg_(el.i),
            "tli_dv" => cis.dv_tli,
            "tli_burn_s" => cis.burn_duration,
            "prop_margin_kg" => prop_margin,
            "lander_wet_t" => lander_mass(lander) / 1e3,
            "lander_dv" => lander_dv(lander),
            "perilune_km" => cis.perilune_alt / 1e3,
            "t_perilune_d" => cis.t_perilune / 86400,
            "loi_dv" => ls.dv_loi,
            "doi_dv" => ls.dv_doi,
            "braking_dv" => d.dv_braking,
            "terminal_dv" => d.dv_terminal,
            "descent_dv" => d.dv_braking + d.dv_terminal,
            "descent_s" => d.t_touchdown,
            "gate_s" => d.t_gate,
            "downrange_km" => d.downrange / 1e3,
            "touchdown_v" => d.v_vertical,
            "touchdown_vh" => d.v_horizontal,
            "min_throttle_pct" => 100 * d.min_throttle,
            "prop_left_kg" => d.prop_left,
            "hover_s" => d.hover_s,
            "land_lat" => rad2deg_(ls.lat_land),
            "land_lon" => rad2deg_(ls.lon_land),
            "ground_elev_m" => d.elev,
            "ground_slope_deg" => rad2deg_(d.slope),
            "site_score_deg" => isnan(d.site_score) ? 0.0 : rad2deg_(d.site_score),
            "site_was_deg" => isnan(d.site_score_nominal) ? 0.0 :
                              rad2deg_(d.site_score_nominal),
            "redesignate_m" => d.redesignated,
            "nav_err_m" => d.nav_err,
            "nav_alt_err_m" => d.nav_dh,
            "t_pdi_d" => ls.t_pdi / 86400,
            "t_days" => ls.t_touchdown / 86400,
        ),
        "cis" => sc.cis, "asc3d" => sc.asc3d, "ascent" => sc.ascent,
        # no entry leg on a landing mission; the client draws whatever is here
        "ent3d" => Dict("t" => Float64[], "x" => Float64[], "y" => Float64[],
                        "z" => Float64[]),
        "sites" => Dict("launch_lat" => rad2deg_(ls.guid.site_lat),
                        "launch_lon" => rad2deg_(ls.guid.site_lon)),
        "moon" => Dict(
            "r_km" => R_MOON / 1e3,
            "orbit" => Dict("t" => [O.t[i] for i in oidx],
                            "x" => [O.x[i] / 1e3 for i in oidx],
                            "y" => [O.y[i] / 1e3 for i in oidx],
                            "z" => [O.z[i] / 1e3 for i in oidx],
                            "ph" => [O.phase[i] for i in oidx]),
            "descent" => Dict("x" => [D.x[i] / 1e3 for i in didx],
                              "y" => [D.y[i] / 1e3 for i in didx],
                              "z" => [D.z[i] / 1e3 for i in didx])),
        "descent" => Dict(
            "t" => [D.t[i] for i in didx],
            "h" => [D.h[i] / 1e3 for i in didx],
            "dr" => [D.downrange[i] / 1e3 for i in didx],
            "v" => [D.v[i] for i in didx],
            "vh" => [D.vh[i] for i in didx],
            "vv" => [D.vv[i] for i in didx],
            "thr" => [100 * D.throttle[i] for i in didx],
            "pitch" => [rad2deg_(D.pitch[i]) for i in didx],
            "elev" => [D.elev[i] for i in didx],
            "navdh" => [D.nav_dh[i] for i in didx],
            "m" => [D.m[i] for i in didx]),
        "site" => merge(descent_local(ls),
                        Dict("terrain" => terrain_payload(terr),
                             "diameter" => ls.lander.diameter,
                             "t_pdi" => ls.t_pdi, "t_gate" => ls.t_pdi + d.t_gate,
                             "t_td" => ls.t_touchdown)),
        "events" => events,
    )
end

function panel_mission(p)::Dict{String,Any}
    mission_mode(p) === :landing && return panel_landing(p)
    ms = moonshot(
        pod_mass = getf(p, "pod_mass", 350.0),
        h_park = getf(p, "h_park_km", 200.0) * 1e3,
        hp_moon = getf(p, "hp_moon_km", 2000.0) * 1e3,
        hp_return = getf(p, "hp_return_km", 50.0) * 1e3,
        inclination = deg2rad_(getf(p, "incl_deg", 28.5)),
        lv = lv_from_params(p),
        tli_mag_err = getf(p, "tli_mag_err_pct", 0.0) / 100,
        tli_point_err = deg2rad_(getf(p, "tli_point_err_deg", 0.0)),
        kick_angle = kick_rad(p),
        optimize_kick = opt_kick(p),
    )
    asc, cis, ent = ms.ascent, ms.cislunar, ms.entry
    el = asc.elements

    # 3D scene payload: true ECI geometry in units of 1000 km. The client
    # renders the inclined trajectory plane, the textured globe about the real
    # pole (scene z = ECI z), and launch/splashdown markers fixed to the
    # rotating surface — all in one consistent frame.
    sc = scene_payload(asc, cis)

    EL = ent.log
    eidx = deci_idx(length(EL.t), 500)

    # the entry log is geodetic — rebuild ECI so it joins the same scene
    ex3 = Float64[]; ey3 = Float64[]; ez3 = Float64[]; et3 = Float64[]
    for i in eidx
        re_ = ecef_from_geodetic(EL.lat[i], EL.lon[i], EL.h[i])
        th = SatelliteSim.earth_rotation_angle(0.0, EL.t[i])
        reci = SatelliteSim.rot_z(re_, -th)
        push!(ex3, reci[1] / 1e6); push!(ey3, reci[2] / 1e6); push!(ez3, reci[3] / 1e6)
        push!(et3, EL.t[i])
    end

    prop_margin = cis.m - (ms.lv.stages[end].mdry + ms.lv.payload_mass)
    ei = findfirst(e -> e.name == :entry_interface, ent.events)

    # did the free-return design actually hit its targets? (a prop-starved
    # TLI still "flies", but the result is not the requested mission)
    hp_moon = getf(p, "hp_moon_km", 2000.0) * 1e3
    hp_ret = getf(p, "hp_return_km", 50.0) * 1e3
    on_target = abs(cis.perilune_alt - hp_moon) <= max(0.05 * hp_moon, 50e3) &&
                abs(cis.vac_perigee_alt - hp_ret) <= 20e3

    events = ascent_events(asc)
    push!(events, Dict("phase" => "cislunar", "name" => "tli_ignition", "t" => cis.t_tli))
    push!(events, Dict("phase" => "cislunar", "name" => "tli_cutoff",
                       "t" => cis.t_tli + cis.burn_duration))
    push!(events, Dict("phase" => "cislunar", "name" => "perilune", "t" => cis.t_perilune))
    push!(events, Dict("phase" => "cislunar", "name" => "entry_handoff", "t" => cis.t))
    for e in ent.events
        push!(events, Dict("phase" => "entry", "name" => string(e.name), "t" => e.t))
    end

    Dict{String,Any}(
        "ok" => true, "mode" => "flyby",
        "metrics" => Dict(
            "on_target" => on_target,
            "tcm_dv" => ms.cruise === nothing ? nothing : ms.cruise.tcm_dv,
            "tcm_prop_kg" => ms.cruise === nothing ? nothing : ms.cruise.tcm_prop,
            "rcs_margin_kg" => ms.cruise === nothing ? nothing : ms.cruise.rcs.margin,
            "liftoff_t" => liftoff_mass(ms.lv) / 1e3,
            "park_perigee_km" => (el.rp - RE_MEAN) / 1e3,
            "park_apogee_km" => (el.ra - RE_MEAN) / 1e3,
            "incl_deg" => rad2deg_(el.i),
            "tli_dv" => cis.dv_tli,
            "tli_burn_s" => cis.burn_duration,
            "prop_margin_kg" => prop_margin,
            "perilune_km" => cis.perilune_alt / 1e3,
            "t_perilune_d" => cis.t_perilune / 86400,
            "vac_perigee_km" => cis.vac_perigee_alt / 1e3,
            "ei_v_ms" => ei === nothing ? NaN : ent.events[ei].vrel,
            "peak_g" => ent.peak_gload,
            "peak_q_wcm2" => ent.peak_qdot / 1e4,
            "heat_mj" => ent.heat_load / 1e6,
            "splash_lat" => rad2deg_(ent.lat_splash),
            "splash_lon" => rad2deg_(ent.lon_splash),
            "v_splash" => ent.v_splash,
            "t_days" => ent.t_splash / 86400,
        ),
        "cis" => sc.cis,
        "asc3d" => sc.asc3d,
        "ent3d" => Dict("t" => et3, "x" => ex3, "y" => ey3, "z" => ez3),
        "sites" => Dict(
            "launch_lat" => rad2deg_(ms.guid.site_lat),
            "launch_lon" => rad2deg_(ms.guid.site_lon),
            "splash_lat" => rad2deg_(ent.lat_splash),
            "splash_lon" => rad2deg_(ent.lon_splash),
        ),
        "ascent" => sc.ascent,
        "entry" => Dict("t" => deci(EL.t[eidx] .- EL.t[1], 500), "h" => deci(EL.h[eidx] ./ 1e3, 500),
                        "v" => deci(EL.vrel[eidx], 500), "g" => deci(EL.gload[eidx], 500),
                        "q" => deci((EL.qdot_conv[eidx] .+ EL.qdot_rad[eidx]) ./ 1e4, 500)),
        "events" => events,
    )
end

"""
Metrics a solve can target. All are scalars from a completed mission, so
each evaluation is a full design-and-fly of the chain.
"""
const SOLVE_METRICS = ["prop_margin_kg", "perilune_km", "vac_perigee_km",
                       "peak_g", "peak_q_wcm2", "t_days", "liftoff_t",
                       "park_apogee_km", "v_splash", "tli_dv", "heat_mj"]

"Metrics a landing mission can be solved against."
const LANDING_METRICS = ["prop_left_kg", "hover_s", "descent_dv", "loi_dv",
                         "touchdown_v", "touchdown_vh", "downrange_km",
                         "prop_margin_kg", "min_throttle_pct", "liftoff_t",
                         "tli_dv", "t_days", "ground_slope_deg",
                         "ground_elev_m", "redesignate_m", "nav_alt_err_m"]

"The metric list for whichever mission the panel is configured for."
solve_metrics(p) = mission_mode(p) === :landing ? LANDING_METRICS : SOLVE_METRICS

"""
Lock every field but one and solve it so a mission metric hits a target —
"the heaviest pod that still leaves propellant in the kick stage" is
`pod_mass` against `prop_margin_kg` = 0.

Each step costs a whole mission, so this is a bracketing search with a hard
iteration budget rather than a scan; a run that fails outright counts as
past the feasible edge and the search retreats from it.
"""
function run_solve(p)::Dict{String,Any}
    param  = get(p, "solve_param", "pod_mass")
    metric = get(p, "solve_metric", "prop_margin_kg")
    param in sweepable(p) || return Dict{String,Any}("ok" => false,
        "error" => "cannot solve for: $param")
    metric in solve_metrics(p) || return Dict{String,Any}("ok" => false,
        "error" => "cannot target metric: $metric")
    lo = getf(p, "solve_min", 200.0)
    hi = getf(p, "solve_max", 600.0)
    target = getf(p, "solve_target", 0.0)
    budget = clamp(round(Int, getf(p, "solve_iters", 14.0)), 4, 30)
    res = find_root(lo, hi; target = target, max_iter = budget) do x
        q = copy(p)
        q[param] = string(x)
        v = panel_mission(q)["metrics"][metric]
        v === nothing ? NaN : Float64(v)
    end
    Dict{String,Any}(
        "ok" => true, "param" => param, "metric" => metric,
        "target" => target, "x" => res.x, "value" => res.value,
        "status" => string(res.status), "iterations" => res.iterations,
        # the bounds actually searched: an end that produced no valid mission
        # was walked inward, which is worth saying out loud
        "lo" => res.lo, "hi" => res.hi,
        "asked_lo" => min(lo, hi), "asked_hi" => max(lo, hi),
        "history" => [Dict("x" => h[1], "value" => h[2]) for h in res.history],
    )
end

"""
Procedural rocket geometry for the panel's vehicle viewer.

Built from the same `LaunchVehicle` the mission flies, so the drawing and
the trajectory can never disagree: tank lengths come from each stage's
propellant density and the bells from its engine count.
"""
function rocket_geometry(p)::Dict{String,Any}
    d = getf(p, "diameter", 1.8)
    lv = lv_from_params(p)
    mesh, secs = rocket_mesh(lv; diameter = d, nseg = 36)
    nst = length(lv.stages)
    # Static performance, so a bad stack is obvious before it is flown: the
    # mass each stage actually pushes is everything above it (the fairing
    # only while it is still on, which through ascent means stage 1).
    payload_above(k) = stack_mass_above(lv, k + 1; fairing = k == 1)
    dv(k) = stage_dv(lv.stages[k], payload_above(k))
    twr(k) = (s = lv.stages[k];
              stage_thrust(s, k == 1 ? SatelliteSim.P0_SEA : 0.0) /
              ((payload_above(k) + s.mdry + s.mprop) * G0))
    # Leaving the pad it is the whole stack that has to be lifted, strap-ons
    # and all, by whatever is lit at t = 0.
    pad_twr = pad_thrust(lv) / (liftoff_mass(lv) * G0)
    # A booster set's delta-v is not additive with the core's — they push the
    # same stack at the same time — so it is reported as the impulse it adds.
    bdv(b) = b.count * b.stage.mprop * G0 * b.stage.isp_vac / liftoff_mass(lv)
    Dict{String,Any}(
        "ok" => true,
        # boosters are appended after the core stack, so the tallest section
        # is not necessarily the last one
        "length" => maximum(s.x1 for s in secs),
        "diameter" => d,
        "liftoff_mass_kg" => liftoff_mass(lv),
        "liftoff_twr" => pad_twr,
        "total_dv_mps" => sum(dv(k) for k in 1:nst) +
                          sum(bdv, lv.boosters; init = 0.0),
        "boosters" => [Dict("name" => string(b.stage.name),
                            "count" => b.count,
                            "propellant" => string(b.stage.prop.name),
                            "engines" => b.stage.n_engines,
                            "dry_kg" => b.stage.mdry, "prop_kg" => b.stage.mprop,
                            "thrust_kn" => b.stage.thrust_vac / 1e3,
                            "isp_s" => b.stage.isp_vac,
                            "diameter_m" => stage_diameter(b.stage, d),
                            "length_m" => stage_length(b.stage, d),
                            "burn_s" => stage_burn_time(b.stage),
                            "set_mass_kg" => booster_mass(b),
                            "dv_mps" => bdv(b),
                            "core_throttle" => b.core_throttle,
                            "ignition_delay_s" => b.ignition_delay,
                            "sep_delay_s" => b.sep_delay)
                       for b in lv.boosters],
        "stages" => [Dict("name" => string(s.name),
                          "propellant" => string(s.prop.name),
                          "engines" => s.n_engines,
                          "dry_kg" => s.mdry, "prop_kg" => s.mprop,
                          "thrust_kn" => s.thrust_vac / 1e3,
                          "isp_s" => s.isp_vac,
                          "diameter_m" => stage_diameter(s, d),
                          "length_m" => stage_length(s, d),
                          "volume_m3" => stage_volume(s),
                          "dv_mps" => dv(k),
                          "twr" => twr(k),
                          "burn_s" => stage_burn_time(s),
                          "interstage_m" => k < nst ?
                              interstage_length(stage_diameter(s, d),
                                  stage_diameter(lv.stages[k+1], d)) : 0.0)
                     for (k, s) in enumerate(lv.stages)],
        "sections" => [Dict("name" => string(s.name), "x0" => s.x0, "x1" => s.x1,
                            "t0" => s.t0, "t1" => s.t1)
                       for s in secs],
        "tris" => [Float64[t[1]..., t[2]..., t[3]...] for t in mesh.tris],
    )
end

"Numeric parameters that may be swept or solved for, for this stack height."
sweepable(p) = vcat(
    ["pod_mass", "h_park_km", "hp_moon_km", "hp_return_km", "incl_deg",
     "diameter", "fairing", "kick_deg"],
    mission_mode(p) === :landing ?
        ["l_dry", "l_prop", "l_thrust_kn", "l_isp", "l_throttle_min",
         "h_moon_park_km", "h_pdi_km", "n_rev"] : String[],
    ["s$(k)_$f" for k in 1:n_stages(p)
                for f in ("prop", "dry", "isp", "thrust_kn", "engines")],
    n_boosters(p) == 0 ? String[] :
        ["nboost", "b_prop", "b_dry", "b_isp", "b_thrust_kn", "b_engines",
         "b_throttle", "b_diameter"])

function run_sweep(p)::Dict{String,Any}
    param = get(p, "sweep_param", "pod_mass")
    param in sweepable(p) || return Dict{String,Any}("ok" => false,
        "error" => "unknown sweep parameter: $param")
    lo = getf(p, "sweep_min", 250.0)
    hi = getf(p, "sweep_max", 450.0)
    nv = clamp(Int(getf(p, "sweep_n", 9.0)), 2, 41)
    vals = collect(range(lo, hi; length = nv))
    runs = Vector{Any}(undef, nv)
    Threads.@threads for i in 1:nv
        q = copy(p)
        q[param] = string(vals[i])
        runs[i] = try
            r = panel_mission(q)
            Dict("ok" => true, "metrics" => r["metrics"])
        catch err
            Dict("ok" => false, "error" => sprint(showerror, err))
        end
    end
    Dict{String,Any}("ok" => true, "param" => param, "values" => vals, "runs" => runs)
end

# -------------------------------------------------------------- http loop --

"When this server started, so /api/health can report how long it has been up."
const START_TIME = Ref(time())

"""
Liveness and environment, for a browser or a script that wants to know the
panel is actually there before blaming the network.
"""
health_payload() = Dict{String,Any}(
    "ok" => true,
    "uptime_s" => time() - START_TIME[],
    "julia" => string(VERSION),
    "threads" => Base.Threads.nthreads())

"""
What the page builds its dropdowns from, so the UI can never offer a
propellant or an engine the simulator does not have.
"""
catalogue_payload() = Dict{String,Any}(
    "propellants" => [Dict("name" => string(k), "bulk" => bulk_density(v))
                      for (k, v) in sort(collect(PROPELLANTS), by = first)],
    "engines" => [Dict("name" => string(k),
                       "thrust_kn" => v.thrust_vac / 1e3,
                       "isp_vac" => v.isp_vac, "isp_sl" => v.isp_sl,
                       "mass_kg" => v.mass,
                       "propellant" => string(v.prop.name))
                  for (k, v) in sort(collect(ENGINES), by = first)],
    "solve_metrics" => SOLVE_METRICS,
    "landing_metrics" => LANDING_METRICS,
    "max_stages" => 5)

"""
Read one request. Returns `(method, path, body)`, or `nothing` if the peer
closed before sending anything — a browser opening and dropping a speculative
connection is not an error worth answering.

Every parse here is total: a `Content-Length` that is not a number is treated
as absent rather than thrown, because throwing at this point is what leaves a
socket unanswered.
"""
function read_request(sock)
    reqline = readline(sock)
    isempty(reqline) && return nothing
    parts = split(reqline, ' ')
    length(parts) < 2 && return ("BAD", "/", "")
    method, path = String(parts[1]), String(parts[2])
    clen = 0
    while true
        h = readline(sock)
        (isempty(h) || h == "\r") && break
        if startswith(lowercase(h), "content-length:")
            kv = split(h, ':'; limit = 2)
            clen = something(tryparse(Int, strip(kv[2])), 0)
        end
    end
    body = clen > 0 ? String(read(sock, clen)) : ""
    (method, path, body)
end

"Write a complete response. `head` suppresses the body but keeps the headers."
function write_response(sock, status::AbstractString, ctype::AbstractString,
                        body::AbstractString; head::Bool = false)
    write(sock, "HTTP/1.1 $status\r\nContent-Type: $ctype\r\n" *
                "Content-Length: $(sizeof(body))\r\nConnection: close\r\n\r\n")
    head || write(sock, body)
    nothing
end

"""
Parse the form and call `f` on it, turning any failure into a result.

This is where the panel's error contract lives: a mission that cannot be
designed, or a field the user mistyped, is a *200 with `ok: false`* — the
pages read `j.ok` from a parsed body, so an HTTP error code would give them
nothing to show. Genuine server faults are the 500 in `handle`.
"""
function safe_call(f, body::AbstractString)
    try
        f(parse_form(body))
    catch err
        Dict{String,Any}("ok" => false, "error" => sprint(showerror, err))
    end
end

"""
    route(method, path, body) -> (status, content_type, payload)

Total function: every input produces a response, including an unknown route
and an unreadable page file.
"""
function route(method::AbstractString, path::AbstractString,
               body::AbstractString)
    try
        if method in ("GET", "HEAD") && (path == "/" || startswith(path, "/?"))
            # re-read per request so page edits show on refresh (dev-friendly)
            return ("200 OK", "text/html; charset=utf-8", read(PAGE_PATH, String))
        elseif method in ("GET", "HEAD") &&
               (path == "/launch" || startswith(path, "/launch?"))
            return ("200 OK", "text/html; charset=utf-8", read(LAUNCH_PATH, String))
        elseif method in ("GET", "HEAD") && path == "/api/catalogue"
            return ("200 OK", "application/json", json(catalogue_payload()))
        elseif method in ("GET", "HEAD") && path == "/api/health"
            return ("200 OK", "application/json", json(health_payload()))
        elseif method == "POST" && path == "/api/run"
            return ("200 OK", "application/json", json(safe_call(panel_mission, body)))
        elseif method == "POST" && path == "/api/sweep"
            return ("200 OK", "application/json", json(safe_call(run_sweep, body)))
        elseif method == "POST" && path == "/api/geometry"
            return ("200 OK", "application/json", json(safe_call(rocket_geometry, body)))
        elseif method == "POST" && path == "/api/solve"
            return ("200 OK", "application/json", json(safe_call(run_solve, body)))
        elseif method == "OPTIONS"
            # no CORS here — the panel is same-origin — but a bare 404 for a
            # preflight is a confusing thing to hand a browser
            return ("204 No Content", "text/plain", "")
        end
        return ("404 Not Found", "application/json",
                json(Dict{String,Any}("ok" => false,
                                      "error" => "no route for $method $path")))
    catch err
        return ("500 Internal Server Error", "application/json",
                json(Dict{String,Any}("ok" => false,
                                      "error" => sprint(showerror, err))))
    end
end

function handle(sock)
    t0 = time()
    method, path, status, nbytes = "-", "-", "500", 0
    try
        req = read_request(sock)
        req === nothing && return
        method, path, body = req
        st, ctype, payload = route(method, path, body)
        status = first(st, 3)
        nbytes = sizeof(payload)
        write_response(sock, st, ctype, payload; head = method == "HEAD")
    catch err
        try
            payload = json(Dict{String,Any}("ok" => false,
                                            "error" => sprint(showerror, err)))
            status = "500"
            nbytes = sizeof(payload)
            write_response(sock, "500 Internal Server Error",
                           "application/json", payload)
        catch
            # the socket itself is gone; nothing left to say
        end
    finally
        # One line per request. This is how an intermittent "Failed to fetch"
        # gets diagnosed from a record rather than from a hypothesis.
        @printf("[panel] %-7s %-44s %s %7.0f ms %9d B\n",
                method, first(path, 44), status, 1e3 * (time() - t0), nbytes)
        flush(stdout)
        close(sock)
    end
end

# ----------------------------------------------------------------- server --

"A running panel: its listeners, their accept loops, and the port they share."
struct PanelServer
    listeners::Vector{Sockets.TCPServer}
    acceptors::Vector{Task}
    port::Int
end

"""
    start_panel(port) -> PanelServer

Bind and start accepting. Returns immediately; the accept loops run as tasks.
"""
function start_panel(port::Int)
    START_TIME[] = time()
    listeners = Sockets.TCPServer[listen(IPv4(127, 0, 0, 1), port)]
    # `localhost` resolves to ::1 before 127.0.0.1 on Windows and on most
    # modern Linux, so an IPv4-only bind makes every new connection pay for a
    # failed attempt first. Best-effort: a host without IPv6 still works.
    try
        push!(listeners, listen(IPv6(0, 0, 0, 0, 0, 0, 0, 1), port))
    catch err
        @warn "IPv6 loopback unavailable; localhost falls back to IPv4" err
    end
    acceptors = Task[]
    for l in listeners
        push!(acceptors, @async begin
            while isopen(l)
                sock = try
                    accept(l)
                catch
                    break          # listener closed: the loop is done
                end
                # @async would schedule on this thread, and panel_mission
                # never yields — so a mission in flight stopped `accept` from
                # running at all. Six concurrent runs took 11.9 s, serialized.
                Threads.@spawn handle(sock)
            end
        end)
    end
    PanelServer(listeners, acceptors, port)
end

"Close every listener; in-flight requests finish on their own."
function stop_panel(s::PanelServer)
    foreach(close, s.listeners)
    nothing
end

"""
    main(port)

What `scripts/panel.jl` calls: warm the stack up so the first browser request
is not also the first compile, then serve until interrupted.
"""
function main(port::Int = 8137)
    println("warming up (first mission run compiles the stack)...")
    t0 = time()
    panel_mission(Dict{String,String}())
    @printf("ready in %.1f s — panel at http://localhost:%d  (Ctrl-C to stop)\n",
            time() - t0, port)
    srv = start_panel(port)
    wait(srv.acceptors[1])
    srv
end

end # module
