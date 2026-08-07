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

# Refs, not consts, so a test can point one at a nonexistent file and exercise
# `route`'s catch without a real file on disk — the only way to reach that
# layer, since every shipped page always exists.
const PAGE_PATH = Ref(joinpath(@__DIR__, "panel_page.html"))
const LAUNCH_PATH = Ref(joinpath(@__DIR__, "launch_page.html"))
const BUILD_PATH = Ref(joinpath(@__DIR__, "build_page.html"))
const ANALYSIS_PATH = Ref(joinpath(@__DIR__, "analysis_page.html"))

"""
When a client last spoke to us, as `time()`. Zero means never.

The desktop launcher needs to know whether a window is actually there, and a
browser process's lifetime is not that: `msedge.exe` exits early for reasons
that have nothing to do with the window — it hands the URL to an Edge that is
already running, fails to create its profile directory, or is mid-update — and
treating that as "the user closed the window" tore the server down under a
window that was still opening. Traffic is the honest signal.
"""
const LAST_REQUEST = Ref(0.0)
const STATIC_DIR = Ref(joinpath(@__DIR__, "static"))

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
    # needs the bulk density to say how much that volume actually holds.
    # A named engine OWNS its mixture — a Raptor does not burn kerolox — so a
    # form that names both an engine and a different mixture is a mistake to
    # report, not to resolve silently (which is what this used to do, and the
    # form showed a propellant the server was not flying).
    pr = if eng != "manual"
        epr = lookup_engine(Symbol(eng)).prop
        typed = gets(p, pre * "propellant", "")
        if !isempty(typed) && Symbol(typed) != epr.name
            throw(ArgumentError("$eng burns $(epr.name); it cannot run on $typed"))
        end
        epr
    else
        propellant(Symbol(gets(p, pre * "propellant", ptype)))
    end
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

"""
What this stage would weigh dry if it were built the way `stage_mass` says
stages are built — tanks, insulation, engines, thrust structure, systems.

`sized_stage` can only do this for a stage that named a catalogue engine,
because it needs an engine to weigh. A hand-entered stage has thrust but no
engine, so its engines are sized from that thrust at an assumed
thrust-to-weight of 100. That number is an assumption and not a measurement:
the real spread is wide (Merlin 1D 183, Raptor 2 143, RL10B-2 37), because a
vacuum engine carries a large nozzle for thrust it does not have. It is
reported so a builder can see whether a hand-entered dry mass is anywhere
near buildable, which is worth a great deal more than the error it carries.
"""
function dry_estimate(st::Stage, vehicle_d::Float64)
    n = max(st.n_engines, 1)
    per = st.thrust_vac / n
    # A Stage does not record which engine built it, but its per-engine thrust,
    # Isp and mixture identify one almost uniquely — so a stack that DID name a
    # catalogue engine gets weighed with that engine's real mass and the
    # estimate agrees with the stage the server actually flies, instead of
    # reporting a nearby-but-different number and calling the true one wrong.
    hit = nothing
    for (_, e) in ENGINES
        if e.prop.name === st.prop.name &&
           abs(e.thrust_vac - per) <= 0.02 * max(per, 1.0) &&
           abs(e.isp_vac - st.isp_vac) <= 0.02 * max(st.isp_vac, 1.0)
            hit = e
            break
        end
    end
    eng = hit === nothing ?
          Engine(:estimate, st.prop, per, st.isp_vac, st.isp_vac,
                 st.ae / n, per / (G0 * 100.0), 0.4) : hit
    stage_mass(eng, n, st.mprop, stage_diameter(st, vehicle_d)).dry
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

"""
The capsule's diameter [m]: whatever the form states, or the mass fit when it
states nothing. Zero and blank both mean "work it out", so an untouched form
behaves exactly as it did before the field existed.
"""
pod_diameter(p, pod_mass) =
    (d = getf(p, "pod_dia", 0.0); d > 0 ? d : 2 * pod_radius(pod_mass))

"""
    stage_slug(vname) -> String

The stem a stage is named after, taken from the vehicle's own name.

Stage names are not internal: an ascent emits `sep_<stage name>`, and the
launch view renders that with the underscores turned into spaces — so
`sep_saturn_v1` reads "SATURN V1 SEPARATION". They were hardcoded to `sable`,
which is why flying a Saturn V announced "SABLE1 SEPARATION".

Anything parenthesised is dropped ("Sable (panel)" is the Sable), runs of
non-alphanumerics become the single underscore the label splits on, and a name
with nothing usable in it falls back to the reference vehicle's stem rather
than producing a nameless event.
"""
function stage_slug(vname::AbstractString)
    base = first(split(vname, '('))
    s = replace(lowercase(strip(base)), r"[^a-z0-9]+" => "_")
    s = strip(s, '_')
    # long enough for "saturn_v", short enough that the HUD's own abbreviator
    # still has room to cut on a word
    isempty(s) ? "sable" : first(s, 14)
end

"Build a LaunchVehicle from panel parameters."
function lv_from_params(p)
    dia = getf(p, "diameter", 1.8)
    nst = n_stages(p)
    slug = stage_slug(gets(p, "vname", "Sable (panel)"))
    stages = map(1:nst) do k
        d = STAGE_ROLE[stage_role(k, nst)]
        f = stage_scale(k, nst)
        nm = k == nst ? Symbol(slug, :k) : Symbol(slug, k)
        stage_from_params(p, "s$(k)_", nm, dia; dry = d.dry * f, prop = d.prop * f,
                          thrust_kn = d.thrust * f, isp = d.isp, ae = d.ae * f,
                          ptype = d.ptype, nedef = d.ne)
    end
    # Whether the stack carries a payload shroud at all. Off, the spacecraft
    # flies in the open the way Apollo, Dragon and Starship do — which is three
    # separate consequences and not one: no mass to carry or drop, no ogive on
    # the nose (so a blunt capsule sets the wave drag), and if the capsule is
    # wider than anything under it, it is what the flow sees.
    fair = getb(p, "fairing_on", true)
    pod = getf(p, "pod_mass", 350.0)
    # A stated capsule diameter wins over the mass fit, here as well as in the
    # mesh — the number the flow sees and the number that is drawn have to be
    # the same number, or a bare Orion flies the drag of a capsule two metres
    # narrower than the one on the screen.
    pod_d = pod_diameter(p, pod)
    sref = stack_sref((stage_diameter(s, dia) for s in stages), pod_d, fair)
    LaunchVehicle(
        # the page sends the name of whatever preset is loaded, so the livery
        # and the launch view say what you are actually flying
        name = gets(p, "vname", "Sable (panel)"),
        stages = stages,
        fairing_mass = fair ? getf(p, "fairing", 150.0) : 0.0,
        payload_mass = pod,
        # drag acts on the widest cross-section in the stack; strap-ons add
        # their own frontal area on top, but only while they are attached
        sref = sref,
        cd = fair ? SatelliteSim.LV_CD_TABLE : bare_payload_cd(pod_d, sref),
        boosters = boosters_from_params(p, dia),
    )
end

"Should the ascent tuner also search for the best pitch-over kick?"
opt_kick(p) = gets(p, "opt_kick", "0") in ("1", "true", "on")

"Pitch-over kick angle [rad] — the one guidance number a big stack has to change."
kick_rad(p) = deg2rad_(clamp(getf(p, "kick_deg", 8.0), 0.5, 30.0))

"""
Which mission the panel is flying: a suborbital hop or shot, an Earth orbit, the
free-return flyby, or a lunar landing.
"""
mission_mode(p) = (m = gets(p, "mode", "flyby");
                   m == "landing" ? :landing : m == "orbit" ? :orbit :
                   m == "suborbital" ? :suborbital : :flyby)

"Build the lander from the panel's `l_*` fields."
lander_from_params(p) = Lander(
    name = :lander,
    mdry = max(100.0, getf(p, "l_dry", 3500.0)),
    mprop = max(10.0, getf(p, "l_prop", 9000.0)),
    thrust = max(1.0e3, getf(p, "l_thrust_kn", 45.0) * 1e3),
    isp = clamp(getf(p, "l_isp", 311.0), 100.0, 500.0),
    throttle_min = clamp(getf(p, "l_throttle_min", 10.0) / 100, 0.02, 1.0),
    diameter = max(0.5, getf(p, "l_diameter", 4.2)))

"Build the launch vehicle that physically carries this lander."
function landing_vehicle_from_params(p, lander::Lander = lander_from_params(p))
    lv0 = lv_from_params(p)
    dia = getf(p, "diameter", 1.8)
    fair = lv0.fairing_mass > 0.0
    sref = stack_sref((stage_diameter(s, dia) for s in lv0.stages),
                      lander.diameter, fair)
    LaunchVehicle(name = lv0.name, stages = lv0.stages,
                  fairing_mass = lv0.fairing_mass,
                  payload_mass = lander_mass(lander), sref = sref,
                  cd = fair ? SatelliteSim.LV_CD_TABLE :
                       bare_payload_cd(lander.diameter, sref),
                  boosters = lv0.boosters)
end

"Decimate a vector to at most n points (keeping ends)."
function deci(v, n)
    length(v) <= n && return collect(Float64, v)
    idx = unique(round.(Int, range(1, length(v); length = n)))
    Float64[v[i] for i in idx]
end
deci_idx(len, n) = len <= n ? collect(1:len) :
                   unique(round.(Int, range(1, len; length = n)))

"""
    flyby_idx(L, n; near) -> indices

Wire indices for a cislunar track, keeping the flyby at full log resolution.

A uniform decimation spends its budget evenly over a coast that is mostly a
straight line, and the one part that is not — the hyperbolic swing past the
Moon — is where every point counts. Measured on a 500 m grazing free return:
a uniform 1600 points leaves 104 s between samples at closest approach, which
is 250 km of arc, and the straight line a viewer draws between two of them
passes 1200 m BELOW the surface. The trajectory was right to within three
metres and the picture had the vehicle inside the Moon.

The chord error is `v^2 dt^2 / 8r`, so it is the STEP that has to be bounded,
not the point count: at 2.4 km/s past a 1737 km body, 104 s of it sags 4.5 km
and the integrator's own 13 s sags 70 m. Keeping what the propagator already
chose to log near the Moon is therefore exactly the right resolution — it
tightened its step there for the same reason.

Two thirds of the wire is the most the flyby may take. The coast still has to
be drawn: a track that is all encounter and no route is not a trajectory.
"""
function flyby_idx(L, n::Int; near::Float64 = 2.0e7)
    m = length(L.t)
    m <= n && return collect(1:m)
    nearidx = [i for i in 1:m if L.d_moon[i] < near]
    isempty(nearidx) && return deci_idx(m, n)
    faridx = [i for i in 1:m if L.d_moon[i] >= near]
    bnear = min(length(nearidx), max(1, (2n) ÷ 3))
    bfar = max(2, n - bnear)
    sel = vcat(nearidx[deci_idx(length(nearidx), bnear)],
               isempty(faridx) ? Int[] : faridx[deci_idx(length(faridx), bfar)])
    sort!(unique(sel))
end

"""
Shared 3D-scene payload: the pad-to-wherever track in true ECI geometry,
units of 1000 km, decimated for the wire. Both missions fly the same launch
and trans-lunar legs, so both scenes are built from the same code.
"""
function scene_payload(asc, cis)
    # a mission whose ascent failed has no cislunar leg at all; the ascent
    # still flew and is still worth the wire
    cisd = nothing
    if cis !== nothing
        L = cis.log
        k = max(2, length(L.t) ÷ 3)
        nrm = SatelliteSim.vunit(SatelliteSim.vcross((L.mx[1], L.my[1], L.mz[1]),
                                                     (L.mx[k], L.my[k], L.mz[k])))
        idx = flyby_idx(L, 1600)
        px = Float64[]; py = Float64[]; pz = Float64[]
        mx = Float64[]; my = Float64[]; mz = Float64[]
        tt = Float64[]; pp = Int[]
        for i in idx
            push!(px, L.rx[i] / 1e6); push!(py, L.ry[i] / 1e6); push!(pz, L.rz[i] / 1e6)
            push!(mx, L.mx[i] / 1e6); push!(my, L.my[i] / 1e6); push!(mz, L.mz[i] / 1e6)
            push!(tt, L.t[i]); push!(pp, L.phase[i])
        end
        cisd = Dict("t" => tt, "x" => px, "y" => py, "z" => pz,
                    "mx" => mx, "my" => my, "mz" => mz, "ph" => pp,
                    "n" => [nrm[1], nrm[2], nrm[3]])
    end
    AL = asc.log
    aidx = deci_idx(length(AL.t), 400)
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
    lv = landing_vehicle_from_params(p, lander)
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

# the entry leg's payload blocks, shared by every mission that ends in one
function entry_payload!(out, metrics, events, ent)
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
    ei = findfirst(e -> e.name == :entry_interface, ent.events)
    metrics["ei_v_ms"] = ei === nothing ? NaN : ent.events[ei].vrel
    metrics["peak_g"] = ent.peak_gload
    metrics["peak_q_wcm2"] = ent.peak_qdot / 1e4
    metrics["heat_mj"] = ent.heat_load / 1e6
    metrics["splash_lat"] = rad2deg_(ent.lat_splash)
    metrics["splash_lon"] = rad2deg_(ent.lon_splash)
    metrics["v_splash"] = ent.v_splash
    for e in ent.events
        push!(events, Dict("phase" => "entry", "name" => string(e.name), "t" => e.t))
    end
    out["ent3d"] = Dict("t" => et3, "x" => ex3, "y" => ey3, "z" => ez3)
    out["entry"] = Dict("t" => deci(EL.t[eidx] .- EL.t[1], 500),
                        "h" => deci(EL.h[eidx] ./ 1e3, 500),
                        "v" => deci(EL.vrel[eidx], 500),
                        "g" => deci(EL.gload[eidx], 500),
                        "q" => deci((EL.qdot_conv[eidx] .+ EL.qdot_rad[eidx]) ./ 1e4, 500))
    out["sites"]["splash_lat"] = rad2deg_(ent.lat_splash)
    out["sites"]["splash_lon"] = rad2deg_(ent.lon_splash)
    nothing
end

"""
Fly an Earth-orbit mission for the panel: ascent, transfer burns on the kick
stage, `n_orbits` of the achieved orbit, and optionally a deorbit + entry.
The trajectory is served in the same `cis` shape the lunar missions use —
the Moon rides along as scenery — so both viewers fly it unchanged.
"""
function panel_orbit(p)::Dict{String,Any}
    tkey = Symbol(gets(p, "orbit", "leo"))
    (haskey(ORBITS, tkey) || tkey === :custom) || (tkey = :leo)
    eo = earthorbit(
        target = tkey,
        lv = lv_from_params(p),
        pod_mass = getf(p, "pod_mass", 350.0),
        h_park = getf(p, "h_park_km", 200.0) * 1e3,
        perigee_alt = tkey === :custom ?
            clamp(getf(p, "orbit_perigee_km", 200.0), 100.0, 100000.0) * 1e3 : NaN,
        apogee_alt = tkey === :custom ?
            clamp(getf(p, "orbit_apogee_km", 200.0), 100.0, 100000.0) * 1e3 : NaN,
        inclination = tkey === :custom ?
            deg2rad_(clamp(getf(p, "orbit_incl_deg", 28.5), 0.0, 180.0)) : NaN,
        n_orbits = clamp(getf(p, "n_orbits", 2.0), 0.25, 16.0),
        deorbit = getb(p, "deorbit", false),
        hp_entry = getf(p, "hp_entry_km", 25.0) * 1e3,
        kick_angle = kick_rad(p),
        optimize_kick = opt_kick(p),
        strict = false,
    )
    asc, ent = eo.ascent, eo.entry
    el = asc.elements
    haslog = length(eo.log.t) > 2
    sc = scene_payload(asc, haslog ? (log = eo.log,) : nothing)

    events = ascent_events(asc)
    for b in eo.burns
        push!(events, Dict("phase" => "orbit", "name" => "$(b.name)_ignition",
                           "t" => b.t_ign))
        push!(events, Dict("phase" => "orbit", "name" => "$(b.name)_cutoff",
                           "t" => b.t_ign + b.duration))
    end
    ent !== nothing &&
        push!(events, Dict("phase" => "orbit", "name" => "entry_handoff",
                           "t" => eo.entry_scn.t0))

    metrics = Dict{String,Any}(
        "liftoff_t" => liftoff_mass(eo.lv) / 1e3,
        "t_days" => (ent !== nothing ? ent.t_splash : eo.t) / 86400,
        "on_target" => eo.on_target,
        "prop_margin_kg" => eo.m - (eo.lv.stages[end].mdry + eo.lv.payload_mass),
        "orbit_rp_km" => (eo.elements.rp - RE_MEAN) / 1e3,
        "orbit_ra_km" => (eo.elements.ra - RE_MEAN) / 1e3,
        "orbit_incl_deg" => rad2deg_(eo.elements.i),
        "period_min" => eo.elements.a > 0 ?
            2pi * sqrt(eo.elements.a^3 / MU_EARTH) / 60 : NaN,
        "burn_dv_total" => sum(b.dv for b in eo.burns; init = 0.0),
    )
    if asc.reached_orbit
        metrics["park_perigee_km"] = (el.rp - RE_MEAN) / 1e3
        metrics["park_apogee_km"] = (el.ra - RE_MEAN) / 1e3
        metrics["incl_deg"] = rad2deg_(el.i)
    end
    out = Dict{String,Any}(
        "ok" => true, "mode" => "orbit",
        "outcome" => eo.outcome in (:on_orbit, :splashdown) ? "nominal" :
                     string(eo.outcome),
        "metrics" => metrics,
        "asc3d" => sc.asc3d,
        "ascent" => sc.ascent,
        "events" => events,
        "sites" => Dict{String,Any}(
            "launch_lat" => rad2deg_(eo.guid.site_lat),
            "launch_lon" => rad2deg_(eo.guid.site_lon),
        ),
        "orbit" => Dict{String,Any}(
            "target" => String(eo.target.name),
            "target_rp_km" => eo.target.perigee_alt / 1e3,
            "target_ra_km" => eo.target.apogee_alt / 1e3,
            "target_incl_deg" => rad2deg_(eo.target.inclination),
            "burns" => [Dict{String,Any}(
                "name" => String(b.name), "t_ign" => b.t_ign,
                "duration_s" => b.duration, "dv_plan" => b.dv_plan,
                "dv" => b.dv) for b in eo.burns],
        ),
    )
    haslog && (out["cis"] = sc.cis)
    ent !== nothing && entry_payload!(out, metrics, events, ent)
    out
end

"""
Fly a suborbital mission for the panel: a hop that closes on an apogee, or a
ballistic shot that closes on a ground range. There is no cislunar leg — the
whole flight is an ascent and an arc — so the payload carries `asc3d`, `ascent`
and the entry blocks and simply omits `cis`, which every consumer already
handles (an ascent that fails to reach orbit produces the same shape).
"""
function panel_suborbital(p)::Dict{String,Any}
    prof = gets(p, "sub_profile", "hop") == "downrange" ? :downrange : :hop
    sb = suborbital(
        profile = prof,
        lv = lv_from_params(p),
        pod_mass = getf(p, "pod_mass", 350.0),
        apogee = clamp(getf(p, "sub_apogee_km", 100.0), 5.0, 3000.0) * 1e3,
        downrange = clamp(getf(p, "sub_range_km", 400.0), 10.0, 12000.0) * 1e3,
        loft = deg2rad_(clamp(getf(p, "sub_loft_deg", 40.0), 5.0, 85.0)),
        azimuth = deg2rad_(clamp(getf(p, "sub_azimuth_deg", 90.0), 0.0, 360.0)),
        kick_angle = kick_rad(p),
        strict = false,
    )
    asc, ent = sb.ascent, sb.entry
    sc = scene_payload(asc, nothing)
    events = ascent_events(asc)

    metrics = Dict{String,Any}(
        "liftoff_t" => liftoff_mass(sb.lv) / 1e3,
        "apogee_km" => sb.apogee / 1e3,
        "target_apogee_km" => sb.target_apogee / 1e3,
        "range_km" => sb.range / 1e3,
        "target_range_km" => sb.target_range / 1e3,
        "t_apogee_s" => sb.t_apogee - asc.t,
        "cutoff_h_km" => asc.h_cut / 1e3,
        "cutoff_gamma_deg" => rad2deg_(asc.gamma_cut),
        "prop_margin_kg" => sum(asc.prop_left),
        # what it was asked for against what it did, as one number each way
        "apogee_err_km" => (sb.apogee - sb.target_apogee) / 1e3,
        "range_err_km" => isnan(sb.target_range) ? NaN :
                          (sb.range - sb.target_range) / 1e3,
        # The designer stops at 0.4% of the commanded quantity. Report the
        # same contract to the page instead of falling through to its flyby
        # off-target message merely because suborbital has no perilune.
        "on_target" => sb.outcome === :splashdown &&
                       abs((prof === :hop ? sb.apogee - sb.target_apogee :
                                             sb.range - sb.target_range)) <
                       0.004 * (prof === :hop ? sb.target_apogee : sb.target_range),
    )
    metrics["t_days"] = (ent !== nothing ? ent.t_splash : asc.t) / 86400

    out = Dict{String,Any}(
        "ok" => true, "mode" => "suborbital",
        "outcome" => sb.outcome === :splashdown ? "nominal" : string(sb.outcome),
        "metrics" => metrics,
        "asc3d" => sc.asc3d,
        "ascent" => sc.ascent,
        "events" => events,
        "sites" => Dict{String,Any}(
            "launch_lat" => rad2deg_(sb.guid.site_lat),
            "launch_lon" => rad2deg_(sb.guid.site_lon),
        ),
        "suborbital" => Dict{String,Any}(
            "profile" => String(sb.profile),
            "apogee_km" => sb.apogee / 1e3,
            "range_km" => sb.range / 1e3,
        ),
    )
    if ent !== nothing
        entry_payload!(out, metrics, events, ent)
        # The arc starts below the entry interface and going UP, so the entry
        # simulator never crosses 120 km downward from outside and never emits
        # the event the viewers frame their ENTRY phase on. It is still a real
        # instant — it is where the capsule comes back through 120 km — so it is
        # found in the log rather than left missing. A hop that never gets that
        # high hands over at apogee instead, which is the same idea: the point
        # after which the only thing left is coming down.
        EL = ent.log
        top = argmax(EL.h)
        ei = findfirst(i -> i > top && EL.h[i] <= 120.0e3, eachindex(EL.h))
        if !any(e -> e["name"] == "entry_interface", events)
            push!(events, Dict("phase" => "entry", "name" => "entry_interface",
                               "t" => EL.t[ei === nothing ? top : ei]))
        end
        push!(events, Dict("phase" => "entry", "name" => "apogee", "t" => EL.t[top]))
    end
    out
end

function panel_mission(p)::Dict{String,Any}
    mission_mode(p) === :landing && return panel_landing(p)
    mission_mode(p) === :orbit && return panel_orbit(p)
    mission_mode(p) === :suborbital && return panel_suborbital(p)
    ms = moonshot(
        pod_mass = getf(p, "pod_mass", 350.0),
        # a stated capsule width is the width in the flow, the width on the
        # screen AND the width at entry — the same number in all three, which
        # is what the builder promises. Omitted, moonshot takes the mass fit.
        pod_diameter = pod_diameter(p, getf(p, "pod_mass", 350.0)),
        h_park = getf(p, "h_park_km", 200.0) * 1e3,

        hp_moon = getf(p, "hp_moon_km", 2000.0) * 1e3,
        hp_return = getf(p, "hp_return_km", 50.0) * 1e3,
        inclination = deg2rad_(getf(p, "incl_deg", 28.5)),
        lv = lv_from_params(p),
        tli_mag_err = getf(p, "tli_mag_err_pct", 0.0) / 100,
        tli_point_err = deg2rad_(getf(p, "tli_point_err_deg", 0.0)),
        kick_angle = kick_rad(p),
        optimize_kick = opt_kick(p),
        strict = false,
    )
    asc, cis, ent = ms.ascent, ms.cislunar, ms.entry
    el = asc.elements

    # 3D scene payload: true ECI geometry in units of 1000 km. The client
    # renders the inclined trajectory plane, the textured globe about the real
    # pole (scene z = ECI z), and launch/splashdown markers fixed to the
    # rotating surface — all in one consistent frame.
    #
    # A mission that ended early is a RESULT, not an error: every leg that was
    # simulated is served, and "outcome" says where the flight stopped
    # ("nominal", "ascent_failed", or the cislunar outcome such as "timeout").
    # The browser flies whatever is here.
    sc = scene_payload(asc, cis)

    events = ascent_events(asc)
    metrics = Dict{String,Any}(
        "liftoff_t" => liftoff_mass(ms.lv) / 1e3,
        "t_days" => (ent !== nothing ? ent.t_splash :
                     cis !== nothing ? cis.t : asc.t) / 86400,
        # what the free-return corrector made of the problem, so the page can
        # say WHY a mission stopped rather than only that it did
        "design_status" => string(ms.design_status),
    )
    out = Dict{String,Any}(
        "ok" => true, "mode" => "flyby",
        # An entry that RAN is not an entry that ARRIVED. `terminated` is
        # :splashdown or :timeout, and a capsule whose ballistic coefficient
        # skips it back out of the atmosphere produces a full 8-hour entry log
        # with NaN splash fields — which this reported as "nominal" while every
        # splashdown metric serialised to null. The text reports have always
        # read this field (`DID NOT SPLASH DOWN` in mission.jl and
        # lunarreturn.jl); the web path was the one that did not. Orbit and
        # suborbital above already test their own outcome the same way.
        "outcome" => ent !== nothing ?
                       (ent.terminated === :splashdown ? "nominal" :
                        "entry_" * string(ent.terminated)) :
                     # a design the corrector never closed outranks the leg
                     # outcome: with strict=false moonshot stops before the
                     # entry, and "entry_interface" as an outcome would read
                     # like a flight that simply ended early rather than a
                     # trajectory that was never the requested one
                     ms.design_status in (:stalled, :unreachable) ? "design_failed" :
                     cis !== nothing ? string(cis.outcome) : "ascent_failed",
        "metrics" => metrics,
        "asc3d" => sc.asc3d,
        "ascent" => sc.ascent,
        "events" => events,
        "sites" => Dict{String,Any}(
            "launch_lat" => rad2deg_(ms.guid.site_lat),
            "launch_lon" => rad2deg_(ms.guid.site_lon),
        ),
    )
    if asc.reached_orbit
        metrics["park_perigee_km"] = (el.rp - RE_MEAN) / 1e3
        metrics["park_apogee_km"] = (el.ra - RE_MEAN) / 1e3
        metrics["incl_deg"] = rad2deg_(el.i)
    end

    if cis !== nothing
        out["cis"] = sc.cis
        metrics["tli_dv"] = cis.dv_tli
        metrics["tli_burn_s"] = cis.burn_duration
        metrics["prop_margin_kg"] = cis.m - (ms.lv.stages[end].mdry + ms.lv.payload_mass)
        metrics["perilune_km"] = cis.perilune_alt / 1e3
        metrics["t_perilune_d"] = cis.t_perilune / 86400
        metrics["vac_perigee_km"] = cis.vac_perigee_alt / 1e3
        metrics["tcm_dv"] = ms.cruise === nothing ? nothing : ms.cruise.tcm_dv
        metrics["tcm_prop_kg"] = ms.cruise === nothing ? nothing : ms.cruise.tcm_prop
        metrics["rcs_margin_kg"] = ms.cruise === nothing ? nothing : ms.cruise.rcs.margin
        # did the free-return design actually hit its targets? (a prop-starved
        # TLI still "flies", but the result is not the requested mission)
        hp_moon = getf(p, "hp_moon_km", 2000.0) * 1e3
        hp_ret = getf(p, "hp_return_km", 50.0) * 1e3
        # The perilune band scales with the target and its floor is 300 m, not
        # 50 km: a flat 50 km band passes anything from the surface to a
        # hundred times a 500 m target, which is not a check, it is a rubber
        # stamp. 8% is comfortably outside the designer's own 2% acceptance, so
        # a converged design is never reported off target by rounding.
        # A grazing flyby is a DIFFERENT MISSION and is judged as one. Below
        # about 10 km no free return comes home — the Moon turns the trajectory
        # so hard that the return leg's perigee is underground — so the designer
        # stops shooting at it and flies the flyby instead. Scoring that against
        # a return corridor it deliberately gave up would report every grazing
        # pass as a failure. Everything else is scored on both ends as before.
        graze = hp_moon < 10e3
        peri_ok = abs(cis.perilune_alt - hp_moon) <= max(0.08 * hp_moon, 300.0)
        metrics["on_target"] = graze ? peri_ok :
            (ent !== nothing && peri_ok &&
             abs(cis.vac_perigee_alt - hp_ret) <= 20e3)
        # `grazing` also says the altitude is above the MEAN SPHERE, which below
        # 10 km stops being the same question as "above the ground": lunar
        # relief runs to roughly +/- 8 km, so this is a clearance against a
        # smooth Moon and the real one has mountains in it.
        metrics["grazing"] = graze
        push!(events, Dict("phase" => "cislunar", "name" => "tli_ignition", "t" => cis.t_tli))
        push!(events, Dict("phase" => "cislunar", "name" => "tli_cutoff",
                           "t" => cis.t_tli + cis.burn_duration))
        isfinite(cis.t_perilune) &&
            push!(events, Dict("phase" => "cislunar", "name" => "perilune",
                               "t" => cis.t_perilune))
        ent !== nothing &&
            push!(events, Dict("phase" => "cislunar", "name" => "entry_handoff",
                               "t" => cis.t))
    else
        metrics["on_target"] = false
    end

    ent !== nothing && entry_payload!(out, metrics, events, ent)
    out
end

"""
Metrics a solve can target. All are scalars from a completed mission, so
each evaluation is a full design-and-fly of the chain.

`perilune_km` and `vac_perigee_km` are deliberately NOT here. They are
COMMANDED, not achieved: the free-return corrector drives both to whatever
the mission targets ask for, so they read the same to four figures across the
whole feasible range of any vehicle parameter and then cliff when the design
stops closing. Solving against one spent the entire iteration budget crawling
along a flat function and returned the cliff edge as if it were a root. The
number to move is the target itself.
"""
const SOLVE_METRICS = ["prop_margin_kg",
                       "peak_g", "peak_q_wcm2", "t_days", "liftoff_t",
                       "park_apogee_km", "v_splash", "tli_dv", "heat_mj"]

"""
Metrics the mission DESIGN drives to a commanded value, mapped to the field
that commands them. Solving a vehicle parameter against one of these is not a
root-find, it is a misunderstanding — so it is refused by name rather than
answered with a meaningless number.
"""
const COMMANDED_METRICS = Dict(
    "perilune_km"    => "perilune [km]",
    "vac_perigee_km" => "return perigee [km]",
)

"""
Metrics a SWEEP may chart. A sweep only plots a metric against a parameter —
there is no root to find — so a commanded one belongs here even though it
cannot be solved for: the flat line and the cliff at its end are exactly the
picture of where this vehicle stops being able to fly the mission.
"""
const SWEEP_METRICS = vcat(SOLVE_METRICS, collect(keys(COMMANDED_METRICS)))

"""
Metrics a suborbital flight can be solved against. Deliberately its own list:
half of SOLVE_METRICS is about an orbit that a suborbital flight never has, and
offering `park_apogee_km` on a hop would only ever return NaN.
"""
const SUBORBITAL_METRICS = ["apogee_km", "range_km", "peak_g", "peak_q_wcm2",
                            "v_splash", "heat_mj", "prop_margin_kg",
                            "liftoff_t", "cutoff_h_km", "t_apogee_s"]

"Metrics a landing mission can be solved against."
const LANDING_METRICS = ["prop_left_kg", "hover_s", "descent_dv", "loi_dv",
                         "touchdown_v", "touchdown_vh", "downrange_km",
                         "prop_margin_kg", "min_throttle_pct", "liftoff_t",
                         "tli_dv", "t_days", "ground_slope_deg",
                         "ground_elev_m", "redesignate_m", "nav_alt_err_m"]

"The metric list for whichever mission the panel is configured for."
solve_metrics(p) = mission_mode(p) === :landing ? LANDING_METRICS :
                   mission_mode(p) === :suborbital ? SUBORBITAL_METRICS :
                   SOLVE_METRICS

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
    haskey(COMMANDED_METRICS, metric) && return Dict{String,Any}("ok" => false,
        "error" => "$metric is commanded, not achieved — the designer already " *
                   "drives it to the target, so it does not respond to $param. " *
                   "Set \"$(COMMANDED_METRICS[metric])\" directly instead.")
    metric in solve_metrics(p) || return Dict{String,Any}("ok" => false,
        "error" => "cannot target metric: $metric")
    lo = getf(p, "solve_min", 200.0)
    hi = getf(p, "solve_max", 600.0)
    target = getf(p, "solve_target", 0.0)
    budget = clamp(round(Int, getf(p, "solve_iters", 14.0)), 4, 30)
    # A metric already on target to within a part in ten thousand IS solved.
    # With ftol = 0 nothing ever counts as hit, so a search that starts on the
    # answer still burns its whole budget and reports :no_bracket.
    ftol = 1e-4 * max(abs(target), 1.0)
    res = find_root(lo, hi; target = target, max_iter = budget, ftol = ftol) do x
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
    lander = mission_mode(p) === :landing ? lander_from_params(p) : nothing
    lv = lander === nothing ? lv_from_params(p) : landing_vehicle_from_params(p, lander)
    mesh, secs = rocket_mesh(lv; diameter = d, nseg = 36,
                             pod_diameter = getf(p, "pod_dia", 0.0),
                             crewed = getb(p, "crewed", true),
                             payload_kind = lander === nothing ? :capsule : :lander,
                             payload_diameter = lander === nothing ? 0.0 : lander.diameter)
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
        # Keep the cinematic title and hull livery tied to the same vehicle
        # the builder and simulator are actually flying.
        "name" => String(lv.name),
        # boosters are appended after the core stack, so the tallest section
        # is not necessarily the last one
        "length" => maximum(s.x1 for s in secs),
        "diameter" => d,
        "payload" => Dict("kind" => lander === nothing ?
                              (getb(p, "crewed", true) ? "capsule" : "bus") : "lander",
                          "mass_kg" => lv.payload_mass,
                          "diameter_m" => lander === nothing ?
                              pod_diameter(p, lv.payload_mass) : lander.diameter),
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
                          # what a stage this size would actually weigh, and
                          # the structural coefficient that follows from what
                          # was entered — the two numbers that say whether a
                          # configuration is a vehicle or a wish
                          "dry_est_kg" => dry_estimate(s, d),
                          "dry_frac" => s.mdry / (s.mdry + s.mprop),
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
    mission_mode(p) === :suborbital ?
        ["sub_apogee_km", "sub_range_km", "sub_loft_deg", "sub_azimuth_deg"] :
        String[],
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

# -------------------------------------------------------------- run store --

"""
The missions this server has actually flown, newest first.

Three pages want to look at the same trajectory, and before this each of them
re-flew it from a query string. That was expensive — seconds of simulation to
show a flight that had just been shown — and worse, it was not reliably the
SAME flight: the query string and the run it produced were only ever as
identical as the form's serialisation, so "open the launch view on this run"
quietly meant "fly something close to this run again".

A run gets an id when it is flown, and the pages pass the id around. That
makes "show me this" a lookup instead of a simulation, and it makes history
mean something: these are flights that happened, not recipes for flights that
might.

In memory, and deliberately: this is the session's history, the session is one
window, and a trajectory is far too big to be worth writing to disk to survive
a restart that also throws away everything else on screen.
"""
const MAX_RUNS = 12
const RUNS = Dict{String,Dict{String,Any}}()
const RUN_ORDER = String[]            # ids, newest first
const RUN_SEQ = Ref(0)
# Runs arrive from `Threads.@spawn handle(sock)`, so two windows or a double
# click can be here at once.
const RUNS_LOCK = ReentrantLock()

"""
    remember_run!(payload, p) -> payload

Give a successful run an id and file it, evicting the oldest beyond `MAX_RUNS`.

The parameters that produced it are stored alongside, which is what lets a
history entry put the form back the way it was rather than only replaying a
result.
"""
function remember_run!(payload::Dict{String,Any}, p::AbstractDict)
    lock(RUNS_LOCK) do
        RUN_SEQ[] += 1
        id = "r$(RUN_SEQ[])"
        payload["id"] = id
        payload["at"] = time()
        payload["params"] = Dict{String,String}(string(k) => string(v) for (k, v) in p)
        RUNS[id] = payload
        pushfirst!(RUN_ORDER, id)
        while length(RUN_ORDER) > MAX_RUNS
            delete!(RUNS, pop!(RUN_ORDER))
        end
        payload
    end
end

"""
    run_and_remember(p) -> Dict

Fly a mission and keep it. Only `/api/run` goes through here — a sweep flies
dozens of missions internally and none of them are runs the user asked for.
"""
function run_and_remember(p)
    out = panel_mission(p)
    get(out, "ok", false) === true ? remember_run!(out, p) : out
end

"""
One history entry: enough to label it, and enough to restore the form from it,
without shipping a whole trajectory per row.
"""
function run_digest(payload::Dict{String,Any})
    p = get(payload, "params", Dict{String,String}())
    Dict{String,Any}(
        "id" => get(payload, "id", ""),
        "at" => get(payload, "at", 0.0),
        "mode" => get(p, "mode", "flyby"),
        "vname" => get(p, "vname", "vehicle"),
        "outcome" => get(payload, "outcome", "?"),
        # flat, a couple of dozen numbers, and the client already knows how to
        # format them — so a chip can say what the run WAS, not just when it ran
        "metrics" => get(payload, "metrics", Dict{String,Any}()),
        "params" => p)
end

"The history list, newest first."
runs_payload() = lock(RUNS_LOCK) do
    Dict{String,Any}("ok" => true,
                     "runs" => [run_digest(RUNS[id]) for id in RUN_ORDER if haskey(RUNS, id)])
end

"""
    stored_run(path) -> (status, content_type, payload)

`GET /api/runs/{id}`. A miss is a 404 that says why it might be a miss —
history is capped and lives in memory, so an id CAN legitimately stop
resolving, and "not found" alone would read as a bug.
"""
function stored_run(path::AbstractString)
    id = first(split(path[length("/api/runs/") + 1:end], '?'))
    payload = lock(RUNS_LOCK) do
        get(RUNS, id, nothing)
    end
    payload === nothing && return ("404 Not Found", "application/json",
        json(Dict{String,Any}("ok" => false,
            "error" => "no run \"$id\" — history keeps the last $MAX_RUNS " *
                       "runs of this session, and starts empty each time the " *
                       "app opens")))
    ("200 OK", "application/json", json(payload))
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
    "orbits" => [Dict("name" => String(v.name),
                      "perigee_km" => v.perigee_alt / 1e3,
                      "apogee_km" => v.apogee_alt / 1e3,
                      "incl_deg" => rad2deg_(v.inclination),
                      "note" => v.note)
                 for (k, v) in sort(collect(ORBITS), by = first)],
    # What this BUILD understands. The pages are served fresh from disk on every
    # request and the module is not — it is compiled into the running process —
    # so a server left up across an edit serves a page with controls it has
    # never heard of. The symptom is silent and baffling: the fairing switch
    # appears, sends fairing_on=0, and the vehicle keeps its fairing, because
    # the code that reads that field is not in the process. The pages check this
    # list against the controls they offer and say so.
    "features" => ["fairing_on", "crewed", "pod_dia", "l_diameter",
                   "grazing", "flyby_wire"],
    "solve_metrics" => SOLVE_METRICS,
    "sweep_metrics" => SWEEP_METRICS,
    "landing_metrics" => LANDING_METRICS,
    "suborbital_metrics" => SUBORBITAL_METRICS,
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

"""
Write a complete response. `head` suppresses the body but keeps the headers.

A `204 No Content` gets neither `Content-Type` nor `Content-Length`: RFC 9112
§6.2 is a MUST NOT for the latter on a 204, and the former has nothing to
describe.
"""
function write_response(sock, status::AbstractString, ctype::AbstractString,
                        body::AbstractString; head::Bool = false)
    no_content = startswith(status, "204")
    write(sock, no_content ?
        "HTTP/1.1 $status\r\nConnection: close\r\n\r\n" :
        "HTTP/1.1 $status\r\nContent-Type: $ctype\r\n" *
        "Content-Length: $(sizeof(body))\r\nConnection: close\r\n\r\n")
    (head || no_content) || write(sock, body)
    nothing
end

"The `{ok: false, error: ...}` shape every failure response shares."
error_payload(err) = Dict{String,Any}("ok" => false, "error" => sprint(showerror, err))

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
        error_payload(err)
    end
end

"""
    static_asset(path) -> (status, content_type, payload)

Serve a shared ES module or stylesheet out of `scripts/static/`.

The requested name is matched against a strict whitelist rather than
sanitised: `[A-Za-z0-9_-]+.(js|css)`, which cannot express a directory
separator at all. Sanitising instead means enumerating every way a path can
escape its root — `..`, `%2e%2e`, backslashes on Windows, drive letters,
symlinks — and losing the moment you miss one. This process reads the user's
own filesystem, so the distinction matters even though the panel only ever
listens on loopback.

Read per request, like the pages, so editing an asset shows up on refresh.
"""
function static_asset(path::AbstractString)
    name = first(split(path[length("/static/") + 1:end], '?'))
    ok = occursin(r"^[A-Za-z0-9_-]+\.(js|css)$", name) &&
         isfile(joinpath(STATIC_DIR[], name))
    ok || return ("404 Not Found", "application/json",
                  json(Dict{String,Any}("ok" => false,
                                        "error" => "no such asset: $name")))
    # a module served as anything but a JS media type is refused by the
    # browser outright, and the console error names CORS rather than the type.
    # A stylesheet served as the wrong type is dropped just as silently.
    ctype = endswith(name, ".css") ? "text/css; charset=utf-8" :
                                     "text/javascript; charset=utf-8"
    return ("200 OK", ctype, read(joinpath(STATIC_DIR[], name), String))
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
            return ("200 OK", "text/html; charset=utf-8", read(PAGE_PATH[], String))
        elseif method in ("GET", "HEAD") &&
               (path == "/launch" || startswith(path, "/launch?"))
            return ("200 OK", "text/html; charset=utf-8", read(LAUNCH_PATH[], String))
        elseif method in ("GET", "HEAD") &&
               (path == "/build" || startswith(path, "/build?"))
            return ("200 OK", "text/html; charset=utf-8", read(BUILD_PATH[], String))
        elseif method in ("GET", "HEAD") &&
               (path == "/analysis" || startswith(path, "/analysis?"))
            # The plots, the event log, the sweep and the solver. Like /launch
            # it reads `?run=<id>` out of the store above; a full query string
            # still flies a mission, which is what a bare link and the page's
            # own selftest use. That shared state was once refused here on the
            # grounds that re-simulating was the honest trade — it was not.
            # Re-flying showed a DIFFERENT run than the one being looked at.
            return ("200 OK", "text/html; charset=utf-8", read(ANALYSIS_PATH[], String))
        elseif method in ("GET", "HEAD") && startswith(path, "/static/")
            return static_asset(path)
        elseif method in ("GET", "HEAD") && path == "/api/catalogue"
            return ("200 OK", "application/json", json(catalogue_payload()))
        elseif method in ("GET", "HEAD") && path == "/api/health"
            return ("200 OK", "application/json", json(health_payload()))
        elseif method in ("GET", "HEAD") &&
               (path == "/api/runs" || startswith(path, "/api/runs?"))
            return ("200 OK", "application/json", json(runs_payload()))
        elseif method in ("GET", "HEAD") && startswith(path, "/api/runs/")
            return stored_run(path)
        elseif method == "POST" && path == "/api/run"
            return ("200 OK", "application/json", json(safe_call(run_and_remember, body)))
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
                json(error_payload(err)))
    end
end

"""
Serve one connection, and answer it whatever happens.

Two layers, because one is not enough: `route` turns any error below it into a
response, and the `catch` here is the last resort for a failure in `route`'s
own serialization or in the socket write.
"""
function handle(sock)
    t0 = time()
    LAST_REQUEST[] = t0
    method, path, status, nbytes = "-", "-", "-", 0
    wrote = false
    try
        req = read_request(sock)
        req === nothing && return
        method, path, body = req
        st, ctype, payload = route(method, path, body)
        # once write_response is called, bytes may already be on the wire —
        # if it throws partway through, a second status line on top of a
        # partial first one would corrupt the stream, so `wrote` must flip
        # before the call, not after
        wrote = true
        write_response(sock, st, ctype, payload; head = method == "HEAD")
        status = first(st, 3)
        nbytes = sizeof(payload)
    catch err
        if wrote
            # the primary response may have partially reached the peer; the
            # only safe thing left to do is say so in the log, not the socket
            status = "ERR"
        else
            try
                payload = json(error_payload(err))
                nbytes = sizeof(payload)
                write_response(sock, "500 Internal Server Error", "application/json",
                               payload; head = method == "HEAD")
                status = "500"
            catch
                # the socket itself is gone; nothing left to say
                status = "ERR"; nbytes = 0
            end
        end
    finally
        # One line per request. This is how an intermittent "Failed to fetch"
        # gets diagnosed from a record rather than from a hypothesis. Wrapped
        # so a throwing @printf/flush cannot skip the close below and leak
        # the socket.
        try
            @printf("[panel] %-7s %-44s %s %7.0f ms %9d B\n",
                    method, first(path, 44), status, 1e3 * (time() - t0), nbytes)
            flush(stdout)
        catch
        end
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
function start_panel(port::Int; public::Bool = false)
    START_TIME[] = time()
    # Render and other container hosts route traffic to 0.0.0.0:$PORT. Local
    # development remains loopback-only unless the entry point explicitly
    # opts in, so starting the panel never exposes it to the LAN by accident.
    listeners = Sockets.TCPServer[listen(public ? IPv4(0, 0, 0, 0) :
                                                  IPv4(127, 0, 0, 1), port)]
    # `localhost` resolves to ::1 before 127.0.0.1 on Windows and on most
    # modern Linux, so an IPv4-only bind makes every new connection pay for a
    # failed attempt first. Best-effort: a host without IPv6 still works.
    if !public
        try
            push!(listeners, listen(IPv6(0, 0, 0, 0, 0, 0, 0, 1), port))
        catch err
            @warn "IPv6 loopback unavailable; localhost falls back to IPv4" err
        end
    end
    acceptors = Task[]
    for l in listeners
        push!(acceptors, @async begin
            fails = 0
            while isopen(l)
                sock = try
                    accept(l)
                catch err
                    isopen(l) || break     # listener closed: the loop is done
                    # accept can also fail transiently — ECONNABORTED when a
                    # client aborts between SYN and accept (routine on
                    # Windows), EMFILE under descriptor pressure. Treating
                    # every such error as "closed" silently kills this
                    # listener's loop; if that listener is the IPv6 one, every
                    # `localhost` request (which resolves ::1 first) starts
                    # failing with nothing in the request log, because the
                    # request never reaches `handle` at all.
                    fails += 1
                    if fails >= 5
                        @error "accept failed $fails times in a row; giving up on this listener" err
                        break
                    end
                    @warn "accept failed; still listening" err
                    continue
                end
                fails = 0
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
function main(port::Int = 8137; public::Bool = false)
    println("warming up (first mission run compiles the stack)...")
    t0 = time()
    panel_mission(Dict{String,String}())
    host = public ? "0.0.0.0" : "localhost"
    @printf("ready in %.1f s — panel at http://%s:%d  (Ctrl-C to stop)\n",
            time() - t0, host, port)
    Threads.nthreads() == 1 &&
        println("single-threaded: a mission in flight will block every other " *
                "request until it finishes. Restart with `-t auto` to serve " *
                "requests concurrently.")
    srv = start_panel(port; public)
    wait(srv.acceptors[1])
    srv
end

end # module
