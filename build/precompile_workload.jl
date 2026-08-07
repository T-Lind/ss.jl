# What the frozen app should already know how to do before it starts.
#
# `create_app` traces this run and freezes every method it compiles into the
# sysimage. The point is the first mission: from source, the panel prints
# "warming up (first mission run compiles the stack)" and takes seconds before
# it will answer a request. Everything exercised here is paid for at build
# time instead.
#
# Deliberately does NOT open a window or bind a port — this runs inside the
# compiler, and a server or a browser would be left behind in the build.

using SatelliteSim

const SCRIPTS = joinpath(dirname(@__DIR__), "scripts")
include(joinpath(SCRIPTS, "panelapp.jl"))

# One flyby through the whole stack: launch, ascent, free-return design,
# cislunar propagation, entry. This is the expensive path and the reason the
# shipped app opens instantly.
PanelApp.panel_mission(Dict{String,String}())

# The other two mission shapes take different branches through targeting and
# entry, so compiling only the flyby would leave a stall on the first orbit or
# landing run.
PanelApp.panel_mission(Dict("mode" => "orbit"))
PanelApp.panel_mission(Dict("mode" => "suborbital"))

# The geometry endpoint drives the mesh and staging code the builder needs on
# its very first keystroke.
PanelApp.rocket_geometry(Dict{String,String}())

# Serialisation and routing: cheap to compile but on the path of every single
# request, so there is no reason to leave them for run time.
PanelApp.route("GET", "/api/catalogue", "")
PanelApp.route("GET", "/api/health", "")
# The run store: every page load after the first goes through one of these,
# because /launch and /analysis fetch a flown trajectory by id rather than
# flying their own.
PanelApp.route("GET", "/api/runs", "")
PanelApp.route("GET", "/api/runs/r1", "")
PanelApp.route("GET", "/static/fmt.js", "")
PanelApp.route("GET", "/static/busy.js", "")
PanelApp.route("GET", "/static/runs.js", "")
PanelApp.route("GET", "/", "")
PanelApp.route("GET", "/build", "")
PanelApp.route("GET", "/launch", "")
PanelApp.route("GET", "/analysis", "")
PanelApp.route("GET", "/no/such/route", "")
