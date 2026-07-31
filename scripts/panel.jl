# Local mission-control panel: configure, run, and explore missions in the
# browser. Pure stdlib (raw Sockets HTTP) — no package dependencies.
#
#   julia --project -t auto scripts/panel.jl [port]
#
# then open http://localhost:8137 (default port). The page posts form-encoded
# parameters to /api/run and /api/sweep; the server executes the full mission
# design + flight (~1 s per run after warmup) and returns JSON with metrics,
# decimated trajectories, and events.
#
# All of the logic lives in `panelapp.jl` as a module, so the test suite can
# start a server without running this script.

include(joinpath(@__DIR__, "panelapp.jl"))

port = length(ARGS) >= 1 ? parse(Int, ARGS[1]) :
       parse(Int, get(ENV, "PORT", "8137"))
public = get(ENV, "HOST", "127.0.0.1") in ("0.0.0.0", "::")
PanelApp.main(port; public)
