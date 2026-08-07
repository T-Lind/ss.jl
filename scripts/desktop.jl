# ss.jl as a desktop application.
#
#   julia --project -t auto scripts/desktop.jl
#
# Starts the panel server on a port nobody else is using, opens it in a
# chromeless Edge window, and exits when that window is closed. No tray, no
# background process left behind.
#
# WHY EDGE RATHER THAN A BUNDLED RUNTIME
#
# Edge and the WebView2 runtime ship with Windows 11, so "self-contained"
# costs zero megabytes here. Bundling Chromium (Electron) would add ~150 MB to
# ship a renderer the machine already has, and a Rust/wry host would add a
# toolchain to the build for the same WebView2 underneath. `--app=` gives a
# window with no tabs, no omnibox and no browser chrome — which is the part
# that makes it read as an application rather than a web page.
#
# It also buys the one thing a web page provably cannot have: control over
# which GPU renders. See GPU_FLAGS below.

include(joinpath(@__DIR__, "panelapp.jl"))

# PanelApp keeps these inside the module, so this file needs its own.
using Sockets
using Printf

const EDGE_CANDIDATES = [
    raw"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    raw"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]
const CHROME_CANDIDATES = [
    raw"C:\Program Files\Google\Chrome\Application\chrome.exe",
    raw"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
]

"First browser that exists, preferring Edge because it is the one Windows guarantees."
function find_browser()
    for p in vcat(EDGE_CANDIDATES, CHROME_CANDIDATES)
        isfile(p) && return p
    end
    nothing
end

"""
    free_port() -> Int

Ask the OS for an unused port by binding port 0, then let it go.

There is a race here — something else may take the port between the close and
the panel's own bind — and it is the right trade anyway. A hardcoded port is
not a race, it is a guaranteed failure the moment a second window opens or a
`scripts/panel.jl` is already running, which is a thing that happens
constantly during development.
"""
function free_port()
    s = Sockets.listen(Sockets.IPv4(127, 0, 0, 1), 0)
    p = Int(Sockets.getsockname(s)[2])
    close(s)
    p
end

# Chromium picks ONE GPU for its whole process at launch, and a page cannot
# move itself off it: `powerPreference: "high-performance"` on the WebGL
# context was measured on a machine with an Intel iGPU and an NVIDIA T550 and
# returned the byte-identical Intel renderer string. Passing the flag to the
# browser process is the lever that does work, and having one is a large part
# of why a desktop shell is worth building at all.
#
# The launch view is fragment-bound — its frame is almost entirely two noise
# shaders — so the discrete adapter is the one to ask for. It stays a request:
# a single-GPU desktop ignores it, and the page reports the adapter that
# actually answered (`__debug.gpu`), which is what to check before believing
# this had any effect.
const GPU_FLAGS = ["--force_high_performance_gpu"]

"""
    launch(; port, open_browser = true, gpu = true)

Serve, open a window, and return when the window closes.

The browser gets a `--user-data-dir` of its own under this process's temp
directory. That is not tidiness: without it, `msedge.exe` hands the URL to an
Edge that is ALREADY running and exits immediately, so the launcher sees its
child die at once and shuts the server down under a window that just opened.
A private profile forces a browser process this launcher actually owns, which
is what makes waiting on it mean "the window is open".
"""
function launch(; port::Int = free_port(), open_browser::Bool = true,
                  gpu::Bool = true, warm::Bool = true)
    if warm
        print("starting the simulator… ")
        t0 = time()
        PanelApp.panel_mission(Dict{String,String}())
        @printf("%.1f s\n", time() - t0)
    end
    srv = PanelApp.start_panel(port)
    url = "http://127.0.0.1:$port/"
    println("panel on $url")

    if !open_browser
        println("(no window requested — Ctrl-C to stop)")
        wait(srv.acceptors[1])
        return srv
    end

    exe = find_browser()
    if exe === nothing
        println("""
        No Edge or Chrome found, so there is no window to open. The server is
        running — point a browser at $url. Ctrl-C to stop.""")
        wait(srv.acceptors[1])
        return srv
    end

    profile = mktempdir(prefix = "ssjl-window-")
    args = String[
        "--app=$url",
        "--user-data-dir=$profile",
        "--window-size=1600,1000",
        # a first run that opens "welcome" tabs or an import prompt would put
        # browser chrome in front of an app window
        "--no-first-run", "--no-default-browser-check",
        "--disable-features=Translate,MediaRouter",
    ]
    gpu && append!(args, GPU_FLAGS)

    println("opening the window…")
    proc = run(Cmd([exe; args]), wait = false)
    try
        wait(proc)                       # returns when the window is closed
    catch err
        err isa InterruptException || rethrow()
    finally
        println("window closed — shutting down")
        PanelApp.stop_panel(srv)
        # best-effort: a profile directory is a few MB and the OS will clear
        # temp eventually, so a failure here is not worth an error on exit
        try; rm(profile; recursive = true, force = true); catch; end
    end
    srv
end

if abspath(PROGRAM_FILE) == @__FILE__
    # `--no-window` serves without opening anything, which is what a headless
    # check or a second window attaching to an existing server wants.
    launch(open_browser = !("--no-window" in ARGS), gpu = !("--no-gpu" in ARGS))
end
