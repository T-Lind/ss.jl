# ss.jl in a window, when run from source.
#
#   julia --project -t auto scripts/desktop.jl
#   julia --project -t auto scripts/desktop.jl --no-window
#
# Starts the panel server on a port nobody else is using, opens a browser at
# it, and exits when the window goes away.
#
# THIS IS THE FALLBACK, NOT THE SHIPPED APP
#
# The released application is `bin/ssjl.exe`, a native WebView2 window written
# in Rust (`host/`), which spawns the frozen simulator with `--no-window` and
# owns the window itself. That is what gives it its own icon, its own taskbar
# entry, no Edge branding, and no console.
#
# What is left here is the from-source path, where there is no host binary to
# run: open the page in whatever browser exists. Edge `--app=` gets a window
# with no tabs and no omnibox, which is as close to an application as a
# browser gets, and it takes `--force_high_performance_gpu` (see GPU_FLAGS) —
# the one thing a web page provably cannot do for itself.
#
# The hard lesson is in `idle_out` below: a browser you launch is not a window
# you own, and v0.3.0 shipped believing otherwise.

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
function launch(; port::Int = free_port(),
                  # Defaults come from the command line, so that ALL launcher
                  # policy lives in this file. That matters more than it looks:
                  # this file is read from disk at run time while `julia_main`
                  # is frozen into the system image, so a flag handled here can
                  # be changed without a 25-minute rebuild, and one handled
                  # there cannot.
                  open_browser::Bool = !("--no-window" in ARGS),
                  gpu::Bool = !("--no-gpu" in ARGS),
                  warm::Bool = true,
                  exit_on_stdin_eof::Bool = "--exit-with-parent" in ARGS)
    if warm
        print("starting the simulator… ")
        t0 = time()
        PanelApp.panel_mission(Dict{String,String}())
        @printf("%.1f s\n", time() - t0)
    end
    srv = PanelApp.start_panel(port)
    url = "http://127.0.0.1:$port/"
    println("panel on $url")
    # The native host reads this line off our stdout to learn the port, and a
    # pipe is block-buffered where a console is not. Without the flush the
    # window can sit on its splash screen until enough log lines accumulate to
    # push this one out.
    flush(stdout)

    if !open_browser
        if exit_on_stdin_eof
            println("(serving for the desktop host — exits when it does)")
            flush(stdout)
            # Our stdin is a pipe held open by the host for exactly this
            # purpose. When the host goes away for ANY reason — window closed,
            # force-killed, user logged off — the write end closes and this
            # read returns. Without it, a host killed from Task Manager would
            # leave half a gigabyte of simulator listening forever.
            #
            # Gated behind the flag rather than done always: run from a service
            # or with stdin redirected from NUL, stdin is at EOF immediately
            # and the server would exit the instant it started.
            try; read(stdin); catch; end
            println("the desktop host is gone — shutting down")
        else
            println("(no window requested — Ctrl-C to stop)")
            flush(stdout)
            wait(srv.acceptors[1])
        end
        PanelApp.stop_panel(srv)
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
        wait(proc)
        # `wait` returning means THE PROCESS WE SPAWNED EXITED. That is not the
        # same statement as "the user closed the window", and treating it as
        # such is what shipped v0.3.0 with a window showing
        # ERR_CONNECTION_REFUSED: Edge exits early whenever it hands the URL to
        # an Edge that is already running, or cannot create its profile
        # directory, or is mid-update — and the server was then torn down
        # underneath the window that had just appeared.
        #
        # `--user-data-dir` above makes the hand-off case rare. Rare is not the
        # same as impossible, and the cost of being wrong was a broken app, so
        # the browser's exit is now only a HINT. What settles it is whether
        # anything is still talking to us.
        idle_out(srv)
    catch err
        err isa InterruptException || rethrow()
    finally
        PanelApp.stop_panel(srv)
        # best-effort: a profile directory is a few MB and the OS will clear
        # temp eventually, so a failure here is not worth an error on exit
        try; rm(profile; recursive = true, force = true); catch; end
    end
    srv
end

"Seconds of silence after which a window is presumed gone."
const IDLE_GRACE = 12.0

"""
    idle_out(srv)

Return once the UI is really gone: no request for `IDLE_GRACE` seconds.

Called after the browser process exits. Two cases have to come apart here and
they used to be one:

  * The window was closed. Traffic stopped when it did, so this returns after
    the grace period and the app shuts down — a few seconds later than before,
    which nobody can see because the window is already gone.

  * The browser handed our URL to another process and exited. A window is
    alive and talking to us. Shutting down here is precisely the bug, so this
    keeps waiting, and the app lives as long as the window does.

A browser that exits before ever loading the page is the third case: there is
no window to serve and nothing to wait for, so say what happened and leave the
server up rather than vanishing without explanation.
"""
function idle_out(srv)
    if PanelApp.LAST_REQUEST[] == 0.0
        println("""
        The browser exited without ever loading the page. That usually means it
        handed the address to a copy of itself that was already running.

        The simulator is still serving — open this in any browser:
            $(server_url(srv))
        Ctrl-C here to stop it.""")
        wait(srv.acceptors[1])
        return
    end
    while time() - PanelApp.LAST_REQUEST[] < IDLE_GRACE
        sleep(1.0)
    end
    println("window closed — shutting down")
end

"The address the panel is actually listening on."
server_url(srv) = "http://127.0.0.1:$(srv.port)/"

if abspath(PROGRAM_FILE) == @__FILE__
    # `--no-window` serves without opening anything, which is what a headless
    # check or a second window attaching to an existing server wants.
    launch()          # every flag is a default of `launch` itself, see above
end
