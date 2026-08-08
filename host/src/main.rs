// ss.jl's desktop window.
//
// WHY THIS EXISTS
//
// The first version of the desktop app opened Edge with `--app=`, which is a
// browser window with the chrome hidden: Edge's process, Edge's icon in the
// taskbar, Edge's error pages. It also made the browser process the signal for
// "the user closed the window", and that signal is a lie — `msedge.exe` exits
// early for reasons that have nothing to do with the window (it hands the URL
// to an Edge that is already running, or fails to create its profile
// directory, or is mid-update). When it did, the launcher tore the server down
// under a window that was still opening, and the user got
// ERR_CONNECTION_REFUSED.
//
// Here the window IS this process. There is no browser to hand off to, no
// separate lifetime to track, and no Edge branding. WebView2 supplies the
// renderer and ships with Windows, so this costs about 3 MB rather than the
// ~150 MB of bundling Chromium.
//
// ARCHITECTURE
//
// This binary is `bin/ssjl.exe` — the thing a user double-clicks. It spawns
// `bin/ssjl-server.exe --no-window` (the PackageCompiler-frozen simulator)
// with no console window, reads the port off its stdout, and points the
// WebView at it. Closing the window kills the server. That inversion is what
// removes the console window from the experience as well: the Julia app is a
// console subsystem binary and always will be, so it is never the thing the
// user launches.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

use tao::dpi::LogicalSize;
use tao::event::{Event, WindowEvent};
use tao::event_loop::{ControlFlow, EventLoopBuilder};
use tao::window::{Icon, WindowBuilder};
use wry::WebViewBuilder;
// `--force_high_performance_gpu` reaches WebView2 through a Windows-only
// extension trait, so the import is cfg'd the same way the flag is.
#[cfg(windows)]
use wry::WebViewBuilderExtWindows;

/// Windows `CREATE_NO_WINDOW` — the server is a console binary and must not
/// flash a black window on startup or keep one around behind the app.
#[cfg(windows)]
const CREATE_NO_WINDOW: u32 = 0x0800_0000;

/// How long to wait for the simulator to report its port. A first run on a
/// cold filesystem has to page in ~600 MB of system image, so this is
/// generous; the splash screen below is what makes the wait legible.
const STARTUP_TIMEOUT: Duration = Duration::from_secs(180);

enum UserEvent {
    /// the page asked for a new window; we navigate this one instead
    Navigate(String),
    /// the server is listening and answered a health check
    Ready(String),
    /// the server exited or never came up, with something to tell the user
    Failed(String),
    UpdateCheck,
    UpdateDownload,
    UpdateInstall,
    UpdateStatus(UpdateMessage),
}

#[derive(Clone)]
struct UpdateInfo {
    version: String,
    url: String,
    sha_url: String,
}

#[derive(Clone)]
struct PreparedUpdate {
    version: String,
    root: PathBuf,
}

#[derive(Clone)]
struct UpdateMessage {
    state: &'static str,
    message: String,
    version: String,
}

fn main() {
    let event_loop = EventLoopBuilder::<UserEvent>::with_user_event().build();
    let proxy = event_loop.create_proxy();

    let window = match WindowBuilder::new()
        .with_title("ss.jl")
        .with_inner_size(LogicalSize::new(1600.0, 1000.0))
        .with_min_inner_size(LogicalSize::new(960.0, 640.0))
        .with_window_icon(Some(app_icon()))
        .build(&event_loop)
    {
        Ok(w) => w,
        Err(e) => return fatal(&format!("ss.jl could not open a window.\n\n{e}")),
    };

    // The window appears immediately with a splash rather than after the
    // simulator has warmed up. Several seconds of nothing on screen is what
    // makes an application feel broken, and the wait here is real work.
    // A WebView2 with no NewWindowRequested handler REFUSES the request and
    // tells nobody: `target="_blank"` and `window.open` do nothing at all, no
    // error, no console message, no navigation. Two of the four nav links and
    // three keyboard shortcuts were dead in the shipped app for exactly this
    // reason, and the pages now use ordinary same-document links instead.
    //
    // This stays as the backstop, because "nothing happens" is the single
    // worst failure an application can have and one stray _blank should not
    // bring it back. Same-origin requests navigate this window; anything else
    // — a genuinely external link — is handed to the user's real browser,
    // where it belongs.
    let nav_proxy = proxy.clone();
    let update_proxy = proxy.clone();
    let builder = WebViewBuilder::new()
        .with_html(SPLASH)
        .with_background_color((10, 13, 18, 255))
        .with_initialization_script(UPDATE_UI)
        .with_ipc_handler(move |request| match request.body().as_str() {
            "update:check" => {
                let _ = update_proxy.send_event(UserEvent::UpdateCheck);
            }
            "update:download" => {
                let _ = update_proxy.send_event(UserEvent::UpdateDownload);
            }
            "update:install" => {
                let _ = update_proxy.send_event(UserEvent::UpdateInstall);
            }
            _ => {}
        })
        .with_new_window_req_handler(move |url: String, _features| {
            if url.starts_with("http://127.0.0.1:") || url.starts_with("http://localhost:") {
                let _ = nav_proxy.send_event(UserEvent::Navigate(url));
            } else {
                open_externally(&url);
            }
            // Deny in both cases: the panel has been navigated instead, and an
            // external link has gone to the real browser. Allow would open a
            // second bare WebView2 with none of this window's chrome.
            wry::NewWindowResponse::Deny
        });
    // Measured, not assumed: a page cannot move itself onto the discrete GPU
    // — Chromium binds its GPU process to one adapter at launch — but the
    // host process can ask. See README.
    #[cfg(windows)]
    let builder = builder.with_additional_browser_args("--force_high_performance_gpu");
    let webview = match builder.build(&window) {
        Ok(w) => w,
        // The one dependency this application has on the machine. Windows 11
        // ships it; a Windows 10 box that has never had Edge may not. Without
        // a console to print to, silence here would look like double-clicking
        // the icon and nothing happening at all.
        Err(e) => {
            return fatal(&format!(
                "ss.jl needs the Microsoft Edge WebView2 runtime, and could not \
                 start it.\n\nInstall the Evergreen WebView2 Runtime from \
                 Microsoft, then try again.\n\n{e}"
            ))
        }
    };

    // Server supervision runs off the UI thread; the channel carries the
    // child handle back so the close handler can kill it.
    let (tx, rx) = mpsc::channel::<Child>();
    // `--url=…` attaches to a server that is already running instead of
    // starting one. That is how this window gets tested against a working
    // tree without a 25-minute freeze, and how a second window opens onto a
    // session that already exists.
    match existing_url() {
        Some(url) => {
            let _ = proxy.send_event(UserEvent::Ready(url));
        }
        None => {
            let server_proxy = proxy.clone();
            std::thread::spawn(move || run_server(&tx, &server_proxy));
        }
    }

    let mut server: Option<Child> = None;
    let available = Arc::new(Mutex::new(None::<UpdateInfo>));
    let prepared = Arc::new(Mutex::new(None::<PreparedUpdate>));

    event_loop.run(move |event, _, control_flow| {
        *control_flow = ControlFlow::Wait;
        // pick up the child handle as soon as the supervisor has spawned it,
        // so a window closed DURING startup still kills the server
        if server.is_none() {
            if let Ok(child) = rx.try_recv() {
                server = Some(child);
            }
        }
        match event {
            Event::UserEvent(UserEvent::Ready(url))
            | Event::UserEvent(UserEvent::Navigate(url)) => {
                let _ = webview.load_url(&url);
            }
            Event::UserEvent(UserEvent::Failed(why)) => {
                let _ = webview.load_html(&failure_page(&why));
            }
            Event::UserEvent(UserEvent::UpdateCheck) => {
                let proxy = proxy.clone();
                let available = Arc::clone(&available);
                std::thread::spawn(move || check_for_update(proxy, available));
            }
            Event::UserEvent(UserEvent::UpdateDownload) => {
                let info = available.lock().ok().and_then(|g| g.clone());
                let proxy = proxy.clone();
                let prepared = Arc::clone(&prepared);
                std::thread::spawn(move || match info {
                    Some(i) => prepare_update(proxy, prepared, i),
                    None => send_update(&proxy, "error", "check for an update first", ""),
                });
            }
            Event::UserEvent(UserEvent::UpdateInstall) => {
                let ready = prepared.lock().ok().and_then(|g| g.clone());
                match ready.and_then(|p| launch_update(&p).ok()) {
                    Some(()) => {
                        if let Some(child) = server.as_mut() {
                            let _ = child.kill();
                            let _ = child.wait();
                        }
                        *control_flow = ControlFlow::Exit;
                    }
                    None => send_update(
                        &proxy,
                        "error",
                        "the prepared update could not be started",
                        "",
                    ),
                }
            }
            Event::UserEvent(UserEvent::UpdateStatus(msg)) => {
                let _ = webview.evaluate_script(&update_event_script(&msg));
            }
            Event::WindowEvent {
                event: WindowEvent::CloseRequested,
                ..
            } => {
                if let Some(child) = server.as_mut() {
                    let _ = child.kill();
                    let _ = child.wait();
                }
                *control_flow = ControlFlow::Exit;
            }
            _ => {}
        }
    });
}

/// Report a failure that happens before there is a window to report it in.
///
/// This binary is a Windows-subsystem process on purpose — a console window
/// behind the app is exactly the thing that makes something not feel like an
/// application — which means it has no stdout and no stderr. A message box is
/// the only way a startup failure reaches the person who double-clicked.
/// Declared by hand rather than pulling in a Windows binding crate for one
/// call.
/// A NUL-terminated UTF-16 string, which is what every `…W` entry point wants.
#[cfg(windows)]
fn wide(s: &str) -> Vec<u16> {
    use std::os::windows::ffi::OsStrExt;
    std::ffi::OsStr::new(s)
        .encode_wide()
        .chain(std::iter::once(0))
        .collect()
}

/// Hand a link that is not ours to whatever the user browses with.
///
/// Restricted to http and https on purpose. This receives a string chosen by
/// the page, and `ShellExecuteW` will happily launch a `file:` path or a
/// registered `ms-…:` protocol handler — an application should not be a
/// general-purpose way to start other programs.
#[cfg(windows)]
fn open_externally(url: &str) {
    if !(url.starts_with("http://") || url.starts_with("https://")) {
        return;
    }
    #[link(name = "shell32")]
    extern "system" {
        fn ShellExecuteW(
            hwnd: *mut core::ffi::c_void,
            op: *const u16,
            file: *const u16,
            params: *const u16,
            dir: *const u16,
            show: i32,
        ) -> *mut core::ffi::c_void;
    }
    const SW_SHOWNORMAL: i32 = 1;
    unsafe {
        ShellExecuteW(
            std::ptr::null_mut(),
            wide("open").as_ptr(),
            wide(url).as_ptr(),
            std::ptr::null(),
            std::ptr::null(),
            SW_SHOWNORMAL,
        );
    }
}

#[cfg(not(windows))]
fn open_externally(_url: &str) {}

#[cfg(windows)]
fn fatal(msg: &str) {
    #[link(name = "user32")]
    extern "system" {
        fn MessageBoxW(
            hwnd: *mut core::ffi::c_void,
            text: *const u16,
            caption: *const u16,
            utype: u32,
        ) -> i32;
    }
    const MB_ICONERROR: u32 = 0x10;
    unsafe {
        MessageBoxW(
            std::ptr::null_mut(),
            wide(msg).as_ptr(),
            wide("ss.jl").as_ptr(),
            MB_ICONERROR,
        );
    }
}

#[cfg(not(windows))]
fn fatal(msg: &str) {
    eprintln!("{msg}");
}

/// A `--url=…` argument, if one was given.
fn existing_url() -> Option<String> {
    std::env::args()
        .find_map(|a| a.strip_prefix("--url=").map(str::to_string))
        .filter(|u| !u.is_empty())
}

fn send_update(
    proxy: &tao::event_loop::EventLoopProxy<UserEvent>,
    state: &'static str,
    message: &str,
    version: &str,
) {
    let _ = proxy.send_event(UserEvent::UpdateStatus(UpdateMessage {
        state,
        message: message.to_string(),
        version: version.to_string(),
    }));
}

fn semver(s: &str) -> (u32, u32, u32) {
    let mut n = s.trim_start_matches('v').split('.').map(|x| {
        x.chars()
            .take_while(|c| c.is_ascii_digit())
            .collect::<String>()
            .parse::<u32>()
            .unwrap_or(0)
    });
    (
        n.next().unwrap_or(0),
        n.next().unwrap_or(0),
        n.next().unwrap_or(0),
    )
}

#[cfg(windows)]
fn powershell(script: &str) -> Result<std::process::Output, String> {
    use std::os::windows::process::CommandExt;
    Command::new("powershell.exe")
        .args(["-NoProfile", "-NonInteractive", "-Command", script])
        .creation_flags(CREATE_NO_WINDOW)
        .output()
        .map_err(|e| format!("could not start PowerShell: {e}"))
}

#[cfg(windows)]
fn check_for_update(
    proxy: tao::event_loop::EventLoopProxy<UserEvent>,
    available: Arc<Mutex<Option<UpdateInfo>>>,
) {
    send_update(&proxy, "checking", "checking GitHub Releases...", "");
    let script = format!(
        r#"$ErrorActionPreference='Stop';
$h=@{{'User-Agent'='ssjl/{}'}};
$r=Invoke-RestMethod -Headers $h -Uri 'https://api.github.com/repos/T-Lind/ss.jl/releases/latest';
$a=$r.assets | Where-Object {{$_.name -eq 'ssjl-windows-x64.zip'}} | Select-Object -First 1;
$s=$r.assets | Where-Object {{$_.name -eq 'ssjl-windows-x64.zip.sha256'}} | Select-Object -First 1;
if($null -eq $a -or $null -eq $s){{throw 'the latest release has no verified Windows package'}};
Write-Output $r.tag_name; Write-Output $a.browser_download_url; Write-Output $s.browser_download_url"#,
        env!("CARGO_PKG_VERSION")
    );
    match powershell(&script) {
        Ok(out) if out.status.success() => {
            let text = String::from_utf8_lossy(&out.stdout);
            let mut lines = text.lines().map(str::trim).filter(|x| !x.is_empty());
            let version = lines
                .next()
                .unwrap_or("")
                .trim_start_matches('v')
                .to_string();
            let url = lines.next().unwrap_or("").to_string();
            let sha_url = lines.next().unwrap_or("").to_string();
            if version.is_empty()
                || !url.starts_with("https://github.com/T-Lind/ss.jl/releases/download/")
                || !sha_url.starts_with("https://github.com/T-Lind/ss.jl/releases/download/")
            {
                return send_update(&proxy, "error", "GitHub returned an invalid release", "");
            }
            if semver(&version) > semver(env!("CARGO_PKG_VERSION")) {
                if let Ok(mut slot) = available.lock() {
                    *slot = Some(UpdateInfo {
                        version: version.clone(),
                        url,
                        sha_url,
                    });
                }
                send_update(
                    &proxy,
                    "available",
                    "a tested Windows build is ready",
                    &version,
                );
            } else {
                send_update(
                    &proxy,
                    "current",
                    "you are on the latest release",
                    env!("CARGO_PKG_VERSION"),
                );
            }
        }
        Ok(out) => send_update(
            &proxy,
            "error",
            String::from_utf8_lossy(&out.stderr).trim(),
            "",
        ),
        Err(e) => send_update(&proxy, "error", &e, ""),
    }
}

#[cfg(not(windows))]
fn check_for_update(
    proxy: tao::event_loop::EventLoopProxy<UserEvent>,
    _available: Arc<Mutex<Option<UpdateInfo>>>,
) {
    send_update(
        &proxy,
        "error",
        "automatic updates are available in the Windows build",
        "",
    );
}

fn ps_quote(path: &Path) -> String {
    format!("'{}'", path.to_string_lossy().replace('\'', "''"))
}

#[cfg(windows)]
fn prepare_update(
    proxy: tao::event_loop::EventLoopProxy<UserEvent>,
    prepared: Arc<Mutex<Option<PreparedUpdate>>>,
    info: UpdateInfo,
) {
    send_update(
        &proxy,
        "downloading",
        "downloading and verifying the release package...",
        &info.version,
    );
    let base = match std::env::var("LOCALAPPDATA").or_else(|_| std::env::var("TEMP")) {
        Ok(x) => PathBuf::from(x).join("ssjl").join("updates"),
        Err(_) => {
            return send_update(
                &proxy,
                "error",
                "no writable update directory is available",
                "",
            )
        }
    };
    let zip = base.join(format!("ssjl-{}.zip", info.version));
    let sha = base.join(format!("ssjl-{}.sha256", info.version));
    let stage = base.join(format!("ssjl-{}", info.version));
    let script = format!(
        r#"$ErrorActionPreference='Stop';$ProgressPreference='SilentlyContinue';
New-Item -ItemType Directory -Force -Path {base} | Out-Null;
if(Test-Path -LiteralPath {stage}){{Remove-Item -LiteralPath {stage} -Recurse -Force}};
Invoke-WebRequest -Headers @{{'User-Agent'='ssjl/{current}'}} -Uri '{url}' -OutFile {zip};
Invoke-WebRequest -Headers @{{'User-Agent'='ssjl/{current}'}} -Uri '{sha_url}' -OutFile {sha};
$expected=((Get-Content -LiteralPath {sha} -Raw).Trim().Split()[0]).ToUpperInvariant();
$actual=(Get-FileHash -LiteralPath {zip} -Algorithm SHA256).Hash.ToUpperInvariant();
if($expected -ne $actual){{throw 'the downloaded package failed its SHA-256 check'}};
Expand-Archive -LiteralPath {zip} -DestinationPath {stage} -Force;
$root={stage};
if(!(Test-Path -LiteralPath (Join-Path $root 'ssjl.exe'))){{$d=Get-ChildItem -LiteralPath $root -Directory | Select-Object -First 1;if($d){{$root=$d.FullName}}}};
if(!(Test-Path -LiteralPath (Join-Path $root 'ssjl.exe')) -or !(Test-Path -LiteralPath (Join-Path $root 'bin\ssjl-server.exe'))){{throw 'the release package is incomplete'}};
Write-Output $root"#,
        base = ps_quote(&base),
        stage = ps_quote(&stage),
        zip = ps_quote(&zip),
        sha = ps_quote(&sha),
        current = env!("CARGO_PKG_VERSION"),
        url = info.url.replace('\'', "%27"),
        sha_url = info.sha_url.replace('\'', "%27")
    );
    match powershell(&script) {
        Ok(out) if out.status.success() => {
            let root = PathBuf::from(String::from_utf8_lossy(&out.stdout).trim());
            if let Ok(mut slot) = prepared.lock() {
                *slot = Some(PreparedUpdate {
                    version: info.version.clone(),
                    root,
                });
            }
            send_update(
                &proxy,
                "ready",
                "download complete - restart to apply",
                &info.version,
            );
        }
        Ok(out) => send_update(
            &proxy,
            "error",
            String::from_utf8_lossy(&out.stderr).trim(),
            &info.version,
        ),
        Err(e) => send_update(&proxy, "error", &e, &info.version),
    }
}

#[cfg(not(windows))]
fn prepare_update(
    proxy: tao::event_loop::EventLoopProxy<UserEvent>,
    _prepared: Arc<Mutex<Option<PreparedUpdate>>>,
    _info: UpdateInfo,
) {
    send_update(
        &proxy,
        "error",
        "automatic updates are available in the Windows build",
        "",
    );
}

#[cfg(windows)]
fn launch_update(update: &PreparedUpdate) -> Result<(), String> {
    use std::os::windows::process::CommandExt;
    let exe = std::env::current_exe().map_err(|e| e.to_string())?;
    let app = exe
        .parent()
        .ok_or("the application directory is missing")?
        .to_path_buf();
    if !app.join("bin").join("ssjl-server.exe").is_file() {
        return Err("updates can only be installed by a packaged build".into());
    }
    let name = app.file_name().unwrap_or_default().to_string_lossy();
    let backup = app.with_file_name(format!("{name}.previous"));
    let script = format!(
        r#"$ErrorActionPreference='Stop';Start-Sleep -Milliseconds 1200;
$app={app};$new={new};$backup={backup};
try{{
 if(Test-Path -LiteralPath $backup){{Remove-Item -LiteralPath $backup -Recurse -Force}};
 Move-Item -LiteralPath $app -Destination $backup;
 Move-Item -LiteralPath $new -Destination $app;
 Start-Process -FilePath (Join-Path $app 'ssjl.exe');
 Start-Sleep -Seconds 3;
 Remove-Item -LiteralPath $backup -Recurse -Force
}}catch{{
 if(!(Test-Path -LiteralPath $app) -and (Test-Path -LiteralPath $backup)){{Move-Item -LiteralPath $backup -Destination $app}};
 if(Test-Path -LiteralPath (Join-Path $app 'ssjl.exe')){{Start-Process -FilePath (Join-Path $app 'ssjl.exe')}};
 throw
}}"#,
        app = ps_quote(&app),
        new = ps_quote(&update.root),
        backup = ps_quote(&backup)
    );
    let base = std::env::var("LOCALAPPDATA")
        .or_else(|_| std::env::var("TEMP"))
        .map_err(|_| "no update directory".to_string())?;
    let path = PathBuf::from(base)
        .join("ssjl")
        .join(format!("apply-{}.ps1", update.version));
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    let mut bytes = vec![0xff, 0xfe];
    for w in script.encode_utf16() {
        bytes.extend_from_slice(&w.to_le_bytes());
    }
    std::fs::write(&path, bytes).map_err(|e| e.to_string())?;
    Command::new("powershell.exe")
        .args([
            "-NoProfile",
            "-NonInteractive",
            "-ExecutionPolicy",
            "Bypass",
            "-File",
        ])
        .arg(path)
        .creation_flags(CREATE_NO_WINDOW)
        .spawn()
        .map_err(|e| e.to_string())?;
    Ok(())
}

#[cfg(not(windows))]
fn launch_update(_update: &PreparedUpdate) -> Result<(), String> {
    Err("automatic updates are available in the Windows build".into())
}

fn js_string(s: &str) -> String {
    format!(
        "\"{}\"",
        s.replace('\\', "\\\\")
            .replace('\"', "\\\"")
            .replace('\n', "\\n")
            .replace('\r', "")
    )
}

fn update_event_script(m: &UpdateMessage) -> String {
    format!("window.dispatchEvent(new CustomEvent('ssjl-update',{{detail:{{state:{},message:{},version:{}}}}}));",
            js_string(m.state), js_string(&m.message), js_string(&m.version))
}

/// Spawn the simulator, announce its URL, then keep reading its output for as
/// long as it lives.
///
/// The draining is not optional and it is not tidiness. The server logs one
/// line per request to stdout, and stdout here is a pipe: stop reading it and
/// the OS buffer fills after a few hundred requests, at which point the server
/// blocks forever inside a `println` and the application appears to freeze
/// mid-session. Everything read is written to a log file, so a user reporting
/// a problem has something to send.
fn run_server(tx: &mpsc::Sender<Child>, proxy: &tao::event_loop::EventLoopProxy<UserEvent>) {
    let mut log = open_log();
    let mut announced = false;
    match start_server(tx) {
        Err(why) => {
            let _ = proxy.send_event(UserEvent::Failed(why));
        }
        Ok(mut lines) => {
            // The timeout has to run on its own thread. The read below blocks
            // until a newline arrives, and the server prints nothing at all
            // while it warms up — so checking the clock inside the loop would
            // never fire for the one failure it exists to catch, a simulator
            // that wedges before saying anything. That leaves a splash screen
            // spinning forever, which is the worst thing an application can do.
            let flagged = Arc::new(AtomicBool::new(false));
            {
                let flagged = Arc::clone(&flagged);
                let proxy = proxy.clone();
                std::thread::spawn(move || {
                    std::thread::sleep(STARTUP_TIMEOUT);
                    if !flagged.load(Ordering::SeqCst) {
                        let _ = proxy.send_event(UserEvent::Failed(
                            "the simulator did not start within three minutes.".into(),
                        ));
                    }
                });
            }
            while let Some(Ok(line)) = lines.next() {
                if let Some(w) = log.as_mut() {
                    use std::io::Write;
                    let _ = writeln!(w, "{line}");
                    let _ = w.flush();
                }
                if !announced {
                    if let Some(url) = parse_url(&line) {
                        announced = true;
                        flagged.store(true, Ordering::SeqCst);
                        let _ = proxy.send_event(UserEvent::Ready(url));
                    }
                }
            }
            // stdout closed: the server is gone.
            flagged.store(true, Ordering::SeqCst);
            if !announced {
                let _ = proxy.send_event(UserEvent::Failed(
                    "the simulator exited before it began serving.".into(),
                ));
            }
        }
    }
}

/// `%LOCALAPPDATA%\ssjl\ssjl.log`, truncated per run. Best effort — a machine
/// where this cannot be opened should still get a window.
fn open_log() -> Option<std::io::BufWriter<std::fs::File>> {
    let base = std::env::var("LOCALAPPDATA")
        .or_else(|_| std::env::var("TMPDIR"))
        .or_else(|_| std::env::var("TEMP"))
        .ok()?;
    let dir = PathBuf::from(base).join("ssjl");
    std::fs::create_dir_all(&dir).ok()?;
    std::fs::File::create(dir.join("ssjl.log"))
        .ok()
        .map(std::io::BufWriter::new)
}

/// Spawn the simulator and hand back its output, line by line.
fn start_server(
    tx: &mpsc::Sender<Child>,
) -> Result<std::io::Lines<BufReader<std::process::ChildStdout>>, String> {
    let exe = server_path()?;
    let mut cmd = Command::new(&exe);
    cmd.arg("--no-window")
        // The server holds this pipe open and exits when it closes. Killing
        // the child in the close handler covers the ordinary path; this covers
        // every other way this process can die — Task Manager, a crash, the
        // user logging off — none of which run our handler, and any of which
        // would otherwise strand half a gigabyte of simulator listening on a
        // port forever.
        .arg("--exit-with-parent")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        // Inherited from us, the child would otherwise start single-threaded
        // and a mission in flight would block every other request. The server
        // re-execs itself to fix that when this is absent; setting it here
        // saves the whole second process.
        .env("JULIA_NUM_THREADS", "auto")
        .env("SSJL_THREADED", "1");
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        cmd.creation_flags(CREATE_NO_WINDOW);
    }

    let mut child = cmd
        .spawn()
        .map_err(|e| format!("could not start {}: {e}", exe.display()))?;

    let stdout = child
        .stdout
        .take()
        .ok_or_else(|| "the simulator gave us no output to read".to_string())?;

    // hand the child over before blocking on its output, so closing the
    // window during a slow start still has something to kill
    let _ = tx.send(child);

    Ok(BufReader::new(stdout).lines())
}

/// Pull the URL out of the server's own announcement line, which reads
/// `panel on http://127.0.0.1:53613/`. Parsed rather than assumed because the
/// port is chosen at run time — a fixed port fails the moment a second window
/// opens or a dev server is already running.
fn parse_url(line: &str) -> Option<String> {
    let start = line.find("http://127.0.0.1:")?;
    let rest = &line[start..];
    let end = rest.find(char::is_whitespace).unwrap_or(rest.len());
    let url = rest[..end].trim_end_matches(|c: char| !c.is_ascii_alphanumeric() && c != '/');
    if url.len() > "http://127.0.0.1:".len() {
        Some(url.to_string())
    } else {
        None
    }
}

/// Where the simulator is. `SSJL_SERVER` overrides, which is how a debug
/// build gets pointed at a working tree.
///
/// Two locations are tried because this binary sits at the ROOT of the unpacked
/// folder while the simulator sits in `bin/` with its ~30 runtime DLLs. That
/// layout is deliberate: the first thing a user sees after unzipping should be
/// one obviously-runnable `ssjl.exe`, not a `bin` directory to go hunting
/// through. Alongside is checked first anyway, so a flat build still works.
fn server_path() -> Result<PathBuf, String> {
    if let Ok(p) = std::env::var("SSJL_SERVER") {
        return Ok(PathBuf::from(p));
    }
    let here =
        std::env::current_exe().map_err(|e| format!("cannot locate this executable: {e}"))?;
    let dir = here
        .parent()
        .ok_or_else(|| "this executable has no directory".to_string())?;
    let name = if cfg!(windows) {
        "ssjl-server.exe"
    } else {
        "ssjl-server"
    };
    for candidate in [dir.join(name), dir.join("bin").join(name)] {
        if candidate.is_file() {
            return Ok(candidate);
        }
    }
    Err(format!(
        "the simulator is missing — expected {name} beside this program or in \
         its bin folder, under {}",
        dir.display()
    ))
}

/// A 64×64 mark, drawn rather than shipped as a file so the build has one
/// fewer binary asset to keep in step: the ground and amber of the app's own
/// palette, an orbit, and the body on it.
fn app_icon() -> Icon {
    const N: u32 = 64;
    let mut rgba = Vec::with_capacity((N * N * 4) as usize);
    let c = (N as f32 - 1.0) / 2.0;
    for y in 0..N {
        for x in 0..N {
            let (dx, dy) = (x as f32 - c, y as f32 - c);
            // an ellipse, seen at an angle, the way every trajectory in the
            // app is drawn
            let r = ((dx * dx) / (28.0 * 28.0) + (dy * dy) / (16.0 * 16.0)).sqrt();
            let orbit = (r - 1.0).abs() < 0.085;
            let body = (dx * dx + dy * dy).sqrt() < 8.5;
            let px: [u8; 4] = if body {
                [0xF0, 0xA5, 0x00, 0xFF] // --amber
            } else if orbit {
                [0x4A, 0x9E, 0xFF, 0xFF] // --data
            } else {
                [0x0A, 0x0D, 0x12, 0xFF] // --ground
            };
            rgba.extend_from_slice(&px);
        }
    }
    Icon::from_rgba(rgba, N, N).expect("the icon is generated, so it cannot be malformed")
}

/// Shown while the simulator warms up. Deliberately styled like the app so
/// the window does not flash a white page first.
const UPDATE_UI: &str = concat!(
    r#"
(() => {
  const current = '"#,
    env!("CARGO_PKG_VERSION"),
    r#"';
  function mount() {
    const nav = document.querySelector('.appnav');
    if (!nav || document.getElementById('host-update')) return;
    const b = document.createElement('button'); b.id = 'host-update';
    b.textContent = 'v' + current; b.title = 'Check for updates';
    b.style.cssText = 'margin-left:auto;background:transparent;color:var(--text-3,#8b97a8);border:1px solid var(--line,#273140);border-radius:4px;padding:4px 9px;font:10px ui-monospace,monospace;cursor:pointer';
    const end = nav.querySelector('.appnav-end'); nav.insertBefore(b, end || null);
    const toast = document.createElement('div'); toast.id = 'host-update-note';
    toast.style.cssText = 'display:none;position:fixed;z-index:120;right:14px;top:66px;max-width:340px;padding:10px 12px;background:var(--surface,#131922);color:var(--text,#e4e9f0);border:1px solid var(--amber,#f0a500);border-radius:5px;box-shadow:0 12px 34px #0008;font:12px/1.45 system-ui';
    document.body.appendChild(toast);
    const send = m => window.ipc && window.ipc.postMessage(m);
    const action = m => { b.onclick = () => send(m); b.disabled = false; };
    b.onclick = () => send('update:check');
    window.addEventListener('ssjl-update', e => {
      const d=e.detail||{}, v=d.version ? ' v'+d.version : '';
      b.title=d.message||'Update status'; toast.textContent=d.message||'';
      if(d.state==='checking'){b.textContent='checking...';b.disabled=true}
      else if(d.state==='current'){b.textContent='v'+current;action('update:check');toast.style.display='none'}
      else if(d.state==='available'){b.textContent='update'+v;action('update:download');toast.textContent=`${d.message}. Click “update${v}” to download it.`;toast.style.display='block'}
      else if(d.state==='downloading'){b.textContent='downloading'+v;b.disabled=true;toast.style.display='block'}
      else if(d.state==='ready'){b.textContent='restart to update';action('update:install');toast.style.display='block'}
      else if(d.state==='error'){b.textContent='update issue';action('update:check');toast.style.borderColor='var(--failed,#f85149)';toast.style.display='block'}
    });
    const last=+(sessionStorage.getItem('ssjl.update.checked')||0);
    if(Date.now()-last>21600000) setTimeout(() => {
      sessionStorage.setItem('ssjl.update.checked',String(Date.now()));
      send('update:check');
    }, 1800);
  }
  document.addEventListener('DOMContentLoaded', mount);
})();
"#
);

const SPLASH: &str = r#"<!doctype html><html><head><meta charset="utf-8">
<style>
  html,body{height:100%;margin:0}
  body{background:#0A0D12;color:#E4E9F0;display:flex;align-items:center;
       justify-content:center;
       font:14px/1.5 ui-monospace,"Cascadia Mono",Consolas,monospace}
  .b{text-align:center}
  .t{font-size:22px;font-weight:600;letter-spacing:.02em}
  .t i{color:#F0A500;font-style:normal}
  .s{color:#8B97A8;margin-top:10px}
  .bar{width:200px;height:2px;background:#232C3A;margin:18px auto 0;
       overflow:hidden}
  .bar i{display:block;width:70px;height:2px;background:#F0A500;
         animation:g 1.1s ease-in-out infinite}
  @keyframes g{0%{transform:translateX(-70px)}100%{transform:translateX(200px)}}
  @media (prefers-reduced-motion:reduce){.bar i{animation:none;width:200px}}
</style></head><body><div class="b">
  <div class="t">ss<i>.</i>jl</div>
  <div class="s">starting the simulator…</div>
  <div class="bar"><i></i></div>
</div></body></html>"#;

/// A failure the user can act on, in the window, rather than an exit code
/// nobody sees. This process has no console to print to by design.
fn failure_page(why: &str) -> String {
    let escaped = why
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;");
    format!(
        r#"<!doctype html><html><head><meta charset="utf-8"><style>
  html,body{{height:100%;margin:0}}
  body{{background:#0A0D12;color:#E4E9F0;display:flex;align-items:center;
       justify-content:center;padding:32px;
       font:14px/1.6 ui-monospace,"Cascadia Mono",Consolas,monospace}}
  .b{{max-width:560px}}
  .h{{color:#F85149;font-size:17px;font-weight:600;margin-bottom:12px}}
  .m{{background:#131922;border-left:3px solid #F85149;padding:12px 14px;
      border-radius:3px;color:#E4E9F0}}
  .n{{color:#8B97A8;margin-top:16px}}
</style></head><body><div class="b">
  <div class="h">ss.jl could not start</div>
  <div class="m">{escaped}</div>
  <div class="n">The simulator lives beside this window as
  <b>ssjl-server.exe</b>. If it is missing, the download was unpacked
  incompletely — unzip the whole folder and keep it together.</div>
</div></body></html>"#
    )
}
