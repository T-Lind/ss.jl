// Saying what the program is doing, and saying when it broke.
//
// Everything here exists because of one report: "Analysis and Launch View
// won't load when I click on them." Two of those clicks were doing nothing at
// all (a WebView2 refusing `target="_blank"`), but the reason that was hard to
// tell APART from slowness is that a page which is thinking looked exactly
// like a page which is broken — both are a screen that does not change. A
// mission takes seconds of real simulation, so "nothing has changed yet" is a
// normal state here and has to be a visible one.
//
// Two surfaces, because they answer different questions:
//
//   busy()  — "is it working?"  A scrim over the page with a live elapsed
//             clock. The clock is the point: a spinner that might be stuck
//             and a spinner that is nine seconds into a twelve second job
//             look the same, and only one of them is worth waiting for.
//   fail()  — "what went wrong?"  A banner that stays until dismissed,
//             carrying the actual message rather than a shrug.
//
// Self-contained on purpose: it injects its own styles from the shared tokens
// and is imported by four pages. Having each page grow its own spinner is how
// they ended up with three different formatters for the same number.

const CSS = `
.busy-scrim {
  position: fixed; inset: 0; z-index: 90;
  display: flex; align-items: center; justify-content: center;
  background: color-mix(in srgb, var(--ground) 78%, transparent);
  backdrop-filter: blur(2px);
  /* Fade in rather than flash: work that finishes in 150 ms should not
     produce a visible blink, and most geometry requests do. */
  animation: busy-in .18s ease-out both; animation-delay: .12s;
}
@keyframes busy-in { from { opacity: 0 } to { opacity: 1 } }
.busy-card {
  display: flex; flex-direction: column; align-items: center; gap: 10px;
  padding: 22px 30px; min-width: 260px;
  background: var(--surface); border: 1px solid var(--line);
  border-radius: var(--radius); box-shadow: 0 18px 50px rgba(0,0,0,.5);
}
.busy-bar { width: 200px; height: 3px; border-radius: 2px;
            background: var(--raise); overflow: hidden; }
.busy-bar i { display: block; width: 40%; height: 100%; border-radius: 2px;
              background: var(--amber); animation: busy-slide 1.1s ease-in-out infinite; }
@keyframes busy-slide { 0% { transform: translateX(-110%) }
                        100% { transform: translateX(360%) } }
.busy-what { font-family: var(--sans); font-size: 13px; color: var(--text); text-align: center; }
.busy-t { font-family: var(--mono); font-size: 11px; color: var(--text-3);
          font-variant-numeric: tabular-nums; }

.busy-errs { position: fixed; z-index: 95; right: 14px; bottom: 14px;
             display: flex; flex-direction: column; gap: 8px;
             max-width: min(520px, calc(100vw - 28px)); }
.busy-err {
  display: grid; grid-template-columns: 1fr auto; gap: 4px 12px;
  padding: 10px 12px; background: var(--surface);
  border: 1px solid var(--failed); border-left-width: 3px;
  border-radius: var(--radius-sm); box-shadow: 0 10px 30px rgba(0,0,0,.45);
}
.busy-err h4 { margin: 0; font-family: var(--sans); font-size: 12px;
               font-weight: 600; color: var(--failed);
               text-transform: uppercase; letter-spacing: .08em; }
.busy-err p { margin: 0; grid-column: 1 / -1; font-family: var(--mono);
              font-size: 11.5px; line-height: 1.45; color: var(--text);
              overflow-wrap: anywhere; }
.busy-err button { background: none; border: 0; cursor: pointer; padding: 0 2px;
                   color: var(--text-3); font-size: 15px; line-height: 1; }
.busy-err button:hover { color: var(--text); }

@media (prefers-reduced-motion: reduce) {
  .busy-scrim { animation: none }
  .busy-bar i { animation: none; width: 100% }
}
`;

let root = null;

function ensure() {
  if (root) return root;
  const style = document.createElement('style');
  style.textContent = CSS;
  document.head.appendChild(style);
  root = document.createElement('div');
  root.className = 'busy-errs';
  document.body.appendChild(root);
  return root;
}

let depth = 0, scrim = null, timer = null, started = 0;

/**
 * Cover the page while something slow happens.
 *
 * Returns the function that ends it. Nested calls are counted, so two
 * overlapping requests do not leave a scrim behind when the first finishes —
 * that failure mode is indistinguishable from a hang, which is the exact
 * thing this is here to rule out.
 *
 *     const done = busy('flying the mission…');
 *     try { await work(); } finally { done(); }
 */
export function busy(what) {
  ensure();
  if (depth++ === 0) {
    started = performance.now();
    scrim = document.createElement('div');
    scrim.className = 'busy-scrim';
    scrim.innerHTML =
      `<div class="busy-card" role="status" aria-live="polite">
         <div class="busy-bar"><i></i></div>
         <div class="busy-what"></div>
         <div class="busy-t">0.0 s</div>
       </div>`;
    scrim.querySelector('.busy-what').textContent = what || 'working…';
    document.body.appendChild(scrim);
    const t = scrim.querySelector('.busy-t');
    timer = setInterval(() => {
      t.textContent = ((performance.now() - started) / 1000).toFixed(1) + ' s';
    }, 100);
  } else if (scrim) {
    scrim.querySelector('.busy-what').textContent = what || 'working…';
  }
  let ended = false;
  return () => {
    if (ended) return;          // a double call must not unbalance the count
    ended = true;
    if (--depth > 0) return;
    clearInterval(timer); timer = null;
    if (scrim) scrim.remove();
    scrim = null;
  };
}

/** Update the message of the overlay currently showing, if there is one. */
export function say(what) {
  if (scrim) scrim.querySelector('.busy-what').textContent = what;
}

/** Is something on screen claiming to be in progress? For the selftests. */
export function isBusy() { return depth > 0; }

/**
 * Report a failure, and keep reporting it until it is dismissed.
 *
 * `title` names the operation ("mission", "geometry") so a banner says which
 * of several things on the page failed — the previous behaviour was a status
 * line that the next successful request overwrote.
 */
export function fail(err, title = 'error') {
  const host = ensure();
  const msg = String((err && err.message) || err || 'unknown error');
  // Identical repeated failures — a poll retrying against a dead server —
  // should not build a wall of banners.
  for (const prev of host.children) {
    if (prev.dataset.msg === msg) return prev;
  }
  const el = document.createElement('div');
  el.className = 'busy-err';
  el.dataset.msg = msg;
  el.innerHTML = '<h4></h4><button title="dismiss" aria-label="dismiss">&times;</button><p></p>';
  el.querySelector('h4').textContent = title;
  el.querySelector('p').textContent = msg;
  el.querySelector('button').onclick = () => el.remove();
  host.appendChild(el);
  return el;
}

/** Remove every error banner. Called when an operation finally succeeds. */
export function clearFailures() {
  if (root) root.replaceChildren();
}

/**
 * Surface the errors nobody caught.
 *
 * A throw inside a render function used to leave a half-drawn page and a
 * message in a console nobody has open — in the desktop app there is no
 * console to open at all. This is the difference between "it's broken" and a
 * report that names the file and line.
 */
export function reportUncaught() {
  window.addEventListener('error', e => {
    fail(e.error || e.message, 'unexpected error');
  });
  window.addEventListener('unhandledrejection', e => {
    fail(e.reason, 'unfinished operation');
  });
}
