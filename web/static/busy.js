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
  display: flex; flex-direction: column; align-items: stretch; gap: 9px;
  padding: 22px 24px; width: min(390px, calc(100vw - 32px));
  background: var(--surface); border: 1px solid var(--line);
  border-radius: var(--radius); box-shadow: 0 18px 50px rgba(0,0,0,.5);
}
.busy-head { display:flex; align-items:baseline; justify-content:space-between; gap:16px }
.busy-bar { width: 100%; height: 4px; border-radius: 2px;
            background: var(--raise); overflow: hidden; }
.busy-bar i { display: block; width: 40%; height: 100%; border-radius: 2px;
              background: var(--amber); animation: busy-slide 1.1s ease-in-out infinite; }
.busy-bar.determinate i { animation:none; transform:none; width:0;
                          transition:width .2s ease-out }
@keyframes busy-slide { 0% { transform: translateX(-110%) }
                        100% { transform: translateX(360%) } }
.busy-what { font-family: var(--sans); font-size: 13px; color: var(--text); }
.busy-t { font-family: var(--mono); font-size: 11px; color: var(--text-3);
          font-variant-numeric: tabular-nums; }
.busy-stage { display:flex; justify-content:space-between; gap:12px;
              font:600 11px/1.3 var(--mono); letter-spacing:.08em;
              text-transform:uppercase; color:var(--amber) }
.busy-count { color:var(--text-3); font-weight:500; letter-spacing:0 }
.busy-detail { min-height:18px; color:var(--text-2); font:12px/1.45 var(--sans) }
.busy-log { list-style:none; padding:7px 0 0; margin:0; border-top:1px solid var(--line);
            display:flex; flex-direction:column; gap:4px }
.busy-log li { color:var(--text-3); font:10.5px/1.35 var(--mono) }
.busy-cancel { width:auto; align-self:flex-end; padding:6px 12px; cursor:pointer;
  color:var(--text); background:var(--raise); border:1px solid var(--line); border-radius:var(--radius-sm); font:12px var(--sans) }
.busy-cancel:focus-visible { outline:2px solid var(--amber); outline-offset:2px }
.busy-log li::before { content:'✓'; color:var(--nominal); margin-right:7px }

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
let background = [], previousFocus = null;

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
export function busy(what, { onCancel, cancelLabel = 'Cancel' } = {}) {
  ensure();
  if (depth++ === 0) {
    started = performance.now();
    previousFocus = document.activeElement;
    scrim = document.createElement('div');
    scrim.className = 'busy-scrim';
    scrim.setAttribute('role', 'dialog');
    scrim.setAttribute('aria-modal', 'true');
    scrim.setAttribute('aria-label', what || 'working…');
    scrim.tabIndex = -1;
    scrim.innerHTML =
      `<div class="busy-card" role="status" aria-live="polite">
         <div class="busy-head">
           <div class="busy-what"></div><div class="busy-t">0.0 s</div>
         </div>
         <div class="busy-bar"><i></i></div>
         <div class="busy-stage"><span>starting</span><span class="busy-count"></span></div>
         <div class="busy-detail">preparing the request…</div>
         <ul class="busy-log" hidden></ul>
       </div>`;
    scrim.querySelector('.busy-what').textContent = what || 'working…';
    if (onCancel) {
      const cancel = document.createElement('button');
      cancel.type = 'button'; cancel.className = 'busy-cancel';
      cancel.textContent = cancelLabel;
      cancel.onclick = () => { cancel.disabled = true; onCancel(); };
      scrim.querySelector('.busy-card').appendChild(cancel);
    }
    document.body.appendChild(scrim);
    background = [...document.body.children].filter(el => el !== scrim)
      .map(el => [el, el.inert]);
    for (const [el] of background) el.inert = true;
    const focusAction = () => (scrim.querySelector('.busy-cancel:not(:disabled)') || scrim)
      .focus({ preventScroll: true });
    scrim.addEventListener('keydown', event => {
      if (event.key === 'Tab') {
        event.preventDefault();
        focusAction();
      } else if (event.key === 'Escape') {
        const cancel = scrim.querySelector('.busy-cancel:not(:disabled)');
        if (cancel) { event.preventDefault(); cancel.click(); }
      }
    });
    focusAction();
    const t = scrim.querySelector('.busy-t');
    timer = setInterval(() => {
      t.textContent = ((performance.now() - started) / 1000).toFixed(1) + ' s';
    }, 100);
  } else if (scrim) {
    scrim.querySelector('.busy-what').textContent = what || 'working…';
  }
  let ended = false;
  const end = () => {
    if (ended) return;          // a double call must not unbalance the count
    ended = true;
    if (--depth > 0) return;
    clearInterval(timer); timer = null;
    if (scrim) scrim.remove();
    scrim = null;
    for (const [el, inert] of background) el.inert = inert;
    background = [];
    const focus = previousFocus;
    previousFocus = null;
    // Callers re-enable their action in the same finally block.
    queueMicrotask(() => { if (focus?.isConnected) focus.focus({ preventScroll: true }); });
  };
  end.progress = p => {
    if (!scrim || !p) return;
    const stage = String(p.stage || 'working');
    const detail = String(p.detail || '');
    const stageEl = scrim.querySelector('.busy-stage span');
    const previous = stageEl.dataset.stage;
    if (previous && previous !== stage && previous !== 'complete' && previous !== 'queued') {
      const log = scrim.querySelector('.busy-log');
      const li = document.createElement('li'); li.textContent = previous;
      log.appendChild(li); log.hidden = false;
      while (log.children.length > 4) log.firstElementChild.remove();
    }
    stageEl.dataset.stage = stage;
    stageEl.textContent = stage;
    scrim.querySelector('.busy-detail').textContent = detail;
    const total = +p.total || 0, current = +p.current || 0;
    scrim.querySelector('.busy-count').textContent = total ? `${current} / ${total}` : '';
    if (total) {
      const bar = scrim.querySelector('.busy-bar');
      bar.classList.add('determinate');
      bar.querySelector('i').style.width = `${Math.max(2, Math.min(100, 100*current/total))}%`;
    }
  };
  end.say = what => say(what);
  return end;
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
