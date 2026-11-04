/* lander_hud.js
 * Lunar-descent instrument console for the ss.jl 3D launch view.
 *
 *   const hud = createLanderHud(document.body);
 *   hud.update(state);      // every frame; see the state fields below
 *   hud.setVisible(false);  // hide without tearing down; updates are still accepted
 *   hud.destroy();          // remove DOM and listeners; drops the shared <style> with the last instance
 *
 * Every field of `state` may be null or missing; nulls render as an em dash.
 * DOM is written only when a value changes. Canvases redraw at most 30 fps.
 * Colours come from tokens.css (--amber, --text, --nominal, ...) with fallbacks.
 *
 * Profile note: the altitude axis is square-root scaled so the last few hundred
 * metres, where the crew's attention is, are not crushed against the baseline.
 */

const DASH = '—';
const MINUS = '−';
const TAPE_STEPS = [20, 200, 2000, 20000, 200000]; // metres, full scale
const VS_STEPS = [5, 20, 100];                      // m/s, half scale
const XP_STEPS = [2, 20, 200];                      // m/s, full scale
const FRAME_MS = 1000 / 30;
const BIG_MS = 4000;
const LAMPS = [
  ['pdi', 'PDI', 'c-white'],
  ['highGate', 'HI GATE', 'c-white'],
  ['lowGate', 'LO GATE', 'c-white'],
  ['contact', 'CONTACT', 'c-blue'],
  ['landed', 'LANDED', 'c-green'],
  ['lowFuel', 'LOW FUEL', 'c-amber blink'],
  ['abort', 'ABORT', 'c-red blink'],
];
const CELLS = [
  ['alt', 'ALT', true],
  ['vs', 'VS', true],
  ['hvel', 'HVEL', false],
  ['tosite', 'TO SITE', false],
  ['ttd', 'T-TD', false],
  ['naverr', 'NAV ERR', false],
  ['ground', 'GROUND', false],
];

const CSS = `
.lh-root{position:fixed;left:50%;bottom:56px;transform:translateX(-50%) translate(var(--lh-dx,0px),var(--lh-dy,0px));width:min(760px,60vw);z-index:2;pointer-events:none;box-sizing:border-box;font-family:var(--sans,system-ui,sans-serif);color:var(--text,#E4E9F0);transition:opacity .35s ease}
.lh-root.lh-off{opacity:0;visibility:hidden}
.lh-console{position:relative;box-sizing:border-box;height:252px;display:flex;flex-direction:column;gap:8px;padding:20px 14px 10px;background:rgba(10,13,18,.74);border:1px solid rgba(255,255,255,.12);border-radius:12px;backdrop-filter:blur(5px);-webkit-backdrop-filter:blur(5px);box-shadow:0 10px 30px rgba(0,0,0,.35);transition:border-color .6s,box-shadow .6s}
.lh-console.lh-term{border-color:rgba(240,165,0,.34);box-shadow:0 0 28px rgba(240,165,0,.10),0 10px 30px rgba(0,0,0,.35)}
.lh-console.lh-abort{border-color:rgba(248,81,73,.62);box-shadow:0 0 30px rgba(248,81,73,.26),0 10px 30px rgba(0,0,0,.35)}
.lh-top{flex:none;display:flex;align-items:center;gap:12px;min-height:24px}
.lh-phase{flex:none;width:152px;box-sizing:border-box;font:600 11px/1.2 var(--mono,monospace);letter-spacing:.14em;text-transform:uppercase;color:var(--amber,#F0A500);border-left:2px solid currentColor;padding-left:8px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.lh-phase.good{color:var(--nominal,#3FB950)}
.lh-phase.bad{color:var(--failed,#F85149)}
.lh-phase.dim{color:var(--text-2,#8B97A8)}
.lh-ann{flex:1;min-width:0;display:grid;grid-template-columns:repeat(7,minmax(0,1fr));gap:4px}
.lh-lamp{position:relative;height:24px;min-width:0;box-sizing:border-box;display:flex;align-items:center;justify-content:center;padding:0 3px;border-radius:4px;border:1px solid rgba(255,255,255,.08);background:var(--ground,#0A0D12);color:var(--text-3,#5A6675);font:600 9.5px/1 var(--mono,monospace);letter-spacing:.04em;text-transform:uppercase;white-space:nowrap;overflow:hidden;transition:background-color .25s,color .25s,box-shadow .25s,border-color .25s}
.lh-lamp.on.c-white{background:linear-gradient(180deg,#F4F9FF,#BCD8FF);color:#08203F;border-color:#D9ECFF;box-shadow:0 0 12px rgba(190,220,255,.6)}
.lh-lamp.on.c-blue{background:var(--data,#4A9EFF);color:#041A36;border-color:#8CC0FF;box-shadow:0 0 12px rgba(74,158,255,.7)}
.lh-lamp.on.c-green{background:var(--nominal,#3FB950);color:#03200C;border-color:#7BE08B;box-shadow:0 0 12px rgba(63,185,80,.65)}
.lh-lamp.on.c-amber{background:var(--amber,#F0A500);color:#2A1C00;border-color:#FFD166;box-shadow:0 0 12px rgba(240,165,0,.7)}
.lh-lamp.on.c-red{background:var(--failed,#F85149);color:#2A0606;border-color:#FF8C84;box-shadow:0 0 12px rgba(248,81,73,.75)}
.lh-lamp.blink.on{animation:lh-blink 1s linear infinite}
.lh-lamp.flash{animation:lh-flash .3s ease-in-out 4}
@keyframes lh-blink{0%,49%{opacity:1}50%,100%{opacity:.18}}
@keyframes lh-flash{0%,100%{filter:none}50%{filter:brightness(2.4) saturate(.6)}}
.lh-main{flex:1;min-height:0;display:grid;grid-template-columns:112px 176px minmax(0,1fr) 112px;gap:12px}
.lh-alt,.lh-ptr,.lh-thr{min-width:0;min-height:0;display:flex;flex-direction:column;gap:4px}
.lh-bars{flex:1;min-height:0;display:flex;gap:8px}
.lh-tape,.lh-sink,.lh-xwrap,.lh-prof{position:relative;overflow:hidden;border:1px solid var(--line,#232C3A);border-radius:6px;background:rgba(0,0,0,.28)}
.lh-tape{flex:1;min-width:0}
.lh-sink{width:18px;flex:none}
.lh-cv{position:absolute;inset:0;width:100%;height:100%;display:block}
.lh-tbox{position:absolute;left:3px;right:3px;top:50%;height:24px;transform:translateY(-50%);box-sizing:border-box;display:flex;align-items:center;justify-content:flex-end;padding:0 5px;border:1px solid var(--amber,#F0A500);border-radius:3px;background:rgba(10,13,18,.92);color:var(--amber,#F0A500);font:600 14px/1 var(--mono,monospace);font-variant-numeric:tabular-nums;white-space:nowrap;box-shadow:0 0 10px rgba(240,165,0,.25)}
.lh-cap{flex:none;display:flex;justify-content:space-between;gap:4px;font:600 9px/1 var(--sans,system-ui),sans-serif;letter-spacing:.1em;text-transform:uppercase;color:var(--text-2,#8B97A8);white-space:nowrap}
.lh-cap b{font:500 9.5px/1 var(--mono,monospace);letter-spacing:0;color:var(--text,#E4E9F0)}
.lh-xwrap{flex:1;min-height:0;background:radial-gradient(circle at 50% 50%,rgba(255,255,255,.035),rgba(0,0,0,.25) 72%)}
.lh-xrow{flex:none;display:flex;align-items:center;justify-content:space-between;gap:6px}
.lh-xnum{font:500 11px/1.35 var(--mono,monospace);color:var(--text-2,#8B97A8);white-space:nowrap;font-variant-numeric:tabular-nums}
.lh-xnum b{font-weight:500;color:var(--text,#E4E9F0)}
.lh-pitch{flex:none;display:flex;align-items:center;gap:4px}
.lh-pitch canvas{position:relative;display:block;width:40px;height:30px}
.lh-pl{display:flex;flex-direction:column;gap:2px;white-space:nowrap}
.lh-pl .k{font:600 8.5px/1 var(--sans,system-ui),sans-serif;letter-spacing:.12em;text-transform:uppercase;color:var(--text-2,#8B97A8)}
.lh-pl .v{font:500 11px/1 var(--mono,monospace);color:var(--text,#E4E9F0);font-variant-numeric:tabular-nums}
.lh-rd{min-width:0;min-height:0;display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);grid-auto-rows:minmax(0,1fr);column-gap:14px}
.lh-cell{min-width:0;display:flex;flex-direction:column;justify-content:center;border-bottom:1px solid rgba(255,255,255,.06);padding:0 2px}
.lh-cell .k{font:600 9px/1 var(--sans,system-ui),sans-serif;letter-spacing:.12em;text-transform:uppercase;color:var(--text-2,#8B97A8);white-space:nowrap}
.lh-cell .v{margin-top:3px;font:500 16px/1.1 var(--mono,monospace);color:var(--text,#E4E9F0);white-space:nowrap;overflow:hidden;font-variant-numeric:tabular-nums}
.lh-cell .v small{margin-left:4px;font-size:9.5px;font-weight:500;letter-spacing:.04em;color:var(--text-2,#8B97A8)}
.lh-cell.big .v{font-size:22px}
.lh-cell[data-cell="ground"]{grid-column:1/-1}
.lh-cell.warn .k,.lh-cell.warn .v{color:var(--failed,#F85149)}
.lh-thr{flex-direction:row;gap:10px}
.lh-col{flex:1;min-width:0;display:flex;flex-direction:column;align-items:center;gap:4px}
.lh-bar{position:relative;flex:1;min-height:0;width:100%;max-width:30px;box-sizing:border-box;border:1px solid var(--line,#232C3A);border-radius:4px;background:rgba(0,0,0,.35);overflow:hidden}
.lh-fill{position:absolute;left:0;right:0;bottom:0;height:0;background:var(--amber,#F0A500);box-shadow:0 0 10px rgba(240,165,0,.3)}
.lh-fill.g{background:var(--nominal,#3FB950);box-shadow:0 0 10px rgba(63,185,80,.3)}
.lh-fill.a{background:var(--amber,#F0A500)}
.lh-fill.r{background:var(--failed,#F85149);box-shadow:0 0 10px rgba(248,81,73,.4)}
.lh-tmin{position:absolute;left:-2px;right:-2px;bottom:0;height:2px;background:var(--text,#E4E9F0);opacity:.85}
.lh-col .k{font:600 9px/1 var(--sans,system-ui),sans-serif;letter-spacing:.12em;text-transform:uppercase;color:var(--text-2,#8B97A8)}
.lh-col .v{font:500 11px/1 var(--mono,monospace);color:var(--text,#E4E9F0);white-space:nowrap;font-variant-numeric:tabular-nums}
.lh-prof{flex:none;height:38px}
.lh-big{position:absolute;left:50%;bottom:calc(100% + 26px);transform:translateX(-50%);opacity:0;pointer-events:none;white-space:nowrap;font:700 clamp(28px,4vw,46px)/1 var(--mono,monospace);letter-spacing:.32em;color:var(--text,#E4E9F0);text-shadow:0 0 18px rgba(240,165,0,.55)}
.lh-big.on{animation:lh-big-in 4s ease forwards}
@keyframes lh-big-in{0%{opacity:0;transform:translate(-50%,6px) scale(.97)}8%{opacity:1;transform:translate(-50%,0) scale(1)}70%{opacity:1}100%{opacity:0}}
@media (max-width:1066px){
  .lh-root{width:min(600px,calc(100vw - 260px))}
  .lh-console{height:236px;padding:19px 12px 9px}
  .lh-main{grid-template-columns:84px 168px minmax(0,1fr) 84px;gap:10px}
  .lh-pitch canvas{width:34px}
  .lh-phase{width:124px;font-size:10px}
  .lh-lamp{font-size:8.5px;letter-spacing:0}
  .lh-cell .v{font-size:14px}
  .lh-cell.big .v{font-size:18px}
}
@media (max-width:819px){
  .lh-root{left:16px;right:16px;bottom:56px;transform:translate(var(--lh-dx,0px),var(--lh-dy,0px));width:auto}
}
@media (max-width:519px){
  .lh-console{height:auto;gap:8px}
  .lh-top{flex-wrap:wrap;gap:8px}
  .lh-phase{width:100%;box-sizing:border-box}
  .lh-ann{grid-template-columns:repeat(4,minmax(0,1fr))}
  .lh-main{flex:none;height:auto;grid-template-columns:76px minmax(0,1fr) 84px;grid-template-rows:150px auto;gap:10px}
  .lh-alt{grid-column:1;grid-row:1}
  .lh-ptr{grid-column:2;grid-row:1}
  .lh-thr{grid-column:3;grid-row:1;gap:6px}
  .lh-rd{grid-column:1/-1;grid-row:2;grid-auto-rows:auto;row-gap:2px}
  .lh-cell{min-height:30px}
  .lh-prof{height:38px}
}
.lh-root .lh-grip,.lh-root .lh-gbtn,.lh-root .lh-mini{pointer-events:auto}
.lh-grip{position:absolute;left:0;right:0;top:0;height:14px;box-sizing:border-box;display:flex;align-items:center;cursor:grab;touch-action:none;border-radius:12px 12px 0 0;background-image:radial-gradient(circle,rgba(228,233,240,.30) .8px,transparent 1.2px);background-size:6px 4px;background-position:center}
.lh-grip:active{cursor:grabbing}
.lh-gbtn{box-sizing:border-box;margin-left:auto;width:16px;height:11px;padding:0;display:flex;align-items:center;justify-content:center;border:1px solid rgba(255,255,255,.18);border-radius:3px;background:rgba(10,13,18,.85);color:var(--text-2,#8B97A8);font:700 10px/1 var(--mono,monospace);cursor:pointer}
.lh-gbtn:hover{color:var(--text,#E4E9F0);border-color:rgba(255,255,255,.35)}
.lh-mini{display:none;align-items:center;gap:8px;height:30px;box-sizing:border-box;padding:0 6px 0 10px;border-radius:15px;background:rgba(10,13,18,.74);border:1px solid rgba(255,255,255,.12);backdrop-filter:blur(5px);-webkit-backdrop-filter:blur(5px);cursor:grab;touch-action:none;white-space:nowrap;font:600 11px/1 var(--mono,monospace);font-variant-numeric:tabular-nums;color:var(--text,#E4E9F0)}
.lh-mini:active{cursor:grabbing}
.lh-mini .lh-mp{color:var(--amber,#F0A500);letter-spacing:.14em;text-transform:uppercase}
.lh-mini .lh-ms{color:var(--text-3,#5A6675)}
.lh-mini .lh-mv{color:var(--text-2,#8B97A8)}
.lh-root.lh-min{width:auto;left:50%;right:auto;bottom:56px;transform:translateX(-50%) translate(var(--lh-dx,0px),var(--lh-dy,0px))}
.lh-root.lh-min .lh-console{display:none}
.lh-root.lh-min .lh-mini{display:flex}
@media (prefers-reduced-motion:reduce){
  .lh-lamp.blink.on,.lh-lamp.flash,.lh-big.on{animation:none}
  .lh-big.on{opacity:1}
}
`;

const KEY_LANDER = 'ssjl.hud.lander';

function readState(key) {
  try {
    const o = JSON.parse(window.localStorage.getItem(key) || 'null');
    return o && typeof o === 'object' ? o : null;
  } catch (e) {
    return null;
  }
}

function writeState(key, v) {
  try {
    window.localStorage.setItem(key, JSON.stringify(v));
  } catch (e) {
    // storage blocked or full: the console still works, it just forgets
  }
}

// Pointer drag for a panel handle. While dragging, the handle holds the pointer
// and the event stops there, so the 3D camera underneath never sees the gesture.
// Buttons inside the handle keep their own clicks.
function bindDrag(handle, o) {
  let st = null;
  const onButton = e => !!(e.target && e.target.closest && e.target.closest('button'));
  const down = e => {
    if (e.button !== 0 || onButton(e)) return;
    e.stopPropagation();
    e.preventDefault();
    st = { id: e.pointerId, x: e.clientX, y: e.clientY, dx: o.getDx(), dy: o.getDy(), moved: false };
    try { handle.setPointerCapture(e.pointerId); } catch (err) { /* not capturable */ }
  };
  const move = e => {
    e.stopPropagation();
    if (!st || e.pointerId !== st.id) return;
    e.preventDefault();
    const ddx = e.clientX - st.x, ddy = e.clientY - st.y;
    if (!st.moved && Math.hypot(ddx, ddy) < 4) return;
    st.moved = true;
    o.onDrag(st.dx + ddx, st.dy + ddy);
  };
  const up = e => {
    e.stopPropagation();
    if (!st || e.pointerId !== st.id) return;
    const moved = st.moved;
    st = null;
    try { handle.releasePointerCapture(e.pointerId); } catch (err) { /* already released */ }
    if (moved) { if (o.onDrop) o.onDrop(); } else if (o.onTap) o.onTap();
  };
  handle.addEventListener('pointerdown', down);
  handle.addEventListener('pointermove', move);
  handle.addEventListener('pointerup', up);
  handle.addEventListener('pointercancel', up);
  return () => {
    handle.removeEventListener('pointerdown', down);
    handle.removeEventListener('pointermove', move);
    handle.removeEventListener('pointerup', up);
    handle.removeEventListener('pointercancel', up);
  };
}

let styleEl = null;
let styleRefs = 0;

function acquireStyle() {
  if (!styleEl) {
    styleEl = document.createElement('style');
    styleEl.id = 'lh-style';
    styleEl.textContent = CSS;
    (document.head || document.documentElement).appendChild(styleEl);
  }
  styleRefs++;
}

function releaseStyle() {
  if (styleRefs > 0) styleRefs--;
  if (styleRefs === 0 && styleEl) {
    styleEl.remove();
    styleEl = null;
  }
}

const isNum = v => typeof v === 'number' && Number.isFinite(v);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const pad2 = n => (n < 10 ? '0' : '') + n;

// Signed fixed-point with a true minus sign. Tiny values print as zero, not -0.
function sgn(x, d = 1, plus = false) {
  if (!isNum(x)) return DASH;
  const r = Math.abs(x) < 0.5 * Math.pow(10, -d) ? 0 : x;
  const s = Math.abs(r).toFixed(d);
  if (r < 0) return MINUS + s;
  return plus && r > 0 ? '+' + s : s;
}

function fmtAlt(a) {
  if (!isNum(a)) return [DASH, ''];
  if (Math.abs(a) >= 10000) return [sgn(a / 1000, 2), 'km'];
  if (Math.abs(a) >= 100) return [sgn(a, 0), 'm'];
  return [sgn(a, 1), 'm'];
}

function fmtDist(d) {
  if (!isNum(d)) return [DASH, ''];
  if (Math.abs(d) >= 1000) return [sgn(d / 1000, 2), 'km'];
  return [sgn(d, 0), 'm'];
}

function fmtTime(t) {
  if (!isNum(t)) return DASH;
  const s = Math.max(0, Math.round(t));
  return pad2(Math.floor(s / 60)) + ':' + pad2(s % 60);
}

// Tape value in the scale's own unit: metres up to 2 km full scale, km above.
function fmtTape(a, M) {
  const u = M >= 2000 ? 1000 : 1;
  const d = M <= 20 ? 1 : M <= 200 ? 0 : M <= 2000 ? 2 : 1;
  return sgn(a / u, d);
}

const fmtTick = x => (Number.isInteger(x) ? String(x) : x.toFixed(1));

// Range selection with hysteresis: step up when the value reaches ~98 % of the
// current full scale, step down only when it falls well inside the lower one,
// so a value hovering at a boundary does not make the scale flicker.
function pickIdx(cur, value, steps) {
  if (!isNum(value)) return cur;
  const v = Math.abs(value);
  let i = cur;
  while (i < steps.length - 1 && v > steps[i] * 0.98) i++;
  while (i > 0 && v < steps[i - 1] * 0.8) i--;
  return i;
}

// Safe sink-rate limit for the current altitude (m/s).
function sinkLimit(alt) {
  if (!isNum(alt)) return null;
  return alt < 30 ? 3 : alt / 10 + 3;
}

function cellHtml(key, label, big) {
  return `<div class="lh-cell${big ? ' big' : ''}" data-cell="${key}"><div class="k">${label}</div>` +
    `<div class="v"><span data-r="${key}">${DASH}</span><small data-r="${key}U"></small></div></div>`;
}

export function createLanderHud(container) {
  const host = container || document.body;
  acquireStyle();

  const root = document.createElement('div');
  root.className = 'lh-root';
  root.setAttribute('aria-hidden', 'true');
  root.innerHTML = `
    <div class="lh-big" data-r="big"></div>
    <div class="lh-console" data-r="console"><div class="lh-grip" data-r="grip"><button class="lh-gbtn" type="button" data-r="minBtn" aria-label="Minimise descent console">–</button></div>
      <div class="lh-top">
        <div class="lh-phase" data-r="phase">${DASH}</div>
        <div class="lh-ann">${LAMPS.map(([k, label, cls]) =>
          `<div class="lh-lamp ${cls}" data-lamp="${k}">${label}</div>`).join('')}</div>
      </div>
      <div class="lh-main">
        <section class="lh-alt">
          <div class="lh-bars">
            <div class="lh-tape"><canvas class="lh-cv" data-cv="tape"></canvas><div class="lh-tbox" data-r="tbox">${DASH}</div></div>
            <div class="lh-sink"><canvas class="lh-cv" data-cv="sink"></canvas></div>
          </div>
          <div class="lh-cap"><span>ALT <b data-r="tunit">M</b></span><span>VS <b data-r="sinkR">±20</b></span></div>
        </section>
        <section class="lh-ptr">
          <div class="lh-xwrap"><canvas class="lh-cv" data-cv="xp"></canvas></div>
          <div class="lh-xrow">
            <div class="lh-xnum">
              <div>FWD <b data-r="fwd">${DASH}</b> m/s</div>
              <div>LAT <b data-r="lat">${DASH}</b> m/s</div>
            </div>
            <div class="lh-pitch"><canvas data-cv="pitch"></canvas>
              <div class="lh-pl"><span class="k">Pitch</span><span class="v" data-r="pitch">${DASH}</span></div>
            </div>
          </div>
        </section>
        <section class="lh-rd">${CELLS.map(([k, l, b]) => cellHtml(k, l, b)).join('')}</section>
        <section class="lh-thr">
          <div class="lh-col">
            <div class="lh-bar"><div class="lh-fill" data-r="thrF"></div><div class="lh-tmin" data-r="thrT"></div></div>
            <div class="k">THR</div><div class="v" data-r="thr">${DASH}</div>
          </div>
          <div class="lh-col">
            <div class="lh-bar"><div class="lh-fill" data-r="propF"></div></div>
            <div class="k">PROP</div><div class="v" data-r="prop">${DASH}</div>
          </div>
        </section>
      </div>
      <div class="lh-prof"><canvas class="lh-cv" data-cv="prof"></canvas></div>
    </div>
    <div class="lh-mini" data-r="mini"><span class="lh-mp" data-r="miniPhase">${DASH}</span><span class="lh-mv" data-r="miniAlt">${DASH}</span><span class="lh-ms">·</span><span class="lh-mv" data-r="miniVs">${DASH}</span><span class="lh-ms">·</span><span class="lh-mv" data-r="miniProp">${DASH}</span><button class="lh-gbtn" type="button" data-r="plusBtn" aria-label="Restore descent console">+</button></div>`;
  host.appendChild(root);

  // ---- cached references ----------------------------------------------
  const E = {};
  root.querySelectorAll('[data-r]').forEach(el => { E[el.dataset.r] = el; });
  const consoleEl = root.querySelector('.lh-console');
  const lampEls = {};
  root.querySelectorAll('[data-lamp]').forEach(el => { lampEls[el.dataset.lamp] = el; });
  const cellEls = {};
  root.querySelectorAll('[data-cell]').forEach(el => { cellEls[el.dataset.cell] = el; });
  const cvs = {};
  root.querySelectorAll('[data-cv]').forEach(el => {
    cvs[el.dataset.cv] = { el, ctx: el.getContext('2d'), w: 0, h: 0, ok: false, stale: true };
  });

  // ---- state ------------------------------------------------------------
  const last = new Map();      // key -> last written text / style / class
  const lampOn = {};
  let S = { alt: null, vs: null, vfwd: null, vlat: null, pitch: null, profile: null, pi: null };
  let pal = null;
  let visible = true;
  let minimised = false;
  const pos = { dx: 0, dy: 0 };
  let destroyed = false;
  const tape = { idx: 0 }, sink = { idx: 0 }, xp = { idx: 0 };
  let dirty = new Set();
  let raf = 0;
  let lastDraw = -Infinity;
  let inited = false;
  let lastPhase = null;
  let lastLanded = false;
  let bigTimer = 0;
  const hasRO = typeof ResizeObserver !== 'undefined';
  let ro = null;

  const put = (el, key, text) => {
    if (last.get(key) !== text) { last.set(key, text); el.textContent = text; }
  };
  const setStyle = (el, key, prop, value) => {
    if (last.get(key) !== value) { last.set(key, value); el.style[prop] = value; }
  };
  const setCls = (el, key, value) => {
    if (last.get(key) !== value) { last.set(key, value); el.className = value; }
  };

  function readPalette() {
    const cs = getComputedStyle(root);
    const g = (name, fb) => cs.getPropertyValue(name).trim() || fb;
    pal = {
      amber: g('--amber', '#F0A500'),
      text: g('--text', '#E4E9F0'),
      text2: g('--text-2', '#8B97A8'),
      text3: g('--text-3', '#5A6675'),
      nominal: g('--nominal', '#3FB950'),
      failed: g('--failed', '#F85149'),
      data: g('--data', '#4A9EFF'),
      mono: g('--mono', 'monospace'),
    };
  }

  function sizeCv(c) {
    const w = c.el.clientWidth, h = c.el.clientHeight;
    if (!w || !h) { c.ok = false; return; }
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const W = Math.round(w * dpr), H = Math.round(h * dpr);
    if (c.el.width !== W) c.el.width = W;
    if (c.el.height !== H) c.el.height = H;
    c.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    c.w = w; c.h = h; c.ok = true; c.stale = false;
  }

  // ---- drawing ----------------------------------------------------------
  function drawTape(c) {
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    const a = S.alt;
    tape.idx = pickIdx(tape.idx, a, TAPE_STEPS);
    const M = TAPE_STEPS[tape.idx];
    const u = M >= 2000 ? 1000 : 1;
    const top = 8, bot = h - 8, span = bot - top;
    const Y = v => bot - (clamp(v, 0, M) / M) * span;

    ctx.font = `9px ${pal.mono}`;
    ctx.textBaseline = 'middle';
    ctx.textAlign = 'left';
    for (let k = 0; k <= 20; k++) {
      const v = (k * M) / 20;
      const y = Math.round(Y(v)) + 0.5;
      const major = k % 5 === 0;
      ctx.strokeStyle = major ? pal.text2 : pal.text3;
      ctx.globalAlpha = major ? 0.85 : 0.45;
      ctx.beginPath();
      ctx.moveTo(w - 1, y);
      ctx.lineTo(w - (major ? 9 : 5), y);
      ctx.stroke();
      if (major) {
        ctx.globalAlpha = 0.8;
        ctx.fillStyle = pal.text2;
        ctx.fillText(fmtTick(v / u), 4, y);
      }
    }
    ctx.globalAlpha = 1;

    let boxY = null;
    if (isNum(a)) {
      const y = Y(a);
      const grad = ctx.createLinearGradient(0, y, 0, bot);
      grad.addColorStop(0, 'rgba(240,165,0,0.22)');
      grad.addColorStop(1, 'rgba(240,165,0,0)');
      ctx.fillStyle = grad;
      ctx.fillRect(0, y, w, bot - y);
      ctx.strokeStyle = pal.amber;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, Math.round(y) + 0.5);
      ctx.lineTo(w, Math.round(y) + 0.5);
      ctx.stroke();
      boxY = clamp(y, 14, h - 14);
    }
    if (boxY === null) {
      put(E.tbox, 'tbox', DASH);
    } else {
      put(E.tbox, 'tbox', fmtTape(a, M));
      setStyle(E.tbox, 'tbox.top', 'top', boxY.toFixed(1) + 'px');
    }
    put(E.tunit, 'tunit', M >= 2000 ? 'KM' : 'M');
  }

  function drawSink(c) {
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    const vs = S.vs;
    sink.idx = pickIdx(sink.idx, vs, VS_STEPS);
    const R = VS_STEPS[sink.idx];
    const mid = h / 2, half = mid - 6;
    const Y = v => mid - clamp(v / R, -1, 1) * half;

    ctx.strokeStyle = 'rgba(255,255,255,0.18)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(1, Math.round(mid) + 0.5);
    ctx.lineTo(w - 1, Math.round(mid) + 0.5);
    for (const f of [-1, -0.5, 0.5, 1]) {
      const y = Math.round(mid - f * half) + 0.5;
      ctx.moveTo(1, y); ctx.lineTo(5, y);
      ctx.moveTo(w - 1, y); ctx.lineTo(w - 5, y);
    }
    ctx.stroke();

    const lim = sinkLimit(S.alt);
    if (lim !== null && lim < R) {
      ctx.save();
      ctx.setLineDash([3, 2]);
      ctx.strokeStyle = pal.failed;
      ctx.globalAlpha = 0.85;
      ctx.beginPath();
      ctx.moveTo(0, Math.round(Y(lim)) + 0.5); ctx.lineTo(w, Math.round(Y(lim)) + 0.5);
      ctx.moveTo(0, Math.round(Y(-lim)) + 0.5); ctx.lineTo(w, Math.round(Y(-lim)) + 0.5);
      ctx.stroke();
      ctx.restore();
    }

    if (isNum(vs)) {
      const y = Y(vs);
      const unsafe = lim !== null && vs < 0 && -vs > lim;
      ctx.fillStyle = unsafe ? pal.failed : vs < 0 ? pal.amber : pal.data;
      if (unsafe) { ctx.shadowColor = pal.failed; ctx.shadowBlur = 8; }
      ctx.fillRect(4, Math.min(mid, y), w - 8, Math.max(1, Math.abs(y - mid)));
      ctx.shadowBlur = 0;
    }
    put(E.sinkR, 'sinkR', '±' + R);
  }

  function drawXP(c) {
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    const vf = S.vfwd, vl = S.vlat;
    const mag = (vf === null && vl === null) ? null : Math.max(Math.abs(vf || 0), Math.abs(vl || 0));
    xp.idx = pickIdx(xp.idx, mag, XP_STEPS);
    const Sc = XP_STEPS[xp.idx];
    const cx = w / 2, cy = h / 2;
    const Rr = Math.max(12, Math.min(w, h) / 2 - 14);

    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(255,255,255,0.16)';
    ctx.beginPath();
    ctx.moveTo(cx - Rr - 4, cy + 0.5); ctx.lineTo(cx + Rr + 4, cy + 0.5);
    ctx.moveTo(cx + 0.5, cy - Rr - 4); ctx.lineTo(cx + 0.5, cy + Rr + 4);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(cx, cy, Rr, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([2, 3]);
    ctx.beginPath();
    ctx.arc(cx, cy, Rr / 2, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.font = `9px ${pal.mono}`;
    ctx.fillStyle = pal.text3;
    ctx.textAlign = 'center'; ctx.textBaseline = 'bottom';
    ctx.fillText('FWD', cx, cy - Rr - 5);
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillText('LAT', cx + Rr + 7, cy);
    ctx.textAlign = 'left'; ctx.textBaseline = 'bottom';
    ctx.fillText('±' + Sc + ' m/s', 4, h - 3);

    if (mag === null) {
      ctx.fillStyle = pal.text3;
      ctx.beginPath(); ctx.arc(cx, cy, 2, 0, Math.PI * 2); ctx.fill();
      return;
    }
    const fx = clamp((vl || 0) / Sc, -1, 1) * Rr;
    const fy = -clamp((vf || 0) / Sc, -1, 1) * Rr;
    ctx.lineCap = 'round';
    ctx.lineWidth = 2.5;
    ctx.strokeStyle = pal.amber;           // forward: along the vertical axis
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy + fy); ctx.stroke();
    ctx.strokeStyle = pal.data;            // lateral: along the horizontal axis
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + fx, cy); ctx.stroke();
    ctx.lineCap = 'butt';
    ctx.shadowColor = pal.amber;
    ctx.shadowBlur = 10;
    ctx.fillStyle = pal.text;
    ctx.beginPath(); ctx.arc(cx + fx, cy + fy, 3.5, 0, Math.PI * 2); ctx.fill();
    ctx.shadowBlur = 0;
  }

  function drawPitch(c) {
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    const cx = w / 2, cy = h - 3;
    const r = Math.min(w / 2 - 3, h - 6);
    const P = (d, rr) => { const a = d * Math.PI / 180; return [cx + rr * Math.sin(a), cy - rr * Math.cos(a)]; };

    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(255,255,255,0.24)';
    ctx.beginPath();
    for (let d = -60; d <= 60; d += 2) {
      const [x, y] = P(d, r);
      if (d === -60) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    for (const d of [-60, -30, 0, 30, 60]) {
      const [x0, y0] = P(d, r - 1), [x1, y1] = P(d, r - 4);
      ctx.moveTo(x0, y0); ctx.lineTo(x1, y1);
    }
    ctx.stroke();

    if (S.pitch !== null) {
      const [x, y] = P(clamp(S.pitch, -60, 60), r - 1);
      ctx.strokeStyle = pal.amber;
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
      ctx.fillStyle = pal.amber;
      ctx.beginPath(); ctx.arc(x, y, 1.8, 0, Math.PI * 2); ctx.fill();
    }
    ctx.fillStyle = pal.text2;
    ctx.beginPath(); ctx.arc(cx, cy, 1.6, 0, Math.PI * 2); ctx.fill();
  }

  function drawProf(c) {
    const { ctx, w, h } = c;
    ctx.clearRect(0, 0, w, h);
    const prof = S.profile;
    const pl = 36, pr = 40, pt = 5, pb = 5;
    const pw = w - pl - pr, ph = h - pt - pb, base = pt + ph;
    ctx.font = `8.5px ${pal.mono}`;

    const xs = prof && Array.isArray(prof.x) ? prof.x : null;
    const hs = prof && Array.isArray(prof.h) ? prof.h : null;
    const n = xs && hs ? Math.min(xs.length, hs.length) : 0;
    if (n < 2) {
      ctx.fillStyle = pal.text3;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('NO PROFILE', w / 2, h / 2);
      return;
    }

    let xmax = 0, xmin = 0, hmax = 0;
    for (let k = 0; k < n; k++) {
      const xv = xs[k], hv = hs[k];
      if (isNum(xv)) { if (xv > xmax) xmax = xv; if (xv < xmin) xmin = xv; }
      if (isNum(hv) && hv > hmax) hmax = hv;
    }
    if (xmax <= xmin) xmax = xmin + 1;
    if (hmax <= 0) hmax = 1;
    // Downrange-to-go on X, reversed so the site (x = 0) is at the right.
    const X = xv => pl + ((xmax - xv) / (xmax - xmin)) * pw;
    const Y = hv => base - Math.sqrt(clamp(isNum(hv) ? hv : 0, 0, hmax) / hmax) * ph;
    const P = k => [X(xs[k]), Y(hs[k])];
    const at = f => {
      const k0 = Math.floor(f), k1 = Math.min(k0 + 1, n - 1), t = f - k0;
      const a = P(k0), b = P(k1);
      return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
    };

    // baseline and axis labels
    ctx.strokeStyle = 'rgba(255,255,255,0.14)';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(pl, base + 0.5); ctx.lineTo(pl + pw, base + 0.5); ctx.stroke();
    ctx.fillStyle = pal.text2;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'top';
    ctx.fillText(hmax >= 1000 ? (hmax / 1000).toFixed(hmax >= 10000 ? 0 : 1) + ' km' : Math.round(hmax) + ' m', pl - 5, pt - 1);
    ctx.textBaseline = 'bottom';
    ctx.fillText('0', pl - 5, base + 2);

    // site marker
    const sx = Math.round(X(0)) + 0.5;
    ctx.save();
    ctx.setLineDash([2, 2]);
    ctx.strokeStyle = pal.data;
    ctx.beginPath(); ctx.moveTo(sx, pt); ctx.lineTo(sx, base); ctx.stroke();
    ctx.restore();
    ctx.fillStyle = pal.data;
    ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
    ctx.fillText('SITE', sx + 4, pt + 6);

    const f = isNum(prof.i) ? clamp(prof.i, 0, n - 1) : null;
    if (f === null) {
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = pal.text3;
      ctx.beginPath(); ctx.moveTo(P(0)[0], P(0)[1]);
      for (let k = 1; k < n; k++) { const q = P(k); ctx.lineTo(q[0], q[1]); }
      ctx.stroke();
      ctx.setLineDash([]);
    } else {
      const kf = Math.floor(f);
      const cur = at(f);

      // flown fill
      const g = ctx.createLinearGradient(0, pt, 0, base);
      g.addColorStop(0, 'rgba(240,165,0,0.26)');
      g.addColorStop(1, 'rgba(240,165,0,0)');
      ctx.beginPath();
      ctx.moveTo(P(0)[0], base);
      for (let k = 0; k <= kf; k++) { const q = P(k); ctx.lineTo(q[0], q[1]); }
      ctx.lineTo(cur[0], cur[1]);
      ctx.lineTo(cur[0], base);
      ctx.closePath();
      ctx.fillStyle = g;
      ctx.fill();

      // remaining path, dashed and dim
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = pal.text3;
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(cur[0], cur[1]);
      for (let k = kf + 1; k < n; k++) { const q = P(k); ctx.lineTo(q[0], q[1]); }
      ctx.stroke();
      ctx.setLineDash([]);

      // flown path, solid amber
      ctx.strokeStyle = pal.amber;
      ctx.lineWidth = 1.8;
      ctx.beginPath(); ctx.moveTo(P(0)[0], P(0)[1]);
      for (let k = 1; k <= kf; k++) { const q = P(k); ctx.lineTo(q[0], q[1]); }
      ctx.lineTo(cur[0], cur[1]);
      ctx.stroke();

      // current position with glow
      ctx.save();
      ctx.shadowColor = pal.amber;
      ctx.shadowBlur = 10;
      ctx.fillStyle = pal.text;
      ctx.beginPath(); ctx.arc(cur[0], cur[1], 2.8, 0, Math.PI * 2); ctx.fill();
      ctx.restore();
      ctx.strokeStyle = pal.amber;
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(cur[0], cur[1], 5.5, 0, Math.PI * 2); ctx.stroke();
    }

    // high-gate flag at gateI
    if (isNum(prof.gateI)) {
      const gi = clamp(Math.round(prof.gateI), 0, n - 1);
      const [gx, gy] = P(gi);
      const gxr = Math.round(gx) + 0.5;
      ctx.strokeStyle = pal.text;
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(gxr, gy); ctx.lineTo(gxr, pt + 1); ctx.stroke();
      ctx.fillStyle = pal.text;
      ctx.beginPath(); ctx.moveTo(gxr, pt + 1); ctx.lineTo(gxr + 6, pt + 4); ctx.lineTo(gxr, pt + 7); ctx.closePath(); ctx.fill();
      ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      ctx.fillText('HG', gxr + 8, pt + 4);
    }
  }

  const DRAW = { tape: drawTape, sink: drawSink, xp: drawXP, pitch: drawPitch, prof: drawProf };

  // ---- scheduling -------------------------------------------------------
  function schedule() {
    if (!raf && !destroyed && visible && !minimised) raf = requestAnimationFrame(frame);
  }

  function markDirty(names) {
    for (const n of names) dirty.add(n);
    schedule();
  }

  function markAll() {
    markDirty(Object.keys(cvs));
  }

  function frame(now) {
    raf = 0;
    if (destroyed || !visible || dirty.size === 0) return;
    if (now - lastDraw < FRAME_MS) { schedule(); return; }
    lastDraw = now;
    const names = dirty;
    dirty = new Set();
    for (const name of names) {
      const c = cvs[name];
      if (!c) continue;
      if (!hasRO || c.stale) sizeCv(c);
      if (c.ok) DRAW[name](c);
    }
  }

  if (hasRO) {
    ro = new ResizeObserver(() => {
      for (const k of Object.keys(cvs)) cvs[k].stale = true;
      readPalette();
      markAll();
    });
    for (const k of Object.keys(cvs)) ro.observe(cvs[k].el);
  }
  readPalette();

  // ---- big call-out -----------------------------------------------------
  function showBig(text) {
    const el = E.big;
    el.textContent = text;
    el.classList.remove('on');
    void el.offsetWidth;
    el.classList.add('on');
    clearTimeout(bigTimer);
    bigTimer = setTimeout(() => el.classList.remove('on'), BIG_MS);
  }

  // ---- public -----------------------------------------------------------
  function update(state) {
    if (destroyed) return;
    const s = state || {};
    const num = k => (isNum(s[k]) ? s[k] : null);
    const alt = num('alt'), vs = num('vs'), vfwd = num('vfwd'), vlat = num('vlat');
    const pitch = num('pitch'), thr = num('throttle'), thrMin = num('throttleMin');
    const propKg = num('propKg'), prop0 = num('prop0Kg');
    const toSite = num('toSite'), navErr = num('navErr'), ground = num('ground'), tTD = num('tToTouchdown');
    const phase = typeof s.phase === 'string' && s.phase ? s.phase : null;
    const L = s.lights || {};
    const landed = L.landed === true;
    const prof = s.profile || null;
    const pi = prof && isNum(prof.i) ? prof.i : null;

    // canvas inputs and what they invalidate
    const dirtyNow = [];
    if (alt !== S.alt) dirtyNow.push('tape', 'sink');
    if (vs !== S.vs) dirtyNow.push('sink');
    if (vfwd !== S.vfwd || vlat !== S.vlat) dirtyNow.push('xp');
    if (pitch !== S.pitch) dirtyNow.push('pitch');
    if (prof !== S.profile || pi !== S.pi) dirtyNow.push('prof');
    S = { alt, vs, vfwd, vlat, pitch, profile: prof, pi };

    // phase banner and console mood
    put(E.phase, 'phase', phase || DASH);
    setCls(E.phase, 'phase.c', 'lh-phase' + (phase === 'ABORT' ? ' bad'
      : (phase === 'TOUCHDOWN' || phase === 'ON SURFACE') ? ' good' : phase ? '' : ' dim'));
    const term = phase === 'TERMINAL DESCENT';
    const abort = phase === 'ABORT' || L.abort === true;
    setCls(consoleEl, 'console', 'lh-console' + (term ? ' lh-term' : '') + (abort ? ' lh-abort' : ''));

    // readouts
    const [an, au] = fmtAlt(alt);
    put(E.alt, 'alt', an); put(E.altU, 'altU', au);
    put(E.vs, 'vs', vs === null ? DASH : sgn(vs, 1, true));
    put(E.vsU, 'vsU', vs === null ? '' : 'm/s');
    const lim = sinkLimit(alt);
    const unsafe = lim !== null && vs !== null && vs < 0 && -vs > lim;
    setCls(cellEls.vs, 'cell.vs', 'lh-cell big' + (unsafe ? ' warn' : ''));

    const hv = (vfwd === null && vlat === null) ? null : Math.hypot(vfwd || 0, vlat || 0);
    put(E.hvel, 'hvel', hv === null ? DASH : hv.toFixed(1));
    put(E.hvelU, 'hvelU', hv === null ? '' : 'm/s');

    const [dt, du] = fmtDist(toSite);
    put(E.tosite, 'tosite', dt); put(E.tositeU, 'tositeU', du);
    put(E.ttd, 'ttd', fmtTime(tTD));
    put(E.naverr, 'naverr', navErr === null ? DASH : navErr < 10 ? navErr.toFixed(1) : String(Math.round(navErr)));
    put(E.naverrU, 'naverrU', navErr === null ? '' : 'm');
    put(E.ground, 'ground', ground === null ? DASH : sgn(ground, 0, true));
    put(E.groundU, 'groundU', ground === null ? '' : 'm');
    put(E.fwd, 'fwd', sgn(vfwd, 1));
    put(E.lat, 'lat', sgn(vlat, 1));
    put(E.pitch, 'pitch', pitch === null ? DASH : sgn(pitch, 1, true) + '°');

    // throttle and propellant
    const th = thr === null ? 0 : clamp(thr, 0, 1);
    put(E.thr, 'thr', thr === null ? DASH : Math.round(th * 100) + '%');
    setStyle(E.thrF, 'thrF', 'height', (th * 100).toFixed(1) + '%');
    setStyle(E.thrT, 'thrT.d', 'display', thrMin === null ? 'none' : '');
    if (thrMin !== null) setStyle(E.thrT, 'thrT', 'bottom', (clamp(thrMin, 0, 1) * 100).toFixed(1) + '%');
    const pf = (propKg !== null && prop0 !== null && prop0 > 0) ? clamp(propKg / prop0, 0, 1) : null;
    setStyle(E.propF, 'propF', 'height', ((pf === null ? 0 : pf) * 100).toFixed(1) + '%');
    setCls(E.propF, 'propF.c', 'lh-fill' + (pf === null ? '' : pf < 0.08 ? ' r' : pf < 0.2 ? ' a' : ' g'));
    put(E.prop, 'prop', propKg === null ? DASH : Math.round(propKg) + ' kg');
    put(E.miniPhase, 'miniPhase', phase || DASH);
    put(E.miniAlt, 'miniAlt', 'ALT ' + an + (au ? ' ' + au : ''));
    put(E.miniVs, 'miniVs', 'VS ' + (vs === null ? DASH : sgn(vs, 1, true)));
    put(E.miniProp, 'miniProp', 'PROP ' + (pf === null ? DASH : Math.round(pf * 100) + '%'));

    // annunciators
    for (const [key] of LAMPS) {
      const on = L[key] === true;
      if (lampOn[key] === on) continue;
      const el = lampEls[key];
      lampOn[key] = on;
      el.classList.toggle('on', on);
      if (on && key === 'contact') {
        el.classList.remove('flash');
        void el.offsetWidth;
        el.classList.add('flash');
      }
    }

    // touchdown call-out: on entering TOUCHDOWN, or the landed lamp rising.
    // The first update only records the state, so a page opened mid-descent
    // does not replay the call-out.
    if (!inited) {
      inited = true;
    } else if ((phase === 'TOUCHDOWN' && lastPhase !== 'TOUCHDOWN') || (landed && !lastLanded)) {
      showBig('TOUCHDOWN');
    }
    lastPhase = phase;
    lastLanded = landed;

    if (dirtyNow.length) markDirty(dirtyNow);
  }

  function setVisible(on) {
    if (destroyed) return;
    visible = !!on;
    root.classList.toggle('lh-off', !visible);
    if (visible) {
      readPalette();
      markAll();
      clampPos();
    }
  }

  function destroy() {
    if (destroyed) return;
    destroyed = true;
    if (raf) cancelAnimationFrame(raf);
    raf = 0;
    clearTimeout(bigTimer);
    if (ro) ro.disconnect();
    window.removeEventListener('resize', onResize);
    for (const off of disposers) off();
    E.grip.removeEventListener('dblclick', onDock);
    root.remove();
    releaseStyle();
  }

  // ---- position, minimise, persistence -----------------------------------
  const applyPos = () => {
    root.style.setProperty('--lh-dx', pos.dx + 'px');
    root.style.setProperty('--lh-dy', pos.dy + 'px');
  };

  // Keep the whole console inside the viewport; measures the rendered box, so
  // it works for the pill too, and does nothing while hidden or not laid out.
  function clampPos() {
    if (destroyed) return;
    applyPos();
    if (!visible) return;
    const r = root.getBoundingClientRect();
    if (!r.width || !r.height) return;
    const L = Math.max(0, Math.min(r.left, window.innerWidth - r.width));
    const T = Math.max(0, Math.min(r.top, window.innerHeight - r.height));
    pos.dx += L - r.left;
    pos.dy += T - r.top;
    applyPos();
  }

  function save() {
    writeState(KEY_LANDER, { dx: pos.dx, dy: pos.dy, min: minimised });
  }

  function setMinimized(b) {
    if (destroyed) return;
    const on = !!b;
    if (on !== minimised) {
      minimised = on;
      root.classList.toggle('lh-min', on);
      clampPos();
      if (!on) markAll();
    }
    save();
  }

  function resetPosition() {
    if (destroyed) return;
    pos.dx = 0;
    pos.dy = 0;
    clampPos();
    save();
  }

  const dragOpts = {
    getDx: () => pos.dx,
    getDy: () => pos.dy,
    onDrag: (x, y) => { pos.dx = x; pos.dy = y; clampPos(); },
    onDrop: save,
  };
  const disposers = [
    bindDrag(E.grip, dragOpts),
    bindDrag(E.mini, Object.assign({}, dragOpts, { onTap: () => setMinimized(false) })),
  ];
  const onDock = e => { if (e.target !== E.minBtn) resetPosition(); };
  E.grip.addEventListener('dblclick', onDock);
  E.minBtn.addEventListener('click', () => setMinimized(true));
  E.plusBtn.addEventListener('click', () => setMinimized(false));
  const onResize = () => clampPos();
  window.addEventListener('resize', onResize);

  const saved = readState(KEY_LANDER);
  if (saved) {
    if (isNum(saved.dx)) pos.dx = saved.dx;
    if (isNum(saved.dy)) pos.dy = saved.dy;
    minimised = saved.min === true;
    root.classList.toggle('lh-min', minimised);
  }
  clampPos();

  markAll();
  return {
    update,
    setVisible,
    destroy,
    toggleMinimized: () => setMinimized(!minimised),
    setMinimized,
    isMinimized: () => minimised,
    resetPosition,
  };
}
