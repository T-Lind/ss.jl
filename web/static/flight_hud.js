// flight_hud.js - flight instruments for the 3D launch view.
//
//   Export: createFlightHud(container) -> { update, setVisible, setMode, destroy }
//   const hud = createFlightHud(document.body);
//   hud.update(state);          // every frame; any field may be null
//
// Self-contained: no imports, one injected <style> (classes prefixed fh-),
// DOM built under a fixed, pointer-transparent overlay. The attitude ball is
// rendered per pixel into an offscreen canvas and overlaid with labels and
// markers. Colours come from tokens.css when present, with fallbacks.

const STYLE_ID = 'fh-style';
const DEG = 180 / Math.PI;
const RAD = Math.PI / 180;
const TAU = Math.PI * 2;
const FRAME_MS = 1000 / 30;
const COMPACT_BELOW_PX = 700;
const FULL_D = 172;
const COMPACT_D = 120;
const MAX_CARDS = 6;
const LEAVE_MS = 520;

const PRO_COLOR = '#d4f04a';
const LBL = 'rgba(228,233,240,0.92)';
const LBL2 = 'rgba(200,210,222,0.78)';
const HALO = 'rgba(8,11,16,0.85)';
const LABEL_FONT = '600 10px ui-monospace, Consolas, "SF Mono", Menlo, monospace';
const SKY_HZ = [61, 134, 216], SKY_ZN = [27, 75, 140];
const GND_HZ = [154, 101, 52], GND_NR = [79, 50, 24];
const HEAD_LABELS = [
  [0, 'N'], [30, '30'], [60, '60'], [90, 'E'], [120, '120'], [150, '150'],
  [180, 'S'], [210, '210'], [240, '240'], [270, 'W'], [300, '300'], [330, '330'],
];
// Fixed vehicle reference: a W, in units of the ball radius.
const W_PTS = [[-0.36, -0.09], [-0.18, 0.07], [0, -0.09], [0.18, 0.07], [0.36, -0.09]];

// Bell, 9 wide by 13 tall, rim at the top.
const BELL = 'M2.5 0H6.5L8 7C8.5 9 9 11 9 13H0C0 11 0.5 9 1 7Z';
const GLYPH = {
  core: `<g transform="translate(5 2.2) scale(1.2)"><path d="${BELL}"/></g><path d="M3.6 18.8H13.4"/>`,
  boosters: `<g transform="translate(0.8 4) scale(0.85)"><path d="${BELL}"/></g>`
    + `<g transform="translate(12.2 4) scale(0.85)"><path d="${BELL}"/></g>`
    + '<path d="M1 17.6H8.6M13.4 17.6H21"/>',
  payload: '<path d="M11 2.2C14.2 4.6 15.4 8.4 15.4 13.4H6.6C6.6 8.4 7.8 4.6 11 2.2Z"/>'
    + '<path d="M6.6 13.4H15.4M8.4 17.6H13.6"/><circle cx="11" cy="8.6" r="1.2"/>',
  lander: '<path d="M7.5 3.5h7v6h-7z"/><path d="M8.5 9.5 4.2 18M13.5 9.5 17.8 18M11 9.5V18.5M2.6 18.5h4.2M15.2 18.5h4.2"/>',
};

const CSS = `
.fh-root{position:fixed;inset:0;z-index:2;pointer-events:none;--fh-d:${FULL_D}px;
  font-family:var(--sans,system-ui,sans-serif);color:var(--text,#E4E9F0)}
.fh-root *{pointer-events:none}
.fh-attitude{position:fixed;left:50%;bottom:30px;transform:translateX(-50%);box-sizing:border-box;
  min-width:280px;display:flex;flex-direction:column;align-items:center;gap:8px;padding:10px 14px 12px;
  background:rgba(10,13,18,.72);border:1px solid rgba(255,255,255,.12);border-radius:10px;
  backdrop-filter:blur(4px);-webkit-backdrop-filter:blur(4px)}
.fh-speed{display:flex;flex-direction:column;align-items:center;gap:3px;width:100%}
.fh-mode{font:600 10px/1 var(--mono,ui-monospace,Consolas,monospace);letter-spacing:.18em;
  text-transform:uppercase;color:var(--text-2,#8B97A8);min-height:10px}
.fh-spd{font:500 22px/1.1 var(--mono,ui-monospace,Consolas,monospace);font-variant-numeric:tabular-nums;
  color:var(--text,#E4E9F0);white-space:nowrap}
.fh-spd small,.fh-alt small,.fh-vs small,.fh-gval small{font-size:.78em;font-weight:400;
  color:var(--text-2,#8B97A8);margin-left:3px}
.fh-altrow{display:flex;justify-content:space-between;align-items:center;gap:14px;width:100%;
  box-sizing:border-box;padding:0 2px;font:500 12px/1.2 var(--mono,ui-monospace,Consolas,monospace);
  font-variant-numeric:tabular-nums;color:var(--text,#E4E9F0);white-space:nowrap}
.fh-altl{font:600 10px/1 var(--mono,ui-monospace,Consolas,monospace);letter-spacing:.16em;
  color:var(--text-3,#5A6675);margin-right:6px}
.fh-vs{display:flex;align-items:center;gap:5px}
.fh-vsa{width:0;height:0;display:none;border-left:4px solid transparent;border-right:4px solid transparent}
.fh-vs[data-d=up] .fh-vsa{display:block;border-bottom:6px solid var(--text,#E4E9F0)}
.fh-vs[data-d=dn] .fh-vsa{display:block;border-top:6px solid var(--text,#E4E9F0)}
.fh-row{display:flex;align-items:center;justify-content:center;gap:12px}
.fh-ball{position:relative;flex:0 0 auto;width:calc(var(--fh-d) + 12px);height:calc(var(--fh-d) + 12px)}
.fh-bezel{position:absolute;inset:0;box-sizing:border-box;padding:6px;border-radius:50%;
  background:conic-gradient(from 200deg,#7d8796,#2a313c 14%,#b4bdc9 27%,#3a4350 40%,#6f7989 52%,
    #262d38 66%,#aab3c0 80%,#3a4350 92%,#7d8796);
  box-shadow:0 0 0 1px rgba(0,0,0,.7),inset 0 0 0 1px rgba(255,255,255,.08),0 2px 10px rgba(0,0,0,.5)}
.fh-canvas{display:block;width:100%;height:100%;border-radius:50%}
.fh-noatt{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);width:112px;text-align:center;
  font:600 10px/1.5 var(--mono,ui-monospace,Consolas,monospace);letter-spacing:.14em;color:#C9D1DB;
  text-shadow:0 1px 2px #000}
.fh-hdg{position:absolute;left:50%;top:0;transform:translate(-50%,-50%);padding:2px 7px;border-radius:3px;
  background:#0A0D12;border:1px solid #334054;font:600 12px/1.3 var(--mono,ui-monospace,Consolas,monospace);
  font-variant-numeric:tabular-nums;color:var(--text,#E4E9F0);white-space:nowrap}
.fh-gauge{display:flex;flex-direction:column;align-items:center;gap:5px;width:34px}
.fh-glbl{font:600 10px/1 var(--mono,ui-monospace,Consolas,monospace);letter-spacing:.16em;color:var(--text-3,#5A6675)}
.fh-track{position:relative;box-sizing:border-box;width:12px;height:calc(var(--fh-d) - 14px);
  background:rgba(255,255,255,.05);border:1px solid rgba(255,255,255,.16);border-radius:3px}
.fh-fill{position:absolute;left:0;right:0;bottom:0;height:0;background:var(--amber,#F0A500);border-radius:0 0 2px 2px}
.fh-tick{position:absolute;left:-6px;right:-6px;height:1px;background:rgba(228,233,240,.45)}
.fh-gval{font:500 12px/1 var(--mono,ui-monospace,Consolas,monospace);font-variant-numeric:tabular-nums;
  white-space:nowrap;color:var(--text,#E4E9F0)}
.fh-gauge[data-lvl=ok] .fh-fill{background:var(--nominal,#3FB950)}
.fh-gauge[data-lvl=warn] .fh-fill{background:var(--amber,#F0A500)}
.fh-gauge[data-lvl=warn] .fh-gval{color:var(--amber,#F0A500)}
.fh-gauge[data-lvl=crit] .fh-fill{background:var(--failed,#F85149)}
.fh-gauge[data-lvl=crit] .fh-gval{color:var(--failed,#F85149)}
.fh-gauge[data-lvl=na] .fh-gval{color:var(--text-3,#5A6675)}
.fh-stack{position:fixed;left:20px;bottom:72px;width:168px;display:flex;flex-direction:column-reverse;gap:6px}
.fh-root.fh-compact .fh-stack{display:none}
.fh-stage{position:relative;box-sizing:border-box;width:168px;height:34px;display:flex;align-items:center;gap:8px;
  padding:0 9px 0 7px;border-radius:4px;overflow:hidden;background:rgba(10,13,18,.72);
  border:1px solid rgba(255,255,255,.12);backdrop-filter:blur(4px);-webkit-backdrop-filter:blur(4px);
  --fh-c:#8B97A8;
  transition:height .45s ease,margin-top .45s ease,opacity .35s ease,border-color .3s ease,box-shadow .3s ease}
.fh-stage.fh-out{height:0;opacity:0;margin-top:-6px}
.fh-stage[data-state=active]{border-color:var(--amber,#F0A500);
  box-shadow:0 0 14px rgba(240,165,0,.28),inset 0 0 0 1px rgba(240,165,0,.15)}
.fh-glyph{flex:0 0 22px;width:22px;height:22px}
.fh-glyph svg{display:block;width:22px;height:22px;fill:none;stroke:var(--fh-c);stroke-width:1.4;
  stroke-linecap:round;stroke-linejoin:round}
.fh-txt{flex:1 1 auto;min-width:0;display:flex;flex-direction:column;gap:3px}
.fh-stage[data-state=active] .fh-txt{padding-right:46px}
.fh-lbl{display:block;font:600 12px/1.1 var(--mono,ui-monospace,Consolas,monospace);color:var(--text,#E4E9F0);
  white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.fh-stage[data-state=pending] .fh-lbl{color:var(--text-2,#8B97A8)}
.fh-sub{display:block;font:400 10px/1.1 var(--mono,ui-monospace,Consolas,monospace);color:var(--text-3,#5A6675);
  white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.fh-pill{display:none;position:absolute;right:7px;top:6px;padding:2px 4px;border-radius:2px;
  background:var(--amber,#F0A500);color:#0A0D12;font:700 8px/1 var(--mono,ui-monospace,Consolas,monospace);
  letter-spacing:.14em}
.fh-stage[data-state=active] .fh-pill{display:block}
.fh-fuel{position:absolute;left:0;right:0;bottom:0;height:3px;background:rgba(255,255,255,.07)}
.fh-fuel i{display:block;height:100%;width:0;background:var(--fh-c);transition:width .4s ease}
.fh-stage[data-fuel=n] .fh-fuel{visibility:hidden}
`;

let liveCount = 0;

// ---------------------------------------------------------------- helpers

const isNum = v => typeof v === 'number' && Number.isFinite(v);
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const lerp = (a, b, t) => a + (b - a) * t;
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

function unit3(v) {
  if (!Array.isArray(v) || v.length < 3) return null;
  const a = v[0], b = v[1], c = v[2];
  if (!isNum(a) || !isNum(b) || !isNum(c)) return null;
  const L = Math.hypot(a, b, c);
  return L > 1e-9 ? [a / L, b / L, c / L] : null;
}

function readBasis(b) {
  if (!b) return null;
  const nose = unit3(b.nose), right = unit3(b.right), roof = unit3(b.roof);
  return nose && right && roof ? { nose, right, roof } : null;
}

const vecKey = v => v.map(x => x.toFixed(4)).join(',');
const basisKey = B => vecKey(B.nose) + ';' + vecKey(B.right) + ';' + vecKey(B.roof);

// Direction on the sphere for a pitch and heading (degrees), NEU components.
function dirOf(pitchDeg, headDeg) {
  const p = pitchDeg * RAD, h = headDeg * RAD;
  return [Math.cos(p) * Math.cos(h), Math.cos(p) * Math.sin(h), Math.sin(p)];
}

function headingOf(n) {
  const h = Math.round(Math.atan2(n[1], n[0]) * DEG);
  return ((h % 360) + 360) % 360;
}

function fmtInt(n) {
  return String(Math.round(n)).replace(/\B(?=(\d{3})+(?!\d))/g, ',');
}

// [value, unit]
function fmtSpeed(v) {
  if (!isNum(v)) return ['—', ''];
  const a = Math.abs(v);
  return [a < 1000 ? a.toFixed(1) : fmtInt(a), 'm/s'];
}

function fmtAlt(m) {
  if (!isNum(m)) return ['—', ''];
  const a = Math.abs(m), s = m < 0 ? '-' : '';
  if (a < 1e4) return [s + fmtInt(a), 'm'];
  if (a < 1e6) return [s + (a / 1e3).toFixed(1), 'km'];
  return [s + fmtInt(a / 1e3), 'km'];
}

// [value, arrow direction 'up' | 'dn' | '']
function fmtVs(v) {
  if (!isNum(v)) return ['—', ''];
  const r = Math.round(v * 10) / 10;
  if (r === 0) return ['0.0', ''];
  return [(r > 0 ? '+' : '-') + Math.abs(r).toFixed(1), r > 0 ? 'up' : 'dn'];
}

function cssVar(name, fallback) {
  try {
    const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    return v || fallback;
  } catch (e) {
    return fallback;
  }
}

// DOM writes only when the value actually changes.
function setText(node, s) {
  if (node._t !== s) { node._t = s; node.textContent = s; }
}
function setData(node, key, val) {
  if (node.dataset[key] !== val) node.dataset[key] = val;
}
function setStyle(node, prop, val) {
  const k = '_s_' + prop;
  if (node[k] !== val) { node[k] = val; node.style[prop] = val; }
}
function setDisplay(node, show) {
  setStyle(node, 'display', show ? '' : 'none');
}
function mk(tag, cls, text) {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text != null) n.textContent = text;
  return n;
}

// Pixel grid for a device-pixel square: only pixels inside (plus a 1px rim).
function buildGrid(size) {
  const r = size / 2;
  const idx = [], sx = [], sy = [], sz = [], ea = [];
  for (let j = 0; j < size; j++) {
    const y = (r - (j + 0.5)) / r;
    for (let i = 0; i < size; i++) {
      const x = (i + 0.5 - r) / r;
      const rr = Math.sqrt(x * x + y * y);
      if (rr > 1 + 1 / r) continue;
      idx.push(j * size + i);
      sx.push(x);
      sy.push(y);
      sz.push(Math.sqrt(Math.max(0, 1 - x * x - y * y)));
      ea.push(clamp((1 - rr) * r + 0.5, 0, 1));
    }
  }
  return {
    idx: Int32Array.from(idx), sx: Float64Array.from(sx), sy: Float64Array.from(sy),
    sz: Float64Array.from(sz), ea: Float64Array.from(ea),
  };
}

// Sky/ground colour plus anti-aliased horizon, pitch and heading lines.
// Line strength is measured in pixels: distance in degrees divided by the
// local footprint, which grows toward the limb.
function paintBall(data, grid, size, B) {
  const { idx, sx, sy, sz, ea } = grid;
  const r = size / 2;
  const [n0, n1, n2] = B.nose, [r0, r1, r2] = B.right, [u0, u1, u2] = B.roof;
  const fpBase = DEG / r;
  for (let k = 0; k < idx.length; k++) {
    const x = sx[k], y = sy[k], z = sz[k];
    const d0 = x * r0 + y * u0 + z * n0;
    const d1 = x * r1 + y * u1 + z * n1;
    const d2 = x * r2 + y * u2 + z * n2;
    const pitch = Math.asin(clamp(d2, -1, 1)) * DEG;
    let hd = Math.atan2(d1, d0) * DEG;
    if (hd < 0) hd += 360;
    const fp = fpBase / Math.max(z, 0.2);
    const shade = 0.55 + 0.45 * z;

    let cr, cg, cb;
    if (pitch >= 0) {
      const t = Math.min(1, pitch / 90);
      cr = lerp(SKY_HZ[0], SKY_ZN[0], t) * shade;
      cg = lerp(SKY_HZ[1], SKY_ZN[1], t) * shade;
      cb = lerp(SKY_HZ[2], SKY_ZN[2], t) * shade;
    } else {
      const t = Math.min(1, -pitch / 90);
      cr = lerp(GND_HZ[0], GND_NR[0], t) * shade;
      cg = lerp(GND_HZ[1], GND_NR[1], t) * shade;
      cb = lerp(GND_HZ[2], GND_NR[2], t) * shade;
    }

    const ap = Math.abs(pitch);
    // Horizon, 2 px.
    let lum = clamp(1.5 - ap / fp, 0, 1);
    // Pitch ladder every 10 deg, 1 px at 70 %.
    const kp = Math.round(pitch / 10) * 10;
    if (kp !== 0 && Math.abs(kp) <= 80) {
      const pl = clamp(1 - Math.abs(pitch - kp) / fp, 0, 1) * 0.7;
      if (pl > lum) lum = pl;
    }
    // Heading meridians every 30 deg, up to +-80 deg pitch.
    if (ap < 80) {
      let dh = hd - Math.round(hd / 30) * 30;
      dh = ((dh + 540) % 360) - 180;
      const dm = (Math.abs(dh) * Math.cos(pitch * RAD)) / fp;
      const mer = clamp(1 - dm, 0, 1) * 0.55;
      if (mer > lum) lum = mer;
    }
    cr += (240 - cr) * lum;
    cg += (240 - cg) * lum;
    cb += (240 - cb) * lum;

    const o = idx[k] * 4;
    data[o] = cr;
    data[o + 1] = cg;
    data[o + 2] = cb;
    data[o + 3] = ea[k] * 255;
  }
}

function haloText(ctx, t, x, y, color) {
  ctx.lineWidth = 3;
  ctx.strokeStyle = HALO;
  ctx.strokeText(t, x, y);
  ctx.fillStyle = color;
  ctx.fillText(t, x, y);
}

function strokeMarker(ctx, x, y, kind) {
  for (const pass of [0, 1]) {
    ctx.beginPath();
    ctx.strokeStyle = pass ? PRO_COLOR : 'rgba(8,11,16,0.7)';
    ctx.lineWidth = pass ? 1.6 : 3.4;
    ctx.arc(x, y, 8, 0, TAU);
    if (kind === 'pro') {
      ctx.moveTo(x, y - 8); ctx.lineTo(x, y - 14);
      ctx.moveTo(x - 8, y); ctx.lineTo(x - 14, y);
      ctx.moveTo(x + 8, y); ctx.lineTo(x + 14, y);
    } else {
      ctx.moveTo(x - 5.5, y - 5.5); ctx.lineTo(x + 5.5, y + 5.5);
      ctx.moveTo(x + 5.5, y - 5.5); ctx.lineTo(x - 5.5, y + 5.5);
    }
    ctx.stroke();
  }
  if (kind === 'pro') {
    ctx.beginPath();
    ctx.fillStyle = PRO_COLOR;
    ctx.arc(x, y, 1.6, 0, TAU);
    ctx.fill();
  }
}

// ---------------------------------------------------------------- instance

class Hud {
  constructor(host) {
    this.dead = false;
    this.visible = true;
    this.mode = 'full';
    this.basis = null;
    this.prograde = null;
    this.wantKey = null;
    this.drawnKey = null;
    this.baseKey = null;
    this.lastDraw = 0;
    this.timer = 0;
    this.d = 0;
    this.size = 0;
    this.dpr = 0;
    this.grid = null;
    this.img = null;
    this.amber = cssVar('--amber', '#F0A500');
    this.stages = new Map();   // id -> card
    this.leaving = new Map();  // id -> removal timer
    this.buildDom(host);
    this.onResize = () => this.applyMode();
    window.addEventListener('resize', this.onResize);
    this.applyMode();
  }

  buildDom(host) {
    this.root = mk('div', 'fh-root');
    this.att = mk('div', 'fh-attitude');

    // speed box
    const speed = mk('div', 'fh-speed');
    this.elMode = mk('div', 'fh-mode');
    const spd = mk('div', 'fh-spd');
    this.elSpdV = mk('span');
    this.elSpdU = mk('small');
    spd.append(this.elSpdV, this.elSpdU);
    const altRow = mk('div', 'fh-altrow');
    const alt = mk('div');
    this.elAltL = mk('span', 'fh-altl', 'ALT');
    this.elAltV = mk('span');
    this.elAltU = mk('small');
    alt.append(this.elAltL, this.elAltV, this.elAltU);
    this.elVs = mk('div', 'fh-vs');
    this.elVsA = mk('i', 'fh-vsa');
    this.elVsV = mk('span');
    this.elVs.append(this.elVsA, this.elVsV, mk('small', null, 'm/s'));
    altRow.append(alt, this.elVs);
    speed.append(this.elMode, spd, altRow);

    // gauges and ball
    const row = mk('div', 'fh-row');
    this.thr = this.makeGauge('THR', [0, 25, 50, 75, 100]);
    this.gg = this.makeGauge('G', [0, 100 / 6, 200 / 6, 50, 400 / 6, 500 / 6, 100]);

    const ball = mk('div', 'fh-ball');
    const bezel = mk('div', 'fh-bezel');
    this.canvas = mk('canvas', 'fh-canvas');
    this.ctx = this.canvas.getContext('2d');
    this.noatt = mk('div', 'fh-noatt', 'NO ATTITUDE DATA');
    bezel.append(this.canvas, this.noatt);
    this.elHdg = mk('div', 'fh-hdg', '—');
    ball.append(bezel, this.elHdg);
    this.base = document.createElement('canvas');
    this.baseCtx = this.base.getContext('2d');

    row.append(this.thr.root, ball, this.gg.root);
    this.att.append(speed, row);

    this.stack = mk('div', 'fh-stack');
    this.root.append(this.att, this.stack);
    host.append(this.root);
  }

  makeGauge(label, ticks) {
    const root = mk('div', 'fh-gauge');
    const track = mk('div', 'fh-track');
    const fill = mk('div', 'fh-fill');
    track.append(fill);
    for (const t of ticks) {
      const tk = mk('div', 'fh-tick');
      tk.style.bottom = t + '%';
      track.append(tk);
    }
    const val = mk('div', 'fh-gval');
    const vn = mk('span');
    const vu = mk('small');
    val.append(vn, vu);
    root.append(mk('div', 'fh-glbl', label), track, val);
    return { root, fill, vn, vu };
  }

  // ------------------------------------------------------------ sizing

  setSize(d) {
    this.d = d;
    this.dpr = window.devicePixelRatio || 1;
    this.root.style.setProperty('--fh-d', d + 'px');
    const size = Math.max(2, Math.round(d * this.dpr));
    if (size !== this.size) {
      this.size = size;
      this.canvas.width = size;
      this.canvas.height = size;
      this.base.width = size;
      this.base.height = size;
      this.grid = buildGrid(size);
      this.img = this.baseCtx.createImageData(size, size);
      this.baseKey = null;
    }
    this.drawnKey = null;
    this.schedule();
  }

  applyMode() {
    if (this.dead) return;
    const compact = this.mode === 'compact' || window.innerWidth < COMPACT_BELOW_PX;
    this.root.classList.toggle('fh-compact', compact);
    const d = compact ? COMPACT_D : FULL_D;
    if (d !== this.d) this.setSize(d);
  }

  setMode(name) {
    if (this.dead) return;
    this.mode = name === 'compact' ? 'compact' : 'full';
    this.applyMode();
  }

  setVisible(b) {
    if (this.dead) return;
    this.visible = !!b;
    this.root.style.display = this.visible ? '' : 'none';
    if (this.visible) {
      this.drawnKey = null;
      this.schedule();
    }
  }

  // ------------------------------------------------------------ ball

  schedule() {
    if (this.timer || this.dead || !this.visible) return;
    const wait = Math.max(0, this.lastDraw + FRAME_MS - performance.now());
    this.timer = setTimeout(() => this.draw(), wait);
  }

  draw() {
    this.timer = 0;
    if (this.dead || !this.visible || !this.grid) return;
    this.lastDraw = performance.now();
    if ((window.devicePixelRatio || 1) !== this.dpr) { this.setSize(this.d); return; }

    const B = this.basis, P = this.prograde;
    const key = (B ? basisKey(B) : '-') + '|' + (P ? vecKey(P) : '-');
    if (key === this.drawnKey) return;

    const ctx = this.ctx, size = this.size, dpr = this.dpr;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    if (!B) {
      this.drawBlank();
    } else {
      const bk = basisKey(B);
      if (bk !== this.baseKey) {
        paintBall(this.img.data, this.grid, size, B);
        this.baseCtx.putImageData(this.img, 0, 0);
        this.baseKey = bk;
      }
      ctx.clearRect(0, 0, size, size);
      ctx.drawImage(this.base, 0, 0);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      this.overlay(B, P);
      ctx.setTransform(1, 0, 0, 1, 0, 0);
    }
    this.drawnKey = key;
  }

  drawBlank() {
    const ctx = this.ctx, size = this.size, r = size / 2;
    ctx.clearRect(0, 0, size, size);
    const g = ctx.createRadialGradient(r * 0.8, r * 0.7, r * 0.1, r, r, r);
    g.addColorStop(0, '#5b6471');
    g.addColorStop(1, '#1d232c');
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.arc(r, r, r - 0.5, 0, TAU);
    ctx.fill();
  }

  overlay(B, P) {
    const ctx = this.ctx, S = this.d, c = S / 2, R = S / 2;
    const N = B.nose, Rt = B.right, U = B.roof;
    const pt = v => [c + dot(v, Rt) * R, c - dot(v, U) * R];

    ctx.save();
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';

    // Pitch ladder labels, on the heading midway between meridians.
    ctx.font = LABEL_FONT;
    for (const p of [30, 60, -30, -60]) {
      const v = dirOf(p, 15);
      if (dot(v, N) <= 0.15) continue;
      const [x, y] = pt(v);
      haloText(ctx, (p > 0 ? '+' : '-') + Math.abs(p), x, y, LBL);
    }
    // Heading labels, just above the horizon.
    for (const [h, t] of HEAD_LABELS) {
      const v = dirOf(0, h);
      if (dot(v, N) <= 0.15) continue;
      const [x, y] = pt(v);
      haloText(ctx, t, x, y - 8, h % 90 === 0 ? LBL : LBL2);
    }

    // Fixed vehicle reference: W chevron and centre dot.
    ctx.beginPath();
    W_PTS.forEach(([px, py], i) => {
      const X = c + px * R, Y = c + py * R;
      if (i) ctx.lineTo(X, Y); else ctx.moveTo(X, Y);
    });
    ctx.lineWidth = 4.2;
    ctx.strokeStyle = 'rgba(8,11,16,0.7)';
    ctx.stroke();
    ctx.lineWidth = 2.4;
    ctx.strokeStyle = this.amber;
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(c, c, 2.4, 0, TAU);
    ctx.fillStyle = this.amber;
    ctx.fill();

    if (P) {
      this.marker(ctx, P, 'pro', c, R, B);
      this.marker(ctx, [-P[0], -P[1], -P[2]], 'retro', c, R, B);
    }
    ctx.restore();
  }

  marker(ctx, v, kind, c, R, B) {
    const dep = dot(v, B.nose), px = dot(v, B.right), py = dot(v, B.roof);
    if (dep > 0) {
      strokeMarker(ctx, c + px * R, c - py * R, kind);
      return;
    }
    // Behind the ball: a dimmed arrow on the limb, pointing toward it.
    const L = Math.hypot(px, py);
    const ux = L > 1e-6 ? px / L : 0;
    const uy = L > 1e-6 ? -py / L : 1;
    const tipX = c + ux * (R - 2), tipY = c + uy * (R - 2);
    const bx = c + ux * (R - 12), by = c + uy * (R - 12);
    ctx.save();
    ctx.globalAlpha = 0.5;
    ctx.fillStyle = PRO_COLOR;
    ctx.beginPath();
    ctx.moveTo(tipX, tipY);
    ctx.lineTo(bx - uy * 5, by + ux * 5);
    ctx.lineTo(bx + uy * 5, by - ux * 5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  // ------------------------------------------------------------ stages

  makeCard() {
    const card = {};
    card.el = mk('div', 'fh-stage');
    card.el.innerHTML = '<span class="fh-glyph"></span>'
      + '<span class="fh-txt"><span class="fh-lbl"></span><span class="fh-sub"></span></span>'
      + '<span class="fh-pill">ACTIVE</span><span class="fh-fuel"><i></i></span>';
    card.glyph = card.el.querySelector('.fh-glyph');
    card.lbl = card.el.querySelector('.fh-lbl');
    card.sub = card.el.querySelector('.fh-sub');
    card.bar = card.el.querySelector('.fh-fuel i');
    card.kind = null;
    card.color = null;
    card.fuel = undefined;
    card.leaving = false;
    return card;
  }

  fillCard(card, st, state) {
    const el = card.el;
    setData(el, 'state', state);
    const kind = GLYPH[st.kind] ? st.kind : 'core';
    if (card.kind !== kind) {
      card.kind = kind;
      card.glyph.innerHTML = '<svg viewBox="0 0 22 22" aria-hidden="true">' + GLYPH[kind] + '</svg>';
    }
    const color = typeof st.color === 'string' && st.color ? st.color : '#8B97A8';
    if (card.color !== color) {
      card.color = color;
      el.style.setProperty('--fh-c', color);
    }
    setText(card.lbl, st.label != null ? String(st.label) : String(st.id));
    setText(card.sub, st.sub != null ? String(st.sub) : '');
    if (isNum(st.fuel)) {
      const f = clamp(st.fuel, 0, 1);
      if (card.fuel !== f) {
        card.fuel = f;
        card.bar.style.width = (f * 100).toFixed(1) + '%';
      }
      setData(el, 'fuel', 'y');
    } else {
      card.fuel = undefined;
      setData(el, 'fuel', 'n');
    }
  }

  drop(id, card) {
    const t = this.leaving.get(id);
    if (t) { clearTimeout(t); this.leaving.delete(id); }
    card.leaving = false;
    card.el.remove();
    this.stages.delete(id);
  }

  leave(id, card) {
    if (card.leaving) return;
    card.leaving = true;
    card.el.classList.add('fh-out');
    this.leaving.set(id, setTimeout(() => {
      this.leaving.delete(id);
      this.drop(id, card);
    }, LEAVE_MS));
  }

  unleave(id, card) {
    if (!card.leaving) return;
    card.leaving = false;
    card.el.classList.remove('fh-out');
    const t = this.leaving.get(id);
    if (t) { clearTimeout(t); this.leaving.delete(id); }
  }

  updateStages(list) {
    const stages = Array.isArray(list) ? list : [];
    const want = [];
    const seen = new Set();
    let shown = 0;
    for (const st of stages) {
      if (!st || st.id == null) continue;
      const id = st.id;
      seen.add(id);
      let card = this.stages.get(id);
      const state = st.state === 'active' || st.state === 'spent' ? st.state : 'pending';
      if (state === 'spent') {
        if (card) {
          this.leave(id, card);
          want.push(card.el);
        }
        continue;
      }
      if (card) this.unleave(id, card);
      if (shown >= MAX_CARDS) {
        if (card) this.drop(id, card);
        continue;
      }
      shown++;
      if (!card) {
        card = this.makeCard();
        this.stages.set(id, card);
      }
      this.fillCard(card, st, state);
      want.push(card.el);
    }
    // Stages that left the list entirely.
    for (const [id, card] of this.stages) if (!seen.has(id)) this.drop(id, card);

    // Firing order, first to fire at the bottom (column-reverse).
    const wantSet = new Set(want);
    for (const node of Array.from(this.stack.children)) if (!wantSet.has(node)) node.remove();
    for (let i = 0; i < want.length; i++) {
      const cur = this.stack.children[i];
      if (cur !== want[i]) this.stack.insertBefore(want[i], cur || null);
    }
  }

  // ------------------------------------------------------------ update

  setGauge(g, frac, text, unit, lvl) {
    setStyle(g.fill, 'height', frac == null ? '0%' : (frac * 100).toFixed(2) + '%');
    setText(g.vn, text);
    setText(g.vu, unit);
    if (lvl != null) setData(g.root, 'lvl', lvl);
  }

  update(s) {
    if (this.dead) return;
    s = s || {};

    const sp = fmtSpeed(s.speed);
    setText(this.elSpdV, sp[0]);
    setText(this.elSpdU, sp[1]);
    setText(this.elMode, typeof s.speedMode === 'string' ? s.speedMode : '');
    setText(this.elAltL, typeof s.altLabel === 'string' && s.altLabel ? s.altLabel : 'ALT');
    const al = fmtAlt(s.alt);
    setText(this.elAltV, al[0]);
    setText(this.elAltU, al[1]);
    const vs = fmtVs(s.vs);
    setText(this.elVsV, vs[0]);
    setData(this.elVs, 'd', vs[1]);

    const th = isNum(s.throttle) ? clamp(s.throttle, 0, 1) : null;
    this.setGauge(this.thr, th, th == null ? '—' : String(Math.round(th * 100)),
      th == null ? '' : '%', null);

    const g = isNum(s.accel_g) ? s.accel_g : null;
    const glvl = g == null ? 'na' : g > 5 ? 'crit' : g > 4 ? 'warn' : 'ok';
    this.setGauge(this.gg, g == null ? null : clamp(g / 6, 0, 1),
      g == null ? '—' : (Math.abs(g) < 10 ? g.toFixed(1) : fmtInt(g)),
      g == null ? '' : 'g', glvl);

    const B = readBasis(s.basis), P = unit3(s.prograde);
    const h = B ? headingOf(B.nose) : null;
    setText(this.elHdg, h == null ? '—' : String(h).padStart(3, '0') + '°');
    setDisplay(this.noatt, !B);
    this.basis = B;
    this.prograde = P;
    const key = (B ? basisKey(B) : '-') + '|' + (P ? vecKey(P) : '-');
    if (key !== this.wantKey) {
      this.wantKey = key;
      this.schedule();
    }

    this.updateStages(s.stages);
  }

  destroy() {
    if (this.dead) return;
    this.dead = true;
    clearTimeout(this.timer);
    for (const t of this.leaving.values()) clearTimeout(t);
    this.leaving.clear();
    window.removeEventListener('resize', this.onResize);
    this.root.remove();
    this.stages.clear();
    liveCount = Math.max(0, liveCount - 1);
    if (liveCount === 0) {
      const style = document.getElementById(STYLE_ID);
      if (style) style.remove();
    }
  }
}

function injectStyle() {
  if (document.getElementById(STYLE_ID)) return;
  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = CSS;
  document.head.appendChild(style);
}

// ---------------------------------------------------------------- public

export function createFlightHud(container) {
  const host = container || document.body;
  injectStyle();
  liveCount++;
  const hud = new Hud(host);
  return {
    update: state => hud.update(state),
    setVisible: b => hud.setVisible(b),
    setMode: m => hud.setMode(m),
    destroy: () => hud.destroy(),
  };
}
