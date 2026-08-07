// The line chart, once.
//
// This lived inside panel_page.html, which was fine while that page drew every
// chart in the app. Splitting the analysis out of mission control gave it a
// second caller, and a chart helper copied into a second page is the same
// mistake `fmt` was: two drawing routines that agree today, disagree in a
// month, and nobody notices because both of them still draw something.
//
// Chrome colours are read from the CSS custom properties in tokens.css rather
// than restated here, so a chart's gridlines are the same grey as the rules on
// the panel around it. Series colours stay with the CALLER: they carry mission
// meaning — the entry leg is the same pink in the chart, the 3D scene and the
// legend — and that mapping belongs where the phases are known.

const css = (name, fallback) => {
  try {
    const v = getComputedStyle(document.documentElement)
      .getPropertyValue(name).trim();
    return v || fallback;
  } catch (e) { return fallback; }
};

// resolved once: this is called per gridline per redraw, and getComputedStyle
// forces style resolution
let INK2, MUTED, GRID, GRIDX;
function palette() {
  if (INK2) return;
  INK2  = css('--text-2', '#8B97A8');
  MUTED = css('--text-3', '#5A6675');
  GRID  = css('--line',   '#232C3A');
  GRIDX = 'rgba(255,255,255,.05)';
}

export function circle(ctx, x, y, r, fill) {
  ctx.beginPath(); ctx.arc(x, y, r, 0, 2*Math.PI); ctx.fillStyle = fill; ctx.fill();
}

/**
 * series: [{xs, ys, color, label, unit}] — the peak of the FIRST series is
 * direct-labeled; additional series get end labels.
 *
 * `target` is a canvas element or its id.
 *
 * opts: {xu, yu, y0, nopeak, labelAt: 'start'|'end', marks: [{x,y,color,label}]}
 */
export function drawLine(target, series, title, opts) {
  palette();
  opts = opts || {};
  const cv = typeof target === 'string' ? document.getElementById(target) : target;
  if (!cv) return;
  const ctx = cv.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== w*dpr) { cv.width = w*dpr; cv.height = h*dpr; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  series = series.filter(s => s.xs && s.xs.length);
  if (!series.length) return;
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  for (const s of series) {
    for (const v of s.xs) { if (v < x0) x0 = v; if (v > x1) x1 = v; }
    for (const v of s.ys) { if (v < y0) y0 = v; if (v > y1) y1 = v; }
  }
  if (y1 - y0 < 1e-9) y1 = y0 + 1;
  if (opts.y0 !== undefined) y0 = Math.min(opts.y0, y0);
  // round the value axis out to nice ticks so the gridlines mean something
  const dy = niceStep(y1 - y0, 3);
  y0 = Math.floor(y0/dy)*dy; y1 = Math.ceil(y1/dy)*dy;
  const L = 40, R = 10, T = 20, B = 26;
  const px = x => L + (x-x0)/(x1-x0)*(w-L-R);
  const py = y => h-B - (y-y0)/(y1-y0)*(h-T-B);
  ctx.font = '10px system-ui';
  ctx.strokeStyle = GRID; ctx.lineWidth = 1;
  for (let v = y0; v <= y1 + 1e-9; v += dy) {
    const y = py(v);
    ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w-R, y); ctx.stroke();
    ctx.fillStyle = MUTED;
    const s = fmtTick(v);
    ctx.fillText(s, L - 5 - ctx.measureText(s).width, y + 3);
  }
  // x ticks across the range actually plotted
  const dx = niceStep(x1 - x0, 2);
  ctx.fillStyle = MUTED;
  for (let v = Math.ceil(x0/dx)*dx; v <= x1 + 1e-9; v += dx) {
    const x = px(v);
    ctx.strokeStyle = GRIDX;
    ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, h-B); ctx.stroke();
    const s = fmtTick(v);
    ctx.fillText(s, Math.min(x - ctx.measureText(s).width/2, w - 20), h - 14);
  }
  if (opts.xu) {
    const s = '[' + opts.xu + ']';
    ctx.fillText(s, w - R - ctx.measureText(s).width, h - 3);
  }
  if (opts.yu) ctx.fillText('[' + opts.yu + ']', 2, 12);
  for (const s of series) {
    ctx.strokeStyle = s.color; ctx.lineWidth = 1.8; ctx.lineJoin = 'round';
    ctx.beginPath();
    for (let i = 0; i < s.xs.length; i++) {
      const X = px(s.xs[i]), Y = py(s.ys[i]);
      i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y);
    }
    ctx.stroke();
  }
  ctx.font = '11px system-ui';
  // peak marker + direct label on the primary series
  const p = series[0];
  if (!opts.nopeak) {
    let ipk = 0;
    for (let i = 1; i < p.ys.length; i++) if (p.ys[i] > p.ys[ipk]) ipk = i;
    circle(ctx, px(p.xs[ipk]), py(p.ys[ipk]), 2.6, p.color);
    ctx.fillStyle = INK2;
    const lbl = `${fmtPeak(p.ys[ipk])} ${p.unit || ''}`;
    const lx = Math.min(px(p.xs[ipk]) + 6, w - ctx.measureText(lbl).width - 4);
    ctx.fillText(lbl, lx, Math.max(py(p.ys[ipk]) - 6, 26));
  }
  // called-out points (high gate, and anything else worth naming)
  for (const mk of (opts.marks || [])) {
    const X = px(mk.x), Y = py(mk.y);
    circle(ctx, X, Y, 3.4, mk.color);
    ctx.strokeStyle = mk.color; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(X, Y, 6, 0, 2*Math.PI); ctx.stroke();
    ctx.fillStyle = mk.color;
    ctx.fillText(mk.label, Math.min(X + 9, w - ctx.measureText(mk.label).width - 4), Y - 6);
  }
  // series name labels at line ends (identity not by color alone)
  if (series.length > 1) {
    for (const s of series) {
      if (!s.label) continue;
      const n = opts.labelAt === 'start' ? 0 : s.xs.length - 1;
      ctx.fillStyle = s.color;
      const tx = opts.labelAt === 'start' ? px(s.xs[0]) + 5
               : Math.min(px(s.xs[n]) - ctx.measureText(s.label).width - 4, w - 60);
      const ty = opts.labelAt === 'start' ? py(s.ys[0]) + 13 : py(s.ys[n]) - 5;
      ctx.fillText(s.label, Math.max(Math.min(tx, w - ctx.measureText(s.label).width - 4), L + 2),
                   Math.min(Math.max(ty, T + 12), h - B - 2));
    }
  }
  ctx.fillStyle = MUTED;
  ctx.fillText(title, L, 12);
}

/** A tick step from the 1/2/2.5/5/10 ladder — the reason gridlines land on
 *  numbers a reader can do arithmetic with. */
export function niceStep(span, n) {
  const raw = span/n, p = Math.pow(10, Math.floor(Math.log10(raw)));
  const f = raw/p;
  return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 2.5 ? 2.5 : f <= 5 ? 5 : 10) * p;
}

/** The direct label on a series peak — the one number on a chart a reader is
 *  meant to take away, so it must not arrive as "2.01e+3 km".
 *
 *  `toPrecision(3)` alone does exactly that above 999, which is where entry
 *  altitudes, heat loads and distances live. Four significant figures is
 *  enough for a callout, and a grouped integer is what a reader expects of a
 *  quantity that large. */
export function fmtPeak(v) {
  const a = Math.abs(+v);
  if (!isFinite(a)) return '—';
  if (a >= 1000) return Math.round(+v).toLocaleString('en-US');
  return String(+(+v).toPrecision(3));
}

/** Axis-tick text: thousands abbreviated, small values kept precise, and an
 *  exact zero written "0" rather than "0.00000". */
export function fmtTick(v) {
  return Math.abs(v) >= 1000 ? (v/1000).toFixed(v % 1000 ? 1 : 0) + 'k'
       : Math.abs(v) >= 10 ? v.toFixed(0)
       : Math.abs(v) >= 1 ? v.toFixed(1)
       : Math.abs(v) < 1e-9 ? '0' : v.toPrecision(2);
}
