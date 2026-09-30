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

const finite = Number.isFinite;
const safeName = s => String(s || 'chart').toLowerCase()
  .replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'chart';

function saveBlob(blob, name) {
  const a=document.createElement('a'),url=URL.createObjectURL(blob);
  a.href=url;a.download=name;document.body.appendChild(a);a.click();a.remove();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
}

function exportCsv(s) {
  const cols=[];
  for(const q of s.series){
    const name=q.label||s.title;
    cols.push({head:`${name} ${s.opts.xu||'x'}`,v:q.xs});
    cols.push({head:`${name} ${q.unit||s.opts.yu||'y'}`,v:q.ys});
  }
  const n=Math.max(...cols.map(c=>c.v.length));
  const quote=v=>`"${String(v??'').replaceAll('"','""')}"`;
  const rows=[cols.map(c=>quote(c.head)).join(',')];
  for(let i=0;i<n;i++)rows.push(cols.map(c=>finite(+c.v[i])?String(c.v[i]):'').join(','));
  saveBlob(new Blob([rows.join('\r\n')],{type:'text/csv;charset=utf-8'}),safeName(s.title)+'.csv');
}

export function chartExtent(s) {
  // A loop, not Math.min(...xs): a spread argument list overflows the call
  // stack somewhere past ~65k elements, and the plotted series are only
  // decimated by convention, not by contract.
  let lo=Infinity,hi=-Infinity;
  for(const q of s.series)for(const x of q.xs)if(finite(x)){if(x<lo)lo=x;if(x>hi)hi=x}
  return [lo,hi];
}

function setView(cv,a,b) {
  const s=cv.__lineChart;if(!s)return;
  const [full0,full1]=chartExtent(s),span=Math.max(full1-full0,1e-12);
  const width=Math.min(Math.max(b-a,span/500),span);
  if(a<full0){a=full0;b=a+width}
  if(b>full1){b=full1;a=b-width}
  drawLine(cv,s.series,s.title,{...s.opts,_view:[a,b]});
}

function zoomView(cv,factor,anchor) {
  const s=cv.__lineChart;if(!s)return;
  const [full0,full1]=chartExtent(s),width=s.x1-s.x0;
  const next=Math.max((full1-full0)/500,Math.min(full1-full0,width*factor));
  const f=anchor === undefined ? .5 : Math.max(0,Math.min(1,anchor));
  const x=s.x0+f*width;
  setView(cv,x-f*next,x+(1-f)*next);
}

function ensureToolbar(cv) {
  const host=cv.parentElement;
  if(!host||host.querySelector(':scope > .chart-tools'))return;
  const bar=document.createElement('div');bar.className='chart-tools';
  const add=(text,title,fn)=>{const b=document.createElement('button');b.type='button';b.textContent=text;b.title=title;b.onclick=e=>{e.stopPropagation();fn()};bar.appendChild(b)};
  add('−','Zoom out',()=>zoomView(cv,1.5));
  add('+','Zoom in',()=>zoomView(cv,.67));
  add('fit','Reset view',()=>{const s=cv.__lineChart;if(!s)return;const o={...s.opts};delete o._view;drawLine(cv,s.series,s.title,o)});
  add('CSV','Export plotted data',()=>{const s=cv.__lineChart;if(s)exportCsv(s)});
  add('PNG','Save chart image',()=>cv.toBlob&&cv.toBlob(b=>b&&saveBlob(b,safeName(cv.__lineChart?.title)+'.png')));
  host.insertBefore(bar,cv);
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
  if (cv.width !== w*dpr || cv.height !== h*dpr) { cv.width = w*dpr; cv.height = h*dpr; }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  series = series.filter(s => s.xs && s.xs.length);
  if (!series.length) return;
  let x0 = Infinity, x1 = -Infinity;
  for (const s of series) {
    for (const v of s.xs) { if (v < x0) x0 = v; if (v > x1) x1 = v; }
  }
  if (!finite(x0) || !finite(x1)) return;
  if (x1-x0 < 1e-12) x1=x0+1;
  if (opts._view && opts._view.length === 2) [x0, x1] = opts._view;
  // Re-fit the value axis to what is actually visible. Without this, zooming
  // into max-q or an RCS event magnifies time while the curve stays vertically
  // flattened against the scale of the entire mission.
  let y0=Infinity,y1=-Infinity;
  for(const s of series)for(let i=0;i<Math.min(s.xs.length,s.ys.length);i++){
    const x=s.xs[i],y=s.ys[i];
    if(finite(x)&&finite(y)&&x>=x0&&x<=x1){if(y<y0)y0=y;if(y>y1)y1=y}
  }
  if(!finite(y0)||!finite(y1))return;
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
  ctx.save();
  ctx.beginPath();ctx.rect(L,T,w-L-R,h-T-B);ctx.clip();
  for (const s of series) {
    ctx.strokeStyle = s.color; ctx.lineWidth = 1.8; ctx.lineJoin = 'round';
    ctx.beginPath();
    for (let i = 0; i < s.xs.length; i++) {
      const X = px(s.xs[i]), Y = py(s.ys[i]);
      i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y);
    }
    ctx.stroke();
  }
  ctx.restore();
  ctx.font = '11px system-ui';
  // peak marker + direct label on the primary series
  const p = series[0];
  if (!opts.nopeak) {
    const visible=p.xs.map((x,i)=>i).filter(i=>p.xs[i]>=x0&&p.xs[i]<=x1&&finite(p.ys[i]));
    if(visible.length){
      const ipk=visible.reduce((best,i)=>p.ys[i]>p.ys[best]?i:best,visible[0]);
      circle(ctx, px(p.xs[ipk]), py(p.ys[ipk]), 2.6, p.color);
      ctx.fillStyle = INK2;
      const lbl = `${fmtPeak(p.ys[ipk])} ${p.unit || ''}`;
      const lx = Math.min(px(p.xs[ipk]) + 6, w - ctx.measureText(lbl).width - 4);
      ctx.fillText(lbl, lx, Math.max(py(p.ys[ipk]) - 6, 26));
    }
  }
  // called-out points (high gate, and anything else worth naming)
  for (const mk of (opts.marks || [])) {
    if(mk.x<x0||mk.x>x1||mk.y<y0||mk.y>y1)continue;
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
      const vis=s.xs.map((_,i)=>i).filter(i=>s.xs[i]>=x0&&s.xs[i]<=x1);
      if(!vis.length)continue;
      const n = opts.labelAt === 'start' ? vis[0] : vis[vis.length-1];
      ctx.fillStyle = s.color;
      const tx = opts.labelAt === 'start' ? px(s.xs[n]) + 5
               : Math.min(px(s.xs[n]) - ctx.measureText(s.label).width - 4, w - 60);
      const ty = opts.labelAt === 'start' ? py(s.ys[n]) + 13 : py(s.ys[n]) - 5;
      ctx.fillText(s.label, Math.max(Math.min(tx, w - ctx.measureText(s.label).width - 4), L + 2),
                   Math.min(Math.max(ty, T + 12), h - B - 2));
    }
  }
  ctx.fillStyle = MUTED;
  ctx.fillText(title, L, 12);

  // Keep interaction state on the canvas. Scroll zooms around the pointer,
  // double-click resets, and pointer inspection gives every curve a precise
  // value without adding a charting dependency to the desktop build.
  cv.__lineChart = { series, title, opts, x0, x1, y0, y1, px, py, L, R, T, B };
  cv.title = 'Move to inspect · scroll to zoom · drag to pan · double-click to reset';
  ensureToolbar(cv);
  if (!cv.__lineInteractive) {
    cv.__lineInteractive = true;
    cv.style.touchAction = 'none';
    cv.style.cursor = 'crosshair';
    const repaint = () => {
      const s = cv.__lineChart;
      if (s) drawLine(cv, s.series, s.title, s.opts);
    };
    cv.addEventListener('pointerleave', () => { if(!cv.__lineDrag) repaint(); });
    cv.addEventListener('dblclick', () => {
      const s=cv.__lineChart;if(!s)return;
      const o={...s.opts};delete o._view;drawLine(cv,s.series,s.title,o);
    });
    cv.addEventListener('wheel', e => {
      e.preventDefault();const s=cv.__lineChart;if(!s)return;
      const f=Math.max(0,Math.min(1,(e.offsetX-s.L)/(cv.clientWidth-s.L-s.R)));
      zoomView(cv,e.deltaY>0?1.3:.75,f);
    }, {passive:false});
    cv.addEventListener('pointerdown',e=>{
      if(e.button!==0)return;const s=cv.__lineChart;if(!s)return;
      cv.__lineDrag={x:e.clientX,view:[s.x0,s.x1]};
      cv.setPointerCapture(e.pointerId);cv.style.cursor='grabbing';
    });
    cv.addEventListener('pointerup',e=>{
      if(cv.hasPointerCapture(e.pointerId))cv.releasePointerCapture(e.pointerId);
      cv.__lineDrag=null;cv.style.cursor='crosshair';
    });
    cv.addEventListener('pointercancel',()=>{cv.__lineDrag=null;cv.style.cursor='crosshair'});
    cv.addEventListener('pointermove', e => {
      const s=cv.__lineChart;if(!s)return;
      if(cv.__lineDrag){
        const d=cv.__lineDrag,plotW=Math.max(1,cv.clientWidth-s.L-s.R);
        const shift=-(e.clientX-d.x)/plotW*(d.view[1]-d.view[0]);
        setView(cv,d.view[0]+shift,d.view[1]+shift);return;
      }
      drawLine(cv,s.series,s.title,s.opts);
      const x=s.x0+Math.max(0,Math.min(1,(e.offsetX-s.L)/(cv.clientWidth-s.L-s.R)))*(s.x1-s.x0);
      const nearest=q=>{let k=0;for(let i=1;i<q.xs.length;i++)if(Math.abs(q.xs[i]-x)<Math.abs(q.xs[k]-x))k=i;return k};
      const primary=s.series[0],k=nearest(primary),X=s.px(primary.xs[k]);
      ctx.save();ctx.setTransform(window.devicePixelRatio||1,0,0,window.devicePixelRatio||1,0,0);
      ctx.strokeStyle='rgba(240,165,0,.55)';ctx.lineWidth=1;ctx.setLineDash([3,3]);ctx.beginPath();ctx.moveTo(X,s.T);ctx.lineTo(X,cv.clientHeight-s.B);ctx.stroke();ctx.setLineDash([]);
      // A dot on every curve where the inspector crosses it, so the value the
      // tooltip reports has a home on the plot.
      for(const a of s.series){const j=nearest(a);circle(ctx,s.px(a.xs[j]),s.py(a.ys[j]),3,a.color||'#e4e9f0');}
      const lines=s.series.map(a=>{const j=nearest(a);return `${a.label||title}: ${fmtPeak(a.ys[j])}${a.unit?' '+a.unit:''}`});
      lines.unshift(`${s.opts.xu||'x'} ${fmtPeak(primary.xs[k])}`);
      ctx.font='11px system-ui';const tw=Math.max(...lines.map(z=>ctx.measureText(z).width))+16,th=lines.length*16+10;
      const tx=Math.min(Math.max(X+8,s.L),cv.clientWidth-tw-4),ty=s.T+5;
      ctx.fillStyle='rgba(8,12,18,.94)';ctx.strokeStyle='rgba(143,190,240,.3)';ctx.fillRect(tx,ty,tw,th);ctx.strokeRect(tx,ty,tw,th);
      lines.forEach((z,i)=>{ctx.fillStyle=i?'#e4e9f0':'#f0a500';ctx.fillText(z,tx+8,ty+16+i*16)});ctx.restore();
    });
  }
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
