// Selenographic ground track for lunar missions.
//
// Longitude 0 is the sub-Earth meridian, so the near side sits in the middle of
// the map and the far side wraps to the edges. The panel ships the sub-point of
// the parking orbit, the descent ellipse and the powered descent; this draws
// them over a real equirectangular lunar map (NASA CGI Moon Kit, public domain)
// — the Moon's answer to the Earth ground track: WHICH part of the surface the
// flight is over.
//
// The map URL is resolved from this module's own location, so the two copies of
// this file (served at /static and at ./static) stay byte-identical.

const MAP_URL = new URL('./moon_map.jpg', import.meta.url).href;

const PARK = '#3987e5';       // parking orbit
const TRANSFER = '#c98500';   // descent-transfer ellipse
const POWERED = '#d55181';    // powered descent
const SITE = '#8fbef0';       // touchdown marker

let mapImg = null, mapReady = false, mapFailed = false, lastDraw = null;

function ensureMap() {
  if (mapImg) return;
  mapImg = new Image();
  mapImg.decoding = 'async';
  mapImg.onload = () => { mapReady = true; if (lastDraw) drawMoonGroundTrack(...lastDraw); };
  mapImg.onerror = () => { mapFailed = true; if (lastDraw) drawMoonGroundTrack(...lastDraw); };
  mapImg.src = MAP_URL;
}

function llRuns(ph, lat, lon) {
  const runs = [];
  let cur = null;
  for (let i = 0; i < lat.length; i++) {
    const p = +((ph && ph[i]) || 0);
    if (!cur || cur.ph !== p) { cur = { ph: p, lat: [], lon: [] }; runs.push(cur); }
    cur.lat.push(lat[i]); cur.lon.push(lon[i]);
  }
  return runs.filter(r => r.lat.length > 1);
}

function strokeLL(ctx, xy, lat, lon, color, width, alpha) {
  ctx.save();
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.globalAlpha = alpha;
  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  ctx.shadowColor = 'rgba(0,0,0,.6)'; ctx.shadowBlur = 3;
  ctx.beginPath();
  let pen = false, pl = null;
  for (let i = 0; i < lat.length; i++) {
    const [x, y] = xy(lon[i], lat[i]);
    if (!pen || (pl !== null && Math.abs(pl - lon[i]) > 180)) { ctx.moveTo(x, y); pen = true; }
    else ctx.lineTo(x, y);
    pl = lon[i];
  }
  ctx.stroke();
  ctx.restore();
}

export function drawMoonGroundTrack(target, moon, opts = {}) {
  const cv = typeof target === 'string' ? document.getElementById(target) : target;
  if (!cv || !moon || !moon.orbit || !moon.orbit.lat || moon.orbit.lat.length < 2) return;
  ensureMap();
  lastDraw = [target, moon, opts];
  const ctx = cv.getContext('2d'), dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const xy = (lo, la) => [(lo + 180) / 360 * w, (90 - la) / 180 * h];

  if (mapReady) {
    ctx.drawImage(mapImg, 0, 0, w, h);
    // Slight darkening so the coloured tracks stay legible on the bright mare.
    ctx.fillStyle = 'rgba(6,14,22,.28)'; ctx.fillRect(0, 0, w, h);
  } else {
    const grd = ctx.createLinearGradient(0, 0, 0, h);
    grd.addColorStop(0, '#0d2132'); grd.addColorStop(1, '#07131e');
    ctx.fillStyle = grd; ctx.fillRect(0, 0, w, h);
  }

  // Graticule every 30 deg, thin and bright over a dark map / dark over the
  // lit mare; either way it reads as a chart rather than a photograph.
  ctx.strokeStyle = mapReady ? 'rgba(10,20,30,.35)' : 'rgba(143,190,240,.10)';
  ctx.lineWidth = 1;
  for (let lo = -150; lo <= 150; lo += 30) {
    const [x] = xy(lo, 0); ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }
  for (let la = -60; la <= 60; la += 30) {
    const [, y] = xy(0, la); ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  // Equator and the near/far seams a touch stronger.
  ctx.strokeStyle = 'rgba(20,32,44,.55)';
  { const [, y0] = xy(0, 0);
    ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(w, y0); ctx.stroke();
    for (const lo of [0, 180, -180]) {
      const [x] = xy(lo, 0);
      ctx.beginPath(); ctx.moveTo(x, y0 - 4); ctx.lineTo(x, y0 + 4); ctx.stroke();
    } }

  const O = moon.orbit;
  for (const run of llRuns(O.ph, O.lat, O.lon))
    strokeLL(ctx, xy, run.lat, run.lon, run.ph === 1 ? TRANSFER : PARK, 1.8,
             run.ph === 1 ? 0.9 : 0.95);
  const D = moon.descent;
  if (D && D.lat && D.lat.length > 1)
    strokeLL(ctx, xy, D.lat, D.lon, POWERED, 2.6, 1);

  const site = opts.site ||
    (D && D.lat ? { lat: D.lat[D.lat.length - 1], lon: D.lon[D.lon.length - 1] } : null);
  if (site && Number.isFinite(+site.lat) && Number.isFinite(+site.lon)) {
    const [x, y] = xy(+site.lon, +site.lat);
    ctx.save();
    ctx.shadowColor = 'rgba(0,0,0,.75)'; ctx.shadowBlur = 5;
    ctx.fillStyle = SITE; ctx.strokeStyle = '#07131e'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y - 8); ctx.lineTo(x + 8, y); ctx.lineTo(x, y + 8); ctx.lineTo(x - 8, y);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    ctx.restore();
    const lbl = opts.label || 'touchdown';
    ctx.font = '11px ui-monospace,monospace';
    const lx = Math.min(x + 11, w - ctx.measureText(lbl).width - 6);
    const ly = Math.max(y - 9, 28);
    ctx.fillStyle = 'rgba(4,8,12,.7)';
    ctx.fillRect(lx - 3, ly - 11, ctx.measureText(lbl).width + 6, 14);
    ctx.fillStyle = SITE;
    ctx.fillText(lbl, lx, ly);
  }

  ctx.fillStyle = 'rgba(8,12,18,.6)';
  ctx.fillRect(0, h - 18, w, 18);
  ctx.fillStyle = 'rgba(200,214,230,.85)'; ctx.font = '9px system-ui';
  ctx.textAlign = 'right';
  ctx.fillText('selenographic · 0° lon = sub-Earth meridian · near side centre',
               w - 8, h - 6);
  ctx.textAlign = 'left';
  ctx.fillStyle = 'rgba(200,214,230,.85)'; ctx.font = '10px ui-monospace,monospace';
  ctx.fillText('Moon ground track', 12, 16);
  if (mapFailed)
    ctx.fillText('lunar map unavailable', 12, 30);
  cv.title = 'Selenographic ground track: parking orbit, descent ellipse and touchdown';
}
