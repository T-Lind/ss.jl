// Moon-centred track view for lunar missions.
//
// A cislunar flight's Earth ground track cannot say anything useful about the
// last few thousand kilometres: a 100 km parking orbit, a descent ellipse with
// a 15 km perilune and the touchdown itself all collapse into a single pixel
// beside a 384,000 km transfer. So the lunar geometry gets its own frame and
// its own canvas, drawn top-down on the orbit plane, where the parking orbit
// is a circle and the descent is a visible ellipse.
//
// The payload is Moon-centred inertial (km): `orbit` is the parking coast and
// the descent-transfer ellipse (its `ph` field separates the two), and
// `descent` is the powered descent itself. No Earth/launch frame is involved,
// so this stays a pure function of what the panel sent.

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const vnorm = v => Math.hypot(v[0], v[1], v[2]);
const vunit = v => { const n = vnorm(v) || 1; return [v[0] / n, v[1] / n, v[2] / n]; };
const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const vscale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const vdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const vcross = (a, b) => [a[1] * b[2] - a[2] * b[1],
                          a[2] * b[0] - a[0] * b[2],
                          a[0] * b[1] - a[1] * b[0]];

const PARK = '#3987e5';       // parking orbit
const TRANSFER = '#c98500';   // descent-transfer ellipse
const POWERED = '#d55181';    // powered descent
const SITE = '#8fbef0';       // touchdown marker

// Split samples into contiguous runs whose phase matches, so the parking orbit
// and the descent ellipse are separate strokes rather than one line jumped
// across an ignition.
function phaseRuns(points) {
  const runs = [];
  let cur = null;
  for (const p of points) {
    if (!cur || cur.ph !== p.ph) { cur = { ph: p.ph, pts: [] }; runs.push(cur); }
    cur.pts.push(p);
  }
  return runs.filter(r => r.pts.length > 1);
}

/** The plane the parking orbit lies in, as an orthonormal basis (p, q, n). */
function orbitBasis(orbit) {
  let n = null;
  for (let i = 1; i < orbit.length && !n; i++) {
    const c = vcross(orbit[0].r, orbit[i].r);
    const scale = vnorm(orbit[0].r) * vnorm(orbit[i].r);
    if (vnorm(c) > 1e-3 * Math.max(scale, 1)) n = vunit(c);
  }
  n = n || [0, 0, 1];
  // p points through the first sample, so the parking orbit always starts at
  // the right of the frame; q completes the right-handed in-plane pair.
  const p = vunit(vsub(orbit[0].r, vscale(n, vdot(orbit[0].r, n))));
  const q = vcross(n, p);
  return [p, q];
}

// A round step near a fifth of the frame, for the scale bar.
function scaleBar(spanKm) {
  const candidates = [10, 20, 50, 100, 200, 500, 1000, 2000, 5000];
  const want = spanKm * 0.28;
  let best = candidates[0];
  for (const c of candidates) if (Math.abs(c - want) < Math.abs(best - want)) best = c;
  return best;
}

export function drawMoonView(target, moon, opts = {}) {
  const cv = typeof target === 'string' ? document.getElementById(target) : target;
  if (!cv) return;
  if (!moon || !moon.orbit || !moon.orbit.x || moon.orbit.x.length < 2) return;
  const ctx = cv.getContext('2d'), dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const O = moon.orbit, ph = O.ph || [];
  const orbit = O.x.map((x, i) =>
    ({ r: [x, O.y[i], O.z[i]], ph: +ph[i] || 0 }));
  const desc = (moon.descent && moon.descent.x)
    ? moon.descent.x.map((x, i) => [x, moon.descent.y[i], moon.descent.z[i]])
    : [];
  const rMoon = +moon.r_km || 1737.4;
  const [p, q] = orbitBasis(orbit);
  const project = r => [vdot(r, p), vdot(r, q)];

  // Extent of everything drawn, Moon included, so the frame fits.
  let Rmax = rMoon;
  for (const o of orbit) Rmax = Math.max(Rmax, Math.hypot(...project(o.r)));
  for (const d of desc) Rmax = Math.max(Rmax, Math.hypot(...project(d)));
  const margin = 26;
  const scale = (Math.min(w, h) / 2 - margin) / (Rmax * 1.06);
  const cx = w / 2, cy = h / 2;
  const X = r => cx + project(r)[0] * scale;
  const Y = r => cy - project(r)[1] * scale;

  // Ground: the same blue-black the Earth map uses, so the two reads sit
  // together without introducing a second sky.
  const grd = ctx.createLinearGradient(0, 0, 0, h);
  grd.addColorStop(0, '#0d2132'); grd.addColorStop(1, '#07131e');
  ctx.fillStyle = grd; ctx.fillRect(0, 0, w, h);

  // A faint grid on round lunar-distance rings.
  ctx.strokeStyle = 'rgba(143,190,240,.09)'; ctx.lineWidth = 1;
  for (const ring of [rMoon, rMoon + 500, rMoon + 2000, rMoon + 5000]) {
    const rr = ring * scale;
    if (rr > Math.min(w, h) / 2) continue;
    ctx.beginPath(); ctx.arc(cx, cy, rr, 0, 2 * Math.PI); ctx.stroke();
  }

  // The Moon, lit from the Earth-facing side (left of the inertial frame's
  // projected sky) so the disc reads as a body rather than a pipe.
  const disc = ctx.createRadialGradient(cx - rMoon * scale * 0.35, cy, rMoon * scale * 0.1,
                                        cx, cy, rMoon * scale);
  disc.addColorStop(0, '#41495a'); disc.addColorStop(0.7, '#262c38');
  disc.addColorStop(1, '#141922');
  ctx.beginPath(); ctx.arc(cx, cy, rMoon * scale, 0, 2 * Math.PI);
  ctx.fillStyle = disc; ctx.fill();
  ctx.strokeStyle = '#5b6675'; ctx.lineWidth = 1; ctx.stroke();

  // Orbit segments, coloured by phase.
  ctx.lineJoin = 'round'; ctx.lineCap = 'round';
  for (const run of phaseRuns(orbit)) {
    ctx.strokeStyle = run.ph === 1 ? TRANSFER : PARK;
    ctx.lineWidth = run.ph === 1 ? 1.6 : 1.8;
    ctx.globalAlpha = run.ph === 1 ? 0.9 : 1;
    ctx.beginPath();
    run.pts.forEach((o, i) => { const x = X(o.r), y = Y(o.r); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();
  }
  ctx.globalAlpha = 1;

  // Powered descent: the last few hundred kilometres, which is the whole
  // reason the section exists.
  if (desc.length > 1) {
    ctx.strokeStyle = POWERED; ctx.lineWidth = 2.2;
    ctx.beginPath();
    desc.forEach((r, i) => { const x = X(r), y = Y(r); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();
    const a = desc[0], b = desc[desc.length - 1];
    ctx.fillStyle = POWERED;
    ctx.beginPath(); ctx.arc(X(a), Y(a), 3, 0, 2 * Math.PI); ctx.fill();
    // Touchdown: the one point on this view with a fixed meaning.
    ctx.fillStyle = SITE; ctx.strokeStyle = '#07131e'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(X(b), Y(b) - 7); ctx.lineTo(X(b) + 6, Y(b));
    ctx.lineTo(X(b), Y(b) + 7); ctx.lineTo(X(b) - 6, Y(b)); ctx.closePath();
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = SITE; ctx.font = '11px ui-monospace,monospace';
    const lbl = opts.label || 'touchdown';
    ctx.fillText(lbl, Math.min(X(b) + 10, w - ctx.measureText(lbl).width - 6),
                 Math.max(Y(b) - 8, 14));
  }

  // Scale bar and caption.
  const stepKm = scaleBar(Rmax);
  const barPx = stepKm * scale;
  ctx.strokeStyle = '#8B97A8'; ctx.lineWidth = 1.5;
  const bx = 14, by = h - 16;
  ctx.beginPath(); ctx.moveTo(bx, by); ctx.lineTo(bx + barPx, by);
  ctx.moveTo(bx, by - 4); ctx.lineTo(bx, by + 4);
  ctx.moveTo(bx + barPx, by - 4); ctx.lineTo(bx + barPx, by + 4);
  ctx.stroke();
  ctx.fillStyle = '#8B97A8'; ctx.font = '10px ui-monospace,monospace';
  ctx.fillText(`${stepKm.toLocaleString('en-US')} km`, bx + barPx + 8, by + 4);
  ctx.fillText('Moon-centred · top-down on the orbit plane', 14, 18);
  ctx.textAlign = 'right';
  ctx.fillText(`Moon r ${rMoon.toFixed(0)} km`, w - 12, 18);
  ctx.textAlign = 'left';

  const last = desc.length ? desc[desc.length - 1] : null;
  const altKm = last ? vnorm(last) - rMoon : null;
  cv.__moonState = { rMoon, stepKm, altKm, scale };
  cv.title = 'Moon-centred orbit geometry: parking orbit, descent ellipse, touchdown';
}

// ---------------------------------------------------------- ground track -----
// The sub-vehicle point in selenographic coordinates, longitude 0 being the
// sub-Earth meridian: the near side sits in the middle of the map and the far
// side wraps to the edges. This is the Moon's answer to the Earth ground track
// — it says WHERE on the Moon the parking orbit, the descent ellipse and the
// touchdown actually are, which the orbit-plane view above cannot.

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
  const ctx = cv.getContext('2d'), dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight;
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const xy = (lo, la) => [(lo + 180) / 360 * w, (90 - la) / 180 * h];
  const grd = ctx.createLinearGradient(0, 0, 0, h);
  grd.addColorStop(0, '#0d2132'); grd.addColorStop(1, '#07131e');
  ctx.fillStyle = grd; ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = 'rgba(143,190,240,.10)'; ctx.lineWidth = 1;
  for (let lo = -150; lo <= 150; lo += 30) {
    const [x] = xy(lo, 0); ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }
  for (let la = -60; la <= 60; la += 30) {
    const [, y] = xy(0, la); ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  ctx.strokeStyle = 'rgba(143,190,240,.22)';
  {
    const [, y0] = xy(0, 0);
    ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(w, y0); ctx.stroke();
    for (const lo of [0, 180, -180]) {
      const [x] = xy(lo, 0);
      ctx.beginPath(); ctx.moveTo(x, y0 - 4); ctx.lineTo(x, y0 + 4); ctx.stroke();
    }
  }

  // The big near-side maria, approximate [lat, lon, r_lat, r_lon]: enough
  // geography to say which face of the Moon the track is over.
  const maria = [[33, -16, 9, 13], [28, 18, 7, 10], [17, 59, 9, 7],
                 [-15, 34, 6, 7], [-24, -39, 5, 6], [-8, -52, 21, 10]];
  ctx.fillStyle = 'rgba(96,106,118,.20)';
  for (const [la, lo, rl, rn] of maria) {
    const [x, y] = xy(lo, la);
    ctx.beginPath(); ctx.ellipse(x, y, rn / 360 * w, rl / 180 * h, 0, 0, 2 * Math.PI); ctx.fill();
  }

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
    ctx.fillStyle = SITE; ctx.strokeStyle = '#07131e'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y - 7); ctx.lineTo(x + 7, y); ctx.lineTo(x, y + 7); ctx.lineTo(x - 7, y);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    const lbl = opts.label || 'touchdown';
    ctx.fillStyle = SITE; ctx.font = '11px ui-monospace,monospace';
    ctx.fillText(lbl, Math.min(x + 10, w - ctx.measureText(lbl).width - 6),
                 Math.max(y - 8, 26));
  }

  ctx.fillStyle = 'rgba(143,151,168,.65)'; ctx.font = '9px system-ui';
  ctx.textAlign = 'right';
  ctx.fillText('selenographic · 0° lon = sub-Earth meridian · near side centre',
               w - 8, h - 7);
  ctx.textAlign = 'left';
  ctx.fillStyle = '#8B97A8'; ctx.font = '10px ui-monospace,monospace';
  ctx.fillText('Moon ground track', 12, 16);
  cv.title = 'Selenographic ground track: parking orbit, descent ellipse and touchdown';
}
