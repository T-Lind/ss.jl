// Interactive Earth ground track backed by bundled Natural Earth 1:110m
// country geometry. Natural Earth data is public domain and is kept local so
// the map is equally useful in the offline desktop executable.
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const fmtTime = s => s >= 86400 ? `T+${(s/86400).toFixed(2)} d` :
                     s >= 3600 ? `T+${(s/3600).toFixed(2)} h` : `T+${s.toFixed(0)} s`;
const PLAY_SECONDS = 24;   // one full pass of the track in the Play button

let WORLD = [];
const STATES = new Set();
// Guarded so the module can be imported outside a browser (the Node helper
// tests import `mapPath` from here) without firing a relative-URL fetch.
if (typeof document !== 'undefined') {
  fetch('/static/ne_110m_admin_0_countries.geojson')
    .then(r => { if (!r.ok) throw new Error(`map data: HTTP ${r.status}`); return r.json(); })
    .then(j => {
      WORLD = (j.features || []).flatMap(f => {
        const g = f.geometry || {};
        if (g.type === 'Polygon') return [g.coordinates];
        if (g.type === 'MultiPolygon') return g.coordinates;
        return [];
      });
      STATES.forEach(draw);
    })
    .catch(e => console.warn('Natural Earth map unavailable', e));
}

function install(canvas) {
  if (canvas.__groundTrack) return canvas.__groundTrack;
  const box = canvas.closest('.track-shell');
  const state = { canvas, box, segments: [], points: [], station: null, cursor: 0,
                  playing: false, frame: 0, last: 0 };
  const slider = box.querySelector('[data-track-time]');
  const play = box.querySelector('[data-track-play]');
  const readout = box.querySelector('[data-track-readout]');
  state.slider = slider; state.play = play; state.readout = readout;
  slider.addEventListener('input', () => { state.cursor = +slider.value; draw(state); });
  play.addEventListener('click', () => {
    // Play runs forward from wherever the cursor is, and pressing it at the end
    // starts over rather than sitting there.
    if (!state.playing && state.cursor >= state.points.length - 1) state.cursor = 0;
    state.playing = !state.playing;
    play.textContent = state.playing ? 'pause' : 'play';
    if (state.playing) { state.last = performance.now(); state.acc = 0; tick(state, state.last); }
    else cancelAnimationFrame(state.frame);
  });
  canvas.addEventListener('pointermove', e => {
    const r = canvas.getBoundingClientRect();
    const lon = (e.clientX-r.left)/r.width*360-180;
    const lat = 90-(e.clientY-r.top)/r.height*180;
    let best = 0, bd = Infinity;
    state.points.forEach((p,i) => {
      const dl = Math.min(Math.abs(p.lon-lon), 360-Math.abs(p.lon-lon));
      const d = dl*dl*Math.cos(lat*Math.PI/180)**2 + (p.lat-lat)**2;
      if (d < bd) { bd=d; best=i; }
    });
    state.cursor = best; slider.value = best; draw(state);
  });
  new ResizeObserver(() => draw(state)).observe(box);
  canvas.__groundTrack = state; STATES.add(state);
  return state;
}

function tick(s, now) {
  if (!s.playing) return;
  const n = s.points.length;
  if (n > 1) {
    // Time-based, and paced so the whole track plays in about PLAY_SECONDS no
    // matter how many samples it holds: one point per 55 ms took minutes over a
    // multi-day coast, and wrapping at the end restarted without warning.
    const dt = Math.min(0.25, (now - s.last) / 1000);
    s.last = now;
    s.acc = (s.acc || 0) + dt * (n / PLAY_SECONDS);
    const adv = Math.floor(s.acc);
    if (adv > 0) {
      s.acc -= adv;
      s.cursor = Math.min(n - 1, s.cursor + adv);
      s.slider.value = s.cursor; draw(s);
      if (s.cursor >= n - 1) { s.playing = false; s.play.textContent = 'play'; return; }
    }
  } else { s.last = now; }
  s.frame = requestAnimationFrame(t => tick(s,t));
}

export function mapPath(ctx, rings, xy) {
  let drawable = false, split = false;
  for (const ring of rings) {
    let pen = false, prev = null;
    for (const pair of ring) {
      const lo=+pair[0], la=+pair[1], [x,y]=xy(lo,la);
      if (!pen || (prev && Math.abs(prev-lo)>180)) {
        if(prev)split=true;ctx.moveTo(x,y);pen=true;
      }
      else ctx.lineTo(x,y);
      prev=lo; drawable=true;
    }
    // Natural Earth rings are closed already. Do not bridge a dateline split
    // back across the whole canvas: that was the source of giant false land.
    if (ring.length>2 && !ring.some((p,i)=>i&&Math.abs(p[0]-ring[i-1][0])>180))
      ctx.closePath();
  }
  return {drawable,split};
}

function draw(s) {
  const cv=s.canvas, ctx=cv.getContext('2d'), dpr=devicePixelRatio||1;
  const w=cv.clientWidth, h=cv.clientHeight;
  if (cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)) {
    cv.width=Math.round(w*dpr); cv.height=Math.round(h*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,w,h);
  const xy=(lo,la)=>[(lo+180)/360*w,(90-la)/180*h];
  const grd=ctx.createLinearGradient(0,0,0,h);
  grd.addColorStop(0,'#0d2132'); grd.addColorStop(1,'#07131e');
  ctx.fillStyle=grd; ctx.fillRect(0,0,w,h);
  ctx.strokeStyle='rgba(143,190,240,.10)'; ctx.lineWidth=1;
  for(let lo=-150;lo<=150;lo+=30){const [x]=xy(lo,0);ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke()}
  for(let la=-60;la<=60;la+=30){const [,y]=xy(0,la);ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}
  ctx.fillStyle='#1a2b31'; ctx.strokeStyle='#3b5964'; ctx.lineWidth=.7;
  for(const poly of WORLD){ctx.beginPath();const p=mapPath(ctx,poly,xy);if(p.drawable){if(!p.split)ctx.fill('evenodd');ctx.stroke()}}
  if(!WORLD.length){ctx.fillStyle='#6f8197';ctx.font='12px system-ui';ctx.fillText('loading Natural Earth map…',14,22)}

  let consumed=0;
  for(const seg of s.segments){
    ctx.strokeStyle=seg.color;ctx.lineWidth=2;ctx.beginPath();let pen=false;
    seg.points.forEach((p,i)=>{const [x,y]=xy(p.lon,p.lat),prev=seg.points[i-1];
      if(!pen||prev&&Math.abs(prev.lon-p.lon)>180){ctx.moveTo(x,y);pen=true}else ctx.lineTo(x,y)});ctx.stroke();
    const upto=clamp(s.cursor-consumed,0,seg.points.length-1);ctx.strokeStyle='#e9f2ff';ctx.lineWidth=2.5;ctx.beginPath();pen=false;
    seg.points.slice(0,upto+1).forEach((p,i,a)=>{const [x,y]=xy(p.lon,p.lat),prev=a[i-1];if(!pen||prev&&Math.abs(prev.lon-p.lon)>180){ctx.moveTo(x,y);pen=true}else ctx.lineTo(x,y)});ctx.stroke();
    consumed+=seg.points.length;
  }
  if(s.station&&Number.isFinite(s.station.lat)&&Number.isFinite(s.station.lon)){
    const [x,y]=xy(s.station.lon,s.station.lat);
    ctx.fillStyle='#8fbef0';ctx.strokeStyle='#07131e';ctx.lineWidth=1.5;ctx.beginPath();
    ctx.moveTo(x,y-6);ctx.lineTo(x+5,y+5);ctx.lineTo(x-5,y+5);ctx.closePath();ctx.fill();ctx.stroke();
  }
  const p=s.points[clamp(s.cursor,0,s.points.length-1)];
  if(p){const [x,y]=xy(p.lon,p.lat);ctx.fillStyle='#f0a500';ctx.shadowColor='#f0a500';ctx.shadowBlur=12;ctx.beginPath();ctx.arc(x,y,4,0,7);ctx.fill();ctx.shadowBlur=0;
    s.readout.textContent=`${p.label} · ${fmtTime(p.t)} · ${Math.abs(p.lat).toFixed(2)}°${p.lat<0?'S':'N'}  ${Math.abs(p.lon).toFixed(2)}°${p.lon<0?'W':'E'}`}
  ctx.fillStyle='rgba(143,151,168,.65)';ctx.font='9px system-ui';ctx.textAlign='right';
  ctx.fillText('Natural Earth 1:110m · public domain',w-8,h-7);ctx.textAlign='left';
}

export function drawGroundTrack(target, segments, station=null) {
  const cv=typeof target==='string'?document.getElementById(target):target;
  if(!cv)return;
  const s=install(cv); s.station=station;
  s.segments=(segments||[]).filter(x=>x.t&&x.lat&&x.lon).map(seg=>({
    ...seg, points:seg.t.map((t,i)=>({t:+t,lat:+seg.lat[i],lon:+seg.lon[i],h:+(seg.h||[])[i]||0,label:seg.label||'flight'}))
      .filter(p=>Number.isFinite(p.lat)&&Number.isFinite(p.lon))})).filter(x=>x.points.length>1);
  const sig=s.segments.map(x=>x.points.length+'@'+x.points[0].t).join('|');
  s.points=s.segments.flatMap(x=>x.points);
  // Start at the beginning so Play runs the flight forward, and hold the cursor
  // across redraws of the SAME track: renderCharts runs on every resize, and
  // resetting to the end there is what made Play jump back to the start.
  if (s.sig !== sig) { s.cursor = 0; s.sig = sig; }
  s.cursor = Math.min(Math.max(0, s.cursor), Math.max(0, s.points.length-1));
  s.slider.max=Math.max(0,s.points.length-1);s.slider.value=s.cursor;draw(s);
}
