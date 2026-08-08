// Interactive, dependency-free Earth ground track. The coastline is a quiet
// contextual layer; the telemetry supplied by the simulator is the data.
const LAND = [
  [[-168,72],[-140,70],[-125,53],[-123,39],[-110,24],[-97,18],[-82,25],[-67,45],[-75,58],[-95,68],[-120,73]],
  [[-81,12],[-73,5],[-76,-15],[-67,-35],[-55,-55],[-39,-25],[-49,-2],[-61,10]],
  [[-18,35],[5,37],[20,32],[33,12],[51,11],[43,-12],[30,-34],[17,-35],[8,-16],[-5,5],[-17,15]],
  [[-10,36],[5,45],[25,60],[60,72],[105,77],[145,66],[178,52],[145,42],[122,20],[104,2],[78,7],[58,25],[35,31],[20,42]],
  [[112,-12],[154,-10],[151,-38],[130,-35],[114,-24]],
  [[-52,60],[-20,82],[-44,84],[-66,73]],
  [[43,-13],[51,-16],[48,-26],[44,-24]],
];

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
const fmtTime = s => s >= 86400 ? `T+${(s/86400).toFixed(2)} d` :
                     s >= 3600 ? `T+${(s/3600).toFixed(2)} h` : `T+${s.toFixed(0)} s`;

function install(canvas) {
  if (canvas.__groundTrack) return canvas.__groundTrack;
  const box = canvas.closest('.track-shell');
  const state = { canvas, box, segments: [], points: [], cursor: 0, playing: false,
                  frame: 0, last: 0 };
  const slider = box.querySelector('[data-track-time]');
  const play = box.querySelector('[data-track-play]');
  const readout = box.querySelector('[data-track-readout]');
  state.slider = slider; state.play = play; state.readout = readout;
  slider.addEventListener('input', () => { state.cursor = +slider.value; draw(state); });
  play.addEventListener('click', () => {
    state.playing = !state.playing;
    play.textContent = state.playing ? 'pause' : 'play';
    if (state.playing) { state.last = performance.now(); tick(state, state.last); }
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
  canvas.__groundTrack = state;
  return state;
}

function tick(s, now) {
  if (!s.playing) return;
  if (now - s.last > 55) {
    s.cursor = (s.cursor + 1) % Math.max(1, s.points.length);
    s.slider.value = s.cursor; s.last = now; draw(s);
  }
  s.frame = requestAnimationFrame(t => tick(s,t));
}

function draw(s) {
  const cv=s.canvas, ctx=cv.getContext('2d'), dpr=devicePixelRatio||1;
  const w=cv.clientWidth, h=cv.clientHeight;
  if (cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)) {
    cv.width=Math.round(w*dpr); cv.height=Math.round(h*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,w,h);
  const xy=(lo,la)=>[(lo+180)/360*w,(90-la)/180*h];
  const grd=ctx.createLinearGradient(0,0,0,h); grd.addColorStop(0,'#0d2132'); grd.addColorStop(1,'#07131e');
  ctx.fillStyle=grd; ctx.fillRect(0,0,w,h);
  ctx.strokeStyle='rgba(143,190,240,.10)'; ctx.lineWidth=1;
  for(let lo=-150;lo<=150;lo+=30){const [x]=xy(lo,0);ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke()}
  for(let la=-60;la<=60;la+=30){const [,y]=xy(0,la);ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}
  for(const poly of LAND){ctx.beginPath();poly.forEach(([lo,la],i)=>{const [x,y]=xy(lo,la);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.closePath();ctx.fillStyle='#1a2b31';ctx.fill();ctx.strokeStyle='#34505a';ctx.stroke()}
  let consumed=0;
  for(const seg of s.segments){
    ctx.strokeStyle=seg.color;ctx.lineWidth=2;ctx.beginPath();let pen=false;
    seg.points.forEach((p,i)=>{const [x,y]=xy(p.lon,p.lat);const prev=seg.points[i-1];
      if(!pen||prev&&Math.abs(prev.lon-p.lon)>180){ctx.moveTo(x,y);pen=true}else ctx.lineTo(x,y)});ctx.stroke();
    const upto=clamp(s.cursor-consumed,0,seg.points.length-1);ctx.strokeStyle='#e9f2ff';ctx.lineWidth=2.5;ctx.beginPath();pen=false;
    seg.points.slice(0,upto+1).forEach((p,i,a)=>{const [x,y]=xy(p.lon,p.lat);const prev=a[i-1];if(!pen||prev&&Math.abs(prev.lon-p.lon)>180){ctx.moveTo(x,y);pen=true}else ctx.lineTo(x,y)});ctx.stroke();
    consumed+=seg.points.length;
  }
  const p=s.points[clamp(s.cursor,0,s.points.length-1)];
  if(p){const [x,y]=xy(p.lon,p.lat);ctx.fillStyle='#f0a500';ctx.shadowColor='#f0a500';ctx.shadowBlur=12;ctx.beginPath();ctx.arc(x,y,4,0,7);ctx.fill();ctx.shadowBlur=0;
    s.readout.textContent=`${p.label} · ${fmtTime(p.t)} · ${Math.abs(p.lat).toFixed(2)}°${p.lat<0?'S':'N'}  ${Math.abs(p.lon).toFixed(2)}°${p.lon<0?'W':'E'}`}
}

export function drawGroundTrack(target, segments) {
  const cv=typeof target==='string'?document.getElementById(target):target;
  if(!cv)return;
  const s=install(cv);
  s.segments=(segments||[]).filter(x=>x.t&&x.lat&&x.lon).map(seg=>({
    ...seg, points:seg.t.map((t,i)=>({t:+t,lat:+seg.lat[i],lon:+seg.lon[i],label:seg.label||'flight'}))
      .filter(p=>Number.isFinite(p.lat)&&Number.isFinite(p.lon))})).filter(x=>x.points.length>1);
  s.points=s.segments.flatMap(x=>x.points);s.cursor=Math.max(0,s.points.length-1);
  s.slider.max=Math.max(0,s.points.length-1);s.slider.value=s.cursor;draw(s);
}
