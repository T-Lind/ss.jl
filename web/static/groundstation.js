// Interactive optical ground station. This is a topocentric camera driven by
// the flown latitude/longitude/altitude samples. Visibility is deliberately
// diagnostic rather than decorative: Earth occultation, atmospheric airmass,
// user-selected cloud opacity and a finite optical range all gate the target.
const D2R=Math.PI/180, R2D=180/Math.PI, RE=6371.0;
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export const wrap180=a=>((a+180)%360+360)%360-180;
const fmtTime=s=>s>=86400?`T+${(s/86400).toFixed(2)} d`:s>=3600?`T+${(s/3600).toFixed(2)} h`:`T+${s.toFixed(0)} s`;

function look(station,p){
  const slat=station.lat*D2R,slon=station.lon*D2R;
  const plat=p.lat*D2R,plon=p.lon*D2R,rr=RE+Math.max(0,p.h||0);
  const sx=RE*Math.cos(slat)*Math.cos(slon),sy=RE*Math.cos(slat)*Math.sin(slon),sz=RE*Math.sin(slat);
  const dx=rr*Math.cos(plat)*Math.cos(plon)-sx,dy=rr*Math.cos(plat)*Math.sin(plon)-sy,dz=rr*Math.sin(plat)-sz;
  const east=-Math.sin(slon)*dx+Math.cos(slon)*dy;
  const north=-Math.sin(slat)*Math.cos(slon)*dx-Math.sin(slat)*Math.sin(slon)*dy+Math.cos(slat)*dz;
  const up=Math.cos(slat)*Math.cos(slon)*dx+Math.cos(slat)*Math.sin(slon)*dy+Math.sin(slat)*dz;
  const range=Math.hypot(east,north,up),el=Math.asin(clamp(up/Math.max(range,1e-9),-1,1))*R2D;
  return {range,el,az:(Math.atan2(east,north)*R2D+360)%360};
}

function install(canvas){
  if(canvas.__station)return canvas.__station;
  const box=canvas.closest('.station-shell');
  const s={canvas,box,samples:[],station:{lat:0,lon:0},cursor:0,auto:true,pan:90,tilt:15,zoom:8,cloud:20};
  for(const k of ['time','pan','tilt','zoom','cloud'])s[k+'El']=box.querySelector(`[data-station-${k}]`);
  s.trackEl=box.querySelector('[data-station-track]');s.readout=box.querySelector('[data-station-readout]');
  s.timeEl.addEventListener('input',()=>{s.cursor=+s.timeEl.value;draw(s)});
  for(const k of ['pan','tilt','zoom','cloud'])s[k+'El'].addEventListener('input',()=>{
    s[k]=+s[k+'El'].value;if(k==='pan'||k==='tilt'){s.auto=false;s.trackEl.classList.remove('on');s.trackEl.textContent='track target'}draw(s);
  });
  s.trackEl.addEventListener('click',()=>{s.auto=!s.auto;s.trackEl.classList.toggle('on',s.auto);s.trackEl.textContent=s.auto?'tracking':'track target';draw(s)});
  let drag=null;
  canvas.addEventListener('pointerdown',e=>{drag={x:e.clientX,y:e.clientY,pan:s.pan,tilt:s.tilt};canvas.setPointerCapture(e.pointerId)});
  canvas.addEventListener('pointermove',e=>{if(!drag)return;const hfov=50/s.zoom;s.auto=false;s.trackEl.classList.remove('on');s.trackEl.textContent='track target';s.pan=(((drag.pan-(e.clientX-drag.x)/canvas.clientWidth*hfov)%360)+360)%360;s.tilt=clamp(drag.tilt+(e.clientY-drag.y)/canvas.clientHeight*hfov,0,90);s.panEl.value=s.pan;s.tiltEl.value=s.tilt;draw(s)});
  canvas.addEventListener('pointerup',()=>drag=null);
  canvas.addEventListener('wheel',e=>{e.preventDefault();s.zoom=clamp(s.zoom*(e.deltaY>0?.88:1.14),1,80);s.zoomEl.value=s.zoom;draw(s)},{passive:false});
  new ResizeObserver(()=>draw(s)).observe(box);canvas.__station=s;return s;
}

function cloudLayer(ctx,w,h,cover){
  if(cover<=0)return;
  ctx.save();ctx.globalAlpha=.07+.24*cover/100;ctx.fillStyle='#c8d7e6';
  for(let i=0;i<24;i++){
    const x=((i*197)%997)/997*w,y=(.08+((i*83)%271)/271*.62)*h;
    const rx=(35+(i*29)%90)*(w/900),ry=10+(i*17)%25;
    ctx.beginPath();ctx.ellipse(x,y,rx,ry,0,0,Math.PI*2);ctx.fill();
  }
  ctx.restore();
}

function draw(s){
  const cv=s.canvas,ctx=cv.getContext('2d'),dpr=devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;
  if(cv.width!==Math.round(w*dpr)||cv.height!==Math.round(h*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr)}
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const p=s.samples[clamp(s.cursor,0,s.samples.length-1)],v=p?look(s.station,p):null;
  if(v&&s.auto){s.pan=v.az;s.tilt=clamp(v.el,0,90);s.panEl.value=s.pan;s.tiltEl.value=s.tilt}
  const hfov=50/s.zoom,vfov=hfov*h/w;
  const sky=ctx.createLinearGradient(0,0,0,h);sky.addColorStop(0,'#020914');sky.addColorStop(1,'#16334b');ctx.fillStyle=sky;ctx.fillRect(0,0,w,h);
  // Camera reticle and azimuth/elevation grid.
  ctx.strokeStyle='rgba(143,190,240,.13)';ctx.lineWidth=1;
  for(let i=1;i<4;i++){ctx.beginPath();ctx.moveTo(i*w/4,0);ctx.lineTo(i*w/4,h);ctx.stroke();ctx.beginPath();ctx.moveTo(0,i*h/4);ctx.lineTo(w,i*h/4);ctx.stroke()}
  const hy=h/2+s.tilt/vfov*h;
  if(hy>-20&&hy<h+20){ctx.fillStyle='#06100e';ctx.fillRect(0,hy,w,h-hy);ctx.strokeStyle='#4c735f';ctx.beginPath();ctx.moveTo(0,hy);ctx.lineTo(w,hy);ctx.stroke()}
  const haze=ctx.createLinearGradient(0,clamp(hy-90,0,h),0,clamp(hy+15,0,h));haze.addColorStop(0,'rgba(145,183,209,0)');haze.addColorStop(1,'rgba(145,183,209,.24)');ctx.fillStyle=haze;ctx.fillRect(0,clamp(hy-90,0,h),w,105);
  cloudLayer(ctx,w,h,s.cloud);
  ctx.strokeStyle='rgba(240,165,0,.65)';ctx.beginPath();ctx.moveTo(w/2-12,h/2);ctx.lineTo(w/2+12,h/2);ctx.moveTo(w/2,h/2-12);ctx.lineTo(w/2,h/2+12);ctx.stroke();

  let status='no trajectory sample',signal=0;
  if(v){
    const airmass=v.el>0?1/(Math.sin(v.el*D2R)+.50572*Math.pow(v.el+6.07995,-1.6364)):Infinity;
    const atmosphere=v.el>0?Math.exp(-.12*Math.max(0,airmass-1)):0;
    const cloudTx=1-.98*s.cloud/100,rangeTx=1/(1+(v.range/1200)**2);
    signal=atmosphere*cloudTx*rangeTx;
    const withinRange=v.range<8000&&signal*Math.sqrt(s.zoom)>=.045;
    const x=w/2+wrap180(v.az-s.pan)/hfov*w,y=h/2-(v.el-s.tilt)/vfov*h;
    const inFrame=x>=0&&x<=w&&y>=0&&y<=h;
    if(v.el<=0)status='below the geometric horizon';
    else if(s.cloud>=92)status='optically blocked by cloud';
    else if(!withinRange)status='below optical detection threshold';
    else if(!inFrame)status='visible, outside camera field';
    else{
      status='optical track';const glow=clamp(4+18*signal*Math.sqrt(s.zoom),4,18);
      ctx.fillStyle='#f7d36a';ctx.shadowColor='#f0a500';ctx.shadowBlur=glow;ctx.beginPath();ctx.arc(x,y,clamp(2+signal*s.zoom,2,7),0,Math.PI*2);ctx.fill();ctx.shadowBlur=0;
      ctx.strokeStyle='#fff';ctx.strokeRect(x-9,y-9,18,18);
    }
    s.readout.textContent=`${status} · ${fmtTime(p.t)} · az ${v.az.toFixed(1)}° · el ${v.el.toFixed(1)}° · ${v.range.toFixed(0)} km · transmission ${(100*signal).toFixed(0)}%`;
  }else s.readout.textContent=status;
  ctx.fillStyle='#9eabc0';ctx.font='11px ui-monospace,monospace';ctx.fillText(`AZ ${s.pan.toFixed(1)}°  EL ${s.tilt.toFixed(1)}°  ${s.zoom.toFixed(0)}×  FOV ${hfov.toFixed(2)}°`,12,20);
  ctx.textAlign='right';ctx.fillText(`${Math.abs(s.station.lat).toFixed(2)}°${s.station.lat<0?'S':'N'}  ${Math.abs(s.station.lon).toFixed(2)}°${s.station.lon<0?'W':'E'}`,w-12,20);ctx.textAlign='left';
}

export function drawGroundStation(target,segments,station){
  const cv=typeof target==='string'?document.getElementById(target):target;if(!cv)return;
  const s=install(cv);s.station={lat:+station.lat||0,lon:+station.lon||0};
  s.samples=(segments||[]).flatMap(seg=>(seg.t||[]).map((t,i)=>({t:+t,lat:+seg.lat[i],lon:+seg.lon[i],h:+(seg.h||[])[i]||0,label:seg.label||'flight'})))
    .filter(p=>Number.isFinite(p.t)&&Number.isFinite(p.lat)&&Number.isFinite(p.lon)&&Number.isFinite(p.h)).sort((a,b)=>a.t-b.t);
  s.cursor=Math.max(0,s.samples.findIndex(p=>p.h>.5));
  s.timeEl.max=Math.max(0,s.samples.length-1);s.timeEl.value=s.cursor;
  s.trackEl.classList.toggle('on',s.auto);s.trackEl.textContent=s.auto?'tracking':'track target';draw(s);
}
