import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createContext, runInContext } from 'node:vm';
import * as flight from '../web/static/flight_state.js';
import { fin, fmt } from '../web/static/fmt.js';
import { escapeHTML } from '../web/static/ui.js';
import { describe, verdict } from '../web/static/runs.js';
import { landerLayout } from '../web/static/lander_model.js';

const source = readFileSync(new URL('../web/launch_page.html', import.meta.url), 'utf8');
function fn(name, text = source) {
  const start = text.indexOf(`function ${name}(`);
  assert.ok(start >= 0, `missing ${name}`);
  return text.slice(start, text.indexOf('\n}', start) + 2);
}
function scene() {
  const c = createContext({ Math, Number, Float32Array, RM: 1737400, VST: 12,
    MS: {u:[1,0,0], ed:[0,1,0], ec:[0,0,1], terrain:{}},
    terrainHeight: (_tr, u) => 100 + 1000*u[1] - 500*u[2], buffer: data => data });
  runInContext(source.slice(source.indexOf('const V = {'), source.indexOf('// --------------------------------------------------------------- gl setup')), c);
  for (const name of ['moonDir','moonPos','liftMoonCamera','buildMoonTerrain']) runInContext(fn(name), c);
  return c;
}
const sub = (a,b) => a.map((v,i) => v-b[i]);
const dot = (a,b) => a.reduce((s,v,i) => s+v*b[i],0);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];

test('lunar terrain faces the sky with finite outward normals, including the site centre', () => {
  const rings = scene().buildMoonTerrain({N:8, extents:[40,160,640]});
  for (const {buf:data} of rings) for (let i=0;i<data.length;i+=36) {
    const a=Array.from(data.slice(i,i+3)), b=Array.from(data.slice(i+12,i+15)), d=Array.from(data.slice(i+24,i+27));
    const face=cross(sub(b,a),sub(d,a));
    assert.ok(dot(face,[a[0],a[1]+1737400,a[2]])>0, 'front face must face away from Moon centre');
    for (const j of [i,i+12,i+24]) {
      const n=Array.from(data.slice(j+3,j+6));
      assert.ok(n.every(Number.isFinite));
      assert.ok(Math.abs(Math.hypot(...n)-1)<1e-6);
      assert.ok(dot(face,n)>0, 'lighting normal agrees with front face');
    }
  }
});

test('far-downrange camera clearance preserves direction and respects actual terrain', () => {
  const c=scene(), x=700000,z=300000;
  const u=c.moonDir(x,z), height=c.terrainHeight({},u);
  const low=c.moonPos(x,z,height-10), high=c.moonPos(x,z,height+100);
  const raised=c.liftMoonCamera(low), radial=p=>[p[0],p[1]+c.RM,p[2]];
  assert.ok(Math.abs(Math.hypot(...radial(raised))-(c.RM+height+1.2))<1e-6);
  assert.ok(Math.hypot(...cross(radial(low),radial(raised)))/(c.RM*c.RM)<1e-12);
  assert.equal(c.liftMoonCamera(high),high);
});

test('ground camera keeps the approaching lander visible without a 25m aim snap', () => {
  const c=scene();
  Object.assign(c,{SURF:{active:false},camMode:'ground',MDIA:4.2,MOONCAM:{x:100,z:0,y:103},
    landerLayout:()=>({s:1}),landerRig:()=>null});
  runInContext('const groundY=(x,z)=>moonPos(x,z,terrainHeight(MS.terrain,moonDir(x,z)))[1];',c);
  runInContext(fn('cameraMoon'),c);
  runInContext(fn('fitFov'),c);
  runInContext(fn('fovZoom'),c);
  let previous;
  for(const h of [1000,600,300,100,26,25,24,10,0]) {
    const pos=c.moonPos(0,0,100+h),ms={pos,up:[0,1,0],fwd:[1,0,0],thrust:[0,1,0],h,thr:0};
    const cam=c.cameraMoon(0,ms,null),direction=sub(cam.look,cam.eye);
    cam.fov=c.fovZoom(cam.fov,1);
    const length=a=>Math.hypot(...a), unit=a=>a.map(x=>x/length(a));
    const axis=unit(direction);
    for(const target of h===0 ? [[0,100,0],[pos[0],pos[1]+2.65,pos[2]]] : [[pos[0],pos[1]+2.65,pos[2]]]) {
      const angle=Math.acos(Math.max(-1,Math.min(1,dot(axis,unit(sub(target,cam.eye))))));
      assert.ok(angle<cam.fov/2,`altitude ${h}: craft stays in view, with terrain at contact`);
      if(target[1]>100)assert.ok(Math.tan(angle)/Math.tan(cam.fov/2)<0.75,'lander clears the top HUD');
    }
    const distance=length(sub([pos[0],pos[1]+2.65,pos[2]],cam.eye));
    assert.ok(600*landerLayout().height/(2*distance*Math.tan(cam.fov/2))>35,'lander must be more than a few pixels tall');
    if(previous && h>=24 && h<=25)assert.ok(length(sub(axis,previous))<0.01,'aim stays continuous across 25m');
    previous=axis;
  }
  assert.equal(c.fovZoom(2.1,1),2.1,'render zoom must preserve the ground rig lens');
  assert.equal(c.fovZoom(1.2,0.4),1.5,'ordinary zoom-out retains its limit');
});

test('chase frames the whole lander clear of the HUD during approach', () => {
  const c=scene();Object.assign(c,{MDIA:4.2,landerLayout});
  for(const name of ['landerRig','fitFov','fovZoom'])runInContext(fn(name),c);
  const unit=a=>a.map(x=>x/Math.hypot(...a));
  for(const h of [1000,500,353,100,25,5,0]) {
    const cam=c.landerRig('chase',[0,h,0],[0,1,0],[1,0,0],[0,1,0],h,0,null);
    const forward=unit(sub(cam.look,cam.eye)),right=unit(cross(forward,cam.up)),up=cross(right,forward);
    const tan=Math.tan(c.fovZoom(cam.fov,1)/2);
    for(const x of [-2.1,2.1])for(const y of [0,landerLayout().height])for(const z of [-2.1,2.1]) {
      const ray=sub([x,h+y,z],cam.eye),depth=dot(ray,forward);
      const px=dot(ray,right)/(depth*tan),py=dot(ray,up)/(depth*tan);
      assert.ok(depth>0 && Math.abs(px)<1 && py>-1 && py<0.75,`altitude ${h}: hull stays in frame below HUD`);
    }
  }
});

test('entry chase frames the deployed canopy and capsule together', () => {
  const c=scene();Object.assign(c,{camMode:'chase',RE:6378137,tDrog:50,tMain:90,
    podRange:()=>({x0:0}),capStations:()=>({xtop:2.2})});
  for(const name of ['cameraEntry','fitFov','fovZoom'])runInContext(fn(name),c);
  const unit=a=>a.map(x=>x/Math.hypot(...a));
  const es={pos:[0,1000,0],up:[0,1,0],vdir:[0,-1,0],h:1000,g:0};
  for(const [t,extent,radius] of [[53,2.2+3.2+1.15,1.15],[93,2.2+8.5+4.6,4.6]]) {
    const cam=c.cameraEntry(t,es,null),axis=unit(sub(cam.look,cam.eye));
    const right=unit(cross(axis,cam.up)),up=cross(right,axis),tan=Math.tan(c.fovZoom(cam.fov,1)/2);
    for(const y of [0,extent])for(const x of [-radius,radius])for(const z of [-radius,radius]) {
      const ray=sub([x,1000+y,z],cam.eye),depth=dot(ray,axis);
      assert.ok(depth>0 && Math.abs(dot(ray,right)/(depth*tan))<1 &&
        Math.abs(dot(ray,up)/(depth*tan))<0.75,'canopy and capsule stay clear of frame edges and the HUD');
    }
  }
  assert.equal(c.cameraEntry(50,es,null).fov,0.3,'deployment starts without a lens jump');
});

test('only confirmed touchdown receives surface exploration time', () => {
  for (const outcome of ['propellant','timeout','throttle_limited','crash','tipped','diverged']) {
    const run={metrics:{outcome},site:{t_td:12345}};
    assert.equal(flight.landingEndTime(run),12345);
    assert.notEqual(flight.lunarEndLabel(run),'SURFACE');
  }
  assert.equal(flight.landingEndTime({metrics:{outcome:'touchdown'},site:{t_td:12345}}),12661);
  const summary=flight.landingSummary({metrics:{outcome:'propellant',loi_dv:3033,doi_dv:19,lander_dv:3882},descent:{h:[15,3.28]}});
  assert.match(summary,/3280 m above ground/);
  assert.match(summary,/830 m\/s remained/);
  assert.match(summary,/no touchdown/);
  assert.match(flight.landingSummary({metrics:{outcome:'crash',touchdown_v:166.7,touchdown_vh:142.7}}),/166.7 m\/s sink, 142.7 m\/s lateral/);
});

test('landing history distinguishes failure and lander fuel from launcher margin', () => {
  const d={mode:'landing',outcome:'?',metrics:{outcome:'propellant',touchdown_v:330,prop_left_kg:0,prop_margin_kg:35000}};
  assert.match(describe(d),/propellant depleted/);assert.doesNotMatch(describe(d),/touchdown|35000/);
  assert.match(describe(d),/0 kg lander fuel/);assert.equal(verdict(d),'failed');
  d.metrics.outcome='touchdown';d.metrics.touchdown_v=.7;d.metrics.prop_left_kg=149;
  assert.match(describe(d),/touchdown 0.7 m\/s/);assert.match(describe(d),/149 kg lander fuel/);
  assert.equal(verdict(d),'nominal');
});

test('analysis does not invent high gate or a terminal phase for a braking failure', () => {
  const plots=new Map(), elements=new Map();
  const $=id=>{
    if(!elements.has(id))elements.set(id,{classList:{toggle(){}},innerHTML:''});
    return elements.get(id);
  };
  const c=createContext({ ...flight, fin, fmt, escapeHTML, $, MODE:'landing', ACC:'#f2c14e',
    drawLine:(id,series,title,options)=>plots.set(id,{series,options}),
    run:{mode:'landing',metrics:{outcome:'propellant',gate_s:null},
      descent:{t:[0,1,2],dr:[0,1,2],h:[15,10,3],v:[],vh:[],vv:[],thr:[],m:[],pitch:[],elev:[],navdh:[]}} });
  const analysis=readFileSync(new URL('../web/analysis_page.html',import.meta.url),'utf8');
  runInContext(fn('drawDescent',analysis),c);c.drawDescent();
  assert.equal(plots.get('c_desc_prof').series[0].xs.length,3);
  assert.equal(plots.get('c_desc_prof').series[1].xs.length,0);
  assert.deepEqual(Array.from(plots.get('c_desc_prof').options.marks,m=>m.label),['propellant depleted']);
  assert.match($('desc_note').innerHTML,/no touchdown/);
});

test('terminal card inspection cannot accidentally restart playback', () => {
  const elements=new Map(), $=id=>{
    if(!elements.has(id))elements.set(id,{classList:{remove(){},add(){}},style:{}});
    return elements.get(id);
  };
  let replayed=0;
  const c=createContext({...flight,fin,$,MS:{},tSplash:NaN,EVS:[],scorch:0,
    run:{mode:'landing',metrics:{outcome:'timeout'},descent:{h:[.071]}},seekTo:()=>replayed++});
  runInContext(fn('showEndCard'),c);c.showEndCard();
  assert.equal($('overlay').onclick,null);
  assert.equal($('ov_title').textContent,'DESCENT TIMED OUT');
  assert.match($('ov_sub').textContent,/71 m above ground/);
  assert.equal(replayed,0);
  $('ov_close').onclick({stopPropagation(){}});
  assert.equal(replayed,0,'inspection must preserve the terminal clock');
  $('ov_go').onclick({stopPropagation(){}});
  assert.equal(replayed,1);
});

test('terminal timeline position stays finite and a short arrow tap rewinds immediately', () => {
  const PH=[{n:'DESCENT',t0:0,t1:100},{n:'PROPELLANT DEPLETED',t0:100,t1:100}];
  let jumped;
  const c=createContext({Math,PH,simT:100,scrubDir:0,scrubT0:0,performance:{now:()=>50},jumpTo:t=>jumped=t});
  for (const name of ['phaseAt','phaseFraction','segX','scrubStep','startScrub'])runInContext(fn(name),c);
  assert.equal(c.segX(100,800),800);
  c.startScrub(-1);
  assert.ok(Number.isFinite(jumped));
  // The real jumpTo clamps a seek to half a second before the endpoint.
  c.simT=99.5;c.startScrub(-1);
  assert.ok(jumped<99.5,'tap must work without waiting for requestAnimationFrame');
});

test('fast playback stops at the exact endpoint and renders its final phase', () => {
  const noop=()=>{},PH=[{n:'DESCENT',t0:0,t1:100,w:'moon',warp:()=>10000},
    {n:'PROPELLANT DEPLETED',t0:100,t1:100,w:'moon',warp:()=>1}];
  let cards=0,painted;
  const c=createContext({Math,PH,simT:99,tEnd:100,lastNow:0,started:true,running:true,ended:false,
    manualWarp:10000,warpCur:10000,warpEff:1,toasts:[],dropT:{},tSplash:NaN,splashed:false,
    camMode:'chase',EVA:{active:false},SURF:{active:false},zoomK:1,PARTWARP:300,parts:[],debris:[],
    requestAnimationFrame:noop,perfPush:noop,scrubStep:noop,audioEvents:noop,rcsStep:noop,cabStep:noop,
    rcsActive:()=>false,rcsMustAlign:()=>false,coastActive:()=>false,landed:()=>false,
    buildScene:()=>({world:'moon',cam:{fov:60,eye:[0,0,0]}}),refreshCams:noop,eciLanderPhase:()=>false,
    fovZoom:f=>f,updateAudio:noop,drawScene:noop,drawHUD:(_s,p)=>painted=p.n,drawSeek:noop,
    showEndCard:()=>cards++,stepDebris:noop,stepParticles:noop});
  runInContext(fn('phaseAt'),c);runInContext(fn('frame'),c);c.frame(50);
  assert.equal(c.simT,100);assert.equal(c.running,false);assert.equal(c.ended,true);
  assert.equal(painted,'PROPELLANT DEPLETED');assert.equal(cards,1);
  c.frame(100);assert.equal(c.simT,100);assert.equal(cards,1);
});

test('minimum throttle failures are clear in history, terminal state and design warnings',()=>{
  const run={mode:'landing',metrics:{outcome:'throttle_limited'},descent:{h:[.078]},events:[{phase:'lunar',name:'throttle_limited'}]};
  assert.equal(flight.lunarEndLabel(run),'THROTTLE LIMIT');assert.equal(flight.successfulTouchdown(run),false);
  assert.match(flight.landingSummary(run),/78 m above ground/);assert.match(flight.landingSummary(run),/Lower minimum throttle or increase lander dry mass/);
  assert.equal(flight.lunarHatchReason(run),'minimum throttle too high');assert.equal(verdict(run),'failed');
  assert.equal(flight.landingOutcome({events:run.events}),'throttle_limited');
  assert.match(flight.landerThrottleWarning(100,45,10),/2,770 kg/);assert.equal(flight.landerThrottleWarning(3500,45,10),'');
});

test('legacy playback drops every booster no later than its first-stage attachment',()=>{
  const start=source.indexOf('  BNAMES.forEach((nm, i) => {'),end=source.indexOf('\n  });',start)+7;
  assert.ok(start>=0&&end>start);
  const c=createContext({BNAMES:['late','early','cold'],dropT:{stage1:15},
    BSEP:[{name:'sep_early',t:5},{name:'sep_late',t:60}],SEPS:[{name:'sep_sable1',t:15}]});
  runInContext(source.slice(start,end),c);
  assert.equal(c.dropT.booster1,15);assert.equal(c.dropT.booster2,5);assert.equal(c.dropT.booster3,15);
});
