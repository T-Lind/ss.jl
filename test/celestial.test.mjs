import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { earthAngle, sunEci, sunAtSite, daylight, sunVisibility, rotateZ, dot, moonAxes, moonOrientation,
  lunarSun, lunarEarthMatrix, moonTexel } from '../web/static/celestial.js';
import { landingOutcome, successfulTouchdown, lunarEndLabel, lunarHatchReason } from '../web/static/flight_state.js';
const close=(a,b,tol=1e-9)=>assert.ok(Math.abs(a-b)<tol,`${a} != ${b}`);

test('launch hour produces noon, sunset, midnight and sunrise at Greenwich',()=>{
  for(const [hour,vertical,east] of [[0,1,0],[6,0,-1],[12,-1,0],[18,0,1]]) {
    const s=sunAtSite(0,hour,0,0);
    close(s[1],vertical,.002);close(s[0],east,.001);close(Math.hypot(...s),1);
  }
  close(daylight(sunAtSite(0,0,0,0)),1);
  close(daylight(sunAtSite(0,12,0,0)),0);
  close(daylight(sunAtSite(0,6,0,0)),.5,.01);
});
test('site lighting agrees with inertial Sun and absolute Earth rotation',()=>{
  for(const lat of [-90,-45,0,28.5,90])for(const lon of [-180,-80.6,0,120,180]) {
    const t=250000,h=9,s=sunAtSite(t,h,lat,lon),la=lat*Math.PI/180,lo=lon*Math.PI/180;
    const up=rotateZ([Math.cos(la)*Math.cos(lo),Math.cos(la)*Math.sin(lo),Math.sin(la)],earthAngle(t,h));
    close(s[1],dot(up,sunEci(t,h)));close(Math.hypot(...s),1);
  }
  assert.ok(sunAtSite(0,0,28.5,-80.6)[1]>0); // Florida, morning at this reference hour
  assert.ok(sunAtSite(0,12,28.5,-80.6)[1]<0);
  assert.deepEqual(sunEci(3600,4),sunEci(0,5));
});
test('Moon site lighting and Earth texture transform share the Moon-fixed basis',()=>{
  const p=[300000,-200000,0],n=[0,0,1],site={ed:[0,1,0],u:[1,0,0],ec:[0,0,1]};
  const axes=moonAxes(p,n),sun=sunEci(260000,6),local=lunarSun(260000,6,p,n,site);
  const matrix=lunarEarthMatrix(p,n,site);
  const restored=[0,1,2].map(i=>matrix[i]*local[0]+matrix[i+3]*local[1]+matrix[i+6]*local[2]);
  restored.forEach((v,i)=>close(v,sun[i]));
  close(dot(axes[0],axes[2]),0);close(dot(axes[0],axes[1]),0);
  assert.deepEqual(moonOrientation(p,n),[0,1,2].map(i=>axes.map(a=>a[i])));
});
test('lunar map wraps the longitude seam and places north above south',()=>{
  assert.deepEqual(moonTexel([0,0,1],360,180),[180,0]);
  assert.deepEqual(moonTexel([0,0,-1],360,180),[180,179]);
  assert.deepEqual(moonTexel([1,0,0],360,180),[180,90]);
  assert.deepEqual(moonTexel([-1,0,0],360,180),[0,90]);
  assert.deepEqual(moonTexel([-1,-0,0],360,180),[0,90]);
});
test('failed descents never masquerade as SURFACE; legacy events retain touchdown',()=>{
  for(const result of ['crash','tipped','propellant','timeout','diverged']) {
    const run={metrics:{outcome:result},events:[{phase:'lunar',name:'touchdown'}]};
    assert.equal(successfulTouchdown(run),false);assert.notEqual(lunarEndLabel(run),'SURFACE');
    assert.notEqual(lunarHatchReason(run),'descent');
  }
  const run={events:[{name:'liftoff'},{phase:'lunar',name:'touchdown'}]};
  assert.equal(landingOutcome(run),'touchdown');assert.equal(lunarEndLabel(run),'SURFACE');
  assert.equal(successfulTouchdown({}),false);
});
test('both frontends ship identical celestial and flight-state modules',()=>{
  for(const file of ['celestial.js','flight_state.js'])assert.equal(
    readFileSync(new URL('../web/static/'+file,import.meta.url),'utf8'),
    readFileSync(new URL('../scripts/static/'+file,import.meta.url),'utf8'));
});

test('the Earth blocks direct sunlight at night and during orbital eclipse',()=>{
  const sun=[1,0,0],centre=[0,0,0],R=6371000;
  assert.equal(sunVisibility(sun,[R+100,0,0],centre,R),1);
  assert.equal(sunVisibility(sun,[-R-100,0,0],centre,R),0);
  assert.equal(sunVisibility(sun,[-R-400000,0,0],centre,R),0);
  assert.equal(sunVisibility(sun,[0,R+400000,0],centre,R),1);
  close(sunVisibility(sun,[0,R,0],centre,R),.5);
});
