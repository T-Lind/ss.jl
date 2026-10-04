import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { panelRun } from '../web/src/panel.js';
import { lander, powered_descent, _descent_leg, target_parking } from '../web/src/landing.js';
import { coplanar_moon, moonfixed_inv } from '../web/src/moon.js';
import { R_MOON, MU_MOON } from '../web/src/constants.js';
import { vdot, vunit, vnorm } from '../web/src/vec3.js';
import { _moon_step } from '../web/src/landing.js';

const html=readFileSync(new URL('../web/panel_page.html',import.meta.url),'utf8');
const object=name=>Function('return ('+new RegExp(`const ${name} = (\\{[\\s\\S]*?\\n\\});`).exec(html)[1]+')')();
function saturn() {
  const p={};
  for (const [,attrs] of html.matchAll(/<input\b([^>]+)>/g)) {
    const id=/\bid="([^"]+)"/.exec(attrs)?.[1],value=/\bvalue="([^"]*)"/.exec(attrs)?.[1];
    if(!id)continue;
    if(/type="checkbox"/.test(attrs))p[id]=/\bchecked\b/.test(attrs)?'1':'0';
    else if(value!==undefined)p[id]=value;
  }
  Object.assign(p,object('VEHICLES')['Saturn V (Apollo)'],{mode:'landing',target:'aitken'});
  return p;
}
function params(p) {
  return Object.fromEntries(Object.entries(p).map(([k,v])=>[k,typeof v==='boolean'?(v?'1':'0'):String(v)]));
}

test('fuel failure records the exact final state without an invented high gate', () => {
  const l=lander({mdry:3500,mprop:100,thrust:45000,isp:311,throttle_min:.1});
  const rp=R_MOON+15000,v=Math.sqrt(MU_MOON*(2/rp-1/(.5*(rp+R_MOON+100000))));
  const d=powered_descent(l,[rp,0,0],[0,v,0],3600);
  assert.equal(d.outcome,'propellant');assert.ok(Number.isNaN(d.t_gate));
  assert.equal(d.log.t.at(-1),d.t_touchdown);assert.equal(d.log.m.at(-1),3500);
  assert.ok(d.log.h.at(-1)>0);
  const hit=_descent_leg(l,[R_MOON+1,0,0],[-100,1000,0],3600,0,0);
  assert.equal(hit.outcome,'surface');assert.ok(hit.t>0&&hit.t<.5);assert.ok(Math.abs(hit.h)<1e-6);
});

test('site phasing works at the achieved radius with a finite-perilune residual', () => {
  const eph=coplanar_moon([7e6,0,0],[0,7500,0]),rad=Math.PI/180;
  const u=[Math.cos(-45.5*rad)*Math.cos(177.6*rad),Math.cos(-45.5*rad)*Math.sin(177.6*rad),Math.sin(-45.5*rad)];
  const rp=R_MOON+166000,T=target_parking(eph,0,[rp,0,0],[1,1700,0],100000,15000,u,1);
  assert.ok(Math.abs(T.v_park[0])<1e-9);
  assert.ok(Math.abs(vnorm(T.v_park)-Math.sqrt(MU_MOON/rp))<1e-9);
  let r=[rp,0,0],v=T.v_park;
  for(let i=0;i<6000;i++)[r,v]=_moon_step(r,v,T.wait/6000,{t:i*T.wait/6000});
  assert.ok(vdot(vunit(r),vunit(moonfixed_inv(u,T.t_pdi,eph)))<-.99998);
  assert.ok(Math.abs(vnorm(r)-rp)<1);
});

test('Saturn V default Aitken mission reports an airborne fuel failure coherently', () => {
  const out=panelRun(params(saturn()),'landing');
  assert.equal(out.ok,true);assert.equal(out.metrics.outcome,'propellant');
  assert.equal(out.metrics.on_target,false);assert.equal(out.metrics.prop_left_kg,0);
  assert.ok(out.descent.h.at(-1)>0);assert.ok(!out.events.some(e=>e.name==='high_gate'));
  assert.equal(out.descent.t.at(-1),out.metrics.descent_s);
  assert.ok(Math.abs(out.site.t_td-out.site.t_pdi-out.descent.t.at(-1))<1e-8);
  assert.equal(out.events.at(-1).name,'propellant');assert.equal(out.events.at(-1).t,out.site.t_td);
});

test('Saturn V with the shipped Aitken lander reaches the named site and lands safely', () => {
  const p=Object.assign(saturn(),object('LANDING_PRESETS')['Aitken lander']);
  const out=panelRun(params(p),'landing'),m=out.metrics;
  assert.equal(out.ok,true);assert.equal(m.outcome,'touchdown',JSON.stringify(m));assert.equal(m.on_target,true);
  assert.ok(m.touchdown_v<2);assert.ok(m.touchdown_vh<1.2);assert.ok(m.prop_left_kg>0);
  const rad=Math.PI/180,lat=m.land_lat*rad,tl=m.target_lat*rad;
  const sep=Math.acos(Math.min(1,Math.max(-1,Math.sin(lat)*Math.sin(tl)+Math.cos(lat)*Math.cos(tl)*Math.cos((m.land_lon-m.target_lon)*rad))));
  assert.ok(R_MOON*sep<10000,'lands within 10 km of the selected site');
  assert.ok(Math.abs(out.descent.h.at(-1))<.005);assert.equal(out.events.at(-1).name,'touchdown');
  assert.ok(out.events.some(e=>e.name==='high_gate'));assert.equal(out.descent.t.at(-1),m.descent_s);
});
