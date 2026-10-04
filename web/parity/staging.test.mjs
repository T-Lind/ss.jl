import {test} from 'node:test';
import assert from 'node:assert/strict';
import {juliaGolden} from './julia_golden.mjs';
import {fileURLToPath} from 'node:url';
import {stage,boosterSet,launchVehicle,LV_CD_TABLE} from '../src/propulsion.js';
import {ascentGuidance,simulate_ascent} from '../src/launch.js';
import {lander,powered_descent} from '../src/landing.js';
import {R_MOON} from '../src/constants.js';
const g=juliaGolden(fileURLToPath(new URL('./emit_staging.jl',import.meta.url)));
const close=(a,b,label,tol=1e-6)=>assert.ok(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);

test('booster attachment transitions match Julia mass, thrust and event timing',t=>{
 if(!g)return t.skip('Julia unavailable');
 for(const [i,[prop,delay,sep]] of [[10000,0,0],[10000,16,0],[400,0,100],[400,0,0]].entries()){
  const lv=launchVehicle({stages:[stage('core',1000,5000,1e6,300,0),stage('upper',300,1500,80000,300,0)],
   fairing_mass:0,payload_mass:100,sref:1,cd:LV_CD_TABLE,
   boosters:[boosterSet({stage:stage('side',200,prop,200000,300,0),count:2,ignition_delay:delay,sep_delay:sep})]});
  const r=simulate_ascent(lv,ascentGuidance(),{dt:.1,log_dt:.01,t_max:22}),j=g.staging[i];
  assert.equal(r.events.length,j.events.length);
  for(const [n,e] of r.events.entries()){
   assert.equal(e.name,j.events[n].name);close(e.t,j.events[n].t,'event time');close(e.m,j.events[n].m,'event mass');
  }
  for(const k of ['t','m','thrust']){
   assert.equal(r.log[k].length,j.log[k].length);
   for(const [n,x] of r.log[k].entries())close(x,j.log[k][n],`${j.name}.${k}[${n}]`,k==='thrust'?.01:1e-6);
  }
 }
});
test('short high-thrust lander braking and throttle-limit outcomes match Julia',t=>{
 if(!g)return t.skip('Julia unavailable');
 const m=3344.6766118550195;
 for(const [i,throttle_min] of [.1,.02].entries()){
  const d=powered_descent(lander({mdry:100,mprop:m-100,thrust:45000,isp:311,throttle_min}),
   [R_MOON+15291.055387647589,0,0],[.148114443,1692.33544,0],m,{h_gate:2000}),j=g.light[i];
  assert.equal(d.outcome,j.outcome);
  for(const k of ['t_touchdown','t_gate','v_vertical','v_horizontal','prop_left','pitch0','pitch_rate'])close(d[k],j[k],k,1e-4);
 }
});
