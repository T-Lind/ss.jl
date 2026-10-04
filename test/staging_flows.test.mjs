import {test} from 'node:test';
import assert from 'node:assert/strict';
import {stage,boosterSet,launchVehicle,LV_CD_TABLE} from '../web/src/propulsion.js';
import {ascentGuidance,simulate_ascent} from '../web/src/launch.js';
import {G0} from '../web/src/constants.js';

for (const [name,prop,delay,sep] of [['burning',10000,0,0],['waiting',10000,16,0],['spent',400,0,100],['gone',400,0,0]]) {
  test(`first-stage separation removes ${name} boosters, including remaining fuel`,()=>{
    const core=stage('core',1000,5000,1e6,300,0),upper=stage('upper',300,1500,80000,300,0);
    const side=stage('side',200,prop,200000,300,0);
    const lv=launchVehicle({stages:[core,upper],fairing_mass:0,payload_mass:100,sref:1,cd:LV_CD_TABLE,
      boosters:[boosterSet({stage:side,count:2,ignition_delay:delay,sep_delay:sep})]});
    const r=simulate_ascent(lv,ascentGuidance(),{dt:.1,log_dt:.01,t_max:22});
    const cs=r.events.find(e=>e.name==='sep_core'),bs=r.events.filter(e=>e.name==='sep_side');
    assert.ok(cs);assert.equal(bs.length,1);assert.ok(Math.abs(cs.t-5000*G0*300/1e6)<1e-8);
    assert.equal(bs[0].t<=cs.t,true);if(name!=='gone')assert.equal(bs[0].t,cs.t);
    const last=r.events.filter(e=>e.t===cs.t).at(-1);
    assert.ok(Math.abs(last.m-1900)<1e-6,`only upper stage and payload remain: ${last.m}`);
    if(name==='waiting')assert.ok(!r.events.some(e=>e.name==='ignition_side'));
    if(name==='burning')assert.ok(!r.events.some(e=>e.name==='burnout_side'),'unused fuel departs, rather than burning to depletion');
    const gap=r.log.t.map((t,i)=>({t,i})).filter(({t})=>t>=cs.t&&t<cs.t+4);
    assert.ok(gap.length>1);for(const {i} of gap){assert.equal(r.log.thrust[i],0);assert.ok(Math.abs(r.log.m[i]-1900)<1e-6);}
    assert.ok(r.events.some(e=>e.name==='ignition_upper'));
  });
}

test('mission cutoff stops attached-booster thrust without discarding unburned mass',()=>{
 const core=stage('core',1000,5000,1e6,300,0),side=stage('side',200,10000,200000,300,0);
 const lv=launchVehicle({stages:[core],fairing_mass:0,payload_mass:100,sref:1,cd:LV_CD_TABLE,
  boosters:[boosterSet({stage:side,count:2})]});
 const r=simulate_ascent(lv,ascentGuidance({cutoff:'apogee',apogee_target:500}),{dt:.1,t_max:22});
 assert.equal(r.events.at(-1).name,'seco');assert.equal(r.log.thrust.at(-1),0);
 assert.ok(!r.events.some(e=>e.name==='sep_side'));assert.ok(r.m>100+2*200);
});
