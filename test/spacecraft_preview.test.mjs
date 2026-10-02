import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {capsulePreview} from '../web/static/capsule_preview.js';
import {panelGeometry} from '../web/src/panel.js';

test('capsule previews reuse flight sections, omit the launcher, and retain crew stations',()=>{
 for(const diameter of [1.1,2.4,3.66,5.02,8]) {
  const geo=panelGeometry({mode:'orbit',payload_kind:'capsule',crewed:'1',pod_dia:String(diameter)});
  for(const cabin of [false,true]) {
   const {vertices,layout}=capsulePreview(geo,cabin);
   const sections=geo.sections.filter(s=>['pod',cabin?'cabin':'glass'].includes(s.name));
   assert.equal(vertices.length/12,sections.reduce((n,s)=>n+3*(s.t1-s.t0+1),0));
   assert.ok(vertices.every(Number.isFinite));assert.equal(layout.diameter,diameter);
   assert.equal(layout.seats.length,layout.crew.length);assert.ok(layout.seats.length>0);
   for(const seat of layout.seats) {
    assert.ok([...seat.eye,...seat.target,...seat.up].every(Number.isFinite));
    assert.ok(Math.hypot(...seat.eye.map((x,i)=>x-seat.target[i]))>.01);
   }
   for(const frame of layout.displays) {
    const corners=[[0,0],[0,1],[1,1],[1,0]].map(p=>frame.at(...p));
    assert.ok(corners.flat().every(Number.isFinite));
    assert.ok(Math.hypot(...corners[0].map((x,i)=>x-corners[1][i]))>0);
   }
  }
 }
});

test('capsule preview stations are independent of launcher geometry',()=>{
 const one=capsulePreview(panelGeometry({pod_dia:'3.66',crewed:'1',s1_prop:'10000'}));
 const two=capsulePreview(panelGeometry({pod_dia:'3.66',crewed:'1',s1_prop:'1000000'}));
 for(let i=0;i<one.layout.seats.length;i++) {
  for(const key of ['eye','target']) for(let j=0;j<3;j++)
   assert.ok(Math.abs(one.layout.seats[i][key][j]-two.layout.seats[i][key][j])<1e-10);
 }
});

test('both spacecraft viewers ship identical preview implementations',()=>{
 for(const name of ['capsule_preview.js','model_inspector.js']) {
  assert.equal(readFileSync(new URL('../web/static/'+name,import.meta.url),'utf8'),readFileSync(new URL('../scripts/static/'+name,import.meta.url),'utf8'));
 }
});
