import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {landerLayout,landerExterior,landerInterior,landerDisplay,clampLanderEye} from '../web/static/lander_model.js';
import {engineCluster,stageBells} from '../web/static/engine_layout.js';
import {engineModel} from '../web/static/engine_model.js';
import {rocket_mesh,mesh_volume} from '../web/src/mesh.js';

const sub=(a,b)=>a.map((v,i)=>v-b[i]);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const dot=(a,b)=>a.reduce((v,x,i)=>v+x*b[i],0);
// Möller–Trumbore ray/triangle test: used to verify actual window sight lines.
function hit(origin,dir,a,b,c) {
  const e1=sub(b,a),e2=sub(c,a),h=cross(dir,e2),det=dot(e1,h);
  if(Math.abs(det)<1e-9)return false;
  const s=sub(origin,a),u=dot(s,h)/det;
  if(u<0||u>1)return false;
  const q=cross(s,e1),v=dot(dir,q)/det;
  if(v<0||u+v>1)return false;
  const t=dot(e2,q)/det;
  return t>1e-5&&t<1.001;
}

test('two crew eyes and display surfaces fit the rebuilt pressure cabin',()=>{
  for(const d of [2.1,4.2,8.4]) {
    const L=landerLayout(d);
    assert.equal(L.crew.length,2);
    for(const z of L.crew) assert.equal(clampLanderEye(L,[L.eyeX,L.eyeY,z]).hit,false);
    const c=clampLanderEye(L,[100,-100,100]);
    assert.equal(c.hit,true);assert.ok(Math.abs(Math.hypot(...c.n)-1)<1e-12);
    for(let k=0;k<3;k++) {
      const F=landerDisplay(L,k);
      assert.ok(Math.abs(F.S/F.H-420/400)<1e-12);
      assert.ok(F.x<L.panel.x&&F.x>L.clear.max[0]);
      assert.ok(F.z0>-L.halfWidth&&F.z1<L.halfWidth);
    }
  }
  assert.throws(()=>landerLayout(NaN),RangeError);
  assert.throws(()=>landerLayout(0),RangeError);
});

test('forward window apertures have clear sight lines from both crew stations',()=>{
  for(const d of [2.1,4.2,8.4]) {
    const L=landerLayout(d),data=landerInterior(d);
    for(const sign of [-1,1]) {
      const eye=[L.eyeX,L.eyeY,sign*L.crew[1]];
      for(const [dy,dz] of [[0,0],[.12,.12],[-.12,-.12]]) {
        const target=[L.front+.1*L.s,L.win.y+dy*L.s,sign*L.win.z+dz*L.s],dir=sub(target,eye);
        let hits=0;
        for(let i=0;i<data.length;i+=36)
          if(hit(eye,dir,Array.from(data.slice(i,i+3)),Array.from(data.slice(i+12,i+15)),Array.from(data.slice(i+24,i+27)))) hits++;
        assert.equal(hits,0,`diameter ${d}, crew ${sign}: window occluded`);
      }
    }
  }
});

test('lander meshes have finite normals, ground contact and the stated envelope',()=>{
  for(const d of [2.1,4.2,8.4]) {
    const exterior=landerExterior(d),L=landerLayout(d);
    let ymin=Infinity,rmax=0;
    for(const data of [exterior,landerInterior(d)]) {
      assert.equal(data.length%36,0);
      for(let i=0;i<data.length;i+=12) {
        assert.ok(Array.from(data.slice(i,i+12)).every(Number.isFinite));
        assert.ok(Math.abs(Math.hypot(data[i+3],data[i+4],data[i+5])-1)<1e-6);
      }
    }
    for(let i=0;i<exterior.length;i+=12) {ymin=Math.min(ymin,exterior[i+1]);rmax=Math.max(rmax,Math.hypot(exterior[i],exterior[i+2]));}
    assert.equal(ymin,0);assert.ok(rmax<=d/2+1e-6);
    assert.ok(L.nozzle[1]>ymin&&L.nozzle[1]<L.floor);
    assert.equal(L.hatch[2],L.ladderFoot[2]);
  }
});

test('packed engine bells stay within the thrust plate and never overlap',()=>{
  for(const n of [1,2,3,4,5,6,9,13,20,27,33,37,60]) {
    const pts=engineCluster(n,3.6,.945);
    assert.equal(pts.length,n);
    for(let i=0;i<n;i++) {
      const [y,z,r]=pts[i];assert.ok(r>0);assert.ok(Math.hypot(y,z)+r<=3.6+1e-12);
      for(let j=0;j<i;j++) assert.ok(Math.hypot(y-pts[j][0],z-pts[j][1])>=r+pts[j][2]);
    }
  }
  const rings=engineCluster(33,3.6,.945).reduce((m,p)=>{
    const r=Math.hypot(p[0],p[1]).toFixed(6);m.set(r,(m.get(r)||0)+1);return m;
  },new Map());
  assert.deepEqual([...rings.values()],[3,10,20]);
});

test('first-stage bell throats enter the skirt and plume exits match actual vertices',()=>{
  for(const n of [1,6,9,13,33]) {
    const B=stageBells(n,9,0,2);
    assert.ok(Math.abs(B.exitX+B.length-.72)<1e-12);
    const [mesh,sections]=rocket_mesh({diameters:[9,9],prop_masses:[3400000,1200000],n_engines:[n,6],fairing:false});
    const sec=sections[0],points=mesh.tris.slice(sec.t0-1,sec.t1).flat();
    assert.ok(Math.abs(Math.min(...points.map(p=>p[0]))-B.exitX)<1e-12);
    assert.equal(sec.x0,B.exitX);
    for(const [y,z,r] of B.pts)
      assert.ok(points.some(p=>Math.abs(p[0]-B.exitX)<1e-10&&Math.abs(Math.hypot(p[1]-y,p[2]-z)-r)<1e-10));
    assert.ok(Array.from(engineModel(n,9).vertices).every(Number.isFinite));
  }
});

test('closed lander engineering envelope matches deployed width and contact plane',()=>{
  const [m,secs]=rocket_mesh({diameters:[9,9],prop_masses:[3400000,1200000],n_engines:[33,6],payload_kind:'lander',payload_diameter:4.2,fairing:false});
  const sec=secs.find(s=>s.name==='lander'),tris=m.tris.slice(sec.t0-1,sec.t1),pts=tris.flat(),xb=sec.x0+.13*9+.02;
  assert.ok(mesh_volume({tris})>0);
  assert.ok(Math.abs(Math.min(...pts.map(p=>p[0]))-xb)<1e-9);
  assert.ok(Math.max(...pts.map(p=>Math.hypot(p[1],p[2])))<=2.1+1e-9);
  assert.ok(Math.abs(Math.max(...pts.map(p=>p[0]))-sec.x1)<1e-9);
});

test('both shipped frontends use identical visual modules',()=>{
  for(const name of ['lander_model.js','engine_layout.js','engine_model.js','model_inspector.js'])
    assert.equal(readFileSync(new URL(`../scripts/static/${name}`,import.meta.url),'utf8'),readFileSync(new URL(`../web/static/${name}`,import.meta.url),'utf8'));
  for (const folder of ['scripts','web']) {
    const page=readFileSync(new URL(`../${folder}/launch_page.html`,import.meta.url),'utf8');
    assert.match(page,/import \{ landerLayout, landerExterior, landerInterior, landerDisplay, clampLanderEye \} from/);
    assert.match(page,/import \{ engineCluster, stageBells \} from/);
  }
});
