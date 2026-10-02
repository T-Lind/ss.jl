import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createContext,runInContext } from 'node:vm';
import { moonTexel } from '../web/static/celestial.js';
const source=readFileSync(new URL('../web/panel_page.html',import.meta.url),'utf8');
function fn(name) {
  const start=source.indexOf(`function ${name}(`),end=source.indexOf('\n}',start);
  assert.ok(start>=0&&end>start);return source.slice(start,end+2);
}
test('mission-control playback includes lunar orbit and the final descent sample',()=>{
  const c=createContext({Math,Float64Array,N_MOON:2*Math.PI/(27.321661*86400),S:null,
    run:{asc3d:{t:[0],x:[6.4],y:[0],z:[0]},
      cis:{t:[1,10],x:[6.5,386],y:[0,0],z:[0,0],mx:[384.4,384.4],my:[0,0],mz:[0,0],ph:[0,2],n:[0,0,1]},
      moon:{orbit:{t:[10,20],x:[1800,1750],y:[0,0],z:[0,0]},descent:{x:[1750,1737.4],y:[0,0],z:[0,0]}},
      descent:{t:[0,5]},site:{t_pdi:20,t_td:25}}});
  runInContext(source.slice(source.indexOf('const dot3 ='),source.indexOf('function norm3(')),c);
  for(const name of ['norm3','rotAxis','precompute'])runInContext(fn(name),c);
  c.precompute();assert.equal(c.S.t.at(-1),25);assert.equal(c.S.ph.at(-1),5);
  assert.ok(c.S.ph.includes(3));
  assert.ok(c.S.t.every((t,i,a)=>i===0||t>a[i-1]));
  const last=c.S.n-1;assert.ok(Math.abs(Math.hypot(c.S.x[last]-c.S.mx[last],c.S.y[last]-c.S.my[last],c.S.z[last]-c.S.mz[last])-1.7374)<1e-9);
});
test('the actual lunar disc raster puts the map north above south',()=>{
  const c=createContext({Math,moonTexel,MTW:4,MTH:2,MDISC:null,MDISC_KEY:'',
    MTEX:new Uint8ClampedArray([...Array(4).fill([0,0,200,255]).flat(),...Array(4).fill([200,0,0,255]).flat()]),
    document:{createElement(){const canvas={};canvas.getContext=()=>({
      createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4)}),
      putImageData:data=>canvas.pixels=data.data});return canvas;}}});
  runInContext(source.slice(source.indexOf('const dot3 ='),source.indexOf('function norm3(')),c);
  for(const name of ['moonDisc'])runInContext(fn(name),c);
  const cv=c.moonDisc(8,[[1,0,0],[0,0,1],[0,-1,0]],[0,0,-1]);
  const north=(1*16+8)*4,south=(14*16+8)*4;
  assert.ok(cv.pixels[north+2]>cv.pixels[north]);
  assert.ok(cv.pixels[south]>cv.pixels[south+2]);
  assert.equal(c.moonDisc(8,[[1,0,0],[0,0,1],[0,-1,0]],[0,0,-1]),cv);
  assert.notEqual(c.moonDisc(8,[[1,0,0],[0,0,-1],[0,1,0]],[0,0,-1]),cv);
});
test('Julia and static mission-control scenes use the same map, axes and clock',()=>{
  const julia=readFileSync(new URL('../scripts/panel_page.html',import.meta.url),'utf8');
  for(const name of ['precompute','drawGlobe','moonDisc','drawPoleAxis','drawMoonScene','drawScene'])
    assert.ok(julia.includes(fn(name)),`${name} differs`);
});
