import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createContext, runInContext } from 'node:vm';
import { landerLayout } from '../web/static/lander_model.js';
import { panelGeometry } from '../web/src/panel.js';

// Run the shipped viewer's flight logic with small telemetry fixtures. No GPU
// is needed to verify which vehicle owns the camera, hatch or attitude matrix.
const source = readFileSync(new URL('../web/launch_page.html', import.meta.url), 'utf8');
function fn(name) {
  const start = source.indexOf(`function ${name}(`);
  assert.ok(start >= 0, `missing ${name}`);
  const end = source.indexOf('\n}', start);
  const oneLine = source.indexOf('\n', start);
  return source.slice(start, source.slice(start, oneLine).endsWith('}') ? oneLine : end + 2);
}
function flight() {
  const c = createContext({ landerLayout, Math, Number,
    run: { mode: 'landing', params: {}, metrics: { outcome: 'touchdown' } },
    HASCAB: false, camMode: 'cabin', simT: 100, warpEff: 1,
    BURNW: [], tLOI: 300, tDOI: 400, tPDI: 500, tLanderDrop: 400,
    tEI: NaN, secoT: 50, RCS_ALIGN_LEAD: 60, MDIA: 4.2,
    RCS: { q: [1, 0, 0, 0] }, EVA: { active: false }, SURF: { active: false },
    MD: { t: [0, 100], m: [9000, 5000], pitch: [0] }, MS: { t_pdi: 500, t_td: 600 },
    geo: { sections: [{ name: 'lander', x0: 40 }], stages: [{ diameter_m: 9 }] }, DIA: 9,
    attachedSpan: () => ({ x0: 0, x1: 50 }), SECR: { stage2: 4.5 }, LASTST: 'stage2',
    TOPD: 9, A: { t: [0, 50], m: [100000, 50000] }, RM: 1737400,
    state: () => ({ thr: 0 }),
  });
  runInContext(source.slice(source.indexOf('const V = {'), source.indexOf('// --------------------------------------------------------------- gl setup')), c);
  for (const name of ['qMat', 'samp', 'isLandingMission', 'landerStackMat', 'landerAttitudeMat',
    'eciLanderFrame', 'eciLanderMat', 'craftInertia', 'coastActive', 'rcsActive',
    'rcsMustAlign', 'rcsAlignIn', 'landed', 'evaGate']) runInContext(fn(name), c);
  const start = source.indexOf('const CAMS_ALL =');
  runInContext(source.slice(start, source.indexOf('function refreshCams(', start)), c);
  return c;
}
const plain = v => JSON.parse(JSON.stringify(v));

test('capsules remain independent of the lunar landing vehicle', () => {
  for (const mode of ['orbit', 'flyby', 'suborbital']) {
    const g = panelGeometry({ mode, payload_kind: 'capsule', crewed: '1', pod_mass: '4200', pod_dia: '3.66' });
    assert.equal(g.payload.kind, 'capsule');
    assert.ok(g.sections.some(s => s.name === 'pod'));
    assert.ok(g.sections.some(s => s.name === 'cabin'));
    assert.ok(!g.sections.some(s => s.name === 'lander'));
  }
  const g = panelGeometry({ mode: 'landing' });
  assert.equal(g.payload.kind, 'lander');
  assert.ok(g.sections.some(s => s.name === 'lander'));
  assert.ok(!g.sections.some(s => s.name === 'pod'));
});

test('lander cockpit is available in launch, cruise and moon worlds', () => {
  const c = flight();
  for (const world of ['pad', 'eci', 'moon']) {
    runInContext(`curWorld='${world}'; curLander=false`, c);
    assert.ok(runInContext('cameraModes().includes("cabin")', c));
  }
  c.run.mode = 'orbit';
  runInContext("curWorld='pad'", c);
  assert.equal(runInContext('cameraModes().includes("cabin")', c), false);
  c.HASCAB = true;
  assert.ok(runInContext('cameraModes().includes("cabin")', c));
});

test('attached cockpit stations follow the stack and detached RCS turns the cabin', () => {
  const c = flight();
  // The deployed footpads sit above the adapter, and the cabin's +y follows
  // the launch stack's +x; +x forward points away from the stack axis.
  const attached = runInContext('landerStackMat(mIdent())', c);
  const foot = runInContext('mApply(landerStackMat(mIdent()),[0,0,0])', c);
  assert.ok(Math.abs(foot[0] - 41.19) < 1e-9);
  assert.deepEqual(plain(foot.slice(1)), [0, 0]);
  assert.deepEqual(plain(attached.slice(4, 7)), [1, 0, 0]);
  const pivot = [0, 2.65, 0];
  const base = runInContext('landerAttitudeMat(mIdent())', c);
  c.RCS.q = [Math.cos(.2), Math.sin(.2), 0, 0];
  const turned = runInContext('landerAttitudeMat(mIdent())', c);
  assert.notDeepEqual(plain(turned), plain(base));
  c.turned = turned; c.pivot = pivot;
  assert.deepEqual(plain(runInContext('mApply(turned,pivot)', c)), pivot);
  // Roll is around the lander's thrust/up axis, so it leaves +y unchanged.
  assert.deepEqual(plain(turned.slice(4, 7)), [0, 1, 0]);
  assert.equal(c.craftInertia(450).M, 9000);
  assert.equal(c.craftInertia(550).M, 7000);
  assert.equal(c.craftInertia(450).L, 4.72);
});

test('hatch supports coast and touchdown while locking for burns, entry and failed landings', () => {
  const c = flight();
  assert.equal(c.evaGate({ w: 'eci' }).ok, true);
  c.BURNW = [[90, 110]];
  assert.equal(c.evaGate({ w: 'eci' }).ok, false);
  assert.equal(c.evaGate({ w: 'entry' }).ok, false);
  c.BURNW = []; c.simT = 300;
  assert.equal(c.evaGate({ w: 'eci' }).ok, false); // impulsive LOI
  c.simT = 550;
  assert.equal(c.evaGate({ w: 'moon' }).ok, false);
  c.simT = 601;
  assert.equal(c.evaGate({ w: 'moon' }).ok, true);
  c.run.metrics.outcome = 'crash';
  assert.equal(c.evaGate({ w: 'moon' }).ok, false);
});

test('lander realigns before each lunar burn and gives the suit ownership outside', () => {
  const c = flight();
  for (const t of [300, 400, 500]) {
    c.simT = t - 30;
    assert.equal(c.rcsMustAlign({ w: 'eci' }), true);
  }
  c.simT = 100;
  assert.equal(c.rcsAlignIn(), 140);
  assert.equal(c.rcsActive({ w: 'eci' }), true);
  c.EVA.active = true;
  assert.equal(c.rcsActive({ w: 'eci' }), false);
});

test('door clicks from both lander seats reach the hatch before a console switch', () => {
  const c = flight();
  Object.assign(c, { CAB_PAGES: ['FLIGHT'], cabPage: 0 });
  const start = source.indexOf('const CAP_FN =');
  runInContext(source.slice(start, source.indexOf('\n];', start) + 3), c);
  for (const name of ['lmK', 'cabK', 'cabControls', 'evaHatchStation', 'raySphere'])
    runInContext(fn(name), c);
  runInContext(`
    const L=landerLayout(MDIA), targets=cabControls(), hatch=evaHatchStation();
    for(const z of L.crew) {
      const eye=[L.eyeX,L.eyeY,z], dir=V.norm(V.sub(hatch.p,eye));
      const nearest=targets.map(t=>({key:t.key,range:raySphere(eye,dir,t.p,t.r)}))
        .sort((a,b)=>a.range-b.range)[0];
      if(nearest.key!=='hatch') throw new Error('hatch intercepted by '+nearest.key);
    }
  `, c);
});

test('touchdown stops propulsion and terminal velocity readouts before moonwalks', () => {
  const c = flight();
  Object.assign(c.MD, { h: [10, 0], v: [100, 1], vh: [90, .4], vv: [-10, -1],
    thr: [100, 11], elev: [0, 0], navdh: [0, 0] });
  Object.assign(c.MS, { lx: [1000, 0], ly: [10000, 0], lz: [0, 0] });
  runInContext(fn('moonState'), c);
  assert.ok(c.moonState(599).thr > 0);
  const landed = c.moonState(601);
  assert.equal(landed.thr, 0);
  assert.equal(landed.h, 0);
  assert.equal(landed.v, 0);
  assert.equal(landed.vh, 0);
  assert.equal(landed.vv, 0);
});

test('Julia and static viewers keep the same flight implementation', () => {
  const julia = readFileSync(new URL('../scripts/launch_page.html', import.meta.url), 'utf8');
  for (const name of ['buildScene', 'drawWorldMeshes', 'craftInertia', 'evaGate', 'landerAttitudeMat']) {
    const body = fn(name);
    assert.ok(julia.includes(body), `${name} differs between frontends`);
  }
});

test('scene cameras and pickers ride the lander throughout launch and transfer', () => {
  const c = flight();
  Object.assign(c, {
    RE: 6371000, OME: 0, LEN: 50, scorch: 0, secGone: {}, secDraw: [],
    rcsPivot: () => [0, 0, 0], burningStage: () => -1, attachedTailX: () => 0,
    anchorMat: () => [], SUNE: [1, 0, 0], run: { ...c.run, sites: {} },
    state: () => ({ pos: [0, 100000, 0], dir: [1, 0, 0], thr: 0, h: 100000, q: 0 }),
    cameraPad: (t, st, M) => ({ transform: M }),
    cisState: () => ({ pos: [2000000, 0, 0], moon: [0, 0, 0], vel: [0, 1000, 0],
      bx: [1, 0, 0], by: [0, 1, 0], bz: [0, 0, 1], burning: false }),
    cameraEci: (t, cs, ph, M) => ({ transform: M }),
  });
  for (const name of ['rcsMat', 'buildScene']) runInContext(fn(name), c);
  for (const w of ['pad', 'eci']) {
    const sc = c.buildScene({ w });
    assert.notDeepEqual(plain(sc.craftM), plain(sc.stackM));
    assert.deepEqual(plain(sc.cam.transform), plain(sc.craftM));
    c.stack = sc.stackM;
    assert.deepEqual(plain(sc.craftM), plain(runInContext('landerStackMat(stack)', c)));
  }
  c.simT = 450;
  const sc = c.buildScene({ w: 'eci' });
  assert.deepEqual(plain(sc.craftM), plain(runInContext('eciLanderMat(cisState())', c)));
  c.run.mode = 'orbit';
  assert.deepEqual(plain(c.buildScene({ w: 'eci' }).craftM), plain(sc.stackM));
});


test('lander console reports descent telemetry and actual crew state',()=>{
  const c=flight();
  Object.assign(c,{EVS:[],CAB:{live:false,seat:0},CAB_LIGHTS:[{n:'BRIGHT'}],cabLight:0});
  c.RCS.rate=0;
  for(const name of ['nextEv','fmtEta','cabRows'])runInContext(fn(name),c);
  const sc={ms:{h:125,vh:2.5,vv:-1.4,thr:.27,m:4100,dr:32,elev:-17,navdh:4}};
  const flightRows=plain(c.cabRows(sc,'FLIGHT'));
  assert.ok(flightRows.some(([label,value])=>label==='SINK RATE'&&value==='1.4 m/s'));
  assert.ok(flightRows.some(([label,value])=>label==='THROTTLE'&&value==='27%'));
  assert.ok(flightRows.some(([label,value])=>label==='PROP LEFT'&&value==='600 kg'));
  assert.ok(plain(c.cabRows(sc,'TRAJECTORY')).some(([label,value])=>label==='NAV ERROR'&&value==='4 m'));
  assert.ok(plain(c.cabRows(sc,'SYSTEMS')).some(([label,value])=>label==='CREW'&&value==='STRAPPED IN'));
  c.SURF.active=true;
  assert.ok(plain(c.cabRows(sc,'SYSTEMS')).some(([label,value])=>label==='CREW'&&value==='EVA'));
  for(const page of ['FLIGHT','TRAJECTORY','SYSTEMS'])
    for(const [label,value] of c.cabRows(sc,page))assert.ok(label.length+value.length<=19,`${label}: ${value}`);
  Object.assign(c,{TRAJ_VIEWS:['PROFILE','ORBIT'],cabTraj:1});
  runInContext("curWorld='moon'",c);
  runInContext(fn('trajView'),c);
  assert.equal(c.trajView(),'DESCENT');
  const julia=readFileSync(new URL('../scripts/launch_page.html',import.meta.url),'utf8');
  for(const name of ['cabRows','cabScreenPaint','cabDecalBuild','decPaint','trajView'])assert.ok(julia.includes(fn(name)),`${name} frontend drift`);
});
