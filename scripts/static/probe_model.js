// Uncrewed science-probe bus, render-only (no flight state). Same conventions as
// lander_model.js: metres, +y is the stack axis (thrust/up), y = 0 is the mating
// plane under the adapter, +x and +z are the cross-section axes. Geometry is
// authored in units of the radius r and scaled once by ModelMesh(r); vertices use
// the same 12-float stride. Stowed, everything sits inside radius r and y <= probeLength(r).
import { ModelMesh } from './lander_model.js';

const C = {
  gold:[.66,.50,.22], copper:[.74,.44,.24], goldEdge:[.60,.46,.22],
  radiator:[.13,.15,.18], fin:[.62,.65,.68], deck:[.18,.20,.23],
  adapter:[.22,.23,.25], bell:[.20,.21,.24], dish:[.86,.87,.88], feed:[.45,.47,.50],
  alu:[.70,.72,.75], dark:[.09,.12,.16], glass:[.035,.13,.21],
  cell:[.122,.184,.353], cellGap:[.55,.60,.68],
};
const sub=(a,b)=>a.map((v,i)=>v-b[i]);
const add=(a,b)=>a.map((v,i)=>v+b[i]);
const scale=(a,s)=>a.map(v=>v*s);
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];

// ModelMesh takes its normal from winding, so flip the winding to face `out`.
function tri(m,a,b,c,col,out) {
  if (dot(cross(sub(b,a),sub(c,a)),out) < 0) m.tri(a,c,b,col); else m.tri(a,b,c,col);
}
function quad(m,a,b,c,d,col,out) { tri(m,a,b,c,col,out); tri(m,a,c,d,col,out); }

// Integer hash in [0,1): the same facet always gets the same jitter.
function hash(a,b,c,d=0) {
  let h=Math.imul(a+1,0x9E3779B1)^Math.imul(b+7,0x85EBCA77)^Math.imul(c+13,0xC2B2AE3D)^Math.imul(d+19,0x27D4EB2F);
  h^=h>>>15; h=Math.imul(h,0x2C1B3C6D); h^=h>>>12; h=Math.imul(h,0x297A2D39); h^=h>>>15;
  return (h>>>0)/4294967296;
}

// Octagonal bus. Vertex j sits at (j+.5)*45 deg; face k (normal at k*45 deg) joins
// vertices k-1 and k. Faces 0 and 4 face +-x (radiators); faces 2 and 6 face +-z
// (stowed arrays), so the deployed wings run along z.
const RB=.93, RC=.87, A=RB*Math.cos(Math.PI/8);  // body vertex radius, bevel/adapter radius, apothem
const V=j=>((j%8)+8)%8;
const vAng=j=>(V(j)+.5)*Math.PI/4;
const fNorm=k=>[Math.cos(k*Math.PI/4),0,Math.sin(k*Math.PI/4)];
const ringPt=(R,y,j)=>[R*Math.cos(vAng(j)),y,R*Math.sin(vAng(j))];

function band(m,R0,y0,R1,y1,col) {
  for (let k=0;k<8;k++) {
    quad(m,ringPt(R0,y0,k-1),ringPt(R0,y0,k),ringPt(R1,y1,k),ringPt(R1,y1,k-1),col,fNorm(k));
  }
}
function cap(m,y,R,col,up) {
  const c=[0,y,0];
  for (let k=0;k<8;k++) tri(m,c,ringPt(R,y,k),ringPt(R,y,k+1),col,[0,up,0]);
}

// Crinkled gold MLI: an NX x NY grid per face. Interior nodes are pushed along the
// face normal by a fixed amount, and each cell gets its own brightness (+-8%) and a
// slight shift toward copper, so the foil reads as faceted and crumpled.
const NX=4, NY=6, Y0=.24, Y1=.96;
function mliFace(m,k) {
  const n=fNorm(k), a=ringPt(RB,0,k-1), b=ringPt(RB,0,k);
  const P=(i,j)=>{
    const s=i/NX, y=Y0+(Y1-Y0)*j/NY;
    let p=[a[0]+(b[0]-a[0])*s, y, a[2]+(b[2]-a[2])*s];
    if (i>0 && i<NX && j>0 && j<NY) p=add(p,scale(n,(hash(k,i,j)-.5)*.012));
    return p;
  };
  for (let i=0;i<NX;i++) for (let j=0;j<NY;j++) {
    const t=.25*hash(k,i,j,2), f=1+.08*(2*hash(k,i,j,1)-1);
    const col=C.gold.map((g,c)=>Math.min(1,(g*(1-t)+C.copper[c]*t)*f));
    quad(m,P(i,j),P(i+1,j),P(i+1,j+1),P(i,j+1),col,n);
  }
}

// Radiator on a face: dark panel with thin fins. Sits on the face, proud by .02.
function radiator(m,k) {
  const n=fNorm(k), t=[-n[2],0,n[0]], o=scale(n,A+.02);
  const at=(s,y)=>add(o,add(scale(t,s),[0,y,0]));
  quad(m,at(-.32,.36),at(.32,.36),at(.32,.84),at(-.32,.84),C.radiator,n);
  for (let i=0;i<=8;i++) {
    const s=-.32+.64*i/8;
    m.beam(at(s,.36),at(s,.84),.012,.03,C.fin);
  }
}

// Solar panel: light substrate, dark cells proud of it, and an aluminium frame.
// c is the centre; uh and vh are the half-extents; n is the outward normal.
function panel(m,c,uh,vh,n,nu,nv) {
  const at=(s,t,o=0)=>add(c,add(scale(uh,s),add(scale(vh,t),scale(n,o))));
  const O=.006, g=.14;
  quad(m,at(-1,-1),at(1,-1),at(1,1),at(-1,1),C.cellGap,n);
  for (let i=0;i<nu;i++) for (let j=0;j<nv;j++) {
    const s0=-1+2*(i+g/2)/nu, s1=-1+2*(i+1-g/2)/nu, t0=-1+2*(j+g/2)/nv, t1=-1+2*(j+1-g/2)/nv;
    const sh=1+.03*(2*hash(i,j,nu,7)-1);
    quad(m,at(s0,t0,O),at(s1,t0,O),at(s1,t1,O),at(s0,t1,O),C.cell.map(v=>v*sh),n);
  }
  const q=[at(-1,-1,O),at(1,-1,O),at(1,1,O),at(-1,1,O)];
  for (let k=0;k<4;k++) m.beam(q[k],q[(k+1)%4],.022,.022,C.alu);
}

const DY=1.05, RD=.44, DD=.06, NR=6, NS=32;  // dish vertex height, radius, depth
const dishPt=(rho,k)=>{
  const ph=2*Math.PI*(k%NS)/NS;
  return [rho*Math.cos(ph), DY+DD*(rho/RD)**2, rho*Math.sin(ph)];
};

// Stowed length along the axis (the probe_mesh body length, 1.35 r).
export function probeLength(radius) {
  if (!(Number.isFinite(radius) && radius > 0)) throw new RangeError('positive probe radius required');
  return 1.35*radius;
}

export function probeExterior(radius=.75, { deployed=false }={}) {
  probeLength(radius);
  const m=new ModelMesh(radius);

  // Adapter (dark octagonal frustum with a bolt flange), engine bell, bus bevels.
  band(m,.66,0,RC,.20,C.adapter);
  band(m,.70,0,.70,.03,C.alu);
  m.bell(0,.20,.30,.10,C.bell);
  cap(m,.20,RC,C.deck,-1);
  band(m,RC,.20,RB,.24,C.goldEdge);
  for (let k=0;k<8;k++) mliFace(m,k);
  band(m,RB,.96,RC,1.0,C.goldEdge);
  cap(m,1.0,RC,C.deck,1);

  // Radiators on the +-x faces, stowed or deployed.
  radiator(m,0); radiator(m,4);

  // Four RCS quads, one per alternate corner: each pod is a short stub with a 2x2 nozzle cluster.
  for (const j of [0,2,4,6]) {
    const ph=vAng(j), r=[Math.cos(ph),0,Math.sin(ph)], t=[-Math.sin(ph),0,Math.cos(ph)];
    const P=(rho,dt,dy)=>[r[0]*rho+t[0]*dt, .30+dy, r[2]*rho+t[2]*dt];
    m.tube(P(.90,0,0),P(.975,0,0),.035,C.dark,10);
    for (const [dt,dy] of [[-.02,-.02],[.02,-.02],[-.02,.02],[.02,.02]])
      m.tube(P(.975,dt,dy),P(1.0,dt,dy),.012,C.alu,8);
  }

  // Solar arrays: stowed panels flat on the +-z faces, or deployed wings on booms
  // along +-z to 3.5 r (wing plane x = 0, span 1.04 .. 3.5 r, height .30 .. .90 r).
  if (deployed) {
    for (const sg of [-1,1]) {
      m.tube([0,.60,sg*.90],[0,.60,sg*1.04],.024,C.alu,10);
      m.box([0,.60,sg*.975],[.07,.66,.05],C.dark);
      panel(m,[0,.60,sg*2.27],[0,0,1.23],[0,.30,0],[1,0,0],12,3);
    }
  } else {
    for (const sg of [-1,1]) panel(m,[0,.60,sg*(A+.02)],[.30,0,0],[0,.30,0],[0,0,sg],3,4);
  }

  // Top deck: high-gain dish on a hub, feed strut and horn, low-gain whip, star trackers.
  m.tube([0,1.0,0],[0,DY,0],.10,C.alu,12);
  for (let k=0;k<NS;k++) tri(m,[0,DY,0],dishPt(RD/NR,k),dishPt(RD/NR,k+1),C.dish,[0,1,0]);
  for (let i=1;i<NR;i++) for (let k=0;k<NS;k++)
    quad(m,dishPt(RD*i/NR,k),dishPt(RD*(i+1)/NR,k),dishPt(RD*(i+1)/NR,k+1),dishPt(RD*i/NR,k+1),C.dish,[0,1,0]);
  for (let k=0;k<NS;k+=2) m.tube(dishPt(RD,k),dishPt(RD,k+2),.012,C.alu,6);
  m.tube([0,DY,0],[0,1.22,0],.010,C.alu,8);
  m.tube([0,1.22,0],[0,1.30,0],.055,C.feed,16,.02);
  m.tube([-.42,1.0,-.40],[-.42,1.27,-.40],.011,C.alu,8);
  m.tube([-.42,1.27,-.40],[-.42,1.34,-.40],.011,C.dark,8,.002);
  for (const [x,z] of [[.52,-.34],[.30,.50]]) {
    m.box([x,1.06,z],[.14,.10,.14],C.dark);
    m.box([x,1.112,z],[.09,.012,.09],C.glass);
  }
  return new Float32Array(m.vertices);
}
