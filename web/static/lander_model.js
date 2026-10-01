// Modern two-person lander concept, not a reconstruction of a specific vehicle.
// Metres, +x forward, +y thrust/up, +z starboard; footpad soles are y = 0.
// diameter is the full deployed radial envelope, including the footpads.
// The same stations drive the hull, interior, cameras, controls and EVA.
export function landerLayout(diameter = 4.2) {
  if (!(Number.isFinite(diameter) && diameter > 0)) throw new RangeError('positive lander diameter required');
  const s = diameter / 4.2, p = v => v.map(x => x*s);
  const panel = { x: .76*s, yTop: 3.08*s, yBot: 2.62*s, zW: .85*s, yc: 2.85*s };
  return {
    s, diameter, floor: 1.83*s, ceiling: 4.14*s,
    rear: -.98*s, front: 1.06*s, halfWidth: 1.05*s,
    panel, win: { x: 1.06*s, y: 3.49*s, z: .53*s, hy: .34*s, hz: .38*s },
    crew: [-.44*s, .44*s], eyeX: -.34*s, eyeY: 3.30*s,
    hatch: p([1.09, 2.22, 0]), hatchNormal: [1, 0, 0],
    ladderFoot: p([1.78, 0, 0]), nozzle: p([0, .34, 0]),
    // Fittings live outside this conservative free-flight camera envelope.
    clear: { min: p([-.53, 2.48, -.66]), max: p([.55, 3.73, .66]) },
    height: 4.72*s,
  };
}

const sub = (a,b) => a.map((v,i) => v-b[i]);
const add = (a,b) => a.map((v,i) => v+b[i]);
const scale = (a,s) => a.map(v => v*s);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const unit = a => scale(a, 1/Math.hypot(...a));

// Renderer-neutral triangle builder. Stride matches the flight viewer:
// position, normal, RGB, UV, paint weight. Also used by the model inspector.
export class ModelMesh {
  constructor(s) { this.s=s; this.vertices=[]; }
  tri(a,b,c,col) {
    const n=unit(cross(sub(b,a),sub(c,a)));
    for (const q of [a,b,c]) this.vertices.push(...scale(q,this.s),...n,...col,0,0,0);
  }
  quad(a,b,c,d,col) { this.tri(a,b,c,col); this.tri(a,c,d,col); }
  beam(a,b,w,t,col) {
    const ax=unit(sub(b,a)), side=unit(cross(ax,Math.abs(ax[1])>.9?[1,0,0]:[0,1,0]));
    const up=cross(side,ax);
    const at=(p,u,v)=>add(add(p,scale(side,u*w/2)),scale(up,v*t/2));
    const q=[at(a,-1,-1),at(a,1,-1),at(a,1,1),at(a,-1,1),
             at(b,-1,-1),at(b,1,-1),at(b,1,1),at(b,-1,1)];
    for (const f of [[0,3,2,1],[4,5,6,7],[0,1,5,4],[3,7,6,2],[0,4,7,3],[1,2,6,5]])
      this.quad(...f.slice().reverse().map(i=>q[i]),col);
  }
  box(c,size,col) { this.beam([c[0],c[1]-size[1]/2,c[2]],[c[0],c[1]+size[1]/2,c[2]],size[2],size[0],col); }
  tube(a,b,r,col,n=12,r1=r,caps=true) {
    const ax=unit(sub(b,a)), side=unit(cross(ax,Math.abs(ax[1])>.9?[1,0,0]:[0,1,0])), up=cross(side,ax);
    const at=(p,rr,k)=>add(p,add(scale(side,rr*Math.cos(2*Math.PI*k/n)),scale(up,rr*Math.sin(2*Math.PI*k/n))));
    for(let k=0;k<n;k++) {
      const a0=at(a,r,k),a1=at(a,r,k+1),b0=at(b,r1,k),b1=at(b,r1,k+1);
      this.quad(a0,b0,b1,a1,col);
      if(caps) { this.tri(a,a0,a1,col); this.tri(b,b1,b0,col); }
    }
  }
  // Revolved open, double-wall nozzle with an annular exit (no fake exit disk).
  bell(y0,y1,r0,r1,col) {
    const profile=Array.from({length:9},(_,i)=>{
      const f=i/8; return [y0+(y1-y0)*f,r1+(r0-r1)*(1-f)**1.6];
    });
    for(let i=0;i<8;i++) {
      const [y,r]=profile[i],[yn,rn]=profile[i+1];
      this.tube([0,y,0],[0,yn,0],r,col,24,rn,false);
      this.tube([0,yn,0],[0,y,0],rn-.018,[.08,.09,.11],24,r-.018,false);
    }
    for(let k=0;k<24;k++) {
      const p=(r,j)=>[r*Math.cos(j*Math.PI/12),y0,r*Math.sin(j*Math.PI/12)];
      this.quad(p(r0,k+1),p(r0-.018,k+1),p(r0-.018,k),p(r0,k),col);
    }
  }
}
const C={white:[.82,.85,.88], silver:[.47,.53,.59], dark:[.09,.12,.16],
  panel:[.16,.20,.25], floor:[.22,.27,.32], trim:[.31,.38,.44],
  glass:[.035,.13,.21], blue:[.14,.43,.59], gold:[.56,.46,.25], light:[.80,.91,.94], orange:[.94,.43,.16]};

// Chamfered pressure shell; exact rectangular apertures replace the old coarse
// grid/centroid window cut. The cabin and hull share these same edges.
function pressureShell(m, inside) {
  const x0=inside?-.95:-.98,x1=inside?1.035:1.06;
  const yz=inside ? [[1.85,-.88],[1.99,-1.02],[3.83,-1.02],[4.11,-.74],
    [4.11,.74],[3.83,1.02],[1.99,1.02],[1.85,.88]] :
    [[1.83,-.91],[1.97,-1.05],[3.86,-1.05],[4.14,-.77],
     [4.14,.77],[3.86,1.05],[1.97,1.05],[1.83,.91]];
  for(let i=0;i<8;i++) {
    const [y,z]=yz[i],[yn,zn]=yz[(i+1)%8];
    m.quad([x0,y,z],[x0,yn,zn],[x1,yn,zn],[x1,y,z],inside?C.panel:C.white);
    m.tri([x0,2.98,0],[x0,yn,zn],[x0,y,z],inside?C.trim:C.silver);
  }
  const face=(ya,yb,za,zb,col)=>m.quad([x1,ya,za],[x1,yb,za],[x1,yb,zb],[x1,ya,zb],col);
  const col=inside?C.panel:C.white;
  const width=-yz[1][1];
  face(yz[1][0],3.15,-width,width,col);
  if(yz[2][0]>3.83) face(3.83,yz[2][0],-width,width,col);
  for(const [a,b] of [[-width,-.91],[-.15,.15],[.91,width]]) face(3.15,3.83,a,b,col);
  // End-cap bevels and bottom/roof bands close every non-window part.
  for(const [a,b,c,d] of [[0,1,6,7],[2,3,4,5]]) {
    const q=[a,b,c,d].map(i=>[x1,...yz[i]]); m.quad(...q,col);
  }
  for(const z of [-.53,.53]) {
    const x=inside?1.017:1.079;
    for(const y of [3.15,3.83]) m.box([x,y,z],[.04,.045,.805],C.trim);
    for(const zz of [z-.38,z+.38]) m.box([x,3.49,zz],[.04,.725,.045],C.trim);
    if(!inside) m.box([1.066,3.49,z],[.008,.635,.715],C.glass);
  }
}

export function landerExterior(diameter=4.2) {
  const L=landerLayout(diameter),m=new ModelMesh(L.s);
  // Squat octagonal propulsion deck, pale thermal blankets and dark radiators.
  m.tube([0,.82,0],[0,1.62,0],1.28,C.silver,8,1.34);
  m.tube([0,1.62,0],[0,1.74,0],1.34,C.dark,8);
  for(let k=0;k<8;k++) {
    const a=(k+.5)*Math.PI/4, radial=[Math.cos(a),0,Math.sin(a)], tangent=[-Math.sin(a),0,Math.cos(a)];
    const at=(y,t)=>add(scale(radial,1.245),add([0,y,0],scale(tangent,t)));
    m.quad(at(.92,-.43),at(1.51,-.43),at(1.51,.43),at(.92,.43),k%2?C.white:C.dark);
    for(const y of [1.03,1.16,1.29,1.42]) m.beam(at(y,-.40),at(y,.40),.012,.012,C.silver);
  }
  m.bell(.34,.98,.43,.16,C.silver);
  m.tube([0,.98,0],[0,1.18,0],.18,C.dark,16);
  for(let k=0;k<4;k++) {
    const a=Math.PI/4+k*Math.PI/2,c=Math.cos(a),z=Math.sin(a);
    const hip=[c*1.13,1.52,z*1.13],ankle=[c*1.84,.17,z*1.84];
    m.tube(hip,ankle,.055,C.silver,10);
    const knee=hip.map((v,i)=>v+.47*(ankle[i]-v));
    m.tube(knee,ankle,.077,C.white,10);
    for(const sign of [-1,1]) {
      const aa=a+sign*.33;
      m.tube([Math.cos(aa)*.91,.88,Math.sin(aa)*.91],knee,.034,C.silver,8);
    }
    m.box(hip,[.19,.22,.19],C.dark);
    m.tube([ankle[0],0,ankle[2]],[ankle[0],.105,ankle[2]],.26,C.silver,20,.20);
    m.tube([ankle[0],.105,ankle[2]],ankle,.065,C.dark,10);
  }
  m.tube([0,1.74,0],[0,1.84,0],.95,C.silver,12);
  pressureShell(m,false);
  // Rear service compartment, radiators, external tanks and real RCS nozzles.
  m.box([-1.11,2.92,0],[.26,1.65,1.5],C.dark);
  for(const z of [-.53,.53]) {
    m.box([-1.25,2.91,z],[.03,1.4,.44],C.blue);
    for(let y=2.3;y<3.6;y+=.16) m.box([-1.271,y,z],[.012,.018,.44],C.silver);
  }
  for(const side of [-1,1]) {
    m.tube([-.51,2.07,side*1.22],[-.51,3.38,side*1.22],.20,C.white,16);
    for(const y of [2.29,3.12]) m.box([-.51,y,side*1.21],[.46,.04,.45],C.silver);
    for(const x of [-.67,.67]) {
      m.box([x,3.58,side*1.10],[.22,.19,.21],C.dark);
      for(const sign of [-1,1])
        m.tube([x+sign*.09,3.58,side*1.1],[x+sign*.23,3.58,side*1.1],.035,C.silver,10,.075,false);
      m.tube([x,3.67,side*1.1],[x,3.82,side*1.1],.035,C.silver,10,.075,false);
      m.tube([x,3.58,side*1.18],[x,3.58,side*1.31],.035,C.silver,10,.075,false);
    }
    m.box([.98,2.31,side*.73],[.20,.43,.30],C.dark); // landing cameras
    m.box([1.089,2.33,side*.73],[.022,.16,.17],C.glass);
    m.beam([1.15,1.77,side*.30],[1.15,2.60,side*.30],.035,.035,C.silver);
  }
  // Thermal-panel seams and inspection fasteners on the forward face.
  m.box([1.075,3.035,0],[.012,.035,1.99],C.blue);
  for(const z of [-.97,.97]) for(const y of [2.02,2.78,3.06,3.88])
    m.box([1.081,y,z],[.016,.026,.026],C.silver);
  for(const side of [-1,1]) {
    m.box([.16,2.85,side*1.061],[.84,1.22,.018],C.trim);
    m.box([.16,2.85,side*1.075],[.76,1.14,.014],C.white);
    m.box([.16,3.36,side*1.086],[.65,.028,.008],C.blue);
    m.box([.45,2.91,side*1.091],[.025,.15,.012],C.silver);
  }
  // Centerline hatch, porch and straight ladder; no ladder ending off-axis.
  m.box([1.087,2.22,0],[.04,.72,.63],C.trim);
  m.box([1.114,2.22,0],[.02,.59,.50],C.white);
  m.box([1.139,2.32,.15],[.035,.035,.17],C.orange);
  m.box([1.36,1.80,0],[.56,.07,.66],C.dark);
  for(const z of [-.27,.27]) m.beam([1.33,1.80,z],[1.78,.15,z],.036,.04,C.silver);
  for(let i=0;i<9;i++) {
    const f=i/8; m.beam([1.33+.45*f,1.8-1.65*f,-.27],[1.33+.45*f,1.8-1.65*f,.27],.035,.035,C.silver);
  }
  // Low-profile docking interface, top-mounted antennas and survey mast.
  m.tube([0,4.14,0],[0,4.37,0],.33,C.silver,24);
  m.tube([0,4.37,0],[0,4.42,0],.40,C.dark,24);
  m.tube([-.66,4.06,.53],[-.66,4.64,.53],.022,C.silver,8);
  m.box([-.66,4.67,.53],[.26,.10,.06],C.white);
  m.tube([-.58,4.04,-.54],[-.58,4.28,-.54],.025,C.silver,8);
  m.tube([-.58,4.28,-.54],[-.58,4.42,-.64],.23,C.white,20,.30,false);
  return new Float32Array(m.vertices);
}

export function landerInterior(diameter=4.2, controls=[]) {
  const L=landerLayout(diameter),m=new ModelMesh(L.s),P=landerLayout(4.2).panel;
  pressureShell(m,true);
  m.box([0,1.86,0],[1.9,.06,1.75],C.floor);
  // Non-slip deck strips and rail-mounted upright crew restraints.
  for(let x=-.8;x<.9;x+=.14) m.box([x,1.895,0],[.018,.009,1.68],C.trim);
  for(const z of [-.44,.44]) {
    m.box([-.34,1.94,z],[.48,.08,.45],C.dark);
    m.box([-.72,2.7,z],[.10,1.50,.39],C.trim);
    for(const dz of [-.13,.13]) m.beam([-.654,3.18,z+dz],[-.654,2.44,z-dz],.045,.015,C.orange);
    m.box([-.645,2.78,z],[.04,.08,.16],C.silver);
    for(const dz of [-.23,.23]) {
      m.beam([-.50,2.48,z+dz],[.39,2.48,z+dz],.034,.034,C.silver);
      m.tube([.32,2.48,z+dz],[.32,2.69,z+dz],.035,C.dark,10);
      m.box([.32,2.72,z+dz],[.06,.10,.07],C.trim);
    }
  }
  // Three inset displays with a separate, reachable physical switch strip.
  m.box([P.x+.055,P.yc,0],[.10,.58,1.84],C.trim);
  for(let k=0;k<3;k++) {
    const z=-P.zW+2*P.zW*(k+.5)/3;
    m.box([P.x-.003,P.yc,z],[.017,.445,.48],C.dark);
  }
  m.box([P.x,2.47,0],[.15,.13,1.78],C.dark);
  controls.forEach((fn,i)=>{
    const z=P.zW*.9*(2*i/Math.max(controls.length-1,1)-1);
    m.box([P.x-.09,2.48,z],[.035,.055,.055],fn.c||C.silver);
  });
  // Side service racks, access doors, handles and visible ventilation slots.
  for(const sign of [-1,1]) {
    m.box([-.16,2.35,sign*.91],[1.30,.80,.20],C.trim);
    for(let x=-.65;x<.55;x+=.38) {
      m.box([x,2.35,sign*.797],[.32,.66,.02],C.panel);
      m.box([x+.10,2.45,sign*.774],[.028,.12,.018],C.silver);
      for(let y=2.13;y<2.29;y+=.045) m.box([x,y,sign*.774],[.22,.012,.012],C.dark);
    }
    m.beam([-.65,3.85,sign*.73],[.77,3.85,sign*.73],.025,.025,C.silver);
    m.box([0,4.02,sign*.57],[1.5,.035,.045],C.light);
    m.box([.73,3.59,sign*.966],[.41,.37,.025],C.dark);
    for(let j=0;j<4;j++) m.box([.61+j*.08,3.60,sign*.946],[.025,.15,.025],j===0?C.orange:C.silver);
  }
  m.box([-.90,3.70,0],[.06,.39,1.25],C.trim);
  for(const z of [-.40,0,.40]) m.box([-.858,3.70,z],[.025,.27,.34],C.panel);
  // Hatch aperture is below the instrument console; same point as outside.
  m.box([1.008,2.22,0],[.035,.72,.63],C.trim);
  m.box([.979,2.22,0],[.02,.59,.50],C.floor);
  m.box([.956,2.32,.15],[.035,.035,.17],C.orange);
  return new Float32Array(m.vertices);
}

export function landerDisplay(L,k,aspect=420/400) {
  const P=L.panel, pitch=2*P.zW/3, zc=-P.zW+pitch*(k+.5);
  const H=(P.yTop-P.yBot)*.44, S=Math.min(pitch*.43,H*aspect);
  const x=P.x-.020*L.s;
  return {x,rc:P.yc,S,H,up:[0,1,0],rt:[0,0,1],zc,yc:P.yc,
    z0:zc-S,z1:zc+S,at:(u,v)=>[x,P.yc+(1-2*v)*H,zc+(2*u-1)*S]};
}

export function clampLanderEye(L,pt) {
  const p=pt.map((v,i)=>Math.max(L.clear.min[i],Math.min(L.clear.max[i],v)));
  const del=sub(p,pt), hit=del.some(v=>v!==0);
  return {p:hit?p:pt,hit,n:hit?unit(del):null};
}
