import { ModelMesh } from './lander_model.js';

// Use the capsule triangles returned by the same geometry API as the builder
// and flight viewer. Strip off the launcher and stand the heat shield on y=0.
export function capsulePreview(geo, interior = false) {
  const pod = geo.sections.find(s => s.name === 'pod');
  if (!pod || geo.payload?.kind !== 'capsule') throw new Error('Capsule geometry is unavailable.');
  const d = geo.payload.diameter_m, rp = d / 2;
  const topD = geo.stages.at(-1).diameter_m;
  const origin = pod.x0 + .13 * topD + .02;
  const s = d / 4.2, ta = Math.tan(32.5 * Math.PI / 180);
  const xsh = 2.4 * rp - Math.sqrt((2.4 * rp) ** 2 - rp ** 2);
  const xb0 = xsh + .05 * rp, Lc = .74 * rp / ta, xa = xb0 + .085 * rp;
  const xcon = xb0 + .62 * Lc, rcin = rp - ta * (xcon - xb0) - .05 * rp;
  const count = rp < 1.1 ? 1 : rp < 1.3 ? 2 : Math.min(6, Math.max(3, Math.min(
    1 + Math.floor(2 * (.85 * .89 * rp - Math.min(.14 * rp, .30)) / Math.min(.44 * rp, .86)),
    1 + Math.floor(2 * .44 * rp / Math.min(.44 * rp, .86)))));
  const crew = count === 1 ? [0] : count === 2 ? [-.30 * rp, .30 * rp]
    : Array.from({length:count}, (_, i) => (i - (count - 1) / 2) * Math.min(.44 * rp, .86));
  const ln = Math.min(.60 * (rp - ta * (xa + .07 * rp - xb0) - .05 * rp), 1.14);
  // Native capsule axes (forward +x) become preview axes (up +y).
  const point = p => [-p[1], p[0] - origin, p[2]];
  const mesh = new ModelMesh(1);
  for (const sec of geo.sections) {
    if (!['pod', interior ? 'cabin' : 'glass'].includes(sec.name)) continue;
    for (const t of geo.tris.slice(sec.t0 - 1, sec.t1)) {
      const p = [t.slice(0,3), t.slice(3,6), t.slice(6,9)];
      const u = p.reduce((v,q) => v + q[0] - origin, 0) / 3;
      const rad = p.reduce((v,q) => v + Math.hypot(q[1],q[2]), 0) / 3;
      let color = sec.name === 'glass' ? [.04,.12,.18] : [.73,.75,.78];
      if (sec.name === 'pod' && u < xb0) color = [.24,.22,.21];
      if (sec.name === 'cabin') {
        color = u > xcon - .07 * rp ? [.17,.21,.25] : [.38,.42,.46];
        const ratio = rad / Math.max(rcin, .01);
        if (u > xcon - .04 * rp && u < xcon && ratio > .46 && ratio < .78) color = [.03,.18,.22];
      }
      mesh.tri(...p.map(point), color);
    }
  }
  const displays=[[-46,-18],[-15,15],[18,46]].map(([b0,b1])=>{
    const a0=b0*Math.PI/180+.030,a1=b1*Math.PI/180-.030;
    const ac=(a0+a1)/2,half=(a1-a0)/2,r0=.485*rcin,r1=.755*rcin,rc=(r0+r1)/2;
    let H=rc-r0-.014*rcin,S=Math.min((rc-H)*Math.tan(half),Math.sqrt(Math.max(r1*r1-(rc+H)**2,0)));
    if(S/H>420/400)S=H*420/400;else H=S*400/420;
    return {at(u,v){const r=rc+(.5-v)*2*H,t=(u-.5)*2*S;
      return point([origin+xcon-.031*rp,r*Math.cos(ac)-t*Math.sin(ac),r*Math.sin(ac)+t*Math.cos(ac)]);}};
  });
  const height = pod.x1 - origin;
  const seats = crew.map((z,i) => {
    const bay=count===1?1:count===2?(i?2:0):Math.round(i*2/(count-1));
    const angle=[-32,0,32][bay]*Math.PI/180;
    return { eye: point([origin + xa + .30 * rp, .26 * ln, z]),
    target: point([origin + xcon, .62 * rcin*Math.cos(angle), .62*rcin*Math.sin(angle)]), up: [-1,0,0] }; });
  return { vertices: new Float32Array(mesh.vertices),
    layout: { s, diameter:d, height, seats, crew, displays, target:[0,height*.48,0] } };
}
