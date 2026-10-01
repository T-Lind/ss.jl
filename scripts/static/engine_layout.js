// Shared by the browser mesh builder and flight-viewer exhaust anchors.
// Keep the matching Julia implementation (_cluster) covered by parity tests.
export function engineCluster(n,rmax,rbMax) {
  if(n<=1) return [[0,0,Math.min(rbMax,rmax)]];
  if(n>=13) {
    const points=[];
    const ring=(m,r,phase=0)=>{
      for(let i=0;i<m;i++) {
        const a=2*Math.PI*i/m+phase; points.push([r*Math.cos(a),r*Math.sin(a)]);
      }
    };
    if(n===33) {
      // Super Heavy concept: 13 inner engines and 20 on the perimeter.
      ring(3,.18*rmax); ring(10,.47*rmax,Math.PI/10); ring(20,.83*rmax,Math.PI/20);
    } else {
      points.push([0,0]);
      const nr=Math.ceil((Math.sqrt(1+4*(n-1)/3)-1)/2);
      let left=n-1;
      for(let j=1;j<=nr;j++) {
        const m=j===nr?left:Math.min(6*j,left);
        ring(m,rmax*j/(nr+.5),Math.PI/m); left-=m;
      }
    }
    let rb=rbMax;
    for(let i=0;i<points.length;i++) {
      rb=Math.min(rb,rmax-Math.hypot(...points[i]));
      for(let j=0;j<i;j++) rb=Math.min(rb,.46*Math.hypot(points[i][0]-points[j][0],points[i][1]-points[j][1]));
    }
    return points.map(p=>[...p,rb]);
  }
  const centre=n%2===1&&n>=5,m=centre?n-1:n,s=Math.sin(Math.PI/m),rr=rmax/(1+s);
  let rb=Math.min(rbMax,.92*rr*s);
  if(centre) rb=Math.min(rb,.5*rr);
  const out=centre?[[0,0,rb]]:[];
  for(let i=0;i<m;i++) {
    const a=2*Math.PI*i/m+Math.PI/m; out.push([rr*Math.cos(a),rr*Math.sin(a),rb]);
  }
  return out;
}

export function stageBells(n,D,index,count,base=0) {
  const last=index===count-1,bex=index===0?.105*D:last?.085*D:.155*D;
  const pts=engineCluster(n,.4*D,bex),length=index===0?4*pts[0][2]:(last?.22*D:.36*D)*pts[0][2]/bex;
  // Throats enter the thrust structure instead of floating below the skirt.
  return {pts,length,exitX:base+(index===0?.08:.04)*D-length,
    throatRadius:(index===0?.48:last?.53:.29)*pts[0][2]};
}
