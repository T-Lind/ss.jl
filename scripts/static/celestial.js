// Lighting shares the simulator's reference epoch, not the computer's date.
// At t=0 Greenwich faces the Sun at the vernal equinox. launch_h advances
// both Earth rotation and the idealised annual solar motion from that epoch.
export const EARTH_RATE=7.2921159e-5;
const YEAR=365.256363004*86400, OBLIQUITY=23.43928*Math.PI/180;
export const launchHours=run=>Number.isFinite(Number(run?.params?.launch_h))?Number(run.params.launch_h):0;
export const earthAngle=(t,h=0)=>EARTH_RATE*(t+3600*h);
export const rotateZ=(v,a)=>[Math.cos(a)*v[0]-Math.sin(a)*v[1],Math.sin(a)*v[0]+Math.cos(a)*v[1],v[2]];
export function sunEci(t,h=0) {
  const a=2*Math.PI*(t+3600*h)/YEAR;
  return [Math.cos(a),Math.sin(a)*Math.cos(OBLIQUITY),Math.sin(a)*Math.sin(OBLIQUITY)];
}
export const dot=(a,b)=>a.reduce((n,x,i)=>n+x*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit=v=>v.map(x=>x/(Math.hypot(...v)||1));
export function sunAtSite(t,h,lat,lon) {
  const s=rotateZ(sunEci(t,h),-earthAngle(t,h)),la=lat*Math.PI/180,lo=lon*Math.PI/180;
  const east=[-Math.sin(lo),Math.cos(lo),0],up=[Math.cos(la)*Math.cos(lo),Math.cos(la)*Math.sin(lo),Math.sin(la)];
  const north=cross(up,east);
  return [dot(s,east),dot(s,up),-dot(s,north)]; // east / up / south
}
export function daylight(sun,up=[0,1,0]) {
  const x=Math.max(0,Math.min(1,(dot(sun,up)+.12)/.24));
  return x*x*(3-2*x);
}
// Fraction of direct sunlight visible above a spherical body's limb. The
// Sun's angular radius gives a small transition at sunrise and during eclipse.
export function sunVisibility(sun,point,centre,radius) {
  const rel=point.map((v,i)=>v-centre[i]),distance=Math.hypot(...rel);
  if(!(distance>0))return 0;
  const elevation=Math.asin(Math.max(-1,Math.min(1,dot(sun,rel)/distance)));
  const dip=Math.acos(Math.min(1,radius/distance)),solarRadius=.00465;
  const x=Math.max(0,Math.min(1,(elevation+dip+solarRadius)/(2*solarRadius)));
  return x*x*(3-2*x);
}
// Simulator Moon-fixed frame: zero longitude faces Earth, north is the
// orbital angular momentum. This same frame defines terrain and site coords.
export function moonAxes(position,normal) {
  const x=unit(position.map(v=>-v)),z=unit(normal),y=unit(cross(z,x));
  return [x,y,z];
}
export function moonOrientation(position,normal) {
  const axes=moonAxes(position,normal);
  return [0,1,2].map(i=>axes.map(axis=>axis[i]));
}
export function lunarSun(t,h,position,normal,site) {
  const axes=moonAxes(position,normal),eci=sunEci(t,h),fixed=axes.map(a=>dot(a,eci));
  return [dot(fixed,site.ed),dot(fixed,site.u),dot(fixed,site.ec)];
}
export function lunarEarthMatrix(position,normal,site) {
  const axes=moonAxes(position,normal);
  return [site.ed,site.u,site.ec].flatMap(v=>[0,1,2].map(i=>axes.reduce((n,a,j)=>n+a[i]*v[j],0)));
}
export function moonTexel(direction,width,height) {
  const lat=Math.asin(Math.max(-1,Math.min(1,direction[2]))),lon=Math.atan2(direction[1],direction[0]);
  const x=((Math.floor((lon+Math.PI)/(2*Math.PI)*width)%width)+width)%width;
  const y=Math.max(0,Math.min(height-1,Math.floor((Math.PI/2-lat)/Math.PI*height)));
  return [x,y];
}
export const MOON_MAP_URL=new URL('./moon_map.jpg',import.meta.url).href;
