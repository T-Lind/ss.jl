import {landerLayout,landerExterior,landerInterior,landerDisplay,ModelMesh} from './lander_model.js';
import {engineModel} from './engine_model.js';

const $=id=>document.getElementById(id),cv=$('view');
const gl=cv.getContext('webgl',{antialias:true,preserveDrawingBuffer:true});
if(!gl) {
  $('error').textContent='WebGL is unavailable. Enable hardware acceleration or try another browser.';
  $('export').disabled=true;
} else {
  try { start(); } catch(err) { $('error').textContent=err.message; }
}
function start() {
  const shader=(type,src)=>{
    const s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);
    if(!gl.getShaderParameter(s,gl.COMPILE_STATUS)) throw Error(gl.getShaderInfoLog(s));
    return s;
  };
  const prog=gl.createProgram();
  gl.attachShader(prog,shader(gl.VERTEX_SHADER,`attribute vec3 p,n,c;uniform mat4 vp;varying vec3 N,C,P;
    void main(){N=n;C=c;P=p;gl_Position=vp*vec4(p,1.);}`));
  gl.attachShader(prog,shader(gl.FRAGMENT_SHADER,`precision highp float;varying vec3 N,C,P;uniform vec3 eye;uniform float cabin;
    void main(){vec3 n=normalize(N);if(!gl_FrontFacing)n=-n;
      vec3 l=normalize(vec3(1.,1.7,1.2));vec3 v=normalize(eye-P);
      float d=max(dot(n,l),0.);float sp=pow(max(dot(n,normalize(l+v)),0.),32.);
      vec3 col=C*(.38+.72*d+cabin*.20)+sp*.14;
      gl_FragColor=vec4(pow(col,vec3(.90)),1.);}`));
  gl.linkProgram(prog);
  if(!gl.getProgramParameter(prog,gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(prog));
  const buf=gl.createBuffer(),vp=gl.getUniformLocation(prog,'vp'),eyeLoc=gl.getUniformLocation(prog,'eye'),cabLoc=gl.getUniformLocation(prog,'cabin');
  let vertices=0,L,kind='lander',yaw=.60,pitch=.17,dist=11,drag=null;
  const sub=(a,b)=>a.map((v,i)=>v-b[i]),dot=(a,b)=>a.reduce((v,x,i)=>v+x*b[i],0);
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],unit=a=>a.map(x=>x/Math.hypot(...a));
  const mul=(a,b)=>Array.from({length:16},(_,i)=>{const r=i%4,c=Math.floor(i/4);return a[r]*b[c*4]+a[4+r]*b[c*4+1]+a[8+r]*b[c*4+2]+a[12+r]*b[c*4+3];});
  function reset() { yaw=kind==='cabin'?0:kind==='engines'?.65:.60;pitch=kind==='engines'?-.65:kind==='cabin'?-.19:.17;dist=(kind==='engines'?2.4:2.65)*L.diameter;render(); }
  function rebuild(resetCamera=false) {
    kind=$('model').value;
    const raw=Number($('diameter').value);
    if(!(Number.isFinite(raw)&&raw>=1&&raw<=20)) { $('error').textContent='Enter a diameter from 1 to 20 m.';return; }
    $('error').textContent=''; L=landerLayout(raw);
    $('engine-field').hidden=kind!=='engines';
    let data;
    if(kind==='engines') {
      const result=engineModel(+$('count').value,raw);data=result.vertices;
      $('status').textContent=`${result.bells.pts.length} engines · bell exit ${result.bells.exitX.toFixed(3)} m · throat ${(raw*.08).toFixed(3)} m · bell radius ${result.bells.pts[0][2].toFixed(3)} m`;
    } else {
      data=kind==='cabin'?landerInterior(raw,Array.from({length:10},()=>({c:[.4,.6,.7]}))):landerExterior(raw);
      if(kind==='cabin') {
        const screen=new ModelMesh(1);
        for(let k=0;k<3;k++) {
          const F=landerDisplay(L,k);
          screen.quad(F.at(0,0),F.at(0,1),F.at(1,1),F.at(1,0),[.035,.20,.29]);
          for(let i=1;i<=5;i++) {
            const a=F.at(.15,.14+i*.12),b=F.at(.78,.14+i*.12);
            screen.beam(a,b,.006*L.s,.006*L.s,i===1?[.3,.85,.86]:[.17,.40,.49]);
          }
        }
        data=new Float32Array([...data,...screen.vertices]);
      }
      $('status').textContent=`Deployed envelope ${raw.toFixed(1)} m · two crew stations · nozzle clearance ${L.nozzle[1].toFixed(2)} m · ${data.length/36} triangles`;
    }
    vertices=data.length/12;gl.bindBuffer(gl.ARRAY_BUFFER,buf);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);
    if(resetCamera) reset(); else render();
    // A small diagnostic hook for automated WebGL smoke checks.
    window.__modelInspector={kind,vertices,layout:L,error:()=>gl.getError()};
  }
  function render() {
    if(!L) return;
    const dpr=Math.min(devicePixelRatio||1,2),w=Math.round(cv.clientWidth*dpr),h=Math.round(cv.clientHeight*dpr);
    if(cv.width!==w||cv.height!==h) {cv.width=w;cv.height=h;}
    gl.viewport(0,0,w,h);if(kind==='cabin')gl.clearColor(.08,.15,.20,1);else gl.clearColor(.047,.067,.086,1);gl.enable(gl.DEPTH_TEST);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    let eye,target;
    if(kind==='cabin') {
      eye=[L.eyeX,L.eyeY,0];target=[eye[0]+Math.cos(pitch)*Math.cos(yaw),eye[1]+Math.sin(pitch),eye[2]+Math.cos(pitch)*Math.sin(yaw)];
    } else {
      target=[0,kind==='engines'?.08*L.diameter:2.4*L.s,0];
      eye=[target[0]+dist*Math.cos(pitch)*Math.cos(yaw),target[1]+dist*Math.sin(pitch),target[2]+dist*Math.cos(pitch)*Math.sin(yaw)];
    }
    const z=unit(sub(eye,target)),x=unit(cross([0,1,0],z)),y=cross(z,x);
    const view=[x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,eye),-dot(y,eye),-dot(z,eye),1];
    const fov=kind==='cabin'?1.30: .62,f=1/Math.tan(fov/2),near=.01*L.s,far=Math.max(100,dist*4);
    const projection=[f/(w/h),0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0];
    gl.useProgram(prog);gl.uniformMatrix4fv(vp,false,new Float32Array(mul(projection,view)));
    gl.uniform3fv(eyeLoc,eye);gl.uniform1f(cabLoc,kind==='cabin'?1:0);gl.bindBuffer(gl.ARRAY_BUFFER,buf);
    for(const [name,offset] of [['p',0],['n',12],['c',24]]) {
      const loc=gl.getAttribLocation(prog,name);gl.enableVertexAttribArray(loc);gl.vertexAttribPointer(loc,3,gl.FLOAT,false,48,offset);
    }
    gl.drawArrays(gl.TRIANGLES,0,vertices);
  }
  $('model').addEventListener('change',()=>{
    $('diameter').value=$('model').value==='engines'?9:4.2;rebuild(true);
  });
  $('diameter').addEventListener('input',()=>rebuild(true));$('count').addEventListener('change',()=>rebuild());
  $('reset').addEventListener('click',reset);
  cv.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId);});
  cv.addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-1.4,Math.min(1.4,pitch+(e.clientY-drag[1])*.006));drag=[e.clientX,e.clientY];render();});
  for(const type of ['pointerup','pointercancel','lostpointercapture']) cv.addEventListener(type,()=>drag=null);
  cv.addEventListener('wheel',e=>{e.preventDefault();if(kind!=='cabin')dist=Math.max(L.diameter*.8,Math.min(L.diameter*8,dist*Math.exp(e.deltaY*.001)));render();},{passive:false});
  cv.addEventListener('keydown',e=>{
    if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','=','-','r','R'].includes(e.key)) return;
    e.preventDefault(); if(e.key.toLowerCase()==='r') {reset();return;}
    if(e.key==='ArrowLeft')yaw-=.08;if(e.key==='ArrowRight')yaw+=.08;
    if(e.key==='ArrowUp')pitch=Math.min(1.4,pitch+.08);if(e.key==='ArrowDown')pitch=Math.max(-1.4,pitch-.08);
    if(kind!=='cabin'&&['+','=','-'].includes(e.key))dist=Math.max(L.diameter*.8,Math.min(L.diameter*8,dist*(e.key==='-'?1.1:.9)));
    render();
  });
  $('export').addEventListener('click',()=>{render();const a=document.createElement('a');a.download=`ssjl-${kind}.png`;a.href=cv.toDataURL('image/png');a.click();});
  new ResizeObserver(render).observe(cv);
  const requested=new URLSearchParams(location.search).get('view');
  if(['lander','cabin','engines'].includes(requested)) { $('model').value=requested;$('diameter').value=requested==='engines'?9:4.2; }
  rebuild(true);
}
