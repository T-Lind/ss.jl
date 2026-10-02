import {landerLayout,landerExterior,landerInterior,landerDisplay} from './lander_model.js';
import {capsulePreview} from './capsule_preview.js';
import * as api from './api.js';
import * as vehicle from './vehicle.js';

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
  gl.attachShader(prog,shader(gl.VERTEX_SHADER,`attribute vec3 p,n,c,t;uniform mat4 vp;varying vec3 N,C,P,T;
    void main(){N=n;C=c;P=p;T=t;gl_Position=vp*vec4(p,1.);}`));
  gl.attachShader(prog,shader(gl.FRAGMENT_SHADER,`precision highp float;varying vec3 N,C,P,T;uniform vec3 eye;uniform float cabin;uniform sampler2D atlas;
    void main(){vec3 n=normalize(N);if(!gl_FrontFacing)n=-n;
      vec3 l=normalize(vec3(1.,1.7,1.2));vec3 v=normalize(eye-P);
      float d=max(dot(n,l),0.);float sp=pow(max(dot(n,normalize(l+v)),0.),32.);
      vec3 col=C*(.38+.72*d+cabin*.20)+sp*.14;
      gl_FragColor=vec4(mix(pow(col,vec3(.90)),texture2D(atlas,T.xy).rgb,T.z),1.);}`));
  gl.linkProgram(prog);
  if(!gl.getProgramParameter(prog,gl.LINK_STATUS)) throw Error(gl.getProgramInfoLog(prog));
  // Static labels explain the preview without inventing live flight state.
  const canvas=document.createElement('canvas');canvas.width=1260;canvas.height=400;
  const g=canvas.getContext('2d');
  const panels=[['FLIGHT',['ALTITUDE    —','VELOCITY    —','LIVE IN LAUNCH VIEW']],
                ['NAVIGATION',['TRAJECTORY','DISPLAY PAGE','TIME / ZOOM / PAN']],
                ['SYSTEMS',['CABIN LIGHTS','CREW STATION','RCS / RATE NULL']]];
  panels.forEach(([title,lines],i)=>{
    const x=420*i;g.fillStyle='#03100d';g.fillRect(x,0,420,400);
    g.strokeStyle='#185144';g.lineWidth=3;g.strokeRect(x+6,6,408,388);
    g.fillStyle='#72e6b1';g.font='600 27px monospace';g.fillText(title,x+24,48);
    g.font='22px monospace';g.fillStyle='#accfc0';lines.forEach((line,j)=>g.fillText(line,x+24,115+j*55));
    g.fillStyle='#557b6b';g.font='20px monospace';g.fillText('GEOMETRY PREVIEW',x+24,360);
  });
  const texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.LINEAR);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_S,gl.CLAMP_TO_EDGE);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_WRAP_T,gl.CLAMP_TO_EDGE);
  gl.texImage2D(gl.TEXTURE_2D,0,gl.RGBA,gl.RGBA,gl.UNSIGNED_BYTE,canvas);
  const buf=gl.createBuffer(),vp=gl.getUniformLocation(prog,'vp'),eyeLoc=gl.getUniformLocation(prog,'eye'),cabLoc=gl.getUniformLocation(prog,'cabin');
  let vertices=0,L,kind='exterior',craft='capsule',yaw=.60,pitch=.17,dist=11,zoom=1,drag=null,request=0;
  const dimensions={capsule:3.66,lander:4.2};
  const sub=(a,b)=>a.map((v,i)=>v-b[i]),dot=(a,b)=>a.reduce((v,x,i)=>v+x*b[i],0);
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]],unit=a=>a.map(x=>x/Math.hypot(...a));
  const mul=(a,b)=>Array.from({length:16},(_,i)=>{const r=i%4,c=Math.floor(i/4);return a[r]*b[c*4]+a[4+r]*b[c*4+1]+a[8+r]*b[c*4+2]+a[12+r]*b[c*4+3];});
  function reset() { yaw=kind==='cabin'?0:.60;pitch=kind==='cabin'?0:.17;zoom=1;dist=2.65*L.diameter;render(); }
  async function rebuild(resetCamera=false) {
    const id=++request, nextKind=$('model').value, nextCraft=$('craft').value;
    const raw=Number($('diameter').value), minimum=nextCraft==='capsule'?1.1:1;
    $('export').disabled=true;
    if(!(Number.isFinite(raw)&&raw>=minimum&&raw<=20)) {
      $('error').textContent=`Enter a diameter from ${minimum} to 20 m.`;return;
    }
    $('error').textContent='';$('status').textContent='Loading spacecraft…';
    try {
      let data,layout;
      if(nextCraft==='capsule') {
        const params=new URLSearchParams(vehicle.load());
        params.set('mode','orbit');params.set('payload_kind','capsule');
        params.set('pod_dia',raw);params.set('crewed','1');params.set('fairing_on','0');
        const geo=await api.geometry(params.toString());
        if(id!==request)return;
        const preview=capsulePreview(geo,nextKind==='cabin');data=preview.vertices;layout=preview.layout;
      } else {
        layout=landerLayout(raw);
        layout.seats=layout.crew.map(z=>({eye:[layout.eyeX,layout.eyeY,z],target:[layout.panel.x,layout.panel.yc+.20*layout.s,z],up:[0,1,0]}));
        layout.target=[0,2.4*layout.s,0];
        data=nextKind==='cabin'?landerInterior(raw,Array.from({length:10},()=>({c:[.4,.6,.7]}))):landerExterior(raw);
        layout.displays=Array.from({length:3},(_,k)=>landerDisplay(layout,k));
      }
      if(id!==request)return;
      if(nextKind==='cabin') {
        const screens=[];
        layout.displays.forEach((F,k)=>{
          const corners=[[F.at(0,1),k/3,1],[F.at(1,1),(k+1)/3,1],
                         [F.at(1,0),(k+1)/3,0],[F.at(0,0),k/3,0]];
          for(const tri of [[0,1,2],[0,2,3]])for(const j of tri)
            screens.push(...corners[j][0],-1,0,0,1,1,1,corners[j][1],corners[j][2],1);
        });
        data=new Float32Array([...data,...screens]);
      }
      kind=nextKind;craft=nextCraft;L=layout;dimensions[craft]=raw;
      $('station_field').hidden=kind!=='cabin';
      const selected=$('station').value===''?Math.floor((L.seats.length-1)/2):Math.min(+$('station').value||0,L.seats.length-1);
      $('station').replaceChildren(...L.seats.map((_,i)=>{
        const option=document.createElement('option');option.value=i;
        option.textContent=craft==='lander'?(i?'Pilot':'Commander'):`Couch ${i+1}`;return option;
      }));$('station').value=selected;
      vertices=data.length/12;gl.bindBuffer(gl.ARRAY_BUFFER,buf);gl.bufferData(gl.ARRAY_BUFFER,data,gl.STATIC_DRAW);
      $('status').textContent=`${craft==='lander'?'Deployed envelope':'Body diameter'} ${raw.toFixed(2)} m · ${L.seats.length} crew stations · ${kind==='cabin'?'cockpit preview':'exterior'}`;
      $('export').disabled=false;
      if(resetCamera)reset();else render();
      window.__modelInspector={kind,craft,vertices,layout:L,error:()=>gl.getError()};
    } catch(err) {
      if(id!==request)return;
      $('error').textContent=err.message;$('status').textContent='Preview unavailable. Check the dimensions and try again.';
    }
  }
  function render() {
    if(!L || !cv.clientWidth || !cv.clientHeight) return;
    const dpr=Math.min(devicePixelRatio||1,2),w=Math.round(cv.clientWidth*dpr),h=Math.round(cv.clientHeight*dpr);
    if(cv.width!==w||cv.height!==h) {cv.width=w;cv.height=h;}
    gl.viewport(0,0,w,h);if(kind==='cabin')gl.clearColor(.08,.15,.20,1);else gl.clearColor(.047,.067,.086,1);gl.enable(gl.DEPTH_TEST);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    let eye,target,up=[0,1,0];
    if(kind==='cabin') {
      const seat=L.seats[+$('station').value||0];eye=seat.eye;up=seat.up;
      const f=unit(sub(seat.target,eye)),r=unit(cross(f,up)),u=cross(r,f);
      const dir=f.map((v,i)=>v*Math.cos(yaw)*Math.cos(pitch)+r[i]*Math.sin(yaw)*Math.cos(pitch)+u[i]*Math.sin(pitch));
      target=eye.map((v,i)=>v+dir[i]);
    } else {
      target=L.target;
      eye=[target[0]+dist*Math.cos(pitch)*Math.cos(yaw),target[1]+dist*Math.sin(pitch),target[2]+dist*Math.cos(pitch)*Math.sin(yaw)];
    }
    const z=unit(sub(eye,target)),x=unit(cross(up,z)),y=cross(z,x);
    const view=[x[0],y[0],z[0],0,x[1],y[1],z[1],0,x[2],y[2],z[2],0,-dot(x,eye),-dot(y,eye),-dot(z,eye),1];
    const fov=(kind==='cabin'?1.30:.62)/zoom,f=1/Math.tan(fov/2),near=.01*L.s,far=Math.max(100,dist*4);
    const projection=[f/(w/h),0,0,0,0,f,0,0,0,0,(far+near)/(near-far),-1,0,0,2*far*near/(near-far),0];
    gl.useProgram(prog);gl.uniformMatrix4fv(vp,false,new Float32Array(mul(projection,view)));
    gl.uniform3fv(eyeLoc,eye);gl.uniform1f(cabLoc,kind==='cabin'?1:0);gl.bindBuffer(gl.ARRAY_BUFFER,buf);
    for(const [name,offset] of [['p',0],['n',12],['c',24],['t',36]]) {
      const loc=gl.getAttribLocation(prog,name);gl.enableVertexAttribArray(loc);gl.vertexAttribPointer(loc,3,gl.FLOAT,false,48,offset);
    }
    gl.drawArrays(gl.TRIANGLES,0,vertices);
  }
  $('craft').addEventListener('change',()=>{
    const craft=$('craft').value;$('diameter').min=craft==='capsule'?1.1:1;
    $('diameter').value=dimensions[craft];$('diameter_label').textContent=craft==='lander'?'Deployed envelope (m)':'Body diameter (m)';rebuild(true);
  });
  $('model').addEventListener('change',()=>rebuild(true));
  $('diameter').addEventListener('input',()=>rebuild(true));
  $('station').addEventListener('change',()=>{if(L)reset();});
  $('reset').addEventListener('click',()=>{if(L)reset();});
  cv.addEventListener('pointerdown',e=>{if(!L)return;cv.focus();drag=[e.clientX,e.clientY];cv.setPointerCapture(e.pointerId);});
  cv.addEventListener('pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag[0])*.008;pitch=Math.max(-1.4,Math.min(1.4,pitch+(kind==='cabin'?-1:1)*(e.clientY-drag[1])*.006));drag=[e.clientX,e.clientY];render();});
  for(const type of ['pointerup','pointercancel','lostpointercapture']) cv.addEventListener(type,()=>drag=null);
  cv.addEventListener('wheel',e=>{e.preventDefault();if(!L)return;if(kind==='cabin')zoom=Math.max(.9,Math.min(3,zoom*Math.exp(-e.deltaY*.001)));else dist=Math.max(L.diameter*.8,Math.min(L.diameter*8,dist*Math.exp(e.deltaY*.001)));render();},{passive:false});
  cv.addEventListener('keydown',e=>{
    if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','=','-','r','R'].includes(e.key)) return;
    if(!L)return; e.preventDefault(); if(e.key.toLowerCase()==='r') {reset();return;}
    if(e.key==='ArrowLeft')yaw-=.08;if(e.key==='ArrowRight')yaw+=.08;
    if(e.key==='ArrowUp')pitch=Math.min(1.4,pitch+.08);if(e.key==='ArrowDown')pitch=Math.max(-1.4,pitch-.08);
    if(['+','=','-'].includes(e.key)) {if(kind==='cabin')zoom=Math.max(.9,Math.min(3,zoom*(e.key==='-'?.9:1.1)));else dist=Math.max(L.diameter*.8,Math.min(L.diameter*8,dist*(e.key==='-'?1.1:.9)));}
    render();
  });
  $('export').addEventListener('click',()=>{render();const a=document.createElement('a');a.download=`ssjl-${craft}-${kind}.png`;a.href=cv.toDataURL('image/png');a.click();});
  new ResizeObserver(render).observe(cv);
  const query=new URLSearchParams(location.search),saved=new URLSearchParams(vehicle.load());
  const requested=query.get('view');
  const initialCraft=query.get('craft')||(['lander','cabin'].includes(requested)?'lander':'capsule');
  $('craft').value=initialCraft==='lander'?'lander':'capsule';
  $('model').value=requested==='cabin'?'cabin':'exterior';
  const selectedCraft=$('craft').value,stored=Number(saved.get(selectedCraft==='lander'?'l_diameter':'pod_dia'));
  $('diameter').value=query.get('diameter')||(stored>=(selectedCraft==='capsule'?1.1:1)?stored:dimensions[selectedCraft]);
  $('diameter').min=selectedCraft==='capsule'?1.1:1;
  $('diameter_label').textContent=selectedCraft==='lander'?'Deployed envelope (m)':'Body diameter (m)';
  rebuild(true);
}
