import {ModelMesh} from './lander_model.js';
import {stageBells} from './engine_layout.js';

// A cut-away first-stage thrust section. Dimensions and exits match rocket_mesh.
export function engineModel(count=33,diameter=9) {
  const m=new ModelMesh(1),B=stageBells(count,diameter,0,2),r=diameter/2;
  m.tube([0,0,0],[0,.28*diameter,0],.8*r,[.45,.50,.55],48,r,false);
  m.tube([0,.07*diameter,0],[0,.09*diameter,0],.8*r,[.15,.19,.23],48);
  for(const [x,z,rb] of B.pts) {
    const bell=new ModelMesh(1);
    bell.bell(B.exitX,.08*diameter,rb,B.throatRadius,[.49,.55,.61]);
    for(let i=0;i<bell.vertices.length;i+=12) {
      bell.vertices[i]+=x; bell.vertices[i+2]+=z;
    }
    m.vertices.push(...bell.vertices);
  }
  return {vertices:new Float32Array(m.vertices),bells:B};
}
