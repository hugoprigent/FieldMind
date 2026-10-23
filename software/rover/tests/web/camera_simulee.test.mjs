import {test} from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from '../../fieldmind/web/vendor/three/three.module.js';
import {configurerCamera} from '../../fieldmind/web/js/camera_simulee.js';

test('projection WebGL : mêmes pixels que le modèle OpenCV, même après rotation du rover',()=>{
  const angle=.18,c=Math.cos(angle),s=Math.sin(angle);
  const r=[[0,-s,c],[-1,0,0],[0,-c,-s]],k=[400,410,319.5,179.5];
  for(const cap of [0,.7,-1.2])for(const y of [.06,-.06]) {
    const centre=[.415,y,.265],pose=[2,-3,cap];
    const message={pose,largeur:640,hauteur:360,cameras:{gauche:{centre,rotation:r,k}}};
    const camera=new THREE.PerspectiveCamera();configurerCamera(camera,message,'gauche');
    for(const p of [[2,0,.1],[3,.4,0],[1,-.2,.3]]) {
      const d=p.map((v,i)=>v-centre[i]);
      const cv=[0,1,2].map(j=>r.reduce((s,row,i)=>s+row[j]*d[i],0));
      const u=k[0]*cv[0]/cv[2]+k[2],v=k[1]*cv[1]/cv[2]+k[3];
      const monde=new THREE.Vector3(pose[0]+Math.cos(cap)*p[0]-Math.sin(cap)*p[1],pose[1]+Math.sin(cap)*p[0]+Math.cos(cap)*p[1],p[2]).project(camera);
      assert.ok(Math.abs((monde.x+1)*320-.5-u)<1e-8);
      assert.ok(Math.abs((1-monde.y)*180-.5-v)<1e-8);
    }
  }
});
