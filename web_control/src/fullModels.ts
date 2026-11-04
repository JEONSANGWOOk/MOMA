import * as T from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import type {SceneData} from './types';
function transform(g:T.Object3D,v:number[]){g.position.set(v[0],v[1],v[2]);g.rotation.set(v[3],v[4],v[5],'ZYX');}
function assetRotation(g:T.Object3D,spec:any){const r=spec.rotation||[0,0,0];g.rotation.set(...r.map(T.MathUtils.degToRad) as [number,number,number],'XYZ');}
export function disposeModel(g:T.Object3D){g.traverse(o=>{const m=o as T.Mesh;m.geometry?.dispose();for(const material of m.material?(Array.isArray(m.material)?m.material:[m.material]):[]){Object.values(material).forEach(x=>{if(x instanceof T.Texture)x.dispose();});material.dispose();}});}
async function completeGLB(url:string){
 const gltf=await new GLTFLoader().loadAsync(url);const parser=gltf.parser as any;const nodes=parser.json.nodes||[];
 const tag=(root:T.Object3D)=>root.traverse(o=>{const index=parser.associations.get(o)?.nodes;if(index!==undefined){o.userData.sourceNode=index;o.userData.sourceName=nodes[index]?.name||'';}});
 tag(gltf.scene);const present=new Set<number>();gltf.scene.traverse(o=>{if(o.userData.sourceNode!==undefined)present.add(o.userData.sourceNode);});
 const children=new Set<number>(nodes.flatMap((n:any)=>n.children||[]));
 // CAD exporters can leave tool subassemblies outside the default scene.
 // Load their actual node dependencies too; do not silently omit them.
 for(let index=0;index<nodes.length;index++)if(!children.has(index)&&!present.has(index)){
  const root=await parser.getDependency('node',index) as T.Object3D;tag(root);gltf.scene.add(root);root.traverse(o=>{if(o.userData.sourceNode!==undefined)present.add(o.userData.sourceNode);});
 }
 let expected=0;
 for(const node of nodes)if(node.mesh!==undefined)for(const primitive of parser.json.meshes[node.mesh].primitives){const count=parser.json.accessors[primitive.indices??primitive.attributes.POSITION].count;const mode=primitive.mode??4;expected+=mode===4?count/3:(mode===5||mode===6)?Math.max(0,count-2):0;}
 gltf.scene.userData.sourceTriangles=Math.round(expected);
 gltf.scene.updateMatrixWorld(true);return gltf.scene;
}
function sourceNode(root:T.Object3D,name:string){let result:T.Object3D|undefined;root.traverse(o=>{if(o.userData.sourceName===name&&!result)result=o;});if(!result)throw new Error(`원본 GLB 부품 없음: ${name}`);return result;}
function linkVisual(root:T.Object3D,name:string,names:Set<string>,used:Set<T.Object3D>){
 const source=sourceNode(root,name);const group=new T.Group();const inverse=source.matrixWorld.clone().invert();
 function collect(o:T.Object3D){
  if(o!==source&&names.has(o.userData.sourceName))return;
  if((o as T.Mesh).isMesh){used.add(o);const original=o as T.Mesh;const copy=new T.Mesh(original.geometry,original.material);copy.name=original.name;copy.matrixAutoUpdate=false;copy.matrix.multiplyMatrices(inverse,original.matrixWorld);copy.castShadow=true;copy.receiveShadow=true;group.add(copy);}
  o.children.forEach(collect);
 }collect(source);return group;
}
export async function loadFullModels(data:SceneData){
 const entries=await Promise.all(Object.entries(data.assets).map(async([role,spec])=>[role,await completeGLB(spec.url)] as const));
 const raw=Object.fromEntries(entries) as Record<string,T.Object3D>;
 const robot=new T.Group();robot.userData.model=true;const cad=new T.Group();const joints:Record<string,T.Group>={};const origins:Record<string,T.Quaternion>={};const doors:any[]=[];
 try{
  if(raw.amr){const amr=new T.Group();amr.userData.role='amr';assetRotation(amr,data.assets.amr);amr.scale.setScalar(data.assets.amr.scale||1);amr.add(raw.amr);robot.add(amr);}
  const arm=data.assets.arm;
  if(raw.arm&&arm){
   const mount=new T.Group();mount.userData.role='arm';transform(mount,data.mount);mount.scale.setScalar(arm.scale||1);robot.add(mount);
   const names=new Set<string>([...arm.roots,...arm.joints.map((j:any)=>j.child)]);const used=new Set<T.Object3D>();const links:Record<string,T.Group>={};
   names.forEach(name=>{links[name]=linkVisual(raw.arm,name,names,used);});arm.roots.forEach((name:string)=>mount.add(links[name]));
   arm.joints.forEach((j:any)=>{const group=links[j.child];const matrix=new T.Matrix4().set(...j.origin.flat() as [number,number,number,number,number,number,number,number,number,number,number,number,number,number,number,number]);matrix.decompose(group.position,group.quaternion,group.scale);origins[j.name]=group.quaternion.clone();joints[j.name]=group;links[j.parent].add(group);});
   if(raw.tool&&links.J6){const tool=new T.Group();tool.userData.role='tool';tool.position.z=.1;const rotation=new T.Group();assetRotation(rotation,data.assets.tool);rotation.scale.setScalar(data.assets.tool.scale||1);rotation.add(raw.tool);tool.add(rotation);links.J6.add(tool);}
  }
  if(raw.cad){
   const authored=new T.Group();assetRotation(authored,data.assets.cad);authored.add(raw.cad);authored.updateMatrixWorld(true);
   for(const d of data.assets.cad.doors||[]){
    const original=sourceNode(raw.cad,d.name);const world=original.matrixWorld.clone();original.removeFromParent();
    const hinge=new T.Group();hinge.position.fromArray(d.pivot);original.matrixAutoUpdate=false;original.matrix.multiplyMatrices(new T.Matrix4().makeTranslation(-d.pivot[0],-d.pivot[1],-d.pivot[2]),world);hinge.add(original);cad.add(hinge);doors.push({...d,group:hinge});
   }
   cad.add(authored);cad.scale.setScalar(data.assets.cad.scale||1);
  }
  const stats:Record<string,number>={};
  function triangles(root:T.Object3D,role:string){let count=0;function visit(o:T.Object3D){if(o!==root&&o.userData.role&&o.userData.role!==role)return;const m=o as T.Mesh;if(m.isMesh)count+=(m.geometry.index?.count??m.geometry.attributes.position.count)/3;o.children.forEach(visit);}visit(root);return Math.round(count);}
  robot.traverse(o=>{if(o.userData.role)stats[o.userData.role]=triangles(o,o.userData.role);});
  // Detached CAD leaves still belong to the displayed original model.
  let cadTriangles=0;cad.traverse(o=>{const m=o as T.Mesh;if(m.isMesh)cadTriangles+=(m.geometry.index?.count??m.geometry.attributes.position.count)/3;});if(raw.cad)stats.cad=Math.round(cadTriangles);
  for(const [role,root] of Object.entries(raw))if(stats[role]!==root.userData.sourceTriangles)throw new Error(`${role}: 원본 ${root.userData.sourceTriangles} / 표시 ${stats[role]} triangles 불일치`);
  return {robot,cad,joints,origins,doors,stats,raw};
 }catch(e){Object.values(raw).forEach(disposeModel);throw e;}
}
