import {useEffect,useRef,useState} from 'react';
import * as T from 'three';
import PlanView from './PlanView';
import {loadFullModels,disposeModel} from './fullModels';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import type {MapData,Status,SceneData} from './types';
T.Object3D.DEFAULT_UP.set(0,0,1);
const color={floor:0x1b252a,line:0x33454c,accent:0x6ed9b5};
function mesh(vertices:number[],hex:string,glass=false){
 const geo=new T.BufferGeometry();geo.setAttribute('position',new T.Float32BufferAttribute(vertices,3));geo.computeVertexNormals();
 return new T.Mesh(geo,new T.MeshStandardMaterial({color:hex,metalness:glass?.1:.45,roughness:.55,side:T.DoubleSide,transparent:glass,opacity:glass?.2:1,depthWrite:!glass}));
}
function place(g:T.Object3D,v:number[]){g.position.set(v[0],v[1],v[2]);g.rotation.set(v[3],v[4],v[5],'ZYX');}
function dispose(g:T.Object3D){g.traverse(o=>{const m=o as T.Mesh;m.geometry?.dispose();const materials=m.material?(Array.isArray(m.material)?m.material:[m.material]):[];materials.forEach(x=>x.dispose());});}
export default function World({map,status,assets,view,follow,reset,onSelect,selected,lidar=true}:{map:MapData|null;status:Status|null;assets:SceneData|null;view:string;follow:boolean;reset:number;onSelect:(id:string)=>void;selected:string;lidar?:boolean}){
 const host=useRef<HTMLDivElement>(null);const live=useRef(status);live.current=status;const mode=useRef(view);mode.current=view;
 const instance=useRef<any>(null);const [failure,setFailure]=useState('');const [loading,setLoading]=useState(true);const [detail,setDetail]=useState('');
 useEffect(()=>{
  if(!host.current)return;let renderer:T.WebGLRenderer;
  try{renderer=new T.WebGLRenderer({antialias:true,alpha:false,powerPreference:'high-performance'});}catch{setFailure('WebGL을 사용할 수 없습니다. 브라우저의 하드웨어 가속을 확인하세요.');return;}
  renderer.setPixelRatio(Math.min(devicePixelRatio,2.5));renderer.setClearColor(0x151e23);renderer.outputColorSpace=T.SRGBColorSpace;renderer.toneMapping=T.ACESFilmicToneMapping;renderer.toneMappingExposure=1.4;
  host.current.appendChild(renderer.domElement);
  const scene=new T.Scene();
  scene.add(new T.HemisphereLight(0xddefff,0x52616a,2.7));const sun=new T.DirectionalLight(0xffffff,3.2);sun.position.set(-5,-4,15);scene.add(sun);
  const camera=new T.PerspectiveCamera(42,1,.03,500);camera.position.set(21,-16,20);
  const controls=new OrbitControls(camera,renderer.domElement);controls.target.set(8,4,0);controls.enableDamping=true;controls.maxPolarAngle=Math.PI/2-.03;controls.minDistance=1;controls.maxDistance=100;
  const world=new T.Group();scene.add(world);const robot=new T.Group();scene.add(robot);const cad=new T.Group();scene.add(cad);
  const ring=new T.Mesh(new T.RingGeometry(.7,.73,64),new T.MeshBasicMaterial({color:0x66e8c0,side:T.DoubleSide}));ring.position.z=.015;robot.add(ring);
  const body=mesh(Array.from(new T.BoxGeometry(.95,.63,.24).toNonIndexed().attributes.position.array),'#8b9ba5');body.position.z=.22;robot.add(body);
  // Keep the scan in map coordinates and draw it over opaque and glass equipment.
  // This is a single-height 2D LiDAR scan, not a reconstruction of the CAD surface.
  const laserGeometry=new T.BufferGeometry();
  const laser=new T.Points(laserGeometry,new T.PointsMaterial({color:0x77efc8,size:4,sizeAttenuation:false,depthTest:false,depthWrite:false,transparent:true,opacity:.95,toneMapped:false}));
  laser.renderOrder=1000;scene.add(laser);
  let animation=0;let labels:{node:any;el:HTMLButtonElement}[]=[];let joints:Record<string,T.Group>={};let origins:Record<string,T.Quaternion>={};let doors:any[]=[];
  const ray=new T.Raycaster();const mouse=new T.Vector2();
  const api={renderer,scene,camera,controls,world,robot,cad,laser,labels,joints,origins,doors,body,motionContext:'',motionTime:0};instance.current=api;
  const ro=new ResizeObserver(()=>{if(!host.current)return;const {width,height}=host.current.getBoundingClientRect();renderer.setSize(width,height);camera.aspect=width/Math.max(1,height);camera.updateProjectionMatrix();renderer.render(scene,camera);});ro.observe(host.current);
  let previous=performance.now();let lastJoints=[0,-Math.PI/3,Math.PI/2,0,Math.PI/3,0];
  function animate(now:number){
   animation=requestAnimationFrame(animate);if(mode.current==='2d')return;const dt=Math.min(.1,(now-previous)/1000);previous=now;const s=live.current;
   if(s){const p=new T.Vector3(s.state?.x||0,s.state?.y||0,0);
    const context=`${s.mode}:${s.host}:${s.connected}`;
    const snap=api.motionContext!==context||now-api.motionTime>500||robot.position.distanceTo(p)>1;
    const alpha=snap?1:1-Math.exp(-dt/.08);
    robot.position.lerp(p,alpha);
    const heading=s.state?.theta||0;robot.rotation.z+=Math.atan2(Math.sin(heading-robot.rotation.z),Math.cos(heading-robot.rotation.z))*alpha;
    api.motionContext=context;api.motionTime=now;
    if(s.joints)lastJoints=s.joints;const j=lastJoints;Object.entries(api.joints).forEach(([name,node],i)=>{
     const target=api.origins[name].clone().multiply(new T.Quaternion().setFromAxisAngle(new T.Vector3(0,0,1),j[i]||0));
     node.quaternion.slerp(target,alpha);
    });
    api.doors.forEach((d:any)=>{const side=d.name.endsWith(':1')?'left':'right';d.group.rotation.z=T.MathUtils.degToRad(s.doors[side])*d.sign;});
   }
   controls.update();renderer.render(scene,camera);
   api.labels.forEach(({node,el}:any)=>{const q=new T.Vector3(node.x,node.y,.12).project(camera);el.style.left=`${(q.x+1)*50}%`;el.style.top=`${(1-q.y)*50}%`;el.style.display=q.z>1?'none':'';});
  }animation=requestAnimationFrame(animate);
  return()=>{cancelAnimationFrame(animation);ro.disconnect();controls.dispose();dispose(scene);renderer.dispose();renderer.domElement.remove();api.labels.forEach((x:any)=>x.el.remove());instance.current=null;};
 },[]);
 useEffect(()=>{
  const a=instance.current;if(!a||!map)return;dispose(a.world);a.world.clear();a.labels.forEach((x:any)=>x.el.remove());a.labels=[];
  const floor=new T.Mesh(new T.PlaneGeometry(100,100),new T.MeshStandardMaterial({color:color.floor,roughness:1}));floor.position.set(8,4,-.03);a.world.add(floor);
  const grid=new T.GridHelper(80,80,color.line,color.line);grid.rotation.x=Math.PI/2;grid.position.set(8,4,-.02);(grid.material as T.Material).transparent=true;(grid.material as T.Material).opacity=.45;a.world.add(grid);
  for(const [x,y,xx,yy] of map.walls){const l=Math.hypot(xx-x,yy-y);const wall=new T.Mesh(new T.BoxGeometry(l,.055,.4),new T.MeshStandardMaterial({color:0x526169,roughness:.8,metalness:.2}));wall.position.set((x+xx)/2,(y+yy)/2,.2);wall.rotation.z=Math.atan2(yy-y,xx-x);a.world.add(wall);}
  const positions:number[]=[];for(const [one,two] of map.edges){const n=map.nodes.find(n=>n.id===one),m=map.nodes.find(n=>n.id===two);if(n&&m)positions.push(n.x,n.y,.015,m.x,m.y,.015);}
  const g=new T.BufferGeometry();g.setAttribute('position',new T.Float32BufferAttribute(positions,3));a.world.add(new T.LineSegments(g,new T.LineBasicMaterial({color:0x52786e,transparent:true,opacity:.7})));
  if(map.cloud?.length){const points=map.cloud.filter((_,i)=>i%Math.max(1,Math.floor(map.cloud!.length/20000))===0).flatMap(p=>[p[0],p[1],.005]);const g=new T.BufferGeometry();g.setAttribute('position',new T.Float32BufferAttribute(points,3));a.world.add(new T.Points(g,new T.PointsMaterial({color:0x73858c,size:.025})));}
  for(const node of map.nodes){const dot=new T.Mesh(new T.RingGeometry(.07,.095,24),new T.MeshBasicMaterial({color:node.kind==='dock'?0x7eacd4:0x82b5a3,side:T.DoubleSide}));dot.position.set(node.x,node.y,.025);a.world.add(dot);const el=document.createElement('button');el.className='station-marker';el.textContent=node.id;el.title=`${node.id} 선택`;el.onclick=()=>onSelect(node.id);host.current?.appendChild(el);a.labels.push({node,el});}
 },[map]);
 useEffect(()=>{
  const a=instance.current;if(!a||!assets)return;let cancelled=false;let loaded:Awaited<ReturnType<typeof loadFullModels>>|null=null;
  setLoading(true);setFailure('');
  loadFullModels(assets).then(result=>{
   loaded=result;
   if(cancelled){disposeModel(result.robot);disposeModel(result.cad);return;}
   a.robot.children.filter((x:any)=>x.userData.model).forEach((x:any)=>{dispose(x);a.robot.remove(x);});
   dispose(a.cad);a.cad.clear();a.cad.add(result.cad);a.robot.add(result.robot);a.body.visible=!assets.assets.amr;
   a.joints=result.joints;a.origins=result.origins;a.doors=result.doors;
   setDetail(`원본 전체 모델 · ${Object.values(result.stats).reduce((a,b)=>a+b,0).toLocaleString()} triangles`);
   a.renderer.domElement.dataset.modelStats=JSON.stringify(result.stats);
   a.renderer.render(a.scene,a.camera);setLoading(false);
  }).catch(e=>{if(!cancelled){setFailure('상세 모델 로드 실패: '+e.message);setLoading(false);}});
  return()=>{cancelled=true;if(loaded)Object.values(loaded.raw).forEach(disposeModel);};
 },[assets]);
 useEffect(()=>{const a=instance.current;if(!a||!assets)return;place(a.cad,assets.cad_origin);},[assets]);
 useEffect(()=>{const a=instance.current;if(!a)return;const data=(status?.laser||[]).flatMap(p=>[p[0],p[1],.1965]);a.laser.geometry.setAttribute('position',new T.Float32BufferAttribute(data,3));a.laser.geometry.computeBoundingSphere();a.laser.visible=lidar;a.renderer.domElement.dataset.lidarPoints=String(lidar?data.length/3:0);},[status?.laser,lidar]);
 useEffect(()=>{const a=instance.current;if(!a)return;a.labels.forEach(({node,el}:any)=>el.classList.toggle('selected',node.id===selected));},[selected,map]);
 useEffect(()=>{const a=instance.current;if(!a||mode.current==='2d')return;const s=live.current;const nodes=map?.nodes||[];const xs=nodes.map(n=>n.x),ys=nodes.map(n=>n.y);const cx=xs.length?(Math.min(...xs)+Math.max(...xs))/2:8,cy=ys.length?(Math.min(...ys)+Math.max(...ys))/2:4;const extent=xs.length?Math.max(5,Math.max(...xs)-Math.min(...xs),Math.max(...ys)-Math.min(...ys)):16;const x=follow?(s?.state.x||0):cx,y=follow?(s?.state.y||0):cy;const distance=follow?3.2:extent*1.15;a.controls.target.set(x,y,follow?.4:0);a.camera.position.set(x+distance*.7,y-distance,distance);a.controls.enableRotate=true;a.controls.update();},[reset,follow,map]);
 useEffect(()=>{if(!follow||view==='2d')return;const timer=setInterval(()=>{const a=instance.current,s=live.current;if(!a||!s)return;const target=new T.Vector3(s.state.x||0,s.state.y||0,.2);const delta=target.clone().sub(a.controls.target);a.camera.position.add(delta);a.controls.target.copy(target);},100);return()=>clearInterval(timer);},[follow,view]);
 return <div className="world-container" data-view={view}><div className="world" ref={host} style={{visibility:view==='2d'?'hidden':'visible',pointerEvents:view==='2d'?'none':'auto'}}>{failure&&<div className="world-error">{failure}</div>}{loading&&!failure&&<div className="loading-model">원본 상세 GLB 모델 불러오는 중…</div>}{detail&&<div className="model-detail">{detail}</div>}</div>{view==='2d'&&<PlanView map={map} status={status} follow={follow} reset={reset} selected={selected} lidar={lidar} onSelect={onSelect}/>}</div>;
}
