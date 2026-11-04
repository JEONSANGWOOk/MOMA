import {useEffect,useRef} from 'react';
import type {MapData,Status} from './types';

type Props={map:MapData|null;status:Status|null;follow:boolean;reset:number;selected:string;lidar:boolean;onSelect:(id:string)=>void};
/** A real Canvas2D map. No perspective, lighting, extruded walls or GLB models. */
export default function PlanView(props:Props){
 const canvas=useRef<HTMLCanvasElement>(null);const latest=useRef(props);latest.current=props;
 const camera=useRef({x:0,y:0,scale:30});const fitPending=useRef(true);const dirty=useRef(true);
 useEffect(()=>{dirty.current=true;fitPending.current=true;},[props.map,props.reset]);
 useEffect(()=>{dirty.current=true;},[props.selected]);
 useEffect(()=>{
  const element=canvas.current;if(!element)return;const ctx=element.getContext('2d');if(!ctx)return;
  const background=document.createElement('canvas');const bg=background.getContext('2d')!;
  let width=1,height=1,dpr=1,frame=0,previous=0;let drag:{x:number;y:number;cx:number;cy:number;distance:number}|null=null;
  const project=(x:number,y:number):[number,number]=>[(x-camera.current.x)*camera.current.scale+width/2,height/2-(y-camera.current.y)*camera.current.scale];
  function fit(){
   const map=latest.current.map;if(!map)return;
   let x0=Infinity,y0=Infinity,x1=-Infinity,y1=-Infinity;
   function include(x:number,y:number){if(!Number.isFinite(x)||!Number.isFinite(y))return;x0=Math.min(x0,x);x1=Math.max(x1,x);y0=Math.min(y0,y);y1=Math.max(y1,y);}
   map.nodes.forEach(n=>include(n.x,n.y));map.walls.forEach(w=>{include(w[0],w[1]);include(w[2],w[3]);});
   map.cloud?.forEach(p=>include(p[0],p[1]));
   if(!Number.isFinite(x0)){x0=0;y0=0;x1=10;y1=10;}
   camera.current={x:(x0+x1)/2,y:(y0+y1)/2,scale:Math.max(.05,Math.min(800,(width-90)/Math.max(2,x1-x0),(height-90)/Math.max(2,y1-y0)))};
   fitPending.current=false;dirty.current=true;
  }
  function line(context:CanvasRenderingContext2D,x:number,y:number,xx:number,yy:number){context.moveTo(...project(x,y));context.lineTo(...project(xx,yy));}
  function staticMap(){
   bg.setTransform(dpr,0,0,dpr,0,0);bg.fillStyle='#17252d';bg.fillRect(0,0,width,height);
   const c=camera.current;const step=10**Math.floor(Math.log10(50/c.scale));
   const minX=c.x-width/(2*c.scale),maxX=c.x+width/(2*c.scale),minY=c.y-height/(2*c.scale),maxY=c.y+height/(2*c.scale);
   bg.strokeStyle='#273943';bg.lineWidth=1;bg.beginPath();
   for(let x=Math.ceil(minX/step)*step;x<=maxX;x+=step)line(bg,x,minY,x,maxY);
   for(let y=Math.ceil(minY/step)*step;y<=maxY;y+=step)line(bg,minX,y,maxX,y);
   bg.stroke();const map=latest.current.map;if(!map){dirty.current=false;return;}
   bg.fillStyle='#8296a1';
   for(const p of map.cloud||[]){const [x,y]=project(p[0],p[1]);if(x>=0&&x<=width&&y>=0&&y<=height)bg.fillRect(Math.round(x),Math.round(y),1.5,1.5);}
   bg.strokeStyle='#9baab3';bg.lineWidth=2;bg.beginPath();map.walls.forEach(w=>line(bg,w[0],w[1],w[2],w[3]));bg.stroke();
   bg.setLineDash([6,4]);bg.strokeStyle='#d98082';bg.beginPath();map.virtual_walls?.forEach(w=>line(bg,w[0],w[1],w[2],w[3]));bg.stroke();bg.setLineDash([]);
   const nodes=new Map(map.nodes.map(n=>[n.id,n]));bg.strokeStyle='#507f75';bg.lineWidth=1.2;bg.beginPath();
   for(const [a,b] of map.edges){const one=nodes.get(a),two=nodes.get(b);if(one&&two)line(bg,one.x,one.y,two.x,two.y);}bg.stroke();
   for(const node of map.nodes){
    const [x,y]=project(node.x,node.y);const selected=node.id===latest.current.selected;
    bg.fillStyle=selected?'#b8efd1':node.kind==='dock'?'#91bde6':'#78b8a2';bg.beginPath();bg.arc(x,y,selected?5:3.5,0,Math.PI*2);bg.fill();
    bg.font=`${selected?'bold ':''}10px Consolas,monospace`;const textWidth=bg.measureText(node.id).width;
    bg.fillStyle=selected?'#b8efd1':'#1a2e35';bg.fillRect(x-textWidth/2-5,y-25,textWidth+10,17);
    bg.fillStyle=selected?'#153a2a':'#bdd7ce';bg.textAlign='center';bg.fillText(node.id,x,y-13);
   }
   dirty.current=false;
  }
  function draw(now:number){
   frame=requestAnimationFrame(draw);if(now-previous<30)return;previous=now;
   const p=latest.current;
   if(fitPending.current&&p.map)fit();
   if(p.follow&&p.status){const {x,y}=p.status.state;if(typeof x==='number'&&typeof y==='number'&&(camera.current.x!==x||camera.current.y!==y)){camera.current.x=x;camera.current.y=y;dirty.current=true;}}
   if(dirty.current)staticMap();
   ctx!.setTransform(1,0,0,1,0,0);ctx!.clearRect(0,0,element!.width,element!.height);ctx!.drawImage(background,0,0);ctx!.setTransform(dpr,0,0,dpr,0,0);
   const points=p.lidar?(p.status?.laser||[]):[];ctx!.fillStyle='#83f1c4';
   for(const point of points){const [x,y]=project(point[0],point[1]);if(x>=0&&x<=width&&y>=0&&y<=height)ctx!.fillRect(x-1.5,y-1.5,3,3);}
   element!.dataset.lidarPoints=String(points.length);
   const state=p.status?.state;
   if(state&&typeof state.x==='number'&&typeof state.y==='number'){
    const [x,y]=project(state.x,state.y);const c=camera.current.scale;
    ctx!.save();ctx!.translate(x,y);ctx!.rotate(-(state.theta||0));
    ctx!.strokeStyle='#a3e3c8';ctx!.fillStyle='#234b43';ctx!.lineWidth=1.8;
    ctx!.beginPath();ctx!.arc(0,0,Math.max(11,.65*c),0,Math.PI*2);ctx!.stroke();
    ctx!.fillRect(-Math.max(7,.479*c),-Math.max(5,.316*c),Math.max(14,.958*c),Math.max(10,.632*c));ctx!.strokeRect(-Math.max(7,.479*c),-Math.max(5,.316*c),Math.max(14,.958*c),Math.max(10,.632*c));
    ctx!.fillStyle='#bff8de';ctx!.beginPath();ctx!.moveTo(Math.max(19,.9*c),0);ctx!.lineTo(Math.max(8,.5*c),-5);ctx!.lineTo(Math.max(8,.5*c),5);ctx!.closePath();ctx!.fill();ctx!.restore();
    ctx!.fillStyle='#d4eee3';ctx!.font='bold 10px Consolas,monospace';ctx!.textAlign='center';ctx!.fillText('MOMA–01',x,y+Math.max(22,.65*c+15));
   }
   const meters=10**Math.floor(Math.log10(90/camera.current.scale));const length=meters*camera.current.scale;
   ctx!.strokeStyle='#adc7ce';ctx!.lineWidth=1;ctx!.beginPath();ctx!.moveTo(width-25-length,height-44);ctx!.lineTo(width-25,height-44);ctx!.stroke();
   ctx!.fillStyle='#a4bbc5';ctx!.font='10px Consolas,monospace';ctx!.textAlign='center';ctx!.fillText(`${meters} m`,width-25-length/2,height-50);
  }
  const resize=new ResizeObserver(()=>{const rect=element.parentElement!.getBoundingClientRect();width=Math.max(1,rect.width);height=Math.max(1,rect.height);dpr=Math.min(devicePixelRatio,2.5);element.width=background.width=Math.round(width*dpr);element.height=background.height=Math.round(height*dpr);element.style.width=`${width}px`;element.style.height=`${height}px`;dirty.current=true;fitPending.current=true;});resize.observe(element.parentElement!);
  const down=(e:PointerEvent)=>{drag={x:e.offsetX,y:e.offsetY,cx:camera.current.x,cy:camera.current.y,distance:0};element.setPointerCapture(e.pointerId);};
  const move=(e:PointerEvent)=>{if(!drag)return;const dx=e.offsetX-drag.x,dy=e.offsetY-drag.y;drag.distance=Math.max(drag.distance,Math.hypot(dx,dy));camera.current.x=drag.cx-dx/camera.current.scale;camera.current.y=drag.cy+dy/camera.current.scale;dirty.current=true;};
  const up=(e:PointerEvent)=>{if(drag&&drag.distance<4&&e.button===0){const nodes=latest.current.map?.nodes||[];let closest:string|undefined,best=16;for(const node of nodes){const [x,y]=project(node.x,node.y);const distance=Math.hypot(e.offsetX-x,e.offsetY-y);if(distance<best){best=distance;closest=node.id;}}if(closest)latest.current.onSelect(closest);}drag=null;};
  const wheel=(e:WheelEvent)=>{e.preventDefault();const c=camera.current;const x=c.x+(e.offsetX-width/2)/c.scale,y=c.y-(e.offsetY-height/2)/c.scale;const scale=Math.max(.05,Math.min(800,c.scale*Math.exp(-e.deltaY*.001)));c.x=x-(e.offsetX-width/2)/scale;c.y=y+(e.offsetY-height/2)/scale;c.scale=scale;dirty.current=true;};
  const menu=(e:Event)=>e.preventDefault();
  element.addEventListener('pointerdown',down);element.addEventListener('pointermove',move);element.addEventListener('pointerup',up);element.addEventListener('pointercancel',up);element.addEventListener('wheel',wheel,{passive:false});element.addEventListener('contextmenu',menu);
  frame=requestAnimationFrame(draw);
  return()=>{cancelAnimationFrame(frame);resize.disconnect();element.removeEventListener('pointerdown',down);element.removeEventListener('pointermove',move);element.removeEventListener('pointerup',up);element.removeEventListener('pointercancel',up);element.removeEventListener('wheel',wheel);element.removeEventListener('contextmenu',menu);};
 },[]);
 return <div className="plan-view"><canvas ref={canvas} data-view="flat-2d" aria-label="2D 평면 지도"/><span className="plan-caption">2D 평면 지도 · 드래그 이동 · 휠 확대</span></div>;
}
