import {useEffect,useState} from 'react';
import type {Status} from './types';

/** Independent read-only viewer; shares the server capture with the main window. */
export default function CameraPopup(){
 const realsense=new URLSearchParams(location.search).get('source')==='realsense';
 const [status,setStatus]=useState<Status|null>(null),[error,setError]=useState(''),[tick,setTick]=useState(0),[failed,setFailed]=useState(false);
 useEffect(()=>{
  document.title=realsense?'MOMA · RealSense':'MOMA · 실시간 카메라';
  let disposed=false;let timer:ReturnType<typeof setTimeout>;
  const controller=new AbortController();
  async function poll(){
   try{
    const response=await fetch('/api/status',{signal:controller.signal,cache:'no-store'});
    if(!response.ok)throw new Error('서버 응답 오류');
    const next=await response.json();
    if(!disposed){setStatus(next);setError('');}
   }catch{if(!disposed){setError('서버 연결 끊김 · 재연결 중');setStatus(null);}}
   finally{if(!disposed)timer=setTimeout(poll,1000);}
  }
  poll();return()=>{disposed=true;clearTimeout(timer);controller.abort();};
 },[]);
 useEffect(()=>{if(!failed)return;const timer=setTimeout(()=>{setTick(n=>n+1);setFailed(false);},2000);return()=>clearTimeout(timer);},[failed]);
 const camera=realsense?status?.realsense:status?.camera;
 const enabled=realsense||status?.camera_enabled;
 const connected=camera?.connected&&!failed;
 return <main className="camera-popup"><header><strong>{realsense?'RealSense':status?.camera_source==='webcam'?'PC 웹캠':'실시간 카메라'}</strong><span>{connected?'LIVE':'연결 대기'}</span><button onClick={()=>{setFailed(false);setTick(n=>n+1);}}>영상 다시 연결</button></header>
  <div className="camera-popup-video">
   {enabled&&<img key={tick} src={`/api/camera/stream?source=${realsense?'realsense':'current'}&v=${tick}`} alt="실시간 카메라 영상" onError={()=>setFailed(true)}/>}
   {(!connected||error)&&<div className="camera-popup-message">{error||(!enabled?'메인 화면에서 웹캠을 켜거나 영상 소스를 선택하세요.':camera?.error||'영상 수신 대기')}</div>}
  </div><footer>창 모서리를 드래그하면 크기를 조절할 수 있습니다. · 창을 닫아도 메인 화면의 영상은 유지됩니다.</footer>
 </main>;
}
