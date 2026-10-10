"""D455 marker pose preview. Camera coordinates only; never sends robot commands."""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

DICTIONARY=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMETERS=cv2.aruco.DetectorParameters()
PARAMETERS.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
# Smaller 50/100/250 dictionaries are prefixes of the corresponding 1000 family.
# Search each full family once so a physical marker is not counted repeatedly.
DICTIONARY_NAMES=('DICT_4X4_1000','DICT_5X5_1000','DICT_6X6_1000',
                  'DICT_7X7_1000','DICT_ARUCO_ORIGINAL','DICT_ARUCO_MIP_36h12')
DETECTOR=cv2.aruco.ArucoDetector(
    [cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,name)) for name in DICTIONARY_NAMES],PARAMETERS)


def detect_all(image):
    corners,ids,_,indices=DETECTOR.detectMarkersMultiDict(image)
    if ids is None:return []
    return [(polygon,int(marker_id),DICTIONARY_NAMES[int(index)])
            for polygon,marker_id,index in zip(corners,ids.ravel(),indices.ravel())]


def render_preview(image,observations):
    # Keep details outside the camera image; OpenCV's default blue ID labels
    # are deliberately replaced with white text on opaque, dark backgrounds.
    height,width=image.shape[:2];row_height=76;panel_width=400;header=64
    rows=max(1,(height-header-30)//row_height)
    columns=max(1,math.ceil(len(observations)/rows))
    canvas=np.full((height,width+columns*panel_width,3),(24,24,24),dtype=np.uint8)
    canvas[:,:width]=image
    colors=[(0,230,255),(80,255,120),(255,180,80),(220,120,255),(180,255,255)]
    boxes=[];occupied=[]
    for item in observations:
        p=np.asarray(item['corners_px'])
        lo=p.min(axis=0);hi=p.max(axis=0)
        boxes.append((lo[0]-3,lo[1]-3,hi[0]+3,hi[1]+3))
    def overlaps(a,b):return a[0]<b[2] and a[2]>b[0] and a[1]<b[3] and a[3]>b[1]
    for index,item in enumerate(observations):
        color=colors[index%len(colors)];number=index+1
        polygon=np.rint(item['corners_px']).astype(np.int32)
        cv2.polylines(canvas,[polygon],True,color,2,cv2.LINE_AA)
        # Only a compact reference number near the marker; do not cover another
        # marker or badge when the camera sees closely packed printed sheets.
        badge=f'#{number}';(tw,th),base=cv2.getTextSize(badge,cv2.FONT_HERSHEY_SIMPLEX,.55,2)
        bw,bh=tw+12,th+base+8;x0,y0,x1,y1=boxes[index]
        candidates=[(x0,y0-bh-4),(x1+4,y0),(x0,y1+4),(x0-bw-4,y0)]
        for bx,by in candidates:
            bx=int(bx);by=int(by);rect=(bx,by,bx+bw,by+bh)
            if bx<0 or by<0 or rect[2]>width or rect[3]>height:continue
            if any(overlaps(rect,b) for b in boxes+occupied):continue
            cv2.rectangle(canvas,(bx,by),(bx+bw,by+bh),(0,0,0),-1)
            cv2.rectangle(canvas,(bx,by),(bx+bw,by+bh),color,1)
            cv2.putText(canvas,badge,(bx+6,by+th+4),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2,cv2.LINE_AA)
            occupied.append(rect);break
        col,row=divmod(index,rows);left=width+col*panel_width+16;top=header+row*row_height
        cv2.rectangle(canvas,(left,top),(left+5,top+62),color,-1)
        cv2.putText(canvas,f'#{number}  ID {item["id"]}',(left+16,top+19),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(canvas,item['dictionary'].removeprefix('DICT_'),(left+16,top+39),cv2.FONT_HERSHEY_SIMPLEX,.46,(220,220,220),1,cv2.LINE_AA)
        xyz=item['camera_xyz_m']
        text=f'X {xyz[0]*1000:.0f}  Y {xyz[1]*1000:.0f}  Z {xyz[2]*1000:.0f} mm' if xyz is not None else 'Position unavailable'
        cv2.putText(canvas,text,(left+16,top+60),cv2.FONT_HERSHEY_SIMPLEX,.46,(255,255,255),1,cv2.LINE_AA)
    cv2.putText(canvas,f'ArUco markers: {len(observations)}',(width+16,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(canvas,'Camera coordinates / marker size: see settings',(width+16,48),cv2.FONT_HERSHEY_SIMPLEX,.42,(200,200,200),1,cv2.LINE_AA)
    cv2.putText(canvas,'ESC / Q: close',(width+16,height-12),cv2.FONT_HERSHEY_SIMPLEX,.48,(220,220,220),1,cv2.LINE_AA)
    return canvas


def estimate(corners, size_m, camera, distortion):
    half=size_m/2
    points=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]],dtype=np.float64)
    image=np.asarray(corners,dtype=np.float64).reshape(4,2)
    ok,rotations,translations,_=cv2.solvePnPGeneric(points,image,camera,distortion,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates=[]
    if ok:
        for rotation,translation in zip(rotations,translations):
            rotation=np.asarray(rotation);translation=np.asarray(translation)
            if not np.isfinite(rotation).all() or not np.isfinite(translation).all():continue
            matrix=cv2.Rodrigues(rotation)[0]
            if np.any((points@matrix.T+translation.reshape(1,3))[:,2]<=0):continue
            projected=cv2.projectPoints(points,rotation,translation,camera,distortion)[0].reshape(4,2)
            error=float(np.sqrt(np.mean(np.sum((projected-image)**2,axis=1))))
            candidates.append((error,rotation,translation))
    return min(candidates,key=lambda item:item[0]) if candidates else None


def generate(folder,size_mm,marker_id):
    folder.mkdir(parents=True,exist_ok=True)
    marker=cv2.aruco.generateImageMarker(DICTIONARY,marker_id,600)
    image=cv2.copyMakeBorder(marker,60,60,60,60,cv2.BORDER_CONSTANT,value=255)
    png=folder/f'aruco_4x4_50_id{marker_id}.png'
    if not cv2.imwrite(str(png),image):raise RuntimeError('마커 이미지 저장 실패')
    # SVG dimensions preserve the physical black-border size when printed at 100%.
    cells=cv2.aruco.generateImageMarker(DICTIONARY,marker_id,6)
    margin=size_mm*.1;total=size_mm+2*margin;step=size_mm/6
    blocks=''.join(f'<rect x="{margin+x*step}" y="{margin+y*step}" width="{step}" height="{step}"/>'
        for y in range(6) for x in range(6) if cells[y,x]==0)
    svg=folder/f'aruco_4x4_50_id{marker_id}_{size_mm:g}mm.svg'
    svg.write_text(f'<svg xmlns="http://www.w3.org/2000/svg" width="{total}mm" height="{total}mm" viewBox="0 0 {total} {total}"><rect width="100%" height="100%" fill="white"/><g fill="black" shape-rendering="crispEdges">{blocks}</g></svg>',encoding='utf-8')
    html=folder/'print_marker.html'
    html.write_text(f'<!doctype html><html lang="ko"><meta charset="utf-8"><title>ArUco 마커 출력</title><style>@page{{size:A4;margin:20mm}}img{{width:{total}mm;height:{total}mm}}@media print{{button{{display:none}}}}</style><body><button onclick="window.print()">인쇄</button><p>DICT_4X4_50 · ID {marker_id} · 검은 사각형 한 변 {size_mm:g}mm</p><img src="{svg.name}"><p>인쇄 배율 100% / 실제 크기. 출력 후 검은 외곽 사각형을 자로 측정하세요.</p></body></html>',encoding='utf-8')
    print('마커 출력 페이지:',html.resolve())


def pose_inputs(rs,intr,polygon,camera,distortion):
    # D455 RGB can report inverse Brown, which is not OpenCV's forward Brown model.
    if intr.model==rs.distortion.inverse_brown_conrady:
        rays=np.asarray([rs.rs2_deproject_pixel_to_point(intr,p.tolist(),1.)
                         for p in polygon.reshape(4,2)],dtype=np.float64)
        pixels=rays[:,:2]/rays[:,2,None]
        pixels=pixels*np.array([intr.fx,intr.fy])+np.array([intr.ppx,intr.ppy])
        return pixels,camera,np.zeros(5),False
    if np.any(np.abs(distortion)>1e-12) and intr.model!=rs.distortion.brown_conrady:
        raise RuntimeError('이 RGB 왜곡 모델은 별도 보정이 필요합니다: '+str(intr.model))
    return polygon,camera,distortion,True


def self_test():
    marker=cv2.aruco.generateImageMarker(DICTIONARY,0,300)
    canvas=cv2.copyMakeBorder(marker,60,60,60,60,cv2.BORDER_CONSTANT,value=255)
    detections=detect_all(canvas)
    assert len(detections)==1 and detections[0][1:]==(0,'DICT_4X4_1000')
    # Mixed families, IDs outside the old 0..49 range, and repeated physical IDs.
    sheet=np.full((660,880),255,dtype=np.uint8)
    expected=[]
    examples=[(name,73) for name in DICTIONARY_NAMES]+[('DICT_4X4_1000',0)]*2
    for slot,(name,marker_id) in enumerate(examples):
        y=20+(slot//4)*320;x=20+(slot%4)*220
        sheet[y:y+180,x:x+180]=cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,name)),marker_id,180)
        expected.append((marker_id,name))
    found=detect_all(sheet)
    assert sorted((item[1],item[2]) for item in found)==sorted(expected),found
    assert detect_all(np.full((480,640),255,dtype=np.uint8))==[]
    camera=np.array([[600.,0,320],[0,600.,240],[0,0,1]])
    points=np.array([[-.05,.05,0],[.05,.05,0],[.05,-.05,0],[-.05,-.05,0]])
    rotation=np.array([3.,.1,.2]);translation=np.array([.02,-.03,.8])
    pixels=cv2.projectPoints(points,rotation,translation,camera,np.zeros(5))[0]
    result=estimate(pixels,.1,camera,np.zeros(5))
    assert result and result[0]<1e-5
    assert np.allclose(result[2].ravel(),translation,atol=1e-5)
    doubled=estimate(pixels,.2,camera,np.zeros(5))
    assert np.allclose(doubled[2].ravel(),translation*2,atol=1e-5)
    assert estimate(np.zeros((4,2)),.1,camera,np.zeros(5)) is None
    import pyrealsense2 as rs
    intr=rs.intrinsics();intr.width=640;intr.height=480;intr.fx=600;intr.fy=600;intr.ppx=320;intr.ppy=240
    intr.model=rs.distortion.inverse_brown_conrady;intr.coeffs=[0.,0.,0.,0.,0.]
    prepared,k,d,axes=pose_inputs(rs,intr,pixels,camera,np.zeros(5))
    result=estimate(prepared,.1,k,d)
    assert result and np.allclose(result[2].ravel(),translation,atol=1e-5) and not axes
    intr.coeffs=[.1,-.01,.001,.002,0.]
    prepared,k,d,axes=pose_inputs(rs,intr,pixels,camera,np.asarray(intr.coeffs))
    assert not np.allclose(prepared,pixels.reshape(4,2)) and np.isfinite(prepared).all()
    print('PASS: 8 simultaneous markers / 6 families, repeated IDs, blank frame, camera XYZ recovery, physical size scaling, invalid pose rejection')


def run(args):
    import pyrealsense2 as rs
    devices=list(rs.context().query_devices())
    if not devices:raise RuntimeError('D455가 없습니다. USB 연결과 Viewer의 장치 표시를 확인하세요.')
    for device in devices:
        print(json.dumps({key:device.get_info(info) if device.supports(info) else None for key,info in
            [('name',rs.camera_info.name),('serial',rs.camera_info.serial_number),('firmware',rs.camera_info.firmware_version),('usb',rs.camera_info.usb_type_descriptor)]},ensure_ascii=False))
    if args.list_devices:return
    choices=[d for d in devices if '455' in d.get_info(rs.camera_info.name) and
             (not args.serial or d.get_info(rs.camera_info.serial_number)==args.serial)]
    if len(choices)!=1:raise RuntimeError('D455를 한 대 선택하세요. 여러 대면 --serial로 지정하세요.')
    device=choices[0]
    usb=device.get_info(rs.camera_info.usb_type_descriptor) if device.supports(rs.camera_info.usb_type_descriptor) else ''
    use_depth=usb.startswith('3')
    if not use_depth:print('USB 3 연결이 아니므로 RGB만 사용합니다. 깊이 확인은 USB 3 포트/케이블로 연결하세요.',flush=True)
    config=rs.config();config.enable_device(device.get_info(rs.camera_info.serial_number))
    config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    if use_depth:config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    pipeline=rs.pipeline();started=False;log=None
    try:
        profile=pipeline.start(config);started=True
        intr=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        camera=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1]],dtype=np.float64)
        distortion=np.asarray(intr.coeffs,dtype=np.float64)
        pose_inputs(rs,intr,np.array([[intr.ppx,intr.ppy]]*4),camera,distortion)
        align=rs.align(rs.stream.color) if use_depth else None
        depth_scale=profile.get_device().first_depth_sensor().get_depth_scale() if use_depth else None
        if args.log:
            args.log.parent.mkdir(parents=True,exist_ok=True);log=args.log.open('a',encoding='utf-8')
        last_log=0.;count=0;start=time.monotonic()
        window='D455 ArUco - camera frame only'
        while not args.probe or time.monotonic()-start<5:
            frames=pipeline.wait_for_frames(3000)
            if align:frames=align.process(frames)
            color=frames.get_color_frame();depth=frames.get_depth_frame()
            if not color or use_depth and not depth:continue
            image=np.asanyarray(color.get_data()).copy()
            depth_image=np.asanyarray(depth.get_data()) if depth else None
            detections=detect_all(image);observations=[]
            if detections:
                for polygon,marker_id,dictionary in detections:
                    observation=dict(id=marker_id,dictionary=dictionary,corners_px=polygon.reshape(4,2).tolist(),
                        marker_size_mm=args.marker_mm,pose_valid=False,camera_xyz_m=None,
                        rotation_vector_rad=None,reprojection_px=None,depth_z_m=None,
                        coordinate_frame='camera_optical',robot_control=False)
                    observations.append(observation)
                    prepared,k,d,draw_axes=pose_inputs(rs,intr,polygon,camera,distortion)
                    result=estimate(prepared,args.marker_mm/1000,k,d)
                    if result is None:continue
                    error,rotation,translation=result;xyz=translation.ravel()
                    cx,cy=np.rint(polygon.reshape(4,2).mean(axis=0)).astype(int)
                    patch=depth_image[max(0,cy-2):min(depth_image.shape[0],cy+3),max(0,cx-2):min(depth_image.shape[1],cx+3)] if depth_image is not None else np.array([])
                    valid=patch[patch>0];depth_m=float(np.median(valid)*depth_scale) if valid.size else None
                    observation.update(pose_valid=True,camera_xyz_m=xyz.tolist(),rotation_vector_rad=rotation.ravel().tolist(),reprojection_px=error,depth_z_m=depth_m)
            count+=1;now=time.monotonic()
            if now-last_log>=.2:
                record=dict(timestamp=time.time(),detected=bool(observations),marker_count=len(observations),markers=observations)
                if log:log.write(json.dumps(record,allow_nan=False)+'\n');log.flush()
                last_log=now
            if args.probe:continue
            cv2.imshow(window,render_preview(image,observations))
            if cv2.waitKey(1)&0xFF in (27,ord('q')) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:break
        if args.probe:
            if count<2:raise RuntimeError('카메라 프레임 수신 부족')
            print(f'PASS: RGB {"+ aligned depth" if use_depth else "only"}, {count} frames / 5 seconds; distortion={intr.model}; markers in last frame={len(observations)}')
    finally:
        if log:log.close()
        if started:pipeline.stop()
        if not args.probe:cv2.destroyAllWindows()


def main():
    parser=argparse.ArgumentParser(description='D455 ArUco preview; camera coordinates, no robot control.')
    parser.add_argument('--marker-mm',type=float,default=100,help='Measured outer black-square side, in mm')
    parser.add_argument('--marker-id',type=int,default=0)
    parser.add_argument('--generate-marker',type=Path)
    parser.add_argument('--list-devices',action='store_true')
    parser.add_argument('--probe',action='store_true',help='Read frames for 5 seconds without opening a window')
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--serial')
    parser.add_argument('--log',type=Path)
    args=parser.parse_args()
    if not math.isfinite(args.marker_mm) or args.marker_mm<=0:parser.error('--marker-mm must be positive and finite')
    if not 0<=args.marker_id<50:parser.error('--marker-id must be 0..49')
    if args.self_test:self_test()
    elif args.generate_marker:generate(args.generate_marker,args.marker_mm,args.marker_id)
    else:run(args)


if __name__=='__main__':
    try:main()
    except (RuntimeError,cv2.error) as error:
        print('오류:',error,'\nRealSense Viewer 등 카메라를 사용하는 프로그램을 닫은 뒤 다시 실행하세요.',file=sys.stderr)
        sys.exit(1)
