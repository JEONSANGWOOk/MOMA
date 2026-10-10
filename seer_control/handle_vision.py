"""Marker-conditioned, taught handle appearance matching. No robot transport."""
import base64,json,math
from pathlib import Path
import cv2
import numpy as np


def camera_parameters(camera):
    k=np.asarray(camera['matrix'],dtype=float);d=np.asarray(camera['distortion'],dtype=float)
    if k.shape!=(3,3) or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0:raise ValueError('카메라 보정값 오류')
    if camera.get('model') not in ('none','brown_conrady','inverse_brown_conrady'):raise ValueError('구멍 검출용 RGB 왜곡 모델 지원 필요')
    return k,d


def rs_intrinsics(camera):
    import pyrealsense2 as rs
    k,d=camera_parameters(camera);intr=rs.intrinsics()
    intr.width=camera.get('width',640);intr.height=camera.get('height',480)
    intr.fx=float(k[0,0]);intr.fy=float(k[1,1]);intr.ppx=float(k[0,2]);intr.ppy=float(k[1,2]);intr.coeffs=d.tolist()
    intr.model=rs.distortion.inverse_brown_conrady
    return rs,intr


def plane_point(pixel,board,camera,z_mm):
    k,d=camera_parameters(camera);r=cv2.Rodrigues(np.asarray(board['rotation_vector_rad'],dtype=float))[0]
    t=np.asarray(board['camera_xyz_m'],dtype=float)
    if camera.get('model')=='inverse_brown_conrady':
        rs,intr=rs_intrinsics(camera);ray=np.asarray(rs.rs2_deproject_pixel_to_point(intr,list(map(float,pixel)),1.))
    else:ray=np.r_[cv2.undistortPoints(np.asarray(pixel,dtype=float).reshape(1,1,2),k,d).reshape(2),1.]
    n=r[:,2];den=float(n@ray)
    if abs(den)<.15:raise ValueError('마커 표면이 너무 비스듬합니다.')
    distance=float(n@(t+n*z_mm/1000))/den
    if distance<=0:raise ValueError('구멍이 카메라 뒤에 있습니다.')
    return ((r.T@(ray*distance-t))*1000).tolist()


def project(points,board,camera):
    k,d=camera_parameters(camera)
    if camera.get('model')=='inverse_brown_conrady':
        rs,intr=rs_intrinsics(camera);r=cv2.Rodrigues(np.asarray(board['rotation_vector_rad'],dtype=float))[0]
        xyz=np.asarray(points,dtype=float)/1000@r.T+np.asarray(board['camera_xyz_m'],dtype=float)
        if np.any(xyz[:,2]<=0):raise ValueError('구멍 표면이 카메라 뒤에 있습니다.')
        return np.asarray([rs.rs2_project_point_to_pixel(intr,p.tolist()) for p in xyz])
    return cv2.projectPoints(np.asarray(points,dtype=float)/1000,np.asarray(board['rotation_vector_rad'],dtype=float),
                            np.asarray(board['camera_xyz_m'],dtype=float),k,d)[0].reshape(-1,2)


def patch(image,board,camera,center,z_mm,size):
    half=(size-1)/2;x,y=center
    corners=project([[x-half,y-half,z_mm],[x+half,y-half,z_mm],[x+half,y+half,z_mm],[x-half,y+half,z_mm]],board,camera)
    target=np.asarray([[0,0],[size-1,0],[size-1,size-1],[0,size-1]],dtype=np.float32)
    h=cv2.getPerspectiveTransform(corners.astype(np.float32),target)
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
    result=cv2.warpPerspective(gray,h,(size,size))
    mask=cv2.warpPerspective(np.full(gray.shape,255,np.uint8),h,(size,size),flags=cv2.INTER_NEAREST)
    return result,mask


class HandleDetector:
    def __init__(self,path):self.path=Path(path);self.signature=None;self.profile=None;self.template=None;self.local=None;self.error='손잡이·구멍 등록 필요';self.count=0;self.previous=None
    def reload(self):
        try:
            signature=self.path.stat().st_mtime_ns
            if signature==self.signature:return
            p=json.loads(self.path.read_text(encoding='utf-8'))
            if p.get('version')!=1 or not p.get('revision') or not math.isfinite(p['surface_z_mm']) or not 0<=p['surface_z_mm']<=100:raise ValueError('손잡이 등록 형식 오류')
            image=cv2.imdecode(np.frombuffer(base64.b64decode(p['image_jpeg_base64'],validate=True),np.uint8),cv2.IMREAD_COLOR)
            if image is None:raise ValueError('등록 영상 오류')
            local=plane_point(p['center_px'],p['board'],p['camera'],p['surface_z_mm'])
            template,mask=patch(image,p['board'],p['camera'],local[:2],p['surface_z_mm'],61)
            if np.mean(mask>0)<.98 or float(template.std())<8:raise ValueError('구멍 주변 무늬/영상 범위 부족 · 다시 등록하세요.')
            self.profile=p;self.template=template;self.local=local;self.signature=signature;self.count=0;self.previous=None;self.error=''
        except (OSError,ValueError,KeyError,TypeError,cv2.error) as exc:
            self.profile=None;self.template=None;self.count=0;self.previous=None;self.error=str(exc) if self.path.exists() else '손잡이·구멍 등록 필요'
    def detect(self,image,record,camera):
        self.reload();result=dict(valid=False,reason=self.error,method='marker_guided_taught_appearance',dimension_source='estimated_plane')
        p=self.profile;b=record.get('raw_board') or record.get('board') or {}
        if p is None:return result
        result.update(profile_revision=p['revision'],board_revision=p['board']['revision'],dimension_source=p.get('dimension_source','estimated_plane'))
        try:
            if not b.get('valid') or b.get('markers_used')!=2 or b.get('revision')!=p['board']['revision']:raise ValueError('등록한 두 마커 필요')
            search,mask=patch(image,b,camera,self.local[:2],p['surface_z_mm'],91)
            response=cv2.matchTemplate(search,self.template,cv2.TM_CCOEFF_NORMED)
            _,score,_,xy=cv2.minMaxLoc(response)
            others=response.copy();x,y=xy;others[max(0,y-6):y+7,max(0,x-6):x+7]=-1
            margin=score-float(others.max());coverage=float(np.mean(mask[y:y+61,x:x+61]>0))
            result.update(confidence=float(score),uniqueness_margin=margin)
            if not math.isfinite(score) or score<.78 or margin<.035 or coverage<.98:raise ValueError('구멍 가림/외형 불일치/후보 중복 · 삽입 보류')
            def subpixel(a,b,c):
                den=a-2*b+c
                return max(-.5,min(.5,.5*(a-c)/den)) if abs(den)>1e-8 else 0.
            dx=subpixel(float(response[y,x-1]),float(response[y,x]),float(response[y,x+1])) if 0<x<response.shape[1]-1 else 0.
            dy=subpixel(float(response[y-1,x]),float(response[y,x]),float(response[y+1,x])) if 0<y<response.shape[0]-1 else 0.
            local=[self.local[0]+x+dx-15,self.local[1]+y+dy-15,p['surface_z_mm']]
            if math.dist(local[:2],self.local[:2])>10:raise ValueError('구멍 보정 10 mm 범위 초과')
            stamp=record['timestamp']
            if not self.previous or self.previous[0]!=stamp:
                self.count=self.count+1 if self.previous and math.dist(local[:2],self.previous[1][:2])<=2 else 1
                self.previous=(stamp,local)
            result.update(valid=self.count>=3,reason='' if self.count>=3 else '구멍 위치 3프레임 확인 중',
                board_xyz_mm=local,registered_board_xyz_mm=self.local,center_px=project([local],b,camera)[0].tolist(),confirmation_frames=self.count)
        except (ValueError,KeyError,TypeError,cv2.error) as exc:self.count=0;self.previous=None;result['reason']=str(exc)
        return result
