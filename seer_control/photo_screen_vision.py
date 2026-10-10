"""Locate a taught catalog picture and its labelled hole; inferred pose is SIM only."""
import json,math
from pathlib import Path
import cv2
import numpy as np
from .handle_vision import camera_parameters,rs_intrinsics


class PhotoScreenDetector:
    def __init__(self,path):
        self.path=Path(path);self.signature=None;self.profile=None;self.count=0;self.stamp=None
        self.orb=cv2.ORB_create(nfeatures=1200,scaleFactor=1.15,nlevels=10,fastThreshold=8)
        self.matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
    def load(self):
        signature=self.path.stat().st_mtime_ns
        if signature==self.signature:return
        p=json.loads(self.path.read_text(encoding='utf-8'));ref=cv2.imdecode(np.frombuffer(Path(p['reference_file']).read_bytes(),np.uint8),cv2.IMREAD_GRAYSCALE)
        if ref is None or p.get('version')!=1 or not .5<=p['reference_px_per_mm']<=10:raise ValueError('사진 등록 오류')
        self.keypoints,self.descriptors=self.orb.detectAndCompute(ref,None)
        if self.descriptors is None or len(self.keypoints)<20:raise ValueError('사진 특징점 부족')
        self.profile=p;self.shape=ref.shape;self.signature=signature;self.count=0;self.stamp=None
    def detect(self,image,camera,stamp):
        out=dict(valid=False,reason='사진 기준 등록 필요',vision_source='photo_screen_sim',geometry_source='photo_screen_estimate',markers_used=0)
        try:
            self.load();p=self.profile;out['revision']=p['revision']
            gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
            keys,desc=self.orb.detectAndCompute(gray,None)
            if desc is None:raise ValueError('화면 사진 미검출')
            pairs=self.matcher.knnMatch(self.descriptors,desc,k=2)
            good=[a for pair in pairs if len(pair)==2 for a,b in [pair] if a.distance<.72*b.distance]
            if len(good)<12:raise ValueError('사진 대응 특징점 부족')
            a=np.float32([self.keypoints[m.queryIdx].pt for m in good]);b=np.float32([keys[m.trainIdx].pt for m in good])
            h,mask=cv2.findHomography(a,b,cv2.RANSAC,2.0)
            if h is None or mask is None:raise ValueError('사진 위치 추정 실패')
            keep=mask.ravel().astype(bool);n=int(keep.sum());ratio=n/len(good)
            if n<12 or ratio<.6:raise ValueError('사진 특징점 일치 부족')
            height,width=self.shape
            quad=cv2.perspectiveTransform(np.float32([[[0,0],[width-1,0],[width-1,height-1],[0,height-1]]]),h)[0]
            if not np.isfinite(quad).all() or not cv2.isContourConvex(quad) or cv2.contourArea(quad)<7000:raise ValueError('사진 크기/외곽 오류 · 더 크게 보여주세요.')
            center=cv2.perspectiveTransform(np.float32([[p['hole_center_px']]]),h)[0,0]
            if not 0<=center[0]<image.shape[1] or not 0<=center[1]<image.shape[0]:raise ValueError('사진 구멍이 화면 밖에 있습니다.')
            # At least three image regions must support the homography: a small
            # matching fragment alone cannot supply a whole screen pose.
            if np.ptp(a[keep,0])<width*.35 or np.ptp(a[keep,1])<height*.35:raise ValueError('사진 전체를 보여주세요.')
            cx,cy=p['hole_center_px'];scale=p['reference_px_per_mm']
            points=np.column_stack(((a[keep,0]-cx)/scale,-(a[keep,1]-cy)/scale,np.zeros(n))).astype(float)/1000
            k,d=camera_parameters(camera);pixels=b[keep].astype(float)
            if camera.get('model')=='inverse_brown_conrady':
                rs,intr=rs_intrinsics(camera);rays=np.asarray([rs.rs2_deproject_pixel_to_point(intr,q.tolist(),1.) for q in pixels])
                pixels=rays[:,:2]/rays[:,2,None]*np.array([k[0,0],k[1,1]])+np.array([k[0,2],k[1,2]]);d=np.zeros(5)
            ok,r,t=cv2.solvePnP(points,pixels,k,d,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or not np.isfinite(t).all() or t[2,0]<=0:raise ValueError('사진 자세 추정 실패')
            projected=cv2.projectPoints(points,r,t,k,d)[0].reshape(-1,2)
            error=float(np.sqrt(np.mean(np.sum((projected-pixels)**2,axis=1))))
            if error>2:raise ValueError('사진 재투영 오차 초과')
            if stamp!=self.stamp:self.count+=1;self.stamp=stamp
            out.update(valid=self.count>=3,reason='' if self.count>=3 else '사진 위치 3프레임 확인 중',center_px=center.tolist(),outline_px=quad.tolist(),
                camera_xyz_m=t.ravel().tolist(),rotation_vector_rad=r.ravel().tolist(),reprojection_px=error,
                inliers=n,match_ratio=ratio,confirmation_frames=self.count,scale_source='assumed_screen_size')
        except (OSError,ValueError,KeyError,TypeError,cv2.error) as exc:self.count=0;self.stamp=None;out['reason']=str(exc) if self.path.exists() else '사진 기준 등록 필요'
        return out
