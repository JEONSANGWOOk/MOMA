"""Photo-based display materials; does not modify the upstream URDF or FK."""
import math
from collections import Counter
from .geometry3d import transform

def ring(outer,inner,height,steps=96):
 result=[]
 for i in range(steps):
  a=i*2*math.pi/steps;b=(i+1)*2*math.pi/steps
  for z in (-height/2,height/2):result.append(((inner*math.cos(a),inner*math.sin(a),z),(outer*math.cos(a),outer*math.sin(a),z),(outer*math.cos(b),outer*math.sin(b),z),(inner*math.cos(b),inner*math.sin(b),z)))
  for radius in (inner,outer):result.append(((radius*math.cos(a),radius*math.sin(a),-height/2),(radius*math.cos(b),radius*math.sin(b),-height/2),(radius*math.cos(b),radius*math.sin(b),height/2),(radius*math.cos(a),radius*math.sin(a),height/2)))
 return result

def style_fr5(asset):
 asset.photo_reference='https://www.fairino.com/FR/4.html'
 for name in asset.links:
  color='#b1b5ba' if name=='wrist3_link' else '#f0f1f3'
  asset.links[name]=[(faces,local,color) for faces,local,_ in asset.links[name]]
 # Joint-ring locations are display estimates from the source mesh geometry.
 for name,radius in [('base_link',.07),('shoulder_link',.067),('upperarm_link',.067),('forearm_link',.052),('wrist1_link',.041),('wrist2_link',.041)]:
  if name not in asset.links:continue
  points=[p for i,(faces,_,_) in enumerate(asset.links[name]) for face in asset.full_visuals.get((name,i),faces) for p in face if .65*radius<math.hypot(p[0],p[1])<1.05*radius]
  if not points:continue
  planes=Counter(round(p[2],4) for p in points);z=planes.most_common(1)[0][0]
  faces=ring(radius*.99,radius*.86,.0018)
  asset.links[name].append((faces,transform((0,0,z+.0015)),'#ee5b4d'))
