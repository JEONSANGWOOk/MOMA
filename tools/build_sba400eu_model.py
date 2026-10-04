"""Generate our photo/spec-based SBA-400EU ROS2 model. No factory CAD claim."""
import json,math,struct,sys,zipfile
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from seer_control.geometry3d import box,cylinder,transform,point
from seer_control.visual_materials import ring
OUT=ROOT/'models/seer_sba400eu_description'

def rounded_profile(hx,hy,r,steps=16):
 pts=[]
 for cx,cy,start in [(hx-r,hy-r,0),(-hx+r,hy-r,90),(-hx+r,-hy+r,180),(hx-r,-hy+r,270)]:
  for i in range(steps+1):
   a=math.radians(start+i*90/steps);pts.append((cx+r*math.cos(a),cy+r*math.sin(a)))
 return pts

def shell():
 lower=rounded_profile(.4791,.3157,.065);upper=rounded_profile(.4591,.2957,.053);faces=[];lo=.023;bevel=.154;hi=.18
 for i,a in enumerate(lower):
  b=lower[(i+1)%len(lower)];u=upper[i];v=upper[(i+1)%len(upper)]
  if abs(a[1]-b[1])<1e-8 and abs(abs(a[1])-.3157)<1e-6:
   # The wheel notch is a real open edge in this mesh, not a painted black patch.
   count=80
   for j in range(count):
    x=a[0]+(b[0]-a[0])*j/count;xx=a[0]+(b[0]-a[0])*(j+1)/count
    za=max(lo,.083+math.sqrt(max(0,.093**2-x*x))) if abs(x)<.093 else lo
    zb=max(lo,.083+math.sqrt(max(0,.093**2-xx*xx))) if abs(xx)<.093 else lo
    if min(za,zb)<bevel:faces.append(((x,a[1],min(za,bevel)),(xx,b[1],min(zb,bevel)),(xx,b[1],bevel),(x,a[1],bevel)))
  else:faces.append(((a[0],a[1],lo),(b[0],b[1],lo),(b[0],b[1],bevel),(a[0],a[1],bevel)))
  faces.append(((a[0],a[1],bevel),(b[0],b[1],bevel),(v[0],v[1],hi),(u[0],u[1],hi)))
  faces.append(((0,0,hi),(u[0],u[1],hi),(v[0],v[1],hi)))
 return faces

def plate(hx,hy,r,z,thickness):
 pts=rounded_profile(hx,hy,r);faces=[]
 for i,a in enumerate(pts):
  b=pts[(i+1)%len(pts)];faces.extend([((0,0,z+thickness),(a[0],a[1],z+thickness),(b[0],b[1],z+thickness)),((a[0],a[1],z),(b[0],b[1],z),(b[0],b[1],z+thickness),(a[0],a[1],z+thickness))])
 return faces

def stl(path,faces):
 tris=[(face[0],face[i],face[i+1]) for face in faces for i in range(1,len(face)-1)];data=bytearray(b'SBA-400EU photo/spec based visual'.ljust(80,b'\0'));data.extend(struct.pack('<I',len(tris)))
 for tri in tris:data.extend(struct.pack('<12fH',0,0,0,*(v for p in tri for v in p),0))
 path.write_bytes(data)

def obj(path,faces):
 lines=[];index=0
 for face in faces:
  lines.extend('v '+' '.join(f'{v:.8f}' for v in p) for p in face);lines.append('f '+' '.join(str(index+i+1) for i in range(len(face))));index+=len(face)
 path.write_text('\n'.join(lines)+'\n',encoding='utf-8')

def main():
 (OUT/'meshes').mkdir(parents=True,exist_ok=True);(OUT/'urdf').mkdir(exist_ok=True);(OUT/'config').mkdir(exist_ok=True)
 root=ET.Element('robot',name='SEER SBA-400EU');materials={'silver':'.43 .46 .48 1','black':'.06 .07 .075 1','blue':'.015 .16 .62 1','teal':'0 .85 .94 1','red':'.92 .04 .04 1','yellow':'.95 .76 .04 1','white':'.95 .97 .98 1'}
 for name,color in materials.items():ET.SubElement(ET.SubElement(root,'material',name=name),'color',rgba=color)
 ET.SubElement(root,'link',name='base_footprint');base=ET.SubElement(root,'link',name='base_link')
 inertial=ET.SubElement(base,'inertial');ET.SubElement(inertial,'origin',xyz='0 0 .091');ET.SubElement(inertial,'mass',value='100');ET.SubElement(inertial,'inertia',ixx=str(100*(.6314**2+.182**2)/12),iyy=str(100*(.9582**2+.182**2)/12),izz=str(100*(.9582**2+.6314**2)/12),ixy='0',ixz='0',iyz='0')
 def fixed(name,parent,child,xyz=(0,0,0),rpy=(0,0,0),kind='fixed'):
  j=ET.SubElement(root,'joint',name=name,type=kind);ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=child);ET.SubElement(j,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)))
  if kind!='fixed':ET.SubElement(j,'axis',xyz='0 0 1')
  return j
 fixed('footprint_to_base','base_footprint','base_link')
 def mesh(link,name,faces,color):
  stl(OUT/'meshes'/f'{name}.stl',faces);obj(OUT/'meshes'/f'{name}.obj',faces)
  vis=ET.SubElement(link,'visual');ET.SubElement(ET.SubElement(vis,'geometry'),'mesh',filename=f'package://seer_sba400eu_description/meshes/{name}.stl');ET.SubElement(vis,'material',name=color)
 def primitive(link,shape,params,color,xyz=(0,0,0),rpy=(0,0,0)):
  vis=ET.SubElement(link,'visual');ET.SubElement(vis,'origin',xyz=' '.join(map(str,xyz)),rpy=' '.join(map(str,rpy)));ET.SubElement(ET.SubElement(vis,'geometry'),shape,**{k:str(v) for k,v in params.items()});ET.SubElement(vis,'material',name=color)
 mesh(base,'chassis',shell(),'silver');mesh(base,'top_plate',plate(.449,.285,.05,.18,.002),'black')
 # Separate lower perimeter trim segments avoid covering the central drive wheels.
 for side in (-1,1):
  for x in (-.295,.295):primitive(base,'box',{'size':'.35 .012 .019'},'black',(x,side*.307,.025))
 for x in (-.471,.471):primitive(base,'box',{'size':'.01 .47 .019'},'black',(x,0,.025))
 for side,name in [(1,'left'),(-1,'right')]:
  wheel=ET.SubElement(root,'link',name=f'drive_{name}_wheel');fixed(f'drive_{name}_joint','base_link',f'drive_{name}_wheel',(0,side*.2932,.081),(math.pi/2,0,0),'continuous')
  primitive(wheel,'cylinder',{'radius':'.081','length':'.045'},'black')
  mesh(wheel,f'drive_{name}_blue_ring',[tuple((x,y,z+offset) for x,y,z in face) for offset in (-.024,.024) for face in ring(.075,.060,.003)],'blue')
  for z in (-.023,.023):primitive(wheel,'cylinder',{'radius':'.052','length':'.002'},'black',(0,0,z))
  # Blue SEER panel and white block-letter mark on the side, photograph-based placement.
  panel=ET.SubElement(root,'link',name=f'{name}_logo_panel');fixed(f'{name}_logo_mount','base_link',f'{name}_logo_panel',(-.27,side*.306,.12),(math.pi/2,0,0))
  primitive(panel,'box',{'size':'.17 .045 .004'},'blue')
  strokes={'S':[(0,1,1,0),(0,1,0,.5),(0,.5,1,.5),(1,.5,1,0),(0,0,1,0)],'E':[(0,0,0,1),(0,1,1,1),(0,.5,.8,.5),(0,0,1,0)],'R':[(0,0,0,1),(0,1,1,1),(1,1,1,.5),(0,.5,1,.5),(.5,.5,1,0)]}
  for k,letter in enumerate('SEER'):
   for x1,y1,x2,y2 in strokes[letter]:
    dx=(x2-x1)*.021;dy=(y2-y1)*.025;length=math.hypot(dx,dy);angle=math.atan2(dy,dx)
    primitive(panel,'box',{'size':f'{length} .0025 .001'},'white',(-.051+k*.032+(x1+x2)*.0105,(y1+y2)*.0125-.0125,.0026),(0,0,angle))
 for x in (-.35,.35):
  for y in (-.22,.22):
   caster=ET.SubElement(root,'link',name=f'caster_{"f" if x>0 else "r"}_{"l" if y>0 else "r"}');fixed('mount_'+caster.get('name'),'base_link',caster.get('name'),(x,y,.028),(math.pi/2,0,0))
   primitive(caster,'cylinder',{'radius':'.028','length':'.035'},'black')
 # Bent turquoise LED strips wrapping the four chamfered corners.
 for x in (-1,1):
  for y in (-1,1):
   primitive(base,'box',{'size':'.055 .008 .013'},'teal',(x*.420,y*.301,.162))
   primitive(base,'box',{'size':'.009 .045 .037'},'teal',(x*.45,y*.270,.145))
   primitive(base,'box',{'size':'.035 .008 .014'},'teal',(x*.443,y*.290,.159),(0,0,-x*y*.6))
 # Front/rear inset sensor panel, camera lens pair, E-stop and side E-stops.
 for x in (-1,1):
  primitive(base,'box',{'size':'.007 .29 .064'},'black',(x*.477,0,.092))
  for y in (-.045,.045):primitive(base,'cylinder',{'radius':'.012','length':'.009'},'black',(x*.478,y,.092),(0,math.pi/2,0))
  primitive(base,'cylinder',{'radius':'.018','length':'.005'},'yellow',(x*.475,-.19,.1),(0,math.pi/2,0));primitive(base,'cylinder',{'radius':'.012','length':'.014'},'red',(x*.475,-.19,.1),(0,math.pi/2,0))
 for side in (-1,1):
  primitive(base,'cylinder',{'radius':'.020','length':'.002'},'yellow',(.25,side*.309,.085),(math.pi/2,0,0));primitive(base,'cylinder',{'radius':'.014','length':'.011'},'red',(.25,side*.309,.085),(math.pi/2,0,0))
 mounts=[(.36,-.235,.1965,0,0,-math.pi/4),(-.36,.235,.1965,0,0,3*math.pi/4)]
 for i,p in enumerate(mounts,1):
  laser=ET.SubElement(root,'link',name=f'lidar_{i}_link');fixed(f'lidar_{i}_mount','base_link',f'lidar_{i}_link',p[:3],p[3:])
  primitive(laser,'box',{'size':'.072 .072 .033'},'black',(0,0,.001))
  primitive(laser,'cylinder',{'radius':'.037','length':'.031'},'black',(0,0,.028))
  primitive(laser,'box',{'size':'.060 .004 .009'},'yellow',(.002,-.036,-.007))
 arm=ET.SubElement(root,'link',name='arm_mount_link');fixed('arm_mount_joint','base_link','arm_mount_link',(0,0,.182))
 ET.indent(root);(OUT/'urdf/sba400eu.urdf').write_text(ET.tostring(root,encoding='unicode'),encoding='utf-8')
 collision=ET.SubElement(base,'collision');ET.SubElement(collision,'origin',xyz='0 0 .101');ET.SubElement(ET.SubElement(collision,'geometry'),'box',size='.9582 .6314 .162')
 ET.indent(root);(OUT/'urdf/sba400eu.urdf').write_text(ET.tostring(root,encoding='unicode'),encoding='utf-8')
 meta=dict(model='SBA-400EU',source='https://seer-robotics.ai/amr/liftingrobot/SBA-400EU',dimensions_m=[.9582,.6314,.182],rotation_diameter_m=1.004,mass_kg=100,ground_clearance_m=.02,lidar_scan_height_m=.1965,lidar='2 (SICK HV)',extrinsics=[dict(link=f'lidar_{i+1}_link',xyz_rpy=list(p),verified=False,basis='photo estimate for x/y/yaw; published scan height for z') for i,p in enumerate(mounts)],arm_mount_xyz_rpy=[0,0,.182,0,0,0],factory_cad=False)
 (OUT/'config/model.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
 (OUT/'package.xml').write_text('<package format="3"><name>seer_sba400eu_description</name><version>1.0.0</version><description>Photo and specification based SBA-400EU visual model; estimated mounts</description><maintainer email="model@example.invalid">MOMA</maintainer><license>MIT</license><buildtool_depend>ament_cmake</buildtool_depend><exec_depend>robot_state_publisher</exec_depend><exec_depend>joint_state_publisher_gui</exec_depend><export><build_type>ament_cmake</build_type></export></package>',encoding='utf-8')
 (OUT/'CMakeLists.txt').write_text('cmake_minimum_required(VERSION 3.8)\nproject(seer_sba400eu_description)\nfind_package(ament_cmake REQUIRED)\ninstall(DIRECTORY urdf meshes config DESTINATION share/${PROJECT_NAME})\nament_package()\n',encoding='utf-8')
 (OUT/'README.md').write_text('SBA-400EU visual reconstruction based on official photographs and published dimensions. This is not factory CAD. Sensor x/y/yaw, wheel sizes, mounting holes and cosmetic details are estimates. Calibrate extrinsics before hardware use. Chassis is 958.2 × 631.4 × 182 mm excluding lasers. Coordinate frame: x forward, y left, z up. Original generated STL/OBJ; colors in URDF.\n',encoding='utf-8')
 (OUT/'launch').mkdir(exist_ok=True)
 (OUT/'launch/display.launch.py').write_text('''from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    root=Path(get_package_share_directory("seer_sba400eu_description"))
    description=(root/"urdf/sba400eu.urdf").read_text()
    return LaunchDescription([
        Node(package="robot_state_publisher",executable="robot_state_publisher",parameters=[{"robot_description":description}]),
        Node(package="joint_state_publisher_gui",executable="joint_state_publisher_gui"),
        Node(package="rviz2",executable="rviz2")])
''',encoding='utf-8')
 cmake=OUT/'CMakeLists.txt';cmake.write_text(cmake.read_text().replace('urdf meshes config','urdf meshes config launch'))
 package=OUT/'package.xml';package.write_text(package.read_text().replace('<export>','<exec_depend>rviz2</exec_depend><exec_depend>launch_ros</exec_depend><exec_depend>ament_index_python</exec_depend><export>'))
 with (OUT/'README.md').open('a',encoding='utf-8') as f:f.write('\nROS2: put this package under workspace/src, run colcon build and source install/setup.bash. Then ros2 launch seer_sba400eu_description display.launch.py. Set RViz Fixed Frame to base_footprint and add RobotModel using /robot_description. Windows app imports urdf/sba400eu.urdf directly, without ROS.\n')
 (OUT/'LICENSE').write_text('MIT License\nCopyright (c) 2026 MOMA model contributors\n\nPermission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to inclusion of this notice. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND. This license covers this original generated model, not vendor trademarks or source photographs.\n',encoding='utf-8')
 target=ROOT.parent/'release';target.mkdir(exist_ok=True);archive=target/'SEER_SBA400EU_ROS2_Description.zip'
 with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
  for path in OUT.rglob('*'):
   if path.is_file():z.write(path,path.relative_to(OUT.parent).as_posix())
 print(OUT/'urdf/sba400eu.urdf');print(archive)
if __name__=='__main__':main()
