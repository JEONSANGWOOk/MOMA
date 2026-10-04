"""Windows OpenGL smooth-shaded offscreen renderer, presented in a Tk canvas.
No compiler, ROS, native visible windows or robot commands are required.
"""
import ctypes as C
import math,sys
from collections import defaultdict
from .geometry3d import identity,multiply,point


def smooth_normals(faces,crease=math.cos(math.radians(48))):
    normals=[];adjacent=defaultdict(list)
    for index,face in enumerate(faces):
        a,b,c=face[:3];u=[b[i]-a[i] for i in range(3)];v=[c[i]-a[i] for i in range(3)]
        raw=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]);length=math.sqrt(sum(x*x for x in raw))
        normal=tuple(x/max(length,1e-12) for x in raw);normals.append((normal,max(length,1e-12)))
        for p in face:adjacent[tuple(round(x,6) for x in p)].append(index)
    result=[]
    for index,face in enumerate(faces):
        normal=normals[index][0];values=[]
        for p in face:
            chosen=[normals[j] for j in adjacent[tuple(round(x,6) for x in p)] if sum(a*b for a,b in zip(normal,normals[j][0]))>=crease]
            n=[sum(v[i]*weight for v,weight in chosen) for i in range(3)];length=math.sqrt(sum(x*x for x in n));values.append(tuple(x/max(length,1e-12) for x in n))
        result.append(values)
    return result


class SmoothRenderer:
    def __init__(self):
        if sys.platform!='win32':raise RuntimeError('Windows OpenGL renderer')
        from PIL import Image
        self.Image=Image;self.cache={};self.size=None;self.closed=False
        self.user=C.WinDLL('user32',use_last_error=True);self.gdi=C.WinDLL('gdi32',use_last_error=True);self.gl=C.WinDLL('opengl32',use_last_error=True)
        def api(dll,name,args,result=None):
            fn=getattr(dll,name);fn.argtypes=args;fn.restype=result;return fn
        H=C.c_void_p;I=C.c_int;U=C.c_uint;F=C.c_float;D=C.c_double
        self.create=api(self.user,'CreateWindowExW',[U,C.c_wchar_p,C.c_wchar_p,U,I,I,I,I,H,H,H,H],H)
        self.getdc=api(self.user,'GetDC',[H],H);self.releasedc=api(self.user,'ReleaseDC',[H,H],I)
        self.resize=api(self.user,'SetWindowPos',[H,H,I,I,I,I,U],I);self.destroy=api(self.user,'DestroyWindow',[H],I)
        self.window=self.create(0,'STATIC','AMR offscreen surface',0x80000000,0,0,64,64,None,None,None,None)
        if not self.window:raise RuntimeError('오프스크린 창 생성 실패')
        self.dc=self.getdc(self.window)
        class PFD(C.Structure):
            _fields_=[('nSize',C.c_ushort),('nVersion',C.c_ushort),('dwFlags',U),('iPixelType',C.c_ubyte),('cColorBits',C.c_ubyte),('cRedBits',C.c_ubyte),('cRedShift',C.c_ubyte),('cGreenBits',C.c_ubyte),('cGreenShift',C.c_ubyte),('cBlueBits',C.c_ubyte),('cBlueShift',C.c_ubyte),('cAlphaBits',C.c_ubyte),('cAlphaShift',C.c_ubyte),('cAccumBits',C.c_ubyte),('cAccumRedBits',C.c_ubyte),('cAccumGreenBits',C.c_ubyte),('cAccumBlueBits',C.c_ubyte),('cAccumAlphaBits',C.c_ubyte),('cDepthBits',C.c_ubyte),('cStencilBits',C.c_ubyte),('cAuxBuffers',C.c_ubyte),('iLayerType',C.c_ubyte),('bReserved',C.c_ubyte),('dwLayerMask',U),('dwVisibleMask',U),('dwDamageMask',U)]
        p=PFD(nSize=C.sizeof(PFD),nVersion=1,dwFlags=0x25,iPixelType=0,cColorBits=24,cDepthBits=24)
        choose=api(self.gdi,'ChoosePixelFormat',[H,C.POINTER(PFD)],I);setformat=api(self.gdi,'SetPixelFormat',[H,I,C.POINTER(PFD)],I)
        if not setformat(self.dc,choose(self.dc,C.byref(p)),C.byref(p)):raise RuntimeError('OpenGL 픽셀 형식 생성 실패')
        self.make_current=api(self.gl,'wglMakeCurrent',[H,H],I);self.delete_context=api(self.gl,'wglDeleteContext',[H],I)
        self.context=api(self.gl,'wglCreateContext',[H],H)(self.dc)
        if not self.context or not self.make_current(self.dc,self.context):raise RuntimeError('OpenGL context 생성 실패')
        specs={'glClearColor':([F,F,F,F],None),'glClear':([U],None),'glEnable':([U],None),'glDisable':([U],None),
         'glViewport':([I,I,I,I],None),'glMatrixMode':([U],None),'glLoadIdentity':([],None),'glFrustum':([D]*6,None),'glOrtho':([D]*6,None),
         'glLoadMatrixf':([C.POINTER(F)],None),'glMultMatrixf':([C.POINTER(F)],None),'glPushMatrix':([],None),'glPopMatrix':([],None),
         'glBegin':([U],None),'glEnd':([],None),'glVertex3f':([F,F,F],None),'glNormal3f':([F,F,F],None),'glColor4f':([F]*4,None),
         'glLightfv':([U,U,C.POINTER(F)],None),'glMaterialfv':([U,U,C.POINTER(F)],None),'glMaterialf':([U,U,F],None),'glColorMaterial':([U,U],None),
         'glShadeModel':([U],None),'glGenLists':([I],U),'glNewList':([U,U],None),'glEndList':([],None),'glCallList':([U],None),'glDeleteLists':([U,I],None),
         'glReadPixels':([I,I,I,I,U,U,H],None),'glReadBuffer':([U],None),'glPixelStorei':([U,I],None),'glFinish':([],None),
         'glBlendFunc':([U,U],None),'glDepthMask':([C.c_ubyte],None),'glLineWidth':([F],None),'glGetString':([U],C.c_char_p)}
        for name,(args,result) in specs.items():setattr(self,name,api(self.gl,name,args,result))
        self.backend=self.glGetString(0x1F01).decode(errors='replace')
        self.glEnable(0x0B71);self.glEnable(0x0BA1);self.glShadeModel(0x1D01);self.glEnable(0x0B50);self.glEnable(0x4000)
        self.glColorMaterial(0x0408,0x1602);self.glEnable(0x0B57)
        self.glLightfv(0x4000,0x1200,(F*4)(.24,.24,.24,1));self.glLightfv(0x4000,0x1201,(F*4)(.85,.85,.85,1))
        self.glMaterialfv(0x0408,0x1202,(F*4)(.25,.25,.25,1));self.glMaterialf(0x0408,0x1601,48.)
        self.glPixelStorei(0x0D05,1)
    def matrix(self,m):return (C.c_float*16)(*(m[i][j] for j in range(4) for i in range(4)))
    def asset(self,asset):
        key=id(asset)
        if key in self.cache:return self.cache[key][1]
        rows=[]
        for name,visuals in asset.links.items():
            for index,(faces,local,color) in enumerate(visuals):
                # Full source geometry is retained separately from the Tk fallback LOD.
                full=getattr(asset,'full_visuals',{}).get((name,index),faces)
                normals=smooth_normals(full);display=self.glGenLists(1);self.glNewList(display,0x1300);self.glBegin(0x0004)
                for face,values in zip(full,normals):
                    for j in range(1,len(face)-1):
                        for i in (0,j,j+1):self.glNormal3f(*values[i]);self.glVertex3f(*face[i])
                self.glEnd();self.glEndList();rows.append((name,local,color,display))
        self.cache[key]=(asset,rows);return rows
    def render(self,camera,width,height,assets=(),faces=(),lines=(),grid=False):
        if not self.make_current(self.dc,self.context):raise RuntimeError('OpenGL context 전환 실패')
        factor=2;w=max(16,int(width)*factor);h=max(16,int(height)*factor)
        if self.size!=(w,h):self.resize(self.window,None,0,0,w,h,0x0014);self.size=(w,h)
        self.glViewport(0,0,w,h);self.glClearColor(.929,.949,.973,1);self.glClear(0x00004000|0x00000100)
        self.glMatrixMode(0x1701);self.glLoadIdentity();small=min(w,h)
        if camera.perspective:
            near=.02;self.glFrustum(-w/(2*small*.95)*near,w/(2*small*.95)*near,-h/(2*small*.95)*near,h/(2*small*.95)*near,near,20000.)
        else:self.glOrtho(-w*camera.distance/(small*3.2),w*camera.distance/(small*3.2),-h*camera.distance/(small*3.2),h*camera.distance/(small*3.2),.02,20000.)
        eye,right,up,forward=camera.basis();view=tuple(tuple((*right,-sum(a*b for a,b in zip(right,eye))) if i==0 else (*up,-sum(a*b for a,b in zip(up,eye))) if i==1 else (*(-v for v in forward),sum(a*b for a,b in zip(forward,eye))) if i==2 else (0,0,0,1)) for i in range(4))
        self.glMatrixMode(0x1700);self.glLoadMatrixf(self.matrix(view));self.glLightfv(0x4000,0x1203,(C.c_float*4)(-2,-3,5,0));self.glEnable(0x0B50)
        for vertices,color,tag in faces:
            self.glColor4f(*(int(color[i:i+2],16)/255 for i in (1,3,5)),1)
            normals=smooth_normals([vertices]);self.glBegin(0x0004)
            for j in range(1,len(vertices)-1):
                for i in (0,j,j+1):self.glNormal3f(*normals[0][i]);self.glVertex3f(*vertices[i])
            self.glEnd()
        opaque=[row for row in assets if row[4]>=1];transparent=[row for row in assets if row[4]<1]
        for asset,positions,world,tint,alpha in opaque+transparent:
            if alpha<1:self.glEnable(0x0BE2);self.glBlendFunc(0x0302,0x0303);self.glDepthMask(0)
            else:self.glDisable(0x0BE2);self.glDepthMask(1)
            poses=asset.link_transforms(positions,world)
            for name,local,color,display in self.asset(asset):
                actual=tint or color;self.glColor4f(*(int(actual[i:i+2],16)/255 for i in (1,3,5)),alpha)
                self.glPushMatrix();self.glMultMatrixf(self.matrix(multiply(poses[name],local)));self.glCallList(display);self.glPopMatrix()
        self.glDepthMask(1);self.glDisable(0x0BE2);self.glDisable(0x0B50)
        if grid:
            for i in range(-10,11):
                v=i*.1;lines=(*lines,((v,-1,0),(v,1,0),'#cbd7e4',1),((-1,v,0),(1,v,0),'#cbd7e4',1))
        for a,b,color,line_width in lines:
            self.glColor4f(*(int(color[i:i+2],16)/255 for i in (1,3,5)),1);self.glLineWidth(line_width*factor);self.glBegin(1);self.glVertex3f(*a);self.glVertex3f(*b);self.glEnd()
        self.glFinish();self.glReadBuffer(0x0405);data=C.create_string_buffer(w*h*3);self.glReadPixels(0,0,w,h,0x1907,0x1401,data)
        img=self.Image.frombytes('RGB',(w,h),data.raw).transpose(self.Image.Transpose.FLIP_TOP_BOTTOM)
        return img.resize((width,height),self.Image.Resampling.LANCZOS)
    def close(self):
        if self.closed:return
        self.closed=True;self.make_current(self.dc,self.context)
        for asset,rows in self.cache.values():
            for row in rows:self.glDeleteLists(row[3],1)
        self.make_current(None,None);self.delete_context(self.context);self.releasedc(self.window,self.dc);self.destroy(self.window)


def renderer_for(canvas):
    if hasattr(canvas,'smooth_renderer'):return canvas.smooth_renderer
    try:
        canvas.smooth_renderer=SmoothRenderer()
        canvas.bind('<Destroy>',lambda e:canvas.smooth_renderer.close() if e.widget==canvas and canvas.smooth_renderer else None,add='+')
    except Exception as error:
        canvas.smooth_renderer=None;canvas.smooth_error=str(error)
    return canvas.smooth_renderer
