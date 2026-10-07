"""Windows HID joystick input through WinMM; no optional dependencies."""
import ctypes
import math
import sys
import time
from dataclasses import dataclass

AXES=('X','Y','Z','R','U','V')


class JoyInfo(ctypes.Structure):
    _fields_=[(name,ctypes.c_uint32) for name in ('size','flags','X','Y','Z','R','U','V','buttons','button_number','pov','reserved1','reserved2')]


class JoyCaps(ctypes.Structure):
    _fields_=[('manufacturer',ctypes.c_uint16),('product',ctypes.c_uint16),('name',ctypes.c_wchar*32)]+[(name,ctypes.c_uint32) for name in ('Xmin','Xmax','Ymin','Ymax','Zmin','Zmax','buttons','period_min','period_max','Rmin','Rmax','Umin','Umax','Vmin','Vmax','caps','max_axes','num_axes','max_buttons')]+[('registry',ctypes.c_wchar*32),('oem',ctypes.c_wchar*260)]


@dataclass
class Sample:
    axes:dict
    buttons:int
    timestamp:float
    identity:tuple


class WindowsPad:
    def __init__(self):
        self.api=None
        if sys.platform=='win32':
            self.api=ctypes.WinDLL('winmm')
            self.api.joyGetDevCapsW.argtypes=[ctypes.c_size_t,ctypes.POINTER(JoyCaps),ctypes.c_uint]
            self.api.joyGetPosEx.argtypes=[ctypes.c_uint,ctypes.POINTER(JoyInfo)]
    def devices(self):
        if self.api is None:return []
        found=[]
        for index in range(min(16,self.api.joyGetNumDevs())):
            caps=JoyCaps();info=JoyInfo();info.size=ctypes.sizeof(info);info.flags=0xff
            if self.api.joyGetDevCapsW(index,ctypes.byref(caps),ctypes.sizeof(caps))==0 and self.api.joyGetPosEx(index,ctypes.byref(info))==0:
                if (caps.manufacturer,caps.product)==(0x054c,0x0ce6):caps.name='DualSense CFI-ZCT1G'
                found.append((index,caps))
        return found
    def read(self,index,caps):
        if self.api is None:raise OSError('Windows에서 컨트롤러 입력을 지원합니다.')
        info=JoyInfo();info.size=ctypes.sizeof(info);info.flags=0xff
        code=self.api.joyGetPosEx(index,ctypes.byref(info))
        if code:raise OSError(f'컨트롤러 연결 끊김 / 입력 오류 ({code})')
        supported={'X','Y'}
        for bit,axis in ((1,'Z'),(2,'R'),(4,'U'),(8,'V')):
            if caps.caps&bit:supported.add(axis)
        axes={}
        for axis in AXES:
            if axis not in supported:continue
            lo,hi=getattr(caps,axis+'min'),getattr(caps,axis+'max')
            if hi>lo:axes[axis]=max(-1.,min(1.,2*(getattr(info,axis)-lo)/(hi-lo)-1))
        return Sample(axes,info.buttons|pov_buttons(info.pov if caps.caps&16 else 65535),time.monotonic(),(index,caps.manufacturer,caps.product,caps.name))


def pov_buttons(pov):
    """WinMM POV hundredths of a degree; diagonal presses include both arrows."""
    if type(pov) is not int or not 0<=pov<36000:return 0
    sector=((pov+2250)//4500)%8
    directions=((0,),(0,1),(1,),(1,2),(2,),(2,3),(3,),(3,0))[sector]
    return sum(1<<(32+index) for index in directions)


def deadzone(value,zone):
    if not math.isfinite(value):raise ValueError('스틱 입력 오류')
    return math.copysign(max(0.,(min(1.,abs(value))-zone)/(1-zone)),value)


class PadGate:
    """Fresh input + neutral/released arming prevents hot-plug motion."""
    def __init__(self):self.reset()
    def reset(self):self.armed=False;self.identity=None;self.last_time=None;self.enabled=False;self.previous_held=True
    def evaluate(self,sample,now,allowed,forward='Y',turn='X',deadman=4,stop=2,zone=.15,invert_forward=True,invert_turn=True,toggle=False):
        if not sample or not allowed or not 0<=now-sample.timestamp<=.25:
            self.reset();return (0.,0.,False,'조종 대기 / 입력 중단')
        if self.last_time is not None and now-self.last_time>.3:self.armed=False;self.enabled=False;self.previous_held=True
        self.last_time=now
        if sample.identity!=self.identity:self.armed=False;self.enabled=False;self.previous_held=True;self.identity=sample.identity
        if not 0<=zone<=.5 or forward==turn or forward not in sample.axes or turn not in sample.axes or deadman==stop:
            self.armed=False;return (0.,0.,False,'축 / 버튼 설정 확인')
        if sample.buttons&(1<<stop):self.reset();return (0.,0.,True,'정지 버튼')
        v=deadzone(sample.axes[forward],zone)*(-1 if invert_forward else 1)
        w=deadzone(sample.axes[turn],zone)*(-1 if invert_turn else 1)
        held=bool(sample.buttons&(1<<deadman))
        if toggle:
            pressed=held and not self.previous_held
            self.previous_held=held
            if not held and v==0 and w==0:self.armed=True
            if not self.armed:return (0.,0.,False,'스틱 중앙 + L1 해제 후 활성화')
            if pressed:
                if self.enabled:self.enabled=False
                elif v==0 and w==0:self.enabled=True
                else:return (0.,0.,False,'스틱 중앙에서 L1을 눌러 활성화하세요.')
            if not self.enabled:return (0.,0.,False,'AMR 비활성 · L1 한번 눌러 활성화')
            return (v,w,False,'AMR 활성 · 주행 중' if v or w else 'AMR 활성 · 스틱 중앙 정지')
        if not held and v==0 and w==0:self.armed=True
        if not self.armed:return (0.,0.,False,'스틱 중앙 + L1 해제 후 시작')
        if not held:return (0.,0.,False,'L1 누르는 동안 조종')
        return (v,w,False,'조종 중' if v or w else '스틱 중앙 · 정지')


def jog_packet(desired,now):
    """Remove internal lease metadata; expired commands never reach the robot."""
    if desired is None or now>desired.get('_expires',float('inf')):return None
    return {k:desired[k] for k in ('vx','vy','w')}
