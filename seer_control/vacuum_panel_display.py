"""Vacuum-command visualization only. Never operates real IO or confirms a seal."""
import math
from .geometry3d import multiply,transform

class VacuumPanelDisplay:
    def __init__(self):
        self.matrix=None;self.attached=False;self.falling=False;self.velocity=0.;self.status='IO 수신 대기'
    def update(self,on,cup,initial,size,dt,floor_z=0.):
        if self.matrix is None:self.matrix=[list(row) for row in initial]
        if on is True and cup is not None:
            self.attached=True;self.falling=False;self.velocity=0.
            self.matrix=[list(row) for row in multiply(cup,transform((0,0,size[2]/2)))]
            self.status='베큠 ON · 부착 표시 (흡착 검증 아님)'
        elif on is False:
            if self.attached:
                self.falling=True;self.velocity=0.
                # Place on the configured equipment shelf only when released
                # near it; never teleport a remote release back to the cabinet.
                from .arm_physics import inverse
                from .geometry3d import point
                local=point(inverse(initial),tuple(self.matrix[i][3] for i in range(3)))
                if abs(local[0])<=.20 and abs(local[1])<=.15 and -.03<=local[2]<=.20:
                    self.matrix=[list(row) for row in multiply(initial,transform((local[0],local[1],0.)))]
                    self.falling=False
            self.attached=False;self.status='베큠 OFF · 분리'
        elif on is None:
            self.status='IO 미수신 · 마지막 부착 상태 유지'
            if self.attached and cup is not None:self.matrix=[list(row) for row in multiply(cup,transform((0,0,size[2]/2)))]
        if self.falling:
            dt=max(0.,min(.1,dt));self.velocity-=9.81*dt
            self.matrix[2][3]+=self.velocity*dt
            half=sum(abs(self.matrix[2][i])*size[i]/2 for i in range(3))
            if self.matrix[2][3]-half<=floor_z:
                self.matrix[2][3]=floor_z+half;self.velocity=0.;self.falling=False
        return self.matrix


def vacuum_output_state(bank,on_channels=(2,4),off_channels=(3,5)):
    channels=tuple(on_channels)+tuple(off_channels)
    if not channels or any(type(bank.get(str(i))) is not bool for i in channels):return None
    # Blow always releases. Both suction circuits must be active to display a grip.
    if any(bank[str(i)] for i in off_channels):return False
    return all(bank[str(i)] for i in on_channels)


def arm_vacuum_signal(app,config,now):
    source=config.get('source','Runtime')
    if source=='Runtime':
        runtime=getattr(app,'runtime_arm_feedback',None)
        if not runtime:return None
        path=config.get('runtime_io_path','')
        if path:return runtime.signal(path)
        return runtime.vacuum_state()
    if now-getattr(app,'fr5_rx',0)>1.:return None
    bank=getattr(app,'fr5_feedback',{}).get('io',{}).get(config.get('bank','DO'),{})
    if config.get('mode','paired')=='paired':
        return vacuum_output_state(bank,config.get('on_channels',[2,4]),config.get('off_channels',[3,5]))
    channel=config.get('channel')
    value=bank.get(str(channel)) if type(channel) is int else None
    if type(value) is not bool:return None
    return value if config.get('active_high',True) else not value
