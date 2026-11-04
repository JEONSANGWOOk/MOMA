"""Passive door-angle estimate from measured hook motion; never robot control."""
import math


class DoorFollower:
    def __init__(self, doors, capture=.035, release=.07, max_angle=math.radians(120)):
        self.doors={d['name']:dict(d) for d in doors}
        self.angles={name:0. for name in self.doors}
        self.capture,self.release,self.max_angle=capture,release,max_angle
        self.active=None
        self.anchor=None
        self.previous=None
        self.stamp=None

    def detach(self):
        self.active=self.anchor=self.previous=self.stamp=None

    def handle(self,name):
        d=self.doors[name];a=self.angles[name]*d['sign'];c,s=math.cos(a),math.sin(a)
        p,h=d['pivot'],d['handle'];x,y=h[0]-p[0],h[1]-p[1]
        return p[0]+c*x-s*y,p[1]+s*x+c*y,h[2]

    def update(self, tip, now):
        if tip is None or len(tip)!=3 or not all(math.isfinite(v) for v in tip):
            self.detach();return
        if self.active is None:
            candidates=[(math.dist(tip,self.handle(name)),name) for name in self.doors]
            if not candidates:return
            distance,name=min(candidates)
            if distance>self.capture:return
            d=self.doors[name];p=d['pivot'];a=-self.angles[name]*d['sign']
            x,y=tip[0]-p[0],tip[1]-p[1]
            if math.hypot(x,y)<.05:return
            self.anchor=(math.cos(a)*x-math.sin(a)*y,math.sin(a)*x+math.cos(a)*y,tip[2])
            self.active=name;self.previous=tuple(tip);self.stamp=now
            return
        dt=now-self.stamp
        # Telemetry gaps and relocation jumps must not fling a virtual door.
        if not 0<dt<=.5 or math.dist(tip,self.previous)>.04+1.0*dt:
            self.detach();return
        d=self.doors[self.active];p=d['pivot'];x,y=tip[0]-p[0],tip[1]-p[1]
        ax,ay,z=self.anchor
        if abs(math.hypot(x,y)-math.hypot(ax,ay))>self.release or abs(tip[2]-z)>self.release:
            self.detach();return
        angle=math.atan2(ax*y-ay*x,ax*x+ay*y)*d['sign']
        if angle<-.08 or angle>self.max_angle+.08:
            self.detach();return
        angle=max(0.,min(self.max_angle,angle))
        if abs(angle-self.angles[self.active])>.08+2.*dt:
            self.detach();return
        self.angles[self.active]=angle
        self.previous=tuple(tip);self.stamp=now


class ContactDoorFollower(DoorFollower):
    """Kinematic display estimate: hook can pull; roller can only push open.

    No mass, friction, torque, latch or force measurement is inferred here.
    """
    def __init__(self, doors, **kwargs):
        super().__init__(doors, **kwargs)
        self.contact_mode='hook'

    def handle(self,name):
        if self.contact_mode=='hook':return super().handle(name)
        d=self.doors[name];h=d.get('push')
        if h is None:return (float('inf'),)*3
        p=d['pivot'];a=self.angles[name]*d['sign'];c,s=math.cos(a),math.sin(a)
        x,y=h[0]-p[0],h[1]-p[1]
        return p[0]+c*x-s*y,p[1]+s*x+c*y,h[2]

    def update_contacts(self,hook,roller,now):
        samples={'hook':hook,'push':roller}
        if self.active is None:
            candidates=[]
            for mode,tip in samples.items():
                if tip is None or len(tip)!=3 or not all(math.isfinite(v) for v in tip):continue
                self.contact_mode=mode
                for name in self.doors:
                    candidates.append((math.dist(tip,self.handle(name)),mode))
            if not candidates:
                self.detach();return
            distance,self.contact_mode=min(candidates)
            if distance>self.capture:return
        previous=dict(self.angles)
        super().update(samples[self.contact_mode],now)
        # A roller losing its pushing contact must not pull a leaf closed.
        if self.contact_mode=='push' and any(self.angles[n]<a-1e-6 for n,a in previous.items()):
            self.angles.update(previous);self.detach()


def _sub(a,b):return tuple(x-y for x,y in zip(a,b))
def _dot(a,b):return sum(x*y for x,y in zip(a,b))
def _add_scaled(a,b,t):return tuple(x+t*y for x,y in zip(a,b))


def closest_triangle(p,triangle):
    """Closest point on a triangle, including its edges and vertices."""
    a,b,c=triangle;ab=_sub(b,a);ac=_sub(c,a);ap=_sub(p,a)
    d1,d2=_dot(ab,ap),_dot(ac,ap)
    if d1<=0 and d2<=0:return a
    bp=_sub(p,b);d3,d4=_dot(ab,bp),_dot(ac,bp)
    if d3>=0 and d4<=d3:return b
    vc=d1*d4-d3*d2
    if vc<=0 and d1>=0 and d3<=0:
        return _add_scaled(a,ab,d1/max(1e-18,d1-d3))
    cp=_sub(p,c);d5,d6=_dot(ab,cp),_dot(ac,cp)
    if d6>=0 and d5<=d6:return c
    vb=d5*d2-d1*d6
    if vb<=0 and d2>=0 and d6<=0:return _add_scaled(a,ac,d2/max(1e-18,d2-d6))
    va=d3*d6-d5*d4
    if va<=0 and d4-d3>=0 and d5-d6>=0:
        return _add_scaled(b,_sub(c,b),(d4-d3)/max(1e-18,(d4-d3)+(d5-d6)))
    denom=va+vb+vc
    if abs(denom)<1e-18:
        # Degenerate triangles contribute only their surviving segments.
        choices=[]
        for one,two in ((a,b),(b,c),(c,a)):
            delta=_sub(two,one);t=max(0.,min(1.,_dot(_sub(p,one),delta)/max(1e-18,_dot(delta,delta))))
            q=_add_scaled(one,delta,t);choices.append((math.dist(p,q),q))
        return min(choices)[1]
    return tuple(a[i]+ab[i]*(vb/denom)+ac[i]*(vc/denom) for i in range(3))


class SurfaceDoorFollower(ContactDoorFollower):
    """Passive unilateral roller/mesh contact with a revolute hinge.

    Uses CAD dimensions, swept roller radius and measured displacement. It is
    a kinematic contact constraint, not a measured force/torque dynamics model.
    """
    def __init__(self,doors,roller_radius=.025,**kwargs):
        super().__init__(doors,**kwargs)
        self.roller_radius=float(roller_radius)
        self._roller_previous=None;self._roller_stamp=None
        self._hook_wait=None
        self._hook_previous=None;self._hook_stamp=None
        self._inside_seen=set();self._exit_ready=set()
        self.angular_velocity=0.;self.penetration=0.;self.contact_point=None
        self._triangles={}
        for name,d in self.doors.items():
            self._triangles[name]=[(face,tuple(min(p[i] for p in face) for i in range(3)),
                tuple(max(p[i] for p in face) for i in range(3))) for face in d.get('surface_faces',())]
        def build(rows):
            if not rows:return None
            lo=tuple(min(row[1][i] for row in rows) for i in range(3))
            hi=tuple(max(row[2][i] for row in rows) for i in range(3))
            if len(rows)<=12:return (lo,hi,None,None,rows)
            axis=max(range(3),key=lambda i:hi[i]-lo[i])
            rows=sorted(rows,key=lambda r:r[1][axis]+r[2][axis]);mid=len(rows)//2
            return (lo,hi,build(rows[:mid]),build(rows[mid:]),None)
        self._trees={name:build(rows) for name,rows in self._triangles.items()}
        self._hook_trees={name:build([(face,tuple(min(p[i] for p in face) for i in range(3)),
            tuple(max(p[i] for p in face) for i in range(3))) for face in d.get('hook_faces',())])
            for name,d in self.doors.items()}
    def detach(self):
        super().detach()
        self._hook_wait=None

    def handle(self,name):
        if self.contact_mode=='hook' and self.doors[name].get('hook_faces'):
            return (float('inf'),)*3
        return super().handle(name)
    def _capture_frame_hook(self,tip,now,contact=False):
        if tip is None or len(tip)!=3 or not all(math.isfinite(v) for v in tip):
            self._hook_wait=None;return False
        candidates=[]
        for name,tree in self._hook_trees.items():
            if tree is None:continue
            if self.doors[name].get('hook_inside_exit') and name not in self._exit_ready:continue
            near=self._nearest(name,tip,self.angles[name],self.capture,self._hook_trees)
            if near is not None:candidates.append((near[0],name))
        if not candidates:
            self._hook_wait=None;return False
        _,name=min(candidates);d=self.doors[name];p=d['pivot']
        dwell=d.get('hook_dwell',0.)
        if dwell and not contact:
            pending=self._hook_wait
            if (pending is None or pending[0]!=name or not 0<now-pending[3]<=.5
                    or math.dist(tip,pending[1])>.005):
                self._hook_wait=(name,tuple(tip),now,now)
                return True
            self._hook_wait=(name,pending[1],pending[2],now)
            if now-pending[2]<dwell:return True
        self._hook_wait=None
        local=self._rotate(name,tip,-self.angles[name]*d['sign'])
        if math.hypot(local[0]-p[0],local[1]-p[1])<.05:return False
        self.active=name;self.contact_mode='hook'
        self.anchor=(local[0]-p[0],local[1]-p[1],tip[2])
        self.previous=tuple(tip);self.stamp=now
        return True
    def _observe_inside_exit(self,tip,previous,stamp,now):
        if tip is None or previous is None or stamp is None or not 0<now-stamp<=.5 or math.dist(tip,previous)>.04+now-stamp:
            self._inside_seen.clear();self._exit_ready.clear();return
        for name,d in self.doors.items():
            if not d.get('hook_inside_exit'):continue
            faces=d.get('hook_faces',())
            if not faces:continue
            local=self._rotate(name,tip,-self.angles[name]*d['sign'])
            old=self._rotate(name,previous,-self.angles[name]*d['sign'])
            points=[p for face in faces for p in face]
            lo=[min(p[i] for p in points) for i in range(3)]
            hi=[max(p[i] for p in points) for i in range(3)]
            plane=(lo[1]+hi[1])/2
            # This CAD's enclosure interior is on the +Y side of the door.
            in_span=lo[0]-.03<=local[0]<=hi[0]+.03 and lo[2]-.03<=local[2]<=hi[2]+.03
            if not in_span or abs(local[1]-plane)>.25:
                self._inside_seen.discard(name);self._exit_ready.discard(name);continue
            if local[1]>plane+.008:self._inside_seen.add(name)
            if name in self._inside_seen and local[1]<old[1]-.001:self._exit_ready.add(name)

    def _swept_hook_contact(self,previous,tip,stamp,now):
        if previous is None or tip is None or stamp is None:return False
        if len(tip)!=3 or not all(math.isfinite(v) for v in tip):return False
        dt=now-stamp;distance=math.dist(previous,tip)
        if not 0<dt<=.5 or distance>.04+dt or distance<1e-6:return False
        displacement=_sub(tip,previous)
        steps=max(1,min(128,math.ceil(distance/.004)))
        for i in range(steps+1):
            sample=_add_scaled(previous,displacement,i/steps)
            for name,tree in self._hook_trees.items():
                if tree is None:continue
                if self._nearest(name,previous,self.angles[name],.006,self._hook_trees) is not None:continue
                near=self._nearest(name,sample,self.angles[name],.006,self._hook_trees)
                if near is None:continue
                r=_sub(near[1],self.doors[name]['pivot']);sign=self.doors[name]['sign']
                tangent=(-r[1]*sign,r[0]*sign,0.)
                if _dot(tangent,tangent)<.0025 or _dot(displacement,tangent)<=1e-7:continue
                # Capture the first surface crossing even if the last sample
                # stopped on the far side. Keep the measured robot pose intact.
                if self._capture_frame_hook(sample,stamp,contact=True):
                    self._update_frame_push(tip,now)
                    return True
        return False

    def _update_frame_push(self,tip,now):
        # A hook may slide along the frame: validate contact against the actual
        # rotated frame rather than a fixed-radius circle around the first hit.
        name=self.active;start=self.angles[name];stamp=self.stamp
        if tip is None or len(tip)!=3 or not all(math.isfinite(v) for v in tip):
            self.detach();return
        dt=now-stamp
        if not 0<dt<=.5 or math.dist(tip,self.previous)>.04+dt:
            self.detach();return
        d=self.doors[name];p=d['pivot'];ax,ay,z=self.anchor
        x,y=tip[0]-p[0],tip[1]-p[1]
        if math.hypot(x,y)<.05:self.detach();return
        target=math.atan2(ax*y-ay*x,ax*x+ay*y)*d['sign']
        if target<start-1e-6:
            self.detach();return
        target=min(self.max_angle,max(0.,target))
        if target-start>.08+2.*dt:
            self.detach();return
        near=self._nearest(name,tip,target,self.release,self._hook_trees)
        if near is None:
            self.detach();return
        self.angles[name]=target
        self.previous=tuple(tip);self.stamp=now
        self.angular_velocity=(target-start)/dt
        self.contact_point=near[1]

    def _rotate(self,name,p,angle):
        pivot=self.doors[name]['pivot'];c,s=math.cos(angle),math.sin(angle)
        x,y=p[0]-pivot[0],p[1]-pivot[1]
        return pivot[0]+c*x-s*y,pivot[1]+s*x+c*y,p[2]
    def _nearest(self,name,p,angle,limit,trees=None):
        local=self._rotate(name,p,-angle*self.doors[name]['sign'])
        best=None;distance=limit
        trees=self._trees if trees is None else trees
        stack=[trees[name]] if trees[name] else []
        while stack:
            lo,hi,left,right,rows=stack.pop()
            if any(local[i]<lo[i]-distance or local[i]>hi[i]+distance for i in range(3)):continue
            if rows is None:
                stack.extend((left,right));continue
            for face,lo,hi in rows:
                if any(local[i]<lo[i]-distance or local[i]>hi[i]+distance for i in range(3)):continue
                q=closest_triangle(local,face);gap=math.dist(local,q)
                if gap<=distance:distance=gap;best=q
        return (distance,self._rotate(name,best,angle*self.doors[name]['sign'])) if best is not None else None
    def update_contacts(self,hook,roller,now):
        hook_previous,hook_stamp=self._hook_previous,self._hook_stamp
        hook_valid=hook is not None and len(hook)==3 and all(math.isfinite(v) for v in hook)
        self._hook_previous=tuple(hook) if hook_valid else None
        self._hook_stamp=now if hook_valid else None
        self._observe_inside_exit(hook if hook_valid else None,hook_previous,hook_stamp,now)
        valid=(roller is not None and len(roller)==3 and all(math.isfinite(v) for v in roller))
        previous=self._roller_previous;stamp=self._roller_stamp
        self.angular_velocity=0.;self.penetration=0.;self.contact_point=None
        self._roller_previous=tuple(roller) if valid else None
        self._roller_stamp=now if valid else None
        dt=now-stamp if stamp is not None else None
        continuous=(valid and previous is not None and dt is not None and 0<dt<=.5
                    and math.dist(previous,roller)<=.04+dt)
        if not continuous:
            if self.contact_mode=='push':self.detach()
            previous=roller if valid else None
        if not any(self._triangles.values()):
            return super().update_contacts(hook,roller,now)
        # Update an attached hook first. If it releases, test the roller in this
        # same sample so handoff does not reset an already-open door.
        hook_updated=self.active is not None and self.contact_mode=='hook'
        if hook_updated:
            if self.doors[self.active].get('hook_open_only') or self.doors[self.active].get('hook_only'):
                self._update_frame_push(hook,now)
            else:DoorFollower.update(self,hook,now)
        if self.active is None and not hook_updated:
            if self._swept_hook_contact(hook_previous,hook,hook_stamp,now):return
            if self._capture_frame_hook(hook,now):return
        best=None
        if valid and previous is not None:
            displacement=_sub(roller,previous)
            steps=max(1,min(64,math.ceil(math.dist(previous,roller)/max(.003,self.roller_radius*.5))))
            for name in self.doors:
                if not self._triangles[name] or self.doors[name].get('hook_only'):continue
                angle=self.angles[name];pivot=self.doors[name]['pivot'];sign=self.doors[name]['sign']
                hit=None
                for i in range(steps+1):
                    sample=_add_scaled(previous,displacement,i/steps)
                    near=self._nearest(name,sample,angle,self.roller_radius+.006)
                    if near is not None:hit=near;break
                if hit is None:continue
                distance,contact=hit;r=_sub(contact,pivot)
                tangent=(-r[1]*sign,r[0]*sign,0.)
                lever2=_dot(tangent,tangent)
                if lever2<.0025:continue
                amount=_dot(displacement,tangent)/lever2 if continuous else 0.
                # The sphere must move into the contacted surface, not away.
                inward=_dot(displacement,_sub(previous,contact))
                if amount>1e-7 and inward>1e-8:continue
                if amount< -1e-7:continue
                if self.active and self.contact_mode=='hook' and self.active!=name:continue
                candidate=(amount,-distance,name,contact)
                if best is None or candidate[:2]>best[:2]:best=candidate
        if best is not None:
            amount,_distance,name,contact=best
            # A nonmoving nearby roller must not steal an active hook.
            if self.active is not None and self.contact_mode=='hook' and amount<=1e-7:return
            start=self.angles[name]
            maximum=min(self.max_angle,start+(2.*dt if continuous else 0.))
            target=min(maximum,start+max(0.,amount))
            # Resolve remaining sphere penetration in the permitted opening
            # direction; never invent a closing pull or exceed the sample cap.
            near=self._nearest(name,roller,target,self.roller_radius)
            while continuous and amount>1e-7 and near is not None and near[0]<self.roller_radius-.001 and target<maximum:
                target=min(maximum,target+.005)
                near=self._nearest(name,roller,target,self.roller_radius)
            self.angles[name]=target
            self.active=name;self.contact_mode='push';self.anchor=None
            self.angular_velocity=(target-start)/dt if continuous else 0.
            self.penetration=max(0.,self.roller_radius-near[0]) if near else 0.
            self.contact_point=contact
            return
        if self.contact_mode=='push':self.detach()
        # Hook acquisition remains at the measured handle, with the original
        # radial/height and telemetry discontinuity checks.
        self.contact_mode='hook'
        if not (hook_updated and self.active is not None):
            DoorFollower.update(self,hook,now)
