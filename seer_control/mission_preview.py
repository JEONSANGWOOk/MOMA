"""Independent map viewport for selecting a loop mission's ordered stops."""
import math
import tkinter as tk
from .smap import path_record_geometry
from .theme import WHITE, INK, BLUE, GREEN, RED
from .route_planner import plan_stops


class MissionMapPreview(tk.Canvas):
    def __init__(self,parent,model,get_route,get_selected,get_pose,select_node,add_node,path_exists,get_plan=None,**kwargs):
        super().__init__(parent,bg=WHITE,highlightthickness=1,highlightbackground='#ccd5e0',**kwargs)
        self.model=model
        self.get_route,self.get_selected,self.get_pose=get_route,get_selected,get_pose
        self.select_node,self.add_node,self.path_exists=select_node,add_node,path_exists
        self.get_plan=get_plan
        self.zoom=1.;self.pan=[0.,0.];self.drag=None;self.transform=(1.,0.,0.)
        self.job=None
        self.bind('<Configure>',lambda event:self.redraw())
        self.bind('<Button-1>',self.click)
        self.bind('<Double-Button-1>',lambda event:self.click(event,True))
        self.bind('<MouseWheel>',lambda event:self.change_zoom(1.15 if event.delta>0 else 1/1.15))
        self.bind('<Button-4>',lambda event:self.change_zoom(1.15))
        self.bind('<Button-5>',lambda event:self.change_zoom(1/1.15))
        self.bind('<ButtonPress-3>',lambda event:setattr(self,'drag',(event.x,event.y,self.pan[:])))
        self.bind('<B3-Motion>',self.pan_move)
        self.bind('<ButtonRelease-3>',lambda event:setattr(self,'drag',None))
        self.bind('<Destroy>',self.cleanup)
        self.refresh()

    def refresh(self):
        self.redraw()
        self.job=self.after(250,self.refresh)

    def cleanup(self,event):
        if event.widget is self and self.job:
            self.after_cancel(self.job);self.job=None

    def fit(self):
        self.zoom=1.;self.pan=[0.,0.];self.redraw()

    def change_zoom(self,factor):
        self.zoom=max(.2,min(12.,self.zoom*factor));self.redraw()

    def pan_move(self,event):
        if self.drag:
            x,y,pan=self.drag
            self.pan=[pan[0]+event.x-x,pan[1]+event.y-y];self.redraw()

    def xy(self,x,y):
        scale,ox,oy=self.transform
        return ox+x*scale,oy-y*scale

    def click(self,event,add=False):
        candidates=[(math.hypot(event.x-self.xy(n['x'],n['y'])[0],event.y-self.xy(n['x'],n['y'])[1]),key)
                    for key,n in self.model.nodes.items()]
        if not candidates:return
        distance,key=min(candidates)
        if distance<=18:
            self.select_node(key)
            if add:self.add_node(key)
            self.redraw()

    def redraw(self):
        self.delete('all')
        width,height=max(1,self.winfo_width()),max(1,self.winfo_height())
        nodes=self.model.nodes
        points=[(n['x'],n['y']) for n in nodes.values()]
        points.extend((wall[i],wall[i+1]) for wall in self.model.walls for i in (0,2))
        cloud=getattr(self.model,'cloud',[])
        points.extend(cloud[::max(1,len(cloud)//1000)])
        if not points:return
        xs,ys=zip(*points);x0,x1=min(xs)-1,max(xs)+1;y0,y1=min(ys)-1,max(ys)+1
        scale=min(max(1,width-24)/(x1-x0),max(1,height-55)/(y1-y0))*.92*self.zoom
        self.transform=(scale,width/2-(x0+x1)*scale/2+self.pan[0],height/2+(y0+y1)*scale/2+self.pan[1])
        for x,y in cloud[::max(1,len(cloud)//2500)]:
            px,py=self.xy(x,y);self.create_rectangle(px,py,px+1,py+1,fill='#8693a2',outline='')
        for wall in self.model.walls:self.create_line(*self.xy(*wall[:2]),*self.xy(*wall[2:]),fill='#596572',width=3)
        records=getattr(self.model,'path_records',[])
        represented=set()
        for record in records:
            pair=frozenset((record.get('a'),record.get('b')))
            if pair in represented:continue
            represented.add(pair)
            geometry=path_record_geometry(record.get('raw',{}))
            if len(geometry)>1:self.create_line(*[v for p in geometry for v in self.xy(*p)],fill='#b3c0ce',width=1)
        for a,b in self.model.edges:
            if a in nodes and b in nodes and frozenset((a,b)) not in represented:
                self.create_line(*self.xy(nodes[a]['x'],nodes[a]['y']),*self.xy(nodes[b]['x'],nodes[b]['y']),fill='#b3c0ce',dash=(3,3))
        route=self.get_route()
        self.plan=self.get_plan() if self.get_plan else plan_stops(self.model,route)
        missing=self.plan['errors']
        for leg in self.plan['legs']:
            if not leg['nodes']:
                for key in (leg['start'],leg['goal']):
                    if key in nodes:
                        x,y=self.xy(nodes[key]['x'],nodes[key]['y'])
                        self.create_oval(x-11,y-11,x+11,y+11,outline=RED,width=2,tags='missing_mission_path')
                continue
            for a,b in zip(leg['nodes'],leg['nodes'][1:]):
                record=next((r for r in records if r.get('a')==a and r.get('b')==b),None)
                geometry=path_record_geometry(record.get('raw',{})) if record else []
                if len(geometry)<2:geometry=[(nodes[a]['x'],nodes[a]['y']),(nodes[b]['x'],nodes[b]['y'])]
                self.create_line(*[v for p in geometry for v in self.xy(*p)],fill=BLUE,width=3,arrow='last',tags='mission_route')
        selected=self.get_selected()
        for key,n in nodes.items():
            x,y=self.xy(n['x'],n['y'])
            steps=[str(i+1) for i,value in enumerate(route) if value==key]
            color=GREEN if steps else '#8b9cb0'
            if key==selected:color=BLUE
            if key in self.plan['excluded']:color=RED
            self.create_oval(x-6,y-6,x+6,y+6,fill=color,outline=WHITE,width=1,tags='preview_node')
            if key in self.plan['excluded']:
                self.create_text(x,y,text='×',fill=WHITE,font=('Malgun Gothic',8,'bold'),tags='excluded_preview_node')
            label=key+('\n['+'/'.join(steps)+']' if steps else '')
            item=self.create_text(x,y-(25 if steps else 17),text=label,fill=INK,font=('Malgun Gothic',8,'bold'),tags='preview_node_label')
            box=self.bbox(item)
            if box:
                backing=self.create_rectangle(box[0]-2,box[1]-1,box[2]+2,box[3]+1,fill=WHITE,outline='')
                self.tag_lower(backing,item)
        pose=self.get_pose()
        if all(type(pose.get(k)) in (int,float) and math.isfinite(pose[k]) for k in ('x','y')):
            x,y=self.xy(pose['x'],pose['y'])
            self.create_oval(x-10,y-10,x+10,y+10,outline=BLUE,width=2,tags='preview_robot')
        caption='경로 확인: '+missing[0] if missing else (('순환 완성' if len(route)>2 and route[0]==route[-1] else '경로 미리보기')+f" · {self.plan['distance']:.1f} m · 예상 {self.plan['seconds']:.0f}초 (대기 제외)" if len(route)>1 else '주행 순서를 추가하세요')
        self.create_text(10,10,anchor='nw',text=caption,fill=RED if missing else GREEN if len(route)>2 and route[0]==route[-1] else INK,
                         width=max(100,width-20),font=('Malgun Gothic',9),tags='preview_status')
