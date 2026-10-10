"""Embedded preview consumer; camera capture lives outside the Tk thread."""
import base64,io,json,subprocess,time,uuid
from pathlib import Path
from tkinter import ttk
import tkinter as tk
from .vision_stream import publish_frame
from PIL import Image,ImageTk


def preview_record(path,now=None):
    now=time.time() if now is None else now
    try:
        value=json.loads(Path(path).read_text(encoding='utf-8'))
        if not -.1<=now-value['timestamp']<=.7:return None
        return value
    except (OSError,ValueError,KeyError,TypeError):return None


class VisionCameraPane(ttk.Frame):
    def __init__(self,parent,root):
        super().__init__(parent);self.root_path=Path(root);self.process=None;self.output=None
        self.stop_path=None;self.closed=False;self.stamp=None;self.photo=None;self.timer=None
        self.path=self.root_path/'.delivery/d455_camera_preview.json'
        row=ttk.Frame(self);row.pack(fill='x',pady=4)
        ttk.Button(row,text='카메라 시작 / 연결',command=self.start).pack(side='left',padx=3)
        ttk.Button(row,text='카메라 중지',command=self.stop).pack(side='left',padx=3)
        teach=ttk.Frame(self);teach.pack(fill='x')
        ttk.Button(teach,text='손잡이·구멍 등록',command=self.register_handle).pack(side='left',padx=3)
        ttk.Button(teach,text='등록 삭제',command=self.delete_handle).pack(side='left',padx=3)
        ttk.Button(self,text='학습한 제품 사진 화면에 표시',command=self.show_product_photo).pack(fill='x',pady=3)
        self.message=tk.StringVar(value='카메라 시작 버튼을 누르세요.')
        ttk.Label(self,textvariable=self.message,wraplength=325).pack(fill='x')
        self.image=ttk.Label(self,text='D455 실시간 영상 대기',anchor='center');self.image.pack(fill='x',pady=5)
        self.details=tk.Text(self,height=10,width=34,wrap='word',font=('맑은 고딕',9),state='disabled')
        scroll=ttk.Scrollbar(self,command=self.details.yview);scroll.pack(side='right',fill='y');self.details.configure(yscrollcommand=scroll.set)
        self.details.pack(fill='both',expand=True)
        ttk.Label(self,text='카메라 광학 좌표 · 실제 로봇 좌표는 실측 보정 필요',wraplength=325).pack(fill='x',pady=4)
        self.timer=self.after(100,self.tick)

    def show_product_photo(self):
        path=self.root_path/'.delivery/kahl_screen_reference.png'
        if not path.exists():self.message.set('제공된 제품 사진 등록이 필요합니다.');return
        dialog=tk.Toplevel(self);dialog.title('KAHL-1057-B(R) · 사진 표시 · F11 전체 화면 / Esc 닫기');dialog.geometry('1000x850')
        view=tk.Canvas(dialog,bg='white',highlightthickness=0);view.pack(fill='both',expand=True)
        original=Image.open(path).convert('RGB')
        def draw(event):
            if event.width<10 or event.height<10:return
            image=original.copy();image.thumbnail((event.width,event.height));dialog.photo=ImageTk.PhotoImage(image,master=dialog)
            view.delete('all');view.create_image(event.width/2,event.height/2,image=dialog.photo)
        view.bind('<Configure>',draw);dialog.bind('<Escape>',lambda e:dialog.destroy())
        dialog.bind('<F11>',lambda e:dialog.attributes('-fullscreen',not dialog.attributes('-fullscreen')))
        return dialog

    def delete_handle(self):
        (self.root_path/'.delivery/d455_handle_target.json').unlink(missing_ok=True)
        self.message.set('구멍 등록 삭제 · 결합 추종의 삽입은 보류됩니다.')

    def register_handle(self):
        snapshot=preview_record(self.path)
        board=(snapshot or {}).get('board') or {}
        if not snapshot or not snapshot.get('image_jpeg_base64') or not board.get('valid') or board.get('markers_used')!=2 or board.get('reprojection_px',99)>2:
            self.message.set('카메라 연결 후 등록된 두 마커와 손잡이를 함께 보여주세요.');return
        try:picture=Image.open(io.BytesIO(base64.b64decode(snapshot['image_jpeg_base64'],validate=True)))
        except (ValueError,OSError):self.message.set('카메라 원본 영상 오류');return
        dialog=tk.Toplevel(self);dialog.title('손잡이 등록 · 실제 열쇠구멍 중심 클릭')
        ttk.Label(dialog,text='실제 열쇠구멍 중심을 클릭한 후 저장하세요. 마커와 손잡이는 같은 판넬에 고정하세요.').pack(padx=10,pady=8)
        view=tk.Canvas(dialog,width=picture.width,height=picture.height,highlightthickness=0);view.pack()
        photo=ImageTk.PhotoImage(picture,master=dialog);view.create_image(0,0,image=photo,anchor='nw');dialog.image=photo
        selected=[];mark=[];note=tk.StringVar(value='구멍 중심 선택 대기')
        ttk.Label(dialog,textvariable=note).pack(pady=4)
        row=ttk.Frame(dialog);row.pack(fill='x',padx=10,pady=6)
        ttk.Label(row,text='마커 평면 → 구멍 입구 높이 mm (SIM 추정)').pack(side='left')
        height=tk.StringVar(value='32');ttk.Entry(row,textvariable=height,width=8).pack(side='left',padx=5)
        def choose(event):
            selected[:]=[event.x,event.y]
            for item in mark:view.delete(item)
            mark[:]=[view.create_oval(event.x-6,event.y-6,event.x+6,event.y+6,outline='#00ff55',width=2)]
            note.set(f'구멍 중심: {event.x}, {event.y} px · 주변 외형도 함께 등록')
        def save():
            try:
                import math
                if not selected:raise ValueError('구멍 중심을 클릭하세요.')
                z=float(height.get())
                if not math.isfinite(z) or not 0<=z<=100:raise ValueError('표면 높이는 0~100 mm입니다.')
                publish_frame(self.root_path/'.delivery/d455_handle_target.json',dict(version=1,revision=uuid.uuid4().hex,
                    center_px=list(selected),surface_z_mm=z,image_jpeg_base64=snapshot['image_jpeg_base64'],
                    board=snapshot['board'],camera=snapshot['camera'],dimension_source='estimated_plane'))
                self.message.set('손잡이 외형·구멍 중심 등록 완료 · 추종 정지 후 다시 시작하세요.');dialog.destroy()
            except (ValueError,OSError,KeyError) as error:note.set(str(error))
        view.bind('<Button-1>',choose);ttk.Button(dialog,text='선택한 구멍·외형 저장',command=save).pack(pady=8)

    def start(self):
        if self.process and self.process.poll() is None:return
        if preview_record(self.path):self.message.set('실행 중인 카메라 영상 공유');return
        executable=self.root_path/'.venv-vision/Scripts/python.exe'
        if not executable.exists():self.message.set('비전 환경 없음 · docs/D455_ARUCO_KO.md 확인');return
        self.stop_path=self.root_path/'.delivery'/('camera-stop-'+uuid.uuid4().hex)
        self.output=(self.root_path/'.delivery/d455_camera_worker.log').open('a',encoding='utf-8')
        try:
            self.process=subprocess.Popen([str(executable),'-X','utf8',str(self.root_path/'tools/d455_aruco_preview.py'),
                '--no-window','--preview-json',str(self.path),'--stop-file',str(self.stop_path),
                '--log',str(self.root_path/'.delivery/d455_aruco.jsonl')],cwd=self.root_path,
                stdout=self.output,stderr=self.output,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            self.message.set('D455 연결 중 · 카메라 전용 작업 프로세스')
        except OSError as exc:
            self.output.close();self.output=None;self.message.set(str(exc))

    def stop(self):
        if self.process and self.process.poll() is None:
            self.stop_path.touch();self.message.set('카메라 종료 요청 · 프레임 처리 후 종료')
        else:self.message.set('이 화면이 시작한 카메라만 중지할 수 있습니다.')

    def show_record(self,record):
        valid=isinstance(record,dict) and record.get('vision_source')!='built_in_demo' and -.1<=time.time()-record.get('timestamp',0)<=.7
        lines=['실제 카메라 인식' if valid else '실제 인식 데이터 대기 / 지연']
        if valid:
            markers=record.get('markers',[]);lines.append(f'인식 마커: {len(markers)}개')
            for index,m in enumerate(markers,1):
                xyz=m.get('depth_camera_xyz_m') or m.get('camera_xyz_m')
                orientation=m.get('orientation_deg') or {}
                lines.append(f"#{index} {m.get('dictionary')} / ID {m.get('id')}")
                if xyz:lines.append('XYZ mm: '+', '.join(f'{v*1000:.1f}' for v in xyz))
                if orientation.get('tilt_deg') is not None:lines.append(f"기울기: {orientation['tilt_deg']:.2f}°")
                if all(orientation.get(k) is not None for k in ('rx_deg','ry_deg','rz_deg')):lines.append('RX/RY/RZ °: '+', '.join(f'{orientation[k]:+.1f}' for k in ('rx_deg','ry_deg','rz_deg')))
            pair=record.get('center_pair') or {}
            if pair.get('center_distance_m') is not None:lines.append(f"두 중심 거리: {pair['center_distance_m']*1000:.2f} mm")
            for field,label in [('line_depth_deg','전후 각도'),('image_line_deg','영상 기울기')]:
                if pair.get(field) is not None:lines.append(f'{label}: {pair[field]:+.2f}°')
            board=record.get('board') or {};lines.append('두 마커 기준판: '+('유효' if board.get('valid') else str(board.get('reason','대기'))))
            target=record.get('handle_target') or {};lines.append('손잡이·구멍: '+('검출 확인' if target.get('valid') else str(target.get('reason','등록 대기'))))
            if target.get('confidence') is not None:lines.append(f"외형 일치: {target['confidence']:.2f}")
            if target.get('board_xyz_mm'):lines.append('구멍 기준판 XYZ mm: '+', '.join(f'{v:.2f}' for v in target['board_xyz_mm']))
            photo=record.get('photo_target') or {}
            lines.append('제품 사진: '+('구멍 위치 확인 · SIM 전용' if photo.get('valid') else photo.get('reason','등록 대기')))
            if photo.get('center_px'):lines.append('사진 구멍 px: '+', '.join(f'{v:.1f}' for v in photo['center_px']))
            if photo.get('inliers'):lines.append(f"사진 일치 특징점: {photo['inliers']} · 크기/깊이 추정값")
        text='\n'.join(lines)
        if getattr(self,'last_text',None)!=text:
            self.details.configure(state='normal');self.details.delete('1.0','end');self.details.insert('end',text);self.details.configure(state='disabled');self.last_text=text

    def tick(self):
        if self.closed:return
        if self.process and self.process.poll() is not None:
            code=self.process.returncode;self.process=None
            if self.output:self.output.close();self.output=None
            if self.stop_path:self.stop_path.unlink(missing_ok=True)
            self.message.set('카메라 종료' if code==0 else '카메라 오류 · .delivery/d455_camera_worker.log 확인')
        record=preview_record(self.path)
        if record:
            self.message.set('LIVE · 실제 D455 ArUco 영상')
            if self.stamp!=record['timestamp'] and self.winfo_viewable():
                try:
                    picture=Image.open(io.BytesIO(base64.b64decode(record['jpeg_base64'],validate=True)))
                    picture.thumbnail((max(240,self.winfo_width()-8),190))
                    self.photo=ImageTk.PhotoImage(picture,master=self);self.image.configure(image=self.photo,text='');self.stamp=record['timestamp']
                except (ValueError,OSError,KeyError):pass
        else:
            if self.message.get().startswith('LIVE'):self.message.set('카메라 영상 지연 / 연결 종료')
            self.image.configure(image='',text='실제 카메라 영상 없음 / 지연');self.photo=None;self.stamp=None
        self.timer=self.after(100,self.tick)

    def close(self):
        if self.closed:return
        self.stop();self.closed=True
        if self.timer:self.after_cancel(self.timer)
        if self.output:self.output.close();self.output=None
