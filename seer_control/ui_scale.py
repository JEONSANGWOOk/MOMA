"""Window-aware density, with a user override. No OS DPI settings changed."""
import tkinter as tk
from tkinter import ttk,font as tkfont


class UIScaleMixin:
    def _ui_scale_control(self,parent):
        self.ui_scale=tk.StringVar(value='자동')
        ttk.Combobox(parent,textvariable=self.ui_scale,values=['자동','65%','75%','85%','100%','115%'],
            state='readonly',width=6).pack(side='right',padx=5)
        self.label(parent,'UI 배율',8).pack(side='right',padx=2)
        self.ui_scale.trace_add('write',lambda *args:self._ui_scale_changed())

    def _ui_scale_init(self):
        self._ui_scale_job=None;self._ui_scale_last=None
        choice=self.studio_config.get('ui_scale','자동')
        self.ui_scale.set(choice if choice in ('자동','65%','75%','85%','100%','115%') else '자동')
        self.bind_all('<Map>',self._ui_scale_new_dialog,add='+')
        # Wheel events originate on controls, not the surrounding canvas.
        self.bind_all('<MouseWheel>',self._ui_panel_wheel,add='+')
        self._ui_scale_schedule()

    def _ui_scale_new_dialog(self,event):
        if isinstance(event.widget,tk.Toplevel):
            win=event.widget;sw=self.winfo_screenwidth();sh=self.winfo_screenheight()
            w=min(win.winfo_width(),sw-40);h=min(win.winfo_height(),sh-80)
            if w!=win.winfo_width() or h!=win.winfo_height():win.geometry(f'{w}x{h}+{max(0,(sw-w)//2)}+{max(0,(sh-h)//2)}')
            self._ui_scale_last=None;self._ui_scale_schedule()

    def _ui_scale_changed(self):
        if not hasattr(self,'studio_config'):return
        self.studio_config['ui_scale']=self.ui_scale.get();self._studio_save_settings()
        self._ui_scale_last=None;self._ui_scale_schedule()

    def _ui_scale_schedule(self):
        if not hasattr(self,'_ui_scale_job'):return
        if self._ui_scale_job:self.after_cancel(self._ui_scale_job)
        self._ui_scale_job=self.after(150,self._ui_scale_apply)

    def _ui_panel_wheel(self,event):
        widget=event.widget
        if widget.winfo_class() in ('Text','Treeview','Listbox','TCombobox'):return
        while widget is not None:
            parent=getattr(widget,'master',None)
            if isinstance(parent,tk.Canvas) and widget is not parent:
                parent.yview_scroll(-3 if event.delta>0 else 3,'units');return 'break'
            widget=parent

    def _ui_scale_apply(self):
        self._ui_scale_job=None
        choice=self.ui_scale.get()
        ratio=max(.65,min(1.,self.winfo_width()/1500,self.winfo_height()/900)) if choice=='자동' else float(choice.rstrip('%'))/100
        ratio=round(ratio/0.025)*.025
        if self._ui_scale_last==ratio:return
        self._ui_scale_last=ratio;self.ui_scale_ratio=ratio
        def scaled(value):
            vals=(value,) if isinstance(value,(int,float)) else self.tk.splitlist(value)
            result=tuple(round(float(v)*ratio) for v in vals)
            return result[0] if len(result)==1 else result
        def visit(widget):
            # Store the unscaled values on each widget to prevent cumulative shrink.
            if not hasattr(widget,'_density_base'):
                base={}
                try:base['font']=tkfont.Font(root=self,font=widget.cget('font')).actual()
                except tk.TclError:pass
                for key in ('padx','pady'):
                    try:base[key]=widget.cget(key)
                    except tk.TclError:pass
                widget._density_base=base
                manager=widget.winfo_manager()
                widget._density_layout=(manager,{k:v for k,v in (widget.pack_info() if manager=='pack' else widget.grid_info() if manager=='grid' else {}).items() if k in ('padx','pady','ipadx','ipady')})
            base=widget._density_base
            if 'font' in base:
                f=base['font'];size=f['size'];widget.configure(font=(f['family'],max(6,round(size*ratio)) if size>0 else -max(8,round(-size*ratio)),f['weight'],f['slant']))
            for key in ('padx','pady'):
                if key in base:
                    try:widget.configure(**{key:scaled(base[key])})
                    except (tk.TclError,ValueError):pass
            manager,layout=widget._density_layout
            # pack/grid configure can remap forgotten widgets. Preserve live visibility.
            if layout and widget.winfo_manager()==manager:
                try:
                    values={k:scaled(v) for k,v in layout.items()}
                    if manager=='pack':widget.pack_configure(**values)
                    elif manager=='grid':widget.grid_configure(**values)
                except (tk.TclError,ValueError):pass
            for child in widget.winfo_children():visit(child)
        visit(self)
        style=ttk.Style(self)
        style.configure('.',font=(self.font,max(6,round(9*ratio))))
        style.configure('Treeview',rowheight=max(16,round(25*ratio)))
        style.configure('Treeview.Heading',font=(self.font,max(6,round(9*ratio)),'bold'),padding=round(5*ratio))
        for name,padding in [('TEntry',4),('TCombobox',3),('TButton',4)]:style.configure(name,padding=round(padding*ratio))
        self.navigation.configure(width=round(150*ratio))
        self.operation_page.columnconfigure(0,minsize=round(450*ratio))
        self.operation_page.columnconfigure(1,minsize=0 if getattr(self,'map_focus',False) else max(220,round(310*ratio)))
        # Widgets in the editor use a horizontal paned layout.
        try:
            panes=self.editor_canvas.master.master
            if isinstance(panes,tk.PanedWindow):
                items=panes.panes()
                if len(items)==2:
                    panes.paneconfigure(items[0],minsize=round(450*ratio))
                    panes.paneconfigure(items[1],minsize=round(310*ratio),width=round(390*ratio))
        except tk.TclError:pass

