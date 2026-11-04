"""Presentation layout over the live viewport and existing operator controls."""
import tkinter as tk
from .theme import CHROME, WHITE, BLUE, RED


def restore(widget,info,order):
    siblings=list(widget.master.pack_slaves())
    following=next((w for w in order[order.index(widget)+1:] if w in siblings),None)
    if following is not None:info=dict(info,before=following)
    widget.pack(**info)


def controls(app):
    app.exhibit_controls=not app.exhibit_controls
    show=app.exhibit_controls
    if not show:
        app.release_drive();app.tabs.select(app.operation_page)
        app.view_mode.set('3D');app._spatial_switch()
    for widget,manager,info,order in app._exhibit_widgets:
        if widget is app.role_header:continue
        if show:
            if manager=='pack':restore(widget,info,order)
            else:widget.grid(**info)
        elif manager=='pack':widget.pack_forget()
        else:widget.grid_remove()
    if show and app.map_focus:app._map_focus_toggle()
    elif not show and not app.map_focus:app._map_focus_toggle()
    app.exhibit_controls_button.configure(text='운영 패널 닫기' if show else '운영 패널')
    app.after_idle(app.draw_map)


def fullscreen(app):
    app.attributes('-fullscreen',not bool(app.attributes('-fullscreen')))


def toggle(app):
    if getattr(app,'exhibit_active',False):
        app.exhibit_active=False
        if app._exhibit_status_job:app.after_cancel(app._exhibit_status_job)
        app.exhibit_bar.destroy()
        for widget,manager,info,order in app._exhibit_widgets:
            if manager=='pack':restore(widget,info,order)
            else:widget.grid(**info)
        if app.map_focus!=app._exhibit_prior_focus:app._map_focus_toggle()
        app.attributes('-fullscreen',app._exhibit_prior_fullscreen)
        app.after_idle(app.draw_map)
        return
    if getattr(app,'role_active','개발자')!='개발자':
        raise ValueError('개발자 모드에서 전시 화면을 열어주세요.')
    app.release_drive()
    app.tabs.select(app.operation_page)
    app.view_mode.set('3D');app._spatial_switch()
    app.exhibit_active=True;app.exhibit_controls=False
    app._exhibit_prior_focus=app.map_focus
    app._exhibit_prior_fullscreen=bool(app.attributes('-fullscreen'))
    widgets=[app.role_header,app.connection_bar,app.status_cards_row,app.navigation,
             app.workspace_titlebar,app.operation_ribbon,app.operation_toolbar,
             app.operation_layers,app.operation_sensor_label,app.operation_info_bar]
    app._exhibit_widgets=[]
    for widget in widgets:
        manager=widget.winfo_manager()
        if manager=='pack':app._exhibit_widgets.append((widget,manager,widget.pack_info(),list(widget.master.pack_slaves())))
        elif manager=='grid':app._exhibit_widgets.append((widget,manager,widget.grid_info(),[]))
    bar=tk.Frame(app,bg=CHROME);app.exhibit_bar=bar
    bar.pack(fill='x',before=app.role_header)
    app.label(bar,'MOMA',18,WHITE,True,bg=CHROME).pack(side='left',padx=(20,8),pady=12)
    app.label(bar,'MOBILE MANIPULATOR',9,'#98acc5',bg=CHROME).pack(side='left',padx=6)
    app.exhibit_status=app.label(bar,'',10,'#54ddb0',True,bg=CHROME)
    app.exhibit_status.pack(side='left',padx=20)
    app.button(bar,'일반 화면',lambda:toggle(app)).pack(side='right',padx=(4,12))
    app.button(bar,'전체 화면',lambda:fullscreen(app)).pack(side='right',padx=4)
    app.button(bar,'■ 정지',lambda:app.action('stop'),RED).pack(side='right',padx=4)
    app.exhibit_controls_button=app.button(bar,'운영 패널',lambda:controls(app),BLUE)
    app.exhibit_controls_button.pack(side='right',padx=4)
    for widget,manager,info,order in app._exhibit_widgets:
        if manager=='pack':widget.pack_forget()
        else:widget.grid_remove()
    if not app.map_focus:app._map_focus_toggle()
    def update():
        if not app.exhibit_active:return
        state=app.current_state()
        mode='LIVE' if app.real else 'SIMULATION'
        linked='연결됨' if app.connected else '연결 대기'
        safety='비상정지' if state.get('emergency') else '장애물 정지' if state.get('blocked') else ''
        battery=state.get('battery')
        power=f' · {battery:.0f}%' if type(battery) in (int,float) else ''
        app.exhibit_status.configure(text=f'{mode} · {linked}{power}'+(' · '+safety if safety else ''),fg='#ffb6b9' if safety or not app.connected else '#54ddb0')
        app._exhibit_status_job=app.after(500,update)
    update();app.after_idle(app.draw_map)
