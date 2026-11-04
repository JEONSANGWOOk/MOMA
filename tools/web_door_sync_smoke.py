"""Exercise actual desktop door methods -> IPC -> browser. No hardware writes."""
import math
import os
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
os.environ['MOMA_DOOR_SYNC_PATH']=str(ROOT/'web_control/.runtime/desktop-door-test.json')
from seer_control.door_sync import publish
from seer_control.spatial_ui import SpatialMixin
from playwright.sync_api import sync_playwright
view=NS(cad_asset=NS(name='SIMPLE_WHOLE.glb',door_parts=[{'name':'door:1'},{'name':'door:2'}]),
    _hook_demo=None,cad_door_hold_angles={},cad_door_hold_angle=None,cad_doors_auto=Mock(),
    real_doors_follow=NS(get=lambda:True),webcam_doors_enabled=NS(get=lambda:False),manual_real_door_display=None)
view.cad_door_angle=lambda part:view.cad_door_hold_angles.get(part['name'],0.)
app=NS(world3d=view,real=False,view_mode=Mock(),_spatial_switch=Mock(),draw_map=Mock(),log=Mock())
app._spatial_publish_door_state=lambda command=False:publish(app,command=command)
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True,args=['--use-angle=swiftshader','--enable-unsafe-swiftshader'])
    page=browser.new_page(viewport={'width':1600,'height':1000})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto('http://127.0.0.1:8766',wait_until='domcontentloaded')
    page.wait_for_function("document.querySelector('.world canvas')?.dataset.modelStats",timeout=120000)
    page.locator('.detail-tabs').get_by_role('button',name='미션',exact=True).click()
    SpatialMixin._spatial_set_single_door(app,'left',True)
    page.wait_for_function("document.querySelector('.door-angles').textContent.includes('120.0')")
    SpatialMixin._spatial_set_single_door(app,'right',True)
    page.wait_for_function("document.querySelector('.door-angles').textContent.match(/120.0/g)?.length===2")
    SpatialMixin._spatial_set_single_door(app,'left',False)
    page.wait_for_function("parseFloat(document.querySelectorAll('.door-angles b')[0].textContent)===0&&parseFloat(document.querySelectorAll('.door-angles b')[1].textContent)===120")
    page.wait_for_function("Number(document.querySelectorAll('.detail-scroll input[type=number]')[0].value)===0&&Number(document.querySelectorAll('.detail-scroll input[type=number]')[1].value)===120")
    assert page.get_by_label('왼쪽 초기 각도').input_value() in ('0','0.0')
    assert page.get_by_label('오른쪽 초기 각도').input_value() in ('120','120.0')
    # Continuously changing runtime angles reuse the same command revision.
    view.cad_door_hold_angles={'door:1':math.pi/3,'door:2':math.radians(80)}
    app._door_sync_at=0;publish(app)
    page.wait_for_function("document.querySelector('.door-angles').textContent.includes('60.0')&&document.querySelector('.door-angles').textContent.includes('80.0')")
    page.screenshot(path=str(ROOT/'artifacts/web_desktop_door_sync.png'))
    page.wait_for_function("document.querySelector('.detail-scroll').textContent.includes('마지막 문 각도 유지')",timeout=10000)
    assert '60.0' in page.locator('.door-angles').inner_text()
    assert not errors,errors
    print('PASS: desktop open/close methods and changing angles mirrored in browser without refresh; stale frame holds last angles')
    browser.close()
