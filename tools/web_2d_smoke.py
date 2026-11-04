"""Canvas2D isolation, map selection and 3D restoration (SIM-only)."""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
BASE=os.environ.get('MOMA_WEB_TEST_URL','http://127.0.0.1:8766')
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True,args=['--use-angle=swiftshader','--enable-unsafe-swiftshader'])
    page=browser.new_page(viewport={'width':1600,'height':1000})
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(BASE,wait_until='domcontentloaded')
    assert page.request.get(BASE+'/api/status').json()['mode']=='sim'
    page.wait_for_function("document.querySelector('.world canvas')?.dataset.modelStats",timeout=120000)
    page.locator('.view-toggle button').first.click()
    canvas=page.locator('.plan-view canvas')
    canvas.wait_for(state='visible')
    page.wait_for_function("Number(document.querySelector('.plan-view canvas')?.dataset.lidarPoints)>0")
    assert canvas.evaluate('(c)=>!!c.getContext("2d")')
    assert not page.locator('.world canvas').is_visible()
    rect=canvas.bounding_box();w,h=rect['width'],rect['height']
    scale=min(800,(w-90)/19,(h-90)/10)
    # The real map hit-test selects LM2; it does not issue navigation commands.
    canvas.click(position={'x':(7-9.5)*scale+w/2,'y':h/2-(2-5)*scale})
    assert page.locator('.destination select').first.input_value()=='LM2'
    page.wait_for_timeout(500)
    page.screenshot(path=str(ROOT/'artifacts/web_true_2d.png'))
    before=canvas.screenshot()
    page.mouse.move(rect['x']+w/2,rect['y']+h/2)
    page.mouse.wheel(0,-250);page.wait_for_timeout(300)
    assert before!=canvas.screenshot()
    page.locator('.view-toggle button').last.click()
    assert not page.locator('.plan-view').count()
    assert page.locator('.world canvas').is_visible()
    page.locator('.view-toggle button').first.click();page.wait_for_timeout(200)
    assert page.locator('.plan-view canvas').is_visible()
    assert page.request.get(BASE+'/api/status').json()['command']['status']=='IDLE'
    assert not errors,errors
    print('PASS: actual Canvas2D context, no visible 3D model, LiDAR, station selection, zoom and 3D restoration; no commands sent')
    browser.close()
