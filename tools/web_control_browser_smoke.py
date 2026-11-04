"""Local browser integration test. Runs SIM only; never connects hardware."""
import os
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path(__file__).resolve().parents[1]
BASE=os.environ.get('MOMA_WEB_TEST_URL','http://127.0.0.1:8765')
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True,args=['--use-angle=swiftshader','--enable-unsafe-swiftshader'])
    page=browser.new_page(viewport={'width':1600,'height':1000})
    page.set_default_timeout(60000)
    errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
    page.goto(BASE,wait_until='domcontentloaded')
    page.wait_for_selector('.station-marker',timeout=30000)
    page.wait_for_function("!document.querySelector('.loading-model')",timeout=120000)
    assert page.request.get(BASE+'/api/status').json()['mode']=='sim','Use a SIM-only test server'
    page.get_by_role('button',name='제어권 가져오기').click()
    page.get_by_role('button',name='운영 제어 중').wait_for()
    page.locator('select[aria-label="이동 목적지"]').select_option('LM2')
    page.get_by_role('button',name='목적지로 이동').click()
    page.wait_for_function("document.querySelector('.event-list').textContent.includes('navigate')")
    page.get_by_role('button',name='정지',exact=True).click()
    page.locator('.detail-tabs').get_by_role('button',name='I/O',exact=True).click()
    page.locator('.io-table button').nth(3).click()
    page.get_by_role('button',name='ON',exact=True).click()
    page.wait_for_function("document.querySelectorAll('.io-table button')[3].querySelector('.signal:last-child').textContent==='ON'")
    page.get_by_role('button',name='OFF',exact=True).click()
    page.wait_for_function("document.querySelectorAll('.io-table button')[3].querySelector('.signal:last-child').textContent==='OFF'")
    page.locator('.detail-tabs').get_by_role('button',name='미션',exact=True).click()
    page.get_by_label('왼쪽 초기 각도').fill('90')
    page.get_by_label('오른쪽 초기 각도').fill('0')
    page.get_by_role('button',name='초기 각도 적용 · 저장').click()
    page.wait_for_function("document.querySelector('.door-angles').textContent.includes('90.0')")
    page.get_by_label('왼쪽 초기 각도').fill('0')
    page.get_by_role('button',name='초기 각도 적용 · 저장').click()
    page.get_by_role('button',name='방문 목적지 추가',exact=True).click()
    page.get_by_role('button',name='미션 시작',exact=True).click()
    page.wait_for_function("document.querySelector('.mission-progress').textContent.includes('RUNNING')")
    page.get_by_role('button',name='미션 취소',exact=True).click()
    page.wait_for_function("document.querySelector('.mission-progress').textContent.includes('CANCELED')")
    page.locator('.detail-tabs').get_by_role('button',name='개요',exact=True).click()
    page.screenshot(path=str(ROOT/'artifacts/web_control_overview.png'))
    page.get_by_role('button',name='전시 화면',exact=True).click()
    assert page.locator('.exhibition').count()==1
    assert not page.locator('.detail-panel').is_visible()
    page.wait_for_timeout(1200)
    page.screenshot(path=str(ROOT/'artifacts/web_control_exhibition.png'))
    page.get_by_role('button',name='운영 화면',exact=True).click()
    assert page.locator('.detail-panel').is_visible()
    page.set_viewport_size({'width':1366,'height':768})
    page.wait_for_timeout(700)
    page.screenshot(path=str(ROOT/'artifacts/web_control_1366.png'))
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
    page.set_viewport_size({'width':390,'height':844})
    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth')
    assert not page.locator('.world-error').count(),'Full model failed to load'
    assert page.locator('canvas').get_attribute('data-model-stats'),'Original GLB models not rendered'
    assert not errors,errors
    assert not page.request.get(BASE+'/api/scene').json()['errors']
    page.get_by_title('이 화면의 제어권 반환').click()
    page.get_by_title('이 화면에서 운영하기').wait_for()
    print('PASS: SIM navigation/stop, mission start/cancel, IO ON/OFF, initial doors, exhibition, responsive layout and control release; no page errors')
    browser.close()
