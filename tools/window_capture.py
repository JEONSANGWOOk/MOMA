"""Capture only the Tk window created by our GUI smoke test (Windows)."""
import ctypes
from ctypes import wintypes as w
import struct
import zlib


def capture_window(app, path):
    user = ctypes.WinDLL('user32', use_last_error=True)
    gdi = ctypes.WinDLL('gdi32', use_last_error=True)
    user.GetAncestor.argtypes = [w.HWND, w.UINT]
    user.GetAncestor.restype = w.HWND
    user.GetWindowDC.argtypes = [w.HWND]
    user.GetWindowDC.restype = w.HDC
    gdi.CreateCompatibleDC.argtypes = [w.HDC]
    gdi.CreateCompatibleDC.restype = w.HDC
    gdi.CreateCompatibleBitmap.argtypes = [w.HDC, ctypes.c_int, ctypes.c_int]
    gdi.CreateCompatibleBitmap.restype = w.HBITMAP
    gdi.SelectObject.argtypes = [w.HDC, w.HGDIOBJ]
    gdi.SelectObject.restype = w.HGDIOBJ
    user.PrintWindow.argtypes = [w.HWND, w.HDC, w.UINT]
    gdi.GetDIBits.argtypes = [w.HDC, w.HBITMAP, w.UINT, w.UINT, ctypes.c_void_p, ctypes.c_void_p, w.UINT]
    gdi.DeleteObject.argtypes = [w.HGDIOBJ]
    gdi.DeleteDC.argtypes = [w.HDC]
    user.ReleaseDC.argtypes = [w.HWND, w.HDC]
    hwnd = user.GetAncestor(app.winfo_id(), 2)
    rect = w.RECT()
    user.GetWindowRect(hwnd, ctypes.byref(rect))
    width, height = rect.right-rect.left, rect.bottom-rect.top
    dc = user.GetWindowDC(hwnd)
    memory = gdi.CreateCompatibleDC(dc)
    bitmap = gdi.CreateCompatibleBitmap(dc, width, height)
    old = gdi.SelectObject(memory, bitmap)
    try:
        if not user.PrintWindow(hwnd, memory, 2):
            raise RuntimeError('Own test window capture failed')
        header = struct.pack('<IiiHHIIiiII', 40, width, -height, 1, 32, 0, width*height*4, 0, 0, 0, 0)
        info = ctypes.create_string_buffer(header + bytes(16))
        pixels = ctypes.create_string_buffer(width*height*4)
        gdi.SelectObject(memory, old)
        if not gdi.GetDIBits(dc, bitmap, 0, height, pixels, info, 0):
            raise RuntimeError('GetDIBits failed')
        raw = pixels.raw
        rows = bytearray()
        for y in range(height):
            rows.append(0)
            line = raw[y*width*4:(y+1)*width*4]
            for x in range(0, len(line), 4):
                rows.extend((line[x+2], line[x+1], line[x], 255))
        def chunk(kind, data):
            return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
        png = b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,6,0,0,0))
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes(png+chunk(b'IDAT',zlib.compress(rows))+chunk(b'IEND',b''))
    finally:
        gdi.DeleteObject(bitmap)
        gdi.DeleteDC(memory)
        user.ReleaseDC(hwnd, dc)
