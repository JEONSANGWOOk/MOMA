"""Door-mounted display marker. Measured handle offset; display geometry, not controller calibration."""
from functools import lru_cache
from .geometry3d import transform,multiply,axis_rotation,box

@lru_cache(maxsize=1)
def marker_cells():
    import cv2
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    bitmap=cv2.aruco.generateImageMarker(dictionary,0,6)
    return tuple(tuple(int(v) for v in row) for row in bitmap)

def marker_center(part):
    handle=[p for face,material in zip(part['asset'].faces,part['asset'].face_materials) if material==36 for p in face]
    if not handle:raise ValueError("CAD 손잡이 형상을 찾을 수 없습니다.")
    center=[(min(p[i] for p in handle)+max(p[i] for p in handle))/2 for i in range(3)]
    # Front-facing, unmirrored equipment coordinates (metres). The marker
    # centre is 30 mm left of the handle left edge, halfway up its 105 mm length.
    center[0]=min(p[0] for p in handle)-.030
    center[2]=min(p[2] for p in handle)+.105/2
    glass=[p for face in getattr(part['asset'],'transparent_faces',()) for p in face]
    center[1]=(min(p[1] for p in glass) if glass else min(p[1] for p in handle))-.0015
    return tuple(center)


def marker_geometry(part):
    try:center=marker_center(part)
    except ValueError:return []
    rows=[('paper',box((.045,.0008,.045)),transform(center),'#ffffff')]
    side=.035;cell=side/6
    for r,line in enumerate(marker_cells()):
        for c,value in enumerate(line):
            if value:continue
            position=(center[0]+((c+.5)/6-.5)*side,center[1]-.0007,center[2]+(.5-(r+.5)/6)*side)
            rows.append((f'cell_{r}_{c}',box((cell,.0006,cell)),transform(position),'#080808'))
    return rows
