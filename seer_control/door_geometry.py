"""Shared door definitions for live display and recorded-trajectory fitting."""

def door_definitions(view):
    if getattr(view,'marker_placement',{}).get('mode')=='hook_teaching':
        from .door_teaching import apply_taught_hinge
        apply_taught_hinge(view)
    scale=view.scales['cad'];doors=[]
    for part in getattr(view.cad_asset,'door_parts',()):
        vertices=[p for face,material in zip(part['asset'].faces,part['asset'].face_materials) if material==36 for p in face]
        if not vertices:continue
        handle=tuple((min(p[i] for p in vertices)+max(p[i] for p in vertices))/2*scale for i in range(3))
        points=[p for face in part['asset'].faces for p in face]
        lo=[min(p[i] for p in points) for i in range(3)];hi=[max(p[i] for p in points) for i in range(3)]
        push=((lo[0]-.018)*scale,(lo[1]-.018)*scale,(lo[2]+.30*(hi[2]-lo[2]))*scale)
        frame=[p for face,mat in zip(part['asset'].faces,part['asset'].face_materials) if mat==33 for p in face]
        plane_y=min(p[1] for p in frame)*scale if frame else None
        doors.append(dict(roller_plane_y=plane_y,name=part['name'],pivot=tuple(v*scale for v in part['pivot']),handle=handle,push=push,sign=part['sign'],surface_faces=[tuple(tuple(v*scale for v in p) for p in face) for face in part['asset'].faces],roller_after_hook_release=part['name'].endswith(':1'),roller_open_angle=True,roller_capture_gap=.04,hook_capture=.065,hook_release=.12,roller_close=part['name'].endswith(':1'),hook_open_only=True,hook_inside_exit=part['name'].endswith(':2'),hook_dwell=0.,hook_faces=[tuple(tuple(v*scale for v in p) for p in face) for face,mat in zip(part['asset'].faces,part['asset'].face_materials) if mat in (33,36,37)]))
    return doors
