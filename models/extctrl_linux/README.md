# extctrl viewer assets in MOMA-main

These GLB assets are copied from `moma/extctrl_linux/viewer/models` for the MOMA Windows 3D view.

- `SDC-400EU-light.glb` uses the same Three.js Euler XYZ rotation as extctrl (`0, 90, 90` degrees). Its spatial LOD keeps all assembly components and renders about 80,000 triangles for a continuous chassis.
- `FR5_Assembly.glb` supplies the articulated FR5 display and six joint pivots at the extctrl mount (`x=0.35 m`, `z=0.5085 m`, `yaw=135 degrees`).
- `gripper/EndEffector V3_EndOnly_Change1-org.glb` is rendered with its full 104,514 source triangles and mounted at the J6 flange with extctrl's `-90 degree X` rotation and 0.1 m flange offset.

The OpenGL display uses brighter two-sided lighting to keep faces visible from the desktop camera. These models are visual assets; they do not change hardware communication or motion safety behavior.
