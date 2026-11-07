"""Isaac Sim --exec bootstrap. Never sends commands to the real robot."""
import asyncio
import builtins
import os
from pathlib import Path
import omni.kit.app
import omni.usd
from pxr import UsdGeom,UsdLux

async def start():
    for _ in range(5):await omni.kit.app.get_app().next_update_async()
    context=omni.usd.get_context();stage=context.get_stage()
    if stage is None:
        await context.new_stage_async();stage=context.get_stage()
        UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    if UsdGeom.GetStageMetersPerUnit(stage)!=1. or UsdGeom.GetStageUpAxis(stage)!=UsdGeom.Tokens.z:
        print('[MOMA] Stage must be Z-up/metres. Open a new stage then run the receiver.');return
    light=UsdLux.DomeLight.Define(stage,'/World/MOMA_MirrorLight');light.CreateIntensityAttr(500)
    previous=getattr(builtins,'_MOMA_MIRROR_NAMESPACE',{})
    if '_MOMA_MIRROR' in previous:previous['_MOMA_MIRROR'].close()
    script=Path(os.environ['MOMA_MIRROR_CLIENT'])
    namespace={'MOMA_MIRROR_URL':os.environ.get('MOMA_MIRROR_URL','http://127.0.0.1:8770')}
    exec(compile(script.read_text(encoding='utf-8'),str(script),'exec'),namespace)
    builtins._MOMA_MIRROR_NAMESPACE=namespace

_MOMA_AUTOSTART=asyncio.ensure_future(start())
