"""Editor limits; these are application input limits, not firmware specifications."""
import math
LIMITS={
 'maxspeed':(0,3,'m/s',False), 'maxacc':(0,5,'m/s²',False), 'maxdec':(0,5,'m/s²',False),
 'maxrot':(0,180,'deg/s',False), 'maxrotacc':(0,360,'deg/s²',False), 'maxrotdec':(0,360,'deg/s²',False),
 'obsDecDist':(0,10,'m',False), 'obsStopDist':(0,10,'m',False), 'obsExpansion':(0,5,'m',False),
 'weight':(0,1000,'',False), 'collisionPointThreshold':(1,10000,'점 · 정수',True)}
def parse_properties(values):
 result={}
 for name,text in values.items():
  text=str(text).strip()
  if not text:continue
  lo,hi,unit,integer=LIMITS[name]
  try:value=float(text)
  except ValueError:raise ValueError(f'{name}: 숫자를 입력하세요. 허용 범위 {lo}~{hi} {unit}')
  if not math.isfinite(value) or not lo<=value<=hi or integer and not value.is_integer():raise ValueError(f'{name}: 허용 범위 {lo}~{hi} {unit}')
  result[name]=int(value) if integer else value
 if 'obsDecDist' in result and 'obsStopDist' in result and result['obsDecDist']<result['obsStopDist']:raise ValueError('obsDecDist 감속 시작거리는 obsStopDist 정지거리 이상이어야 합니다.')
 return result
