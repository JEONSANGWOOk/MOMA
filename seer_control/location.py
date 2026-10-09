"""Display-only node tracking. Never substitutes nearest node for arrival."""
from .decision_log import decide
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def field(sources, *names):
    for source in sources:
        if isinstance(source, dict):
            for name in names:
                value = source.get(name)
                if isinstance(value, str) and value.strip():
                    return value.strip()
    return ''


class LocationTracker:
    def __init__(self):
        self.context = None
        self.last = ''
        self.candidate = ''
        self.since = 0.
        self.latched = ''

    def update(self, nodes, state, *, context, now, valid=True, real=False, sources=(), route=()):
        if context != self.context:
            self.context = context
            self.last = self.candidate = self.latched = ''
            self.since = now
        result = dict(last=self.last, next='', goal='', distance=None, at='', estimated=False, valid=valid, next_is_goal=False)
        if not valid:
            decide(self,'위치.표시 판단',state,dict(valid=False),'유효 상태 수신 없음','노드 추정 표시 보류')
            self.candidate = ''
            return result
        x, y = state.get('x'), state.get('y')
        pose_valid = finite(x) and finite(y)
        confidence = state.get('localization')
        if real and finite(confidence) and confidence < .5:
            pose_valid = False
        all_sources = [state, *sources]
        goal = field(all_sources, 'target_id', 'target')
        if real:
            reported = field(all_sources, 'last_station', 'previous_station', 'last_node')
            if reported:
                self.last = reported
            # Without a controller arrival field, require sustained proximity;
            # hysteresis prevents switching back and forth between nearby nodes.
            if pose_valid:
                if self.latched in nodes and math.hypot(x-nodes[self.latched]['x'], y-nodes[self.latched]['y']) > .30:
                    self.latched = ''
                nearby = min(nodes, key=lambda k:math.hypot(x-nodes[k]['x'], y-nodes[k]['y'])) if nodes else ''
                current = field(all_sources, 'current_station', 'current_point', 'current_id')
                if current in nodes:
                    nearby = current
                distance = math.hypot(x-nodes[nearby]['x'], y-nodes[nearby]['y']) if nearby else math.inf
                if distance <= .18 and (not self.latched or nearby == self.latched):
                    if nearby != self.candidate:
                        self.candidate, self.since = nearby, now
                    if now-self.since >= .30:
                        self.latched = nearby
                        if not reported:
                            self.last = nearby
                        result['at'] = nearby
                else:
                    self.candidate = ''
            remaining = state.get('unfinished_path', [])
            if isinstance(remaining, list):
                result['next'] = next((k for k in remaining if isinstance(k,str) and k in nodes and k != result['at']), '')
            if not result['next']:
                result['next'] = goal if goal and goal != result['at'] else ''
                result['next_is_goal'] = bool(result['next'])
            result['estimated'] = not bool(reported)
        else:
            reported = field([state], 'last_node')
            self.last = reported if reported in nodes else ''
            result['next'] = next((k for k in route if k != self.last), '')
            if pose_valid and self.last in nodes and math.hypot(x-nodes[self.last]['x'], y-nodes[self.last]['y']) <= .05:
                result['at'] = self.last
        result.update(last=self.last, goal=goal)
        conclusion=('SIM 시뮬레이터 위치 사용' if not real else
                    '제어기 보고 위치 사용' if not result['estimated'] else
                    '근접 지속 관측으로 표시 추정' if pose_valid else '좌표/신뢰도 조건 미충족')
        decide(self,'위치.표시 판단',state,dict(result=result,confidence=confidence,min_real_confidence=.5,pose_valid=pose_valid,near_distance_m=.18,proximity_hold_s=.30,release_distance_m=.30),conclusion,'화면 노드 표시 갱신 · 주행 완료 판단에는 사용하지 않음',identity=(result['last'],result['at'],result['next'],result['estimated'],pose_valid))
        target = result['next']
        if pose_valid and target in nodes:
            result['distance'] = math.hypot(x-nodes[target]['x'], y-nodes[target]['y'])
        return result
