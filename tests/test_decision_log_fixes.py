"""Regressions from events00.csv: noise, wait bypass and misleading state."""
import unittest
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from seer_control.decision_log import DecisionJournal, decision_text
from seer_control.location import LocationTracker
from seer_control.model import MapModel, Simulator
from seer_control.route_planner import plan_stops
from seer_control.studio_core import MissionRunner
from seer_control.studio_ui import ConsoleAdapter


def scene(obstacle=False):
    return MapModel(dict(format='amr-console-map-v1',
        nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',3,0),('C',0,2),('D',3,2)]],
        edges=[['A','B'],['A','C'],['C','D'],['D','B']],walls=[],
        obstacles=[dict(id='BOX',x=1.,y=0.,radius=.3,map_fixed=True)] if obstacle else []))


class DecisionLogFixTests(unittest.TestCase):
    def test_empty_and_single_stop_previews_never_claim_success(self):
        m=scene();m.decision_journal=DecisionJournal()
        for _ in range(1000):
            plan_stops(m,[]);plan_stops(m,['A'])
        self.assertEqual(len(m.decision_journal.records),0)

    def test_preview_redraw_is_deduplicated_but_changed_policy_is_logged(self):
        m=scene();j=m.decision_journal=DecisionJournal()
        for _ in range(100):plan_stops(m,['A','B'])
        self.assertEqual(len(j.records),1)
        plan_stops(m,['A','B'],'time')
        self.assertEqual(len(j.records),2)
        self.assertEqual(j.records[-1]['evidence']['policy'],'time')

    def test_invalid_empty_preview_still_records_validation_errors(self):
        m=scene();j=m.decision_journal=DecisionJournal()
        result=plan_stops(m,[],excluded=['missing'])
        self.assertTrue(result['errors'])
        self.assertEqual(j.records[-1]['action'],'이동 작업 생성 차단')

    def test_wait_policies_hold_before_any_graph_or_local_reroute(self):
        for scenario in ('wait_recover','wait_avoid','wait_reroute'):
            for line_rejoin in (False,True):
                with self.subTest(scenario=scenario,line_rejoin=line_rejoin):
                    j=DecisionJournal();s=Simulator(scene(True),j)
                    s.obstacle_policy='auto';s.auto_scenarios['static']=scenario
                    s.auto_wait_s=5.;s.prefer_line_rejoin=line_rejoin
                    s.blocked_recovery.enabled=True;s.navigate('B')
                    initial_route=list(s.route);s.tick(.1)
                    self.assertIsNotNone(s._auto_block)
                    started=s._auto_block_at;first_plan=None
                    for _ in range(140):
                        s.tick(.1)
                        plans=[e for e in j.records if e['source'] in ('SIM.대체 경로','SIM.국소 우회') and '발견' in e['conclusion']]
                        if plans:
                            first_plan=s._avoid_time;break
                        if s._avoid_time-started<5.:
                            self.assertEqual(s.route,initial_route)
                            self.assertFalse(s._local_avoidance_active)
                            self.assertFalse(s.blocked_recovery.active)
                            self.assertAlmostEqual(s.state.x,0.)
                            self.assertAlmostEqual(s.state.y,0.)
                            self.assertEqual(s.state.speed,0.)
                    self.assertIsNotNone(first_plan)
                    self.assertGreaterEqual(first_plan-started,5.)
                    self.assertTrue(plans[-1]['evidence']['reason'])

    def test_wait_guards_direct_recovery_entry_points(self):
        s=Simulator(scene(True));s.obstacle_policy='auto';s.navigate('B')
        s._auto_block=('BOX','static','wait_recover');s._auto_block_at=0.;s._avoid_time=4.
        node=s.map.nodes['B'];radius=s.map.robot_model['radius']
        with patch('seer_control.avoidance.detour') as detour,patch('seer_control.avoidance.rejoin_detour') as rejoin:
            self.assertFalse(s._try_line_bypass(node,radius))
            self.assertFalse(s._try_local_detour(node,radius))
            self.assertFalse(s.try_recovery_detour())
            detour.assert_not_called();rejoin.assert_not_called()
        s._avoid_time=5.;self.assertFalse(s._auto_wait_active())

    def test_short_mission_timeout_cannot_override_configured_obstacle_wait(self):
        s=Simulator(scene(True));s.obstacle_policy='auto'
        s.auto_scenarios['static']='wait_recover';s.auto_wait_s=5.
        c=NS(sim=s,map=s.map,real=False,connected=True,sim_powered=True,
             studio_arm_safe=True,current_state=s.status,log=Mock(),studio_config={'arm':{}})
        r=c.studio_runner=MissionRunner(ConsoleAdapter(c))
        r.start([dict(type='Path Nav',goal='B')]);now=time.monotonic();r.tick(now,.1)
        r.actions[0]['timeout_s']=.5
        for i in range(1,41):
            s.tick(.1);r.tick(now+i*.1,.1)
            self.assertEqual(r.status,'RUNNING',r.error)
            self.assertFalse(s.skipped_goals)
            self.assertFalse(s._local_avoidance_active)
            self.assertEqual(s.state.x,0.)
        for i in range(41,701):
            s.tick(.1);r.tick(now+i*.1,.1)
            if not r.active:break
        self.assertEqual(r.status,'COMPLETED',r.error)
        self.assertEqual(s.state.last_node,'B')

    def test_a_new_blocker_cannot_reuse_the_previous_blocker_wait_time(self):
        j=DecisionJournal();s=Simulator(scene(True),j)
        s.obstacle_policy='auto';s.auto_scenarios['static']='wait_reroute'
        s.navigate('B');s._auto_block=('previous','static','wait_reroute')
        s._auto_block_at=-10.;s.tick(.1)
        event=next(e for e in j.records if e['source']=='SIM.전방 전체 경로 검사')
        self.assertFalse(event['evidence']['permit'])
        self.assertEqual(event['evidence']['elapsed_s'],0.)
        self.assertEqual(s.route,['B'])

    def test_current_speed_is_measured_before_velocity_decision(self):
        j=DecisionJournal(repeat_seconds=0);s=Simulator(scene(),j);s.navigate('B')
        observed=[]
        for _ in range(15):
            speed=s.state.speed;s.tick(.1)
            event=next(e for e in reversed(j.records) if e['source']=='SIM.속도 결정')
            self.assertAlmostEqual(event['situation']['speed'],speed)
            observed.append(event['situation']['speed'])
        self.assertTrue(any(v>.1 for v in observed))
        self.assertGreater(s.state.x,0.)

    def test_stop_and_heading_only_returns_publish_zero_actual_speed(self):
        s=Simulator(scene());s.navigate('B')
        for _ in range(10):s.tick(.1)
        self.assertGreater(s.state.speed,0.)
        s.state.stopped=True;origin=(s.state.x,s.state.y);s.tick(.1)
        self.assertEqual((s.state.x,s.state.y),origin)
        self.assertEqual(s.state.speed,0.)
        s.state.stopped=False;s._velocity=0.;s.state.speed=.2;s.state.theta=3.14
        s.tick(.1)
        self.assertEqual((s.state.x,s.state.y),origin)
        self.assertEqual(s.state.speed,0.)

    def test_full_route_reroute_carries_the_actual_blocking_reason(self):
        j=DecisionJournal();s=Simulator(scene(True),j)
        s.obstacle_policy='auto';s.auto_scenarios['static']='reroute';s.navigate('B')
        s.tick(.1)
        scan=next(e for e in j.records if e['source']=='SIM.전방 전체 경로 검사')
        applied=next(e for e in j.records if e['source']=='SIM.대체 경로')
        self.assertTrue(scan['evidence']['reason'])
        self.assertEqual(applied['evidence']['reason'],scan['evidence']['reason'])

    def test_repeat_count_belongs_to_the_correct_conclusion(self):
        j=DecisionJournal()
        for _ in range(4):j.emit('state',{}, {},'경로 장애물 없음','주행 계속')
        change=j.emit('state',{}, {},'대기 시간 미도달','정지 유지')
        self.assertEqual(change['repeated'],0)
        self.assertEqual(change['previous_summary']['repeated'],3)
        self.assertEqual(change['previous_summary']['conclusion'],'경로 장애물 없음')
        self.assertIn('직전 판단 종료',decision_text(change))

    def test_sim_location_is_not_labeled_as_controller_proximity_inference(self):
        j=DecisionJournal();tracker=LocationTracker();tracker.decision_journal=j
        tracker.update(scene().nodes,dict(x=0.,y=0.,last_node='A',localization=.99),context=1,now=0.,real=False)
        self.assertEqual(j.records[-1]['conclusion'],'SIM 시뮬레이터 위치 사용')
        e=j.emit('SIM.속도 결정',{}, {},'안전','주행')
        self.assertIn('[SIM/속도 결정]',decision_text(e))
        self.assertNotIn('SIM/SIM.',decision_text(e))


if __name__=='__main__':unittest.main()
