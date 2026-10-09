import copy
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from seer_control.decision_log import DecisionJournal, decide, decision_text
from seer_control.mission_report import MissionReport
from seer_control.model import MapModel, Simulator
from seer_control.obstacle_tracking import ObstacleTracker
from seer_control.real_obstacle import RealObstacleRecovery
from seer_control.studio_core import MissionRunner, EditHistory
from seer_control.studio_ui import ConsoleAdapter


def model(obstacle=True):
    return MapModel(dict(format='amr-console-map-v1',
        nodes=[dict(id=k,x=x,y=y) for k,x,y in [('A',0,0),('B',3,0),('C',0,2),('D',3,2)]],
        edges=[['A','B'],['A','C'],['C','D'],['D','B']],walls=[],
        obstacles=[dict(id='BOX',x=1.5,y=0,radius=.3,map_fixed=False)] if obstacle else []))


class DecisionLogTests(unittest.TestCase):
    def test_sim_arm_in_real_host_is_identified_as_simulation(self):
        j=DecisionJournal();j.bind('REAL')
        e=j.emit('SIM.팔 연속 충돌 검사',{}, {},'안전','SIM 관절 이동')
        self.assertEqual((e['backend'],e['host_backend']),('SIM','REAL'))

    def test_arm_collision_logs_contact_and_preserves_rejection(self):
        from seer_control.arm_physics import ArmPhysics
        p=object.__new__(ArmPhysics);p.decision_journal=DecisionJournal()
        p.objects=[];p.held=None;p.config={'stop_on_collision':True};p.events=[]
        p.contacts=lambda q:([('arm','wall')],[])
        with self.assertRaisesRegex(ValueError,'이동 차단'):p.check_motion([0]*6,[.1]*6)
        e=p.decision_journal.records[-1]
        self.assertEqual(e['evidence']['contacts'],[['arm','wall']])
        self.assertEqual(e['action'],'이동 차단')

    def test_threaded_files_preserve_order_and_shutdown_flush(self):
        with tempfile.TemporaryDirectory() as folder:
            j=DecisionJournal(folder)
            def producer():
                for i in range(10):
                    j.emit('검증',dict(pose=[i,0]),dict(threshold=.2),str(i),'요청',force=True)
            workers=[threading.Thread(target=producer) for _ in range(3)]
            for w in workers:w.start()
            for w in workers:w.join()
            j.close()
            rows=[json.loads(line) for line in (Path(folder)/'decisions.jsonl').read_text(encoding='utf-8').splitlines()]
            self.assertEqual([r['id'] for r in rows],list(range(1,31)))
            self.assertEqual(len({r['session'] for r in rows}),1)
            text=(Path(folder)/'decisions.txt').read_text(encoding='utf-8')
            for field in ('상황:','근거:','결론:','동작:'):self.assertIn(field,text)

    def test_dedup_keeps_changes_and_repeated_observation_summary(self):
        j=DecisionJournal(repeat_seconds=5)
        with patch('seer_control.decision_log.time.monotonic',side_effect=[0,1,2,6,7]):
            for i in range(4):j.emit('wait',{'elapsed':i},{'limit':5},'대기','정지 유지')
            j.emit('wait',{'elapsed':7},{'limit':5},'대기 종료','우회 탐색')
        self.assertEqual(len(j.records),3)
        self.assertEqual(j.records[1]['repeated'],3)
        self.assertEqual(j.records[-1]['action'],'우회 탐색')

    def test_context_and_report_dont_change_success_counts(self):
        j=DecisionJournal();report=MissionReport(2);j.bind('REAL',report,2,3)
        j.emit('DI',{'channel':1},{'observed':1,'expected':1},'일치','5단계로 이동')
        report.add(1,2,'성공');d=report.data()
        self.assertEqual(d['success'],1)
        self.assertEqual(d['decisions'][0]['backend'],'REAL')
        self.assertEqual((d['decisions'][0]['loop'],d['decisions'][0]['step']),(2,3))
        self.assertIn('근거:',report.text())
        self.assertIs(copy.deepcopy(j),j)

    def test_log_rotation_and_secrets_redaction(self):
        with tempfile.TemporaryDirectory() as folder:
            j=DecisionJournal(folder,max_bytes=1000,backups=2)
            for i in range(12):j.emit('rotation',{'password':'hidden'},{'access_token':'sensitive'},str(i),'계속',force=True)
            j.close()
            files=list(Path(folder).glob('decisions.jsonl*'))
            self.assertGreater(len(files),1);self.assertLessEqual(len(files),3)
            combined=''.join(p.read_text(encoding='utf-8') for p in files)
            self.assertNotIn('hidden',combined);self.assertNotIn('sensitive',combined)
            self.assertIn('[REDACTED]',combined)

    def test_write_failure_is_exposed_without_changing_control_result(self):
        with tempfile.TemporaryDirectory() as folder:
            j=DecisionJournal(folder)
            with patch.object(j._handlers[0],'flush',side_effect=OSError('disk full')):
                result=j.emit('io',{}, {},'허용','이동 요청')
                self.assertIsNotNone(result);self.assertFalse(j.flush())
            self.assertIn('disk full',j.error);self.assertGreater(j.failed_writes,0)
            j.close()

    def test_static_obstacle_rerouting_records_reason_plan_and_arrival(self):
        j=DecisionJournal();s=Simulator(model(),j);s.obstacle_policy='auto'
        s.auto_scenarios['static']='wait_reroute';s.auto_static_s=.5;s.auto_wait_s=.5
        s.navigate('B')
        for _ in range(800):
            s.tick(.1)
            if not s.route:break
        self.assertEqual(s.state.last_node,'B')
        sources={r['source'] for r in j.records}
        self.assertTrue({'SIM.경로 선택','SIM.관측 분류','SIM.전방 전체 경로 검사','SIM.대체 경로','SIM.도착'}<=sources)
        plan=next(r for r in j.records if r['source']=='SIM.대체 경로')['evidence']['plan']
        self.assertIn('selection_policy',plan);self.assertGreater(plan['candidate_count'],0)
        self.assertEqual(EditHistory().capture(s.map)['decision_journal'],j)

    def test_dynamic_classification_explains_observed_speed_threshold(self):
        j=DecisionJournal();tracker=ObstacleTracker();tracker.decision_journal=j
        o={'x':0.,'y':0.,'map_fixed':False,'id':'moving'}
        for _ in range(15):o['x']+=.03;tracker.update([o],.1)
        event=next(e for e in j.records if e['conclusion']=='분류: dynamic')
        self.assertGreaterEqual(event['evidence']['observed_speed'],event['evidence']['moving_threshold'])
        self.assertGreaterEqual(event['evidence']['displacement_m'],.025)

    def test_safety_rejection_explains_why_no_navigation_started(self):
        j=DecisionJournal();s=Simulator(model(False),j);s.state.stopped=True
        with self.assertRaises(ValueError):s.navigate('B')
        self.assertFalse(s.route)
        event=j.records[-1]
        self.assertIn('정지 해제',event['evidence']['reason'])
        self.assertEqual(event['conclusion'],'검증/안전 조건 불충족')

    def real_console(self,j):
        now=time.monotonic();j.bind('REAL')
        c=NS(decision_journal=j,studio_config={'real_obstacle_recovery':True},
            sim=NS(obstacle_policy='reroute',reroute_wait_s=1,auto_wait_s=2,reroute_attempt_limit=1,skipped_goals={}),
            studio_runner=NS(report=None),log=Mock(),send_command=Mock(),last_state=now,
            last_laser_rx=now,real_laser_points=[(1,1)],real_command_acks={},
            map=NS(nodes={'B':{}}),_station_nav_payload=lambda node:{'id':'B'})
        return c,now

    def test_real_cancel_wait_explains_missing_ack_then_fresh_stop(self):
        j=DecisionJournal();c,now=self.real_console(j);r=RealObstacleRecovery(c,'B')
        state=dict(blocked=True,speed=0,task='WAITING',task_status=2)
        r.poll(state,now);r.poll(state,now+2);r.poll(state,now+3)
        event=j.records[-1]
        self.assertEqual(event['source'],'REAL.취소/정지 확인')
        self.assertEqual(event['evidence']['cancel_ack_at'],0)
        self.assertEqual(c.send_command.call_count,1)
        c.real_command_acks['cancel']=(now+2.1,{'ret_code':0});c.last_state=now+2.2
        r.poll(dict(state,task='CANCELED',task_status=6),now+3.1)
        self.assertEqual(c.send_command.call_args.args[0],'navigate_free')
        self.assertTrue(any(e['conclusion']=='취소 ACK 이후 최신 정지 상태 확인' for e in j.records))

    def test_stale_real_lidar_logs_why_reroute_is_withheld(self):
        j=DecisionJournal();c,now=self.real_console(j);c.last_laser_rx=now-10
        r=RealObstacleRecovery(c,'B');state=dict(blocked=True,speed=0,task='WAITING')
        r.poll(state,now);r.poll(state,now+3)
        c.send_command.assert_not_called()
        self.assertEqual(j.records[-1]['source'],'REAL.재탐색 센서 검증')
        self.assertGreater(j.records[-1]['evidence']['lidar_age_s'],2)

    def test_di_branch_log_contains_actual_input_expected_and_destination(self):
        j=DecisionJournal();c=NS(decision_journal=j,_studio_io_value=lambda *args:True)
        adapter=ConsoleAdapter(c)
        self.assertEqual(adapter.branch_target(dict(channel=3,value=True,target=5)),5)
        self.assertEqual(j.records[-1]['evidence'],dict(channel=3,observed=True,expected=True,target=5))
        self.assertIn('5',j.records[-1]['action'])

    def test_timeout_and_skipped_followups_are_audited(self):
        j=DecisionJournal()
        class Adapter:
            decision_journal=j
            def __init__(self):self.decision_journal=j
            def begin(self,a):pass
            def poll(self,a,dt,elapsed):return dict(skip=True,goal='B',reason='통로 없음')
            def cancel(self):pass
        runner=MissionRunner(Adapter());runner.start([dict(type='Path Nav',goal='B'),dict(type='Arm Action',operation='work')])
        runner.tick(1,.1)
        self.assertEqual(runner.status,'COMPLETED')
        sources={e['source'] for e in runner.report.decisions}
        self.assertIn('미션.후속 작업 생략',sources)
        runner.start([dict(type='Wait',duration_s=10,timeout_s=.1)])
        runner.adapter.poll=lambda *args:False
        runner.tick(2,.1);runner.tick(3,.1)
        self.assertEqual(runner.status,'FAILED')
        self.assertTrue(any(e['source']=='미션.제한 시간' for e in runner.report.decisions))


if __name__=='__main__':unittest.main()
