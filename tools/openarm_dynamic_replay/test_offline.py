"""Offline tests only. No sockets, no robot commands. python -m unittest -v test_offline"""
import csv
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
import numpy as np
from replay import Series, Replay, metrics, load_run
from run_dynamic import targets, screen


def make_rows(n=21):
    rows=[]
    for k in range(n):
        row={'stage':'record_hold','pose_id':'start','feedback_valid':'1','command_sent':'1',
             'rx_skew_ms':'0','transaction_ms':'1','max_kernel_age_ms':'1','time_s':str(k*.05)}
        for j in range(1,8):
            row.update({f'rx{j}':str(k*.05),f'q{j}':'0',f'dq{j}':'0',f'tau{j}':'0',f'status{j}':'1'})
        rows.append(row)
    return rows


class DataTests(unittest.TestCase):
    def test_causal_force(self):
        rows=make_rows()
        for k,r in enumerate(rows):r['tau1']=str(k)
        s=Series(rows)
        self.assertEqual(s.force(.049)[0],0)
        self.assertEqual(s.force(.05)[0],1)
        self.assertAlmostEqual(s.state(.025)[0][0],0)

    def test_per_joint_time(self):
        rows=make_rows()
        for k,r in enumerate(rows):
            for j in range(1,8):
                t=k*.05+j*.001;r[f'rx{j}']=str(t);r[f'q{j}']=str(t*2)
        s=Series(rows)
        np.testing.assert_allclose(s.state(.1)[0],np.full(7,.2))
        self.assertAlmostEqual(s.start,.007)

    def test_no_bridging_gap(self):
        rows=make_rows();del rows[5]
        with self.assertRaisesRegex(ValueError,'gap'):Series(rows)

    def test_no_sorting(self):
        rows=make_rows();rows[2],rows[3]=rows[3],rows[2]
        with self.assertRaisesRegex(ValueError,'Nonmonotonic'):Series(rows)

    def test_disabled_or_nan_rejected(self):
        rows=make_rows();rows[3]['status2']='0'
        with self.assertRaises(ValueError):Series(rows)
        rows=make_rows();rows[3]['tau2']='nan'
        with self.assertRaises(ValueError):Series(rows)

    def test_metrics_units(self):
        result=metrics(np.zeros((3,7)),np.ones((3,7))*math.pi/180)
        self.assertAlmostEqual(result[0]['rmse_deg'],1)
        self.assertAlmostEqual(result[0]['bias_deg'],1)

    def test_partial_requires_explicit_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)
            (path/'metadata.json').write_text(json.dumps({'version':'openarm-dynamic-replay-v1','completed':False}))
            rows=make_rows()
            with (path/'samples.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            with self.assertRaisesRegex(ValueError,'incomplete'):load_run(path)
            self.assertEqual(len(load_run(path,True)[1].rows),21)

    def test_trajectory_single_axis_and_returns(self):
        base=[0,.2,0,.5,0,0,0]
        p=targets(base,[2,4,7],5,2,'both')
        self.assertEqual(len(p),24)
        self.assertEqual(p[-1]['q_motor_rad'],base)
        for x in p:self.assertLessEqual(sum(abs(a-b)>1e-9 for a,b in zip(base,x['q_motor_rad'])),1)

    def test_screen_rejections(self):
        cfg={'q_min_rad':[-1]*7,'q_max_rad':[1]*7,'torque_alarm_nm':[1]*7}
        base=[0]*7;p=targets(base,[2],5,1,'both')
        class Model:
            contact=0;gravity=.1
            def evaluate(self,q):return [self.gravity]*7,[0]*7,self.contact
        model=Model();self.assertGreater(screen(model,cfg,base,p)[0],1)
        model.contact=1
        with self.assertRaisesRegex(ValueError,'contact'):screen(model,cfg,base,p)
        model.contact=0;model.gravity=.7
        with self.assertRaisesRegex(ValueError,'gravity'):screen(model,cfg,base,p)


class IntegratorTests(unittest.TestCase):
    def engine(self):
        e=object.__new__(Replay)
        e.sign=np.ones(7);e.offset=np.zeros(7);e.scale=np.ones(7);e.bias=np.zeros(7)
        e.qadr=e.dadr=np.arange(7);e.ids=list(range(7));e.dt=.003
        e.model=SimpleNamespace(opt=SimpleNamespace(timestep=.003),jnt_limited=[False]*7)
        e.data=SimpleNamespace(qpos=np.zeros(7),qvel=np.zeros(7),qfrc_applied=np.zeros(7),xfrc_applied=np.zeros((1,6)),ctrl=np.zeros(7),time=0,ncon=0,warning=[])
        class ExactUnitInertia:
            @staticmethod
            def mj_resetData(m,d):d.qpos[:]=0;d.qvel[:]=0;d.time=0
            @staticmethod
            def mj_forward(m,d):pass
            @staticmethod
            def mj_step(m,d):
                h=m.opt.timestep;d.qpos[:]+=d.qvel*h+.5*d.qfrc_applied*h*h;d.qvel[:]+=d.qfrc_applied*h;d.time+=h
        e.mj=ExactUnitInertia
        return e

    def test_force_event_boundary_and_free_prediction(self):
        rows=make_rows()
        # Unit inertia, force changes at t=.25: q(1)=.5*(.75)^2.
        for k,r in enumerate(rows):r['tau1']='1' if k>=5 else '0'
        e=self.engine();pred,result=e.simulate(Series(rows),0,1,max_error_deg=180)
        self.assertTrue(result['completed']);self.assertAlmostEqual(pred[-1]['q_sim'][0],.28125,places=7)
        self.assertEqual(pred[-1]['q_real'][0],0) # Simulation is not overwritten by measurement.

    def test_window_initial_state(self):
        rows=make_rows()
        for r in rows:r['q1']='.2';r['dq1']='.1'
        pred,result=self.engine().simulate(Series(rows),.3,.8,max_error_deg=180)
        self.assertTrue(result['completed']);self.assertAlmostEqual(pred[-1]['q_sim'][0],.25,places=7)

    def test_divergence_reported(self):
        rows=make_rows()
        for r in rows:r['tau1']='1'
        _,result=self.engine().simulate(Series(rows),0,1,max_error_deg=1)
        self.assertFalse(result['completed']);self.assertEqual(result['stop_reason'],'prediction_diverged')


@unittest.skipUnless(importlib.util.find_spec('mujoco'),'MuJoCo not installed')
class MuJoCoTests(unittest.TestCase):
    def test_real_plan_from_fresh_snapshot(self):
        from run_dynamic import prepare
        import mujoco
        bodies=''.join(f'<body pos="{i} 0 0"><joint name="openarm_right_joint{i+1}" type="hinge" axis="0 0 1" limited="true" range="-180 180"/><inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/></body>' for i in range(7))
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);model=root/'model.xml';model.write_text(f'<mujoco><worldbody>{bodies}</worldbody></mujoco>')
            base=[0,.3,0,.5,0,0,0]
            (root/'metadata.json').write_text(json.dumps({'completed':True,'mode':'diagnose'}))
            (root/'snapshot.json').write_text(json.dumps({'schema':'openarm-snapshot-v1','interface':'can0','run_time_ns':time.time_ns(),'q_motor_rad':base}))
            rows=make_rows()
            for r in rows:
                r['stage']='diagnose'
                for j in range(1,8):r[f'q{j}']=str(base[j-1]);r[f'status{j}']='0'
            with (root/'samples.csv').open('w',newline='') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            args=SimpleNamespace(torque_profile='proximal_5_3',speed=.02,amplitude_deg=5,joints=[2],repeats=1,directions='both')
            plan=prepare(root/'snapshot.json',model,root/'prepared',args)
            self.assertEqual(len(plan['poses']),4)
            self.assertFalse(plan['config']['trim_j2'])
            self.assertEqual(plan['mujoco_version'],mujoco.__version__)
            self.assertTrue((root/'prepared/model.mjb').exists())

    def test_real_engine_actuators_disabled_and_mapping(self):
        bodies=''.join(f'<body pos="{i} 0 0"><joint name="j{i}" type="hinge" axis="0 0 1"/><inertial pos="0 0 0" mass="1" diaginertia=".1 .1 .1"/></body>' for i in range(7))
        actuators=''.join(f'<position joint="j{i}" kp="100"/>' for i in range(7))
        xml=f'<mujoco><option gravity="0 0 -9.81"/><worldbody>{bodies}</worldbody><actuator>{actuators}</actuator></mujoco>'
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'model.xml';path.write_text(xml)
            cfg={'joint_names':[f'j{i}' for i in range(7)],'joint_sign':[-1,1,1,1,1,1,1],'zero_offset_rad':[.3,0,0,0,0,0,0]}
            engine=Replay(path,cfg,.001)
            rows=make_rows()
            for r in rows:r['tau1']='.01'
            pred,result=engine.simulate(Series(rows),0,1,max_error_deg=180)
            self.assertTrue(result['completed'])
            self.assertAlmostEqual(pred[-1]['q_sim'][0],.05,delta=.0002)
            self.assertTrue(np.allclose(engine.data.qfrc_actuator,0))
            self.assertTrue(np.allclose(engine.model.body_gravcomp,0))


if __name__=='__main__':unittest.main()
