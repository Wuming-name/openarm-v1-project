"""Offline tests; no hardware imports or CAN writes."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from core import Reference,observation,load,read,save,effort,Physics,contract,DT

HERE=Path(__file__).absolute().parent

def config():
    c=read(HERE/"config.template.json")
    c["home"]=[0,0,0,.2,0,0,0]
    c["goal_q"]=[.1,0,0,.2,0,0,0]
    return c

def fixture(path):
    # Seven genuine MuJoCo hinge DOFs, zero gravity for algorithmic tests only.
    body=''
    for i in range(1,8):
        body+=f'<body pos=".08 0 0"><joint name="openarm_right_joint{i}" type="hinge" axis="0 0 1" range="-4 4" damping=".1" armature=".1"/><geom type="sphere" size=".02" mass=".1" contype="0" conaffinity="0"/>'
    body+='<site name="ee_site" pos=".08 0 0"/>'+'</body>'*7
    path.write_text('<mujoco><compiler angle="radian"/><option gravity="0 0 0"/><worldbody>'+body+'</worldbody></mujoco>')

class Tests(unittest.TestCase):
    def test_template_cannot_move(self):
        with self.assertRaises((ValueError,TypeError)):load(HERE/"config.template.json")

    def test_zero_j4_rejected(self):
        c=config();c["home"][3]=0
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'c.json';save(p,c)
            with self.assertRaises(ValueError):load(p)

    def test_action_rate_and_acceleration(self):
        c=config();r=Reference(c,c["home"]);last=r.q.copy();v=np.zeros(7)
        for k in range(500):
            a=np.ones(7)*(1 if k<200 else -1)
            q=r.step(a);nv=(q-last)/DT
            self.assertLessEqual(np.max(np.abs(nv)),c['reference_speed']+1e-10)
            self.assertLessEqual(np.max(np.abs(nv-v)),c['reference_accel']*DT+1e-10)
            last=q;v=nv

    def test_boundary_brakes(self):
        c=config();r=Reference(c,c['home'])
        for _ in range(2000):r.step(np.ones(7))
        self.assertLessEqual(np.max(r.q-np.array(c['home'])),c['reference_radius'])

    def test_nan_rejected(self):
        with self.assertRaises(ValueError):Reference(config(),[float('nan')]*7)

    def test_observation(self):
        c=config();q=np.array(c['home'])
        o=observation(c,q,np.zeros(7),q,np.zeros(7),np.ones(3),np.zeros(3))
        self.assertEqual(o.shape,(34,));self.assertEqual(o.dtype,np.float32)

    def test_contract_changes(self):
        c=config();d=copy.deepcopy(c);d['reference_speed']=.03
        self.assertNotEqual(contract(c),contract(d))

    def test_quantized_effort(self):
        c=config();q=np.array(c['home'])
        # 'Zero' packets quantize slightly negative; do not hide this in sim.
        tau=effort(c,q,q,np.zeros(7))
        self.assertTrue(np.all(tau<0));self.assertLess(np.max(np.abs(tau)),.1)

    def test_physics_and_env(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'robot.xml';fixture(p);c=config();c['model']=str(p)
            model=Physics(c);model.check_pose(c['home']);model.reset(np.array(c['home']))
            self.assertEqual(model.fk(c['home']).shape,(3,))
            model.advance(np.array(c['home']))
            self.assertTrue(np.all(np.isfinite(model.state()[0])))
            try:from reach_env import ReachEnv
            except ImportError:self.skipTest('Gymnasium absent; Physics completed')
            env=ReachEnv(c,False);o,_=env.reset(seed=1)
            self.assertEqual(o.shape,(34,))
            for _ in range(5):
                o,r,done,truncated,info=env.step(np.zeros(7))
                self.assertFalse(bool(info['fault']),info)

if __name__=='__main__':unittest.main()
