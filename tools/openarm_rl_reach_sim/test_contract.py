"""Run after prepare.py: python test_contract.py --config config.sim.json."""
import argparse
import copy
import unittest
import numpy as np
from core import load, Reference, effort, vec, contract
from reach_env import ReachEnv

class ContractTests(unittest.TestCase):
    def test_reference_acceleration(self):
        r=Reference(CONFIG,CONFIG['home']);before=r.q.copy()
        r.step(np.ones(7))
        np.testing.assert_allclose(r.v,CONFIG['reference_accel']*.02)
        np.testing.assert_allclose(r.q-before,r.v*.02)

    def test_zero_action_keeps_existing_position_reference(self):
        r=Reference(CONFIG,CONFIG['home']);r.step(np.ones(7));r.step(np.zeros(7))
        stopped=r.q.copy();r.step(np.zeros(7))
        np.testing.assert_allclose(r.q,stopped)
        self.assertGreater(float(np.max(np.abs(r.q-vec(CONFIG['home'])))),0)

    def test_contract_changes_with_control(self):
        c=copy.deepcopy(CONFIG);c['reference_speed']*=.9
        self.assertNotEqual(contract(c),contract(CONFIG))

    def test_preload_holds_and_observation_is_valid(self):
        env=ReachEnv(CONFIG,False);obs,info=env.reset(seed=1)
        q,dq=env.p.state()
        self.assertTrue(env.observation_space.contains(obs))
        self.assertLess(np.max(np.abs(q-vec(CONFIG['home']))),.005)
        self.assertLess(np.max(np.abs(dq)),CONFIG['success_speed'])
        self.assertGreater(np.max(np.abs(env.ref.q-q)),.01)
        self.assertTrue(np.all(np.abs(effort(CONFIG,env.ref.q,q,dq))<vec(CONFIG['torque_alarm'])))
        self.assertAlmostEqual(env.distance,np.linalg.norm(env.goal-env.p.fk(q)))

    def test_zero_action_remains_held(self):
        env=ReachEnv(CONFIG,False);env.reset(seed=1)
        for _ in range(10):
            _,_,terminated,_,info=env.step(np.zeros(7))
            self.assertFalse(terminated,info)
            self.assertEqual(info['fault'],'')

    def test_randomized_reset_reproducible(self):
        env=ReachEnv(CONFIG,True)
        a,_=env.reset(seed=17);qa=env.p.state()[0].copy()
        b,_=env.reset(seed=17);qb=env.p.state()[0].copy()
        np.testing.assert_allclose(a,b);np.testing.assert_allclose(qa,qb)

    def test_reference_envelope_still_enforced(self):
        env=ReachEnv(CONFIG,False)
        q=vec(CONFIG['home']);outside=q.copy();outside[0]+=CONFIG['reference_radius']+.01
        with self.assertRaisesRegex(ValueError,'Reference outside'):
            env.p.check_reference_path(q,outside)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',default='config.sim.json')
    args=p.parse_args();CONFIG=load(args.config)
    unittest.main(argv=['test_contract.py'])
