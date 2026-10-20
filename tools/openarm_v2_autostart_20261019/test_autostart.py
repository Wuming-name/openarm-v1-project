"""Offline tests; do not open CAN or enable motors."""
import argparse
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from sim_core import load, read, vec
from startup_control import (AutoHome, RateReference, parking_kind, reference_bounds,
                             HOME_TOL, TRIM_LIMIT)

CONFIG=None


def fixture():
    return {"home":[0,0,0,.4,0,0,0],"reference_speed":.02,"reference_accel":.04,
            "reference_radius":.4,"actual_radius":.45,"success_speed":.02,
            "q_min":[-1.396263,-.174532,-1.570796,0,-1.570796,-.785398,-1.570796],
            "q_max":[3.490659,3.316125,1.570796,2.443461,1.570796,.785398,1.570796]}


class PreparationTests(unittest.TestCase):
    def test_observed_small_negative_is_only_a_parking_reading(self):
        c=fixture()
        q=np.zeros(7);q[3]=-.0020981155108
        self.assertEqual(parking_kind(c,q),"near_zero")
        self.assertEqual(reference_bounds(c,True)[0][3],0.)
        self.assertEqual(reference_bounds(c,False)[0][3],.04)
        q[3]=-.006
        with self.assertRaises(ValueError):
            parking_kind(c,q)

    def test_arbitrary_pose_or_nonfinite_input_refused(self):
        c=fixture()
        for q in ([0,0,0,.2,0,0,0],[.04,0,0,0,0,0,0],[float("nan")]*7):
            with self.assertRaises(ValueError):
                parking_kind(c,q)
        self.assertEqual(parking_kind(c,c["home"]),"near_A")

    def test_reference_ramp_and_trim_preserve_speed_acceleration(self):
        c=fixture()
        start=np.array([0,0,0,0,0,0,0.])
        final=vec(c["home"])+np.array([.038,0,0,.068,0,.014,.02])
        controller=AutoHome(c,start,final)
        last_q=controller.ref.q.copy();last_v=np.zeros(7)
        for _ in range(int(controller.duration/.02)+15):
            q,_=controller.step(c["home"],np.zeros(7))
            v=(q-last_q)/.02
            self.assertLessEqual(np.max(np.abs(v)),.02+1e-8)
            self.assertLessEqual(np.max(np.abs(v-last_v)),.04*.02+1e-7)
            last_q,last_v=q.copy(),v.copy()

    def test_ready_requires_50_consecutive_settled_samples(self):
        c=fixture()
        controller=AutoHome(c,c["home"],c["home"])
        for _ in range(99):
            _,ready=controller.step(c["home"],np.zeros(7))
            self.assertFalse(ready)
        controller.step(c["home"],np.zeros(7))
        for _ in range(49):
            _,ready=controller.step(c["home"],np.zeros(7))
            self.assertFalse(ready)
        _,ready=controller.step(c["home"],np.zeros(7))
        self.assertTrue(ready)
        q=vec(c["home"]);q[0]+=HOME_TOL+.001
        _,ready=controller.step(q,np.zeros(7))
        self.assertFalse(ready)

    def test_unreachable_actual_A_times_out_without_unbounded_trim(self):
        c=fixture()
        controller=AutoHome(c,c["home"],c["home"])
        actual=vec(c["home"]);actual[0]-=.02
        stopped=False
        for _ in range(2000):
            try:
                target,ready=controller.step(actual,np.zeros(7))
                self.assertFalse(ready)
                self.assertLessEqual(np.max(np.abs(target-vec(c["home"]))),TRIM_LIMIT+1e-8)
            except ValueError as exc:
                self.assertIn("did not settle",str(exc))
                stopped=True
                break
        self.assertTrue(stopped)

    def test_brake_at_lower_startup_boundary_never_commands_negative_J4(self):
        c=fixture()
        r=RateReference(c,np.zeros(7))
        for _ in range(1000):
            r.step(-np.ones(7))
            self.assertGreaterEqual(r.q[3],0.)
        for _ in range(30):
            previous=r.v.copy()
            r.step(np.zeros(7))
            self.assertLessEqual(np.max(np.abs(r.v-previous)),.04*.02+1e-10)

    def test_user_model_automatic_preparation(self):
        if CONFIG is None:
            self.skipTest("Pass --config for user-model simulations")
        from simulate_startup import simulate
        cases=[(np.array([0,0,0,-.0020981155108,0,0,0]),None),
               (np.zeros(7),17),
               (vec(CONFIG["home"])+np.array([.01,-.01,.01,-.03,-.01,.01,-.01]),29)]
        results=[]
        for q,seed in cases:
            with self.subTest(seed=seed):
                result,_=simulate(CONFIG,q,seed)
                self.assertTrue(result["ready"])
                self.assertLessEqual(result["final_error_max_rad"],HOME_TOL)
                results.append(result)
        print("USER_MODEL_RESULTS="+json.dumps(results,ensure_ascii=False))


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--config")
    args=parser.parse_args()
    if args.config:
        CONFIG=load(args.config)
    # Keep temporary compiled-model files inside the test workspace.
    scratch=Path(__file__).resolve().parent/".test_tmp"
    scratch.mkdir(exist_ok=True)
    tempfile.tempdir=str(scratch)
    unittest.main(argv=["test_autostart.py"])
