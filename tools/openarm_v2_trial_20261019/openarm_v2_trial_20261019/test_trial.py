"""Offline checks: no CAN, no model weights, no motor enabling."""
import tempfile
import time
import unittest
from pathlib import Path
import numpy as np
from sim_core import Reference, save
from trial import require_home, preload_duration, quintic_reference, prior_pass


def fixture():
    return {"home": [0, 0, 0, .4, 0, 0, 0], "reference_speed": .02,
            "reference_accel": .04, "reference_radius": .4,
            "q_min": [-1.396263, -.174532, -1.570796, 0, -1.570796, -.785398, -1.570796],
            "q_max": [3.490659, 3.316125, 1.570796, 2.443461, 1.570796, .785398, 1.570796]}


class TrialTests(unittest.TestCase):
    def test_arbitrary_pose_is_not_automatically_homed(self):
        c = fixture()
        q = np.array(c["home"])
        q[0] = .006
        with self.assertRaisesRegex(ValueError, "No automatic homing"):
            require_home(c, q)
        q[0] = .004
        require_home(c, q)

    def test_nonfinite_pose_rejected(self):
        with self.assertRaises(ValueError):
            require_home(fixture(), [float("nan")]*7)

    def test_preload_ramp_keeps_speed_acceleration_and_endpoints(self):
        c = fixture()
        a = np.array(c["home"])
        b = a + np.array([.005, -.08, 0, .11, -.02, .01, .003])
        duration = preload_duration(c, a, b)
        t = np.arange(0, duration+.06, .02)
        points = np.array([quintic_reference(a,b,duration,x) for x in t])
        velocity = np.diff(points, axis=0)/.02
        acceleration = np.diff(velocity, axis=0)/.02
        self.assertLessEqual(np.max(np.abs(velocity)), .02+1e-7)
        self.assertLessEqual(np.max(np.abs(acceleration)), .04+1e-7)
        np.testing.assert_allclose(points[0], a)
        np.testing.assert_allclose(points[-1], b)

    def test_brake_is_bounded_not_an_instant_stop(self):
        c = fixture()
        r = Reference(c, c["home"])
        for _ in range(50):
            r.step(np.ones(7))
        previous_v = r.v.copy()
        for _ in range(30):
            r.step(np.zeros(7))
            self.assertLessEqual(np.max(np.abs(r.v-previous_v)), .04*.02+1e-10)
            previous_v = r.v.copy()
        np.testing.assert_allclose(r.v, 0)

    def test_reference_boundary_guard_retained(self):
        c = fixture()
        r = Reference(c, c["home"])
        for _ in range(1500):
            r.step(np.ones(7))
        self.assertLessEqual(np.max(r.q-np.array(c["home"])), .4+1e-9)

    def test_stage_evidence_must_match_and_be_recent(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            proof = {"passed": True, "unix_time": time.time(), "key": {"policy":"A"}}
            save(root/"shadow_passed.json", proof)
            prior_pass(root, "shadow", {"policy":"A"})
            with self.assertRaises(ValueError):
                prior_pass(root, "shadow", {"policy":"B"})
            proof["unix_time"] -= 3601
            save(root/"shadow_passed.json", proof)
            with self.assertRaises(ValueError):
                prior_pass(root, "shadow", {"policy":"A"})
            proof.update(unix_time=time.time(), passed=False)
            save(root/"shadow_passed.json", proof)
            with self.assertRaises(ValueError):
                prior_pass(root, "shadow", {"policy":"A"})


if __name__ == "__main__":
    unittest.main()
