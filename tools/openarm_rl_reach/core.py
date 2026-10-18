"""Shared simulation/deployment contract. No CAN access in this module."""
import hashlib
import json
from pathlib import Path
import numpy as np

DT = .02
DECIMATION = 5
SCHEMA = "openarm-reach-v1"
LOW = np.array([-1.396263, -.174532, -1.570796, 0, -1.570796, -.785398, -1.570796])
HIGH = np.array([3.490659, 3.316125, 1.570796, 2.443461, 1.570796, .785398, 1.570796])
KP = np.array([35, 35, 20, 20, 6, 6, 6.])
KD = np.array([4, 4, 2, 2, .8, .8, .8])
CAP = np.array([5, 5, 3, 3, .7, .7, .7])

def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))

def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+"\n", encoding="utf-8")

def vec(value, n=7):
    v = np.asarray(value, dtype=float)
    if v.shape != (n,) or not np.all(np.isfinite(v)):
        raise ValueError(f"Expected {n} finite numbers")
    return v

def load(path, poses=True):
    c = read(path)
    if c.get("schema") != SCHEMA:
        raise ValueError("Unknown config schema")
    for k in ("q_min", "q_max", "kp", "kd", "torque_alarm", "speed_alarm", "sign", "offset"):
        vec(c[k])
    lo, hi = vec(c["q_min"]), vec(c["q_max"])
    if np.any(lo < LOW-1e-8) or np.any(hi > HIGH+1e-8) or np.any(lo >= hi):
        raise ValueError("Joint limits exceed inherited reviewed envelope")
    if not np.array_equal(c["kp"], KP) or not np.array_equal(c["kd"], KD):
        raise ValueError("v1 retains previously verified KP/KD")
    if np.any(vec(c["torque_alarm"]) <= 0) or np.any(vec(c["torque_alarm"]) > CAP):
        raise ValueError("Torque alarms exceed inherited profile")
    if np.any(vec(c["speed_alarm"]) <= c["reference_speed"]) or np.any(vec(c["speed_alarm"]) > .5):
        raise ValueError("Invalid velocity alarms")
    for k, a, b in [("reference_speed", .005, .05), ("reference_accel", .005, .08),
                    ("tracking_alarm", .02, .16), ("temperature_alarm", 25, 55),
                    ("home_tolerance", .005, .04), ("home_capture_radius", .01, .10),
                    ("episode_seconds", 5, 120), ("success_distance", .005, .03),
                    ("success_speed", .005, .04), ("reset_jitter", 0, .01),
                    ("reference_radius", .02, .35), ("actual_radius", .02, .40)]:
        if not np.isfinite(c[k]) or not a <= c[k] <= b:
            raise ValueError(f"Invalid {k}: expected {a}..{b}")
    if c["actual_radius"] < c["reference_radius"]:
        raise ValueError("actual_radius must cover reference_radius")
    if len(c["joint_names"]) != 7 or len(set(c["joint_names"])) != 7:
        raise ValueError("Need seven unique joint names")
    if not np.all(np.isin(c["sign"], [-1, 1])):
        raise ValueError("Joint sign must be +/-1")
    if poses:
        home, goal = vec(c["home"]), vec(c["goal_q"])
        for q in (home, goal):
            if np.any(q < lo+.04) or np.any(q > hi-.04):
                raise ValueError("Recorded pose lacks 0.04 rad travel margin; all-zero J4 is not a valid home")
        if np.max(np.abs(goal-home)) > c["reference_radius"]:
            raise ValueError("Goal outside initial local reference envelope")
    c["model"] = str(Path(c["model"]).expanduser().absolute())
    return c

def contract(c):
    # Changing a control or observation setting invalidates the trained policy.
    keys = ("schema", "joint_names", "base_body", "ee_site", "sign", "offset", "q_min", "q_max", "kp", "kd",
            "torque_alarm", "speed_alarm", "tracking_alarm", "reference_speed", "reference_accel", "home",
            "goal_q", "reference_radius", "actual_radius", "success_distance", "success_speed", "episode_seconds")
    return hashlib.sha256(json.dumps({k:c[k] for k in keys}, sort_keys=True).encode()).hexdigest()

def effective(x, low, high, bits):
    n = (1 << bits)-1
    return low + np.floor((np.asarray(x)-low)/(high-low)*n)*(high-low)/n

def effort(c, reference, q, dq):
    # Exactly the legacy MIT encoder's truncation, including nominal-zero offsets.
    vm = np.array([45,45,10,10,30,30,30.]); tm = np.array([54,54,28,28,10,10,10.])
    return (effective(c["kp"],0,500,12)*(effective(reference,-12.5,12.5,16)-q)
            + effective(c["kd"],0,5,12)*(effective(np.zeros(7),-vm,vm,12)-dq)
            + effective(np.zeros(7),-tm,tm,12))

class Reference:
    """Bounded velocity action integrated at 50 Hz; shared by sim and real."""
    def __init__(self, c, q):
        self.c = c
        self.q = vec(q).copy()
        self.v = np.zeros(7)

    def step(self, action):
        a = np.clip(vec(action), -1, 1)
        wanted = a*self.c["reference_speed"]
        accel = self.c["reference_accel"]
        home = vec(self.c["home"])
        low = np.maximum(vec(self.c["q_min"])+.04, home-self.c["reference_radius"])
        high = np.minimum(vec(self.c["q_max"])-.04, home+self.c["reference_radius"])
        # Braking distance guard, before integrating. Never silently jump/clamp a reference.
        for j in range(7):
            direction = np.sign(self.v[j]) if abs(self.v[j])>1e-12 else np.sign(wanted[j])
            distance = high[j]-self.q[j] if direction >= 0 else self.q[j]-low[j]
            if distance <= self.v[j]**2/(2*accel)+2*abs(self.v[j])*DT+accel*DT**2:
                if wanted[j]*direction > 0:
                    wanted[j] = 0
        self.v += np.clip(wanted-self.v, -accel*DT, accel*DT)
        q = self.q + self.v*DT
        if np.any(q < low-1e-9) or np.any(q > high+1e-9):
            raise ValueError("Reference reached local envelope")
        self.q = q
        return q.copy()

def observation(c, q, dq, reference, previous, goal, position):
    lo, hi = vec(c["q_min"]), vec(c["q_max"])
    result = np.concatenate([(q-(lo+hi)/2)/((hi-lo)/2), dq/vec(c["speed_alarm"]),
                             (reference-q)/c["tracking_alarm"], previous, goal/.6, (goal-position)/.3])
    if result.shape != (34,) or not np.all(np.isfinite(result)):
        raise ValueError("Invalid observation")
    return result.astype(np.float32)

class Physics:
    def __init__(self, c):
        import mujoco as mj
        self.mj, self.c = mj, c
        self.m = mj.MjModel.from_xml_path(c["model"])
        if self.m.nq != 7 or self.m.nv != 7:
            raise ValueError("Model must contain only seven hinge DOFs: no J8, free base or gripper")
        self.j = np.array([mj.mj_name2id(self.m,mj.mjtObj.mjOBJ_JOINT,n) for n in c["joint_names"]])
        if np.any(self.j < 0) or np.any(self.m.jnt_type[self.j] != mj.mjtJoint.mjJNT_HINGE):
            raise ValueError("Joint mapping not seven named hinges")
        self.qa, self.da = self.m.jnt_qposadr[self.j], self.m.jnt_dofadr[self.j]
        self.site = mj.mj_name2id(self.m,mj.mjtObj.mjOBJ_SITE,c["ee_site"])
        self.base = mj.mj_name2id(self.m,mj.mjtObj.mjOBJ_BODY,c["base_body"])
        if self.site < 0 or self.base < 0:
            raise ValueError("Unknown EE site or base body; inspect model names")
        body = self.base
        while body:
            if self.m.body_jntnum[body]:
                raise ValueError("Base frame must be fixed")
            body = self.m.body_parentid[body]
        self.sign, self.offset = vec(c["sign"]), vec(c["offset"])
        self.m.opt.timestep = .001
        self.m.opt.disableflags |= int(mj.mjtDisableBit.mjDSBL_ACTUATION)
        self.m.body_gravcomp[:] = 0
        self.d, self.kin = mj.MjData(self.m), mj.MjData(self.m)
        # Hash compiled nominal model, including resolved meshes, before randomization.
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            path = str(Path(td)/"model.mjb")
            mj.mj_saveModel(self.m, path, None)
            self.model_hash = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        self.mass = self.m.body_mass.copy(); self.inertia = self.m.body_inertia.copy()
        self.damping = self.m.dof_damping.copy(); self.friction = self.m.dof_frictionloss.copy()

    def fk(self, q):
        self.kin.qpos[self.qa] = self.sign*vec(q)+self.offset
        self.mj.mj_kinematics(self.m,self.kin)
        return self.kin.xmat[self.base].reshape(3,3).T @ (self.kin.site_xpos[self.site]-self.kin.xpos[self.base])

    def check_pose(self,q):
        q = vec(q)
        if np.any(q < vec(self.c["q_min"])+.04) or np.any(q > vec(self.c["q_max"])-.04):
            raise ValueError("Pose lacks motor travel margin")
        qm = self.sign*q+self.offset
        if np.any(qm < self.m.jnt_range[self.j,0]+.04) or np.any(qm > self.m.jnt_range[self.j,1]-.04):
            raise ValueError("Pose lacks model travel margin")
        self.kin.qpos[self.qa] = qm
        self.kin.qvel[:] = 0
        self.mj.mj_forward(self.m,self.kin)
        if self.kin.ncon:
            raise ValueError("Nominal model contact; review geometry, do not ignore")
        if np.any(np.abs(self.kin.qfrc_bias[self.da]-self.kin.qfrc_passive[self.da]) > .6*vec(self.c["torque_alarm"])):
            raise ValueError("Nominal holding effort exceeds 60% alarm headroom")

    def check_path(self,a,b):
        a,b=vec(a),vec(b)
        for u in np.linspace(0,1,max(2,int(np.max(np.abs(b-a))/.005)+2)):
            self.check_pose(a+u*(b-a))

    def reset(self,q,rng=None):
        self.m.body_mass[:] = self.mass
        self.m.body_inertia[:] = self.inertia
        self.m.dof_damping[:] = self.damping
        self.m.dof_frictionloss[:] = self.friction
        if rng is not None:
            factors = rng.uniform(.9,1.1,self.m.nbody)
            self.m.body_mass[:] *= factors
            self.m.body_inertia[:] *= factors[:,None]
            self.m.dof_damping[:] *= rng.uniform(.7,1.3,self.m.nv)
            self.m.dof_frictionloss[:] *= rng.uniform(.7,1.3,self.m.nv)
        self.mj.mj_setConst(self.m,self.d)
        self.mj.mj_resetData(self.m,self.d)
        self.d.qpos[self.qa] = self.sign*q+self.offset
        self.mj.mj_forward(self.m,self.d)

    def state(self):
        return (self.sign*(self.d.qpos[self.qa]-self.offset),self.sign*self.d.qvel[self.da])

    def advance(self,ref):
        for _ in range(20):
            q,dq = self.state()
            tau = effort(self.c,ref,q,dq)
            if np.any(np.abs(tau)>vec(self.c["torque_alarm"])):
                raise ValueError("Predicted effort alarm")
            self.d.qfrc_applied[:] = 0
            self.d.qfrc_applied[self.da] = self.sign*tau
            self.mj.mj_step(self.m,self.d)
            self.mj.mj_forward(self.m,self.d)
            q,dq = self.state()
            if (not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq)) or self.d.ncon
                or np.any(q<vec(self.c["q_min"])+.04) or np.any(q>vec(self.c["q_max"])-.04)
                or np.max(np.abs(q-vec(self.c["home"])))>self.c["actual_radius"]
                or np.any(np.abs(dq)>vec(self.c["speed_alarm"]))
                or np.max(np.abs(ref-q))>self.c["tracking_alarm"]
                or np.any(self.d.warning.number)):
                raise ValueError("Simulation collision/position/velocity/tracking/numerical guard")
