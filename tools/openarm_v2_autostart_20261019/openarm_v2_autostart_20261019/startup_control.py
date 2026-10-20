"""Restricted zero-neighborhood -> A preparation, separate from the RL contract."""
import math
import numpy as np
from sim_core import DT, Reference, effective, effort, vec

ZERO_RADIUS = .03
J4_ZERO_LOW = -.005
J4_ZERO_HIGH = .06
HOME_CAPTURE = .08
HOME_TOL = .005
TRIM_LIMIT = .03
TRIM_SPEED = .003
TRIM_TIMEOUT = 30.


def parking_kind(c, q):
    q = vec(q)
    if np.max(np.abs(q-vec(c["home"]))) <= HOME_CAPTURE:
        return "near_A"
    if (np.max(np.abs(q[[0,1,2,4,5,6]])) <= ZERO_RADIUS
            and J4_ZERO_LOW <= q[3] <= J4_ZERO_HIGH):
        return "near_zero"
    raise ValueError("Start is neither near zero nor near A; no arbitrary-pose homing. "
                     f"actual_q={q.tolist()}")


def reference_bounds(c, startup):
    lo = np.maximum(vec(c["q_min"])+.04, vec(c["home"])-c["reference_radius"])
    hi = np.minimum(vec(c["q_max"])-.04, vec(c["home"])+c["reference_radius"])
    if startup:
        lo[3] = max(0., c["home"][3]-c["reference_radius"])
    return lo, hi


def screen_pose(c, p, q, *, startup, reference=False):
    """Keep collision/effort checks; only J4's startup lower margin differs."""
    q = vec(q)
    lo = np.maximum(vec(c["q_min"])+.04, p.m.jnt_range[p.j,0]+.04)
    hi = np.minimum(vec(c["q_max"])-.04, p.m.jnt_range[p.j,1]-.04)
    if startup:
        lo[3] = 0. if reference else J4_ZERO_LOW
    if np.any(q < lo-1e-10) or np.any(q > hi+1e-10):
        raise ValueError("Startup/normal travel check failed: " + str(q.tolist()))
    bound = c["reference_radius"] if reference else c["actual_radius"]
    if np.max(np.abs(q-vec(c["home"]))) > bound+1e-9:
        raise ValueError("Local reference/actual envelope exceeded")
    p.kin.qpos[p.qa] = p.sign*q+p.offset
    p.kin.qvel[:] = 0
    p.mj.mj_forward(p.m, p.kin)
    if p.kin.ncon:
        raise ValueError("Model contact on preparation path")
    if not reference:
        required = p.sign*(p.kin.qfrc_bias[p.da]-p.kin.qfrc_passive[p.da])
        if np.any(np.abs(required) > .6*vec(c["torque_alarm"])):
            raise ValueError("Nominal holding effort exceeds existing 60% headroom")


def screen_path(c, p, a, b, *, startup, reference=False):
    a, b = vec(a), vec(b)
    for u in np.linspace(0,1,max(2,int(np.max(np.abs(b-a))/.005)+2)):
        screen_pose(c,p,a+u*(b-a),startup=startup,reference=reference)


def held_reference(c, p, q):
    """Estimate a static PD preload at measured q, with no runtime torque FF."""
    q = vec(q)
    screen_pose(c,p,q,startup=True)
    required = p.sign*(p.kin.qfrc_bias[p.da]-p.kin.qfrc_passive[p.da])
    target = q + (required-effort(c,q,q,np.zeros(7)))/effective(c["kp"],0,500,12)
    # Never request a negative J4 angle. The tiny raw negative allowance is
    # for startup feedback only, not a negative commanded motion range.
    target[3] = max(0., target[3])
    screen_pose(c,p,target,startup=True,reference=True)
    if np.max(np.abs(target-q)) > c["tracking_alarm"]:
        raise ValueError("Initial static preload exceeds tracking allowance")
    if np.any(np.abs(effort(c,target,q,np.zeros(7))) > vec(c["torque_alarm"])):
        raise ValueError("Initial static preload exceeds effort alarm")
    return target


def trajectory_seconds(c, a, b):
    delta = float(np.max(np.abs(vec(b)-vec(a))))
    return max(2.,1.875*delta/c["reference_speed"],
               math.sqrt(5.774*delta/c["reference_accel"]))


class RateReference(Reference):
    def __init__(self,c,q,startup=True):
        super().__init__(c,q)
        self.low,self.high=reference_bounds(c,startup)

    def advance_velocity(self,wanted,low=None,high=None):
        wanted=np.clip(vec(wanted),-self.c["reference_speed"],self.c["reference_speed"])
        low=self.low if low is None else np.maximum(low,self.low)
        high=self.high if high is None else np.minimum(high,self.high)
        accel=self.c["reference_accel"]
        for j in range(7):
            direction=np.sign(self.v[j]) if abs(self.v[j])>1e-12 else np.sign(wanted[j])
            distance=high[j]-self.q[j] if direction>=0 else self.q[j]-low[j]
            if distance<=self.v[j]**2/(2*accel)+2*abs(self.v[j])*DT+accel*DT**2:
                if wanted[j]*direction>0:
                    wanted[j]=0
        self.v+=np.clip(wanted-self.v,-accel*DT,accel*DT)
        target=self.q+self.v*DT
        if np.any(target<low-1e-9) or np.any(target>high+1e-9):
            raise ValueError("Preparation reference boundary reached")
        self.q=target
        return self.q.copy()

    def step(self,action):
        return self.advance_velocity(np.clip(vec(action),-1,1)*self.c["reference_speed"])


class AutoHome:
    def __init__(self,c,start_reference,home_reference):
        self.c=c
        self.start=vec(start_reference).copy()
        self.home_reference=vec(home_reference).copy()
        self.ref=RateReference(c,self.start)
        self.duration=trajectory_seconds(c,self.start,self.home_reference)
        self.tick=0
        self.trim_tick=0
        self.stable=0
        self.phase="move_to_A"
        low,high=reference_bounds(c,True)
        self.trim_low=np.maximum(low,self.home_reference-TRIM_LIMIT)
        self.trim_high=np.minimum(high,self.home_reference+TRIM_LIMIT)

    def step(self,q,dq):
        q,dq=vec(q),vec(dq)
        if self.phase=="move_to_A":
            self.tick+=1
            u=min(self.tick*DT/self.duration,1.)
            target=self.start+(10*u**3-15*u**4+6*u**5)*(self.home_reference-self.start)
            self.ref.v=(target-self.ref.q)/DT
            self.ref.q=target
            if u>=1:
                self.phase="settle_A"
        else:
            self.trim_tick+=1
            error=vec(self.c["home"])-q
            wanted=np.clip(.3*error,-TRIM_SPEED,TRIM_SPEED)
            wanted[np.abs(error)<.0025]=0
            self.ref.advance_velocity(wanted,self.trim_low,self.trim_high)
            good=(np.max(np.abs(error))<=HOME_TOL
                  and np.max(np.abs(dq))<self.c["success_speed"]
                  and np.max(np.abs(self.ref.v))<1e-10)
            self.stable=self.stable+1 if good else 0
            if self.trim_tick*DT>=TRIM_TIMEOUT and self.stable<50:
                raise ValueError("A did not settle within bounded trim; RL was not started")
        return self.ref.q.copy(),self.stable>=50
