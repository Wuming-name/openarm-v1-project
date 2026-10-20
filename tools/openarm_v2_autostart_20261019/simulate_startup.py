"""No CAN: exercise the automatic preparation using the user's MuJoCo model."""
import argparse
import json
import numpy as np
from sim_core import Physics, DT, effort, load, vec
from startup_control import AutoHome, held_reference, parking_kind, screen_path, screen_pose


def simulate(c, initial_q, random_seed=None):
    p=Physics(c)
    q=vec(initial_q)
    parking_kind(c,q)
    p.reset(q)
    start=held_reference(c,p,q)
    final=held_reference(c,p,vec(c["home"]))
    screen_path(c,p,q,c["home"],startup=True)
    screen_path(c,p,start,final,startup=True,reference=True)
    # Keep the controller's preload nominal, then perturb the physical plant.
    # Recomputing preload from the perturbed plant would hide model mismatch.
    if random_seed is not None:
        p.reset(q,np.random.default_rng(random_seed))
    controller=AutoHome(c,start,final)
    records=[]
    for tick in range(6500):
        q,dq=p.state()
        screen_pose(c,p,q,startup=True)
        target,ready=controller.step(q,dq)
        for _ in range(20):
            q,dq=p.state()
            tau=effort(c,target,q,dq)
            if np.any(np.abs(tau)>vec(c["torque_alarm"])):
                raise ValueError("Simulation startup torque alarm")
            p.d.qfrc_applied[:]=0
            p.d.qfrc_applied[p.da]=p.sign*tau
            p.mj.mj_step(p.m,p.d)
            p.mj.mj_forward(p.m,p.d)
            q,dq=p.state()
            if np.any(np.abs(dq)>vec(c["speed_alarm"])):
                raise ValueError("Simulation startup speed alarm")
            if np.max(np.abs(target-q))>c["tracking_alarm"]:
                raise ValueError("Simulation startup tracking alarm")
        records.append({"time_s":(tick+1)*DT,"phase":controller.phase,
                        "q":q.tolist(),"dq":dq.tolist(),"reference":target.tolist(),
                        "reference_velocity":controller.ref.v.tolist()})
        if ready:
            screen_pose(c,p,q,startup=False)
            for _ in range(250):
                p.advance(target)
                q,dq=p.state()
                if np.max(np.abs(q-vec(c["home"])))>.005 or np.max(np.abs(dq))>=c["success_speed"]:
                    raise ValueError("Simulated 5-second holding check failed")
            return {"ready":True,"seconds":(tick+1)*DT,"initial_q":vec(initial_q).tolist(),
                    "final_q":q.tolist(),"final_error_max_rad":float(np.max(np.abs(q-vec(c["home"])))),
                    "final_speed_max":float(np.max(np.abs(dq))),
                    "trim_max_rad":float(np.max(np.abs(target-final))),
                    "holding_5s_passed":True,
                    "scope":"MuJoCo preparation only; not hardware validation"},records
    raise ValueError("Simulation preparation time bound exceeded")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",required=True)
    args=parser.parse_args()
    c=load(args.config)
    q=np.zeros(7)
    q[3]=-.0020981155108
    report,_=simulate(c,q)
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=="__main__":
    main()
