"""Watch a trained policy in MuJoCo. Local desktop window required; no CAN."""
import argparse
import time
from pathlib import Path
import numpy as np
from core import load,read,contract,vec
from reach_env import ReachEnv

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='config.sim.json');p.add_argument('--run',type=Path,default=Path('runs/goal14cm'))
    p.add_argument('--episodes',type=int,default=3);args=p.parse_args()
    import torch
    import mujoco
    import mujoco.viewer
    from stable_baselines3 import PPO
    torch.set_num_threads(1)
    c=load(args.config);env=ReachEnv(c,False);meta=read(args.run/'contract.json')
    if meta['contract']!=contract(c) or meta['model_hash']!=env.p.model_hash:raise ValueError('Model/config mismatch')
    policy=PPO.load(args.run/'policy.zip',device='cpu')
    obs,_=env.reset(seed=999)
    with mujoco.viewer.launch_passive(env.p.m,env.p.d) as viewer:
        viewer.cam.lookat[:]=[0,-.15,.35];viewer.cam.distance=1.2;viewer.cam.azimuth=130;viewer.cam.elevation=-20
        # Red sphere marks the fixed model target in world coordinates.
        env.p.fk(c['goal_q']);target=env.p.kin.site_xpos[env.p.site].copy()
        for ep in range(args.episodes):
            obs,_=env.reset(seed=999+ep);done=trunc=False;info={}
            while viewer.is_running() and not (done or trunc):
                t=time.monotonic();action,_=policy.predict(obs,deterministic=True)
                obs,_,done,trunc,info=env.step(action)
                with viewer.lock():
                    viewer.user_scn.ngeom=1
                    mujoco.mjv_initGeom(viewer.user_scn.geoms[0],mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([.015,.015,.015]),target,np.eye(3).flatten(),np.array([1.,0.,0.,.6]))
                viewer.sync();time.sleep(max(0,.1-(time.monotonic()-t)))
            print('Episode',ep+1,info)
            if not viewer.is_running():break
if __name__=='__main__':main()
