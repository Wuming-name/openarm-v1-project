"""Watch a trained policy in MuJoCo. Local desktop window required; no CAN."""
import argparse
import time
import io
import zipfile
from pathlib import Path
import numpy as np
from core import load,read,contract,vec
from reach_env import ReachEnv

def checkpoint_snapshot(run, checkpoint=None, latest=False):
    """Read one immutable in-memory snapshot; skip partially written latest ZIPs."""
    if latest:
        candidates=sorted((run/'checkpoints').glob('*.zip'),key=lambda p:p.stat().st_mtime,reverse=True)
        if (run/'policy.zip').exists():
            candidates.append(run/'policy.zip')
    else:
        candidates=[checkpoint or run/'policy.zip']
    for path in candidates:
        try:
            payload=path.read_bytes()
            with zipfile.ZipFile(io.BytesIO(payload)) as z:
                if z.testzip() is not None:raise ValueError('Incomplete checkpoint')
                if not {'data','policy.pth'}.issubset(z.namelist()):raise ValueError('Not an SB3 checkpoint')
            return path,io.BytesIO(payload)
        except (OSError,zipfile.BadZipFile,ValueError):
            if not latest:raise
    raise ValueError('No complete checkpoint available yet; try again after a checkpoint is saved')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='config.sim.json');p.add_argument('--run',type=Path,default=Path('runs/goal14cm'))
    choice=p.add_mutually_exclusive_group()
    choice.add_argument('--checkpoint',type=Path,help='Checkpoint file from the specified --run; relative to current directory')
    choice.add_argument('--latest',action='store_true',help='Load newest complete periodic checkpoint once, at startup')
    p.add_argument('--episodes',type=int,default=3);args=p.parse_args()
    if args.episodes<1:raise ValueError('episodes must be positive')
    import torch
    import mujoco
    import mujoco.viewer
    from stable_baselines3 import PPO
    torch.set_num_threads(1)
    c=load(args.config);env=ReachEnv(c,False);meta=read(args.run/'contract.json')
    if meta['contract']!=contract(c) or meta['model_hash']!=env.p.model_hash:raise ValueError('Model/config mismatch')
    path,snapshot=checkpoint_snapshot(args.run,args.checkpoint,args.latest)
    # Require checkpoint to belong to the run whose configuration was verified.
    if path.absolute().parent not in (args.run.absolute(),(args.run/'checkpoints').absolute()):
        raise ValueError('Checkpoint must be inside the specified run or its checkpoints directory')
    policy=PPO.load(snapshot,device='cpu')
    print('Viewing saved policy:',path,flush=True)
    print('Separate simulation only; background training continues. No live weight reload.',flush=True)
    obs,_=env.reset(seed=999)
    with mujoco.viewer.launch_passive(env.p.m,env.p.d) as viewer:
        viewer.cam.lookat[:]=[0,-.15,.35];viewer.cam.distance=1.2;viewer.cam.azimuth=130;viewer.cam.elevation=-20
        # Red sphere marks the fixed model target in world coordinates.
        env.p.fk(c['goal_q']);target=env.p.kin.site_xpos[env.p.site].copy()
        for ep in range(args.episodes):
            obs,_=env.reset(seed=999+ep);done=trunc=False;info={}
            while viewer.is_running() and not (done or trunc):
                t=time.monotonic();action,_=policy.predict(obs,deterministic=True)
                with viewer.lock():
                    obs,_,done,trunc,info=env.step(action)
                    viewer.user_scn.ngeom=1
                    mujoco.mjv_initGeom(viewer.user_scn.geoms[0],mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([.015,.015,.015]),target,np.eye(3).flatten(),np.array([1.,0.,0.,.6]))
                viewer.sync();time.sleep(max(0,.1-(time.monotonic()-t)))
            print('Episode',ep+1,info)
            if not viewer.is_running():break
if __name__=='__main__':main()
