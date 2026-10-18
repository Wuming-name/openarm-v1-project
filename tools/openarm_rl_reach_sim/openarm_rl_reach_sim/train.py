"""Simulation only: train PPO, or evaluate a saved checkpoint. Never opens CAN."""
import argparse
import csv
from pathlib import Path
import numpy as np
from core import load,save,read,contract,DT,DECIMATION

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config",default="config.sim.json")
    p.add_argument("--out",default="runs/goal14cm")
    p.add_argument("--steps",type=int,default=500000)
    p.add_argument("--envs",type=int,default=4)
    p.add_argument("--seed",type=int,default=42)
    p.add_argument("--evaluate",type=Path)
    p.add_argument("--episodes",type=int,default=20)
    p.add_argument("--resume",type=Path,help="Previous run folder; continue PPO, but use a NEW --out directory")
    args=p.parse_args()
    if args.episodes<1 or (args.resume and args.evaluate):raise ValueError("Invalid episodes/resume/evaluate")
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_checker import check_env
    from stable_baselines3.common.vec_env import DummyVecEnv
    from stable_baselines3.common.monitor import Monitor
    from reach_env import ReachEnv
    torch.set_num_threads(1)
    c=load(args.config)
    probe=ReachEnv(c,False)
    check_env(probe,warn=True)
    if args.evaluate:
        folder=args.evaluate
        meta=read(folder/"contract.json")
        if meta["contract"]!=contract(c) or meta["model_hash"]!=probe.p.model_hash:
            raise ValueError("Config/model differs from training")
        model=PPO.load(folder/"policy.zip",device="cpu")
        records=[];episodes=[]
        for randomize in (False,True):
            env=ReachEnv(c,randomize)
            for episode in range(args.episodes):
                obs,_=env.reset(seed=10000+episode)
                terminal=truncated=False;step=0
                while not (terminal or truncated):
                    action,_=model.predict(obs,deterministic=True)
                    obs,r,terminal,truncated,info=env.step(action)
                    q,dq=env.p.state();step+=1
                    records.append([int(randomize),episode,step*DT*DECIMATION,*q,*dq,info["distance_m"],info["fault"]])
                episodes.append({"randomized":randomize,"episode":episode,**info})
        import hashlib
        report={"simulation_only":True,"policy_sha256":hashlib.sha256((folder/"policy.zip").read_bytes()).hexdigest(),"episodes":episodes,"nominal_success_rate":np.mean([e["is_success"] for e in episodes if not e["randomized"]]).item(),
                "randomized_success_rate":np.mean([e["is_success"] for e in episodes if e["randomized"]]).item(),
                "scope":"fixed recorded goal; independent seeds, not unseen-goal validation; no real-world accuracy guarantee"}
        save(folder/"evaluation.json",report)
        with (folder/"evaluation.csv").open("w",newline="") as f:
            w=csv.writer(f);w.writerow(["randomized","episode","time_s",*[f"q{i}" for i in range(1,8)],*[f"dq{i}" for i in range(1,8)],"distance_m","fault"]);w.writerows(records)
        print(report)
    else:
        if args.steps<32 or not 1<=args.envs<=16:raise ValueError("Invalid steps/envs")
        folder=Path(args.out);folder.mkdir(parents=True,exist_ok=False)
        save(folder/"config.json",c)
        save(folder/"contract.json",{"contract":contract(c),"model_hash":probe.p.model_hash,"observation":34,"action":7,"algorithm":"PPO","seed":args.seed})
        env=DummyVecEnv([lambda:Monitor(ReachEnv(c,True)) for _ in range(args.envs)])
        model=PPO("MlpPolicy",env,learning_rate=3e-4,n_steps=512,batch_size=128,n_epochs=10,
                  gamma=.995,gae_lambda=.95,clip_range=.2,ent_coef=.001,target_kl=.03,
                  policy_kwargs={"net_arch":{"pi":[128,128],"vf":[128,128]},"activation_fn":torch.nn.Tanh},
                  device="cpu",verbose=1,seed=args.seed)
        if args.resume:
            previous=read(args.resume/"contract.json")
            if previous["contract"]!=contract(c) or previous["model_hash"]!=probe.p.model_hash:
                raise ValueError("Resume configuration/model mismatch; old v1 weights are incompatible")
            model=PPO.load(args.resume/"policy.zip",env=env,device="cpu")
        from stable_baselines3.common.logger import configure
        model.set_logger(configure(str(folder),["stdout","csv"]))
        from stable_baselines3.common.callbacks import CheckpointCallback
        checkpoint=CheckpointCallback(save_freq=max(1,25000//args.envs),save_path=str(folder/"checkpoints"),name_prefix="ppo")
        start=int(model.num_timesteps);complete=False
        try:
            model.learn(total_timesteps=args.steps,callback=checkpoint,reset_num_timesteps=not bool(args.resume))
            complete=True
        finally:
            model.save(folder/"policy.zip");env.close()
            save(folder/"training_status.json",{"budget_completed":complete,"start_steps":start,
                     "total_steps":int(model.num_timesteps),"simulation_only":True})
        print("Training saved; evaluate in simulation. No hardware deployment in this package:",folder)

if __name__=="__main__":main()
