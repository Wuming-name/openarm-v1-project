"""Test model, randomized initialization, and a deterministic PD reference baseline.
The baseline is a feasibility diagnostic, NOT a trained policy or imitation dataset.
"""
import argparse
import csv
from pathlib import Path
import numpy as np
from core import load,save,vec,DT,effort
from reach_env import ReachEnv

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='config.sim.json')
    p.add_argument('--resets',type=int,default=20)
    p.add_argument('--out',default='preflight')
    a=p.parse_args();c=load(a.config);out=Path(a.out);out.mkdir(parents=True,exist_ok=False)
    report={'simulation_only':True,'initializations':[],'passed':False};rows=[]
    try:
        env=ReachEnv(c,True)
        for seed in range(a.resets):
            _,info=env.reset(seed=seed);q,dq=env.p.state()
            report['initializations'].append({'seed':seed,**info,'max_speed':float(np.max(np.abs(dq)))})
        env=ReachEnv(c,False);_,info=env.reset(seed=100)
        goal_ref=env.p.initial_reference(vec(c['goal_q']))
        report['nominal_initial']=info;report['baseline_goal_reference']=goal_ref.tolist()
        env.p.check_reference_path(env.ref.q,goal_ref)
        stable=0
        for step in range(int(60/DT)):
            # Position-reference servo at 20ms; PPO uses the SAME shaper, but chooses its own actions.
            action=np.clip((goal_ref-env.ref.q)*2/c['reference_speed'],-1,1)
            before=env.ref.q.copy();ref=env.ref.step(action)
            env.p.check_pose(env.p.state()[0]);env.p.check_reference_path(before,ref);env.p.advance(ref)
            q,dq=env.p.state();dist=float(np.linalg.norm(env.goal-env.p.fk(q)))
            tau=effort(c,ref,q,dq)
            rows.append([step*DT,*q,*dq,*ref,*tau,dist])
            ok=dist<c['success_distance'] and np.max(np.abs(dq))<c['success_speed'] and np.max(np.abs(env.ref.v))<.002
            stable=stable+1 if ok else 0
            if stable>=50:break
        report.update(baseline_success=stable>=50,baseline_distance_m=dist,baseline_seconds=(step+1)*DT,
                      model_hash=env.p.model_hash,passed=stable>=50)
        if not report['passed']:raise ValueError('Deterministic baseline failed; do not launch long PPO training')
        print('PASS: resets and guarded baseline completed. This is not PPO convergence.')
    except Exception as e:
        report['error']=repr(e)
        raise
    finally:
        save(out/'report.json',report)
        with (out/'baseline.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s',*[f'q{i}' for i in range(1,8)],*[f'dq{i}' for i in range(1,8)],
                *[f'reference{i}' for i in range(1,8)],*[f'command_effort{i}' for i in range(1,8)],'distance_m']);w.writerows(rows)
        print('Report:',out/'report.json')
if __name__=='__main__':main()
