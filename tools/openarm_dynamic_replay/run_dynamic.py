#!/usr/bin/env python3
"""Ubuntu single-entry diagnosis -> screened trajectory -> guarded acquisition.
Default: plan only. No enable without --execute and RUN_DYNAMIC in C++.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
LOW = [-1.396263,-.174533,-1.570796,0,-1.570796,-.785398,-1.570796]
HIGH = [3.490659,3.316125,1.570796,2.443461,1.570796,.785398,1.570796]
VERSION = 'openarm-dynamic-replay-v1'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked(command):
    print('\n>', ' '.join(map(str, command)), flush=True)
    child = subprocess.Popen(list(map(str, command)), cwd=HERE)
    try:
        code = child.wait()
    except KeyboardInterrupt:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print(f'Controller PID {child.pid} still active; check physical support. No force-kill.')
        raise
    if code:
        raise RuntimeError(f'Step failed ({code}); no automatic retry.')


def check_can():
    result = subprocess.check_output(['ip','-json','-details','link','show','can0'], text=True)
    link = json.loads(result)[0]
    data = link.get('linkinfo',{}).get('info_data',{})
    if ('UP' not in link.get('flags',[]) or link.get('mtu')!=72 or
        data.get('state')!='ERROR-ACTIVE' or data.get('bittiming',{}).get('bitrate')!=1000000 or
        data.get('data_bittiming',{}).get('bitrate')!=5000000):
        raise ValueError('can0 must be UP, FD, 1Mbps/5Mbps, ERROR-ACTIVE. Verify channel/wiring first.')


def targets(base, joints, amplitude_deg, repeats, directions):
    points=[]
    for rep in range(1,repeats+1):
        for j in joints:
            for sign in ([1,-1] if directions=='both' else [1] if directions=='positive' else [-1]):
                q=list(base); q[j-1]+=sign*math.radians(amplitude_deg)
                tag=f'j{j}_{"plus" if sign>0 else "minus"}_r{rep}'
                points += [dict(pose_id=tag,q_motor_rad=q,hold_s=2),
                           dict(pose_id=tag+'_return',q_motor_rad=list(base),hold_s=2)]
    return points


def screen(model, cfg, base, poses):
    previous=base; peak=[0.]*7; count=0
    for target in [p['q_motor_rad'] for p in poses]:
        steps=max(1,math.ceil(max(abs(x-y) for x,y in zip(previous,target))/.005))
        for n in range(steps+1):
            q=[x+(y-x)*n/steps for x,y in zip(previous,target)]
            # Inherits the previous profile's nominal .04rad travel margin plus
            # .005rad startup tolerance. NOT a tracking/collision uncertainty tube.
            for j,x in enumerate(q):
                if not cfg['q_min_rad'][j]+.045<=x<=cfg['q_max_rad'][j]-.045:
                    raise ValueError(f'J{j+1}: nominal travel margin fails at {math.degrees(x):.3f}deg')
            gravity,passive,contacts=model.evaluate(q)
            if contacts:raise ValueError('Nominal model contact along trajectory')
            if any(abs(x)>1e-4 for x in passive):raise ValueError('Passive spring force requires separate screening')
            for j,x in enumerate(gravity):
                if not math.isfinite(x) or abs(x)>.6*cfg['torque_alarm_nm'][j]:
                    raise ValueError(f'J{j+1}: model gravity exceeds 60% software torque alarm')
                peak[j]=max(peak[j],abs(x))
            count+=1
        previous=target
    return count,peak


def prepare(snapshot, model_path, out, args):
    import mujoco
    import torque_model as tm
    from prepare_local_pilot import fresh_snapshot
    _,_,base=fresh_snapshot(str(snapshot),snapshot.parent)
    cfg={'joint_names':[f'openarm_right_joint{i}' for i in range(1,8)],
         'joint_sign':[1]*7,'zero_offset_rad':[0]*7,'zero_accuracy':'unverified',
         'kp':[35,35,20,20,6,6,6],'kd':[4,4,2,2,.8,.8,.8],
         'torque_profile':args.torque_profile,
         'torque_alarm_nm':([5,5,3,3,.7,.7,.7] if args.torque_profile=='proximal_5_3' else [2,2,1.5,1.5,.7,.7,.7]),
         'speed_alarm_rad_s':[.5]*7,'tracking_alarm_rad':.16,
         'transit_speed_rad_s':args.speed,'transit_accel_rad_s2':.04,
         'temperature_alarm_c':55,'startup_tolerance_rad':.005,
         'trim_j2':False,'amplitude_deg':args.amplitude_deg,
         'feedback_is_joint_nm_confirmed':False}
    model=tm.Model(cfg,model_path)
    if any(not model.model.jnt_limited[j] for j in model.ids):raise ValueError('All 7 joints need model limits')
    cfg['q_min_rad']=[max(LOW[i],float(model.model.jnt_range[j,0])) for i,j in enumerate(model.ids)]
    cfg['q_max_rad']=[min(HIGH[i],float(model.model.jnt_range[j,1])) for i,j in enumerate(model.ids)]
    poses=targets(base,args.joints,args.amplitude_deg,args.repeats,args.directions)
    count,peak=screen(model,cfg,base,poses)
    out.mkdir()
    mujoco.mj_saveModel(model.model,str(out/'model.mjb'),None)
    plan={'schema':'openarm-dynamic-v1','model_path_checked':True,'config':cfg,
          'baseline_q_motor_rad':base,'poses':poses,'frozen_model_file':'model.mjb',
          'frozen_model_sha256':sha(out/'model.mjb'),'model_xml_path':str(model_path),
          'model_xml_sha256':sha(model_path),'mujoco_version':mujoco.__version__,
          'source_snapshot':str(snapshot),'checked_points':count,'peak_gravity_nm':peak,
          'limitations':'Nominal screening only; physical zero/mass/clearance and feedback torque meaning unverified.'}
    save(out/'plan.json',plan)
    with (out/'trajectory_targets.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['phase']+[f'q{i}_rad' for i in range(1,8)]+['hold_s'])
        for p in poses:w.writerow([p['pose_id']]+p['q_motor_rad']+[p['hold_s']])
    previous=base;duration=2
    for p in poses:
        d=max(abs(x-y) for x,y in zip(previous,p['q_motor_rad']))
        duration+=max(.05,1.875*d/args.speed,math.sqrt(5.774*d/.04))+p['hold_s'];previous=p['q_motor_rad']
    print(f'Screened {len(poses)} targets; about {duration/60:.1f} min plus initial/return/end hold.')
    print('Review:',out/'trajectory_targets.csv')
    return plan


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true')
    mode.add_argument('--plan-only',action='store_true')
    p.add_argument('--joints',default='2',help='Comma-separated 1..7; one axis moves at a time')
    p.add_argument('--amplitude-deg',type=float,default=5)
    p.add_argument('--directions',choices=['both','positive','negative'],default='both')
    p.add_argument('--repeats',type=int,choices=[1,2,3],default=2)
    p.add_argument('--speed',type=float,default=.02,help='Reference peak rad/s, .005.. .05')
    p.add_argument('--torque-profile',choices=['legacy_low','proximal_5_3'],default='legacy_low')
    p.add_argument('--model',type=Path,default=HERE.parent.parent/'models/openarm_v1_right/openarm_v1_7dof.xml')
    p.add_argument('--output-root',type=Path,default=Path.home()/'experiments/dynamic_replay')
    args=p.parse_args()
    args.joints=[int(x) for x in args.joints.split(',')]
    if not args.joints or len(set(args.joints))!=len(args.joints) or any(j not in range(1,8) for j in args.joints):p.error('Unique joints 1..7 required')
    if not 1<=args.amplitude_deg<=15 or not .005<=args.speed<=.05:p.error('Amplitude 1..15deg; speed .005.. .05rad/s')
    if sys.platform!='linux':p.error('Acquisition runs on Ubuntu; replay.py can run on Windows or Ubuntu')
    if os.geteuid()==0:p.error('Run as normal user, not sudo')
    model=args.model.expanduser().resolve(strict=True)
    # Fail before any hardware use if MuJoCo is missing.
    import mujoco
    import fcntl
    lock_path=Path.home()/'.cache/openarm_static_torque.lock';lock_path.parent.mkdir(parents=True,exist_ok=True)
    with lock_path.open('a+') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        checked(['cmake','-S',HERE,'-B',HERE/'build'])
        checked(['cmake','--build',HERE/'build','-j2'])
        binary=HERE/'build/openarm_dynamic'
        if subprocess.check_output([str(binary),'--version'],text=True).strip()!=VERSION:raise ValueError('Stale binary')
        check_can()
        root=args.output_root.expanduser();root.mkdir(parents=True,exist_ok=True)
        session=root/f'session_{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns()}';session.mkdir()
        record={'session':str(session),'completed':False,'execute_requested':args.execute,'mujoco_version':mujoco.__version__}
        print('Session:',session,flush=True)
        try:
            checked([binary,'--diagnose','--output-root',session])
            snapshots=list(session.glob('diagnose_*/snapshot.json'))
            if len(snapshots)!=1:raise ValueError('Diagnosis snapshot missing/ambiguous')
            plan=prepare(snapshots[0],model,session/'prepared',args)
            record['plan']=str(session/'prepared/plan.json')
            if args.execute:
                if sha(session/'prepared/model.mjb')!=plan['frozen_model_sha256']:raise ValueError('Frozen model hash mismatch')
                checked([binary,'--execute','--plan',session/'prepared/plan.json','--output-root',session])
                runs=list(session.glob('dynamic_*/metadata.json'))
                if len(runs)!=1:raise ValueError('Result missing/ambiguous')
                meta=json.loads(runs[0].read_text())
                if not meta.get('completed'):raise ValueError('Acquisition incomplete')
                record['run']=str(runs[0].parent)
                print('Acquisition complete. Offline replay command:\n',sys.executable,HERE/'replay.py','--run',runs[0].parent)
            else:print('Plan-only complete. No enable/motion commands sent. --execute will diagnose and prepare afresh.')
            record['completed']=True
        except BaseException as exc:
            record['error']=str(exc);raise
        finally:
            save(session/'session.json',record)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:raise SystemExit(130)
    except Exception as exc:raise SystemExit(f'ERROR: {exc}')
