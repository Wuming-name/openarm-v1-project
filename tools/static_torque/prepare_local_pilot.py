#!/usr/bin/env python3
"""Prepare an offline, one-degree exploratory experiment using verified holding gains.

No CAN access. Unknown physical masses/zero accuracy stay UNCONFIRMED. This pilot
does not authorize full-range motion or claim calibrated torque/model accuracy.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import torque_model as tm

KP=[35,35,20,20,6,6,6]
KD=[4,4,2,2,0.8,0.8,0.8]
ALARMS=[2,2,1.5,1.5,0.7,0.7,0.7]
STEP=math.pi/180


def make_config(base, model_path, q, model_low, model_high):
    cfg=dict(base)
    cfg.update(experiment_scope='local_pilot_1deg',holding_gains_verified=True,
               fall_protection_confirmed=True,motion_limits_reviewed=False,
               geometry_and_mass_reviewed=False,feedback_is_joint_nm_confirmed=False,
               model_xml=str(model_path),joint_names=[f'openarm_right_joint{i}' for i in range(1,8)],
               joint_sign=[1]*7,zero_offset_rad=[0]*7,zero_uncertainty_rad=[None]*7,
               pilot_baseline_q_motor_rad=q,
               q_min_rad=[max(model_low[i],q[i]-0.12) for i in range(7)],
               q_max_rad=[min(model_high[i],q[i]+0.12) for i in range(7)],
               kp=KP.copy(),kd=KD.copy(),torque_alarm_nm=ALARMS.copy(),speed_alarm_rad_s=[0.15]*7,
               tracking_alarm_rad=0.03,transit_speed_rad_s=0.005,transit_accel_rad_s2=0.01,
               temperature_alarm_c=55,settle_timeout_s=20,sample_seconds=3,
               startup_tolerance_rad=0.003,settle_position_error_rad=0.025,
               settle_speed_rad_s=0.04,settle_spread_rad=0.003,stable_seconds=1,
               minimum_valid_samples=40,path_margin_rad=0.02,gravity_alarm_fraction=0.6)
    tm.validate_config(cfg)
    return cfg


def make_poses(q):
    poses=[{'pose_id':'base_initial','q_motor_rad':q.copy()}]
    for i in range(7):
        moved=q.copy(); moved[i]+=STEP
        poses.append({'pose_id':f'j{i+1}_plus_1deg','q_motor_rad':moved})
        poses.append({'pose_id':f'base_after_j{i+1}','q_motor_rad':q.copy()})
    return poses


def fresh_snapshot(path, log_root):
    if path=='latest':
        candidates=[]
        for p in log_root.glob('diagnose_*/snapshot.json'):
            try:
                meta=tm.read_json(p.parent/'metadata.json')
                if meta.get('completed') and meta.get('mode')=='diagnose' and meta.get('interface')=='can0':
                    stamp=tm.read_json(p).get('run_time_ns',0)
                    candidates.append((stamp,p))
            except (OSError,ValueError):
                continue
        if not candidates: raise ValueError('No completed can0 diagnosis. Run --diagnose first.')
        path=max(candidates,key=lambda item:item[0])[1]
    path=Path(path).expanduser().resolve(strict=True)
    snap=tm.read_json(path)
    if snap.get('schema')!='openarm-snapshot-v1' or snap.get('interface')!='can0':
        raise ValueError('Expected completed can0 diagnosis snapshot')
    meta=tm.read_json(path.parent/'metadata.json')
    if meta.get('mode')!='diagnose' or not meta.get('completed'):
        raise ValueError('Snapshot diagnosis did not complete')
    age=time.time()-snap['run_time_ns']/1e9
    if not -5<=age<=180:
        raise ValueError('Snapshot is older than 3 minutes / clock mismatch. Run --diagnose again, then prepare promptly.')
    q=tm.vector7(snap,'q_motor_rad')
    with (path.parent/'samples.csv').open(encoding='utf-8',newline='') as stream:
        recent=[r for r in csv.DictReader(stream) if r['stage']=='diagnose'][-20:]
    if len(recent)!=20: raise ValueError('Need at least 20 diagnostic samples')
    for row in recent:
        if any(row[f'status{i}']!='0' for i in range(1,8)) or float(row['transaction_ms'])>50:
            raise ValueError('Initial motors must be disabled with valid feedback')
    for i in range(7):
        actual=[float(r[f'q{i+1}']) for r in recent]
        if not all(math.isfinite(x) for x in actual) or max(actual)-min(actual)>0.003 or max(abs(x-q[i]) for x in actual)>0.003:
            raise ValueError(f'J{i+1} moved during diagnosis. Stabilize supported arm and repeat; do not increase tolerance.')
    return path,snap,q


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',default='latest')
    parser.add_argument('--log-root',type=Path,default=Path.home()/'experiments/torque_compare')
    parser.add_argument('--model',type=Path,default=Path.home()/'openarm_v1_ws/src/openarm-v1-project/models/openarm_v1_right/openarm_v1_7dof.xml')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    try:
        snapshot,snap,q=fresh_snapshot(args.snapshot,args.log_root)
        import mujoco
        model_path=args.model.expanduser().resolve(strict=True)
        raw=mujoco.MjModel.from_xml_path(str(model_path))
        names=[f'openarm_right_joint{i}' for i in range(1,8)]
        ids=[mujoco.mj_name2id(raw,mujoco.mjtObj.mjOBJ_JOINT,n) for n in names]
        if raw.nq!=7 or raw.nv!=7 or any(j<0 or not raw.jnt_limited[j] for j in ids):
            raise ValueError('Expected fixed-base 7DOF model with finite joint ranges')
        base=tm.read_json(Path(__file__).parent/'safety.template.json')
        cfg=make_config(base,model_path,q,[float(raw.jnt_range[j,0]) for j in ids],[float(raw.jnt_range[j,1]) for j in ids])
        model=tm.Model(cfg,model_path)
        poses=make_poses(q)
        points=[q]+[p['q_motor_rad'] for p in poses]+[q]
        checked=0; peak=[0.0]*7
        for a,b in zip(points,points[1:]):
            count=max(1,math.ceil(max(abs(y-x) for x,y in zip(a,b))/0.002))
            for n in range(count+1):
                current=[x+(y-x)*n/count for x,y in zip(a,b)]
                gravity=model.check_point(current,cfg); checked+=1
                for i,g in enumerate(gravity): peak[i]=max(peak[i],abs(g))
        out=args.out.expanduser().resolve()
        out.mkdir(parents=True,exist_ok=False)
        frozen=out/'model.mjb'; mujoco.mj_saveModel(model.model,str(frozen),None)
        tm.save_json(out/'pilot_config.json',cfg)
        with (out/'pilot_poses.csv').open('x',encoding='utf-8',newline='') as stream:
            writer=csv.writer(stream); writer.writerow(['pose_id']+[f'q{i}_rad' for i in range(1,8)])
            for pose in poses: writer.writerow([pose['pose_id']]+pose['q_motor_rad'])
        plan={'schema':'openarm-static-v1','model_path_checked':True,'config':cfg,
              'baseline_q_motor_rad':q,'poses':poses,'path_samples_checked':checked,
              'frozen_model_file':'model.mjb','frozen_model_sha256':tm.sha256(frozen),
              'model_xml_path':str(model_path),'model_xml_sha256':tm.sha256(model_path),
              'mujoco_version':mujoco.__version__,'source_snapshot':str(snapshot),
              'config_sha256':tm.sha256(out/'pilot_config.json'),'poses_sha256':tm.sha256(out/'pilot_poses.csv'),
              'zero_calibration_status':'unknown accuracy; no numerical uncertainty bound established',
              'operator_confirmations':'existing KP/KD stable holding and physical fall protection confirmed in conversation',
              'limitations':'local one-degree exploratory trial only; physical limits, mass/COM and zero accuracy NOT confirmed; model contact filters remain active',
              'nominal_model_peak_gravity_nm':peak}
        tm.save_json(out/'plan.json',plan)
        print('Prepared LOCAL PILOT. No CAN access; no motor commands.')
        print('Snapshot:',snapshot)
        print('15 poses: base, J1 +1deg/base, J2 +1deg/base, ... J7 +1deg/base.')
        print('KP:',KP,'KD:',KD)
        print('Software torque alarms (NOT ratings or hard limits):',ALARMS,'Nm')
        print('Nominal model peak gravity:',[round(x,4) for x in peak],'Nm')
        print('Nominal speed <=0.005 rad/s; tracking alarm=0.03 rad. Expected duration about 3 minutes.')
        print('Uncertain masses/zero remain unconfirmed; this plan does NOT authorize wider motion.')
        print('Plan:',out/'plan.json')
        print('Run from tools/static_torque AFTER reviewing the prepared plan:')
        print(f'./build/openarm_static_torque --interface can0 --plan "{out / "plan.json"}" --execute')
        return 0
    except (ValueError,KeyError,OSError,TypeError,ImportError) as exc:
        parser.exit(1,f'ERROR: {exc}\n')


if __name__=='__main__':
    raise SystemExit(main())
