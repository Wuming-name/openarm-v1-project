#!/usr/bin/env python3
"""Prepare exactly 50 nominally distinct static poses; offline, no CAN access."""
import argparse
from collections import Counter
import csv
import math
from pathlib import Path
import random
import torque_model as tm
import prepare_local_pilot as pp

LOW=[-1.396263,-.174533,-1.570796,0,-1.570796,-.785398,-1.570796]
HIGH=[3.490659,3.316125,1.570796,2.443461,1.570796,.785398,1.570796]


def distance(a,b):
    return max(abs(x-y) for x,y in zip(a,b))


def make_config(template,model_path,baseline,model_low,model_high,angle):
    cfg=pp.make_config(template,model_path,baseline,model_low,model_high)
    cfg.update(experiment_scope=tm.LEGACY_SCOPE,
               q_min_rad=[max(LOW[i],model_low[i]) for i in range(7)],
               q_max_rad=[min(HIGH[i],model_high[i]) for i in range(7)],
               tracking_alarm_rad=.16,transit_speed_rad_s=.05,
               transit_accel_rad_s2=.08,speed_alarm_rad_s=[.5]*7,
               settle_position_error_rad=.04,startup_tolerance_rad=.01,
               stable_seconds=1,sample_seconds=3,minimum_valid_samples=40,
               settle_timeout_s=30,path_margin_rad=.04,
               requested_pose_count=50,max_excursion_rad=math.radians(angle),
               minimum_distinct_pose_rad=math.radians(1))
    tm.validate_config(cfg)
    return cfg


def path_check(model,cfg,start,end):
    steps=max(1,math.ceil(distance(start,end)/.01))
    for n in range(steps+1):
        model.check_point([a+(b-a)*n/steps for a,b in zip(start,end)],cfg)


def select_poses(model,cfg,baseline,*,seed=20261015,pool_size=2000,progress=False):
    """Deterministic candidate pool + checked nearest-neighbor route.

    Max-norm separation measures degrees of at least one joint, not Cartesian
    tool displacement. All targets and return transitions must pass screening.
    No clipping a rejected path into an apparently accepted one.
    """
    amplitude=cfg['max_excursion_rad']
    separation=math.radians(5)
    rng=random.Random(seed)
    candidates=[]; rejected=Counter()
    seeds=[]
    for joint in range(7):
        for scale in (-1,-.5,.5,1):
            q=list(baseline); q[joint]+=scale*amplitude
            seeds.append(q)
    seeds.extend([[q+rng.uniform(-amplitude,amplitude) for q in baseline] for _ in range(pool_size)])
    model.check_point(baseline,cfg)
    for number,q in enumerate(seeds,1):
        if progress and number%500==0:
            print(f'Screened {number}/{len(seeds)} candidates; {len(candidates)} nominal targets retained.',flush=True)
        if distance(q,baseline)<amplitude*.35:
            continue
        try:
            model.check_point(q,cfg)
        except ValueError as exc:
            rejected[str(exc)]+=1
            continue
        if all(distance(q,old)>=separation for old in candidates):
            candidates.append(q)
    route=[]; current=list(baseline)
    while candidates and len(route)<50:
        candidates.sort(key=lambda q: (distance(current,q),q))
        chosen=None
        for index,q in enumerate(candidates):
            try:
                path_check(model,cfg,current,q)
                # Each chosen pose can nominally return to the start. Normal
                # execution follows the route; there is no automatic fault retreat.
                path_check(model,cfg,q,baseline)
            except ValueError as exc:
                rejected['path: '+str(exc)]+=1
                continue
            chosen=index
            break
        if chosen is None:
            break
        current=candidates.pop(chosen)
        route.append(current)
        candidates=[q for q in candidates if distance(q,current)>=separation]
    if len(route)!=50:
        raise ValueError(f'Only {len(route)}/50 candidates passed. No executable plan written. '
                         f'Do not bypass limits; rejection counts: {dict(rejected)}')
    # Refuse an apparently large experiment whose accepted targets cover only
    # a tiny part of the requested range.
    spans=[math.degrees(max(q[i] for q in route)-min(q[i] for q in route)) for i in range(7)]
    if sum(x>=math.degrees(amplitude)*.75 for x in spans)<3:
        raise ValueError(f'Insufficient multi-joint coverage: spans_deg={spans}')
    return route,dict(rejected),spans


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',required=True)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--max-angle-deg',type=int,choices=[10,15,20,30],default=20)
    args=parser.parse_args()
    try:
        snapshot,_,baseline=pp.fresh_snapshot(args.snapshot,Path.home()/'experiments/torque_compare')
        import mujoco
        path=args.model.expanduser().resolve(strict=True)
        raw=mujoco.MjModel.from_xml_path(str(path))
        ids=[mujoco.mj_name2id(raw,mujoco.mjtObj.mjOBJ_JOINT,f'openarm_right_joint{i}') for i in range(1,8)]
        if raw.nq!=7 or raw.nv!=7 or any(j<0 or not raw.jnt_limited[j] for j in ids):
            raise ValueError('Expected fixed-base seven-hinge model with finite ranges')
        cfg=make_config(tm.read_json(Path(__file__).parent/'safety.template.json'),path,baseline,
                        [float(raw.jnt_range[j,0]) for j in ids],
                        [float(raw.jnt_range[j,1]) for j in ids],args.max_angle_deg)
        model=tm.Model(cfg,path)
        print(f'Preparing 50 poses; each joint target within +/-{args.max_angle_deg} deg of current start.',flush=True)
        print('Legacy KP/KD unchanged. Tracking .16 rad; target speed .05 rad/s. Lower torque alarms retained.',flush=True)
        print('Nominal model screening only; physical zero/mass/clearance are not calibrated.',flush=True)
        route,rejections,spans=select_poses(model,cfg,baseline,progress=True)
        out=args.out.expanduser().resolve(); out.mkdir(parents=True,exist_ok=False)
        frozen=out/'model.mjb'; mujoco.mj_saveModel(model.model,str(frozen),None)
        poses=[{'pose_id':f'pose_{i:02d}','q_motor_rad':q} for i,q in enumerate(route,1)]
        tm.save_json(out/'config_50.json',cfg)
        with (out/'poses_50.csv').open('x',encoding='utf-8',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(['pose_id']+[f'q{i}_rad' for i in range(1,8)]+[f'delta{i}_deg' for i in range(1,8)])
            for pose in poses:
                q=pose['q_motor_rad']
                writer.writerow([pose['pose_id']]+q+[math.degrees(x-y) for x,y in zip(q,baseline)])
        plan={'schema':'openarm-static-v1','model_path_checked':True,'config':cfg,
              'baseline_q_motor_rad':baseline,'poses':poses,'frozen_model_file':'model.mjb',
              'frozen_model_sha256':tm.sha256(frozen),'model_xml_path':str(path),
              'model_xml_sha256':tm.sha256(path),'mujoco_version':mujoco.__version__,
              'source_snapshot':str(snapshot),'zero_calibration_status':'unknown; no numerical uncertainty bound',
              'limitations':'Nominal contact/travel/gravity screening; physical masses/zero and collision exclusions unverified.'}
        tm.save_json(out/'plan.json',plan)
        duration=0; previous=baseline
        for q in route+[baseline]:
            delta=distance(previous,q)
            duration+=max(.05,1.875*delta/cfg['transit_speed_rad_s'],math.sqrt(5.774*delta/cfg['transit_accel_rad_s2']))
            previous=q
        duration+=50*(cfg['stable_seconds']+cfg['sample_seconds'])
        tm.save_json(out/'coverage.json',{'nominal_span_deg':spans,'rejections':rejections,
                     'pose_count':50,'approximate_duration_minutes':duration/60,
                     'maximum_offset_deg':args.max_angle_deg,'seed':20261015})
        print('50 poses prepared. Nominal span J1-J7 (deg):',[round(x,2) for x in spans],flush=True)
        print(f'Estimated duration: {duration/60:.1f} minutes + extra settling/end hold.',flush=True)
        print('Targets:',out/'poses_50.csv',flush=True)
        return 0
    except (ValueError,KeyError,OSError,TypeError,ImportError) as exc:
        parser.exit(1,f'ERROR: {exc}\n')


if __name__=='__main__':
    raise SystemExit(main())
