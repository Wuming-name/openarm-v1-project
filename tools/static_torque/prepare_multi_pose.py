#!/usr/bin/env python3
"""Screen larger single-joint poses without increasing existing effort/gain alarms.

Uses previous measured excursions and a fresh supported/disabled diagnosis.
Offline only: creates a plan, never opens CAN or moves hardware.
"""
import argparse
import csv
import math
from pathlib import Path
import statistics
import torque_model as tm
import prepare_local_pilot as pp


def previous_motion(run):
    meta=tm.read_json(run/'metadata.json')
    if meta.get('mode')!='static_motion' or not meta.get('completed'):
        raise ValueError('Previous acquisition is not completed. If still holding, support the arm and enter SUPPORTED first.')
    groups={}
    with (run/'samples.csv').open(encoding='utf-8',newline='') as f:
        for row in csv.DictReader(f):
            if row['stage']=='sample' and row['stable']=='1' and all(row[f'status{i}']=='1' for i in range(1,8)):
                groups.setdefault(row['pose_id'],[]).append(row)
    results=[]
    for i in range(7):
        before='base_initial' if i==0 else f'base_after_j{i}'
        pose=f'j{i+1}_plus_1deg'
        a=groups.get(before,[]); b=groups.get(pose,[])
        if min(len(a),len(b))<40:
            results.append({'joint':i+1,'eligible':False,'reason':'missing prior 1deg/base samples'})
            continue
        mean=lambda rows,name: statistics.fmean(float(r[name]) for r in rows)
        delta=mean(b,f'q{i+1}')-mean(a,f'q{i+1}')
        results.append({'joint':i+1,'eligible':delta>=math.radians(0.25),
                        'actual_previous_delta_deg':math.degrees(delta),
                        'torque_before_nm':mean(a,f'tau{i+1}'),'torque_after_nm':mean(b,f'tau{i+1}'),
                        'reason':'ok' if delta>=math.radians(0.25) else 'previous actual positive excursion <0.25deg'})
    return results


def screen(model,cfg,baseline,target):
    steps=max(1,math.ceil(max(abs(x-y) for x,y in zip(baseline,target))/0.002))
    peak=[0.0]*7; nominal_pd_error=[0.0]*7
    try:
        for n in range(steps+1):
            q=[x+(y-x)*n/steps for x,y in zip(baseline,target)]
            gravity=model.check_point(q,cfg)
            for i,g in enumerate(gravity):
                peak[i]=max(peak[i],abs(g))
                nominal_pd_error[i]=max(nominal_pd_error[i],abs(g)/cfg['kp'][i])
        # A deliberately conservative nominal check; friction/feedback/model errors
        # can still prevent settling. Never enlarge settle/effort thresholds to pass.
        if max(nominal_pd_error)>cfg['settle_position_error_rad']:
            raise ValueError('nominal gravity/KP exceeds holding-position tolerance; needs separate holding-control review')
        return True,'ok',peak,nominal_pd_error
    except ValueError as exc:
        return False,str(exc),peak,nominal_pd_error


def eligible_for_j7_probe(row):
    # Separate diagnostic scope, NOT general permission to enlarge all joints.
    # J7 previously showed measurable positive movement AND an effort response.
    return (row.get('joint')==7 and row.get('actual_previous_delta_deg',0)>=0.05 and
            row.get('torque_after_nm',0)-row.get('torque_before_nm',0)>=0.02)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous-run',required=True,type=Path)
    parser.add_argument('--report-only',action='store_true')
    parser.add_argument('--probe-j7',action='store_true',
                        help='Diagnostic J7 +3deg only, with unchanged alarms/gains; not a general multi-pose approval')
    parser.add_argument('--snapshot',default='latest')
    parser.add_argument('--log-root',type=Path,default=Path.home()/'experiments/torque_compare')
    parser.add_argument('--model',type=Path,default=Path.home()/'openarm_v1_ws/src/openarm-v1-project/models/openarm_v1_right/openarm_v1_7dof.xml')
    parser.add_argument('--max-angle-deg',type=int,choices=(5,10,15),default=10)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    try:
        prior=previous_motion(args.previous_run.expanduser().resolve(strict=True))
        print('Previous actual excursions (one-degree TARGETS are not proof of one-degree actual motion):')
        for row in prior:
            if 'actual_previous_delta_deg' in row:
                print(f"  J{row['joint']}: {row['actual_previous_delta_deg']:.3f}deg; feedback torque "
                      f"{row['torque_before_nm']:.4f} -> {row['torque_after_nm']:.4f} Nm; {row['reason']}")
            else: print(f"  J{row['joint']}: {row['reason']}")
        if args.report_only: return 0
        if args.out is None: raise ValueError('supply --out for a new plan directory')
        snapshot,_,baseline=pp.fresh_snapshot(args.snapshot,args.log_root)
        import mujoco
        model_path=args.model.expanduser().resolve(strict=True)
        raw=mujoco.MjModel.from_xml_path(str(model_path))
        ids=[mujoco.mj_name2id(raw,mujoco.mjtObj.mjOBJ_JOINT,f'openarm_right_joint{i}') for i in range(1,8)]
        if raw.nq!=7 or raw.nv!=7 or any(j<0 or not raw.jnt_limited[j] for j in ids):
            raise ValueError('requires fixed-base seven-joint model with finite model ranges')
        low=[float(raw.jnt_range[j,0]) for j in ids]; high=[float(raw.jnt_range[j,1]) for j in ids]
        base=tm.read_json(Path(__file__).parent/'safety.template.json')
        cfg=pp.make_config(base,model_path,baseline,low,high)
        cfg.update(experiment_scope='screened_multi_pose_15deg',
                   q_min_rad=[max(low[i],baseline[i]-0.35) for i in range(7)],
                   q_max_rad=[min(high[i],baseline[i]+0.35) for i in range(7)])
        tm.validate_config(cfg)
        model=tm.Model(cfg,model_path)
        model.check_point(baseline,cfg)
        angles=[3] if args.probe_j7 else [d for d in (3,5,10,15) if d<=args.max_angle_deg]
        poses=[{'pose_id':'multi_base_initial','q_motor_rad':baseline.copy()}]
        assessment=[]
        for i in range(7):
            if args.probe_j7 and i!=6: continue
            diagnostic_exception=args.probe_j7 and eligible_for_j7_probe(prior[i])
            if not prior[i]['eligible'] and not diagnostic_exception:
                print(f'J{i+1} skipped: prior test did not verify actual movement.'); continue
            if args.probe_j7:
                print('J7 DIAGNOSTIC ONLY: +3deg; earlier 0.229deg was insufficient for general multi-pose approval.')
                print('KP/KD, speed, tracking and effort alarms unchanged. Other joint TARGETS remain fixed.')
            for direction in ((1,) if args.probe_j7 else (-1,1)):
                best=None
                for angle in angles:
                    target=baseline.copy(); target[i]+=direction*math.radians(angle)
                    ok,reason,peak,error=screen(model,cfg,baseline,target)
                    assessment.append({'joint':i+1,'direction':direction,'angle_deg':angle,'accepted':ok,
                                       'reason':reason,'peak_gravity_nm':peak,'nominal_pd_error_rad':error})
                    if ok: best=(angle,target)
                if best:
                    angle,target=best
                    label=f'j{i+1}_{"plus" if direction>0 else "minus"}_{angle}deg'
                    poses.append({'pose_id':label,'q_motor_rad':target})
                    poses.append({'pose_id':'return_after_'+label,'q_motor_rad':baseline.copy()})
                    print(f'J{i+1}: selected {direction*angle:+d}deg, then return to start.')
                else:
                    print(f'J{i+1} direction {direction:+d}: no candidate passed existing limits; not enlarged.')
        if len(poses)==1: raise ValueError('No candidate passed. Review prior actual motion and holding control; do not increase alarms blindly.')
        out=args.out.expanduser().resolve(); out.mkdir(parents=True,exist_ok=False)
        frozen=out/'model.mjb'; mujoco.mj_saveModel(model.model,str(frozen),None)
        tm.save_json(out/'multi_config.json',cfg); tm.save_json(out/'candidate_assessment.json',assessment)
        tm.save_json(out/'previous_actual_motion.json',prior)
        with (out/'multi_poses.csv').open('x',encoding='utf-8',newline='') as f:
            w=csv.writer(f); w.writerow(['pose_id']+[f'q{i}_rad' for i in range(1,8)])
            for pose in poses: w.writerow([pose['pose_id']]+pose['q_motor_rad'])
        plan={'schema':'openarm-static-v1','model_path_checked':True,'config':cfg,
              'baseline_q_motor_rad':baseline,'poses':poses,'frozen_model_file':'model.mjb',
              'frozen_model_sha256':tm.sha256(frozen),'model_xml_path':str(model_path),
              'model_xml_sha256':tm.sha256(model_path),'mujoco_version':mujoco.__version__,
              'source_snapshot':str(snapshot),'previous_run':str(args.previous_run),
              'diagnostic_j7_start_probe':bool(args.probe_j7),
              'config_sha256':tm.sha256(out/'multi_config.json'),'poses_sha256':tm.sha256(out/'multi_poses.csv'),
              'zero_calibration_status':'unknown accuracy; no numerical uncertainty bound established',
              'limitations':'screened single-joint excursions only; physical limits/masses/zero not calibrated; collision exclusions remain active'}
        tm.save_json(out/'plan.json',plan)
        duration=len(poses)*4
        previous=baseline
        for pose in poses+[{'q_motor_rad':baseline}]:
            delta=max(abs(x-y) for x,y in zip(previous,pose['q_motor_rad']))
            duration+=max(0.05,1.875*delta/cfg['transit_speed_rad_s'],math.sqrt(5.774*delta/cfg['transit_accel_rad_s2']))
            previous=pose['q_motor_rad']
        print(f'PREPARED {len(poses)} sample poses. Approximate duration {duration/60:.1f} minutes, excluding extra settling/end hold.')
        print('Alarms/KP/KD unchanged from successful pilot. No CAN access; no motion.')
        print('Review candidate_assessment.json and multi_poses.csv before execution.')
        print('Plan:',out/'plan.json')
        print(f'./build/openarm_static_torque --interface can0 --plan "{out / "plan.json"}" --execute')
        return 0
    except (ValueError,KeyError,OSError,TypeError,ImportError) as exc:
        parser.exit(1,f'ERROR: {exc}\n')


if __name__=='__main__':
    raise SystemExit(main())
