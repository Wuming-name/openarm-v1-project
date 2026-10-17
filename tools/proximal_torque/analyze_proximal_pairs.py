#!/usr/bin/env python3
"""Paired direction diagnostics at measured configurations, not friction calibration."""
import argparse
import csv
import math
from pathlib import Path
import statistics
import torque_model as tm
from analyze_directional import write_analysis


def pair_rows(plan,summary,angle_tolerance_deg=.5):
    indexed={(row['pose_id'],row['joint']):row for row in summary}
    groups={};patterns={}
    for pose in plan['poses']:
        key=(pose['base_pose_id'],pose['repetition'])
        if plan.get('config',{}).get('approach_direction_policy')=='per_base_opposed_vectors' and 'approach_axis_sign' not in pose:
            raise ValueError('Missing per-axis approach directions')
        pattern=pose.get('approach_axis_sign',[1]*7)
        if len(pattern)!=7 or any(type(x) not in (int,float) or x not in (-1,1) for x in pattern):
            raise ValueError('Invalid per-axis approach directions')
        if key in patterns and patterns[key]!=pattern:raise ValueError('Paired per-axis directions differ')
        patterns[key]=pattern
        if pose['approach_command_sign'] in groups.setdefault(key,{}):
            raise ValueError('Duplicate acquisition direction')
        groups[key][pose['approach_command_sign']]=pose['pose_id']
    rows=[]
    for (base,repeat),group in groups.items():
        if set(group)!={-1,1}:raise ValueError('Incomplete directional pair')
        inc=[indexed[(group[1],f'J{i}')] for i in range(1,8)]
        dec=[indexed[(group[-1],f'J{i}')] for i in range(1,8)]
        mismatch=math.degrees(max(abs(float(a['q_mean_rad'])-float(b['q_mean_rad'])) for a,b in zip(inc,dec)))
        for i,(a,b) in enumerate(zip(inc,dec),1):
            pos_id,neg_id=group[1],group[-1]
            if patterns[(base,repeat)][i-1]==-1:
                a,b=b,a;pos_id,neg_id=neg_id,pos_id
            ta=float(a['feedback_mean_nm']);tb=float(b['feedback_mean_nm'])
            ga=float(a['gravity_mean_nm']);gb=float(b['gravity_mean_nm'])
            ra=ta-ga;rb=tb-gb
            rows.append({'base_pose_id':base,'repetition':repeat,'joint':f'J{i}',
                         'inc_pose_id':pos_id,'dec_pose_id':neg_id,
                         'q_inc_deg':math.degrees(float(a['q_mean_rad'])),'q_dec_deg':math.degrees(float(b['q_mean_rad'])),
                         'max_seven_joint_pair_mismatch_deg':mismatch,'angles_matched':mismatch<=angle_tolerance_deg,
                         'torque_inc_nm':ta,'torque_dec_nm':tb,'model_inc_nm':ga,'model_dec_nm':gb,
                         'paired_motor_mean_nm':(ta+tb)/2,'residual_even_nm':(ra+rb)/2,
                         'residual_odd_nm':(ra-rb)/2})
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args()
    try:
        root=args.run.expanduser().resolve(strict=True)
        meta=tm.read_json(root/'metadata.json');report=tm.read_json(root/'comparison/report.json')
        plan=meta['plan']
        if plan['config']['experiment_scope']!=tm.PROXIMAL_SCOPE or not report['dataset_complete']:
            raise ValueError('Requires a complete proximal paired dataset')
        with (root/'comparison/summary.csv').open(encoding='utf-8',newline='') as stream:
            rows=pair_rows(plan,list(csv.DictReader(stream)))
        out=root/'comparison'
        with (out/'paired_directions.csv').open('x',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        stats=[]
        for i in range(1,8):
            joint=[r for r in rows if r['joint']==f'J{i}']
            stats.append({'joint':f'J{i}','pairs':len(joint),'matched_angle_pairs':sum(r['angles_matched'] for r in joint),
                          'residual_even_mean_nm':statistics.fmean(r['residual_even_nm'] for r in joint),
                          'residual_even_rmse_nm':math.sqrt(statistics.fmean(r['residual_even_nm']**2 for r in joint)),
                          'residual_odd_mean_nm':statistics.fmean(r['residual_odd_nm'] for r in joint)})
        tm.save_json(out/'paired_report.json',{'joint_summary':stats,'angle_match_threshold_deg':.5,
                     'validation_only':report.get('validation_only',False),
                     'full_proximal_coverage_complete':report.get('full_proximal_coverage_complete',False),
                     'actual_proximal_span_deg':report['proximal_measured_span_deg'],
                     'interpretation':'Even/odd residual components are diagnostics using model gravity at EACH actual configuration. '
                                      'Motor mean is not certified gravity. Unmatched angles, friction asymmetry and torque scaling prevent direct calibration.'})
        print('Paired diagnostics:',out/'paired_directions.csv',flush=True)
        write_analysis(rows,out,plan['config'],report.get('validation_only',False))
        if plan['config'].get('trim_j2',False):
            print('J2 trim changes approach history: directional results are protocol groups, not pure friction identification.')
        return 0
    except (ValueError,KeyError,OSError,TypeError) as exc:
        parser.exit(1,f'ERROR: {exc}\n')


if __name__=='__main__':raise SystemExit(main())
