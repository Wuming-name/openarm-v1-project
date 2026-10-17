#!/usr/bin/env python3
"""Offline direction-stratified diagnostics. Never sends CAN commands."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics as st


def metrics(x,y):
    error=[b-a for a,b in zip(x,y)]
    r=None
    if len(x)>=3:
        dx=[a-st.fmean(x) for a in x];dy=[b-st.fmean(y) for b in y]
        den=math.sqrt(sum(a*a for a in dx)*sum(b*b for b in dy))
        if den>1e-12:r=sum(a*b for a,b in zip(dx,dy))/den
    return {'bases':len(x),'pearson_r':r,
            'MAE_nm':st.fmean(abs(e) for e in error),
            'RMSE_nm':math.sqrt(st.fmean(e*e for e in error)),
            'bias_nm':st.fmean(error),'model_span_nm':max(x)-min(x)}


def direction_stats(rows):
    result=[]
    for joint in sorted({r['joint'] for r in rows}):
        rr=[r for r in rows if r['joint']==joint]
        for direction in ('inc','dec'):
            # pair_rows already corrects each joint's axis sign. Do not use
            # acquisition label alone: a mixed vector can reverse that joint.
            groups={}
            for row in rr:groups.setdefault(row['base_pose_id'],[]).append(row)
            x=[st.fmean(float(r[f'model_{direction}_nm']) for r in group) for group in groups.values()]
            y=[st.fmean(float(r[f'torque_{direction}_nm']) for r in group) for group in groups.values()]
            result.append(dict(joint=joint,actual_approach_direction=direction,**metrics(x,y),
                               matched_pairs=sum(str(r['angles_matched']).lower()=='true' for r in rr),
                               total_pairs=len(rr),
                               interpretation='diagnostic only; r omitted for <3 bases or constant values'))
    return result


def write_analysis(rows,out,cfg=None,validation_only=False):
    out=Path(out);cfg=cfg or {}
    stats=direction_stats(rows)
    for row in stats:
        row['reference_trim_enabled']=bool(cfg.get('trim_j2',False))
        if cfg.get('trim_j2',False):
            row['interpretation']='J2 trim enabled; nominal approach groups, not pure directional friction identification'
    with (out/'directional_metrics.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(stats[0]));writer.writeheader();writer.writerows(stats)
    joints={}
    for joint in sorted({r['joint'] for r in rows}):
        rr=[r for r in rows if r['joint']==joint]
        q=[float(r[f'q_{d}_deg']) for r in rr for d in ('inc','dec')]
        g=[float(r[f'model_{d}_nm']) for r in rr for d in ('inc','dec')]
        joints[joint]={'actual_angle_min_deg':min(q),'actual_angle_max_deg':max(q),
                       'actual_angle_span_deg':max(q)-min(q),
                       'model_at_actual_pose_span_nm':max(g)-min(g),
                       'max_own_joint_pair_mismatch_deg':max(abs(float(r['q_inc_deg'])-float(r['q_dec_deg'])) for r in rr)}
    report={'validation_only':validation_only,'joint_coverage':joints,
            'reference_trim_enabled':bool(cfg.get('trim_j2',False)),
            'requested_nominal_j4_span_deg':cfg.get('minimum_j4_nominal_span_deg'),
            'requested_nominal_j3_model_span_nm':cfg.get('minimum_j3_model_span_nm'),
            'matched_pair_rows':sum(str(r['angles_matched']).lower()=='true' for r in rows),
            'total_pair_rows':len(rows),
            'notes':['Nominal planning requirements do not certify actual coverage.',
                     'Repeated acquisitions are averaged per base within each joint direction.',
                     'Direction labels reflect each joint axis, not whole-arm pose labels.',
                     'No friction cancellation or torque calibration is claimed.',
                     'Existing dataset completion flags are not changed.']}
    (out/'directional_report.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print('Direction-separated metrics:',out/'directional_metrics.csv')
    print('Actual J4 span:',round(joints['J4']['actual_angle_span_deg'],3),'deg; model J3 span at actual poses:',
          round(joints['J3']['model_at_actual_pose_span_nm'],3),'Nm')
    if report['matched_pair_rows']<len(rows):
        print('Paired actual angles are unmatched: even/odd components are diagnostics, not friction-corrected gravity.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--comparison-dir',type=Path,required=True)
    a=p.parse_args();root=a.comparison_dir.expanduser().resolve(strict=True)
    with (root/'paired_directions.csv').open(encoding='utf-8-sig',newline='') as f:rows=list(csv.DictReader(f))
    report=json.loads((root/'report.json').read_text(encoding='utf-8'))
    if not rows:raise ValueError('No paired rows')
    cfg={}
    meta=root.parent/'metadata.json'
    if meta.exists():cfg=json.loads(meta.read_text(encoding='utf-8'))['plan']['config']
    write_analysis(rows,root,cfg,report.get('validation_only',False))


if __name__=='__main__':main()
