#!/usr/bin/env python3
"""Offline adaptive shoulder coverage with paired, explicitly checked paths."""
import argparse
import copy
from collections import Counter
import csv
import itertools
import math
from pathlib import Path
import torque_model as tm
import prepare_local_pilot as pp
import prepare_static_50 as old


def linspace(low,high,count=5):
    return [low+(high-low)*i/(count-1) for i in range(count)]


class CheckedModel:
    def __init__(self,model,cfg):
        self.model=model;self.cfg=cfg;self.cache={};self.rejections=Counter()
    def point(self,q):
        key=tuple(round(x,10) for x in q)
        if key not in self.cache:
            try:
                self.cache[key]=(True,self.model.check_point(q,self.cfg))
            except ValueError as exc:
                self.cache[key]=(False,str(exc))
                self.rejections[str(exc)]+=1
        ok,value=self.cache[key]
        if not ok: raise ValueError(value)
        return value
    def path(self,start,end):
        count=max(1,math.ceil(old.distance(start,end)/.015))
        for i in range(count+1):
            self.point([a+(b-a)*i/count for a,b in zip(start,end)])


def make_config(template,path,q,low,high,min_span,approach,repeats,torque_profile='legacy_low'):
    cfg=old.make_config(template,path,q,low,high,30)
    cfg.pop('max_excursion_rad')
    cfg.update(experiment_scope=tm.PROXIMAL_SCOPE,
               max_excursion_per_joint_rad=list(map(math.radians,[60,75,30,90,0,0,0])),
               approach_offset_rad=[math.radians(approach)]*7,
               minimum_proximal_actual_span_rad=math.radians(min_span),
               minimum_actual_approach_rad=math.radians(1),
               approach_position_error_rad=.10,
               return_position_error_rad=.08,
               repetitions=repeats,requested_acquisition_count=100*repeats)
    if torque_profile not in ('legacy_low','proximal_5_3'):
        raise ValueError('Unknown torque profile')
    cfg['torque_profile']=torque_profile
    if torque_profile=='proximal_5_3':cfg['torque_alarm_nm']=[5,5,3,3,.7,.7,.7]
    tm.validate_config(cfg)
    return cfg


def elbow_modes(cfg,q):
    modes=[]
    for j3,j4 in [(0,x) for x in (0,20,40,60,80)]+[(-20,60),(20,60)]:
        candidate=list(q)
        candidate[2]+=math.radians(j3); candidate[3]+=math.radians(j4)
        if all(cfg['q_min_rad'][i]+.05+cfg['approach_offset_rad'][i]<=candidate[i]<=
               cfg['q_max_rad'][i]-.05-cfg['approach_offset_rad'][i] for i in (2,3)):
            modes.append(candidate)
    if len(modes)<2: raise ValueError('Not enough elbow modes have approach clearance.')
    return modes


def candidate_centers(low,high,span,preferred):
    if high-low<span-1e-9: return []
    a=low+span/2;b=high-span/2
    values=[min(max(preferred,a),b),a,b,(a+b)/2]
    return list(dict.fromkeys(round(x,10) for x in values))


def make_base_grid(q,levels1,levels2,modes):
    rows=[]
    for k,mode in enumerate(modes):
        indices=[(i,j) for i in range(5) for j in (range(5) if i%2==0 else range(4,-1,-1))]
        if k%2: indices.reverse()
        for i,j in indices:
            target=list(mode);target[0]=levels1[i];target[1]=levels2[j]
            rows.append({'base_pose_id':f'base_{k*25+i*5+j+1:03d}',
                         'grid_j1':i,'grid_j2':j,'elbow_mode':k,'q_motor_rad':target})
    return rows


def acquisitions(bases,cfg):
    rows=[]
    for base in bases:
        q=base['q_motor_rad']
        pattern=base.get('approach_axis_sign',[1]*7)
        for repeat in range(1,cfg['repetitions']+1):
            for sign,label in ((1,'inc'),(-1,'dec')):
                rows.append(dict(base,pose_id=f"{base['base_pose_id']}_{label}_r{repeat}",
                                 repetition=repeat,approach_command_sign=sign,
                                 pre_q_motor_rad=[x-sign*s*d for x,s,d in zip(q,pattern,cfg['approach_offset_rad'])]))
    return rows


def select_grid(model,cfg,q,max_span,progress=False):
    checked=CheckedModel(model,cfg);checked.point(q)
    modes=elbow_modes(cfg,q)
    pairs=[pair for pair in itertools.combinations(modes,2)
           if old.distance(*pair)>=math.radians(19.9)]
    pairs.sort(key=lambda pair:-old.distance(*pair))
    margin=cfg['path_margin_rad']+cfg['startup_tolerance_rad']
    bounds=[]
    for i in (0,1):
        extra=cfg['approach_offset_rad'][i]
        bounds.append((max(cfg['q_min_rad'][i]+margin+extra,q[i]-cfg['max_excursion_per_joint_rad'][i]),
                       min(cfg['q_max_rad'][i]-margin-extra,q[i]+cfg['max_excursion_per_joint_rad'][i])))
    # Worst-case capture errors at both ends must not erase required actual span.
    required=cfg['minimum_proximal_actual_span_rad']+2*cfg['static_capture_error_rad']
    degrees=[x for x in (90,75,60,45,30) if x<=max_span and math.radians(x)>=required]
    combinations=sorted(itertools.product(degrees,repeat=2),key=lambda x:(-min(x),-(x[0]*x[1])))
    if not combinations: raise ValueError('Requested actual coverage leaves no admissible nominal grid span.')
    attempts=0
    for span1,span2 in combinations:
        centers1=candidate_centers(*bounds[0],math.radians(span1),q[0])
        centers2=candidate_centers(*bounds[1],math.radians(span2),q[1])
        if not centers1 or not centers2: continue
        if progress: print(f'Screening J1/J2 grid spans {span1}/{span2} deg; motor torque alarms unchanged.',flush=True)
        for c1,c2 in itertools.product(centers1,centers2):
            levels1=linspace(c1-math.radians(span1)/2,c1+math.radians(span1)/2)
            levels2=linspace(c2-math.radians(span2)/2,c2+math.radians(span2)/2)
            for pair in pairs:
                attempts+=1
                bases=make_base_grid(q,levels1,levels2,pair)
                samples=acquisitions(bases,cfg)
                try:
                    # First reject endpoints cheaply before traversing paths.
                    gravity=[checked.point(b['q_motor_rad']) for b in bases]
                    for sample in samples: checked.point(sample['pre_q_motor_rad'])
                    current=list(q)
                    for sample in samples:
                        pre=sample['pre_q_motor_rad']; target=sample['q_motor_rad']
                        checked.path(current,pre);checked.path(pre,target)
                        current=target
                    checked.path(current,q)
                except ValueError:
                    continue
                coverage={'nominal_j1_span_deg':span1,'nominal_j2_span_deg':span2,
                          'minimum_required_actual_span_deg':math.degrees(cfg['minimum_proximal_actual_span_rad']),
                          'nominal_ranges_deg':[[math.degrees(min(b['q_motor_rad'][i] for b in bases)),
                                                math.degrees(max(b['q_motor_rad'][i] for b in bases))] for i in range(7)],
                          'model_gravity_ranges_nm':[[min(g[i] for g in gravity),max(g[i] for g in gravity)] for i in range(7)],
                          'candidate_grids_tried':attempts,'cached_path_points':len(checked.cache),
                          'rejections':dict(checked.rejections)}
                return bases,samples,coverage
    worst=checked.rejections.most_common(8)
    error=ValueError(f'No complete 5x5x2 grid meets proximal coverage and both approach paths; grids tried={attempts}; '
                     f'primary model rejections={worst}. No executable plan created. '
                     'Selected motor torque alarms may exclude high-load shoulder postures; review rated loads and geometry instead of bypassing checks.')
    error.assessment={'candidate_grids_tried':attempts,'rejections':dict(checked.rejections),
                      'nominal_shoulder_bounds_deg':[[math.degrees(x) for x in b] for b in bounds],
                      'minimum_actual_span_deg':math.degrees(cfg['minimum_proximal_actual_span_rad']),
                      'max_searched_span_deg':max_span,'config':cfg,'executable_plan_created':False}
    raise error


def select_adaptive(model,cfg,q,max_span,progress=False):
    """Choose 50 separate feasible configurations, rather than a Cartesian grid.

    Endpoint feasibility is not path feasibility. Every selected candidate must
    connect to the baseline in both directions and admit both approach paths.
    Baseline routing is a planned normal transit, never a fault recovery.
    """
    cfg['proximal_layout']='adaptive_coverage'
    cfg['approach_direction_policy']='per_base_opposed_vectors'
    tm.validate_config(cfg)
    checked=CheckedModel(model,cfg)
    checked.point(q)
    margin=cfg['path_margin_rad']+cfg['startup_tolerance_rad']
    bounds=[]
    for i in (0,1):
        extra=cfg['approach_offset_rad'][i]
        low=max(cfg['q_min_rad'][i]+margin+extra+1e-8,q[i]-cfg['max_excursion_per_joint_rad'][i])
        high=min(cfg['q_max_rad'][i]-margin-extra-1e-8,q[i]+cfg['max_excursion_per_joint_rad'][i])
        width=min(math.radians(max_span),high-low)
        if width<=0: raise ValueError(f'J{i+1}: no shoulder range with approach clearance')
        center=min(max(q[i],low+width/2),high-width/2)
        bounds.append((center-width/2,center+width/2))
    modes=[]
    for d3,d4 in itertools.product(range(-30,31,10),range(-80,81,5)):
        mode=list(q);mode[2]+=math.radians(d3);mode[3]+=math.radians(d4)
        if all(cfg['q_min_rad'][i]+margin+cfg['approach_offset_rad'][i]<=mode[i]<=
               cfg['q_max_rad'][i]-margin-cfg['approach_offset_rad'][i] and
               abs(mode[i]-q[i])<=cfg['max_excursion_per_joint_rad'][i]+1e-9 for i in (2,3)):
            modes.append(mode)
    patterns=sorted([(1,)+tail for tail in itertools.product((-1,1),repeat=6)],key=lambda s:s.count(-1))
    pool=[];attempts=0;target_count=0;all_same_sign_count=0;stage_rejections=Counter()
    limits=[cfg['gravity_alarm_fraction']*x for x in cfg['torque_alarm_nm']]
    def endpoint_options(target,exclude=None):
        for pattern in patterns:
            if pattern==exclude:continue
            pre=[[x-sign*s*d for x,s,d in zip(target,pattern,cfg['approach_offset_rad'])] for sign in (1,-1)]
            try:gs=[checked.point(point) for point in pre]
            except ValueError as exc:
                stage_rejections['approach endpoint: '+str(exc)]+=1;continue
            utilization=max(abs(g[i])/limits[i] for g in gs for i in range(7))
            yield {'pattern':pattern,'pre':pre,'utilization':utilization}
    for a in linspace(*bounds[0],17):
        for b,mode in itertools.product(linspace(*bounds[1],17),modes):
            attempts+=1;target=list(mode);target[0]=a;target[1]=b
            try:
                checked.point(target)
                if cfg.get('trim_j2',False):
                    # Reference corridor screening is nominal, not a physical
                    # clearance proof. Runtime also checks actual feedback.
                    for sign in (-1,1):
                        end=list(target);end[1]+=sign*.10
                        checked.path(target,end)
            except ValueError as exc:
                stage_rejections['target: '+str(exc)]+=1;continue
            target_count+=1
            option=next(endpoint_options(target),None)
            if option:
                if option['pattern']==(1,)*7:all_same_sign_count+=1
                pool.append({'q_motor_rad':target,'first_option':option})
        if progress: print(f'Adaptive endpoint screening {attempts}; target-only passed {target_count}; '
                           f'opposed-vector endpoints passed {len(pool)}.',flush=True)
    endpoint_pool=list(pool)
    selected=[];selected_g=[];path_trials=0
    j4_required=math.radians(cfg.get('minimum_j4_nominal_span_deg',0))
    g3_required=cfg.get('minimum_j3_model_span_nm',0)
    if not (0<=j4_required<=math.radians(90)) or not (0<=g3_required<=3):
        raise ValueError('Invalid J4/J3 experimental coverage requirements')
    required=cfg['minimum_proximal_actual_span_rad']+2*cfg['static_capture_error_rad']
    # Restore the controller's existing 5deg target separation. Capture error
    # is not a measured inter-pose displacement; C++ still requires distinct
    # ACTUAL configurations >=1deg and actual shoulder coverage after sampling.
    spacing=math.radians(5)
    def span(rows,i):
        values=[r['q_motor_rad'][i] for r in rows]
        return max(values)-min(values) if values else 0
    while pool and len(selected)<50:
        lows=[min((r['q_motor_rad'][i] for r in selected),default=0) for i in range(4)]
        highs=[max((r['q_motor_rad'][i] for r in selected),default=0) for i in range(4)]
        def score(row):
            if not selected:return (0,0,-old.distance(q,row['q_motor_rad']))
            v=row['q_motor_rad']
            spans=[max(highs[i],v[i])-min(lows[i],v[i]) for i in (0,1)]
            coverage_score=sum(min(s/required,1) for s in spans)
            if j4_required:
                coverage_score+=min((max(highs[3],v[3])-min(lows[3],v[3]))/j4_required,1)
            if g3_required:
                gs=[g[2] for g in selected_g]+[checked.point(v)[2]]
                coverage_score+=min((max(gs)-min(gs))/g3_required,1)
            return (coverage_score,row['nearest'],sum(spans))
        pool.sort(key=score,reverse=True)
        accepted=False
        while pool:
            row=pool.pop(0);target=row['q_motor_rad']
            path_trials+=1
            try:checked.path(target,q)
            except ValueError as exc:
                stage_rejections['candidate return path: '+str(exc)]+=1;continue
            chosen=None
            options=itertools.chain([row['first_option']],endpoint_options(target,row['first_option']['pattern']))
            for option in options:
                try:
                    for pre in option['pre']:
                        checked.path(q,pre);checked.path(pre,target)
                        checked.path(target,pre)
                except ValueError as exc:
                    stage_rejections['candidate path: '+str(exc)]+=1;continue
                chosen=option;break
            if chosen is None:continue
            row['approach_axis_sign']=list(chosen['pattern'])
            selected.append(row);selected_g.append(checked.point(target));accepted=True
            retained=[]
            for candidate in pool:
                candidate['nearest']=min(candidate.get('nearest',math.inf),old.distance(target,candidate['q_motor_rad']))
                if candidate['nearest']>=spacing-1e-9:retained.append(candidate)
            pool=retained
            if progress: print(f'Adaptive path-checked bases {len(selected)}/50; J1/J2 spans '
                               f'{math.degrees(span(selected,0)):.1f}/{math.degrees(span(selected,1)):.1f} deg.',flush=True)
            break
        if not accepted:break
    assessment={'layout':'adaptive_coverage','candidate_endpoints_tried':attempts,
                'target_only_pass_count':target_count,'all_same_sign_endpoint_pass_count':all_same_sign_count,
                'stage_rejections':dict(stage_rejections),
                'feasible_endpoint_count':len(endpoint_pool),'candidate_paths_tried':path_trials,
                'selected_base_count':len(selected),'rejections':dict(checked.rejections),
                'endpoint_only_shoulder_spans_deg':[math.degrees(span(endpoint_pool,i)) for i in (0,1)],
                'selected_path_checked_shoulder_spans_deg':[math.degrees(span(selected,i)) for i in (0,1)],
                'minimum_required_nominal_span_deg':math.degrees(required),
                'minimum_nominal_base_spacing_deg':math.degrees(spacing),
                'minimum_actual_span_deg':math.degrees(cfg['minimum_proximal_actual_span_rad']),
                'max_searched_span_deg':max_span,'config':cfg,'executable_plan_created':False}
    g3_span=max((g[2] for g in selected_g),default=0)-min((g[2] for g in selected_g),default=0)
    assessment.update(nominal_j4_span_deg=math.degrees(span(selected,3)),
                      model_j3_span_nm=g3_span,
                      required_j4_nominal_span_deg=math.degrees(j4_required),
                      required_j3_model_span_nm=g3_required,
                      endpoint_only_j4_span_deg=math.degrees(span(endpoint_pool,3)))
    if (len(selected)!=50 or any(span(selected,i)<required-1e-9 for i in (0,1))
            or span(selected,3)<j4_required-1e-9 or g3_span<g3_required-1e-9):
        error=ValueError(f'Adaptive search did not find 50 path-connected bases with required shoulder coverage; '
                         f'selected={len(selected)}, spans_deg={assessment["selected_path_checked_shoulder_spans_deg"]}, '
                         f'required_nominal_deg={math.degrees(required):.3f}; '
                         f'J4 nominal span={math.degrees(span(selected,3)):.2f}/{math.degrees(j4_required):.2f}deg; '
                         f'J3 model span={g3_span:.3f}/{g3_required:.3f}Nm; '
                         f'primary model rejections={checked.rejections.most_common(8)}. No executable plan created.')
        error.assessment=assessment;raise error
    # Order chosen configurations by nearest transit; recheck every actual leg.
    bases=[];current=list(q);remaining=list(selected)
    while remaining:
        row=min(remaining,key=lambda x:old.distance(current,x['q_motor_rad']))
        remaining.remove(row)
        bases.append({'base_pose_id':f'base_{len(bases)+1:03d}','q_motor_rad':row['q_motor_rad'],
                      'approach_axis_sign':row['approach_axis_sign']})
        current=row['q_motor_rad']
    samples=acquisitions(bases,cfg);current=list(q);via_count=0
    for row in samples:
        pre=row['pre_q_motor_rad'];target=row['q_motor_rad']
        try:checked.path(current,pre)
        except ValueError:
            checked.path(current,q);checked.path(q,pre)
            row['via_start']=True;via_count+=1
        checked.path(pre,target);current=target
    checked.path(current,q)
    coverage=dict(assessment,executable_plan_created=True,
                  nominal_j1_span_deg=math.degrees(span(selected,0)),
                  nominal_j2_span_deg=math.degrees(span(selected,1)),
                  minimum_required_actual_span_deg=math.degrees(cfg['minimum_proximal_actual_span_rad']),
                  nominal_ranges_deg=[[math.degrees(min(b['q_motor_rad'][i] for b in bases)),
                                       math.degrees(max(b['q_motor_rad'][i] for b in bases))] for i in range(7)],
                  model_gravity_ranges_nm=[[min(g[i] for g in selected_g),max(g[i] for g in selected_g)] for i in range(7)],
                  planned_baseline_transits=via_count,cached_path_points=len(checked.cache),
                  rejections=dict(checked.rejections))
    return bases,samples,coverage


def save_plan(model,cfg,model_path,snapshot,q,bases,samples,coverage,out,mujoco):
    out.mkdir(parents=True,exist_ok=False)
    frozen=out/'model.mjb';mujoco.mj_saveModel(model.model,str(frozen),None)
    tm.save_json(out/'config_proximal.json',cfg)
    for filename,rows in [('base_poses_50.csv',bases),('acquisitions.csv',samples)]:
        with (out/filename).open('x',encoding='utf-8',newline='') as stream:
            writer=csv.writer(stream)
            writer.writerow(['base_pose_id','pose_id','approach_sign','repetition']+[f'q{i}_deg' for i in range(1,8)]+
                            [f'approach_axis_sign{i}' for i in range(1,8)]+['via_start']+
                            [f'pre_q{i}_deg' for i in range(1,8)])
            for row in rows:
                writer.writerow([row['base_pose_id'],row.get('pose_id',''),row.get('approach_command_sign',''),
                                 row.get('repetition','')]+[math.degrees(x) for x in row['q_motor_rad']]+
                                row.get('approach_axis_sign',[1]*7)+[row.get('via_start',False)]+
                                ([math.degrees(x) for x in row['pre_q_motor_rad']] if 'pre_q_motor_rad' in row else ['']*7))
    plan={'schema':'openarm-static-v1','model_path_checked':True,'config':cfg,
          'baseline_q_motor_rad':q,'poses':samples,'base_poses':bases,'coverage':coverage,
          'frozen_model_file':'model.mjb','frozen_model_sha256':tm.sha256(frozen),
          'model_xml_path':str(model_path),'model_xml_sha256':tm.sha256(model_path),
          'mujoco_version':mujoco.__version__,'source_snapshot':str(snapshot),
          'zero_calibration_status':'unknown; no numerical uncertainty bound',
          'limitations':'Nominal model screening only; actual zero/mass/geometry and motor-output torque scale unverified.'}
    duration=0;current=list(q)
    for row in samples:
        waypoints=([q] if row.get('via_start') else [])+[row['pre_q_motor_rad'],row['q_motor_rad']]
        for target in waypoints:
            d=old.distance(current,target)
            duration+=max(.05,1.875*d/cfg['transit_speed_rad_s'],math.sqrt(5.774*d/cfg['transit_accel_rad_s2']))
            current=target
        duration+=(2+int(bool(row.get('via_start'))))*cfg['stable_seconds']+cfg['sample_seconds']
    d=old.distance(current,q)
    duration+=max(.05,1.875*d/cfg['transit_speed_rad_s'],math.sqrt(5.774*d/cfg['transit_accel_rad_s2']))
    coverage['approximate_duration_minutes']=duration/60
    tm.save_json(out/'coverage.json',coverage);tm.save_json(out/'plan.json',plan)
    return duration/60


def short_validation(cfg,bases,samples,coverage,model):
    """Keep the first two complete directional pairs, never claim 50-base coverage."""
    original_coverage=copy.deepcopy(coverage)
    cfg.update(validation_only=True,requested_pose_count=2,requested_acquisition_count=4*cfg['repetitions'])
    tm.validate_config(cfg)
    chosen=bases[:2];ids={b['base_pose_id'] for b in chosen}
    rows=[p for p in samples if p['base_pose_id'] in ids]
    if len(chosen)!=2 or len(rows)!=4*cfg['repetitions']:raise ValueError('Incomplete two-base validation prefix')
    g=[model.check_point(b['q_motor_rad'],cfg) for b in chosen]
    result={'validation_only':True,'full_search_coverage':original_coverage,
            'nominal_ranges_deg':[[math.degrees(min(b['q_motor_rad'][i] for b in chosen)),
                                  math.degrees(max(b['q_motor_rad'][i] for b in chosen))] for i in range(7)],
            'model_gravity_ranges_nm':[[min(x[i] for x in g),max(x[i] for x in g)] for i in range(7)],
            'planned_baseline_transits':sum(bool(r.get('via_start')) for r in rows),
            'minimum_actual_shoulder_span_enforced':False}
    for i in (0,1):result[f'nominal_j{i+1}_span_deg']=result['nominal_ranges_deg'][i][1]-result['nominal_ranges_deg'][i][0]
    return chosen,rows,result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot',required=True)
    parser.add_argument('--model',required=True,type=Path)
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--max-proximal-span-deg',type=int,choices=[45,60,75,90],default=90)
    parser.add_argument('--min-proximal-span-deg',type=int,choices=[20,30,40,45],default=30)
    parser.add_argument('--approach-deg',type=int,choices=[10,12,15],default=12)
    parser.add_argument('--repeats',type=int,choices=[1,2,3],default=1)
    parser.add_argument('--torque-profile',choices=['legacy_low','proximal_5_3'],default='legacy_low')
    parser.add_argument('--short-test',action='store_true',help='Two-base holding validation; not a full coverage experiment')
    parser.add_argument('--trim-j2',action='store_true')
    parser.add_argument('--min-j4-span-deg',type=int,choices=[0,20,30,40,50,60],default=0,
                        help='Minimum NOMINAL target span; does not enlarge joint limits')
    parser.add_argument('--min-j3-model-span-nm',type=float,choices=[0,.2,.3,.4,.5,.6],default=0,
                        help='Minimum predicted gravity range, not motor torque threshold')
    args=parser.parse_args()
    try:
        snap,_,q=pp.fresh_snapshot(args.snapshot,Path.home()/'experiments/torque_compare')
        import mujoco
        path=args.model.expanduser().resolve(strict=True)
        raw=mujoco.MjModel.from_xml_path(str(path))
        ids=[mujoco.mj_name2id(raw,mujoco.mjtObj.mjOBJ_JOINT,f'openarm_right_joint{i}') for i in range(1,8)]
        if raw.nq!=7 or raw.nv!=7 or any(j<0 or not raw.jnt_limited[j] for j in ids):
            raise ValueError('Expected fixed-base seven-hinge model with finite ranges.')
        cfg=make_config(tm.read_json(Path(__file__).parent/'safety.template.json'),path,q,
                        [float(raw.jnt_range[j,0]) for j in ids],[float(raw.jnt_range[j,1]) for j in ids],
                        args.min_proximal_span_deg,args.approach_deg,args.repeats,args.torque_profile)
        print('Explicit torque profile:',cfg['torque_profile'],'software alarms Nm:',cfg['torque_alarm_nm'],flush=True)
        cfg['minimum_j4_nominal_span_deg']=args.min_j4_span_deg
        cfg['minimum_j3_model_span_nm']=args.min_j3_model_span_nm
        cfg['trim_j2']=args.trim_j2
        if args.trim_j2:
            cfg['trim_j2_protocol']='bounded-j2-v1'
            print('J2 bounded trim: max reference offset 0.10rad; sample target error <=0.003rad; no gain/torque increase.',flush=True)
        bases,samples,coverage=select_adaptive(tm.Model(cfg,path),cfg,q,args.max_proximal_span_deg,progress=True)
        print('Full-plan J4 nominal span:',coverage['nominal_j4_span_deg'],'deg; J3 model gravity span:',
              coverage['model_j3_span_nm'],'Nm. Predicted coverage, not measured coverage.',flush=True)
        if args.short_test:
            bases,samples,coverage=short_validation(cfg,bases,samples,coverage,tm.Model(cfg,path))
        duration=save_plan(tm.Model(cfg,path),cfg,path,snap,q,bases,samples,coverage,args.out.expanduser().resolve(),mujoco)
        print('J1/J2 nominal spans:',coverage['nominal_j1_span_deg'],coverage['nominal_j2_span_deg'],'deg',flush=True)
        print(f'{len(bases)} bases; {len(samples)} directional acquisitions. Approximate {duration:.1f} minutes + extra settling/end hold.',flush=True)
        if args.trim_j2:
            print('Estimate excludes J2 trim: each acquisition can add up to about 90s correction plus 38s reference release; faults stop the run.',flush=True)
        if args.short_test:print('SHORT HOLD VALIDATION ONLY: full 50-base shoulder coverage is NOT tested.',flush=True)
        print('Review:',args.out/'base_poses_50.csv',args.out/'coverage.json',flush=True)
        return 0
    except (ValueError,KeyError,OSError,TypeError,ImportError) as exc:
        if hasattr(exc,'assessment'):
            rejected=args.out.expanduser().resolve()
            rejected.mkdir(parents=True,exist_ok=False)
            tm.save_json(rejected/'planning_rejection.json',exc.assessment)
            print('Planning rejection report:',rejected/'planning_rejection.json',flush=True)
        parser.exit(1,f'ERROR: {exc}\n')


if __name__=='__main__':raise SystemExit(main())
