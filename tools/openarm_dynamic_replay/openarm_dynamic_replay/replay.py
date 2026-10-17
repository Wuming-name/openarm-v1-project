#!/usr/bin/env python3
"""Offline feedback-torque replay. This file NEVER imports CAN or controls hardware."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def save(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Series:
    def __init__(self, rows, max_gap=.075):
        if len(rows)<4:raise ValueError('Need at least four consecutive enabled data rows')
        self.rows=rows
        self.rx=np.array([[float(r[f'rx{i}']) for i in range(1,8)] for r in rows])
        self.q=np.array([[float(r[f'q{i}']) for i in range(1,8)] for r in rows])
        self.dq=np.array([[float(r[f'dq{i}']) for i in range(1,8)] for r in rows])
        self.tau=np.array([[float(r[f'tau{i}']) for i in range(1,8)] for r in rows])
        for a in (self.rx,self.q,self.dq,self.tau):
            if not np.all(np.isfinite(a)):raise ValueError('NaN/Inf in input data')
        gap=np.diff(self.rx,axis=0)
        if np.any(gap<=0):raise ValueError('Nonmonotonic per-joint timestamp; no automatic sorting')
        if np.max(gap)>max_gap:raise ValueError(f'Feedback gap {np.max(gap):.6f}s exceeds {max_gap}s; no gap bridging')
        for r in rows:
            if r.get('feedback_valid')!='1' or r.get('command_sent')!='1' or any(r[f'status{i}']!='1' for i in range(1,8)):
                raise ValueError('Invalid/disabled row inside acquisition; no silently dropped rows')
            if float(r['rx_skew_ms'])>20 or float(r['transaction_ms'])>50 or float(r['max_kernel_age_ms'])>25:
                raise ValueError('Feedback timing guard violated')
        self.start=float(np.max(self.rx[0]));self.end=float(np.min(self.rx[-1]))
        if self.end<=self.start:raise ValueError('No common seven-joint time support')
        self.max_gap=float(np.max(gap))
        self.events=np.unique(self.rx.ravel())

    def state(self,t):
        if t<self.start-1e-9 or t>self.end+1e-9:raise ValueError('State extrapolation prohibited')
        q=np.array([np.interp(t,self.rx[:,j],self.q[:,j]) for j in range(7)])
        v=np.array([np.interp(t,self.rx[:,j],self.dq[:,j]) for j in range(7)])
        return q,v

    def force(self,t):
        # Causal sample-and-hold: never use a future feedback torque sample.
        ids=[max(0,np.searchsorted(self.rx[:,j],t+1e-10,side='right')-1) for j in range(7)]
        return np.array([self.tau[k,j] for j,k in enumerate(ids)])


def load_run(path,allow_partial=False):
    meta=json.loads((path/'metadata.json').read_text(encoding='utf-8'))
    if meta.get('version')!='openarm-dynamic-replay-v1':
        raise ValueError('Use this package\'s dynamic run. Old static logs need separate explicit conversion.')
    if not meta.get('completed') and not allow_partial:raise ValueError('Run incomplete; --allow-partial explicitly analyzes the recorded prefix only')
    with (path/'samples.csv').open(encoding='utf-8',newline='') as f:rows=list(csv.DictReader(f))
    indices=[i for i,r in enumerate(rows) if r['stage'] in ('record_hold','transit') and r['pose_id']!='align_baseline']
    if not indices:raise ValueError('No trajectory acquisition data')
    # Keep ALL rows between boundaries. Never concatenate across invalid gaps.
    rows=rows[min(indices):max(indices)+1]
    return meta,Series(rows)


def metrics(real,sim):
    if len(real)<2:return None
    e=np.rad2deg(np.asarray(sim)-np.asarray(real))
    return [{'joint':f'J{j+1}','n':len(e),'bias_deg':float(e[:,j].mean()),
             'rmse_deg':float(np.sqrt(np.mean(e[:,j]**2))),
             'p95_abs_deg':float(np.percentile(np.abs(e[:,j]),95)),
             'max_abs_deg':float(np.max(np.abs(e[:,j])))} for j in range(7)]


class Replay:
    def __init__(self,path,cfg,dt=.001,scale=None,bias=None):
        import mujoco as mj
        self.mj=mj
        self.model=(mj.MjModel.from_binary_path(str(path)) if path.suffix=='.mjb' else mj.MjModel.from_xml_path(str(path)))
        m=self.model
        if m.nq!=7 or m.nv!=7 or m.njnt!=7 or m.neq or m.ntendon:
            raise ValueError('Requires fixed-base seven-hinge model without equality/tendon constraints')
        ids=[mj.mj_name2id(m,mj.mjtObj.mjOBJ_JOINT,n) for n in cfg['joint_names']]
        if len(ids)!=7 or len(set(ids))!=7 or any(j<0 or m.jnt_type[j]!=mj.mjtJoint.mjJNT_HINGE for j in ids):
            raise ValueError('Seven unique hinge names must map to J1..J7')
        self.qadr=np.array([m.jnt_qposadr[j] for j in ids]);self.dadr=np.array([m.jnt_dofadr[j] for j in ids]);self.ids=ids
        self.sign=np.array(cfg['joint_sign'],float);self.offset=np.array(cfg['zero_offset_rad'],float)
        if self.sign.shape!=(7,) or self.offset.shape!=(7,) or not np.all(np.isfinite(self.offset)) or not np.all(np.isin(self.sign,[-1,1])):
            raise ValueError('Only +/-1 direction and finite zero offset supported; no implicit gear ratio')
        self.scale=np.ones(7) if scale is None else np.array(scale,float)
        self.bias=np.zeros(7) if bias is None else np.array(bias,float)
        if any(x.shape!=(7,) or not np.all(np.isfinite(x)) for x in (self.scale,self.bias)):raise ValueError('Invalid calibration arrays')
        self.dt=dt
        self.audit={'original_timestep_s':float(m.opt.timestep),'original_disableflags':int(m.opt.disableflags),
                    'actuator_count':int(m.nu),'original_gravcomp':m.body_gravcomp.tolist(),
                    'damping':m.dof_damping[self.dadr].tolist(),'frictionloss':m.dof_frictionloss[self.dadr].tolist(),
                    'armature':m.dof_armature[self.dadr].tolist(),'gravity':m.opt.gravity.tolist(),
                    'torque_scale':self.scale.tolist(),'torque_bias_nm':self.bias.tolist()}
        for flag in (mj.mjtDisableBit.mjDSBL_GRAVITY,mj.mjtDisableBit.mjDSBL_CONTACT,mj.mjtDisableBit.mjDSBL_CONSTRAINT):
            if int(m.opt.disableflags)&int(flag):raise ValueError('Model disables gravity/contact/constraints; review it first')
        if not math.isclose(float(np.linalg.norm(m.opt.gravity)),9.81,abs_tol=.03):raise ValueError('Unexpected gravity magnitude')
        # Disable actuator force even when ctrl=0 would still generate position PD.
        m.opt.disableflags=int(m.opt.disableflags)|int(mj.mjtDisableBit.mjDSBL_ACTUATION)
        m.body_gravcomp[:]=0
        m.opt.timestep=dt
        self.data=mj.MjData(m)
        self.audit['replay_actuation']='actuators disabled; body gravcomp zero; joint qfrc_applied only; passive/constraints retained'

    def state(self):
        return self.sign*(self.data.qpos[self.qadr]-self.offset),self.sign*self.data.qvel[self.dadr]

    def problem(self):
        q,v=self.state()
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):return 'nonfinite_simulation'
        if self.data.ncon:return 'model_contact'
        for j in self.ids:
            if self.model.jnt_limited[j]:
                x=self.data.qpos[self.model.jnt_qposadr[j]]
                if x<self.model.jnt_range[j,0] or x>self.model.jnt_range[j,1]:return 'model_joint_limit'
        if any(w.number>0 for w in self.data.warning):return 'mujoco_warning'
        return None

    def simulate(self,series,start,end,output_dt=.05,max_error_deg=30):
        mj=self.mj;m=self.model;d=self.data
        mj.mj_resetData(m,d)
        q,v=series.state(start)
        d.qpos[self.qadr]=self.sign*q+self.offset;d.qvel[self.dadr]=self.sign*v
        d.time=0;mj.mj_forward(m,d)
        times=np.arange(start,end,output_dt).tolist()+[end]
        result=[];reason=self.problem();cursor=start
        for target in times:
            while reason is None and cursor<target-1e-10:
                index=np.searchsorted(series.events,cursor+1e-10,side='right')
                next_event=series.events[index] if index<len(series.events) else target
                h=min(self.dt,target-cursor,float(next_event-cursor))
                if h<=1e-10:raise ValueError('Integration time did not advance')
                d.qfrc_applied[:]=0;d.xfrc_applied[:]=0;d.ctrl[:]=0
                d.qfrc_applied[self.dadr]=self.sign*(series.force(cursor)*self.scale+self.bias)
                m.opt.timestep=h;mj.mj_step(m,d);cursor+=h
                # Refresh contacts and other derived fields at the new state.
                mj.mj_forward(m,d)
                reason=self.problem()
                if reason is None:
                    qr,_=series.state(cursor)
                    if np.max(np.abs(np.rad2deg(self.state()[0]-qr)))>max_error_deg:reason='prediction_diverged'
            if reason is not None:break
            qr,vr=series.state(target);qs,vs=self.state()
            result.append({'time_s':float(target),'elapsed_s':float(target-start),
                           'q_real':qr.tolist(),'q_sim':qs.tolist(),'dq_real':vr.tolist(),'dq_sim':vs.tolist()})
        return result,{'start_s':start,'end_s':end,'reached_s':float(cursor),'completed':reason is None,
                       'stop_reason':reason,'metrics':metrics([r['q_real'] for r in result],[r['q_sim'] for r in result])}


def write_prediction(path,rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['time_s','elapsed_s']+[f'{name}{i}' for name in ('q_real','q_sim','error_deg','dq_real','dq_sim') for i in range(1,8)])
        for r in rows:w.writerow([r['time_s'],r['elapsed_s']]+r['q_real']+r['q_sim']+np.rad2deg(np.array(r['q_sim'])-r['q_real']).tolist()+r['dq_real']+r['dq_sim'])


def plot(path,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if len(rows)<2:return
    t=np.array([r['elapsed_s'] for r in rows]);real=np.rad2deg([r['q_real'] for r in rows]);sim=np.rad2deg([r['q_sim'] for r in rows])
    fig,axes=plt.subplots(7,2,figsize=(13,17),sharex=True)
    for j in range(7):
        axes[j,0].plot(t,real[:,j],label='Measured');axes[j,0].plot(t,sim[:,j],label='Torque replay',alpha=.85)
        axes[j,0].set_ylabel(f'J{j+1} angle (deg)');axes[j,0].grid(alpha=.25)
        axes[j,1].plot(t,sim[:,j]-real[:,j],color='tab:red');axes[j,1].set_ylabel('Error (deg)');axes[j,1].grid(alpha=.25)
    axes[0,0].legend();axes[-1,0].set_xlabel('Elapsed time (s)');axes[-1,1].set_xlabel('Elapsed time (s)')
    fig.suptitle('Motor-estimate torque replay: exploratory, not calibrated output torque')
    fig.tight_layout(rect=(0,0,1,.98));fig.savefig(path,dpi=160);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path)
    p.add_argument('--out',type=Path)
    p.add_argument('--model',type=Path,help='Explicit alternative XML/MJB for offline comparison only')
    p.add_argument('--calibration',type=Path,help='JSON with torque_scale[7], torque_bias_nm[7]; never fit automatically')
    p.add_argument('--allow-partial',action='store_true')
    p.add_argument('--dt',type=float,default=.001)
    p.add_argument('--window-seconds',type=float,default=1.)
    p.add_argument('--window-stride',type=float,default=2.)
    p.add_argument('--max-windows',type=int,default=100)
    p.add_argument('--max-error-deg',type=float,default=30,help='Offline divergence stop, NOT a hardware limit')
    p.add_argument('--position-budget-deg',type=float,help='Optional user task budget, not a safety threshold')
    p.add_argument('--no-plot',action='store_true')
    args=p.parse_args()
    if not .0001<=args.dt<=.005 or not .1<=args.window_seconds<=10 or not .1<=args.window_stride<=60 or not 1<=args.max_windows<=1000 or not 1<=args.max_error_deg<=180:p.error('Invalid replay bounds')
    if args.position_budget_deg is not None and not 0<args.position_budget_deg<=180:p.error('Invalid task budget')
    # absolute() avoids Windows sandbox GetFinalPathNameByHandle directory ACL
    # failures; individual files are still opened and checked normally.
    run=args.run.expanduser().absolute();meta,series=load_run(run,args.allow_partial)
    plan=meta['plan'];model=args.model.expanduser().resolve(strict=True) if args.model else run/'model.mjb'
    import mujoco
    if model.suffix=='.mjb' and mujoco.__version__!=plan['mujoco_version']:raise ValueError('MJB version differs; use the recorded MuJoCo version or explicitly supply original XML')
    if not args.model and digest(model)!=plan['frozen_model_sha256']:raise ValueError('Frozen model SHA256 mismatch')
    calibration=json.loads(args.calibration.read_text()) if args.calibration else {}
    engine=Replay(model,plan['config'],args.dt,calibration.get('torque_scale'),calibration.get('torque_bias_nm'))
    out=args.out.expanduser().absolute() if args.out else run/'replay'
    out.mkdir(parents=True,exist_ok=False)
    report={'schema':'openarm-replay-v1','source_run':str(run),'acquisition_completed':meta.get('completed',False),
            'model':str(model),'model_sha256':digest(model),'alternative_model':args.model is not None,
            'samples_sha256':digest(run/'samples.csv'),'mujoco_version':mujoco.__version__,
            'input_torque':'motor estimate, not calibrated output-shaft measurement',
            'alignment':'per-joint RX time; linear state interpolation, causal ZOH torque; no delay fitting',
            'sampling_limit':'50ms acquisition cannot resolve fast torque dynamics; RX time is not physical sensor sample time',
            'max_feedback_gap_s':series.max_gap,'simulation_dt_max_s':args.dt,'model_audit':engine.audit,
            'start_s':series.start,'end_s':series.end,'windows':[],'completed':False}
    save(out/'report.json',report)
    rows,full=engine.simulate(series,series.start,series.end,max_error_deg=args.max_error_deg)
    report['full']=full;write_prediction(out/'full_prediction.csv',rows)
    if not args.no_plot:plot(out/'angles_and_errors.png',rows)
    starts=np.arange(series.start,series.end-args.window_seconds+1e-9,args.window_stride)
    report['window_candidates']=len(starts);report['windows_truncated']=len(starts)>args.max_windows
    all_window_rows=[]
    with (out/'window_predictions.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.writer(f);w.writerow(['window','elapsed_s','time_s']+[f'{name}{i}' for name in ('q_real','q_sim') for i in range(1,8)])
        for k,start in enumerate(starts[:args.max_windows],1):
            prediction,result=engine.simulate(series,float(start),float(start+args.window_seconds),max_error_deg=args.max_error_deg)
            report['windows'].append(result)
            # Failed windows are reported, never silently removed from the pass/fail decision.
            for r in prediction:w.writerow([k,r['elapsed_s'],r['time_s']]+r['q_real']+r['q_sim'])
            all_window_rows.extend(prediction[1:]) # Exclude artificially exact initialized row.
            if k%10==0:print(f'Replayed {k}/{min(len(starts),args.max_windows)} windows',flush=True)
    report['window_pooled_metrics']=metrics([r['q_real'] for r in all_window_rows],[r['q_sim'] for r in all_window_rows])
    report['evaluation']='UNASSESSED: no task budget supplied; torque meaning remains unverified'
    if args.position_budget_deg is not None:
        report['position_budget_deg']=args.position_budget_deg
        completed=bool(meta.get('completed')) and full['completed'] and bool(report['windows']) and all(w['completed'] for w in report['windows']) and not report['windows_truncated']
        relevant=[full['metrics'],report['window_pooled_metrics']]
        ok=completed and all(m is not None and all(j['max_abs_deg']<=args.position_budget_deg for j in m) for m in relevant)
        report['evaluation']='WITHIN_SUPPLIED_ANGLE_BUDGET_ON_TESTED_DATA_ONLY' if ok else 'NOT_WITHIN_BUDGET_OR_INCOMPLETE'
    report['completed']=True # Analysis finished; NOT a declaration of model correctness.
    save(out/'report.json',report)
    table=[]
    for scope,values in [('full',full['metrics']),('short_windows_pooled',report['window_pooled_metrics'])]:
        for item in values or []:table.append({'scope':scope,**item})
    if table:
        with (out/'metrics.csv').open('w',newline='',encoding='utf-8') as f:
            w=csv.DictWriter(f,fieldnames=list(table[0]));w.writeheader();w.writerows(table)
    print('Results:',out,'\nFull replay complete:',full['completed'],'stop:',full['stop_reason'])
    print(report['evaluation'])


if __name__=='__main__':
    try:main()
    except Exception as exc:raise SystemExit(f'ERROR: {exc}')
