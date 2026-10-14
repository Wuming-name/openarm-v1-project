#!/usr/bin/env python3
"""Offline MuJoCo path checks and static-gravity comparisons; never opens CAN.

Use the actual measured seven-angle vector for each row. q_model = sign*q_motor+offset.
With unit transmission assumed, tau_motor_gravity = sign*tau_model_gravity.
No actuator ctrl value is treated as joint torque; no moving data is compared to gravity.
"""
import argparse
import csv
import hashlib
import json
import math
import re
import statistics
from pathlib import Path

PMAX = [12.5]*7
TMAX = [54,54,28,28,10,10,10]
PERIOD = 0.05


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def save_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def vector7(cfg, key):
    values = cfg[key]
    if not isinstance(values, list) or len(values)!=7:
        raise ValueError(f"{key}: needs 7 numbers")
    if any(type(x) not in (int,float) or not math.isfinite(x) for x in values):
        raise ValueError(f"{key}: null/nonfinite/non-number; fill reviewed values first")
    return values


def number(cfg, key, low, high):
    value=cfg[key]
    if type(value) not in (int,float) or not math.isfinite(value) or not low<=value<=high:
        raise ValueError(f"invalid {key}: expected {low}..{high}")
    return value


def validate_config(cfg):
    pilot=cfg.get("experiment_scope")=="local_pilot_1deg"
    flags=("holding_gains_verified","fall_protection_confirmed") if pilot else (
        "motion_limits_reviewed", "geometry_and_mass_reviewed", "fall_protection_confirmed")
    for flag in flags:
        if cfg.get(flag) is not True:
            raise ValueError(f"{flag} is not confirmed; template cannot authorize motion")
    low=vector7(cfg,"q_min_rad"); high=vector7(cfg,"q_max_rad")
    kp=vector7(cfg,"kp"); kd=vector7(cfg,"kd")
    torque=vector7(cfg,"torque_alarm_nm"); speed=vector7(cfg,"speed_alarm_rad_s")
    if pilot:
        if cfg.get("zero_uncertainty_rad")!=[None]*7:
            raise ValueError("pilot must preserve unknown zero uncertainty as null")
        uncertainty=[0]*7 # Schema check only; NOT a known/calibrated uncertainty bound.
    else:
        uncertainty=vector7(cfg,"zero_uncertainty_rad")
    for i in range(7):
        if not (-PMAX[i]<low[i]<high[i]<PMAX[i] and 0<kp[i]<=500 and 0<=kd[i]<=5
                and 0<torque[i]<0.9*TMAX[i] and 0<speed[i]<=0.5 and 0<=uncertainty[i]<=0.2):
            raise ValueError(f"invalid settings for J{i+1}; torque encoding range is not a safety rating")
    number(cfg,"tracking_alarm_rad",0.01,0.16)
    number(cfg,"transit_speed_rad_s",0.001,0.05)
    number(cfg,"transit_accel_rad_s2",0.001,0.1)
    number(cfg,"temperature_alarm_c",25,70)
    number(cfg,"settle_timeout_s",2,60)
    number(cfg,"sample_seconds",1,15)
    number(cfg,"startup_tolerance_rad",0.0005,0.01)
    number(cfg,"settle_position_error_rad",0.001,cfg["tracking_alarm_rad"])
    number(cfg,"settle_speed_rad_s",0.001,0.04)
    number(cfg,"settle_spread_rad",0.0005,0.01)
    number(cfg,"stable_seconds",0.5,3)
    number(cfg,"path_margin_rad",0.001,0.2)
    number(cfg,"gravity_alarm_fraction",0.1,0.8)
    count=cfg["minimum_valid_samples"]
    if type(count) is not int or not 10<=count<=cfg["sample_seconds"]/PERIOD*0.9:
        raise ValueError("minimum_valid_samples incompatible with dwell time")
    if any(x<=cfg["transit_speed_rad_s"] for x in speed):
        raise ValueError("speed alarms must exceed transit speed")
    names=cfg["joint_names"]
    if len(names)!=7 or any(not isinstance(n,str) or not n for n in names) or len(set(names))!=7:
        raise ValueError("supply 7 actual, unique MuJoCo joint names in J1-J7 order")
    signs=vector7(cfg,"joint_sign"); vector7(cfg,"zero_offset_rad")
    if any(x not in (-1,1) for x in signs):
        raise ValueError("joint_sign must be +1/-1; this tool assumes output-shaft angles, unit transmission")
    if pilot:
        baseline=vector7(cfg,"pilot_baseline_q_motor_rad")
        if kp!=[35,35,20,20,6,6,6] or kd!=[4,4,2,2,0.8,0.8,0.8]:
            raise ValueError("pilot requires the holding gains verified by the operator")
        caps=[2,2,1.5,1.5,0.7,0.7,0.7]
        if any(torque[i]>caps[i] or speed[i]>0.15 or low[i]<baseline[i]-0.120001 or
               high[i]>baseline[i]+0.120001 for i in range(7)):
            raise ValueError("pilot software alarm/envelope exceeds initial small-motion scope")
        if cfg["tracking_alarm_rad"]>0.03 or cfg["transit_speed_rad_s"]>0.005 or cfg["transit_accel_rad_s2"]>0.01:
            raise ValueError("pilot tracking/speed/acceleration exceeds initial scope")


class Model:
    def __init__(self, cfg, path, binary=False):
        import mujoco
        self.mj=mujoco
        self.cfg=cfg
        self.model=(mujoco.MjModel.from_binary_path(str(path)) if binary
                    else mujoco.MjModel.from_xml_path(str(path)))
        self.data=mujoco.MjData(self.model)
        model=self.model
        if model.nq!=7 or model.nv!=7 or model.njnt!=7:
            raise ValueError("requires fixed-base seven-hinge right-arm model; remove J8/gripper/free joints")
        if model.neq or model.ntendon:
            raise ValueError("equality/tendon constraints need a separate force analysis")
        if not math.isclose(float(sum(x*x for x in model.opt.gravity))**0.5,9.81,abs_tol=0.03):
            raise ValueError("unexpected gravity magnitude; check units/base orientation")
        disabled=int(model.opt.disableflags)
        for flag in (mujoco.mjtDisableBit.mjDSBL_GRAVITY,mujoco.mjtDisableBit.mjDSBL_CONTACT,
                     mujoco.mjtDisableBit.mjDSBL_CONSTRAINT):
            if disabled & int(flag): raise ValueError("gravity/contact/constraint computation is disabled")
        self.ids=[]
        for name in cfg["joint_names"]:
            jid=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
            if jid<0 or model.jnt_type[jid]!=mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f"missing/nonhinge joint {name}")
            self.ids.append(jid)
        self.qadr=[int(model.jnt_qposadr[i]) for i in self.ids]
        self.dadr=[int(model.jnt_dofadr[i]) for i in self.ids]
        self.sign=cfg["joint_sign"]; self.offset=cfg["zero_offset_rad"]

    def evaluate(self, q):
        self.mj.mj_resetData(self.model,self.data)
        for i,adr in enumerate(self.qadr):
            self.data.qpos[adr]=self.sign[i]*q[i]+self.offset[i]
        self.data.qvel[:]=0
        self.data.ctrl[:]=0
        self.data.qfrc_applied[:]=0
        self.mj.mj_forward(self.model,self.data)
        gravity=[float(self.data.qfrc_bias[d])*self.sign[i] for i,d in enumerate(self.dadr)]
        passive=[float(self.data.qfrc_passive[d])*self.sign[i] for i,d in enumerate(self.dadr)]
        return gravity,passive,int(self.data.ncon)

    def check_point(self, q, cfg):
        for i,x in enumerate(q):
            # Uncertain zero, possible servo tracking error, and startup variation are
            # included as a conservative limit margin, not a calibrated probability.
            uncertainty=cfg["zero_uncertainty_rad"][i]
            # Pilot checks NOMINAL model travel only. Unknown zero errors are explicitly
            # retained as null, so this is not proof of physical collision/limit clearance.
            margin=cfg["path_margin_rad"]+(uncertainty if uncertainty is not None else 0)+cfg["tracking_alarm_rad"]+cfg["startup_tolerance_rad"]
            if not cfg["q_min_rad"][i]+margin<=x<=cfg["q_max_rad"][i]-margin:
                raise ValueError(f"J{i+1}: insufficient reviewed travel margin at {x:.5f} rad")
            jid=self.ids[i]
            if self.model.jnt_limited[jid]:
                center=self.sign[i]*x+self.offset[i]
                low,high=self.model.jnt_range[jid]
                if not low+margin<=center<=high-margin:
                    raise ValueError(f"J{i+1}: insufficient model limit margin")
        gravity,passive,contacts=self.evaluate(q)
        if contacts: raise ValueError("model contact along path; investigate geometry/contact filters, do not bypass")
        if any(abs(p)>1e-4 for p in passive):
            raise ValueError("model passive spring force at rest; gravity-only comparison would be incomplete")
        for i,t in enumerate(gravity):
            if not math.isfinite(t) or abs(t)>cfg["gravity_alarm_fraction"]*cfg["torque_alarm_nm"][i]:
                raise ValueError(f"J{i+1}: model gravity exceeds planned torque headroom")
        return gravity


def read_poses(path):
    expected=["pose_id"]+[f"q{i}_rad" for i in range(1,8)]
    with Path(path).open(encoding="utf-8-sig",newline="") as stream:
        reader=csv.DictReader(stream)
        if reader.fieldnames!=expected: raise ValueError("pose CSV needs: "+",".join(expected))
        poses=[]
        for row in reader:
            name=row["pose_id"]
            if not re.fullmatch(r"[A-Za-z0-9_-]{1,60}",name or "") or name in {p["pose_id"] for p in poses}:
                raise ValueError("pose_id must be unique, ASCII letters/numbers/_/-")
            q=[float(row[f"q{i}_rad"]) for i in range(1,8)]
            if not all(math.isfinite(x) for x in q): raise ValueError("nonfinite pose")
            poses.append({"pose_id":name,"q_motor_rad":q})
    if not 1<=len(poses)<=100: raise ValueError("fill 1..100 reviewed poses; blank template cannot move the arm")
    return poses


def prepare(args):
    cfg=read_json(args.config); validate_config(cfg)
    snapshot=read_json(args.snapshot)
    if snapshot["schema"]!="openarm-snapshot-v1": raise ValueError("invalid snapshot")
    baseline=vector7(snapshot,"q_motor_rad")
    model_path=Path(cfg["model_xml"]).expanduser().resolve(strict=True)
    model=Model(cfg,model_path)
    poses=read_poses(args.poses)
    points=[baseline]+[p["q_motor_rad"] for p in poses]+[baseline]
    count=0
    for first,last in zip(points,points[1:]):
        # Dense geometric path sampling, NOT a continuous collision guarantee.
        steps=max(1,math.ceil(max(abs(b-a) for a,b in zip(first,last))/0.005))
        for n in range(steps+1):
            q=[a+(b-a)*n/steps for a,b in zip(first,last)]
            model.check_point(q,cfg); count+=1
    out=Path(args.out).resolve()
    out.mkdir(parents=True,exist_ok=False)
    frozen=out/"model.mjb"
    model.mj.mj_saveModel(model.model,str(frozen),None)
    value={"schema":"openarm-static-v1","model_path_checked":True,"config":cfg,
           "baseline_q_motor_rad":baseline,"poses":poses,"path_samples_checked":count,
           "frozen_model_file":"model.mjb","frozen_model_sha256":sha256(frozen),
           "model_xml_path":str(model_path),"model_xml_sha256":sha256(model_path),
           "config_sha256":sha256(args.config),"poses_sha256":sha256(args.poses),
           "mujoco_version":model.mj.__version__,"source_snapshot":str(Path(args.snapshot).resolve()),
           "zero_calibration_status":"uncertain; offsets and uncertainty supplied by operator",
           "limitations":"sampled nominal path only; real obstacles, contact filters, servo deviation and dynamics require operator review"}
    save_json(out/"plan.json",value)
    print(f"Prepared {len(poses)} poses / {count} model path samples. Plan: {out/'plan.json'}")
    print("No CAN access, no motion. Inspect model and actual workspace before executing.")


def compare(args):
    run=Path(args.run).resolve(strict=True)
    meta=read_json(run/"metadata.json")
    if meta["mode"]!="static_motion": raise ValueError("diagnostic rows are not loaded holding-torque measurements")
    plan=meta["plan"]; cfg=plan["config"]
    validate_config(cfg)
    frozen=run/"model.mjb"
    if sha256(frozen)!=plan["frozen_model_sha256"]: raise ValueError("frozen model changed; refusing mismatched comparison")
    model=Model(cfg,frozen,binary=True)
    if model.mj.__version__!=plan["mujoco_version"]:
        raise ValueError("MuJoCo version differs from preparation; use recorded version")
    out=run/"comparison"
    out.mkdir(exist_ok=False)
    groups={}; accepted=0; rejected=0
    fields=["time_s","pose_id","joint","q_motor_rad","q_model_rad","torque_feedback_nm",
            "model_gravity_nm","model_passive_nm","residual_nm","mos_c","coil_c",
            "zero_sensitivity_nm_per_rad","linear_zero_uncertainty_nm"]
    with (run/"samples.csv").open(encoding="utf-8",newline="") as stream, \
            (out/"comparison.csv").open("x",encoding="utf-8",newline="") as dest:
        reader=csv.DictReader(stream); writer=csv.DictWriter(dest,fieldnames=fields); writer.writeheader()
        for row in reader:
            if row["stage"]!="sample": continue
            q=[float(row[f"q{i}"]) for i in range(1,8)]
            dq=[float(row[f"dq{i}"]) for i in range(1,8)]
            tau=[float(row[f"tau{i}"]) for i in range(1,8)]
            good=(row["stable"]=="1" and all(row[f"status{i}"]=="1" for i in range(1,8)) and
                  all(math.isfinite(x) for x in q+dq+tau) and max(map(abs,dq))<=cfg["settle_speed_rad_s"] and
                  float(row["transaction_ms"])<=50 and float(row["rx_skew_ms"])<=20 and
                  float(row["max_kernel_age_ms"])<=25 and float(row["cycle_gap_ms"])<=75)
            good=good and all(cfg["q_min_rad"][i]<=q[i]<=cfg["q_max_rad"][i] and
                              abs(tau[i])<=cfg["torque_alarm_nm"][i] and
                              float(row[f"mos{i+1}"])<cfg["temperature_alarm_c"] and
                              float(row[f"coil{i+1}"])<cfg["temperature_alarm_c"] for i in range(7))
            if not good: rejected+=1; continue
            gravity,passive,contacts=model.evaluate(q)
            if contacts or not all(math.isfinite(x) for x in gravity+passive) or any(abs(x)>1e-4 for x in passive):
                rejected+=1; continue
            # Full 7x7 local sensitivity: any joint zero error can affect any torque.
            gradients=[[0.0]*7 for _ in range(7)]
            for j in range(7):
                plus=q.copy(); minus=q.copy(); plus[j]+=1e-4; minus[j]-=1e-4
                gp,_,_=model.evaluate(plus); gm,_,_=model.evaluate(minus)
                for i in range(7): gradients[i][j]=(gp[i]-gm[i])/2e-4
            for i in range(7):
                values={"time_s":row["time_s"],"pose_id":row["pose_id"],"joint":f"J{i+1}",
                        "q_motor_rad":q[i],"q_model_rad":model.sign[i]*q[i]+model.offset[i],
                        "torque_feedback_nm":tau[i],"model_gravity_nm":gravity[i],
                        "model_passive_nm":passive[i],"residual_nm":tau[i]-gravity[i],
                        "mos_c":row[f"mos{i+1}"],"coil_c":row[f"coil{i+1}"],
                        "zero_sensitivity_nm_per_rad":gradients[i][i],
                        "linear_zero_uncertainty_nm":(None if any(x is None for x in cfg["zero_uncertainty_rad"]) else
                            sum(abs(gradients[i][j])*cfg["zero_uncertainty_rad"][j] for j in range(7)))}
                writer.writerow(values)
                groups.setdefault((row["pose_id"],i),[]).append(values)
            accepted+=1
    summary=[]
    for (pose,i),rows in groups.items():
        count=len(rows); mean=lambda name: statistics.fmean(r[name] for r in rows)
        summary.append({"pose_id":pose,"joint":f"J{i+1}","count":count,
                        "enough_samples":count>=cfg["minimum_valid_samples"],
                        "q_mean_rad":mean("q_motor_rad"),"feedback_mean_nm":mean("torque_feedback_nm"),
                        "feedback_std_nm":statistics.stdev(r["torque_feedback_nm"] for r in rows) if count>1 else 0,
                        "gravity_mean_nm":mean("model_gravity_nm"),"residual_mean_nm":mean("residual_nm"),
                        "rmse_nm":math.sqrt(statistics.fmean(r["residual_nm"]**2 for r in rows)),
                        "zero_linear_bound_mean_nm":(None if rows[0]["linear_zero_uncertainty_nm"] is None else mean("linear_zero_uncertainty_nm"))})
    if summary:
        with (out/"summary.csv").open("x",encoding="utf-8",newline="") as dest:
            writer=csv.DictWriter(dest,fieldnames=list(summary[0])); writer.writeheader(); writer.writerows(summary)
    expected={(p["pose_id"],f"J{i}") for p in plan["poses"] for i in range(1,8)}
    complete=({(s["pose_id"],s["joint"]) for s in summary}==expected and
              all(s["enough_samples"] for s in summary) and meta["completed"])
    report={"accepted_rounds":accepted,"rejected_rounds":rejected,"acquisition_completed":meta["completed"],
            "dataset_complete":complete,"frozen_model_sha256":sha256(frozen),
            "zero_calibration_status":plan["zero_calibration_status"],
            "feedback_is_joint_nm_confirmed":cfg["feedback_is_joint_nm_confirmed"],
            "experiment_scope":cfg.get("experiment_scope","reviewed_full"),
            "interpretation":"Residual includes zero/model/friction/current-to-torque errors; not a calibrated accuracy certificate. Zero bound is first-order only, not a guaranteed error bound."}
    save_json(out/"report.json",report)
    print(f"Comparison: {out}; valid rounds={accepted}; complete={complete}")
    if not cfg["feedback_is_joint_nm_confirmed"]:
        print("Torque units/scaling at output shaft not confirmed: results are exploratory only.")
    return 0 if complete else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="command",required=True)
    prep=sub.add_parser("prepare")
    for flag in ("config","snapshot","poses","out"): prep.add_argument("--"+flag,required=True)
    comp=sub.add_parser("compare"); comp.add_argument("--run",required=True)
    args=parser.parse_args()
    try:
        return (prepare(args) or 0) if args.command=="prepare" else compare(args)
    except (ValueError,KeyError,OSError,TypeError,ImportError) as exc:
        parser.exit(1,f"ERROR: {exc}\n")


if __name__=="__main__":
    raise SystemExit(main())
