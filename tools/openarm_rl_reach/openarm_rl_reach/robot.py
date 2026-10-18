"""Ubuntu hardware coordinator. Default commands do not enable motors."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import numpy as np
from core import load,save,read,vec,Physics,Reference,observation,contract,DT,DECIMATION

HERE=Path(__file__).absolute().parent

def stamp():return time.strftime("%Y%m%d_%H%M%S")+"_"+str(time.time_ns())

def build():
    if sys.platform!="linux":raise RuntimeError("CAN hardware commands run on Ubuntu only; Windows may train/evaluate")
    subprocess.run(["cmake","-S",str(HERE),"-B",str(HERE/"build")],check=True)
    subprocess.run(["cmake","--build",str(HERE/"build"),"-j2"],check=True)
    return HERE/"build/can_bridge"

def can_check(interface):
    links=json.loads(subprocess.check_output(["ip","-json","-details","link","show",interface],text=True))
    l=links[0];d=l.get("linkinfo",{}).get("info_data",{})
    if "UP" not in l.get("flags",[]) or l.get("mtu")!=72 or d.get("state")!="ERROR-ACTIVE":
        raise RuntimeError("Require UP CAN-FD MTU72 ERROR-ACTIVE; configure interface first")
    if d.get("bittiming",{}).get("bitrate")!=1000000 or d.get("data_bittiming",{}).get("bitrate")!=5000000:
        raise RuntimeError("Require nominal 1 Mbps / data 5 Mbps")

def diagnose(exe,args,root):
    out=root/("diagnose_"+stamp())
    subprocess.run([str(exe),"--config",str(Path(args.config).absolute()),"--interface",args.interface,"--output",str(out)],check=True)
    return read(out/"snapshot.json"),out

def receive(s):
    data=s.recv(16384)
    if not data:raise RuntimeError("Bridge closed; inspect metadata.json")
    return json.loads(data)

def send(s,seq,op,**kwargs):
    s.sendall(json.dumps({"seq":seq,"op":op,**kwargs},allow_nan=False).encode())

def deployment(exe,args,c,root,p):
    import torch
    from stable_baselines3 import PPO
    torch.set_num_threads(1)
    folder=Path(args.policy)
    meta=read(folder/"contract.json")
    if meta["contract"]!=contract(c) or meta["model_hash"]!=p.model_hash:
        raise ValueError("Policy config/model mismatch; do not reuse stale training")
    policy=PPO.load(folder/"policy.zip",device="cpu")
    policy.predict(np.zeros(34,np.float32),deterministic=True) # warm up BEFORE any enable
    goal=p.fk(vec(c["goal_q"]))
    p.check_path(c["home"],c["goal_q"])
    if not args.execute:
        print("Preflight only: policy/model contract matches; nominal home-to-goal path checked.")
        print("Goal in fixed base frame (m):",goal)
        print("No CAN opened, no enable/motion sent. Add --execute only after simulation evaluation.")
        return
    if not c.get("hardware_reviewed",False):
        raise ValueError("Review config.local.json and set hardware_reviewed=true; never marks zero calibration")
    evaluation=read(folder/"evaluation.json")
    import hashlib
    if evaluation.get("policy_sha256")!=hashlib.sha256((folder/"policy.zip").read_bytes()).hexdigest():
        raise ValueError("Evaluation belongs to a different policy checkpoint")
    if len(evaluation["episodes"])<40:
        raise ValueError("Need at least 20 nominal + 20 randomized evaluation episodes")
    if evaluation["nominal_success_rate"]<.95 or evaluation["randomized_success_rate"]<.8:
        raise ValueError("Simulation evaluation gate failed; improve policy, do not start hardware")
    can_check(args.interface)
    snap,_=diagnose(exe,args,root)
    q0=vec(snap["q"])
    if np.max(np.abs(q0-vec(c["home"])))>c["home_capture_radius"]:
        raise ValueError("Current pose outside local home capture area; no arbitrary-pose auto-homing")
    p.check_path(q0,c["home"])
    print("Seven-joint motion: no J8. Home:",c["home"],"Goal estimate:",goal)
    print("Provide fall protection without loading arm; stop/timeout may disable, not gravity-safe.")
    if input("Type RUN_REACH to enable, home and run the trained policy: ").strip()!="RUN_REACH":
        raise RuntimeError("Cancelled; no enable sent")
    out=root/("reach_"+stamp())
    parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
    parent.settimeout(5)
    proc=subprocess.Popen([str(exe),"--config",str(Path(args.config).absolute()),"--interface",args.interface,
                           "--output",str(out),"--execute","--fd",str(child.fileno())],pass_fds=(child.fileno(),))
    child.close()
    logs=[];info={"acquisition_completed":False,"supported_exit_confirmed":False,"goal_base_m":goal.tolist()}
    try:
        state=receive(parent)
        q=vec(state["q"])
        if np.max(np.abs(q-q0))>.003:raise ValueError("Arm moved after path review")
        q0=q.copy()
        p.check_path(q0,c["home"])
        ref=Reference(c,q)
        send(parent,state["seq"],"enable",expected_q=q0.tolist())
        state=receive(parent)
        phase="home";phase_start=time.monotonic();phase_steps=0;stable=0
        previous=np.zeros(7);action=np.zeros(7)
        supported=threading.Event()
        def wait_support():
            while True:
                try:answer=input("Holding current reference. Install physical support, type SUPPORTED then Enter: ")
                except EOFError:return
                if answer.strip()=="SUPPORTED":supported.set();return
        for tick in range(43000):
            now=time.monotonic();q=vec(state["q"]);dq=vec(state["dq"])
            elapsed=now-phase_start
            if phase=="home":
                # Smooth quintic schedule; following it does not require exact intermediate arrival.
                delta=vec(c["home"])-q0
                duration=max(2.,1.875*np.max(np.abs(delta))/c["reference_speed"],
                             np.sqrt(5.774*np.max(np.abs(delta))/c["reference_accel"]))
                u=min(phase_steps*DT/duration,1.)
                desired=q0+(10*u**3-15*u**4+6*u**5)*delta
                next_ref=desired
                ref.v=(next_ref-ref.q)/DT;ref.q=next_ref.copy()
                if u>=1:
                    settled=np.max(np.abs(q-vec(c["home"])))<c["home_tolerance"] and np.max(np.abs(dq))<c["success_speed"]
                    stable=stable+1 if settled else 0
                    if stable>=50:
                        phase="policy";phase_start=now;phase_steps=-1;stable=0
                        # Exact reference state must match observation and smoothing in simulation.
                        ref.v[:]=0;previous[:]=0;action[:]=0
                        print("Home settled; policy started.",flush=True)
                    elif elapsed>duration+30:raise RuntimeError("Home did not settle within tolerance")
            elif phase=="policy":
                if phase_steps%DECIMATION==0:
                    obs=observation(c,q,dq,ref.q,previous,goal,p.fk(q))
                    action,_=policy.predict(obs,deterministic=True)
                    action=np.clip(vec(action),-1,1);previous=action.copy()
                old_ref=ref.q.copy();next_ref=ref.step(action)
                # Screening actual and requested path; no claim of certified clearance.
                p.check_pose(q);p.check_path(old_ref,next_ref)
                distance=float(np.linalg.norm(goal-p.fk(q)))
                settled=distance<c["success_distance"] and np.max(np.abs(dq))<c["success_speed"] and np.max(np.abs(ref.v))<.002
                stable=stable+1 if settled else 0
                if stable>=50 or elapsed>=c["episode_seconds"]:
                    info["acquisition_completed"]=stable>=50
                    info["result"]="success_estimated_fk" if stable>=50 else "task_timeout"
                    phase="brake";phase_steps=0
            elif phase=="brake":
                next_ref=ref.step(np.zeros(7))
                p.check_path(state["ref"],next_ref)
                if np.max(np.abs(ref.v))<1e-9:
                    phase="hold";phase_start=now
                    threading.Thread(target=wait_support,daemon=True).start()
            else:
                next_ref=ref.q.copy()
                if supported.is_set():
                    send(parent,state["seq"],"supported")
                    info["supported_exit_confirmed"]=True
                    break
                if elapsed>300:raise RuntimeError("Support confirmation timeout; disable attempted")
            logs.append({"monotonic":now,"feedback_time_s":state["time_s"],"observation_reference":state["ref"],"phase":phase,"seq":state["seq"],"action":action.tolist(),
                         "reference":next_ref.tolist(),"position_base_m":p.fk(q).tolist(),"q":q.tolist(),"dq":dq.tolist()})
            send(parent,state["seq"],"command",reference=next_ref.tolist())
            state=receive(parent);phase_steps+=1
        else:raise RuntimeError("Session bounded duration exceeded")
    except BaseException as e:
        info["error"]=repr(e)
        raise
    finally:
        parent.close() # bridge detects EOF and disables before writing bulk logs
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.terminate();proc.wait(timeout=8)
        if out.exists():
            save(out/"session.json",info)
            with (out/"policy.jsonl").open("w",encoding="utf-8") as f:
                for row in logs:f.write(json.dumps(row,allow_nan=False)+"\n")
        print("Result:",out,"Bridge exit:",proc.returncode)
    if proc.returncode:raise RuntimeError("Bridge fault; inspect metadata and timing.csv")

def timing(exe,args,c,root):
    """30 seconds with real queries + FK + policy inference, without enabling."""
    import torch
    from stable_baselines3 import PPO
    torch.set_num_threads(1)
    p=Physics(c);folder=Path(args.policy);meta=read(folder/"contract.json")
    if meta["contract"]!=contract(c) or meta["model_hash"]!=p.model_hash:raise ValueError("Policy mismatch")
    policy=PPO.load(folder/"policy.zip",device="cpu")
    policy.predict(np.zeros(34,np.float32),deterministic=True)
    can_check(args.interface)
    out=root/("timing_"+stamp())
    parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET);parent.settimeout(5)
    proc=subprocess.Popen([str(exe),"--config",str(Path(args.config).absolute()),"--interface",args.interface,
                           "--output",str(out),"--shadow","--fd",str(child.fileno())],pass_fds=(child.fileno(),))
    child.close()
    try:
        s=receive(parent);send(parent,s["seq"],"read",expected_q=s["q"]);s=receive(parent)
        action=np.zeros(7);goal=p.fk(c["goal_q"])
        for i in range(1500):
            q,dq=vec(s["q"]),vec(s["dq"])
            if i%5==0:
                action,_=policy.predict(observation(c,q,dq,q,action,goal,p.fk(q)),deterministic=True)
            # Exercise the same model screening work, discard all motion outputs.
            p.check_pose(q);p.check_path(q,q)
            send(parent,s["seq"],"command",reference=q.tolist());s=receive(parent)
        send(parent,s["seq"],"done")
    finally:
        parent.close()
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=8)
        print("Read-only timing result:",out)
    if proc.returncode:raise RuntimeError("Timing check failed; do not enable motors")
    print("30-second query/inference timing check passed; not a hard real-time guarantee.")

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command",choices=["init","inspect","diagnose","capture-home","capture-goal","timing","deploy"])
    parser.add_argument("--config",default="config.local.json")
    parser.add_argument("--model")
    parser.add_argument("--interface",default="can0")
    parser.add_argument("--output-root",default="~/experiments/rl_reach")
    parser.add_argument("--policy",default="runs/ppo_first")
    parser.add_argument("--execute",action="store_true")
    args=parser.parse_args()
    if args.execute and args.command!="deploy":raise ValueError("--execute is only valid for deploy")
    if args.command=="init":
        if Path(args.config).exists():raise ValueError("Config exists; will not overwrite")
        c=read(HERE/"config.template.json")
        if args.model:c["model"]=str(Path(args.model).expanduser().absolute())
        else:c["model"]=str(HERE.parent.parent/"models/openarm_v1_right/openarm_v1_7dof.xml")
        save(args.config,c);print("Created",args.config,"Home/goal intentionally unset.");return
    c=load(args.config,poses=args.command in ("inspect","deploy","timing"))
    if args.command=="inspect":
        p=Physics(c);p.check_path(c["home"],c["goal_q"])
        print("home q:",c["home"],"goal q:",c["goal_q"])
        print("Home position:",p.fk(c["home"]),"Goal position:",p.fk(c["goal_q"]))
        print("Model hash:",p.model_hash)
        if np.linalg.norm(p.fk(c["goal_q"])-p.fk(c["home"]))<2*c["success_distance"]:
            print("WARNING: goal too close for a meaningful reaching test; initial pose may already pass.")
        return
    root=Path(args.output_root).expanduser()
    if args.command!="deploy" or args.execute:root.mkdir(parents=True,exist_ok=True)
    if args.command=="deploy":
        p=Physics(c)
        exe=build() if args.execute else None
        deployment(exe,args,c,root,p);return
    if args.command=="timing":
        timing(build(),args,c,root);return
    exe=build();can_check(args.interface)
    snap,out=diagnose(exe,args,root);print("Snapshot:",out)
    if args.command.startswith("capture-"):
        p=Physics(c);q=vec(snap["q"]);p.check_pose(q)
        key="home" if args.command=="capture-home" else "goal_q"
        if key=="goal_q" and c["home"] is not None:
            p.check_path(c["home"],q)
        if c[key] is not None:
            raise ValueError("Pose already recorded; explicitly edit config to null before replacing")
        c[key]=q.tolist();c["hardware_reviewed"]=False
        save(args.config,c);print("Recorded",key,"without enable or zero-setting:",c[key])

if __name__=="__main__":main()
