"""Attended, bounded automatic preparation to A before running a fixed PPO policy."""
import argparse
import hashlib
import json
import math
import queue
import socket
import subprocess
import threading
import time
from pathlib import Path
import numpy as np
from sim_core import DT, DECIMATION, Reference, observation, save, vec
from session_io import load_session, build, check_can, recv, send, prior_pass
from startup_control import (AutoHome, HOME_TOL, parking_kind, held_reference,
                             screen_pose, screen_path)

HERE=Path(__file__).resolve().parent


def motion_ack(command):
    print("Automatic preparation supports only near-zero or near-A stationary starts.")
    print("No precision manual positioning is needed. Clear the complete path to A.")
    print("Keep fall protection available and unloaded during motion; do not hold the links in place.")
    print("STOP+Enter requests braking. Fault/Ctrl+C/communication loss can remove torque.")
    token="RUN_AUTO_HOME" if command=="home-test" else "RUN_AUTO_RL"
    if input(f"Type {token} to enable and run one experiment: ").strip()!=token:
        raise RuntimeError("Cancelled before CAN access")


def run(args,c,p,policy,key):
    motion=args.command in ("home-test","trial")
    if motion and not args.execute:
        raise ValueError("Use --execute only when ready for automatic motion")
    output=Path(args.output_root).expanduser().resolve()
    output.mkdir(parents=True,exist_ok=True)
    if motion:
        prior_pass(output,"shadow",key)
        if args.command=="trial":
            prior_pass(output,"home-test",key)
    save(output/(args.command+"_passed.json"),
         {"passed":False,"unix_time":time.time(),"key":key})
    exe=build()
    check_can(args.interface)
    if motion:
        motion_ack(args.command)
    stamp=time.strftime("%Y%m%d_%H%M%S")+"_"+str(time.time_ns())
    out=output/(args.command+"_"+stamp)
    out.mkdir()
    runtime=dict(c,supervised_trial=True,auto_start=True)
    save(out/"runtime.json",runtime)
    report={"command":args.command,"key":key,"passed":False,
            "automatic_preparation":True,"hardware_accuracy_measured":False,
            "simulation_success_claim":False,"supported_exit_confirmed":False}
    save(out/"session.json",report)
    parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
    parent.settimeout(3)
    cmd=[str(exe),"--config",str(out/"runtime.json"),"--interface",args.interface,
         "--output",str(out/"bridge"),"--fd",str(child.fileno()),
         "--execute" if motion else "--shadow"]
    proc=None
    logs=[]
    notices,messages=queue.Queue(),queue.Queue()
    def console():
        while True:
            message=notices.get()
            if message is None:
                return
            print(message,flush=True)
    def keyboard():
        while True:
            try:
                messages.put(input().strip().upper())
            except EOFError:
                messages.put("STOP")
                return
    threading.Thread(target=console,daemon=True).start()
    try:
        proc=subprocess.Popen(cmd,pass_fds=(child.fileno(),))
        child.close()
        state=recv(parent)
        q0=vec(state["q"])
        kind=parking_kind(c,q0)
        print("PARKING:",kind,"actual q (rad):",q0.tolist(),flush=True)
        start_ref=held_reference(c,p,q0)
        final_ref=held_reference(c,p,vec(c["home"]))
        screen_path(c,p,q0,c["home"],startup=True)
        screen_path(c,p,start_ref,final_ref,startup=True,reference=True)
        report.update(parking_kind=kind,initial_q=q0.tolist(),
                      initial_reference=start_ref.tolist(),nominal_A_reference=final_ref.tolist())
        send(parent,state,"enable" if motion else "read",expected_q=q0.tolist(),
             initial_reference=start_ref.tolist())
        state=recv(parent)
        automatic=AutoHome(c,vec(state["ref"]) if motion else start_ref,final_ref)
        ref=automatic.ref
        if motion:
            threading.Thread(target=keyboard,daemon=True).start()
            notices.put(f"AUTO_HOME from {kind}: moving to A; nominal reference path {automatic.duration:.1f}s. "
                        "RL has NOT started. Do not constrain the moving links.")
        phase="auto-home" if motion else "shadow"
        previous,action=np.zeros(7),np.zeros(7)
        goal=p.fk(c["goal_q"])
        ticks=policy_ticks=0
        result=""
        for cycle in range(22000):
            q,dq=vec(state["q"]),vec(state["dq"])
            startup=bool(state.get("startup_guard",True))
            screen_pose(c,p,q,startup=startup)
            if motion:
                if (np.any(np.abs(dq)>vec(c["speed_alarm"]))
                        or np.max(np.abs(q-ref.q))>c["tracking_alarm"]):
                    raise ValueError("Actual speed/tracking alarm")
            user=[]
            while True:
                try:
                    user.append(messages.get_nowait())
                except queue.Empty:
                    break
            if "STOP" in user and phase not in ("brake","hold"):
                result,phase,ticks="operator_stop","brake",0
                notices.put("STOP requested: braking then holding for support.")
            complete_startup=False
            if phase=="shadow":
                parking_kind(c,q)
                if ticks%DECIMATION==0:
                    action,_=policy.predict(observation(c,q,dq,state["ref"],previous,goal,p.fk(q)),
                                            deterministic=True)
                    previous=vec(action).copy()
                # Same current-pose screening and inference workload, without MIT writes.
                screen_path(c,p,start_ref,start_ref,startup=True,reference=True)
                next_ref=vec(state["ref"])
                if ticks>=1500:
                    send(parent,state,"done")
                    report["passed"]=True
                    result="read_only_shadow_passed"
                    break
            elif phase=="auto-home":
                try:
                    next_ref,ready=automatic.step(q,dq)
                except ValueError as exc:
                    result="automatic_preparation_failed: "+str(exc)
                    phase,ticks="brake",-1
                    next_ref=ref.step(np.zeros(7))
                    notices.put(result+"; RL NOT started. Braking then hold.")
                    ready=False
                if ready and int(state.get("home_stable_count",0))>=50:
                    # No reset of q_ref: retain the holding reference and its PD load.
                    complete_startup=True
                    phase,ticks="holding-test",-1
                    report["A_reached"]={"q":q.tolist(),"dq":dq.tolist(),
                                         "reference":ref.q.tolist(),"time_s":state["time_s"]}
                    notices.put("A_STABLE: beginning 5-second real holding check; RL still NOT started.")
            elif phase=="holding-test":
                next_ref=ref.q.copy()
                ref.v[:]=0
                if np.max(np.abs(q-vec(c["home"])))>HOME_TOL or np.max(np.abs(dq))>=c["success_speed"]:
                    result,phase,ticks="A_holding_check_failed","brake",-1
                    notices.put("A holding check failed; RL NOT started. Braking then hold.")
                elif ticks>=250:
                    report["A_hold_passed"]=True
                    if args.command=="home-test":
                        result,phase,ticks="automatic_home_and_hold_passed","brake",-1
                    else:
                        # The same integration/observation state, not q_ref=q_actual.
                        ref=Reference(c,ref.q)
                        previous[:]=0
                        action[:]=0
                        phase,ticks="policy",-1
                        notices.put(f"RL_STARTED: bounded {args.seconds:g}s; STOP requests braking.")
            elif phase=="policy":
                if ticks%DECIMATION==0:
                    action,_=policy.predict(observation(c,q,dq,ref.q,previous,goal,p.fk(q)),
                                            deterministic=True)
                    action=np.clip(vec(action),-1,1)
                    previous=action.copy()
                next_ref=ref.step(action)
                policy_ticks+=1
                if policy_ticks*DT>=args.seconds:
                    result,phase,ticks="bounded_policy_trial_finished","brake",-1
                    notices.put("Policy limit reached: braking then holding; no automatic return.")
            elif phase=="brake":
                next_ref=ref.step(np.zeros(7))
                if np.max(np.abs(ref.v))<1e-10:
                    phase,ticks="hold",-1
                    notices.put("HOLD: install load-bearing support, then type SUPPORTED. "
                                "If preparation failed, total startup time is bounded to 180s.")
            elif phase=="hold":
                next_ref=ref.q.copy()
                if "SUPPORTED" in user and ticks>=0:
                    send(parent,state,"supported")
                    report["supported_exit_confirmed"]=True
                    report["passed"]=result in ("automatic_home_and_hold_passed","bounded_policy_trial_finished")
                    break
                if ticks>=15000:
                    raise RuntimeError("Support confirmation timeout; torque may be removed")
            else:
                raise RuntimeError("Unknown stage")
            if phase!="shadow":
                screen_path(c,p,state["ref"],next_ref,
                            startup=startup and not complete_startup,reference=True)
            if motion and cycle%50==0:
                notices.put(f"{phase}: q4={q[3]:.4f} rad; max A error="
                            f"{np.max(np.abs(q-vec(c['home']))):.4f} rad")
            logs.append({"cycle":cycle,"phase":phase,"time_s":state["time_s"],
                         "q":q.tolist(),"dq":dq.tolist(),"tau_feedback":state["tau"],
                         "action":vec(action).tolist(),"observation_reference":state["ref"],
                         "next_reference":next_ref.tolist(),"reference_velocity":ref.v.tolist(),
                         "startup_complete_requested":complete_startup,
                         "home_stable_count":state.get("home_stable_count",0),
                         "distance_fk_m":float(np.linalg.norm(goal-p.fk(q)))})
            send(parent,state,"command",reference=next_ref.tolist(),startup_complete=complete_startup)
            state=recv(parent)
            ticks+=1
        else:
            raise RuntimeError("Total session bound exceeded")
        report.update(result=result,policy_seconds=policy_ticks*DT)
    except BaseException as exc:
        report["error"]=repr(exc)
        raise
    finally:
        parent.close()
        child.close()
        if proc is not None:
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=8)
            report["bridge_exit"]=proc.returncode
            if proc.returncode!=0:
                report["passed"]=False
        save(out/"session.json",report)
        with (out/"policy.jsonl").open("w",encoding="utf-8") as f:
            for row in logs:
                f.write(json.dumps(row,allow_nan=False)+"\n")
        print("Result:",out,report,flush=True)
    if report["passed"]:
        save(output/(args.command+"_passed.json"),
             {"passed":True,"unix_time":time.time(),"key":key,"session":str(out)})
    else:
        raise RuntimeError("Stage did not pass; inspect logs before retrying")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command",choices=["inspect","shadow","home-test","trial"])
    parser.add_argument("--config",required=True)
    parser.add_argument("--run",required=True)
    parser.add_argument("--interface",default="can0")
    parser.add_argument("--output-root",default="~/experiments/rl_v2_autostart")
    parser.add_argument("--seconds",type=float,default=5.)
    parser.add_argument("--execute",action="store_true")
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or not 0<args.seconds<=15:
        parser.error("Policy duration must be >0 and <=15 seconds")
    if args.execute and args.command not in ("home-test","trial"):
        parser.error("inspect/shadow never enable motors")
    c,p,policy,_,key=load_session(args)
    key["autostart_sources"]={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest()
                              for name in ("autostart.py","startup_control.py","session_io.py")}
    if args.command=="inspect":
        print("Offline policy contract passed; no CAN opened.")
        print("Preparation supports near zero (other joints +/-0.03rad, J4 -0.005..0.06rad)")
        print("or near A (each joint +/-0.08rad). Testing preparation in MuJoCo...")
        from simulate_startup import simulate
        zero=np.zeros(7);zero[3]=-.0020981155108
        result,_=simulate(c,zero)
        print("SIMULATED_STARTUP:",json.dumps(result,ensure_ascii=False))
        print("This is model-only verification, not a real-arm result.")
        return
    run(args,c,p,policy,key)


if __name__=="__main__":
    main()
