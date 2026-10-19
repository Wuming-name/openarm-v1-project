"""A bounded, attended hardware experiment for the saved sim-v2 policy.

Not a production deployer and not a claim that simulation passed. No CAN is
opened by inspect. Only explicit --execute on hold-test/trial can enable motors.
"""
import argparse
import hashlib
import json
import math
import queue
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
from sim_core import (DT, DECIMATION, Physics, Reference, contract, load,
                      observation, read, save, vec)

HERE = Path(__file__).resolve().parent
EXPECTED_POLICY = "e56c2e17c48c2fa2157661be8f842d3b03cde96869eb90b01d24efbd4ca663b3"
HOME_TOL = .005


def require_home(c, q):
    error = float(np.max(np.abs(vec(q) - vec(c["home"]))))
    if error > HOME_TOL:
        raise ValueError(f"Start differs from A by {error:.6f} rad; require <= {HOME_TOL}. "
                         "No automatic homing; use existing supported positioning method.")


def quintic_reference(start, target, seconds, t):
    u = min(max(t / seconds, 0.), 1.)
    return start + (10*u**3 - 15*u**4 + 6*u**5) * (target-start)


def preload_duration(c, start, target):
    delta = float(np.max(np.abs(target-start)))
    return max(2., 1.875*delta/c["reference_speed"],
               math.sqrt(5.774*delta/c["reference_accel"]))


def load_session(args):
    import torch
    from stable_baselines3 import PPO
    torch.set_num_threads(1)
    # Relative model paths have the same meaning as in the user's sim directory.
    c = load(args.config)
    if not np.array_equal(c["sign"], np.ones(7)) or np.any(vec(c["offset"]) != 0):
        raise ValueError("This bridge requires the recorded identity encoder mapping")
    p = Physics(c)
    folder = Path(args.run).resolve()
    meta = read(folder / "contract.json")
    if meta["contract"] != contract(c) or meta["model_hash"] != p.model_hash:
        raise ValueError("Saved policy/config/model mismatch; do not edit hashes to bypass")
    digest = hashlib.sha256((folder/"policy.zip").read_bytes()).hexdigest()
    if digest != EXPECTED_POLICY:
        raise ValueError("This is not the evaluated 2026-10-19 policy requested for this trial")
    evaluation = read(folder/"evaluation.json")
    if evaluation.get("policy_sha256") != digest:
        raise ValueError("Evaluation does not belong to policy.zip")
    policy = PPO.load(folder/"policy.zip", device="cpu")
    if policy.observation_space.shape != (34,) or policy.action_space.shape != (7,):
        raise ValueError("Policy observation/action dimensions differ")
    policy.predict(np.zeros(34, np.float32), deterministic=True)
    p.check_path(c["home"], c["goal_q"])
    p.reset(vec(c["home"]))
    preload = p.initial_reference(vec(c["home"]))
    p.check_reference_path(c["home"], preload)
    key = {"contract": contract(c), "model_hash": p.model_hash,
           "policy_sha256": digest, "interface": args.interface,
           "bridge_sha256": hashlib.sha256((HERE/"can_bridge.cpp").read_bytes()).hexdigest(),
           "coordinator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
           "control_sha256": hashlib.sha256((HERE/"sim_core.py").read_bytes()).hexdigest()}
    print("Policy SHA256:", digest)
    print("Simulation success rates:", evaluation.get("nominal_success_rate"),
          evaluation.get("randomized_success_rate"), "(NOT promoted to passed)")
    print("A (rad):", c["home"], "goal FK (m):", p.fk(c["goal_q"]).tolist())
    print("Model-estimated held reference (rad):", preload.tolist())
    return c, p, policy, preload, key


def build():
    if sys.platform != "linux":
        raise RuntimeError("CAN stages run on Ubuntu only")
    subprocess.run(["cmake", "-S", str(HERE), "-B", str(HERE/"build")], check=True)
    subprocess.run(["cmake", "--build", str(HERE/"build"), "-j2"], check=True)
    return HERE/"build/can_bridge"


def check_can(interface):
    info = json.loads(subprocess.check_output(
        ["ip", "-json", "-details", "link", "show", interface], text=True))[0]
    d = info.get("linkinfo", {}).get("info_data", {})
    if ("UP" not in info.get("flags", []) or info.get("mtu") != 72
            or d.get("state") != "ERROR-ACTIVE"
            or d.get("bittiming", {}).get("bitrate") != 1000000
            or d.get("data_bittiming", {}).get("bitrate") != 5000000):
        raise ValueError("Require existing UP CAN-FD interface: 1Mbps / 5Mbps, ERROR-ACTIVE")


def recv(s):
    raw = s.recv(16384)
    if not raw:
        raise RuntimeError("Bridge closed; inspect metadata.json and timing.csv")
    return json.loads(raw)


def send(s, state, op, **kwargs):
    s.sendall(json.dumps({"seq": state["seq"], "op": op, **kwargs},
                        allow_nan=False).encode())


def prior_pass(root, name, key):
    report = read(root/(name+"_passed.json"))
    age = time.time() - report["unix_time"]
    if report.get("key") != key or not 0 <= age < 3600 or report.get("passed") is not True:
        raise ValueError(f"Run {name} again: current-policy pass required within one hour")


def check_state(c, p, state, ref):
    q, dq = vec(state["q"]), vec(state["dq"])
    if np.any(np.abs(dq) > vec(c["speed_alarm"])):
        raise ValueError("Actual speed alarm")
    if np.max(np.abs(q-vec(c["home"]))) > c["actual_radius"]:
        raise ValueError("Actual local envelope")
    if np.max(np.abs(q-ref.q)) > c["tracking_alarm"]:
        raise ValueError("Actual tracking alarm")
    p.check_pose(q)
    return q, dq


def run(args, c, p, policy, preload, key):
    motion = args.command in ("hold-test", "trial")
    if motion and not args.execute:
        raise ValueError("Motion stage needs explicit --execute; inspect/shadow never enable")
    root = Path(args.output_root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    if motion:
        prior_pass(root, "shadow", key)
        if args.command == "trial":
            prior_pass(root, "hold-test", key)
    # A failed repeat must invalidate the previous pass for this stage.
    save(root/(args.command+"_passed.json"),
         {"passed": False, "unix_time": time.time(), "key": key})
    exe = build()
    check_can(args.interface)
    stamp = time.strftime("%Y%m%d_%H%M%S") + "_" + str(time.time_ns())
    out = root/(args.command+"_"+stamp)
    out.mkdir()
    runtime = dict(c, supervised_trial=True)
    save(out/"runtime.json", runtime)
    report = {"command": args.command, "key": key, "passed": False,
              "simulation_success_claim": False, "physical_accuracy_measured": False,
              "max_policy_seconds": args.seconds, "supported_exit_confirmed": False}
    save(out/"session.json", report)
    if motion:
        print("Attended experiment, NOT a passed/deployed policy. No auto-return to A.")
        print("Have a physical catch/support for every loaded segment, stop control available,")
        print("and a second person to manage support. Timeout/fault/Ctrl+C may remove torque.")
        print("Start supported at A with all motors disabled. No other CAN control tool running.")
        token = "RUN_HOLD_TEST" if args.command == "hold-test" else "RUN_SHORT_TRIAL"
        if input(f"Type {token} to start this single experiment: ").strip() != token:
            raise RuntimeError("Cancelled; no CAN opened")
    parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    parent.settimeout(3)
    cmd = [str(exe), "--config", str(out/"runtime.json"), "--interface", args.interface,
           "--output", str(out/"bridge"), "--fd", str(child.fileno()),
           "--execute" if motion else "--shadow"]
    proc = None
    logs = []
    messages, notices = queue.Queue(), queue.Queue()
    def console():
        while True:
            line = notices.get()
            if line is None:
                return
            print(line, flush=True)
    def keyboard():
        while True:
            try:
                line = input().strip().upper()
            except EOFError:
                messages.put("STOP")
                return
            # Do not accept SUPPORTED/RELEASED before the corresponding phase.
            messages.put(line)
    threading.Thread(target=console, daemon=True).start()
    try:
        proc = subprocess.Popen(cmd, pass_fds=(child.fileno(),))
        child.close()
        state = recv(parent)
        require_home(c, state["q"])
        ref = Reference(c, state["q"])
        start_q = ref.q.copy()
        p.check_reference_path(start_q, preload)
        duration = preload_duration(c, start_q, preload)
        send(parent, state, "enable" if motion else "read", expected_q=start_q.tolist())
        state = recv(parent)
        if motion:
            threading.Thread(target=keyboard, daemon=True).start()
            notices.put("PRELOAD: keep the arm supported while reference is ramped; STOP requests braking.")
        phase = "preload" if motion else "shadow"
        ticks = policy_ticks = stable = released_ticks = 0
        released = False
        previous, action = np.zeros(7), np.zeros(7)
        goal = p.fk(c["goal_q"])
        result = ""
        for cycle in range(20000):
            q, dq = check_state(c, p, state, ref)
            user = []
            while True:
                try:
                    user.append(messages.get_nowait())
                except queue.Empty:
                    break
            if motion and "STOP" in user and phase not in ("brake", "hold"):
                result = "operator_stop"
                phase, ticks = "brake", 0
                notices.put("Braking reference; will hold for physical support. Not an emergency-stop replacement.")
            # Faults are independently checked by the C++ process before sending commands.
            if phase == "shadow":
                require_home(c, q)
                if ticks % DECIMATION == 0:
                    action, _ = policy.predict(observation(c, q, dq, ref.q, previous, goal, p.fk(q)),
                                               deterministic=True)
                    previous = vec(action).copy()
                # Exercise model screening, never send MIT packets in bridge shadow mode.
                p.check_reference_path(ref.q, ref.q)
                next_ref = ref.q.copy()
                if ticks >= 1500:
                    send(parent, state, "done")
                    report["passed"] = True
                    break
            elif phase == "preload":
                next_ref = quintic_reference(start_q, preload, duration, (ticks+1)*DT)
                ref.v = (next_ref-ref.q)/DT
                ref.q = next_ref.copy()
                if (ticks+1)*DT >= duration:
                    phase, ticks = "release", -1
                    notices.put("PRELOAD_READY: slowly unload/remove the supporting force; keep a noncontact catch. "
                                "Only after release type RELEASED. If arm drifts, type STOP; do not force it into place.")
            elif phase == "release":
                next_ref = ref.q.copy()
                ref.v[:] = 0
                if "RELEASED" in user:
                    released = True
                    stable = released_ticks = 0
                if released:
                    released_ticks += 1
                    good = (np.max(np.abs(q-vec(c["home"]))) <= HOME_TOL
                            and np.max(np.abs(dq)) < c["success_speed"])
                    stable = stable+1 if good else 0
                    if stable >= 50:
                        phase = "holding-test" if args.command == "hold-test" else "policy"
                        ticks = -1
                        previous[:] = 0
                        action[:] = 0
                        notices.put("START: " + phase + "; STOP requests braking. Keep catch unloaded.")
                    elif released_ticks >= 250:
                        result, phase, ticks = "start_did_not_match_training", "brake", -1
                        notices.put("Start did not settle near A. Policy NOT started; braking then hold.")
                if ticks >= 3000:
                    result, phase, ticks = "support_release_timeout", "brake", -1
            elif phase == "holding-test":
                next_ref = ref.q.copy()
                ref.v[:] = 0
                require_home(c, q)
                if np.max(np.abs(dq)) >= c["success_speed"]:
                    raise ValueError("Holding test speed did not remain below success threshold")
                if ticks >= 250:
                    result, phase, ticks = "holding_test_passed", "brake", -1
            elif phase == "policy":
                if ticks % DECIMATION == 0:
                    action, _ = policy.predict(observation(c, q, dq, ref.q, previous, goal, p.fk(q)),
                                               deterministic=True)
                    action = np.clip(vec(action), -1, 1)
                    previous = action.copy()
                next_ref = ref.step(action)
                policy_ticks += 1
                if policy_ticks*DT >= args.seconds:
                    result, phase, ticks = "bounded_policy_trial_finished", "brake", -1
                    notices.put("Policy time limit reached; braking then holding. No automatic return.")
            elif phase == "brake":
                next_ref = ref.step(np.zeros(7))
                if np.max(np.abs(ref.v)) < 1e-10:
                    phase, ticks = "hold", -1
                    notices.put("HOLD: install load-bearing support. Only then type SUPPORTED to disable and exit. "
                                "Do not just close the terminal.")
            elif phase == "hold":
                next_ref = ref.q.copy()
                if "SUPPORTED" in user and ticks >= 0:
                    send(parent, state, "supported")
                    report["supported_exit_confirmed"] = True
                    report["passed"] = result in ("holding_test_passed", "bounded_policy_trial_finished")
                    break
                if ticks >= 15000:
                    raise RuntimeError("Five-minute support timeout; torque may be removed")
            else:
                raise RuntimeError("Unknown phase")
            # The same reference/travel checks as sim-v2; no gravity headroom check on PD targets.
            p.check_reference_path(state["ref"], next_ref)
            logs.append({"cycle": cycle, "phase": phase, "time_s": state["time_s"],
                         "q": q.tolist(), "dq": dq.tolist(), "tau_feedback": state["tau"],
                         "action": vec(action).tolist(), "observation_reference": state["ref"],
                         "next_reference": next_ref.tolist(), "reference_velocity": ref.v.tolist(),
                         "distance_fk_m": float(np.linalg.norm(goal-p.fk(q)))})
            send(parent, state, "command", reference=next_ref.tolist())
            state = recv(parent)
            ticks += 1
        else:
            raise RuntimeError("Session duration bound reached")
        report["result"] = result or "read_only_shadow_passed"
        report["policy_seconds"] = policy_ticks*DT
    except BaseException as exc:
        report["error"] = repr(exc)
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
            report["bridge_exit"] = proc.returncode
            if proc.returncode != 0:
                report["passed"] = False
        save(out/"session.json", report)
        with (out/"policy.jsonl").open("w", encoding="utf-8") as f:
            for row in logs:
                f.write(json.dumps(row, allow_nan=False)+"\n")
        print("Result:", out, report, flush=True)
    if report["passed"]:
        save(root/(args.command+"_passed.json"),
             {"passed": True, "unix_time": time.time(), "key": key, "session": str(out)})
    else:
        raise RuntimeError("Stage did not pass; inspect logs, do not advance stages")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["inspect", "shadow", "hold-test", "trial"])
    parser.add_argument("--config", required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--output-root", default="~/experiments/rl_v2_trial")
    parser.add_argument("--seconds", type=float, default=5.)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 15:
        parser.error("Single policy trial must be > 0 and <= 15 seconds")
    if args.execute and args.command not in ("hold-test", "trial"):
        parser.error("inspect/shadow never accept --execute")
    c, p, policy, preload, key = load_session(args)
    if args.command == "inspect":
        print("Offline contract check passed. No CAN opened; no enable/motion.")
        return
    run(args, c, p, policy, preload, key)


if __name__ == "__main__":
    main()
