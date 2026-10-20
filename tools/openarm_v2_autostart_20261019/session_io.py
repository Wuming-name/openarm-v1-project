"""Policy contract, build and IPC helpers. Importing opens no CAN interface."""
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

