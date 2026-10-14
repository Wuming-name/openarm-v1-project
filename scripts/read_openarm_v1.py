#!/usr/bin/env python3
"""Seven-axis OpenArm V1 diagnostic. Only parameter (0x33) and status (0xCC) requests.

No enable/disable, movement, calibration, parameter writes, or interface changes.
Protocol references: enactic/openarm_can dm_motor_control.cpp and dm_motor_constants.hpp.
Python 3.10 standard library; Linux SocketCAN only for hardware execution.
"""
import argparse
import csv
import datetime
import math
from pathlib import Path
import platform
import select
import socket
import struct
import sys
import time

MODELS = (
    "DM-J8009P-2EC", "DM-J8009P-2EC", "DM-J4340P-2EC",
    "DM-J4340-2EC", "DM-J4310-2EC V1.1", "DM-J4310-2EC V1.1",
    "DM-J4310-2EC V1.1",
)
STATUS = {0: "DISABLED", 1: "ENABLED", 8: "OVERVOLTAGE", 9: "UNDERVOLTAGE",
          10: "OVERCURRENT", 11: "MOS_OVERHEAT", 12: "COIL_OVERHEAT",
          13: "COMM_LOST", 14: "OVERLOAD"}
PARAMS = {7: ("master_id", "I"), 8: ("motor_id", "I"),
          10: ("control_mode", "I"), 21: ("pmax", "f"),
          22: ("vmax", "f"), 23: ("tmax", "f"), 35: ("baud_code", "I")}
FIELDS = ["time", "joint", "model", "motor_id", "reply_id", "position_reg_rad",
          "output_position_reg_rad", "feedback_position_rad", "velocity_rad_s",
          "torque_feedback_nm", "mos_c", "coil_c", "state", "state_reply",
          "position_reply", "output_position_reply"]


def request_payload(motor_id, opcode, rid=0):
    if motor_id not in range(1, 8) or opcode not in (0x33, 0xCC):
        raise ValueError("Only J1-J7 read requests are permitted")
    return struct.pack("<HBB4x", motor_id, opcode, rid)


def decode_param(data, motor_id, rid, fmt):
    if len(data) != 8 or data[:2] != struct.pack("<H", motor_id):
        return None
    if data[2] != 0x33 or data[3] != rid:
        return None
    value = struct.unpack("<" + fmt, data[4:8])[0]
    return value if math.isfinite(value) else None


def decode_state(data, motor_id, limits):
    if len(data) != 8 or (data[0] & 0xF) != motor_id:
        return None
    p = (data[1] << 8) | data[2]
    v = (data[3] << 4) | (data[4] >> 4)
    t = ((data[4] & 0xF) << 8) | data[5]
    def scale(raw, maximum, bits):
        return raw * (2 * maximum) / ((1 << bits) - 1) - maximum
    code = data[0] >> 4
    return {"feedback_position_rad": scale(p, limits["pmax"], 16),
            "velocity_rad_s": scale(v, limits["vmax"], 12),
            "torque_feedback_nm": scale(t, limits["tmax"], 12),
            "mos_c": data[6], "coil_c": data[7],
            "state": STATUS.get(code, "UNKNOWN_0x{:X}".format(code)),
            "state_reply": True}


class ReadBus:
    def __init__(self, interface, timeout):
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            # Exact standard reply IDs, excluding extended/RTR frames.
            filters = b"".join(struct.pack("=II", i + 0x10, 0xC00007FF) for i in range(1, 8))
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.sock.settimeout(timeout)
            self.sock.bind((interface,))
        except Exception:
            self.sock.close()
            raise

    def close(self):
        self.sock.close()

    def drain(self):
        # Do not use previous queued samples as a reply to a new request.
        for _ in range(4096):
            if not select.select([self.sock], [], [], 0)[0]:
                return
            self.sock.recv(72)
        raise RuntimeError("CAN queue continuously busy; stop other control/query programs")

    def query(self, motor_id, opcode, decoder, rid=0):
        self.drain()
        payload = request_payload(motor_id, opcode, rid)
        # struct canfd_frame: 8-byte header, 64-byte data; CANFD_BRS = 1.
        frame = struct.pack("=IBBBB64s", 0x7FF, 8, 1, 0, 0, payload)
        if self.sock.send(frame) != 72:
            raise RuntimeError("Incomplete CAN-FD request write")
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.sock], [], [], remaining)[0]:
                return None
            raw = self.sock.recv(72)
            if len(raw) not in (16, 72):
                continue
            can_id, length = struct.unpack_from("=IB", raw)
            if can_id != motor_id + 0x10 or length != 8:
                continue
            value = decoder(raw[8:16])
            if value is not None:
                return value

    def param(self, motor_id, rid, fmt="f"):
        return self.query(motor_id, 0x33,
                          lambda d: decode_param(d, motor_id, rid, fmt), rid)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--duration", type=float, default=0,
                        help="Seconds of repeated readings; default one seven-joint snapshot")
    parser.add_argument("--period", type=float, default=1,
                        help="Minimum time between rounds, seconds (default 1). Not a realtime test")
    parser.add_argument("--timeout", type=float, default=0.15,
                        help="Reply timeout per request, seconds (default 0.15)")
    parser.add_argument("--params-only", action="store_true",
                        help="Read only position registers; do not send status refresh requests")
    parser.add_argument("--csv", type=Path, help="New CSV log path; existing file will not be overwritten")
    args = parser.parse_args()
    if not (0 <= args.duration <= 3600 and 0.05 <= args.period <= 60 and 0.01 <= args.timeout <= 2):
        parser.error("duration: 0..3600, period: 0.05..60, timeout: 0.01..2 seconds")
    if platform.system() != "Linux" or not hasattr(socket, "AF_CAN"):
        parser.error("Run hardware tests on Ubuntu; Windows is only for editing this program")
    bus, stream = None, None
    failures = 0
    try:
        bus = ReadBus(args.interface, args.timeout)
        print("Read-only requests: J1-J7, CAN-FD, interface {}. No J8 or gripper.".format(args.interface), flush=True)
        print("Does not change enable state. Close other robot control/query programs first.", flush=True)
        configs = {}
        for motor_id, model in enumerate(MODELS, 1):
            cfg = {}
            for rid, (name, fmt) in PARAMS.items():
                value = bus.param(motor_id, rid, fmt)
                if value is None:
                    raise RuntimeError("J{}: no valid reply for register {} ({})".format(motor_id, rid, name))
                cfg[name] = value
            if cfg["motor_id"] != motor_id or cfg["master_id"] != motor_id + 0x10:
                raise RuntimeError("J{}: unexpected motor/reply ID configuration".format(motor_id))
            if not all(math.isfinite(cfg[k]) and cfg[k] > 0 for k in ("pmax", "vmax", "tmax")):
                raise RuntimeError("J{}: invalid actual feedback scaling limits".format(motor_id))
            configs[motor_id] = cfg
            print("J{} {} | TX=0x{:02X} RX=0x{:02X} | PMAX={} VMAX={} TMAX={} mode={} baud_code={}".format(
                motor_id, model, motor_id, motor_id + 0x10, cfg["pmax"], cfg["vmax"],
                cfg["tmax"], cfg["control_mode"], cfg["baud_code"]), flush=True)
        writer = None
        if args.csv:
            stream = args.csv.open("x", newline="", encoding="utf-8")
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
        start = time.monotonic()
        rounds = 0
        try:
            while True:
                round_start = time.monotonic()
                rounds += 1
                print("\nRound {}: {}".format(rounds, datetime.datetime.now().isoformat(timespec="seconds")), flush=True)
                for motor_id, model in enumerate(MODELS, 1):
                    row = {"time": datetime.datetime.now().astimezone().isoformat(),
                           "joint": "J{}".format(motor_id), "model": model,
                           "motor_id": motor_id, "reply_id": "0x{:02X}".format(motor_id + 0x10)}
                    p = bus.param(motor_id, 80)
                    output = bus.param(motor_id, 81)
                    row.update(position_reg_rad=p, output_position_reg_rad=output,
                               position_reply=p is not None, output_position_reply=output is not None)
                    if not args.params_only:
                        state = bus.query(motor_id, 0xCC,
                                          lambda d, i=motor_id: decode_state(d, i, configs[i]))
                        if state is None:
                            row.update(state="NO_STATUS_REPLY", state_reply=False)
                        else:
                            row.update(state)
                    else:
                        row.update(state="NOT_REQUESTED", state_reply="")
                    if p is None or output is None or (not args.params_only and not row["state_reply"]):
                        failures += 1
                    def show(key):
                        value = row.get(key)
                        return "---" if value is None else "{:.4f}".format(value)
                    print("J{} p_reg={} out_reg={} | p_fb={} vel={} torque_fb={} | MOS={} coil={} | {}".format(
                        motor_id, show("position_reg_rad"), show("output_position_reg_rad"),
                        show("feedback_position_rad"), show("velocity_rad_s"), show("torque_feedback_nm"),
                        row.get("mos_c", "---"), row.get("coil_c", "---"), row["state"]), flush=True)
                    if writer:
                        writer.writerow(row)
                    if row["state"] not in ("DISABLED", "ENABLED", "NOT_REQUESTED", "NO_STATUS_REPLY"):
                        failures += 1
                        raise RuntimeError("J{} reports {}; stop and inspect motor".format(motor_id, row["state"]))
                if stream:
                    stream.flush()
                if args.duration == 0 or time.monotonic() - start >= args.duration:
                    break
                remaining = min(args.period - (time.monotonic() - round_start),
                                args.duration - (time.monotonic() - start))
                if remaining > 0:
                    time.sleep(remaining)
        except KeyboardInterrupt:
            print("Stopped by user; no enable/disable commands were sent.")
            return 130
        print("\n{} rounds; {} joint-rounds with missing replies.".format(rounds, failures))
        if failures:
            print("Incomplete feedback. Do not enable motors to make this test pass; inspect logs first.")
        else:
            print("PASS: all requested replies received during this run; not a 20 ms timing guarantee.")
        return 1 if failures else 0
    except (OSError, RuntimeError) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        print("Check interface UP, 1M/5M FD, motor power, wiring and reply IDs.", file=sys.stderr)
        return 1
    finally:
        if stream:
            stream.close()
        if bus:
            bus.close()


if __name__ == "__main__":
    sys.exit(main())
