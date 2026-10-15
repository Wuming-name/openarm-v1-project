#!/usr/bin/env python3
"""One entry point for OpenArm V1 diagnostics, planning and acquisition.

Linux only. No motion unless --execute is supplied AND RUN_STATIC is typed in
the controller. Never changes gains/alarms or retries a failed motion.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
VERSION = 'openarm-static-v1-multi7-proximal'


def run(command):
    print('\n>', ' '.join(map(str, command)), flush=True)
    child = subprocess.Popen([str(x) for x in command], cwd=str(HERE))
    try:
        return child.wait()
    except KeyboardInterrupt:
        # The terminal normally signals both processes. Also forward SIGINT for
        # callers which signal only this wrapper. Never SIGKILL a holding arm.
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            print('Waiting for controller shutdown; no automatic restart.', flush=True)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                print(f'Controller PID {child.pid} has not exited. Check physical support and motor state.', flush=True)
        raise


def checked(command):
    code = run(command)
    if code:
        raise RuntimeError(f'Step failed (exit {code}); subsequent steps cancelled.')


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def single_run(root, mode):
    matches = list(root.glob(f'{mode}_*/metadata.json'))
    if len(matches) != 1:
        raise RuntimeError(f'Expected exactly one {mode} result inside this session; found {len(matches)}.')
    path = matches[0].parent
    meta = json.loads(matches[0].read_text(encoding='utf-8'))
    if meta.get('mode') != ('static_motion' if mode == 'static' else 'diagnose'):
        raise RuntimeError('Unexpected result mode.')
    return path, meta


def check_can():
    result = subprocess.run(['ip', '-json', '-details', 'link', 'show', 'can0'],
                            capture_output=True, text=True, check=True)
    links = json.loads(result.stdout)
    if len(links) != 1:
        raise RuntimeError('can0 not found.')
    link = links[0]
    info = link.get('linkinfo', {})
    data = info.get('info_data', {})
    if (info.get('info_kind') != 'can' or 'UP' not in link.get('flags', [])
            or link.get('mtu') != 72):
        raise RuntimeError('can0 must be UP with CAN-FD enabled. Use --setup-can if this is the correct adapter channel.')
    if data.get('bittiming', {}).get('bitrate') != 1000000 or data.get('data_bittiming', {}).get('bitrate') != 5000000:
        raise RuntimeError('Expected arbitration 1 Mbps / data 5 Mbps. Use --setup-can to configure can0.')
    if data.get('state') != 'ERROR-ACTIVE':
        raise RuntimeError(f"Unexpected CAN state: {data.get('state')}; inspect wiring/power/termination first.")
    print('can0: CAN-FD 1 Mbps / 5 Mbps, ERROR-ACTIVE.', flush=True)


def ensure_build():
    checked(['cmake', '-S', HERE, '-B', HERE / 'build'])
    checked(['cmake', '--build', HERE / 'build', '-j2'])
    binary = HERE / 'build' / 'openarm_static_torque'
    version = subprocess.check_output([str(binary), '--version'], text=True).strip()
    if version != VERSION:
        raise RuntimeError('Wrong controller version; copy the updated C++ source and rebuild.')
    return binary


def summarize(run_dir, meta):
    from torque_model import row_in_accepted_window
    print('\nResult directory:', run_dir, flush=True)
    print('Completed:', meta.get('completed'), 'Error:', meta.get('error', 'none'), flush=True)
    samples_path = run_dir / 'samples.csv'
    if samples_path.exists():
        with samples_path.open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
    else:
        rows = []  # Parameter verification can fail before Recorder exists.
    samples = Counter(r['pose_id'] for r in rows if r['stage'] == 'sample' and r['stable'] == '1'
                      and row_in_accepted_window(r,meta))
    print('Accepted stable samples by pose:', dict(samples), flush=True)
    if meta.get('last_sampling_quality_event'):
        print('Last sampling quality event:',json.dumps(meta['last_sampling_quality_event'],ensure_ascii=False),flush=True)
    fault = meta.get('safety_fault')
    if fault:
        print('Exact captured fault (not merely the last safe CSV row):', flush=True)
        print(json.dumps(fault, ensure_ascii=False, indent=2), flush=True)
        joint = fault.get('joint')
        if joint and fault.get('command_q_rad') is not None:
            i = joint - 1
            error = fault['q_rad'][i] - fault['command_q_rad'][i]
            print(f'J{joint} tracking error: {math.degrees(error):+.4f} deg', flush=True)
    if not meta.get('completed'):
        print('STOPPED: no retry or automatic gain/limit changes. Disable was best effort, not confirmed.', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Prepare and offer motion confirmation; default is diagnosis only')
    parser.add_argument('--plan-only', action='store_true', help='Read motors and prepare poses; never enable or execute')
    parser.add_argument('--setup-can', action='store_true', help='Configure can0 at 1M/5M FD using sudo, before reading motors')
    parser.add_argument('--mode', choices=['j7', 'multi', 'pose50','proximal'], default='j7')
    parser.add_argument('--max-proximal-span-deg',type=int,choices=[45,60,75,90],default=90)
    parser.add_argument('--min-proximal-span-deg',type=int,choices=[20,30,40,45],default=30)
    parser.add_argument('--approach-deg',type=int,choices=[10,12,15],default=12)
    parser.add_argument('--repeats',type=int,choices=[1,2,3],default=1)
    parser.add_argument('--torque-profile',choices=['legacy_low','proximal_5_3'],default='legacy_low')
    parser.add_argument('--max-angle-deg', type=int, choices=[5,10,15,20,30])
    parser.add_argument('--previous-run', type=Path, default=Path.home() / 'experiments/torque_compare/static_1792044758710916613',
                        help='Completed pilot used for measured-movement eligibility; never used as current position')
    parser.add_argument('--model', type=Path, default=HERE.parent.parent / 'models/openarm_v1_right/openarm_v1_7dof.xml')
    parser.add_argument('--output-root', type=Path, default=Path.home() / 'experiments/torque_compare')
    parser.add_argument('--inspect-run', type=Path, help='Read an existing run only; no build, CAN access or configuration')
    args = parser.parse_args(argv)
    if args.execute and args.plan_only:
        parser.error('Choose --execute or --plan-only, not both')
    if args.max_angle_deg is None:
        args.max_angle_deg=20 if args.mode=='pose50' else 5
    if args.mode=='pose50' and args.max_angle_deg not in (10,15,20,30):
        parser.error('50-pose mode supports 10, 15, 20 or 30 deg maximum per-joint offsets')
    if args.mode=='multi' and args.max_angle_deg not in (5,10,15):
        parser.error('Existing multi mode supports 5, 10 or 15 deg')
    if args.mode=='proximal' and args.max_proximal_span_deg<args.min_proximal_span_deg+9.1674:
        parser.error('Maximum nominal proximal span must exceed required actual span by at least 9.17deg capture margin')
    if args.mode!='proximal' and args.torque_profile!='legacy_low':
        parser.error('The explicit proximal torque profile is available only in proximal mode')
    if args.inspect_run:
        if args.execute or args.setup_can or args.plan_only:
            parser.error('--inspect-run cannot be combined with execution, planning or CAN setup')
        return run([sys.executable, HERE / 'torque_model.py', 'inspect-stop', '--run', args.inspect_run.expanduser()])
    if sys.platform != 'linux':
        parser.error('Run this entry point on the Ubuntu desktop, not in Windows PowerShell.')
    if os.geteuid() == 0:
        parser.error('Run as your normal Ubuntu user; only CAN setup uses sudo.')
    for command in ['cmake', 'ip'] + (['sudo'] if args.setup_can else []):
        if not shutil.which(command):
            parser.error(f'Missing command: {command}')
    wants_plan=args.execute or args.plan_only
    if wants_plan:
        if not importlib.util.find_spec('mujoco'):
            parser.error('MuJoCo unavailable. Use .venv/bin/python and install requirements.txt first.')
        if not args.model.expanduser().is_file():
            parser.error(f'Model not found: {args.model}')
        if args.mode not in ('pose50','proximal'):
            for name in ['metadata.json', 'samples.csv']:
                if not (args.previous_run.expanduser() / name).is_file():
                    parser.error(f'Pilot file not found: {args.previous_run / name}; supply --previous-run.')
            # The historical pilot gate belongs only to the older pilot profiles.
            # pose50 explicitly selects the operator-supplied legacy profile;
            # model screening and measured movement checks still apply.
            import prepare_multi_pose as planner
            prior = planner.previous_motion(args.previous_run.expanduser())
            if args.mode == 'j7' and not planner.eligible_for_j7_probe(prior[6]):
                parser.error('Previous pilot does not qualify for the J7 diagnostic.')
            if args.mode == 'multi' and not any(row['eligible'] for row in prior):
                parser.error('No joint has verified enough actual motion for general multi-pose testing. Diagnose tracking first.')
    # Lock covers diagnosis -> planning -> execution for this launcher. Other
    # utilities do not honor it; the user still must close all CAN control tools.
    import fcntl
    lock_path = Path.home() / '.cache/openarm_static_torque.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another unified launcher is already running.')
        binary = ensure_build()
        root = args.output_root.expanduser()
        root.mkdir(parents=True, exist_ok=True)
        session = root / ('session_' + datetime.now().strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns()}')
        session.mkdir()
        manifest = {'version': VERSION, 'execute_requested': args.execute, 'mode': args.mode,
                    'completed': False, 'session_dir': str(session)}
        try:
            save(session / 'session.json', manifest)
            print('Session:', session, flush=True)
            print('Close all other robot control/query tools. Support the disabled arm for initial diagnosis.', flush=True)
            if args.setup_can:
                checked(['sudo', '-v'])
                checked(['sudo', 'ip', 'link', 'set', 'can0', 'down'])
                checked(['sudo', 'ip', 'link', 'set', 'can0', 'type', 'can',
                         'bitrate', '1000000', 'sample-point', '0.75', 'dbitrate', '5000000',
                         'dsample-point', '0.75', 'dsjw', '2', 'fd', 'on', 'restart-ms', '0'])
                checked(['sudo', 'ip', 'link', 'set', 'can0', 'up'])
            check_can()
            checked([binary, '--diagnose', '--interface', 'can0', '--output-root', session])
            diagnosis, meta = single_run(session, 'diagnose')
            if not meta.get('completed'):
                raise RuntimeError('Diagnosis incomplete; motion cancelled.')
            manifest['diagnosis_dir'] = str(diagnosis)
            if not wants_plan:
                manifest['completed'] = True
                print('Diagnosis passed. No enable/motion commands sent.', flush=True)
                return 0
            plan_dir = session / 'prepared'
            if args.mode=='proximal':
                command=[sys.executable,HERE/'prepare_proximal_50.py','--snapshot',diagnosis/'snapshot.json',
                         '--model',args.model.expanduser(),'--out',plan_dir,
                         '--max-proximal-span-deg',str(args.max_proximal_span_deg),
                         '--min-proximal-span-deg',str(args.min_proximal_span_deg),
                         '--approach-deg',str(args.approach_deg),'--repeats',str(args.repeats)]
                command+=['--torque-profile',args.torque_profile]
            elif args.mode=='pose50':
                command=[sys.executable,HERE/'prepare_static_50.py','--snapshot',diagnosis/'snapshot.json',
                         '--model',args.model.expanduser(),'--out',plan_dir,'--max-angle-deg',str(args.max_angle_deg)]
            else:
                command = [sys.executable, HERE / 'prepare_multi_pose.py', '--previous-run', args.previous_run.expanduser(),
                           '--snapshot', diagnosis / 'snapshot.json', '--model', args.model.expanduser(), '--out', plan_dir]
                command += ['--probe-j7'] if args.mode == 'j7' else ['--max-angle-deg', str(args.max_angle_deg)]
            checked(command)
            manifest['plan'] = str(plan_dir / 'plan.json')
            save(session / 'session.json', manifest)
            if args.mode == 'j7':
                print('\nPlan ready. J7 diagnostic: +3 deg then return; other joint targets fixed.', flush=True)
            if args.plan_only:
                manifest['completed']=True
                print('Plan-only finished. No enable/motion commands sent. Plan:',plan_dir/'plan.json',flush=True)
                return 0
            print('Before RUN_STATIC: remove weight-bearing contact; keep fall protection throughout the path.', flush=True)
            print('Review the selected profile and actual planned ranges. Tracking success is not guaranteed.', flush=True)
            code = run([binary, '--interface', 'can0', '--plan', plan_dir / 'plan.json', '--output-root', session, '--execute'])
            acquisition, meta = single_run(session, 'static')
            manifest['acquisition_dir'] = str(acquisition)
            summarize(acquisition, meta)
            if code or not meta.get('completed'):
                raise RuntimeError('Acquisition did not complete; see captured fault. No automatic retry.')
            checked([sys.executable, HERE / 'torque_model.py', 'compare', '--run', acquisition])
            if args.mode=='proximal':
                checked([sys.executable,HERE/'analyze_proximal_pairs.py','--run',acquisition])
            manifest['completed'] = True
            return 0
        except (Exception, KeyboardInterrupt) as exc:
            manifest['error'] = str(exc) or 'Interrupted by user'
            raise
        finally:
            save(session / 'session.json', manifest)
            print('Session record:', session / 'session.json', flush=True)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\nInterrupted. Check physical support and motor state; disable is not acknowledged.', file=sys.stderr)
        raise SystemExit(130)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)
