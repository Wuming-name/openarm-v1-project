#!/usr/bin/env python3
"""Convenient entry point for the explicitly selected legacy 50-pose profile."""
import sys
import run_static_test

if __name__=='__main__':
    if '--mode' in sys.argv[1:] or any(x.startswith('--mode=') for x in sys.argv[1:]):
        raise SystemExit('This entry point is fixed to pose50; use run_static_test.py for other modes.')
    try:
        raise SystemExit(run_static_test.main(['--mode','pose50']+sys.argv[1:]))
    except KeyboardInterrupt:
        print('Interrupted; check physical support and actual motor state.',file=sys.stderr)
        raise SystemExit(130)
    except (OSError,ValueError,RuntimeError,run_static_test.subprocess.SubprocessError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        raise SystemExit(1)
