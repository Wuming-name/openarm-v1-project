#!/usr/bin/env python3
"""One entry point for wide proximal coverage and paired direction sampling."""
import sys
import run_static_test

if __name__=='__main__':
    if '--mode' in sys.argv[1:] or any(x.startswith('--mode=') for x in sys.argv[1:]):
        raise SystemExit('Use run_static_test.py to select a different profile.')
    try:
        raise SystemExit(run_static_test.main(['--mode','proximal']+sys.argv[1:]))
    except KeyboardInterrupt:
        print('Interrupted; check physical support and actual motor state.',file=sys.stderr)
        raise SystemExit(130)
    except (OSError,ValueError,RuntimeError,run_static_test.subprocess.SubprocessError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr)
        raise SystemExit(1)
