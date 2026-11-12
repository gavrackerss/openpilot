#!/usr/bin/env python3
"""XNOR V275 automatic on-road shadow observation (NO speed-control authority).

Manager invokes this module only for on-road Tesla and only when the V275
installation marker exists. It never publishes a speed limit, sends CAN or
changes Tesla settings. All observations go to a JSONL file under /data.
"""
import os
import sys
from pathlib import Path

BASE=Path('/data/V275_Shadow')
LOGS=Path('/data/V275_Shadow_logs')


def main():
    if not (BASE/'ENABLED').is_file():
        print('V275 automatic shadow disabled; exiting')
        return 0
    # Voluntary low priority: never steal scheduling priority from modeld/controlsd.
    try:os.nice(15)
    except OSError:pass
    sys.path.insert(0,str(BASE))
    os.environ.update({'V275_MANAGED':'1','OPENBLAS_NUM_THREADS':'1',
                       'OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'})
    # Avoid endlessly respawning if module imports or hardware dependencies fail.
    try:
        from shadow import main as shadow_main
        sys.argv=['v275_shadow_manager','--live','--mode','auto',
                  '--sample-seconds','5','--max-infer-seconds','12',
                  '--max-seconds','0','--max-samples','0',
                  '--max-candidates','5',
                  '--log-file',str(LOGS/'observations.jsonl')]
        result=shadow_main()
        return int(result or 0)
    except BaseException as exc:
        if isinstance(exc,SystemExit) and exc.code in (0,None):
            return 0
        try:
            (BASE/'ENABLED').unlink(missing_ok=True)
            (BASE/'AUTO_FAULT.json').write_text('{"reason":"manager_exception; check process logs"}\n')
        except Exception:
            pass
        raise

if __name__=='__main__':
    raise SystemExit(main())
