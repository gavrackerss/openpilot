#!/usr/bin/env bash
set -euo pipefail
cd "${1:-/data/openpilot}"

python - <<'PY'
import hashlib
import inspect

import opendbc.car.tesla.carcontroller as carcontroller
import opendbc.car.tesla.carstate as carstate
import opendbc.car.tesla.teslacan as teslacan
import opendbc.car.tesla.teslacan_legacy as teslacan_legacy

expected = {
  "carcontroller": "92c010676563b37b96104e434ee89bf2d86fbf366ae2cebeaa94f7da48fd0587",
  "carstate": "a9e331a973c8ad4cccad51f7dc5b90502c2d78c6b08dfc59b138dfcd3f5f66e2",
  "teslacan": "8386e86a182fbec5d6a50bef087aff4d06754483294230373caa03fbbdf74932",
  "teslacan_legacy": "18365eecc2351aebceba9cf574cea48c384e8c6fd6cdec757280fc03fe478c0f",
}
modules = {
  "carcontroller": carcontroller,
  "carstate": carstate,
  "teslacan": teslacan,
  "teslacan_legacy": teslacan_legacy,
}

failed = False
for name, module in modules.items():
  path = inspect.getsourcefile(module)
  digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
  ok = digest == expected[name]
  print(f"{name}: {digest} {'OK' if ok else 'MISMATCH'} {path}")
  failed = failed or not ok

raise SystemExit(1 if failed else 0)
PY
