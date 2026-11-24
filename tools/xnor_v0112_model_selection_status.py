#!/usr/bin/env python3
"""Read-only SunnyPilot saved selection status for staged XNOR 0.11.2.

Offline JSON mode reads one local file and works on a workstation without
compiled Params/Cereal. Device mode requires an explicit confirmed offroad
state. Neither mode writes Params, downloads files, changes modeld selection
or activates a custom model.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

# Standalone workstation invocation via "python tools/..." does not promise
# that the repository root is already on sys.path.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
  sys.path.insert(0, str(ROOT))

from openpilot.sunnypilot.models.catalog_guard import audit_bundle
from openpilot.sunnypilot.models.artifact_audit import verify_artifacts


def read_selection(offline_json: Path | None):
  if offline_json is not None:
    if not offline_json.is_file():
      raise ValueError("Offline bundle JSON file does not exist")
    if offline_json.stat().st_size > 2 * 1024 * 1024:
      raise ValueError("Offline bundle JSON is too large")
    return json.loads(offline_json.read_text(encoding="utf-8"))
  # Import the native library only after the user explicitly chooses device
  # mode; never open persistent state in workstation/offline JSON mode.
  from openpilot.common.params import Params
  params = Params()
  if not params.get_bool("IsOffroad"):
    raise RuntimeError("Device status inspection requires confirmed IsOffroad=True")
  return params.get("ModelManager_ActiveBundle")


def main(argv=None) -> int:
  parser = argparse.ArgumentParser(description="Read-only native 0.11.2 SunnyPilot selection audit")
  parser.add_argument("--offline-json", type=Path,
                      help="Inspect an exported JSON bundle without opening Params")
  parser.add_argument('--artifact-root', type=Path, help='Verify saved artifact bytes in this directory without deserialising')
  args = parser.parse_args(argv)
  try:
    selected = read_selection(args.offline_json)
    result = audit_bundle(selected)
    output = asdict(result)
    output.update({
      "native_runtime": "XNOR 0.11.2",
      "catalog_manager_activated": False,
      "custom_model_authority": False,
      "notice": "Metadata-only inspection; stock native modeld remains active",
    })
    if args.artifact_root is not None:
      output['artifact_audit'] = asdict(verify_artifacts(selected, args.artifact_root))
      output['notice'] = 'Artifact integrity inspection; native ABI unverified; stock remains active'
    print(json.dumps(output, indent=2, sort_keys=True))
    return 0
  except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
    parser.exit(2, f"Cannot inspect selection: {exc}\n")


if __name__ == "__main__":
  raise SystemExit(main())
