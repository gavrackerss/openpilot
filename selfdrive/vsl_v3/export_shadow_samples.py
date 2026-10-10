#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import zipfile
from pathlib import Path

DEFAULT_SOURCE = Path("/data/media/0/xnor_vsl_shadow_samples/v242")
DEFAULT_OUTPUT = Path("/data/media/0/v242_shadow_samples.zip")


def suggested_label(row: dict) -> tuple[str, str]:
  decision = str(row.get("decision", ""))
  reason = str(row.get("capture_reason", decision))
  v3_class = str(row.get("v3_class", ""))
  try:
    ocr_speed = int(row.get("ocr_speed_mph", 0) or 0)
    ocr_count = int(row.get("ocr_count", 0) or 0)
    ocr_conf = float(row.get("ocr_confidence", 0.0) or 0.0)
  except (TypeError, ValueError):
    return "", "invalid_metadata"

  if decision == "V3_CONFLICT" and ocr_count >= 2 and ocr_conf >= 0.72 and ocr_speed > 0:
    return str(ocr_speed), "repeated_ocr_conflict"
  if decision == "V3_RESCUE" and ocr_count >= 1 and str(ocr_speed) == v3_class:
    return v3_class, "v3_matches_raw_ocr"
  if reason in ("V3_OTHER", "V3_SINGLE_OTHER"):
    return "OTHER", "high_confidence_other"
  if reason == "V3_OCR_AGREE" and ocr_speed > 0 and str(ocr_speed) == v3_class:
    return v3_class, "single_v3_ocr_agreement"
  if reason == "V3_OCR_CONFLICT":
    return "", "single_v3_ocr_conflict_review"
  if reason == "V3_SINGLE_STRONG":
    return "", "strong_v3_without_ocr_review"
  return "", "manual_review"


def load_manifest(source: Path) -> list[dict]:
  path = source / "manifest.jsonl"
  rows = []
  if not path.is_file():
    return rows
  with path.open("r", encoding="utf-8") as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try:
        row = json.loads(line)
      except json.JSONDecodeError:
        continue
      file_name = str(row.get("file", ""))
      if file_name and (source / file_name).is_file():
        rows.append(row)
  return rows


def main() -> None:
  ap = argparse.ArgumentParser(description="Package V241 V3 shadow hard-example sign crops for review.")
  ap.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
  ap.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
  args = ap.parse_args()

  rows = load_manifest(args.source)
  if not rows:
    raise SystemExit(f"No captured V241 samples found under {args.source}")

  args.output.parent.mkdir(parents=True, exist_ok=True)
  review_fields = [
    "file", "decision", "capture_reason", "v3_class", "v3_confidence", "v3_consensus",
    "ocr_speed_mph", "ocr_count", "ocr_confidence", "legacy_speed_mph",
    "published_speed_mph", "source", "suggested_label", "suggestion_reason",
    "verified_label",
  ]

  review_rows = []
  for row in rows:
    suggestion, reason = suggested_label(row)
    review_rows.append({
      **{k: row.get(k, "") for k in review_fields},
      "suggested_label": suggestion,
      "suggestion_reason": reason,
      "verified_label": "",
    })

  review_csv = args.source / "review.csv"
  with review_csv.open("w", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=review_fields)
    writer.writeheader()
    writer.writerows(review_rows)

  with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED) as zf:
    zf.write(args.source / "manifest.jsonl", "manifest.jsonl")
    zf.write(review_csv, "review.csv")
    for row in rows:
      p = args.source / str(row["file"])
      zf.write(p, f"crops/{p.name}")

  print(f"V242 shadow samples: {len(rows)}")
  print(f"Review CSV: {review_csv}")
  print(f"Package: {args.output}")


if __name__ == "__main__":
  main()
