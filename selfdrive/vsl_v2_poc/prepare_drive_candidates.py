#!/usr/bin/env python3
"""Extract UNLABELLED UK speed-sign candidates from ZIPs of drive footage.

This is an offline review tool, not an automatic labeller. It will not train
or update the vehicle model. Review the numbered candidate images and assign
ground-truth labels before adding them to a training dataset.

Install: pip install opencv-python-headless numpy pillow
Usage:
  python selfdrive/vsl_v2_poc/prepare_drive_candidates.py "D:/drive-archives/*.zip" --output drive_candidates

Results: candidates/*.jpg, review_sheets/*.jpg, candidates.csv, report.json.
Supports video and still-image members of ZIP files. Videos are extracted
one at a time to a temporary directory, not all at once.
"""
from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import math
import re
import tempfile
import zipfile
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageOps

VIDEOS = {".mp4", ".mov", ".mkv", ".avi", ".hevc", ".h265", ".h264", ".ts", ".m4v", ".webm"}
IMAGES = {".jpg", ".jpeg", ".png", ".webp", ".bmp"}
FIELDS = ("candidate_id", "archive", "member", "time_s", "group", "image", "label", "source")


def grouping(archive: Path) -> str:
    # Archives like 0000042f--682a958432--12F.zip share the same route.
    parts = archive.stem.split("--")
    return "--".join(parts[:2]) if len(parts) >= 3 else archive.stem


def signature(archive: zipfile.ZipFile):
    # Detect archive duplicates without decompressing or hashing gigabytes.
    return tuple(sorted((x.filename, x.file_size, x.CRC) for x in archive.infolist()
                        if not x.is_dir()))


def dhash(rgb: np.ndarray) -> int:
    g = cv2.cvtColor(cv2.resize(rgb, (9, 8)), cv2.COLOR_RGB2GRAY)
    bits = (g[:, 1:] > g[:, :-1]).reshape(-1)
    return sum(int(bit) << i for i, bit in enumerate(bits))


def proposals(bgr: np.ndarray, min_diameter: int = 12):
    """Find red circular sign-like objects; classes must be reviewed manually."""
    h, w = bgr.shape[:2]
    scale = min(1.0, 1440.0 / max(h, w))
    if scale < 1:
        bgr = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    r = cv2.inRange(hsv, (0, 65, 45), (13, 255, 255)) | cv2.inRange(hsv, (167, 65, 45), (179, 255, 255))
    r = cv2.morphologyEx(r, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    scored = []
    hh, ww = bgr.shape[:2]
    for c in contours:
        x, y, cw, ch = cv2.boundingRect(c)
        if min(cw, ch) < min_diameter or cw > ww * 0.55 or ch > hh * 0.55:
            continue
        aspect = cw / max(ch, 1)
        if not 0.7 <= aspect <= 1.4:
            continue
        circumference = cv2.arcLength(c, True)
        circularity = 4 * math.pi * cv2.contourArea(c) / max(circumference * circumference, 1.0)
        if circularity < 0.13:
            continue
        roi = hsv[y:y + ch, x:x + cw]
        if roi.size == 0:
            continue
        # White centre is a useful heuristic, not a verified numeric label.
        inset = roi[ch // 4:max(ch // 4 + 1, 3 * ch // 4),
                    cw // 4:max(cw // 4 + 1, 3 * cw // 4)]
        white = np.mean((inset[:, :, 1] < 110) & (inset[:, :, 2] > 115))
        if white < 0.13:
            continue
        score = math.sqrt(cw * ch) * (0.4 + min(circularity, 1) + white)
        scored.append((score, x, y, cw, ch))
    scored.sort(reverse=True)
    for _score, x, y, cw, ch in scored[:5]:
        pad = max(3, int(0.23 * max(cw, ch)))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(ww, x + cw + pad), min(hh, y + ch + pad)
        crop = bgr[y0:y1, x0:x1]
        if min(crop.shape[:2]) < 20:
            continue
        yield cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)


def create_sheets(paths: list[Path], out: Path):
    out.mkdir(parents=True, exist_ok=True)
    for start in range(0, len(paths), 80):
        selected = paths[start:start + 80]
        sheet = Image.new("RGB", (10 * 130, 8 * 160), "white")
        draw = ImageDraw.Draw(sheet)
        for i, path in enumerate(selected):
            with Image.open(path) as im:
                thumb = ImageOps.contain(im.convert("RGB"), (118, 125))
            x, y = (i % 10) * 130 + 6, (i // 10) * 160 + 4
            sheet.paste(thumb, (x + (118 - thumb.width) // 2, y))
            draw.text((x, y + 130), path.stem, fill="black")
        sheet.save(out / f"sheet_{start // 80 + 1:03d}.jpg", quality=86)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("archives", nargs="+", help="ZIP paths or quoted wildcard, e.g. *.zip")
    parser.add_argument("--output", default="drive_candidates")
    parser.add_argument("--sample-seconds", type=float, default=0.5)
    parser.add_argument("--max-candidates", type=int, default=3000)
    parser.add_argument("--max-frames-per-video", type=int, default=20000)
    args = parser.parse_args()
    if args.sample_seconds <= 0 or args.max_candidates < 1:
        parser.error("sample-seconds and max-candidates must be positive")
    archives = []
    for pattern in args.archives:
        archives.extend(Path(x) for x in glob.glob(pattern))
    archives = sorted({x.resolve() for x in archives if x.is_file() and x.suffix.lower() == ".zip"})
    if not archives:
        parser.error("No ZIP archives found")
    out = Path(args.output)
    crop_dir = out / "candidates"
    crop_dir.mkdir(parents=True, exist_ok=True)
    rows, paths, seen_archives, video_stats = [], [], set(), Counter()
    # Skip near-identical proposals *within the same recording*, never across
    # different routes. Route-level independence is retained in candidates.csv.
    last_hash_by_member = {}

    def write_candidates(bgr, archive, member, seconds, source):
        if len(rows) >= args.max_candidates:
            return
        key = (archive.name, member)
        recent = last_hash_by_member.setdefault(key, [])
        for rgb in proposals(bgr):
            hash_value = dhash(rgb)
            if any((hash_value ^ h).bit_count() <= 6 and abs(seconds - t) < 8 for h, t in recent):
                continue
            recent.append((hash_value, seconds))
            if len(recent) > 40:
                recent.pop(0)
            candidate_id = f"{len(rows) + 1:06d}"
            dest = crop_dir / (candidate_id + ".jpg")
            Image.fromarray(rgb).save(dest, quality=94)
            paths.append(dest)
            rows.append({
                "candidate_id": candidate_id, "archive": archive.name,
                "member": member, "time_s": f"{seconds:.3f}",
                "group": grouping(archive), "image": str(dest),
                "label": "", "source": source,
            })
            if len(rows) >= args.max_candidates:
                return

    for archive in archives:
        if len(rows) >= args.max_candidates:
            break
        try:
            with zipfile.ZipFile(archive) as z:
                sig = signature(z)
                if sig in seen_archives:
                    video_stats["duplicate_archives_skipped"] += 1
                    print("SKIP duplicate archive", archive, flush=True)
                    continue
                seen_archives.add(sig)
                members = [x for x in z.infolist() if not x.is_dir()]
                print(f"ARCHIVE {archive.name}: {len(members)} entries", flush=True)
                for info in members:
                    if len(rows) >= args.max_candidates:
                        break
                    ext = Path(info.filename).suffix.lower()
                    if ext in IMAGES and info.file_size <= 50 * 1024 * 1024:
                        data = z.read(info)
                        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                        if frame is not None:
                            write_candidates(frame, archive, info.filename, 0.0, "still")
                            video_stats["images_seen"] += 1
                    elif ext in VIDEOS:
                        with tempfile.TemporaryDirectory(prefix="vsl-drive-") as directory:
                            temp = Path(directory) / ("footage" + ext)
                            with z.open(info) as reader, open(temp, "wb") as writer:
                                while block := reader.read(4 * 1024 * 1024):
                                    writer.write(block)
                            cap = cv2.VideoCapture(str(temp))
                            if not cap.isOpened():
                                video_stats["videos_unreadable"] += 1
                                print("WARN cannot decode", info.filename, flush=True)
                                continue
                            fps = cap.get(cv2.CAP_PROP_FPS)
                            if not 0.5 <= fps <= 240:
                                fps = 20.0
                            stride = max(1, round(fps * args.sample_seconds))
                            index = 0
                            while index < args.max_frames_per_video and len(rows) < args.max_candidates:
                                ok = cap.grab()
                                if not ok:
                                    break
                                if index % stride == 0:
                                    ok, frame = cap.retrieve()
                                    if ok:
                                        write_candidates(frame, archive, info.filename, index / fps, "video")
                                        video_stats["sampled_frames"] += 1
                                index += 1
                            cap.release()
                            video_stats["videos_seen"] += 1
                video_stats["archives_seen"] += 1
        except (OSError, zipfile.BadZipFile) as exc:
            print(f"ERROR {archive.name}: {exc}", flush=True)
            video_stats["archives_failed"] += 1
    with open(out / "candidates.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    create_sheets(paths, out / "review_sheets")
    report = {
        **dict(video_stats), "candidate_count": len(rows),
        "route_groups": dict(Counter(x["group"] for x in rows)),
        "archives_requested": len(archives), "archives_unique": len(seen_archives),
        "status": "UNLABELLED - human review required before training",
        "note": "Red roundel candidate detector. NSL/negative examples need separate review; no class inferred automatically.",
    }
    (out / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
