from __future__ import annotations

import argparse
import io
import json
import os
import random
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from PIL import Image, ImageEnhance, ImageFilter, ImageOps
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
from torchvision import models, transforms

CLASSES = ["20", "30", "40", "50", "60", "70", "NSL", "OTHER"]
C2I = {c: i for i, c in enumerate(CLASSES)}
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
SEED = 20261009


def synth_clean(im: Image.Image) -> Image.Image:
  im = im.convert("RGB")
  s = min(im.size)
  im = ImageOps.fit(im, (s, s))
  target = random.randint(42, 116)
  sign = im.resize((target, target), Image.Resampling.LANCZOS)
  bgc = random.choice([(60,72,56),(84,84,78),(113,107,96),(138,137,129),(154,151,142),(74,82,88),(112,91,72)])
  arr = np.array(bgc, dtype=np.int16)[None, None, :] + np.random.normal(0, random.uniform(2, 13), (128, 128, 1))
  arr = np.clip(arr, 0, 255).astype(np.uint8)
  bg = Image.fromarray(arr, "RGB").filter(ImageFilter.GaussianBlur(random.uniform(0, .8)))
  yy, xx = np.mgrid[0:target, 0:target]
  c = (target - 1) / 2
  rr = np.sqrt((xx-c)**2 + (yy-c)**2)
  mask = Image.fromarray(((rr < target * .49) * 255).astype(np.uint8))
  x = (128-target)//2 + random.randint(-6, 6)
  y = (128-target)//2 + random.randint(-6, 6)
  bg.paste(sign, (x, y), mask)
  return bg


class CameraAug:
  def __call__(self, im: Image.Image, clean: bool = False) -> Image.Image:
    im = synth_clean(im) if clean else im.convert("RGB")
    if random.random() < .8:
      im = im.rotate(random.uniform(-9, 9), resample=Image.Resampling.BICUBIC, fillcolor=(100, 100, 100))
    if random.random() < .35:
      im = transforms.RandomPerspective(distortion_scale=.16, p=1.0,
        interpolation=transforms.InterpolationMode.BILINEAR, fill=100)(im)
    if random.random() < .9:
      side = random.randint(18, 108)
      im = ImageOps.fit(im, (side, side), method=Image.Resampling.LANCZOS)
      im = im.resize((128, 128), Image.Resampling.BICUBIC)
    else:
      im = ImageOps.fit(im, (128, 128), method=Image.Resampling.LANCZOS)
    if random.random() < .75:
      im = ImageEnhance.Brightness(im).enhance(random.uniform(.45, 1.45))
    if random.random() < .7:
      im = ImageEnhance.Contrast(im).enhance(random.uniform(.65, 1.55))
    if random.random() < .5:
      im = ImageEnhance.Color(im).enhance(random.uniform(.55, 1.3))
    if random.random() < .65:
      im = im.filter(ImageFilter.GaussianBlur(random.uniform(.15, 1.6)))
    if random.random() < .65:
      b = io.BytesIO()
      im.save(b, format="JPEG", quality=random.randint(30, 82))
      b.seek(0)
      im = Image.open(b).convert("RGB")
    return im


class SignDataset(Dataset):
  def __init__(self, items, train: bool):
    self.items = items
    self.train = train
    self.aug = CameraAug()
    self.final = transforms.Compose([
      transforms.Resize((128, 128)),
      transforms.ToTensor(),
      transforms.Normalize(MEAN.tolist(), STD.tolist()),
    ])

  def __len__(self):
    return len(self.items)

  def __getitem__(self, i):
    item = self.items[i]
    im = Image.open(item["path"]).convert("RGB")
    if self.train:
      im = self.aug(im, item.get("clean", False))
    else:
      im = ImageOps.fit(im, (128, 128), method=Image.Resampling.LANCZOS)
    return self.final(im), C2I[item["label"]], item["path"], item.get("group", "")


def load_drive_items(dataset_root: Path):
  df = pd.read_csv(dataset_root / "manifest.csv")
  items = []
  for _, row in df.iterrows():
    items.append({
      "path": str(dataset_root / str(row.file)),
      "label": str(row.label),
      "source": "drive",
      "group": str(row.event_id),
      "route": str(row.route_group),
      "clean": False,
    })
  return items, df


def add_base_items(items, base: Path):
  for label in CLASSES:
    d = base / "sources" / "real" / label
    if d.exists():
      for p in sorted(d.glob("*")):
        if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".webp"}:
          items.append({"path": str(p), "label": label, "source": "wikimedia", "group": "wiki:" + p.name, "clean": False})
  for label in CLASSES:
    p = base / "sources" / "clean" / f"clean_{label}.png"
    if p.exists():
      items.append({"path": str(p), "label": label, "source": "clean", "group": "clean:" + label, "clean": True})
  d = base / "sources" / "clean" / "other_generated"
  if d.exists():
    for p in sorted(d.glob("*.jpg")):
      items.append({"path": str(p), "label": "OTHER", "source": "synthetic_other", "group": "syn:" + p.name, "clean": False})


def build_items(mode: str, dataset_root: Path, base: Path):
  drive, df = load_drive_items(dataset_root)
  fifty_events = sorted(df[(df.label.astype(str) == "50") & (df.route_group == "00000431--913c2d3cf5")].event_id.dropna().unique())
  fifty_val = set(fifty_events[-2:]) if len(fifty_events) >= 2 else set()
  train, val = [], []
  for item in drive:
    hold = mode == "eval" and (item["route"] == "0000042f--682a958432" or (item["label"] == "50" and item["group"] in fifty_val))
    (val if hold else train).append(item)
  add_base_items(train, base)
  return train, val, fifty_val, df


def evaluate(model, loader, device):
  model.eval()
  cm = np.zeros((len(CLASSES), len(CLASSES)), dtype=np.int64)
  rows = []
  with torch.no_grad():
    for x, y, paths, groups in loader:
      probs = model(x.to(device)).softmax(1).cpu().numpy()
      pred = probs.argmax(1)
      yt = y.numpy()
      for a, b, p, g, pr in zip(yt, pred, paths, groups, probs):
        cm[a, b] += 1
        rows.append({"path": p, "group": g, "actual": CLASSES[a], "pred": CLASSES[b], "confidence": float(pr[b])})
  present = np.where(cm.sum(1) > 0)[0]
  recalls = [cm[i, i] / cm[i].sum() for i in present]
  return float(np.trace(cm) / max(cm.sum(), 1)), float(np.mean(recalls)) if recalls else 0.0, cm, rows


def make_model(init_path: Path):
  model = models.mobilenet_v3_small(weights=None)
  model.classifier[3] = nn.Linear(model.classifier[3].in_features, len(CLASSES))
  ck = torch.load(init_path, map_location="cpu", weights_only=False)
  model.load_state_dict(ck["state_dict"] if "state_dict" in ck else ck)
  for block in list(model.features)[:3]:
    for p in block.parameters():
      p.requires_grad = False
  return model


def train(mode: str, out: Path, epochs: int, init_path: Path, dataset_root: Path, base: Path):
  random.seed(SEED); np.random.seed(SEED); torch.manual_seed(SEED)
  torch.set_num_threads(min(8, os.cpu_count() or 4))
  tr, va, fifty_val, df = build_items(mode, dataset_root, base)
  out.mkdir(parents=True, exist_ok=True)
  print("MODE", mode, "TRAIN", len(tr), "VAL", len(va), "50_VAL_EVENTS", sorted(fifty_val), flush=True)
  print("TRAIN_COUNTS", Counter(x["label"] for x in tr), flush=True)
  print("VAL_COUNTS", Counter(x["label"] for x in va), flush=True)

  ds = SignDataset(tr, True)
  counts = Counter(x["label"] for x in tr)
  weights = [1.0 / max(counts[x["label"]], 1) for x in tr]
  sampler = WeightedRandomSampler(weights, num_samples=2400 if mode == "eval" else 3000, replacement=True)
  train_loader = DataLoader(ds, batch_size=64, sampler=sampler, num_workers=2)
  val_loader = DataLoader(SignDataset(va, False), batch_size=96, shuffle=False, num_workers=2) if va else None

  model = make_model(init_path)
  device = torch.device("cpu")
  model.to(device)
  opt = torch.optim.AdamW(filter(lambda p: p.requires_grad, model.parameters()), lr=1.3e-4 if mode == "eval" else 6e-5, weight_decay=1e-4)
  sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=max(epochs, 1), eta_min=1e-5)
  lossfn = nn.CrossEntropyLoss(label_smoothing=.03)
  history, best, best_metric = [], None, -1e9

  for ep in range(epochs):
    model.train(); total = 0.0; seen = 0
    for x, y, _, _ in train_loader:
      opt.zero_grad(set_to_none=True)
      logits = model(x.to(device))
      loss = lossfn(logits, y.to(device))
      loss.backward(); opt.step()
      total += float(loss.detach()) * len(y); seen += len(y)
    sched.step()
    rec = {"epoch": ep + 1, "loss": total / max(seen, 1), "lr": opt.param_groups[0]["lr"]}
    metric = -rec["loss"]
    if val_loader:
      acc, bal, _, _ = evaluate(model, val_loader, device)
      rec.update({"val_accuracy": acc, "val_balanced_accuracy": bal})
      metric = bal
    history.append(rec); print("EPOCH", json.dumps(rec), flush=True)
    if metric > best_metric:
      best_metric = metric
      best = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}

  model.load_state_dict(best)
  val_report = {}
  if val_loader:
    acc, bal, cm, rows = evaluate(model, val_loader, device)
    val_report = {"accuracy": acc, "balanced_accuracy": bal, "confusion_matrix": cm.tolist(), "predictions": rows}

  pt = out / ("uk_speed_classifier_v3_drive_eval.pt" if mode == "eval" else "uk_speed_classifier_v3_drive_final.pt")
  torch.save({"state_dict": model.state_dict(), "classes": CLASSES, "input_size": 128, "mode": mode}, pt)
  report = {
    "mode": mode, "classes": CLASSES,
    "train_counts": dict(Counter(x["label"] for x in tr)),
    "val_counts": dict(Counter(x["label"] for x in va)),
    "drive_total_counts": dict(Counter(df.label.astype(str))),
    "history": history, "validation": val_report, "init_model": str(init_path),
  }
  (out / "report.json").write_text(json.dumps(report, indent=2))
  return model, pt, report


def preprocess_cv(path: str):
  bgr = cv2.imread(path)
  if bgr is None:
    raise RuntimeError(f"cannot read {path}")
  h, w = bgr.shape[:2]; s = min(h, w); x = (w-s)//2; y = (h-s)//2
  rgb = cv2.cvtColor(bgr[y:y+s, x:x+s], cv2.COLOR_BGR2RGB)
  rgb = cv2.resize(rgb, (128,128), interpolation=cv2.INTER_AREA if s > 128 else cv2.INTER_LINEAR)
  arr = rgb.astype(np.float32) / 255.0
  arr = (arr - MEAN) / STD
  return np.transpose(arr, (2,0,1))[None]


def export_and_verify(model, onnx_path: Path, dataset_root: Path):
  model.eval().cpu()
  dummy = torch.randn(1,3,128,128)
  torch.onnx.export(model, dummy, onnx_path, input_names=["image"], output_names=["logits"],
                    opset_version=17, do_constant_folding=True, dynamo=False, external_data=False)
  net = cv2.dnn.readNetFromONNX(str(onnx_path))
  manifest = pd.read_csv(dataset_root / "manifest.csv")
  sample_paths = [str(dataset_root / x) for x in manifest.file.iloc[::max(1, len(manifest)//24)].head(24)]
  max_abs = 0.0; disagreements = 0
  for p in sample_paths:
    inp = preprocess_cv(p)
    with torch.no_grad():
      t = model(torch.from_numpy(inp)).numpy()
    net.setInput(inp); c = net.forward()
    max_abs = max(max_abs, float(np.max(np.abs(t-c))))
    disagreements += int(int(np.argmax(t)) != int(np.argmax(c)))
  if disagreements or max_abs > 2e-3:
    raise RuntimeError(f"ONNX parity failed: disagreements={disagreements} max_abs={max_abs}")
  return {"samples": len(sample_paths), "argmax_disagreements": disagreements, "max_abs_logit_diff": max_abs}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--dataset-root", required=True)
  ap.add_argument("--base-v2", required=True)
  ap.add_argument("--out", required=True)
  args = ap.parse_args()
  dataset_root = Path(args.dataset_root); base = Path(args.base_v2); out = Path(args.out)
  eval_model, eval_pt, eval_report = train("eval", out / "eval", 8, base / "uk_speed_classifier_v2_poc.pt", dataset_root, base)
  if eval_report["validation"]["balanced_accuracy"] < 0.65:
    raise RuntimeError(f"route-aware balanced accuracy regressed: {eval_report['validation']['balanced_accuracy']:.3f}")
  final_model, final_pt, final_report = train("final", out / "final", 5, eval_pt, dataset_root, base)
  onnx_path = out / "speed_limit_v3_classifier.onnx"
  parity = export_and_verify(final_model, onnx_path, dataset_root)
  summary = {
    "classes": CLASSES,
    "supplied_video_files": 51,
    "unique_video_contents": 49,
    "verified_drive_crops": int(len(pd.read_csv(dataset_root / "manifest.csv"))),
    "route_aware_eval_accuracy": eval_report["validation"]["accuracy"],
    "route_aware_eval_balanced_accuracy": eval_report["validation"]["balanced_accuracy"],
    "onnx_parity": parity,
    "shadow_only": True,
  }
  (out / "build_report.json").write_text(json.dumps(summary, indent=2))
  print("BUILD_REPORT", json.dumps(summary, indent=2), flush=True)

if __name__ == "__main__":
  main()
