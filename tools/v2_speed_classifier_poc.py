#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
import random
import shutil
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import requests
import torch
from PIL import Image
from torch import nn
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
from torchvision.models import MobileNet_V3_Small_Weights, mobilenet_v3_small

SEED = 20261009
random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)

ROOT = Path(os.environ.get("V2_OUT", "v2_poc_output"))
RAW = ROOT / "raw"
CROPS = ROOT / "crops"
CANON = ROOT / "canonical"
for p in (ROOT, RAW, CROPS, CANON):
  p.mkdir(parents=True, exist_ok=True)

CLASSES = ["20", "30", "40", "50", "60", "70", "NSL", "OTHER"]
LABEL_TO_IDX = {v: i for i, v in enumerate(CLASSES)}
COMMONS_API = "https://commons.wikimedia.org/w/api.php"
UA = "openpilot-xnor-v2-speed-sign-poc/1.0 (research; GitHub Actions)"

# Root categories are recursively traversed, which picks up England/Scotland/
# Wales/NI subcategories without hard-coding every regional page.
REAL_CATEGORIES = {
  "20": ["Category:20 mph speed limit road signs in the United Kingdom"],
  "30": ["Category:30 mph speed limit road signs in the United Kingdom"],
  "40": ["Category:40 mph speed limit road signs in the United Kingdom"],
  "50": ["Category:50 mph speed limit road signs in the United Kingdom"],
  "60": ["Category:60 mph speed limit road signs in the United Kingdom"],
  "70": ["Category:70 mph speed limit road signs in the United Kingdom"],
  "NSL": ["Category:Speed limit de-restriction road signs in the United Kingdom"],
  "OTHER": [
    "Category:No entry road signs in the United Kingdom",
    "Category:Minimum speed road signs in the United Kingdom",
    "Category:Weight limit road signs in the United Kingdom",
  ],
}

MAX_REAL = {
  "20": 70, "30": 90, "40": 70, "50": 60,
  "60": 45, "70": 45, "NSL": 60, "OTHER": 90,
}

CANONICAL_FILES = {
  "20": "File:UK traffic sign 670V20.svg",
  "30": "File:UK traffic sign 670V30.svg",
  "40": "File:UK traffic sign 670V40.svg",
  "50": "File:UK traffic sign 670V50.svg",
  "60": "File:UK traffic sign 670V60.svg",
  "70": "File:UK traffic sign 670V70.svg",
  "NSL": "File:UK traffic sign 671.svg",
  # 10 is intentionally a negative for the current runtime's supported set.
  "OTHER": "File:UK traffic sign 670V10.svg",
}

session = requests.Session()
session.headers.update({"User-Agent": UA})


def api(params, tries=4):
  params = dict(params)
  params["format"] = "json"
  params["origin"] = "*"
  last = None
  for i in range(tries):
    try:
      r = session.get(COMMONS_API, params=params, timeout=30)
      r.raise_for_status()
      return r.json()
    except Exception as e:
      last = e
      time.sleep(1.0 + i)
  raise last


def category_members(category: str, recurse: int = 2):
  files = []
  seen_cats = set()

  def walk(cat: str, depth: int):
    if cat in seen_cats:
      return
    seen_cats.add(cat)
    cont = None
    while True:
      params = {
        "action": "query", "list": "categorymembers", "cmtitle": cat,
        "cmlimit": "500", "cmtype": "file|subcat",
      }
      if cont:
        params["cmcontinue"] = cont
      data = api(params)
      for item in data.get("query", {}).get("categorymembers", []):
        ns = int(item.get("ns", -1))
        title = str(item.get("title", ""))
        if ns == 6:
          files.append(title)
        elif ns == 14 and depth > 0:
          walk(title, depth - 1)
      cont = data.get("continue", {}).get("cmcontinue")
      if not cont:
        break

  walk(category, recurse)
  # deterministic and unique
  return list(dict.fromkeys(files))


def image_url(title: str, width: int = 1600):
  data = api({
    "action": "query",
    "titles": title,
    "prop": "imageinfo",
    "iiprop": "url|mime|size",
    "iiurlwidth": str(width),
  })
  for page in data.get("query", {}).get("pages", {}).values():
    info = page.get("imageinfo", [])
    if not info:
      continue
    x = info[0]
    mime = str(x.get("mime", ""))
    if not mime.startswith("image/"):
      return None
    return x.get("thumburl") or x.get("url")
  return None


def download_image(url: str):
  if not url:
    return None
  try:
    r = session.get(url, timeout=45)
    r.raise_for_status()
    arr = np.frombuffer(r.content, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img
  except Exception:
    return None


def resize_max(img, max_side=1600):
  h, w = img.shape[:2]
  scale = min(1.0, max_side / max(h, w))
  if scale < 1.0:
    return cv2.resize(img, (int(round(w*scale)), int(round(h*scale))), interpolation=cv2.INTER_AREA)
  return img


def square_pad_crop(img, box, pad=0.22):
  h, w = img.shape[:2]
  x1, y1, x2, y2 = [float(v) for v in box]
  cx, cy = (x1+x2)*0.5, (y1+y2)*0.5
  side = max(x2-x1, y2-y1) * (1.0 + 2.0*pad)
  side = max(side, 12.0)
  ax1 = max(int(round(cx-side/2)), 0)
  ay1 = max(int(round(cy-side/2)), 0)
  ax2 = min(int(round(cx+side/2)), w)
  ay2 = min(int(round(cy+side/2)), h)
  if ax2 <= ax1 or ay2 <= ay1:
    return None
  crop = img[ay1:ay2, ax1:ax2]
  if crop.size == 0:
    return None
  return crop


def numeric_roundel_crop(img):
  img = resize_max(img)
  h, w = img.shape[:2]
  hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
  hue, sat, val = hsv[:,:,0], hsv[:,:,1], hsv[:,:,2]
  red = ((((hue <= 13) | (hue >= 167)) & (sat >= 65) & (val >= 50))).astype(np.uint8)*255
  red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))
  contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  candidates = []
  image_area = float(max(h*w, 1))
  for c in contours:
    area = float(cv2.contourArea(c))
    if area < max(18.0, image_area*0.00002):
      continue
    x, y, cw, ch = cv2.boundingRect(c)
    if cw < 12 or ch < 12:
      continue
    aspect = cw / max(ch, 1)
    if not 0.55 <= aspect <= 1.75:
      continue
    # Use a padded candidate to measure ring/centre geometry.
    crop = square_pad_crop(img, (x,y,x+cw,y+ch), pad=0.24)
    if crop is None:
      continue
    hh, ww = crop.shape[:2]
    hsv2 = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    H,S,V = hsv2[:,:,0], hsv2[:,:,1], hsv2[:,:,2]
    r = (((H<=13)|(H>=167)) & (S>=60) & (V>=45))
    white = ((V>=120) & (S<=105))
    dark = (V<=135)
    yy, xx = np.mgrid[:hh,:ww]
    nx=(xx-(ww-1)/2)/max(ww/2,1)
    ny=(yy-(hh-1)/2)/max(hh/2,1)
    rr=np.sqrt(nx*nx+ny*ny)
    ring=(rr>=0.45)&(rr<=0.96)
    centre=rr<=0.46
    ring_red=float(r[ring].mean()) if ring.any() else 0.0
    centre_white=float(white[centre].mean()) if centre.any() else 0.0
    centre_dark=float(dark[centre].mean()) if centre.any() else 0.0
    red_ratio=float(r.mean())
    if ring_red < 0.045 or centre_white < 0.10 or centre_dark < 0.003:
      continue
    circularity = 4.0*math.pi*area/max(float(cv2.arcLength(c,True))**2, 1.0)
    score = (
      ring_red*3.0 + centre_white*1.8 + min(centre_dark/0.18,1.0)*0.7 +
      min(max(circularity,0.0),1.0)*0.5 + min(cw*ch/image_area*80.0,1.0)*0.4
    )
    candidates.append((score, crop))

  if candidates:
    candidates.sort(key=lambda t:t[0], reverse=True)
    return candidates[0][1]

  # Hough fallback: useful when the red perimeter is fragmented.
  gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  scale = min(1.0, 1000.0/max(w,1))
  small = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA) if scale < 1 else gray
  sm = cv2.GaussianBlur(small,(5,5),1.2)
  circles = cv2.HoughCircles(sm,cv2.HOUGH_GRADIENT,dp=1.2,minDist=25,param1=100,param2=25,
                            minRadius=8,maxRadius=max(10,int(min(sm.shape[:2])*0.25)))
  if circles is not None:
    best=None
    for cx,cy,radius in np.round(circles[0]).astype(int)[:50]:
      if radius<=0: continue
      x1=(cx-radius)/scale; y1=(cy-radius)/scale
      x2=(cx+radius)/scale; y2=(cy+radius)/scale
      crop=square_pad_crop(img,(x1,y1,x2,y2),pad=0.12)
      if crop is None: continue
      hsv2=cv2.cvtColor(crop,cv2.COLOR_BGR2HSV)
      H,S,V=hsv2[:,:,0],hsv2[:,:,1],hsv2[:,:,2]
      rmask=(((H<=13)|(H>=167))&(S>=55)&(V>=45))
      white=((V>=115)&(S<=115))
      hh,ww=crop.shape[:2]
      yy,xx=np.mgrid[:hh,:ww]
      rr=np.sqrt(((xx-(ww-1)/2)/max(ww/2,1))**2+((yy-(hh-1)/2)/max(hh/2,1))**2)
      ring=(rr>=.55)&(rr<=.96); centre=rr<=.48
      score=float(rmask[ring].mean())*3+float(white[centre].mean())*2
      if best is None or score>best[0]: best=(score,crop)
    if best is not None and best[0]>=0.35:
      return best[1]
  return None


def nsl_crop(img):
  img = resize_max(img)
  h,w=img.shape[:2]
  gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
  scale=min(1.0,1000.0/max(w,1))
  sm=cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA) if scale<1 else gray
  sm=cv2.GaussianBlur(sm,(5,5),1.2)
  circles=cv2.HoughCircles(sm,cv2.HOUGH_GRADIENT,dp=1.2,minDist=30,param1=100,param2=24,
                          minRadius=10,maxRadius=max(12,int(min(sm.shape[:2])*.28)))
  best=None
  if circles is not None:
    for cx,cy,radius in np.round(circles[0]).astype(int)[:60]:
      x1=(cx-radius)/scale; y1=(cy-radius)/scale
      x2=(cx+radius)/scale; y2=(cy+radius)/scale
      crop=square_pad_crop(img,(x1,y1,x2,y2),pad=.10)
      if crop is None: continue
      c=cv2.resize(crop,(128,128),interpolation=cv2.INTER_AREA)
      hsv=cv2.cvtColor(c,cv2.COLOR_BGR2HSV); S=hsv[:,:,1]; V=hsv[:,:,2]
      white=(V>=120)&(S<=100); dark=V<=115
      yy,xx=np.mgrid[:128,:128]; nx=(xx-63.5)/64; ny=(yy-63.5)/64; rr=np.sqrt(nx*nx+ny*ny)
      face=rr<=.82
      # UK NSL has a white face with a broad diagonal black stripe.
      stripe=(np.abs((nx+ny)-0.05)<=0.24)&face
      opp=(np.abs((nx+ny)+0.65)<=0.15)&face
      sw=float(dark[stripe].mean()); fw=float(white[face].mean()); od=float(dark[opp].mean())
      score=sw*2.4+fw*1.7-max(od-.45,0)*.5
      if best is None or score>best[0]: best=(score,crop)
  if best is not None and best[0] >= 1.0:
    return best[1]

  # Category images are often tight sign photographs. A conservative central
  # fallback is allowed only when most of the centre is bright and dark-striped.
  side=min(h,w)
  c=img[(h-side)//2:(h+side)//2,(w-side)//2:(w+side)//2]
  c128=cv2.resize(c,(128,128),interpolation=cv2.INTER_AREA)
  hsv=cv2.cvtColor(c128,cv2.COLOR_BGR2HSV); S=hsv[:,:,1]; V=hsv[:,:,2]
  white=(V>=125)&(S<=95); dark=V<=110
  yy,xx=np.mgrid[:128,:128]; nx=(xx-63.5)/64; ny=(yy-63.5)/64; rr=np.sqrt(nx*nx+ny*ny)
  face=rr<=.80; stripe=(np.abs((nx+ny)-.05)<=.24)&face
  if float(white[face].mean())>=.40 and float(dark[stripe].mean())>=.28:
    return c
  return None


def other_crop(img):
  # Prefer a circular/red regulatory object; otherwise no sample.
  crop=numeric_roundel_crop(img)
  if crop is not None:
    return crop
  img=resize_max(img)
  hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
  H,S,V=hsv[:,:,0],hsv[:,:,1],hsv[:,:,2]
  blue=((H>=90)&(H<=135)&(S>=70)&(V>=45)).astype(np.uint8)*255
  contours,_=cv2.findContours(blue,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  cand=[]
  for c in contours:
    x,y,w,h=cv2.boundingRect(c)
    if w<18 or h<18: continue
    a=w/max(h,1)
    if .6<=a<=1.65:
      crop=square_pad_crop(img,(x,y,x+w,y+h),pad=.22)
      if crop is not None: cand.append((w*h,crop))
  if cand:
    cand.sort(key=lambda x:x[0],reverse=True); return cand[0][1]
  return None


def save_crop(label, idx, crop, prefix):
  if crop is None or crop.size==0: return None
  h,w=crop.shape[:2]
  if min(h,w)<18: return None
  out=CROPS/label/f"{prefix}_{idx:04d}.jpg"
  out.parent.mkdir(parents=True,exist_ok=True)
  cv2.imwrite(str(out),crop,[cv2.IMWRITE_JPEG_QUALITY,94])
  return out


def get_canonical():
  rows=[]
  for label,title in CANONICAL_FILES.items():
    try:
      url=image_url(title,700)
      img=download_image(url)
      if img is None: continue
      out=CANON/f"{label}.png"
      cv2.imwrite(str(out),img)
      rows.append((label,out,title,url))
    except Exception as e:
      print("canonical failed",label,e)
  return rows


def harvest_real():
  manifest=[]
  stats=Counter()
  for label,roots in REAL_CATEGORIES.items():
    titles=[]
    for root in roots:
      try:
        titles.extend(category_members(root,recurse=2))
      except Exception as e:
        print("category failed",root,e)
    titles=list(dict.fromkeys(titles))
    random.Random(SEED+LABEL_TO_IDX[label]).shuffle(titles)
    target=MAX_REAL[label]
    print(f"{label}: discovered {len(titles)} files, target {target}")
    accepted=0
    for n,title in enumerate(titles):
      if accepted>=target: break
      try:
        url=image_url(title,1600)
        img=download_image(url)
        if img is None: continue
        if label=="NSL": crop=nsl_crop(img)
        elif label=="OTHER": crop=other_crop(img)
        else: crop=numeric_roundel_crop(img)
        out=save_crop(label,accepted,crop,"real")
        if out is None: continue
        manifest.append({
          "label":label,"path":str(out),"source":"real","source_title":title,"source_url":url
        })
        accepted+=1
      except Exception as e:
        print("image failed",label,title,e)
    stats[label]=accepted
    print(f"{label}: accepted {accepted}")
  return manifest,stats


def augment_np(img):
  # img BGR square-ish sign crop.
  h,w=img.shape[:2]
  # Random border/padding reproduces loose detector boxes/backing boards.
  if random.random()<0.55:
    frac=random.uniform(.04,.28)
    px=max(1,int(w*frac)); py=max(1,int(h*frac))
    if random.random()<.25:
      border=(random.randint(120,230),random.randint(160,245),random.randint(180,255)) # yellowish BGR
    else:
      base=random.randint(0,80); border=(base,base,base)
    img=cv2.copyMakeBorder(img,py,py,px,px,cv2.BORDER_CONSTANT,value=border)

  img=cv2.resize(img,(144,144),interpolation=cv2.INTER_CUBIC)
  # Perspective/rotation
  if random.random()<.8:
    ang=random.uniform(-12,12)
    sc=random.uniform(.78,1.08)
    M=cv2.getRotationMatrix2D((72,72),ang,sc)
    img=cv2.warpAffine(img,M,(144,144),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
  if random.random()<.5:
    jitter=12
    src=np.float32([[0,0],[143,0],[143,143],[0,143]])
    dst=src+np.random.uniform(-jitter,jitter,(4,2)).astype(np.float32)
    P=cv2.getPerspectiveTransform(src,dst)
    img=cv2.warpPerspective(img,P,(144,144),borderMode=cv2.BORDER_REPLICATE)
  # Camera degradation
  if random.random()<.65:
    gamma=random.uniform(.45,1.45)
    lut=np.clip(((np.arange(256)/255.0)**gamma)*255,0,255).astype(np.uint8)
    img=cv2.LUT(img,lut)
  if random.random()<.45:
    sigma=random.uniform(.4,1.8)
    k=3 if sigma<1 else 5
    img=cv2.GaussianBlur(img,(k,k),sigma)
  if random.random()<.35:
    # emulate tiny distant sign and upsample
    small=random.randint(26,90)
    img=cv2.resize(img,(small,small),interpolation=cv2.INTER_AREA)
    img=cv2.resize(img,(144,144),interpolation=cv2.INTER_LINEAR)
  if random.random()<.25:
    q=random.randint(32,78)
    ok,enc=cv2.imencode(".jpg",img,[cv2.IMWRITE_JPEG_QUALITY,q])
    if ok: img=cv2.imdecode(enc,cv2.IMREAD_COLOR)
  if random.random()<.20:
    # horizontal motion blur
    k=random.choice([3,5,7])
    ker=np.zeros((k,k),np.float32); ker[k//2,:]=1.0/k
    img=cv2.filter2D(img,-1,ker)
  return img


class SignDataset(Dataset):
  def __init__(self, rows, train=False):
    self.rows=rows
    self.train=train
  def __len__(self): return len(self.rows)
  def __getitem__(self,i):
    row=self.rows[i]
    img=cv2.imread(row["path"],cv2.IMREAD_COLOR)
    if img is None: raise RuntimeError(row["path"])
    if self.train: img=augment_np(img)
    else: img=cv2.resize(img,(144,144),interpolation=cv2.INTER_AREA)
    img=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    arr=img.astype(np.float32)/255.0
    arr=(arr-np.array([.485,.456,.406],np.float32))/np.array([.229,.224,.225],np.float32)
    arr=np.transpose(arr,(2,0,1))
    return torch.from_numpy(arr.copy()).float(), LABEL_TO_IDX[row["label"]]


def split_real(rows):
  by=defaultdict(list)
  for r in rows: by[r["label"]].append(r)
  train=[]; val=[]
  for label,items in by.items():
    rng=random.Random(SEED+LABEL_TO_IDX[label]*17)
    rng.shuffle(items)
    if len(items)>=5:
      nval=max(1,int(round(len(items)*.22)))
    elif len(items)>=2:
      nval=1
    else: nval=0
    val.extend(items[:nval]); train.extend(items[nval:])
  return train,val


def add_canonical_train(train_rows, canon_rows, repeats=10):
  for label,path,title,url in canon_rows:
    for i in range(repeats):
      train_rows.append({
        "label":label,"path":str(path),"source":"canonical",
        "source_title":title,"source_url":url,"replica":i,
      })


def model_build():
  weights=MobileNet_V3_Small_Weights.DEFAULT
  model=mobilenet_v3_small(weights=weights)
  model.classifier[3]=nn.Linear(model.classifier[3].in_features,len(CLASSES))
  return model


@torch.no_grad()
def evaluate(model,loader,device):
  model.eval()
  total=0; correct=0
  conf=np.zeros((len(CLASSES),len(CLASSES)),dtype=np.int64)
  rows=[]
  for xb,yb in loader:
    xb=xb.to(device); yb=yb.to(device)
    logits=model(xb); probs=torch.softmax(logits,dim=1)
    pred=probs.argmax(1)
    for y,p,pr in zip(yb.cpu().numpy(),pred.cpu().numpy(),probs.max(1).values.cpu().numpy()):
      conf[int(y),int(p)]+=1
      rows.append((CLASSES[int(y)],CLASSES[int(p)],float(pr)))
    total+=len(yb); correct+=int((pred==yb).sum())
  return correct/max(total,1),conf,rows


def train_model(train_rows,val_rows):
  device=torch.device("cpu")
  ds=SignDataset(train_rows,train=True)
  counts=Counter(r["label"] for r in train_rows)
  weights=[1.0/max(counts[r["label"]],1) for r in train_rows]
  sampler=WeightedRandomSampler(weights,num_samples=max(800,len(train_rows)*3),replacement=True)
  loader=DataLoader(ds,batch_size=48,sampler=sampler,num_workers=2)
  val_loader=DataLoader(SignDataset(val_rows,train=False),batch_size=64,shuffle=False,num_workers=2)

  model=model_build().to(device)
  # Fine-tune the full compact network: digit differences are subtle.
  opt=torch.optim.AdamW(model.parameters(),lr=3e-4,weight_decay=1e-4)
  criterion=nn.CrossEntropyLoss()
  history=[]
  best_state=None; best_acc=-1.0
  for epoch in range(14):
    model.train()
    loss_sum=0.0; n=0
    for xb,yb in loader:
      xb=xb.to(device); yb=yb.to(device)
      opt.zero_grad(set_to_none=True)
      logits=model(xb)
      loss=criterion(logits,yb)
      loss.backward()
      opt.step()
      loss_sum+=float(loss.item())*len(yb); n+=len(yb)
    acc,conf,_=evaluate(model,val_loader,device) if val_rows else (0.0,None,None)
    item={"epoch":epoch+1,"loss":loss_sum/max(n,1),"val_accuracy":acc}
    history.append(item); print(item)
    if acc>best_acc:
      best_acc=acc
      best_state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
  if best_state is not None: model.load_state_dict(best_state)
  return model,history


def export_onnx(model):
  model.eval()
  out=ROOT/"uk_speed_classifier_v2_poc.onnx"
  dummy=torch.zeros(1,3,144,144)
  torch.onnx.export(
    model,dummy,str(out),input_names=["image"],output_names=["logits"],
    dynamic_axes={"image":{0:"batch"},"logits":{0:"batch"}},
    opset_version=17,dynamo=False,
  )
  return out


def main():
  real_rows,harvest_stats=harvest_real()
  canon_rows=get_canonical()
  train_rows,val_rows=split_real(real_rows)
  add_canonical_train(train_rows,canon_rows,repeats=14)

  # Ensure every class exists in training even if a Commons category was sparse.
  for label in CLASSES:
    if not any(r["label"]==label for r in train_rows):
      found=[x for x in canon_rows if x[0]==label]
      if found:
        add_canonical_train(train_rows,found,repeats=24)

  with open(ROOT/"dataset_manifest.csv","w",newline="",encoding="utf-8") as f:
    fieldnames=["split","label","path","source","source_title","source_url"]
    w=csv.DictWriter(f,fieldnames=fieldnames); w.writeheader()
    for split,rows in (("train",train_rows),("val",val_rows)):
      for r in rows:
        w.writerow({k:(split if k=="split" else r.get(k,"")) for k in fieldnames})

  print("train",Counter(r["label"] for r in train_rows))
  print("val",Counter(r["label"] for r in val_rows))
  if len(val_rows)<10:
    raise RuntimeError(f"Too few real validation crops: {len(val_rows)}")

  model,history=train_model(train_rows,val_rows)
  val_loader=DataLoader(SignDataset(val_rows,train=False),batch_size=64,shuffle=False)
  acc,conf,pred_rows=evaluate(model,val_loader,torch.device("cpu"))

  onnx=export_onnx(model)
  torch.save({"classes":CLASSES,"state_dict":model.state_dict()},ROOT/"uk_speed_classifier_v2_poc.pt")

  with open(ROOT/"validation_predictions.csv","w",newline="",encoding="utf-8") as f:
    w=csv.writer(f); w.writerow(["actual","predicted","confidence"])
    w.writerows(pred_rows)

  report={
    "classes":CLASSES,
    "harvest_accepted":dict(harvest_stats),
    "real_total":len(real_rows),
    "train_total":len(train_rows),
    "validation_total":len(val_rows),
    "validation_accuracy":acc,
    "confusion_matrix":conf.tolist(),
    "history":history,
    "model":"MobileNetV3-Small fine-tuned, 144x144 RGB",
    "onnx":str(onnx),
    "seed":SEED,
  }
  (ROOT/"report.json").write_text(json.dumps(report,indent=2))
  print(json.dumps(report,indent=2))


if __name__=="__main__":
  main()
