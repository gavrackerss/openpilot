from __future__ import annotations

import argparse
import io
import json
import math
import random
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import requests
from PIL import Image, ImageEnhance, ImageFilter, ImageOps
import torch
from torch import nn
from torch.utils.data import Dataset, DataLoader, WeightedRandomSampler
from torchvision import models, transforms

CLASSES = ["20", "30", "40", "50", "60", "70", "NSL", "OTHER"]
CLASS_TO_IDX = {c: i for i, c in enumerate(CLASSES)}
COMMONS_API = "https://commons.wikimedia.org/w/api.php"
USER_AGENT = "OpenPilot-UK-VSL-V2-Research/0.1 (non-commercial research)"

CATEGORY_MAP = {
    "20": ["20 mph speed limit road signs in the United Kingdom"],
    "30": ["30 mph speed limit road signs in the United Kingdom"],
    "40": ["40 mph speed limit road signs in the United Kingdom"],
    "50": ["50 mph speed limit road signs in the United Kingdom"],
    "60": ["60 mph speed limit road signs in the United Kingdom"],
    "70": ["70 mph speed limit road signs in the United Kingdom"],
    "NSL": ["Speed limit de-restriction road signs in the United Kingdom"],
    "OTHER": [
        "Minimum speed limit road signs in the United Kingdom",
        "No entry road signs in the United Kingdom",
        "Mandatory road signs in the United Kingdom",
    ],
}

# These Commons SVGs are redraws of the official DfT/TSRGD artwork.
CLEAN_FILES = {
    "20": "UK traffic sign 670V20.svg",
    "30": "UK traffic sign 670V30.svg",
    "40": "UK traffic sign 670V40.svg",
    "50": "UK traffic sign 670V50.svg",
    "60": "UK traffic sign 670V60.svg",
    "70": "UK traffic sign 670V70.svg",
    "NSL": "UK traffic sign 671.svg",
}


def request_json(session: requests.Session, params: dict, attempts: int = 3):
    last = None
    for i in range(attempts):
        try:
            r = session.get(COMMONS_API, params=params, timeout=30)
            r.raise_for_status()
            return r.json()
        except Exception as e:
            last = e
            time.sleep(1.5 * (i + 1))
    raise RuntimeError(f"Commons API failed: {last}")


def commons_category_files(session, category: str, max_files: int = 45, max_depth: int = 2):
    out = []
    seen_cat = set()
    queue = [(category, 0)]
    while queue and len(out) < max_files:
        cat, depth = queue.pop(0)
        if cat in seen_cat:
            continue
        seen_cat.add(cat)
        cont = None
        while len(out) < max_files:
            params = {
                "action": "query", "format": "json", "list": "categorymembers",
                "cmtitle": "Category:" + cat, "cmlimit": "500", "cmtype": "file|subcat",
            }
            if cont:
                params["cmcontinue"] = cont
            data = request_json(session, params)
            members = data.get("query", {}).get("categorymembers", [])
            for m in members:
                title = m.get("title", "")
                ns = int(m.get("ns", -1))
                if ns == 6 and title.startswith("File:"):
                    out.append(title)
                    if len(out) >= max_files:
                        break
                elif ns == 14 and depth < max_depth and title.startswith("Category:"):
                    queue.append((title[len("Category:"):], depth + 1))
            cont = data.get("continue", {}).get("cmcontinue")
            if not cont:
                break
    return list(dict.fromkeys(out))[:max_files]


def commons_thumb_url(session, file_title: str, width: int = 1280):
    data = request_json(session, {
        "action": "query", "format": "json", "titles": file_title,
        "prop": "imageinfo", "iiprop": "url|mime", "iiurlwidth": str(width),
    })
    pages = data.get("query", {}).get("pages", {})
    for page in pages.values():
        ii = page.get("imageinfo", [])
        if not ii:
            continue
        info = ii[0]
        return info.get("thumburl") or info.get("url"), info.get("mime", "")
    return None, ""


def download_image(session, url: str):
    try:
        r = session.get(url, timeout=35)
        r.raise_for_status()
        im = Image.open(io.BytesIO(r.content)).convert("RGB")
        if min(im.size) < 64:
            return None
        return im
    except Exception:
        return None


def red_roundel_crop(im: Image.Image):
    bgr = cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)
    h, w = bgr.shape[:2]
    scale = min(1.0, 1400.0 / max(w, h))
    if scale < 1.0:
        bgr_small = cv2.resize(bgr, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    else:
        bgr_small = bgr
    hs, ws = bgr_small.shape[:2]
    hsv = cv2.cvtColor(bgr_small, cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    red = ((((hue <= 12) | (hue >= 168)) & (sat >= 70) & (val >= 45))).astype(np.uint8) * 255
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_score = 0.0
    img_area = float(max(ws * hs, 1))
    for c in contours:
        x, y, cw, ch = cv2.boundingRect(c)
        if cw < 10 or ch < 10:
            continue
        ar = cw / max(ch, 1)
        if ar < 0.55 or ar > 1.8:
            continue
        box_ratio = cw * ch / img_area
        if box_ratio < 0.00008 or box_ratio > 0.55:
            continue
        roi = bgr_small[max(0,y):min(hs,y+ch), max(0,x):min(ws,x+cw)]
        if roi.size == 0:
            continue
        rhsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        rs, rv = rhsv[:, :, 1], rhsv[:, :, 2]
        white = ((rv >= 105) & (rs <= 120)).mean()
        dark = ((rv <= 135) & (rs <= 165)).mean()
        squareness = min(ar, 1/ar)
        score = math.sqrt(cw*ch) * (
            0.55 + 0.25*squareness +
            0.12*min(float(white)/0.25,1) +
            0.08*min(float(dark)/0.08,1)
        )
        if score > best_score:
            best_score, best = score, (x,y,cw,ch)
    if best is None:
        return None
    x,y,cw,ch = best
    pad = int(max(cw,ch) * 0.20)
    x1=max(x-pad,0); y1=max(y-pad,0); x2=min(x+cw+pad,ws); y2=min(y+ch+pad,hs)
    crop = bgr_small[y1:y2,x1:x2]
    if crop.size == 0:
        return None
    return Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))


def nsl_crop(im: Image.Image):
    bgr = cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)
    h,w = bgr.shape[:2]
    scale=min(1.0, 1400/max(h,w))
    if scale < 1:
        bgr=cv2.resize(bgr,(int(w*scale),int(h*scale)),interpolation=cv2.INTER_AREA)
    hs,ws=bgr.shape[:2]
    gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
    gray=cv2.GaussianBlur(gray,(5,5),1.2)
    circles=cv2.HoughCircles(
        gray,cv2.HOUGH_GRADIENT,dp=1.25,minDist=24,param1=120,param2=28,
        minRadius=8,maxRadius=max(12,int(min(hs,ws)*0.18))
    )
    if circles is None:
        return None
    best=None; best_score=-1
    hsv=cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    sat=hsv[:,:,1]; val=hsv[:,:,2]
    for cx,cy,r in np.round(circles[0]).astype(int)[:60]:
        if r < 8:
            continue
        x1=max(cx-r,0); y1=max(cy-r,0); x2=min(cx+r,ws); y2=min(cy+r,hs)
        roi=gray[y1:y2,x1:x2]
        if roi.size==0:
            continue
        rh,rw=roi.shape
        yy,xx=np.mgrid[0:rh,0:rw]
        nx=(xx-(rw-1)/2)/max(rw/2,1); ny=(yy-(rh-1)/2)/max(rh/2,1)
        rr=np.sqrt(nx*nx+ny*ny)
        face=rr<0.88; core=rr<0.65
        v=val[y1:y2,x1:x2]; s=sat[y1:y2,x1:x2]
        white=((v>120)&(s<100)); dark=((v<130)&(s<170))
        white_face=float(white[face].mean()) if face.any() else 0
        dark_core=float(dark[core].mean()) if core.any() else 0
        score=white_face*0.65+min(dark_core/0.22,1)*0.35 + min(r/100,1)*0.1
        if score>best_score and white_face>0.28 and dark_core>0.05:
            best_score=score; best=(cx,cy,r)
    if best is None:
        return None
    cx,cy,r=best; p=int(r*1.20)
    x1=max(cx-p,0); y1=max(cy-p,0); x2=min(cx+p,ws); y2=min(cy+p,hs)
    crop=bgr[y1:y2,x1:x2]
    return Image.fromarray(cv2.cvtColor(crop,cv2.COLOR_BGR2RGB)) if crop.size else None


def centre_square(im: Image.Image):
    w,h=im.size
    s=min(w,h)
    x=(w-s)//2; y=(h-s)//2
    return im.crop((x,y,x+s,y+s))


def fetch_clean_art(session, out_dir: Path):
    out={}
    for label, filename in CLEAN_FILES.items():
        url,_mime=commons_thumb_url(session,"File:"+filename,768)
        if not url:
            continue
        im=download_image(session,url)
        if im is None:
            continue
        im=centre_square(im)
        p=out_dir/f"clean_{label}.png"
        im.save(p)
        out[label]=p
    return out


def fetch_real_crops(session, out_root: Path, per_class=48):
    manifest=[]
    for label in CLASSES:
        wanted=per_class if label not in ("60","70") else max(per_class,65)
        titles=[]
        for cat in CATEGORY_MAP[label]:
            try:
                titles += commons_category_files(session,cat,max_files=wanted,max_depth=3)
            except Exception as e:
                print(f"WARN category {cat}: {e}")
        titles=list(dict.fromkeys(titles))[:wanted]
        d=out_root/label
        d.mkdir(parents=True,exist_ok=True)
        for idx,title in enumerate(titles):
            url,mime=commons_thumb_url(session,title,1280)
            if not url or (mime and "image" not in mime):
                continue
            im=download_image(session,url)
            if im is None:
                continue
            if label=="NSL":
                crop=nsl_crop(im)
            elif label=="OTHER":
                crop=red_roundel_crop(im)
                if crop is None:
                    crop=nsl_crop(im)
            else:
                crop=red_roundel_crop(im)
            if crop is None or min(crop.size)<24:
                continue
            p=d/f"real_{idx:03d}.jpg"
            crop.save(p,quality=94)
            manifest.append({
                "label":label,"path":str(p),
                "source_title":title,"source_url":url
            })
        print(label,"downloaded/cropped",
              sum(1 for m in manifest if m["label"]==label),
              "from",len(titles),"titles")
    return manifest


class CommaAugment:
    def __init__(self, size=128):
        self.size=size

    def __call__(self, im: Image.Image):
        im=im.convert("RGB")
        if random.random()<0.75:
            im=im.rotate(
                random.uniform(-11,11),
                resample=Image.Resampling.BICUBIC,
                expand=False,
                fillcolor=(110,110,110)
            )
        if random.random()<0.7:
            pad=random.randint(2,18)
            bg=random.choice([(45,45,45),(90,90,90),(125,125,125),(180,180,180)])
            im=ImageOps.expand(im,border=pad,fill=bg)
        if random.random()<0.85:
            side=random.randint(22,105)
            im=im.resize((side,side),Image.Resampling.LANCZOS)
            im=im.resize((self.size,self.size),Image.Resampling.BICUBIC)
        else:
            im=ImageOps.fit(im,(self.size,self.size),method=Image.Resampling.LANCZOS)
        if random.random()<0.65:
            im=ImageEnhance.Brightness(im).enhance(random.uniform(0.35,1.45))
        if random.random()<0.6:
            im=ImageEnhance.Contrast(im).enhance(random.uniform(0.65,1.55))
        if random.random()<0.45:
            im=ImageEnhance.Color(im).enhance(random.uniform(0.45,1.35))
        if random.random()<0.55:
            im=im.filter(ImageFilter.GaussianBlur(radius=random.uniform(0.2,1.8)))
        if random.random()<0.55:
            buf=io.BytesIO()
            im.save(buf,format="JPEG",quality=random.randint(25,78))
            buf.seek(0)
            im=Image.open(buf).convert("RGB")
        return im


class SignDataset(Dataset):
    def __init__(self, items, train=False, repeats=1, size=128):
        self.items=items*repeats
        self.train=train
        self.aug=CommaAugment(size)
        self.size=size
        self.final=transforms.Compose([
            transforms.Resize((size,size)),
            transforms.ToTensor(),
            transforms.Normalize([0.485,0.456,0.406],[0.229,0.224,0.225]),
        ])

    def __len__(self):
        return len(self.items)

    def __getitem__(self,i):
        p,label=self.items[i]
        im=Image.open(p).convert("RGB")
        if self.train:
            im=self.aug(im)
        else:
            im=ImageOps.fit(im,(self.size,self.size),method=Image.Resampling.LANCZOS)
        return self.final(im), CLASS_TO_IDX[label], str(p)


def evaluate(model, loader, device):
    model.eval()
    total=0; correct=0; rows=[]
    cm=np.zeros((len(CLASSES),len(CLASSES)),dtype=np.int64)
    with torch.no_grad():
        for x,y,paths in loader:
            x=x.to(device)
            probs=model(x).softmax(1).cpu().numpy()
            pred=probs.argmax(1)
            yt=y.numpy()
            total+=len(yt)
            correct+=int((pred==yt).sum())
            for a,b,p,pr in zip(yt,pred,paths,probs):
                cm[a,b]+=1
                rows.append({
                    "path":p,"actual":CLASSES[a],"pred":CLASSES[b],
                    "confidence":float(pr[b]),
                    "probs":{c:float(pr[j]) for j,c in enumerate(CLASSES)}
                })
    return correct/max(total,1), cm, rows


def make_generated_other(out_dir: Path):
    out_dir.mkdir(exist_ok=True)
    items=[]
    for i in range(12):
        arr=np.full((512,512,3),245,dtype=np.uint8)
        if i%3==0:
            cv2.circle(arr,(256,256),200,(210,20,20),55)
            cv2.rectangle(arr,(100,225),(412,287),(245,245,245),-1)
        elif i%3==1:
            cv2.circle(arr,(256,256),205,(25,95,190),-1)
            cv2.arrowedLine(arr,(120,256),(390,256),(245,245,245),55,tipLength=0.25)
        else:
            cv2.circle(arr,(256,256),205,(210,20,20),55)
            cv2.line(arr,(145,145),(365,365),(210,20,20),55)
        p=out_dir/f"other_{i:02d}.jpg"
        Image.fromarray(arr).save(p,quality=95)
        items.append((str(p),"OTHER"))
    return items


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--output",default="v2_out")
    ap.add_argument("--epochs",type=int,default=8)
    args=ap.parse_args()

    out=Path(args.output)
    out.mkdir(parents=True,exist_ok=True)
    random.seed(20261009)
    np.random.seed(20261009)
    torch.manual_seed(20261009)

    s=requests.Session()
    s.headers.update({"User-Agent":USER_AGENT})

    source=out/"sources"
    clean_dir=source/"clean"
    real_dir=source/"real"
    clean_dir.mkdir(parents=True,exist_ok=True)
    real_dir.mkdir(parents=True,exist_ok=True)

    clean=fetch_clean_art(s,clean_dir)
    manifest=fetch_real_crops(s,real_dir,per_class=48)
    with open(out/"source_manifest.json","w") as f:
        json.dump(manifest,f,indent=2)

    by=defaultdict(list)
    for m in manifest:
        by[m["label"]].append((m["path"],m["label"]))

    train=[]; val=[]; split_info={}
    for label in CLASSES:
        items=by[label][:]
        random.shuffle(items)
        nval=max(1,int(round(len(items)*0.25))) if len(items)>=4 else max(0,len(items)//3)
        val_part=items[:nval]
        train_part=items[nval:]
        if label in clean:
            train_part.append((str(clean[label]),label))
        train += train_part
        val += val_part
        split_info[label]={
            "real_total":len(items),
            "train_base":len(train_part),
            "val_real":len(val_part)
        }

    train += make_generated_other(clean_dir/"other_generated")
    with open(out/"split_info.json","w") as f:
        json.dump(split_info,f,indent=2)
    print("SPLIT",json.dumps(split_info,indent=2))

    if len(train)<20 or len(val)<5:
        raise RuntimeError(f"Insufficient data: train={len(train)} val={len(val)}")

    train_ds=SignDataset(train,train=True,repeats=12,size=128)
    val_ds=SignDataset(val,train=False,repeats=1,size=128)

    counts=Counter(label for _p,label in train_ds.items)
    sample_weights=[1.0/max(counts[label],1) for _p,label in train_ds.items]
    sampler=WeightedRandomSampler(
        sample_weights,
        num_samples=min(max(len(train_ds),1800),5200),
        replacement=True
    )
    train_loader=DataLoader(train_ds,batch_size=64,sampler=sampler,num_workers=2)
    val_loader=DataLoader(val_ds,batch_size=64,shuffle=False,num_workers=2)

    device=torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("DEVICE",device)

    weights=models.MobileNet_V3_Small_Weights.IMAGENET1K_V1
    model=models.mobilenet_v3_small(weights=weights)
    model.classifier[3]=nn.Linear(model.classifier[3].in_features,len(CLASSES))
    model.to(device)

    criterion=nn.CrossEntropyLoss(label_smoothing=0.04)
    opt=torch.optim.AdamW(model.parameters(),lr=2.5e-4,weight_decay=1e-4)
    sched=torch.optim.lr_scheduler.CosineAnnealingLR(opt,T_max=max(args.epochs,1))

    history=[]; best_acc=-1; best_state=None
    for epoch in range(args.epochs):
        model.train()
        running=0; seen=0
        for x,y,_paths in train_loader:
            x=x.to(device); y=y.to(device)
            opt.zero_grad(set_to_none=True)
            logits=model(x)
            loss=criterion(logits,y)
            loss.backward()
            opt.step()
            running += float(loss.item())*len(y)
            seen += len(y)
        sched.step()
        acc,_cm,_rows=evaluate(model,val_loader,device)
        rec={
            "epoch":epoch+1,
            "loss":running/max(seen,1),
            "val_accuracy":acc,
            "lr":opt.param_groups[0]["lr"]
        }
        history.append(rec)
        print("EPOCH",rec)
        if acc>best_acc:
            best_acc=acc
            best_state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}

    model.load_state_dict(best_state)
    model.to(device)
    val_acc,cm,val_rows=evaluate(model,val_loader,device)

    torch.save(
        {"state_dict":model.state_dict(),"classes":CLASSES,"input_size":128},
        out/"uk_speed_classifier_v2_poc.pt"
    )

    model.eval()
    dummy=torch.randn(1,3,128,128,device=device)
    torch.onnx.export(
        model,dummy,out/"uk_speed_classifier_v2_poc.onnx",
        input_names=["image"],output_names=["logits"],opset_version=17,
        dynamic_axes={"image":{0:"batch"},"logits":{0:"batch"}}
    )

    with open(out/"validation_predictions.json","w") as f:
        json.dump(val_rows,f,indent=2)

    report={
        "classes":CLASSES,
        "best_real_photo_val_accuracy":val_acc,
        "confusion_matrix":cm.tolist(),
        "split_info":split_info,
        "history":history,
        "training_source":"DfT/TSRGD-derived Wikimedia clean artwork + labelled real UK Wikimedia Commons photographs + synthetic camera degradation",
        "comma_test_in_training":False
    }
    with open(out/"report.json","w") as f:
        json.dump(report,f,indent=2)

    print("FINAL_REPORT")
    print(json.dumps(report,indent=2))


if __name__=="__main__":
    main()
