from __future__ import annotations

from dataclasses import dataclass

import numpy as np


SUPPORTED_UK_LIMITS_MPH = (20, 30, 40, 50, 60, 70)
FIRST_DIGITS = tuple(str(v // 10) for v in SUPPORTED_UK_LIMITS_MPH)


@dataclass(frozen=True)
class UKSpeedRead:
  speed_limit_mph: int
  confidence: float
  first_digit_score: float
  zero_score: float
  first_digit_margin: float
  whole_value_score: float
  method: str = ""


class UKSpeedValueReader:
  """Read 20/30/40/50/60/70 from a cropped UK red-circle speed sign.

  The legacy ONNX network is used only to propose a sign box. This reader
  independently verifies the two digits inside the sign. UK values handled by
  V1.1 all end in zero, which gives us a useful second-digit sanity check.
  """

  DIGIT_MIN_SCORE = 0.30
  ZERO_MIN_SCORE = 0.32
  DIGIT_MIN_MARGIN = 0.025
  WHOLE_MIN_SCORE = 0.29
  STRONG_DIGIT_SCORE = 0.43
  STRONG_DIGIT_MARGIN = 0.055

  def __init__(self, cv2):
    self.cv2 = cv2
    self.digit_templates = self._build_digit_templates()
    self.value_templates = self._build_value_templates()
    self.last_reject_reason = "not_read"
    self.last_debug = ""

  @staticmethod
  def _shift_mask(mask: np.ndarray, dx: int, dy: int) -> np.ndarray:
    out = np.zeros_like(mask)
    h, w = mask.shape
    sx1, sx2 = max(-dx, 0), min(w - dx, w)
    sy1, sy2 = max(-dy, 0), min(h - dy, h)
    dx1, dy1 = max(dx, 0), max(dy, 0)
    dx2 = dx1 + max(sx2 - sx1, 0)
    dy2 = dy1 + max(sy2 - sy1, 0)
    if sx2 > sx1 and sy2 > sy1:
      out[dy1:dy2, dx1:dx2] = mask[sy1:sy2, sx1:sx2]
    return out

  @classmethod
  def _mask_similarity(cls, candidate: np.ndarray, template: np.ndarray) -> float:
    cand = candidate > 0
    tmpl = template > 0
    best = 0.0
    for dy in (-2, 0, 2):
      for dx in (-2, 0, 2):
        shifted = cls._shift_mask(tmpl, dx, dy) > 0
        intersection = float(np.logical_and(cand, shifted).sum())
        total = float(cand.sum() + shifted.sum())
        if total <= 0.0:
          continue
        dice = 2.0 * intersection / total
        union = float(np.logical_or(cand, shifted).sum())
        iou = intersection / union if union > 0.0 else 0.0
        best = max(best, 0.70 * dice + 0.30 * iou)
    return float(best)

  def _normalize_mask(self, binary: np.ndarray, size: tuple[int, int], padding: int = 5):
    points = self.cv2.findNonZero(binary)
    if points is None:
      return None
    x, y, w, h = self.cv2.boundingRect(points)
    if w <= 0 or h <= 0:
      return None
    src = binary[y:y + h, x:x + w]
    target_w, target_h = size
    scale = min((target_w - 2 * padding) / max(w, 1), (target_h - 2 * padding) / max(h, 1))
    if scale <= 0.0:
      return None
    rw = max(int(round(w * scale)), 1)
    rh = max(int(round(h * scale)), 1)
    resized = self.cv2.resize(src, (rw, rh), interpolation=self.cv2.INTER_NEAREST)
    canvas = np.zeros((target_h, target_w), dtype=np.uint8)
    ox, oy = (target_w - rw) // 2, (target_h - rh) // 2
    canvas[oy:oy + rh, ox:ox + rw] = resized
    return canvas

  def _render_text_mask(self, text: str, font: int, scale: float, thickness: int,
                        size: tuple[int, int]):
    canvas = np.full((150, 180), 255, dtype=np.uint8)
    ts, baseline = self.cv2.getTextSize(text, font, scale, thickness)
    x = max((canvas.shape[1] - ts[0]) // 2, 0)
    y = max((canvas.shape[0] + ts[1]) // 2 - baseline, ts[1])
    self.cv2.putText(canvas, text, (x, y), font, scale, 0, thickness, self.cv2.LINE_AA)
    _, binary = self.cv2.threshold(canvas, 0, 255, self.cv2.THRESH_BINARY_INV + self.cv2.THRESH_OTSU)
    return self._normalize_mask(binary, size)

  def _build_digit_templates(self):
    templates: dict[str, list[np.ndarray]] = {d: [] for d in (*FIRST_DIGITS, "0")}
    fonts = (
      self.cv2.FONT_HERSHEY_SIMPLEX,
      self.cv2.FONT_HERSHEY_DUPLEX,
      self.cv2.FONT_HERSHEY_COMPLEX,
      self.cv2.FONT_HERSHEY_TRIPLEX,
    )
    for digit in templates:
      for font in fonts:
        for scale, thickness in ((1.5, 2), (1.7, 3), (1.9, 3), (2.1, 4), (2.3, 4)):
          normalized = self._render_text_mask(digit, font, scale, thickness, (48, 72))
          if normalized is not None:
            templates[digit].append(normalized)
    return templates

  def _build_value_templates(self):
    templates: dict[int, list[np.ndarray]] = {v: [] for v in SUPPORTED_UK_LIMITS_MPH}
    fonts = (
      self.cv2.FONT_HERSHEY_SIMPLEX,
      self.cv2.FONT_HERSHEY_DUPLEX,
      self.cv2.FONT_HERSHEY_COMPLEX,
      self.cv2.FONT_HERSHEY_TRIPLEX,
    )
    for value in templates:
      for font in fonts:
        for scale, thickness in ((1.3, 2), (1.5, 2), (1.7, 3), (1.9, 3), (2.1, 4)):
          normalized = self._render_text_mask(str(value), font, scale, thickness, (84, 72))
          if normalized is not None:
            templates[value].append(normalized)
    return templates

  def _best_template(self, mask: np.ndarray, templates: dict):
    scores = {}
    for label, label_templates in templates.items():
      best = 0.0
      for template in label_templates:
        best = max(best, self._mask_similarity(mask, template))
      scores[label] = float(best)
    ordered = sorted(scores.items(), key=lambda item: item[1], reverse=True)
    if not ordered:
      return None, 0.0, 0.0
    best_label, best_score = ordered[0]
    runner_up = ordered[1][1] if len(ordered) > 1 else 0.0
    return best_label, float(best_score), float(best_score - runner_up)

  def _find_ring_bbox(self, sign_crop: np.ndarray):
    """Locate the dominant red circular sign inside a possibly larger board.

    Bounding all red pixels together is fragile when a detector crop includes
    a yellow backing panel, other road furniture, or small red artefacts. Use
    the largest plausible near-square red component instead.
    """
    hsv = self.cv2.cvtColor(sign_crop, self.cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    red = (
      (((hue <= 12) | (hue >= 168))) &
      (sat >= 65) &
      (val >= 50)
    ).astype(np.uint8) * 255
    red = self.cv2.morphologyEx(
      red, self.cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8)
    )

    contours, _hier = self.cv2.findContours(
      red, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE
    )
    crop_h, crop_w = sign_crop.shape[:2]
    crop_area = float(max(crop_w * crop_h, 1))
    best = None
    best_score = 0.0
    for contour in contours:
      x, y, w, h = self.cv2.boundingRect(contour)
      if w < 8 or h < 8:
        continue
      aspect = w / max(h, 1)
      if aspect < 0.55 or aspect > 1.65:
        continue
      if float(w * h) / crop_area < 0.025:
        continue
      squareness = min(aspect, 1.0 / max(aspect, 1e-6))
      score = float(w * h) * (0.75 + 0.25 * squareness)
      if score > best_score:
        best_score = score
        best = (x, y, w, h)

    return best

  def _digit_roi_variants(self, sign_crop: np.ndarray):
    """Return normalized centre regions likely to contain the two digits.

    The first variant preserves the original V1.x crop. Additional variants
    normalize the detected red-ring bbox to a square before extracting slightly
    different inner regions. This compensates for detector padding, yellow
    backing boards, perspective and small off-centre crops without weakening
    the digit acceptance thresholds.
    """
    if sign_crop is None or sign_crop.size == 0:
      return []

    h, w = sign_crop.shape[:2]
    variants = []

    ring = self._find_ring_bbox(sign_crop)
    if ring is not None:
      rx, ry, rw, rh = ring

      # Preserve the established direct crop first.
      x1 = max(rx + int(rw * 0.17), 0)
      x2 = min(rx + int(rw * 0.83), w)
      y1 = max(ry + int(rh * 0.20), 0)
      y2 = min(ry + int(rh * 0.82), h)
      if x2 > x1 and y2 > y1:
        variants.append(("direct", sign_crop[y1:y2, x1:x2]))

      # Normalize the actual circular sign, not any larger detector/backing box.
      px = max(int(round(rw * 0.07)), 2)
      py = max(int(round(rh * 0.07)), 2)
      sx1, sy1 = max(rx-px, 0), max(ry-py, 0)
      sx2, sy2 = min(rx+rw+px, w), min(ry+rh+py, h)
      ring_crop = sign_crop[sy1:sy2, sx1:sx2]
      if ring_crop.size > 0:
        square = self.cv2.resize(ring_crop, (144, 144), interpolation=self.cv2.INTER_CUBIC)
        for name, fx1, fy1, fx2, fy2 in (
          ("ring_mid", 0.14, 0.16, 0.86, 0.84),
          ("ring_wide", 0.10, 0.12, 0.90, 0.88),
          ("ring_tight", 0.18, 0.18, 0.82, 0.82),
        ):
          ix1, iy1 = int(144*fx1), int(144*fy1)
          ix2, iy2 = int(144*fx2), int(144*fy2)
          variants.append((name, square[iy1:iy2, ix1:ix2]))
    else:
      x1, x2 = int(w * 0.17), int(w * 0.83)
      y1, y2 = int(h * 0.20), int(h * 0.82)
      if x2 > x1 and y2 > y1:
        variants.append(("fallback", sign_crop[y1:y2, x1:x2]))

    return [(name, roi) for name, roi in variants if roi is not None and roi.size > 0]

  def _binary_variants(self, roi: np.ndarray):
    gray = self.cv2.cvtColor(roi, self.cv2.COLOR_BGR2GRAY)
    min_dim = max(min(gray.shape), 1)
    scale = float(np.clip(150.0 / min_dim, 2.0, 6.0))
    gray = self.cv2.resize(gray, None, fx=scale, fy=scale, interpolation=self.cv2.INTER_CUBIC)
    clahe = self.cv2.createCLAHE(clipLimit=2.5, tileGridSize=(4, 4)).apply(gray)
    blur = self.cv2.GaussianBlur(clahe, (3, 3), 0)

    _thr, otsu = self.cv2.threshold(
      blur, 0, 255, self.cv2.THRESH_BINARY_INV + self.cv2.THRESH_OTSU
    )
    kernel = np.ones((2, 2), dtype=np.uint8)
    variants = [
      ("otsu_open", self.cv2.morphologyEx(otsu, self.cv2.MORPH_OPEN, kernel)),
      ("otsu_raw", otsu),
      ("otsu_close", self.cv2.morphologyEx(otsu, self.cv2.MORPH_CLOSE, kernel)),
    ]

    # Adaptive threshold is useful when half of a small sign is shaded or
    # motion/compression makes one digit noticeably lighter than the other.
    block = max(15, (min(blur.shape) // 5) | 1)
    block = min(block, 51)
    if block % 2 == 0:
      block += 1
    adaptive = self.cv2.adaptiveThreshold(
      clahe, 255, self.cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
      self.cv2.THRESH_BINARY_INV, block, 7,
    )
    variants.append(("adaptive", self.cv2.morphologyEx(adaptive, self.cv2.MORPH_OPEN, kernel)))
    return variants

  def _component_digit_masks(self, binary: np.ndarray):
    count, labels, stats, _ = self.cv2.connectedComponentsWithStats(binary, 8)
    roi_area = binary.shape[0] * binary.shape[1]
    components = []
    for idx in range(1, count):
      x, y, cw, ch, area = [int(v) for v in stats[idx]]
      if area < roi_area * 0.006:
        continue
      if ch < binary.shape[0] * 0.25:
        continue
      if cw < binary.shape[1] * 0.025 or cw > binary.shape[1] * 0.58:
        continue
      if y > binary.shape[0] * 0.70:
        continue
      components.append((x, y, cw, ch, area, idx))

    components.sort(key=lambda item: item[4] * item[3], reverse=True)
    components = components[:3]

    # Test all two-component combinations instead of assuming the two largest
    # blobs are always the two complete digits.
    candidates = []
    for ai in range(len(components)):
      for bi in range(ai + 1, len(components)):
        pair = [components[ai], components[bi]]
        pair.sort(key=lambda item: item[0])
        a, b = pair
        if a[0] + a[2] > b[0] + int(min(a[2], b[2]) * 0.35):
          continue
        height_ratio = min(a[3], b[3]) / max(a[3], b[3], 1)
        if height_ratio < 0.48:
          continue

        masks = []
        combined = np.zeros_like(binary)
        valid = True
        for x, y, cw, ch, _area, idx in pair:
          component = np.zeros_like(binary)
          component[labels == idx] = 255
          combined[labels == idx] = 255
          component = component[y:y + ch, x:x + cw]
          normalized = self._normalize_mask(component, (48, 72))
          if normalized is None:
            valid = False
            break
          masks.append(normalized)
        if not valid:
          continue
        whole = self._normalize_mask(combined, (84, 72))
        if whole is not None:
          candidates.append(("components", whole, masks))

    return candidates

  def _projection_digit_masks(self, binary: np.ndarray):
    """Fallback segmentation when compression fragments/merges digit blobs."""
    points = self.cv2.findNonZero(binary)
    if points is None:
      return []

    x, y, w, h = self.cv2.boundingRect(points)
    if w < 6 or h < 8:
      return []
    src = binary[y:y+h, x:x+w]

    # Find a low-ink split near the middle of the two-digit value.
    projection = (src > 0).sum(axis=0).astype(np.float32)
    if projection.size < 6:
      return []
    lo = max(int(round(projection.size * 0.30)), 1)
    hi = min(int(round(projection.size * 0.70)), projection.size - 1)
    if hi <= lo:
      return []
    split = int(lo + np.argmin(projection[lo:hi]))

    # Try the best valley and two nearby cuts because blur may fill the actual
    # inter-digit gap by a pixel or two.
    cuts = []
    for cut in (split, split-2, split+2):
      if 2 <= cut <= src.shape[1]-2 and cut not in cuts:
        cuts.append(cut)

    candidates = []
    for cut in cuts:
      left = src[:, :cut]
      right = src[:, cut:]
      if int((left > 0).sum()) < 8 or int((right > 0).sum()) < 8:
        continue

      lm = self._normalize_mask(left, (48, 72))
      rm = self._normalize_mask(right, (48, 72))
      whole = self._normalize_mask(src, (84, 72))
      if lm is None or rm is None or whole is None:
        continue
      candidates.append(("projection", whole, [lm, rm]))
    return candidates

  def _evaluate_masks(self, whole_mask: np.ndarray, digit_masks: list[np.ndarray], method: str):
    if whole_mask is None or len(digit_masks) != 2:
      return None, "segmentation", 0.0

    first_label, first_score, first_margin = self._best_template(
      digit_masks[0],
      {d: self.digit_templates[d] for d in FIRST_DIGITS},
    )
    zero_label, zero_score, _ = self._best_template(
      digit_masks[1],
      {"0": self.digit_templates["0"]},
    )
    whole_value, whole_score, _whole_margin = self._best_template(
      whole_mask, self.value_templates
    )

    if first_label is None or zero_label != "0":
      quality = max(float(first_score), float(zero_score))
      return None, f"labels:first={first_label},zero={zero_label}", quality

    value = int(first_label) * 10
    if value not in SUPPORTED_UK_LIMITS_MPH:
      return None, f"unsupported:{value}", 0.0
    if first_score < self.DIGIT_MIN_SCORE:
      return None, f"first_score:{first_score:.3f}", float(first_score)
    if zero_score < self.ZERO_MIN_SCORE:
      return None, f"zero_score:{zero_score:.3f}", float(zero_score)
    if first_margin < self.DIGIT_MIN_MARGIN:
      return None, f"margin:{first_margin:.3f}", float(first_score)

    whole_agrees = whole_value == value and whole_score >= self.WHOLE_MIN_SCORE
    strong_digits = (
      first_score >= self.STRONG_DIGIT_SCORE and
      first_margin >= self.STRONG_DIGIT_MARGIN and
      zero_score >= self.STRONG_DIGIT_SCORE
    )
    if not whole_agrees and not strong_digits:
      quality = (
        float(first_score) * 0.45 +
        float(zero_score) * 0.35 +
        float(whole_score if whole_value == value else 0.0) * 0.20
      )
      return None, (
        f"agreement:value={value},whole={whole_value},wholeScore={whole_score:.3f},"
        f"first={first_score:.3f},zero={zero_score:.3f},margin={first_margin:.3f}"
      ), quality

    agreement_bonus = 0.15 if whole_agrees else 0.0
    confidence = float(np.clip(
      0.38 * first_score +
      0.28 * zero_score +
      0.24 * (whole_score if whole_agrees else 0.0) +
      agreement_bonus,
      0.0,
      0.98,
    ))

    return UKSpeedRead(
      speed_limit_mph=value,
      confidence=confidence,
      first_digit_score=float(first_score),
      zero_score=float(zero_score),
      first_digit_margin=float(first_margin),
      whole_value_score=float(whole_score if whole_agrees else 0.0),
      method=method,
    ), "accepted", confidence

  def read(self, sign_crop: np.ndarray) -> UKSpeedRead | None:
    self.last_reject_reason = "no_candidate"
    self.last_debug = ""

    roi_variants = self._digit_roi_variants(sign_crop)
    if not roi_variants:
      self.last_reject_reason = "no_digit_roi"
      return None

    best_read = None
    best_reject_quality = -1.0
    best_reject = "no_segmentation"
    attempted = 0

    for roi_name, roi in roi_variants:
      for binary_name, binary in self._binary_variants(roi):
        segmentations = self._component_digit_masks(binary)

        # Preserve the established connected-component path as the first choice.
        # Only add the projection fallback when needed, or as a second opinion
        # for selecting the strongest valid read.
        segmentations.extend(self._projection_digit_masks(binary))

        for seg_name, whole, masks in segmentations:
          attempted += 1
          method = f"{roi_name}/{binary_name}/{seg_name}"
          result, reason, quality = self._evaluate_masks(whole, masks, method)
          if result is not None:
            if best_read is None or result.confidence > best_read.confidence:
              best_read = result
          elif quality > best_reject_quality:
            best_reject_quality = float(quality)
            best_reject = f"{method}:{reason}"

        # Fast exit when the original-style crop gives a very strong answer.
        if best_read is not None and best_read.confidence >= 0.82:
          break
      if best_read is not None and best_read.confidence >= 0.82:
        break

    if best_read is not None:
      self.last_reject_reason = ""
      self.last_debug = (
        f"accepted method={best_read.method} confidence={best_read.confidence:.3f} "
        f"first={best_read.first_digit_score:.3f} zero={best_read.zero_score:.3f} "
        f"margin={best_read.first_digit_margin:.3f} whole={best_read.whole_value_score:.3f} "
        f"attempted={attempted}"
      )
      return best_read

    self.last_reject_reason = best_reject
    self.last_debug = (
      f"rejected best={best_reject} quality={max(best_reject_quality, 0.0):.3f} "
      f"attempted={attempted}"
    )
    return None



@dataclass(frozen=True)
class UKNationalRead:
  confidence: float
  stripe_dark: float
  background_white: float
  red_ratio: float
  opposite_dark: float
  stripe_end_dark: float
  rim_dark: float


class UKNationalSpeedLimitReader:
  """Strict geometry scorer for a UK national-speed-limit sign.

  The reader identifies only the white circular sign with the black diagonal
  band. It does not assign 60/70 itself; mapd road context does that separately.

  V1.3 intentionally prefers false negatives over false positives. The previous
  scorer accepted any roughly circular white crop with a dark diagonal, which
  produced two false NSL detections on a drive containing no NSL signs.
  """

  MIN_CONFIDENCE = 0.78

  def __init__(self, cv2):
    self.cv2 = cv2

  def read(self, sign_crop: np.ndarray) -> UKNationalRead | None:
    if sign_crop is None or sign_crop.size == 0:
      return None

    h, w = sign_crop.shape[:2]
    if h < 18 or w < 18:
      return None

    # Hough already proposes a circle. Keep the crop near-square so elongated
    # vehicle trim, wheels, lamps and road furniture cannot pass as signs.
    aspect = w / max(h, 1)
    if aspect < 0.72 or aspect > 1.38:
      return None

    crop = self.cv2.resize(sign_crop, (128, 128), interpolation=self.cv2.INTER_CUBIC)
    hsv = self.cv2.cvtColor(crop, self.cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    white = ((val >= 125) & (sat <= 82)).astype(np.uint8)
    dark = ((val <= 108) & (sat <= 155)).astype(np.uint8)
    red = (
      (((hue <= 12) | (hue >= 168))) &
      (sat >= 70) &
      (val >= 55)
    ).astype(np.uint8)

    yy, xx = np.mgrid[0:128, 0:128]
    nx = (xx - 63.5) / 64.0
    ny = (yy - 63.5) / 64.0
    rr = np.sqrt(nx * nx + ny * ny)

    face = rr <= 0.88
    core = rr <= 0.70
    rim = (rr >= 0.72) & (rr <= 0.92)

    # UK NSL diagonal rises lower-left -> upper-right, i.e. y ~= -x in image
    # coordinates. Use a narrow centre stripe and independently demand dark
    # evidence at both ends of the physical band.
    diagonal_distance = np.abs(nx + ny)
    stripe = core & (diagonal_distance <= 0.12)
    opposite = core & (np.abs(nx - ny) <= 0.12)
    side_pos = core & ((nx + ny) >= 0.30)
    side_neg = core & ((nx + ny) <= -0.30)
    stripe_ll = core & (nx <= -0.18) & (ny >= 0.18) & (diagonal_distance <= 0.16)
    stripe_ur = core & (nx >= 0.18) & (ny <= -0.18) & (diagonal_distance <= 0.16)

    masks = (face, core, rim, stripe, opposite, side_pos, side_neg, stripe_ll, stripe_ur)
    if not all(mask.any() for mask in masks):
      return None

    stripe_dark = float(dark[stripe].mean())
    opposite_dark = float(dark[opposite].mean())

    side_pos_white = float(white[side_pos].mean())
    side_neg_white = float(white[side_neg].mean())
    background_white = min(side_pos_white, side_neg_white)

    side_pos_dark = float(dark[side_pos].mean())
    side_neg_dark = float(dark[side_neg].mean())
    background_dark = max(side_pos_dark, side_neg_dark)

    stripe_end_dark = min(
      float(dark[stripe_ll].mean()),
      float(dark[stripe_ur].mean()),
    )
    rim_dark = float(dark[rim].mean())
    face_white = float(white[face].mean())
    red_ratio = float(red[face].mean())
    diagonal_advantage = stripe_dark - opposite_dark

    # Hard gates before scoring. These specifically reject the V1.2 failure
    # class: white/grey circular clutter with an incidental dark diagonal.
    if red_ratio > 0.035:
      return None
    if stripe_dark < 0.48:
      return None
    if stripe_end_dark < 0.30:
      return None
    if diagonal_advantage < 0.16:
      return None
    if background_white < 0.52:
      return None
    if background_dark > 0.20:
      return None
    if face_white < 0.42:
      return None
    if rim_dark < 0.045:
      return None

    score = (
      min(stripe_dark / 0.72, 1.0) * 0.30 +
      min(background_white / 0.80, 1.0) * 0.25 +
      min(diagonal_advantage / 0.38, 1.0) * 0.20 +
      min(stripe_end_dark / 0.62, 1.0) * 0.15 +
      min(rim_dark / 0.20, 1.0) * 0.05 +
      max(0.0, 1.0 - red_ratio / 0.035) * 0.05
    )
    score = float(np.clip(score, 0.0, 1.0))
    if score < self.MIN_CONFIDENCE:
      return None

    return UKNationalRead(
      confidence=score,
      stripe_dark=stripe_dark,
      background_white=background_white,
      red_ratio=red_ratio,
      opposite_dark=opposite_dark,
      stripe_end_dark=stripe_end_dark,
      rim_dark=rim_dark,
    )
