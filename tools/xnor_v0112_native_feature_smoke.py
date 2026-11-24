#!/usr/bin/env python3
"""Desktop CI smoke test of locally built STOCK model and real preview pixels.

Do not point this at downloaded/untrusted pickle files. It loads only the
stock bundle just compiled from the checked-out native model source.
No manager, messaging publishers, camera capture or vehicle control.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path


def stock_model(output: Path):
  from openpilot.common.file_chunker import open_file_chunked
  from openpilot.selfdrive.modeld.helpers import modeld_pkl_path, load_oob
  from openpilot.sunnypilot.models.native_envelope import inspect_envelope
  from openpilot.sunnypilot.models.native_abi import describe_metadata
  path = modeld_pkl_path(False)
  with open_file_chunked(path) as stream:
    framing = inspect_envelope(stream)
  with open_file_chunked(path) as stream:
    bundle = load_oob(stream)
  metadata = bundle["metadata"]
  shapes = metadata["input_shapes"]
  slices = metadata["output_slices"]
  for name in ("img", "big_img", "desire_pulse", "traffic_convention", "action_t", "features_buffer"):
    assert name in shapes, name
    assert all(isinstance(dim, int) and dim > 0 for dim in shapes[name]), name
  assert tuple(shapes["action_t"]) == (1, 2), shapes["action_t"]
  assert "plan" in slices and "lead" in slices and "hidden_state" in slices
  if "action" in slices:
    action = slices["action"]
    assert action.stop - action.start == 2, action
  for key in ("run_policy", (1928, 1208), (1344, 760)):
    assert callable(bundle[key]), key
  # Exercise the locally built model in an isolated desktop process, without
  # any publishers, camera devices or calls to modeld.main().
  import numpy as np
  import time
  from openpilot.selfdrive.modeld.modeld import ModelState
  from openpilot.selfdrive.modeld.constants import ModelConstants
  del bundle
  model = ModelState(1928, 1208, False)
  frames = {name: np.zeros(model.frame_buf_params[name][3], dtype=np.uint8)
            for name in model.vision_input_names}
  transforms = {name: np.eye(3, dtype=np.float32) for name in frames}
  durations = []
  for _ in range(5):
    inputs = {"desire_pulse": np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32),
              "traffic_convention": np.array([1., 0.], dtype=np.float32),
              "action_t": np.array([.2, .3], dtype=np.float32)}
    start = time.perf_counter()
    result = model.run(frames, transforms, inputs)
    durations.append((time.perf_counter() - start) * 1000.)
    assert result is not None
    for name in ("plan", "lead", "hidden_state"):
      assert name in result and np.all(np.isfinite(result[name])), name
    assert all(np.all(np.isfinite(value)) for value in result.values())
  output.write_text(json.dumps(describe_metadata(metadata), indent=2) + "\n")
  print(json.dumps({"synthetic_stock_inference_frames": len(durations),
                    "desktop_inference_ms": durations,
                    "stock_native_envelope": asdict(framing),
                    "stock_input_shapes": shapes,
                    "stock_output_names": sorted(slices),
                    "stock_bundle_deserialised": True,
                    "custom_model_qualified": False}, indent=2))


def ui_preview(output: Path):
  import pyray as rl
  from openpilot.common.hardware import PC
  from openpilot.system.ui.lib.application import gui_app
  from openpilot.selfdrive.ui.onroad.manual_speed_limit_preview import ManualSpeedLimitPreview
  assert PC, "Desktop smoke only"
  gui_app.init_window("XNOR 0.11.2 preview smoke")
  target = None
  image = None
  try:
    widget = ManualSpeedLimitPreview()
    assert not widget.enabled
    target = rl.load_render_texture(960, 220)
    assert target.id != 0
    rl.begin_texture_mode(target)
    try:
      rl.clear_background(rl.Color(0, 0, 0, 0))
      widget.set_rect(rl.Rectangle(30, 35, 900, 132))
      widget._render(widget.rect)
    finally:
      rl.end_texture_mode()
    image = rl.load_image_from_texture(target.texture)
    rl.image_flip_vertical(image)
    assert image.width == 960 and image.height == 220
    # Background remains transparent; sample rings and faces away from glyphs.
    assert rl.get_image_color(image, 2, 2).a == 0
    counts = {"red": 0, "white": 0, "dark": 0}
    for y in range(70, 167, 2):
      for x in range(30, 930, 2):
        c = rl.get_image_color(image, x, y)
        if c.a > 200:
          if c.r > 150 and c.g < 80 and c.b < 80:
            counts["red"] += 1
          elif min(c.r, c.g, c.b) > 230:
            counts["white"] += 1
          elif max(c.r, c.g, c.b) < 50:
            counts["dark"] += 1
    assert counts["red"] > 100 and counts["white"] > 100 and counts["dark"] > 20, counts
    assert rl.export_image(image, str(output)), "Could not export preview PNG"
    print(json.dumps({"preview_rendered": True, "touch_enabled": False,
                      "pixel_counts": counts, "output": str(output)}))
  finally:
    if image is not None:
      rl.unload_image(image)
    if target is not None:
      rl.unload_render_texture(target)
    gui_app.close()


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("mode", choices=("stock", "ui"))
  parser.add_argument("--output", type=Path)
  args = parser.parse_args()
  if args.mode == "stock":
    stock_model(args.output or Path('native-stock-abi.json'))
  else:
    ui_preview(args.output or Path('speed-sign-preview.png'))
