"""Desktop-only NumPy reference for the legacy split-model camera input.

Mirrors the pinned OPM10V3 compile_warp.py (Git blob
47511f2a2b6b23ea8a893015bec4598942355e90) nearest-neighbor clipping,
NV12 plane layout and five-frame image history. This is not a live runner.
"""
from collections import deque
import numpy as np


def sample(plane, matrix, width, height):
  y, x = np.indices((height, width), dtype=np.float32)
  u = matrix[0, 0]*x + matrix[0, 1]*y + matrix[0, 2]
  v = matrix[1, 0]*x + matrix[1, 1]*y + matrix[1, 2]
  w = matrix[2, 0]*x + matrix[2, 1]*y + matrix[2, 2]
  if not np.all(np.isfinite(w)) or np.any(w == 0):
    raise ValueError("Invalid projective transform")
  x_src = np.clip(np.rint(u/w), 0, plane.shape[1]-1).astype(np.intp)
  y_src = np.clip(np.rint(v/w), 0, plane.shape[0]-1).astype(np.intp)
  return plane[y_src, x_src]


def prepare(frame, transform, camera, layout):
  width, height = camera
  stride, y_height, uv_height, size = layout
  if frame.dtype != np.uint8 or frame.shape != (size,):
    raise ValueError("Invalid native NV12 frame")
  transform = np.asarray(transform, dtype=np.float32)
  if transform.shape != (3, 3) or not np.all(np.isfinite(transform)):
    raise ValueError("Invalid warp matrix")
  luma = frame[:height*stride].reshape(height, stride)[:, :width]
  offset = stride*y_height
  uv = frame[offset:offset+stride*uv_height].reshape(uv_height, stride)[:height//2, :width]
  uv_matrix = transform * np.array([[1, 1, .5], [1, 1, .5], [2, 2, 1]], dtype=np.float32)
  y = sample(luma, transform, 512, 256)
  u = sample(uv[:, 0::2], uv_matrix, 256, 128)
  v = sample(uv[:, 1::2], uv_matrix, 256, 128)
  return np.stack((y[0::2, 0::2], y[1::2, 0::2], y[0::2, 1::2], y[1::2, 1::2], u, v))


class CameraHistory:
  def __init__(self, camera, layout):
    self.camera = camera
    self.layout = layout
    self.queues = {name: deque([np.zeros((6, 128, 256), dtype=np.uint8) for _ in range(5)], maxlen=5)
                   for name in ("img", "big_img")}

  def update(self, road, wide, road_transform, wide_transform):
    outputs = {}
    for key, frame, tfm in (("img", road, road_transform), ("big_img", wide, wide_transform)):
      queue = self.queues[key]
      queue.append(prepare(frame, tfm, self.camera, self.layout))
      outputs[key] = np.concatenate((queue[0], queue[-1]), axis=0)[None]
    return outputs


def self_test():
  camera = (512, 256)
  layout = (512, 256, 128, 512*384)
  frame = np.zeros(layout[3], dtype=np.uint8)
  y = frame[:512*256].reshape(256, 512)
  y[0::2, 0::2] = 11
  y[1::2, 0::2] = 22
  y[0::2, 1::2] = 33
  y[1::2, 1::2] = 44
  uv = frame[512*256:].reshape(128, 512)
  uv[:, 0::2] = 55
  uv[:, 1::2] = 66
  tfm = np.eye(3, dtype=np.float32)
  expected = np.array([11,22,33,44,55,66], dtype=np.uint8)
  assert np.all(prepare(frame, tfm, camera, layout) == expected[:,None,None])
  history = CameraHistory(camera, layout)
  for i in range(5):
    image = history.update(frame, frame, tfm, tfm)["img"]
    assert np.all(image[0,6:] == expected[:,None,None])
    assert np.all(image[0,:6] == (expected[:,None,None] if i == 4 else 0))
  print("NV12 channel and image-history reference self-test passed")


if __name__ == "__main__":
  self_test()
