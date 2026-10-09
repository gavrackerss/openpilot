# V239 VSL V3 shadow classifier

This branch adds a **shadow-only** V3 UK speed-sign classifier alongside the existing
V1.10 authoritative vision/OCR/NSL pipeline.

## Runtime model

Repository-relative path expected by the runtime:

`selfdrive/assets/vision_models/speed_limit_v3_classifier.onnx`

SHA-256:

`0a835d417950957fd524c76156eec7d301acb7d8b6aa5a5035cae63adc753bb2`

Input: NCHW float32 RGB, 128x128, ImageNet normalization.
Output class order: `20, 30, 40, 50, 60, 70, NSL, OTHER`.
The exported ONNX supports dynamic batch size; runtime caps a batch at 4 proposals.

The binary model is distributed in the V239 changed-files overlay rather than
committed to this branch through the ChatGPT GitHub connector.

## Training data

51 supplied drive-video files were reviewed, representing 49 unique video contents
after byte-identical replacements were deduplicated.

Verified real drive crops used by V3:

- 30 mph: 48
- 40 mph: 111
- 50 mph: 22
- NSL: 4
- OTHER/regulatory negatives: 95
- Total: 280

20/60/70 retain the clean/external reference material because those signs were not
present as verified examples in the supplied footage.

## Safety boundary

V3 predictions are emitted only as `[XNOR_VSL_V3_SHADOW]` logs with
`authoritative=0`. They do not enter temporal history, publishing, lower-candidate
logic, map/vision arbitration, or longitudinal control.

If the V3 model is absent, fails to load, or fails inference, the established V1.10
pipeline continues unchanged.

## Validation

GitHub CI syntax-checks the exact branch source. The ONNX exported from the final
checkpoint was compared with PyTorch on 64 spread samples: 0 argmax disagreements,
maximum absolute logit difference 3.31e-05. OpenCV DNN was also tested with batch
sizes 1, 2, and 4.
