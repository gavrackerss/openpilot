# Cinque Terre Original (4 September 2026) — parked Jetlink integration gate

**Branch:** `gavrackerss/openpilot:xnor-dev`

**Compiled on the phone:** Tensor_G6, original ONNX SHA-256
`e8d821733be15ebe9e27498bc27ad8bbbd741980ece37d77f377294010b8ff28`.
The phone APK must be **signed dual-model Jetlink v0.6**, with Cinque Terre
already imported and selected using **Prepare CTM**.

## Marker contract

Run `bash tools/jetlink_ctm_markers.sh stage` on a **parked** comma.

- `/data/jetlink_xnor/CTM_INTEGRATION_STAGED`: exact source identity.
- `/data/jetlink_xnor/CTM_PARKED_TEST_ENABLED`: allows only the standalone passive benchmark.
- `/data/jetlink_xnor/CTM_AUTHORITY_NOT_QUALIFIED`: explicitly records that modeld authority is *not* approved.
- `/data/jetlink_xnor/AUTHORITY_ENABLED`: **must be absent**.
- `/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED`: **must be absent**.
- `/data/jetlink_xnor/CTM_AUTHORITY_QUALIFIED`: **must be absent** and will never be created by this workflow.

The original LS2 modeld joining path is deliberately disabled while
`CTM_PARKED_TEST_ENABLED` exists. No code publishes remote outputs into modelV2,
sets Panda controls, touches steering, or upgrades the modeld authority gate.

## One parked connected camera test

From the `xnor-dev` checkout:

```bash
cd /data/openpilot
git pull --ff-only origin xnor-dev
bash tools/jetlink_ctm_markers.sh stage
bash tools/run_jetlink_ctm_park.sh
```

Prerequisites: phone prepared with CTM; USB tether `usb0`, external XNOR
Panda, **real carState reporting Park and approximately zero speed**, no
engagement/active controls; fresh road+wide camera frames and calibration.
The script refuses testing with any authority marker present.

The launcher **discovers the phone from the current usb0 gateway** rather than
assuming the previous `10.169.232.180` address. It tests TCP port 5599
before starting the 180-second measurement. On the Pixel, open Jetlink
**Settings → Enable Jetlink TCP** and ensure its server is running.
A phone-only benchmark can pass even if this network listener is disabled.

If this is a nonstandard tether and the gateway is not the phone, use
`JETLINK_CTM_HOST=<phone_usb_ip> bash tools/run_jetlink_ctm_park.sh`.
The target must still route over `usb0`; never use an unverified Wi-Fi route.

It collects 180 seconds at 20 Hz, with a two-frame UDP pipeline and
`sendmmsg` transport. Results are in
`/data/jetlink_ctm_park/park-<UTC timestamp>-<pid>/summary.json`,
`frames.csv`, and adjacent `.log` files.

## If TCP connects but Jetlink does not offer UDP

The v0.6 Android server only recognised the older fixed USB cable address,
not Android's dynamic USB NCM/RNDIS tether address. HELLO succeeds over TCP
but contains no `frame_port` or `frame_token`.

Until the corrected v0.7 signed APK is installed, a valid parked TCP baseline
can run with no app changes:

```bash
JETLINK_CTM_TRANSPORT=tcp bash tools/run_jetlink_ctm_park.sh
```

This retains the real cameras, 180-second test, 20 Hz target and two-frame
pipeline; USB frames travel over TCP instead of UDP. Do not compare the
latency with an unmeasured UDP result or infer model safety from throughput.
To use the original UDP pathway again once v0.7 is verified and installed,
run `bash tools/run_jetlink_ctm_park.sh` without the environment override.

The v0.7 server recognises **the actual local USB-tether interface** (NCM,
RNDIS, USB) when offering UDP. It does not treat arbitrary 10.x IPv4
connections or remote HELLO claims as USB.

In `summary.json` inspect:

- `ctm_parked_integration.complete_path_ms`: comma warp/preprocess start until
  the reply has been received and decoded; percentiles **including p95/p99/max**.
- `ctm_parked_integration.complete_path_over_50_ms`: number of single-frame
  complete paths exceeding 50 ms. **Not the same as 20 Hz throughput.**
- `ctm_parked_integration.capture_to_reply_ms`: adds camerad frame age and
  scheduling from capture (further upstream).
- `counts.achieved_fps`, `cadence_gate`, `errors`, and `phone_stage_timing_ms`.

The 50 ms target should be judged against both complete processing and
capture-to-reply, not just the Tensor G6 `server_npu_ms` (36.2 ms mean in
the earlier phone-only benchmark).

**To disarm after the test:**

```bash
bash tools/jetlink_ctm_markers.sh disarm
```

A successful connected parked test is **not** numerical parity, safe
trajectory decoding, reliable on-road fallback, steering qualification or
authority permission. The existing `jetlink_native_authority.py` is
**LS2-specific** and must not be pointed at CTM by just changing the hash:
CTM has a different six-input history-buffer interface. Those conditions
need a separate verified modeld adapter and shadow/handover assessment.

## New dedicated native CTM adapter (non-authoritative)

Implemented in `openpilot/jetlink_ctm_adapter.py`, backed by the pure
strict wire manifest `openpilot/jetlink_ctm_contract.py`, and tested by
`openpilot/jetlink_ctm_park_handover.py`. This does **not** change
`selfdrive/modeld/modeld.py` or the existing LS2 authority adapter.

CTM Original uses the six-input **queued graph**: `img`, `big_img`,
`desire_pulse`, `traffic_convention`, `action_t` and `features_buffer`.
The Tensor G6 Android server's `PolicyQueues` owns its image, desire and
feature history at `frame_skip=4`. The comma sends two warped images plus
12 scalar floats, not the full history. Jetlink's native Zoompilot
`JoiningModelState` handles swap/hold/demotion and the local stock
model remains available as the fallback.

The adapter validates the CTM source SHA, exact input shapes, 18452-float
output, expected Parser output slices, 2068-float compressed reply, and
parsed plan/lanes/leads/desire finite shapes. **This slice mapping still
needs confirmation from a live CTM ENGINE_RESP and numerical validation
against the model graph; no empirical parity is claimed.** Any disagreement
fails closed instead of interpreting outputs as LS2.

### Run a real isolated parked Zoompilot handover/fallback test

Use Jetlink **v0.7** on the Pixel with original CTM prepared and USB-NCM
tether enabled. The comma must be in Park with fresh valid car/CAN state,
two live cameras and calibration. Ensure there are no simultaneous Jetlink
benchmark, shadow or authority sessions competing for the phone.

```bash
cd /data/openpilot
git pull --ff-only origin xnor-dev
bash tools/jetlink_ctm_markers.sh stage
bash tools/run_jetlink_ctm_handover_park.sh
```

The harness imports the actual stock `ModelState`, creates a separate
Zoompilot `JoiningModelState`, consumes real road/wide camera frames, and
checks strict live parked state before each frame. It waits for real CTM
inferences, validates decoded predictions, invokes the same modeld action
math **without publishing it**, closes only its own Jetlink socket and
requires actual local-model fallback. All metrics and failures go to
`/data/jetlink_ctm_park/handover/ctm-handover-*.log`. It never publishes
`modelV2`, sends CAN, changes Panda or modifies the real modeld process.

A passing result has an event `"result"` with `"passed": true`,
`"remote_frames"` nonzero, `"socket_loss_triggered": true`,
`"fallback": true`, and `"model_messages_published": 0`.
Any exception or lack of handover is a failed test. The source
`openpilot/jetlink_ctm_contract.py` has pure Python regression tests:

```bash
cd /data/openpilot
python3 -m unittest discover -s tests -p test_jetlink_ctm_contract.py -v
```

To remove parked staging after testing:

```bash
bash tools/jetlink_ctm_markers.sh disarm
```

### Authority remains explicitly unavailable

`openpilot.jetlink_ctm_adapter.attach_authority()` always fails closed,
and production `selfdrive/modeld/modeld.py` does not import this module.
A parked handover proves a real protocol / decoder / fallback sequence,
**not** numerical output parity, plausible trajectories, reliable
on-road operation, steering safety, or authority. Independent output
parity, actual camera-to-reply deadline behaviour with live local modeld,
and a non-actuating on-road shadow trial are still needed before a
separately reviewed production integration.
