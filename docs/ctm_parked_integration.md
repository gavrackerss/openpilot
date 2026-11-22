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
