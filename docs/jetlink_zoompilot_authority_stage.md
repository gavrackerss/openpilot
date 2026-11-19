# XNOR/Zoompilot authority port (staged, NOT validated for driving)

This branch is deliberately separate from xnor-dev. It does not enable authority
on any device by itself. Do not enable remote vehicle-control predictions until
the on-device bench tests, matched camera preprocessing, action semantics, and
controlled disengaged fallback tests are completed.

## Reused upstream source
- gavrackerss/jetlink, main: jetlink.openpilot.joining.JoiningModelState
  performs disengaged handover, deadline/hold-based demotion, and reconnect.
- gavrackerss/jetlink, main: jetlink.openpilot.model_state.JetlinkModelState
  handles recurrent LS2 inference, parser output, held predictions, deadlines.
- gavrackerss/jetlink, main: jetlink.client.JetlinkClient.open_tcp()
  supports the Android USB tether 10.169.232.180:5599, including UDP frames.
- gavrackerss/openpilot, xnor-dev: the already proven V5.8 NV12 mapping
  (openpilot/jetlink_shadow_bench.py: warp_indices) is reused.

Unlike Zoompilot's standard USB gadget, Android USB tethering is a **phone-hosted
network**. Do not run the Zoompilot comma gadget owner; it reverses USB roles.
The upstream jetlink/ Python package must be available for import in the
openpilot Python environment. It is not part of this feature-branch source.

## Full replacement files
- openpilot/jetlink_authority.py (new)
- selfdrive/modeld/modeld.py
- system/manager/process_config.py
- cereal/log.capnp
- selfdrive/selfdrived/events.py
- selfdrive/selfdrived/selfdrived.py

The traditional sunnypilot/modeld_v2/modeld.py is **not** yet integrated.
This port applies to the stock modeld runner only. Never enable with the
tinygrad custom-model runner. Full live compatibility is not verified.

## Fail-closed gate
Both /data/jetlink_xnor/AUTHORITY_ENABLED and
/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED must exist **before** modeld starts;
the qualification marker is intentionally not supplied or created by this
package. The route to the phone must resolve over usb0. Without ALL of these,
the ordinary local model continues unchanged.

The onroad V5.8 shadow observer is disabled when AUTHORITY_ENABLED exists, so
it does not contend for a single Pixel server connection.

## Onroad safeguards (ported but NOT vehicle validated)
- The local split model is loaded and retained as fallback.
- Model switch is performed by Zoompilot's JoiningModelState with fresh control
  status, while disengaged, and only after a successful remote join.
- Remote model SHA, frame geometry and output size are verified against saved
  Tensor G6 V5.8 session metadata; no arbitrary remote model accepted.
- Repeated held frames, deadlines, invalid outputs and loss of link are handled
  by the upstream demotion/rejoin implementation.
- modelV2.big reports actual publisher; SwapEvents can refuse engagement during
  the first second after switch.
- The local model resets split state queues on demotion.

## Open verification items
1. Verify that the correct pinned gavrackerss/jetlink Python package is on device.
2. Build capnp/cereal after schema change. A py_compile-only run is NOT enough.
3. Verify both camera streams have compatible buffer widths and heights.
4. Prove Warp output parity on identical frames against the V5.8 shadow samples.
5. Verify full local model initialization, frame cadence and fallback in replay
   and a stationary bench test, including deliberate network loss and thermal
   events, before any driver-authoritative test.
6. Check selfdrived's event timing and control availability on the actual car.
7. Verify current tinygrad source and Android APK handshake compatible with
   upstream JetlinkClient. V5.8 benchmarks did not exercise this client path.

No APK changes should be assumed. The model SHA is
909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8;
model input/output spec is from V5.8 summary.json.

This port is not an assurance of autonomous vehicle safety or a recommendation
to use remote model authority on public roads.
