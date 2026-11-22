#!/usr/bin/env python3
"""Contract and fail-closed tests. No openpilot runtime, GPU or phone needed."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from openpilot import jetlink_ctm_contract as C
from openpilot import jetlink_ctm_adapter as A


def known_spec():
    return {
        "sha256": C.SOURCE_SHA,
        "nbytes": C.SOURCE_NBYTES,
        "frame_skip": 4,
        "input_shapes": {k: list(v) for k, v in C.INPUT_SHAPES.items()},
        "output_shapes": {"outputs": [1, 18452]},
        "output_slices": {k: list(v) for k, v in C.PARSER_SLICES.items()},
    }


class NativeSpec:
    model_hw = (128, 256)
    packed_nelem = 12
    warped_nbytes = 393216
    reply_nelem = 2068
    stateful = False
    state_pairs = {}

    def to_dict(self):
        return known_spec()


class ContractTests(unittest.TestCase):
    def test_original_queued_six_input_contract(self):
        C.validate_spec(known_spec())
        C.validate_native_spec(NativeSpec())

    def test_ls2_rejected(self):
        s = known_spec()
        s["sha256"] = "909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8"
        with self.assertRaisesRegex(C.ContractError, "source SHA"):
            C.validate_spec(s)

    def test_reject_changed_history_or_scalar_layout(self):
        for name, changed in [
            ("features_buffer", [1, 32, 16, 512]),
            ("img", [2, 6, 128, 256]),
            ("desire_pulse", [1, 32, 8]),
            ("action_t", [1, 4]),
        ]:
            with self.subTest(name=name):
                s = known_spec()
                s["input_shapes"][name] = changed
                with self.assertRaises(C.ContractError):
                    C.validate_spec(s)

    def test_reject_changed_output_layout(self):
        for key in ("plan", "action", "hidden_state", "pad", "lead", "road_edges"):
            with self.subTest(key=key):
                s = known_spec()
                s["output_slices"][key] = [0, 7]
                with self.assertRaisesRegex(C.ContractError, "slice"):
                    C.validate_spec(s)

    def test_reject_missing_output_slices(self):
        s = known_spec()
        del s["output_slices"]
        with self.assertRaises(C.ContractError):
            C.validate_spec(s)

    def test_reject_wrong_nbytes_stride_and_output_shape(self):
        for key, value in [("nbytes", 123), ("frame_skip", 5),
                           ("output_shapes", {"outputs": [1, 18450]})]:
            with self.subTest(key=key):
                s = known_spec()
                s[key] = value
                with self.assertRaises(C.ContractError):
                    C.validate_spec(s)

    def test_reject_unexpected_native_state_pairs(self):
        native = NativeSpec()
        native.state_pairs = {"state_img": "next_state_img"}
        with self.assertRaisesRegex(C.ContractError, "queued graph"):
            C.validate_native_spec(native)

    def test_no_authority_entrypoint(self):
        self.assertFalse(A.enabled())
        with self.assertRaisesRegex(RuntimeError, "NOT QUALIFIED"):
            A.attach_authority()

    def test_parked_attach_rejects_without_isolated_process(self):
        with patch.dict("os.environ", {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "isolated"):
                A.attach_parked(None, 1928, 1208)

    def test_usb_gateway_auto_discovery(self):
        def route(cmd, **kwargs):
            dest = cmd[-1]
            if dest == A.PROBE_TARGET:
                return SimpleNamespace(stdout="10.169.232.180 via 10.162.48.109 dev usb0 src 10.162.48.71")
            return SimpleNamespace(stdout="10.162.48.109 dev usb0 src 10.162.48.71")
        with patch.dict("os.environ", {"JETLINK_CTM_HOST": ""}):
            with patch.object(A.subprocess, "run", side_effect=route):
                self.assertEqual(A.discover_phone(), "10.162.48.109")

    def test_wifi_never_accepted_as_usb(self):
        with patch.dict("os.environ", {"JETLINK_CTM_HOST": "192.168.1.50"}):
            with patch.object(A.subprocess, "run", return_value=SimpleNamespace(
                stdout="192.168.1.50 dev wlan0 src 192.168.1.9"
            )):
                with self.assertRaisesRegex(ConnectionError, "usb0"):
                    A.discover_phone()


if __name__ == "__main__":
    unittest.main()
