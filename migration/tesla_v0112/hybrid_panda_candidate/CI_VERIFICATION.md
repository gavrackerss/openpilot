# XNOR 0.11.2 Hybrid validation (scratch only)

This change is intentionally isolated on a **draft PR** into the *staging branch*.
It exists to request GitHub Actions compilation of the complete Tesla Hybrid candidate
without modifying the active openpilot tree, its original xnor-dev branch, or either
pinned Panda/opendbc submodule.

Required gates:
- Source pin and patch export checks
- GCC host C safety + genuine-AEB and V229 state negative tests
- Tesla CarState Cap'n Proto schema parsing
- Native Tesla HW1/HW2/HW3/Model3 compiled safety tests
- Independent STM32F4 + STM32H7 cross-compilation

A passing result authorizes **only further offline replay and bench validation**.
It does not authorize flashing, vehicle CAN transmission, driving, or LS2 authority.

Verification branch synchronized with latest staging code. Scratch CI only; no on-device modification.
