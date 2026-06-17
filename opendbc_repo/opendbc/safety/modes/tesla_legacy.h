// opendbc_repo/opendbc/safety/modes/tesla_legacy.h
#pragma once

#include "opendbc/safety/declarations.h"

#define XNOR_V167_AEB_ONLY_EARLY_BASE 1
static const char xnor_v167_aeb_only_early_base_marker[] __attribute__((used)) =
    "XNOR_V167_AEB_ONLY_EARLY_BASE";

// Tesla Legacy (HW1/HW2/HW3) Unity-parity safety for XNOR harnessing.
//
// Unity parity implemented:
//  - Stalk (0x45) gated by op_autopilot_disabled from internal 0x659 (byte5 bit7)
//  - Forwarding mods (HUD hiding + EPAS eacStatus workaround) with additive last-byte checksum
//  - No relay faults: all TX allowlist entries use check_relay=false
//
// XNOR contract:
//  - safetyParam uses TeslaSafetyFlags (opendbc_repo/opendbc/car/tesla/values.py):
//      LONG_CONTROL=1, FSD_14=2 (non-legacy only), FLAG_EXTERNAL_PANDA=4, FLAG_HW1=8, FLAG_HW2=16, FLAG_HW3=32, OP_STALK_ENABLE=64
//  - Main panda: lateral TX + stock LKAS passthrough + Unity forwarding mods
//  - External panda: longitudinal TX + stock AEB passthrough

// --- safetyParam bits (TeslaSafetyFlags) ---
#define TESLA_LEGACY_FLAG_LONG_CONTROL       0x01U
#define TESLA_LEGACY_FLAG_EXTERNAL_PANDA     0x04U
#define TESLA_LEGACY_FLAG_HW1                0x08U
#define TESLA_LEGACY_FLAG_HW2                0x10U
#define TESLA_LEGACY_FLAG_HW3                0x20U
#define TESLA_LEGACY_FLAG_OP_STALK_ENABLE    0x40U

// --- Unity timing ---
static const uint32_t TESLA_LEGACY_TIME_TO_HIDE_ERRORS_US = 4000000U;
static const uint32_t TESLA_LEGACY_TIME_FOR_HANDS_ON_US   = 1000000U;
static const uint32_t TESLA_LEGACY_AEB_HUD_SCRUB_US       = 3500000U;

// --- runtime state (namespaced; safety.h includes tesla.h too) ---
static bool tesla_legacy_external_panda = false;
static bool tesla_legacy_has_ap_hw = false;
static bool tesla_legacy_op_stalk_enable = false;
static int tesla_legacy_chassis_bus = 0;
// DAS_control lives in different ID namespaces per bus: 0x2B9 on the chassis/party bus (main
// panda) and 0x2BF on the powertrain bus (external panda). Unity (das_control_addr) and vanilla
// opendbc (das_control_msg) both split it; we set it per-panda in init and use it for every
// DAS_control match. The additive checksum always bases on 0x2B9 regardless (see
// tesla_legacy_calc_checksum8), so that path is unchanged.
static int tesla_legacy_das_control_addr = 0x2BF;

// internal OP->safety carrier (0x659) — Unity parity bits (byte5)
static bool tesla_legacy_op_autopilot_disabled = false;  // bit7
static bool tesla_legacy_pedal_enabled = false;          // 0x659 byte5 bit5 (Unity parity)
static bool tesla_legacy_op_stalk_main_edge = false;     // bit1 (edge)
static bool tesla_legacy_op_stalk_cancel_edge = false;   // bit0 (edge)

// stock system detection on AP-side bus (bus2)
static bool tesla_legacy_stock_lkas = false;  // from 0x488 steerControlType
static bool tesla_legacy_stock_aeb = false;   // from 0x2BF AEB event

// stock AP states from DAS/AP frames
static bool tesla_legacy_autopilot_enabled = false;  // 0x399
static bool tesla_legacy_eac_enabled = false;        // 0x219
static bool tesla_legacy_autopark_enabled = false;   // 0x219

// hands on wheel (from 0x370)
static bool tesla_legacy_hands_on = false;
static uint32_t tesla_legacy_hands_on_last_signal = 0U;

// time tracking for HUD hiding after disengage
static uint32_t tesla_legacy_time_op_disengaged = 0U;
static bool tesla_legacy_controls_allowed_prev = false;
static bool tesla_legacy_hide_errors_armed = false;
static uint32_t tesla_legacy_last_aeb_hud_warning_us = 0U;

// gear tracking for reverse -> drive re-arm
static bool tesla_legacy_in_reverse = false;
static uint8_t tesla_legacy_last_gear = 0U;

// local safety reset used on reverse entry and reverse -> drive
static void tesla_legacy_reset_after_gear_change(void) {
  controls_allowed = false;
  cruise_engaged_prev = false;
  steering_disengage = false;

  tesla_legacy_stock_lkas = false;
  tesla_legacy_stock_aeb = false;
  tesla_legacy_autopilot_enabled = false;
  tesla_legacy_eac_enabled = false;
  tesla_legacy_autopark_enabled = false;

  tesla_legacy_op_stalk_main_edge = false;
  tesla_legacy_op_stalk_cancel_edge = false;

  tesla_legacy_time_op_disengaged = 0U;
  tesla_legacy_controls_allowed_prev = false;
  tesla_legacy_hide_errors_armed = false;
  tesla_legacy_last_aeb_hud_warning_us = 0U;
}


// --- helpers ---

static void tesla_legacy_track_controls_allowed_edge(void) {
  if (tesla_legacy_controls_allowed_prev && !controls_allowed) {
    tesla_legacy_time_op_disengaged = microsecond_timer_get();
    tesla_legacy_hide_errors_armed = true;
    tesla_legacy_controls_allowed_prev = false;
  } else if (controls_allowed) {
    tesla_legacy_hide_errors_armed = false;
  }
  tesla_legacy_controls_allowed_prev = controls_allowed;
}

static uint8_t tesla_legacy_calc_checksum8(const CANPacket_t *msg, int len) {
  // Unity parity: additive checksum includes addr low+high bytes.
  // Special-case: 0x2BF uses 0x2B9 for checksum.
  uint16_t addr = (uint16_t)msg->addr;
  if (addr == 0x2BFU) {
    addr = 0x2B9U;
  }
  uint8_t checksum = (uint8_t)(addr & 0xFFU) + (uint8_t)((addr >> 8) & 0xFFU);
  for (int i = 0; i < (len - 1); i++) {
    checksum = (uint8_t)(checksum + msg->data[i]);
  }
  return checksum;
}

static void tesla_legacy_set_last_byte_checksum(CANPacket_t *msg) {
  const int len = GET_LEN(msg);
  if (len > 0) {
    msg->data[len - 1] = tesla_legacy_calc_checksum8(msg, len);
  }
}

static bool tesla_legacy_vehicle_stopped_or_unknown(void) {
  const int speed_sample = vehicle_speed.values[0];
  return !vehicle_moving || ((speed_sample >= -500) && (speed_sample <= 500));
}

static bool tesla_legacy_stock_ap_idle(void) {
  return !tesla_legacy_autopilot_enabled &&
         !tesla_legacy_eac_enabled &&
         !tesla_legacy_autopark_enabled;
}

static bool tesla_legacy_aeb_event_is_warning_only(int aeb_event) {
  return (aeb_event == 2) || (aeb_event == 3) ||
         ((aeb_event == 1) && tesla_legacy_vehicle_stopped_or_unknown());
}

static void tesla_legacy_note_aeb_hud_warning(int aeb_event) {
  if (tesla_legacy_aeb_event_is_warning_only(aeb_event)) {
    tesla_legacy_last_aeb_hud_warning_us = microsecond_timer_get();
  }
}

static bool tesla_legacy_aeb_hud_scrub_active(void) {
  return (tesla_legacy_last_aeb_hud_warning_us != 0U) &&
         (get_ts_elapsed(microsecond_timer_get(), tesla_legacy_last_aeb_hud_warning_us) <= TESLA_LEGACY_AEB_HUD_SCRUB_US);
}

static bool tesla_legacy_hud_takeover_owner(void) {
  return tesla_legacy_op_autopilot_disabled && tesla_legacy_stock_ap_idle();
}

static bool tesla_legacy_should_scrub_aeb_event(int aeb_event) {
  if (aeb_event == 1) {
    return tesla_legacy_vehicle_stopped_or_unknown();
  }
  return (aeb_event == 2) || (aeb_event == 3) ||
         ((aeb_event != 0) &&
          (tesla_legacy_hud_takeover_owner() || tesla_legacy_aeb_hud_scrub_active()));
}

static void tesla_legacy_scrub_das_control_aeb(CANPacket_t *msg) {
  msg->data[2] &= 0xFCU;
  tesla_legacy_set_last_byte_checksum(msg);
}

static void __attribute__((unused)) tesla_legacy_scrub_status2_warnings(CANPacket_t *msg) {
  uint32_t w0 = (uint32_t)GET_BYTES(msg, 0, 4);
  uint32_t w1 = (uint32_t)GET_BYTES(msg, 4, 4);

  // Keep speed/offset/counter, but force AP warning/fault fields to Unity-like idle values.
  w0 &= ~((0x7U << 10) | (1U << 13) | (0x3U << 14) |
          (0x7U << 16) | (0x3U << 19) | (0x7U << 21) |
          (0x3U << 24) | (0x1FU << 26) | (1U << 31));
  w1 &= ~((0xFFU << 0) | (0xFU << 16) | (0xFFU << 24));

  w0 |= (1U << 26);     // DAS_ACC_report = target CIPV / benign active target
  w0 |= (1U << 31);     // DAS_lssState bit 0
  w1 |= 0x03U;          // DAS_lssState bits 1..2 => 7/off
  w1 |= (1U << 2);      // DAS_radarTelemetry = normal
  w1 |= (2U << 4);      // DAS_robState = active
  w1 |= (0xFU << 16);   // DAS_longCollisionWarning = 15 (SNA) -- Unity's no-warning value. 0/NONE
                        // (the prior XNOR_V167 value) is what the IC renders as the AEB warning.

  msg->data[0] = (uint8_t)(w0 & 0xFFU);
  msg->data[1] = (uint8_t)((w0 >> 8) & 0xFFU);
  msg->data[2] = (uint8_t)((w0 >> 16) & 0xFFU);
  msg->data[3] = (uint8_t)((w0 >> 24) & 0xFFU);
  msg->data[4] = (uint8_t)(w1 & 0xFFU);
  msg->data[5] = (uint8_t)((w1 >> 8) & 0xFFU);
  msg->data[6] = (uint8_t)((w1 >> 16) & 0xFFU);
  msg->data[7] = (uint8_t)((w1 >> 24) & 0xFFU);
  tesla_legacy_set_last_byte_checksum(msg);
}

static void __attribute__((unused)) tesla_legacy_scrub_status_warnings(CANPacket_t *msg, uint8_t autopilot_status) {
  msg->data[0] = (uint8_t)((msg->data[0] & 0xF0U) | (autopilot_status & 0x0FU));
  msg->data[2] &= 0x3FU;  // DAS_forwardCollisionWarning = 0
  msg->data[3] &= 0x3FU;  // DAS_sideCollisionAvoid = 0
  msg->data[4] = 0U;      // DAS_sideCollisionWarning/Inhibit/CSA/LDW = 0
  tesla_legacy_set_last_byte_checksum(msg);
}


static void __attribute__((unused)) tesla_legacy_clear_warning_matrix(CANPacket_t *msg) {
  for (int i = 0; i < GET_LEN(msg); i++) {
    msg->data[i] = 0U;
  }
  tesla_legacy_set_last_byte_checksum(msg);
}

// LEAN 0x399 scrub for the bus2->bus0 forward when OP does NOT own the HUD. Touches ONLY the
// two AEB-relevant fields: autopilotStatus(0|4)=2 (Unity's benign "AP active" state that makes
// the IC drop the warning) + DAS_forwardCollisionWarning(22|2)=0. Deliberately does NOT zero
// side-collision / LDW (data[3]/data[4]) -- doing so makes the frame incoherent and the IC
// refuses to clear the latched AEB. This lean form is what produced a momentary,
// self-clearing flash; the full warning rewrite regressed it to a sticky latch.
static void __attribute__((unused)) tesla_legacy_scrub_fcw_only(CANPacket_t *msg) {
  msg->data[0] = (uint8_t)((msg->data[0] & 0xF0U) | 0x02U);  // autopilotStatus = 2 (benign)
  msg->data[2] &= 0x3FU;                                     // DAS_forwardCollisionWarning = 0
  tesla_legacy_set_last_byte_checksum(msg);
}

// LEAN 0x389 (DAS_status2) scrub -- the DAS_status2 analogue of tesla_legacy_scrub_fcw_only.
// Touches ONLY the two AEB-relevant warning fields and leaves the live counter + every other
// field intact, so the stock frame's own continuous DAS_status2Counter reaches the IC. Unity's
// AP-car forward keeps the stock counter and zeroes the warnings; the full status2 rewrite
// (tesla_legacy_scrub_status2_warnings) regressed to a sticky latch, so ownership mode uses this.
//   DAS_activationFailureStatus 14|2  -> 0  (byte1 bits 6-7)
//   DAS_longCollisionWarning    48|4  -> 15 (SNA)  (byte6 low nibble). DBC: 0=NONE, 1..12=warnings,
//      15=SNA. Unity sets this to 15/SNA for the no-warning state: the IC drops the AEB indicator on
//      SNA ("forward-collision signal unavailable") but treats a valid 0/NONE as a live report and
//      keeps the warning lit. Forcing 0/NONE was the porting bug that kept the flash on.
//   DAS_status2Counter          52|4  preserved (byte6 high nibble)
static void __attribute__((unused)) tesla_legacy_scrub_status2_lean(CANPacket_t *msg) {
  msg->data[1] &= 0x3FU;   // DAS_activationFailureStatus = 0
  msg->data[6] |= 0x0FU;   // DAS_longCollisionWarning = 15 (SNA, Unity parity); keep counter (high nibble)
  tesla_legacy_set_last_byte_checksum(msg);
}

// --- RX hook ---
static void tesla_legacy_rx_hook(const CANPacket_t *msg) {
  const int bus = (int)msg->bus;
  const int addr = (int)msg->addr;

  // Vehicle state — bus-aware IDs.
  // The chassis (tesla_can) and powertrain (tesla_powertrain) DBCs are separate ID
  // namespaces: the SAME signal has a different message ID on each bus. The main panda
  // sits on the chassis bus; the external panda sits on the powertrain bus. Keying these
  // reads off the panda's role lets the external panda derive gas/brake/cruise/speed from
  // the frames it actually sees on its bus, instead of relying solely on the 0x659 carrier
  // — the prior source of the intermittent controls mismatch. Bit layouts are identical
  // across both DBCs for these signals (verified against the uploaded DBCs), EXCEPT speed:
  // chassis exposes it via ESP_B (0x155); PT exposes it via DI_torque2.DI_vehicleSpeed.
  const int id_pedal   = tesla_legacy_external_panda ? 0x106 : 0x108;  // DI_torque1.DI_pedalPos 48|8 (0.4)
  const int id_brake   = tesla_legacy_external_panda ? 0x1F8 : 0x20A;  // BrakeMessage.driverBrakeStatus 2|2
  const int id_distate = tesla_legacy_external_panda ? 0x256 : 0x368;  // DI_state.DI_cruiseState 12|4
  const int id_ditrq2  = tesla_legacy_external_panda ? 0x116 : 0x118;  // DI_torque2 (PT speed source)

  // Chassis state (HW3 uses bus1)
  if (bus == tesla_legacy_chassis_bus) {
    if (addr == id_pedal) {  // DI_torque1
      const float pedal_pct = ((float)msg->data[6]) * 0.4f;  // DI_pedalPos
      gas_pressed = pedal_pct > 3.0f;
    } else if (addr == id_brake) {  // BrakeMessage
      const uint8_t st = (msg->data[0] >> 2) & 0x3U;         // driverBrakeStatus
      brake_pressed = (st == 2U);
    } else if (addr == id_distate) {  // DI_state
      const uint8_t cruise_state = (msg->data[1] >> 4) & 0x0FU;
      const bool cruise_engaged = (cruise_state == 2U) || (cruise_state == 3U);
      vehicle_moving = (cruise_state != 3U);

      if (!tesla_legacy_op_autopilot_disabled) {
        pcm_cruise_check(cruise_engaged);
      }
    } else if (!tesla_legacy_external_panda && (addr == 0x155)) {  // ESP_B (chassis-only vehicle speed)
      const uint16_t raw_kph = (uint16_t)((((uint16_t)msg->data[5]) << 8) | msg->data[4]);
      const float speed_ms = ((float)raw_kph) * 0.01f * (float)KPH_TO_MS;
      UPDATE_VEHICLE_SPEED(speed_ms);
    } else if (tesla_legacy_external_panda && (addr == id_ditrq2)) {  // PT: DI_torque2.DI_vehicleSpeed 16|12 (0.05,-25) MPH
      const uint16_t raw_spd = (uint16_t)((((uint16_t)(msg->data[3] & 0x0FU)) << 8) | msg->data[2]);
      const float speed_mph = ((float)raw_spd) * 0.05f - 25.0f;
      const float speed_ms = (speed_mph > 0.0f) ? (speed_mph * 0.44704f) : 0.0f;
      UPDATE_VEHICLE_SPEED(speed_ms);
    } else {
    }
  }

  // Gear state (DI_torque2 0x280) on chassis / mirrored PT bus
  if (((bus == tesla_legacy_chassis_bus) || (bus == 2)) && (addr == 0x280)) {
    const uint8_t gear = (msg->data[1] >> 4) & 0x07U;
    const bool prev_reverse = tesla_legacy_in_reverse;
    const bool now_reverse = gear == 2U;  // DI_GEAR_R
    const bool now_drive = gear == 4U;    // DI_GEAR_D

    if (now_reverse && !prev_reverse) {
      tesla_legacy_reset_after_gear_change();
    } else if (prev_reverse && now_drive) {
      tesla_legacy_reset_after_gear_change();
    }

    tesla_legacy_in_reverse = now_reverse;
    tesla_legacy_last_gear = gear;
  }

  // EPAS_sysStatus (0x370) on bus0
  if ((bus == 0) && (addr == 0x370)) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = (msg->data[4] >> 6) & 0x03;
    const int eac_status = (msg->data[6] >> 5) & 0x07;
    const int eac_error_code = (msg->data[2] >> 4) & 0x0F;

    tesla_legacy_hands_on = hands_on_level > 0;
    if (tesla_legacy_hands_on) {
      tesla_legacy_hands_on_last_signal = microsecond_timer_get();
    } else {
      const uint32_t dt = get_ts_elapsed(microsecond_timer_get(), tesla_legacy_hands_on_last_signal);
      tesla_legacy_hands_on = dt <= TESLA_LEGACY_TIME_FOR_HANDS_ON_US;
    }

    // Unity parity: do NOT disengage on hands-on escalation; only on EPAS inhibit/error.
    const bool disengage = ((eac_status == 0) && (eac_error_code == 9));
    steering_disengage = disengage;
    if (disengage) {
      controls_allowed = false;
    }
  }

  if ((bus == 2) && (addr == tesla_legacy_das_control_addr)) {
    tesla_legacy_note_aeb_hud_warning((int)(msg->data[2] & 0x03U));
  }

  // Unity stalk gating on 0x45 (bus0)
  if ((bus == 0) && (addr == 0x45) && tesla_legacy_op_stalk_enable) {
    if ((!tesla_legacy_has_ap_hw) || tesla_legacy_op_autopilot_disabled) {
      const int ap_lever_position = (int)(msg->data[0] & 0x3F);
      if (ap_lever_position == 2) {
        pcm_cruise_check(true);
      } else if (ap_lever_position == 1) {
        pcm_cruise_check(false);
      } else {
      }
    }
  }

  // Stock system detection + Unity "disable when stock features active" (bus2)
  if (bus == 2) {
    if (!tesla_legacy_external_panda && (addr == 0x488)) {
      const int steer_control_type = (int)((msg->data[2] >> 6) & 0x03);
      tesla_legacy_stock_lkas = (steer_control_type == 2) || (steer_control_type == 3);
      if (tesla_legacy_stock_lkas) {
        controls_allowed = false;
      }
    } else if (tesla_legacy_external_panda && (addr == tesla_legacy_das_control_addr)) {
      const int aeb_event = (int)(msg->data[2] & 0x03);
      tesla_legacy_stock_aeb = (aeb_event == 1);
      if (tesla_legacy_stock_aeb) {
        controls_allowed = false;
      }
    } else {
    }

    if (!tesla_legacy_external_panda && tesla_legacy_has_ap_hw) {
      if (addr == 0x399) {
        const uint8_t st = msg->data[0] & 0x0FU;  // AutopilotStatus is the LOW nibble on this DBC
        tesla_legacy_autopilot_enabled = (st == 3U) || (st == 4U) || (st == 5U);
        if (tesla_legacy_autopilot_enabled) {
          controls_allowed = false;
        }
      } else if (addr == 0x219) {
        const int eac_status = (int)((msg->data[1] >> 2) & 0x0FU);
        const int psc_status = (int)((msg->data[0] >> 1) & 0x1FU);
        tesla_legacy_eac_enabled = (eac_status == 2) || (eac_status == 3);
        tesla_legacy_autopark_enabled = (psc_status == 14) || ((psc_status >= 1) && (psc_status <= 8));
        if (tesla_legacy_autopilot_enabled || tesla_legacy_eac_enabled || tesla_legacy_autopark_enabled) {
          controls_allowed = false;
        }
      } else {
      }
    }
  }

  tesla_legacy_track_controls_allowed_edge();
}

// --- TX hook ---
static bool tesla_legacy_tx_hook(const CANPacket_t *msg) {
  const int addr = (int)msg->addr;

  // Internal carrier (0x659): consume + block from CAN
  if (addr == 0x659) {
    const uint8_t b5 = msg->data[5];
    tesla_legacy_pedal_enabled = (b5 & 0x20U) != 0U;
    tesla_legacy_op_autopilot_disabled = (b5 & 0x80U) != 0U;
    tesla_legacy_op_stalk_main_edge = (b5 & 0x02U) != 0U;
    tesla_legacy_op_stalk_cancel_edge = (b5 & 0x01U) != 0U;
        if (tesla_legacy_op_stalk_enable && (!tesla_legacy_has_ap_hw || tesla_legacy_op_autopilot_disabled)) {
      if (tesla_legacy_op_stalk_main_edge) {
        pcm_cruise_check(true);
      }
      if (tesla_legacy_op_stalk_cancel_edge) {
        pcm_cruise_check(false);
      }
    }
    // Unity parity for 0x659: Unity blocks the carrier only `if (has_das_hw)`, which is FALSE on AP
    // cars, so on an AP car Unity lets 0x659 onto the CHASSIS bus. Match that on the MAIN (chassis/
    // party = bus 0) panda -> allow it on the wire. Keep blocking it on the EXTERNAL panda, whose
    // bus is the separate powertrain bus 4; Unity's AP1 topology has CAN_POWERTRAIN=chassis(0), so
    // it never puts 0x659 on a dedicated powertrain bus. Bits were already consumed above.
    return !tesla_legacy_external_panda;   // main panda: allow on bus0; external panda: block on bus4
  }

  // Vanilla-xnor HUD ownership: userspace must not transmit competing DAS_status/DAS_status2.
  // Stock AP/DAS frames are forwarded below, with only narrow safety scrubs where needed.
  if ((addr == 0x399) || (addr == 0x389)) {
    return false;
  }

  // Unity parity: on AP hardware cars, block OP actuation unless stock AP is disabled
  if (tesla_legacy_has_ap_hw && !tesla_legacy_op_autopilot_disabled) {
    if ((addr == 0x488) || (addr == 0x27D) || (addr == tesla_legacy_das_control_addr)) {
      return false;
    }
  }

  // Main panda (lateral)
  if (!tesla_legacy_external_panda) {
    if (addr == tesla_legacy_das_control_addr) {  // main: block chassis DAS_control (0x2B9)
      return false;
    }

    if ((addr == 0x488) || (addr == 0x27D)) {
      if (tesla_legacy_stock_lkas) {
        return false;
      }
    }

    if (addr == 0x488) {
      const int steer_control_type = (int)((msg->data[2] >> 6) & 0x03);
      const bool steer_control_enabled = (steer_control_type == 1);

      if ((steer_control_type != 0) && (steer_control_type != 1)) {
        return false;
      }
      if (steer_control_type == 0) {
        return true;
      }

      const int raw_angle_can = (((int)(msg->data[0] & 0x7FU)) << 8) | (int)msg->data[1];
      const int desired_angle = raw_angle_can - 16384;

      if (!controls_allowed) {
        if (!tesla_legacy_hands_on) {
          return false;
        }
        if ((desired_angle < (angle_meas.min - 1)) || (desired_angle > (angle_meas.max + 1))) {
          return false;
        }
        return true;
      }

      const AngleSteeringLimits limits = {
        .max_angle = 3600,
        .angle_deg_to_can = 10.0f,
        .frequency = 50U,
      };

      const AngleSteeringParams params = {
        .slip_factor = -0.000750000000000000f,
        .steer_ratio = 16.5f,
        .wheelbase = 2.96f,
      };

      return !steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, limits, params);
    }

    return true;
  }

  // External panda (longitudinal)
  if (tesla_legacy_external_panda) {
    if (addr != tesla_legacy_das_control_addr) {  // external: only DAS_control (0x2BF) past here
      return false;
    }

    const int aeb_event = (int)(msg->data[2] & 0x03);
    if (aeb_event != 0) {
      return false;
    }
    if (tesla_legacy_stock_aeb) {
      return false;
    }

    const LongitudinalLimits limits = {.max_accel = 425, .min_accel = 288, .inactive_accel = 375};

    const int raw_accel_max = (((int)(msg->data[6] & 0x1FU)) << 4) | ((int)(msg->data[5] >> 4));
    const int raw_accel_min = (((int)(msg->data[5] & 0x0FU)) << 5) | ((int)(msg->data[4] >> 3));

    const bool long_active = (raw_accel_max != limits.inactive_accel) || (raw_accel_min != limits.inactive_accel);
    if (long_active && (!controls_allowed || !get_longitudinal_allowed())) {
      return false;
    }

    if ((raw_accel_max < limits.inactive_accel) && (raw_accel_min < limits.inactive_accel)) {
      return false;
    }

    if (longitudinal_accel_checks(raw_accel_max, limits)) {
      return false;
    }
    if (longitudinal_accel_checks(raw_accel_min, limits)) {
      return false;
    }

    return true;
  }

  return true;
}

// fwd_hook/fwd_msg return true => block forwarding
static bool tesla_legacy_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  return addr == 0x659;
}

static bool tesla_legacy_fwd_msg_hook(int bus_num, CANPacket_t *to_fwd) {
  const int addr = (int)to_fwd->addr;

  // Never forward internal carrier
  if (addr == 0x659) {
    return true;
  }

  // Prevent OP actuation echoes back to AP side.
  // Keep stock DAS_longControl (0x2BF) flowing before OP is actively controlling;
  // blocking that startup heartbeat can trigger transient AEB-unavailable on the IC.
  if ((bus_num == 0) && ((addr == 0x488) || (addr == 0x27D))) {
    return true;
  }
  if ((bus_num == 0) && (addr == tesla_legacy_das_control_addr) && controls_allowed) {
    return true;
  }

  // External panda:
  // - While OP is not actively controlling, pass stock DAS_longControl through unchanged.
  //   The IC expects this stock heartbeat during startup/standby; blocking it can latch
  //   transient "AEB unavailable" warnings before OP has taken ownership.
  // - While OP is actively controlling, keep the previous Unity-style behavior: block
  //   stock longControl except real stock AEB events.
  if (tesla_legacy_external_panda) {
    if ((bus_num == 2) && (addr == tesla_legacy_das_control_addr)) {
      const int aeb_event = (int)(to_fwd->data[2] & 0x03U);
      tesla_legacy_note_aeb_hud_warning(aeb_event);
      if (tesla_legacy_should_scrub_aeb_event(aeb_event)) {
        tesla_legacy_scrub_das_control_aeb(to_fwd);
        return false;
      }
      if (!controls_allowed) {
        return false;
      }
      return !tesla_legacy_stock_aeb;
    }
    // [BUS-128 FIX] The external panda forwards the high-rate, IC-facing copy of DAS_status2
    // (0x389). The main-panda bus2->bus0 scrub never touches it, so the spurious
    // DAS_pmmObstacleSeverity = 6 (PMM_ACCEL_LIMIT) reaches the IC here and flashes the AEB icon.
    // Scrub it on this path too (same mask + checksum as the main-panda scrub). REAL PMM states
    // (1..5) pass unchanged; only the spurious 6 is forced to 0 (PMM_NONE).
    if (addr == 0x389) {
      const int pmm_sev = (int)((to_fwd->data[1] >> 2) & 0x07U);
      if (pmm_sev == 6) {
        to_fwd->data[1] = (uint8_t)(to_fwd->data[1] & 0xE3U);  // pmmObstacleSeverity -> 0
        tesla_legacy_set_last_byte_checksum(to_fwd);
      }
    }
    // Vanilla parity: forward everything else on the external panda. This bridges the powertrain
    // bus (local bus0) to the AP-powertrain bus (local bus2 = global bus6), feeding the AP ~134k
    // frames/run of vehicle dynamics. Blocking it (the old `return true`) starved the AP, which
    // then asserted AEB-unavailable (DAS_control aeb_event=2) ~0.5s after relay-open -> the IC
    // latched that warning. Confirmed by vanilla-vs-dev rlog diff (vanilla src134=~134k, dev=0).
    return false;
  }

  // Main panda: stock LKAS passthrough (bus2 -> car)
  if ((bus_num == 2) && ((addr == 0x488) || (addr == 0x27D))) {
    return !tesla_legacy_stock_lkas;
  }

  // Unity mods only on main panda with AP HW
  if (!tesla_legacy_has_ap_hw) {
        if (tesla_legacy_op_stalk_enable && (!tesla_legacy_has_ap_hw || tesla_legacy_op_autopilot_disabled)) {
      if (tesla_legacy_op_stalk_main_edge) {
        pcm_cruise_check(true);
      }
      if (tesla_legacy_op_stalk_cancel_edge) {
        pcm_cruise_check(false);
      }
    }
    return false;
  }

  // bus0 -> bus2: preserve vanilla forwarding, but keep the Unity EPAS_eacStatus rewrite
  // that prevents stock AP from declaring steering temporarily unavailable while OP steers.
  if (bus_num == 0) {

    if (addr == 0x370) {
      const uint8_t b6 = to_fwd->data[6];
      const uint8_t eac_status = (b6 >> 5) & 0x07U;
      if (controls_allowed && !(tesla_legacy_autopilot_enabled || tesla_legacy_eac_enabled || tesla_legacy_autopark_enabled) && (eac_status == 2U)) {
        to_fwd->data[6] = (uint8_t)((b6 & 0x1FU) | (1U << 5));
        tesla_legacy_set_last_byte_checksum(to_fwd);
      }
    }

    return false;
  }

  // bus2 -> bus0: forward the stock 0x399/0x389 (DAS_status/DAS_status2) UNCHANGED, exactly like
  // vanilla-xnor -- which has NO packet-modifying fwd_msg hook and never flashes on this car. The
  // empirical result (cold11 trace + the vanilla baseline) is that REWRITING these frames is what
  // causes the flash: modifying the warning/status bits and recomputing the checksum makes the IC
  // reject / mis-render the frame and latch the AEB/FCW warning, whereas the authentic AP frame
  // (its own counter + checksum, its own SNA->NONE recovery) is handled correctly and clears. The
  // scrub WAS the bug. We keep ONLY the DAS_control (0x2B9/0x2BF) AEB-event handling here -- that is
  // longitudinal actuation, not the IC HUD, and matches the stock-AEB intent.
  if (bus_num == 2) {
    // OPTION A v2 (blue-D via authentic-frame injection): while engaged (controls_allowed), forward
    // the STOCK DAS_status (0x399) but flip ONLY the autopilotStatus low nibble to 5 -> blue
    // Autopilot 'D'. Every other field (FCW, side-collision, LDW, speed-limit, the high nibble of
    // byte0 ...) keeps its authentic stock value, so NONE of them can default to 0 and render a
    // warning -- that whole-frame rebuild by OP was what flashed the AEB only at engage. When
    // disengaged, forward unchanged. 0x389 is forwarded unchanged in all cases (stock longColl is
    // already SNA/clean). OP transmits NO 0x399/0x389 in this mode (carcontroller HUD stays off),
    // so there is a single coherent source. Additive checksum verified for 0x399 (12/12 frames).
    // NO blue-D: forward stock DAS_status (0x399) and DAS_status2 (0x389) UNCHANGED. Asserting
    // autopilotStatus=5 on this car (autopilot-disabled mode) makes the IC render the AEB/FCW
    // warning because it couples that indicator to the collision HUD -- so we do not touch 0x399.
    // DAS_control (0x2B9) AEB scrub -- suppress the spurious "AEB-unavailable" STATUS only.
    //   - aeb_event == 2 or 3: the AP's spurious "AEB temporarily unavailable" status (set while
    //     OP is engaged and HELD even after disengage). Scrub -> 0 so the IC never renders it,
    //     regardless of controls_allowed. This closes the DISENGAGE leak: the engaged-only gate
    //     stopped scrubbing the instant OP dropped, exposing the AP's still-held aeb=2 until
    //     re-engage. Suppressing 2/3 unconditionally covers engaged, disengaging, and disengaged.
    //   - aeb_event == 1: a REAL stock AEB braking event -- NEVER scrubbed; always reaches the IC
    //     (safety preserved).
    //   - aeb_event == 0: idle -- untouched.
    // Cold-boot safe: at boot the external-panda bridge keeps the AP at aeb=0, so nothing is
    // rewritten before first engagement; and the additive checksum is verified correct for 0x2B9
    // (8/8 real frames), so the scrubbed frame is valid -- the latch was the bridge bug, not this.
    if (addr == tesla_legacy_das_control_addr) {
      const int aeb_event = (int)(to_fwd->data[2] & 0x03U);
      tesla_legacy_note_aeb_hud_warning(aeb_event);
      if ((aeb_event == 2) || (aeb_event == 3)) {
        tesla_legacy_scrub_das_control_aeb(to_fwd);
      }
    }

    // DAS_status2 (0x389): scrub the spurious DAS_pmmObstacleSeverity = 6 (PMM_ACCEL_LIMIT) that the
    // stock AP asserts in autopilot-disabled mode (OP owns longitudinal, the AP's PMM isn't actually
    // limiting anything). The IC renders that PMM state as the AEB/collision icon -- it is THE
    // engage-AEB source in autopilot-disabled mode (found via AP-disabled ON-vs-OFF rlog diff: sev=6
    // on, sev=0 off). Force it to 0 (PMM_NONE). REAL PMM states (1..5: IMMINENT/CRASH/BRAKE_REQUEST)
    // are passed UNCHANGED so a genuine warning still reaches the IC. In normal mode sev is never 6,
    // so this is a no-op there. Additive checksum verified for 0x389 (10/10).
    if (addr == 0x389) {
      const int pmm_sev = (int)((to_fwd->data[1] >> 2) & 0x07U);
      if (pmm_sev == 6) {
        to_fwd->data[1] = (uint8_t)(to_fwd->data[1] & 0xE3U);  // pmmObstacleSeverity -> 0 (PMM_NONE)
        tesla_legacy_set_last_byte_checksum(to_fwd);
      }
    }

    if (!controls_allowed) {
      tesla_legacy_hide_errors_armed = false;
    }
    return false;
  }

  return false;
}

// --- init ---
static safety_config tesla_legacy_init(uint16_t param) {
  tesla_legacy_external_panda = GET_FLAG(param, TESLA_LEGACY_FLAG_EXTERNAL_PANDA);
  tesla_legacy_op_stalk_enable = GET_FLAG(param, TESLA_LEGACY_FLAG_OP_STALK_ENABLE);
  tesla_legacy_has_ap_hw = GET_FLAG(param, TESLA_LEGACY_FLAG_HW2) || GET_FLAG(param, TESLA_LEGACY_FLAG_HW3);
  // chassis_bus: only the MAIN panda on HW3 (raven) reads chassis vehicle-state off bus 1. The
  // external (powertrain) panda always reads its frames off its local bus 0, and HW1/HW2 main read
  // chassis off bus 0. (Vanilla parity: HW3 main -> chassis_bus=1; external or HW2 -> 0.)
  tesla_legacy_chassis_bus = (!tesla_legacy_external_panda && GET_FLAG(param, TESLA_LEGACY_FLAG_HW3)) ? 1 : 0;
  // DAS_control namespace per panda: external(PT)=0x2BF, main(chassis)=0x2B9 (Unity/vanilla parity).
  tesla_legacy_das_control_addr = tesla_legacy_external_panda ? 0x2BF : 0x2B9;

  tesla_legacy_op_autopilot_disabled = false;
  tesla_legacy_pedal_enabled = false;

  tesla_legacy_autopilot_enabled = false;
  tesla_legacy_eac_enabled = false;
  tesla_legacy_autopark_enabled = false;

  tesla_legacy_hands_on = false;
  tesla_legacy_hands_on_last_signal = 0U;

  tesla_legacy_time_op_disengaged = 0U;
  tesla_legacy_controls_allowed_prev = false;
  tesla_legacy_hide_errors_armed = false;
  tesla_legacy_last_aeb_hud_warning_us = 0U;
  tesla_legacy_in_reverse = false;
  tesla_legacy_last_gear = 0U;

  cruise_engaged_prev = false;

  static const CanMsg TESLA_LEGACY_TX_MSGS_LATERAL[] = {
    {0x488, 0, 4, .check_relay = false},  // DAS_steeringControl
    {0x27D, 0, 3, .check_relay = false},  // APS_eacMonitor
    {0x659, 0, 8, .check_relay = false},  // OP->safety internal carrier (blocked in tx_hook)
    {0x45, 0, 8, .check_relay = false},  // STW_ACTN_RQ
    // Vanilla-xnor HUD ownership: do not allow userspace DAS_status/DAS_status2 TX.

  };

  static const CanMsg TESLA_LEGACY_TX_MSGS_LONG[] = {
    {0x2BF, 0, 8, .check_relay = false},  // DAS_longControl
    {0x659, 0, 8, .check_relay = false},  // OP->safety internal carrier (blocked in tx_hook)
    {0x45, 0, 8, .check_relay = false},  // STW_ACTN_RQ
    // Vanilla-xnor HUD ownership: do not allow userspace DAS_status/DAS_status2 TX.

  };

  // RX checks: MINIMAL torque-only set (reverted from the Unity-parity EPAS/brake/DI_state set).
  // The expanded set required EPAS(0x370)/BrakeMessage(0x20A)/DI_state(0x368) on specific buses; if
  // any of those lagged >1s mid-drive on this harness it set safety_rx_checks_invalid -> controls
  // mismatch + steerFaultTemporary mid-drive. Torque frames (0x106/0x108 + 0x116/0x118) are the most
  // reliably-present signals and never tripped, so we validate only those. Each lists bus0 + its
  // bus2 forward-mirror so a frame on either bus satisfies the check.
  static RxCheck tesla_legacy_rx_checks_external[] = {
  {.msg = {
    {0x106, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // DI_torque1 (PT)
    {0x106, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // mirror
    {0},
  }},
  {.msg = {
    {0x116, 0, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // DI_torque2 (PT)
    {0x116, 2, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // mirror
    {0},
  }},
};

  static RxCheck tesla_legacy_rx_checks_main[] = {
  {.msg = {
    {0x108, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // DI_torque1
    {0x108, 2, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // mirror
    {0},
  }},
  {.msg = {
    {0x118, 0, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // DI_torque2
    {0x118, 2, 6, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},  // mirror
    {0},
  }},
};

  safety_config ret = tesla_legacy_external_panda
    ? BUILD_SAFETY_CFG(tesla_legacy_rx_checks_external, TESLA_LEGACY_TX_MSGS_LONG)
    : BUILD_SAFETY_CFG(tesla_legacy_rx_checks_main, TESLA_LEGACY_TX_MSGS_LATERAL);

  // The external panda MUST forward (vanilla parity): it bridges the powertrain bus to the AP's
  // powertrain interface (bus6), keeping the AP fed with vehicle dynamics so it does not assert
  // AEB-unavailable. Disabling it was the cold-boot IC-flash root cause. Only disable forwarding
  // when there is no AP hardware at all.
  ret.disable_forwarding = !tesla_legacy_has_ap_hw;
  return ret;
}

const safety_hooks tesla_legacy_hooks = {
  .init = tesla_legacy_init,
  .rx = tesla_legacy_rx_hook,
  .tx = tesla_legacy_tx_hook,
  .fwd = tesla_legacy_fwd_hook,
  .fwd_msg = tesla_legacy_fwd_msg_hook,
};
