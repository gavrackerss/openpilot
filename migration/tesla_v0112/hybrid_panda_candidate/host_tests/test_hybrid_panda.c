// XNOR v0.11.2 full Hybrid Panda: host-only compiled C safety smoke.
// Compiled against disposable, patched opendbc source, NOT firmware.
// No car, sockets, CAN interface, Panda hardware, or device I/O.
#include <assert.h>
#include <stdio.h>
#include <string.h>

// Include the same C safety test entry used by native opendbc. Its synthetic
// microsecond clock and helper APIs make these tests deterministic.
#include "opendbc/safety/tests/libsafety/safety.c"

#define TESLA_MAIN_HW2_FLAGS (0x01U | 0x10U | 0x40U)
#define TESLA_EXTERNAL_HW2_FLAGS (TESLA_MAIN_HW2_FLAGS | 0x04U)

static CANPacket_t packet(int addr, int bus) {
  CANPacket_t p = {0};
  p.addr = (uint32_t)addr;
  p.bus = (uint8_t)bus;
  p.data_len_code = 8U;
  return p;
}

// Canonical Tesla DAS_control raw fields (from native Tesla DBC):
// setSpeed bits 0..11, accState 12..15, aebEvent 16..17,
// accelMin 35..43, accelMax 44..52, counter 53..55.
static CANPacket_t das_packet(int addr, int bus, int speed, int aeb, int counter) {
  CANPacket_t p = packet(addr, bus);
  p.data[0] = (uint8_t)(speed & 0xFF);
  p.data[1] = (uint8_t)(((speed >> 8) & 0x0F) | 0x40U);
  p.data[2] = (uint8_t)(aeb & 0x03);
  // raw 375 == 0.00 m/s^2, valid neutral envelope.
  const int raw_accel = 375;
  p.data[4] = (uint8_t)((raw_accel & 0x1F) << 3);
  p.data[5] = (uint8_t)(((raw_accel >> 5) & 0x0F) | ((raw_accel & 0x0F) << 4));
  p.data[6] = (uint8_t)(((raw_accel >> 4) & 0x1F) | ((counter & 0x07) << 5));
  return p;
}

// Build a valid former V229 disengaged-prime request: setSpeed=0,
// accel envelope [-1.4,+1.8], jerk in the old narrow priming band.
// The candidate must reject it when longitudinal authority is absent.
static CANPacket_t unsafe_old_priming_template(void) {
  CANPacket_t p = das_packet(0x2BF, 0, 0, 0, 1);
  const int raw_min = 340;
  const int raw_max = 420;
  const int jerk_min = 492;
  const int jerk_max = 8;
  p.data[2] = (uint8_t)((jerk_min & 0x3F) << 2);
  p.data[3] = (uint8_t)(((jerk_min >> 6) & 0x07) | ((jerk_max & 0x1F) << 3));
  p.data[4] = (uint8_t)(((jerk_max >> 5) & 0x07) | ((raw_min & 0x1F) << 3));
  p.data[5] = (uint8_t)(((raw_min >> 5) & 0x0F) | ((raw_max & 0x0F) << 4));
  p.data[6] = (uint8_t)(((raw_max >> 4) & 0x1F) | ((1U & 0x07U) << 5));
  return p;
}

static void arm_external_owner(void) {
  assert(set_safety_hooks(SAFETY_TESLA_LEGACY, TESLA_EXTERNAL_HW2_FLAGS) == 0);
  init_tests();
  timer_cnt = 10000U;
  set_controls_allowed(true);
  CANPacket_t carrier = packet(0x659, 0);
  carrier.data[5] = 0x48U; // Hybrid + V229 selected, no physical main edge.
  assert(!safety_tx_hook(&carrier)); // consumed internally, not transmitted.
}

static void test_external_owner_and_genuine_aeb(void) {
  arm_external_owner();
  CANPacket_t carrier = packet(0x659, 0);
  assert(safety_fwd_hook_pkt(0, &carrier) == -1);
  CANPacket_t wrong_bus = packet(0x2B9, 0);
  assert(!safety_tx_hook(&wrong_bus));

  CANPacket_t genuine_aeb = das_packet(0x2BF, 2, 42, 1, 3);
  const CANPacket_t copy_aeb = genuine_aeb;
  assert(safety_fwd_hook_pkt(2, &genuine_aeb) == 0);
  assert(memcmp(genuine_aeb.data, copy_aeb.data, 8) == 0);
  // Real stock intervention revokes the owner, regardless of HUD setting.
  (void)safety_rx_hook(&genuine_aeb);
  assert(!controls_allowed);
}

static void test_v229_native_counter_preserved_when_forward_rewritten(void) {
  arm_external_owner();
  CANPacket_t templ = das_packet(0x2BF, 0, 55, 0, 0);
  // In this mode a valid template is consumed; it is never a direct sender.
  assert(!safety_tx_hook(&templ));

  CANPacket_t stock = das_packet(0x2BF, 2, 30, 0, 5);
  const uint8_t stock_counter = (uint8_t)(stock.data[6] & 0xE0U);
  assert(safety_fwd_hook_pkt(2, &stock) == 0);
  // On eligible owner, the OP set-speed replaces genuine AP set-speed,
  // but the genuine rolling counter survives the substitution.
  const int new_speed = stock.data[0] | ((stock.data[1] & 0x0FU) << 8);
  assert(new_speed == 55);
  assert((stock.data[6] & 0xE0U) == stock_counter);
  assert((stock.data[2] & 0x03U) == 0U);
}

static void test_unqualified_or_stale_owner_forwards_native(void) {
  arm_external_owner();
  CANPacket_t templ = das_packet(0x2BF, 0, 55, 0, 0);
  assert(!safety_tx_hook(&templ));
  timer_cnt = 2000000U; // Owner and template validity have expired.
  CANPacket_t stock = das_packet(0x2BF, 2, 30, 0, 4);
  const CANPacket_t baseline = stock;
  assert(safety_fwd_hook_pkt(2, &stock) == 0);
  assert(memcmp(stock.data, baseline.data, 8) == 0);
}

static void test_main_panda_does_not_send_unsolicited_chassis_long(void) {
  assert(set_safety_hooks(SAFETY_TESLA_LEGACY, TESLA_MAIN_HW2_FLAGS) == 0);
  init_tests();
  timer_cnt = 10000U;
  CANPacket_t chassis = das_packet(0x2B9, 0, 30, 0, 0);
  assert(!safety_tx_hook(&chassis)); // controls disabled
  CANPacket_t carrier = packet(0x659, 0);
  assert(!safety_tx_hook(&carrier)); // cannot get onto vehicle CAN
  assert(safety_fwd_hook_pkt(0, &carrier) == -1);
}

static void test_disengaged_priming_never_rewrites_a_genuine_ap_frame(void) {
  arm_external_owner();
  set_controls_allowed(false);
  CANPacket_t prime = unsafe_old_priming_template();
  assert(!safety_tx_hook(&prime));  // consumed as a template, never wire TX
  assert(!tesla_legacy_v229_tmpl_valid);  // template MUST be invalid
  CANPacket_t native = das_packet(0x2BF, 2, 30, 0, 5);
  CANPacket_t original = native;
  assert(safety_fwd_hook_pkt(2, &native) == 0);
  assert(memcmp(native.data, original.data, 8) == 0);
}

static void test_captured_template_revoked_before_stock_frame_is_not_applied(void) {
  arm_external_owner();
  CANPacket_t tmpl = das_packet(0x2BF, 0, 55, 0, 1);
  assert(!safety_tx_hook(&tmpl));
  assert(tesla_legacy_v229_tmpl_valid);
  set_controls_allowed(false);  // permission can change between TX and AP RX
  CANPacket_t genuine = das_packet(0x2BF, 2, 30, 0, 4);
  CANPacket_t original = genuine;
  assert(safety_fwd_hook_pkt(2, &genuine) == 0);
  assert(memcmp(genuine.data, original.data, 8) == 0);
}

int main(void) {
  test_external_owner_and_genuine_aeb();
  test_v229_native_counter_preserved_when_forward_rewritten();
  test_unqualified_or_stale_owner_forwards_native();
  test_main_panda_does_not_send_unsolicited_chassis_long();
  test_disengaged_priming_never_rewrites_a_genuine_ap_frame();
  test_captured_template_revoked_before_stock_frame_is_not_applied();
  puts("XNOR v0.11.2 Tesla Hybrid Panda host smoke: PASS");
  return 0;
}
