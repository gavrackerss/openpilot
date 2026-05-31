#include "board/drivers/drivers.h"

#define XNOR_V137_CAN_DRIVER_WARNING_RX_SCRUB 1
#define XNOR_V137_CAN_DRIVER_WARNING_RX_SCRUB_US 180000000U
__attribute__((used)) static const char xnor_v137_fw_marker[] = "XNOR_V137_CAN_DRIVER_WARNING_RX_SCRUB";

static bool xnor_v137_startup_safety_mode(void) {
  return (current_safety_mode == SAFETY_SILENT) ||
         (current_safety_mode == SAFETY_NOOUTPUT) ||
         (current_safety_mode == SAFETY_ELM327);
}

static bool xnor_v137_warning_quarantine_active(void) {
  return xnor_v137_startup_safety_mode() ||
         (microsecond_timer_get() < XNOR_V137_CAN_DRIVER_WARNING_RX_SCRUB_US);
}

static bool xnor_v137_tesla_warning_addr(uint32_t addr) {
  return (addr == 0x2BFU) ||  // DAS_control / AEB event
         (addr == 0x389U) ||  // DAS_status2 / long collision HUD
         (addr == 0x399U);    // AutopilotStatus / FCW HUD
}

static bool xnor_v137_block_startup_forwarding(uint8_t bus_number, int forward_bus, const CANPacket_t *msg) {
  (void)bus_number;
  return xnor_v137_warning_quarantine_active() &&
         (forward_bus == 0) &&
         xnor_v137_tesla_warning_addr(msg->addr);
}

static uint8_t xnor_v137_tesla_checksum_addr(uint32_t addr) {
  const uint32_t checksum_addr = (addr == 0x2BFU) ? 0x2B9U : addr;
  return (uint8_t)(checksum_addr & 0xFFU) + (uint8_t)((checksum_addr >> 8) & 0xFFU);
}

static uint8_t xnor_v137_tesla_calc_checksum8(const CANPacket_t *msg, uint8_t len) {
  uint8_t checksum = xnor_v137_tesla_checksum_addr(msg->addr);
  for (uint8_t i = 0U; i < (len - 1U); i++) {
    checksum = (uint8_t)(checksum + msg->data[i]);
  }
  return checksum;
}

static void xnor_v137_tesla_set_last_byte_checksum(CANPacket_t *msg) {
  const uint8_t len = dlc_to_len[msg->data_len_code];
  if (len > 0U) {
    msg->data[len - 1U] = xnor_v137_tesla_calc_checksum8(msg, len);
  }
}

static void xnor_v137_scrub_tesla_status2_warnings(CANPacket_t *msg) {
  uint32_t w0 = ((uint32_t)msg->data[0]) |
                (((uint32_t)msg->data[1]) << 8) |
                (((uint32_t)msg->data[2]) << 16) |
                (((uint32_t)msg->data[3]) << 24);
  uint32_t w1 = ((uint32_t)msg->data[4]) |
                (((uint32_t)msg->data[5]) << 8) |
                (((uint32_t)msg->data[6]) << 16) |
                (((uint32_t)msg->data[7]) << 24);

  w0 &= ~((0x7U << 10) | (1U << 13) | (0x3U << 14) |
          (0x7U << 16) | (0x3U << 19) | (0x7U << 21) |
          (0x3U << 24) | (0x1FU << 26) | (1U << 31));
  w1 &= ~((0xFFU << 0) | (0xFU << 16) | (0xFFU << 24));

  w0 |= (1U << 26);
  w0 |= (1U << 31);
  w1 |= 0x03U;
  w1 |= (1U << 2);
  w1 |= (2U << 4);
  w1 |= (0x0FU << 16);

  msg->data[0] = (uint8_t)(w0 & 0xFFU);
  msg->data[1] = (uint8_t)((w0 >> 8) & 0xFFU);
  msg->data[2] = (uint8_t)((w0 >> 16) & 0xFFU);
  msg->data[3] = (uint8_t)((w0 >> 24) & 0xFFU);
  msg->data[4] = (uint8_t)(w1 & 0xFFU);
  msg->data[5] = (uint8_t)((w1 >> 8) & 0xFFU);
  msg->data[6] = (uint8_t)((w1 >> 16) & 0xFFU);
  msg->data[7] = (uint8_t)((w1 >> 24) & 0xFFU);
  xnor_v137_tesla_set_last_byte_checksum(msg);
}

static void xnor_v137_scrub_tesla_status_warnings(CANPacket_t *msg) {
  msg->data[2] &= 0x3FU;
  msg->data[3] &= 0x3FU;
  msg->data[4] = 0U;
  xnor_v137_tesla_set_last_byte_checksum(msg);
}

static void xnor_v137_scrub_tesla_das_control_aeb(CANPacket_t *msg) {
  msg->data[2] &= 0xFCU;
  xnor_v137_tesla_set_last_byte_checksum(msg);
}

static void xnor_v137_scrub_startup_rx(CANPacket_t *msg) {
  if (!xnor_v137_warning_quarantine_active()) {
    return;
  }

  if ((msg->addr != 0x2BFU) && (msg->addr != 0x389U) && (msg->addr != 0x399U)) {
    return;
  }

  const uint8_t len = dlc_to_len[msg->data_len_code];
  if (len != 8U) {
    return;
  }

  if (msg->addr == 0x2BFU) {
    xnor_v137_scrub_tesla_das_control_aeb(msg);
  } else if (msg->addr == 0x389U) {
    xnor_v137_scrub_tesla_status2_warnings(msg);
  } else if (msg->addr == 0x399U) {
    xnor_v137_scrub_tesla_status_warnings(msg);
  } else {
  }

  can_set_checksum(msg);
}

FDCAN_GlobalTypeDef *cans[PANDA_CAN_CNT] = {FDCAN1, FDCAN2, FDCAN3};

static bool can_set_speed(uint8_t can_number) {
  bool ret = true;
  FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  ret &= llcan_set_speed(
    FDCANx,
    bus_config[bus_number].can_speed,
    bus_config[bus_number].can_data_speed,
    bus_config[bus_number].canfd_non_iso,
    can_loopback,
    can_silent
  );
  return ret;
}

void can_clear_send(FDCAN_GlobalTypeDef *FDCANx, uint8_t can_number) {
  static uint32_t last_reset = 0U;
  uint32_t time = microsecond_timer_get();

  // Resetting CAN core is a slow blocking operation, limit frequency
  if (get_ts_elapsed(time, last_reset) > 100000U) {  // 10 Hz
    can_health[can_number].can_core_reset_cnt += 1U;
    can_health[can_number].total_tx_lost_cnt += (FDCAN_TX_FIFO_EL_CNT - (FDCANx->TXFQS & FDCAN_TXFQS_TFFL)); // TX FIFO msgs will be lost after reset
    llcan_clear_send(FDCANx);
    last_reset = time;
  }
}

void update_can_health_pkt(uint8_t can_number, uint32_t ir_reg) {
  uint8_t can_irq_number[PANDA_CAN_CNT][2] = {
    { FDCAN1_IT0_IRQn, FDCAN1_IT1_IRQn },
    { FDCAN2_IT0_IRQn, FDCAN2_IT1_IRQn },
    { FDCAN3_IT0_IRQn, FDCAN3_IT1_IRQn },
  };

  FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
  uint32_t psr_reg = FDCANx->PSR;
  uint32_t ecr_reg = FDCANx->ECR;

  can_health[can_number].bus_off = ((psr_reg & FDCAN_PSR_BO) >> FDCAN_PSR_BO_Pos);
  can_health[can_number].bus_off_cnt += can_health[can_number].bus_off;
  can_health[can_number].error_warning = ((psr_reg & FDCAN_PSR_EW) >> FDCAN_PSR_EW_Pos);
  can_health[can_number].error_passive = ((psr_reg & FDCAN_PSR_EP) >> FDCAN_PSR_EP_Pos);

  can_health[can_number].last_error = ((psr_reg & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos);
  if ((can_health[can_number].last_error != 0U) && (can_health[can_number].last_error != 7U)) {
    can_health[can_number].last_stored_error = can_health[can_number].last_error;
  }

  can_health[can_number].last_data_error = ((psr_reg & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos);
  if ((can_health[can_number].last_data_error != 0U) && (can_health[can_number].last_data_error != 7U)) {
    can_health[can_number].last_data_stored_error = can_health[can_number].last_data_error;
  }

  can_health[can_number].receive_error_cnt = ((ecr_reg & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);
  can_health[can_number].transmit_error_cnt = ((ecr_reg & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);

  can_health[can_number].irq0_call_rate = interrupts[can_irq_number[can_number][0]].call_rate;
  can_health[can_number].irq1_call_rate = interrupts[can_irq_number[can_number][1]].call_rate;

  if (ir_reg != 0U) {
    // Clear error interrupts
    FDCANx->IR |= (FDCAN_IR_PED | FDCAN_IR_PEA | FDCAN_IR_EP | FDCAN_IR_BO | FDCAN_IR_RF0L);
    can_health[can_number].total_error_cnt += 1U;
    // Check for RX FIFO overflow
    if ((ir_reg & (FDCAN_IR_RF0L)) != 0U) {
      can_health[can_number].total_rx_lost_cnt += 1U;
    }
    // Cases:
    // 1. while multiplexing between buses 1 and 3 we are getting ACK errors that overwhelm CAN core, by resetting it recovers faster
    // 2. H7 gets stuck in bus off recovery state indefinitely
    if ((((can_health[can_number].last_error == CAN_ACK_ERROR) || (can_health[can_number].last_data_error == CAN_ACK_ERROR)) && (can_health[can_number].transmit_error_cnt > 127U)) ||
     ((ir_reg & FDCAN_IR_BO) != 0U)) {
      can_clear_send(FDCANx, can_number);
    }
  }
}

// ***************************** CAN *****************************
// FDFDCANx_IT1 IRQ Handler (TX)
void process_can(uint8_t can_number) {
  if (can_number != 0xffU) {
    ENTER_CRITICAL();

    FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
    uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

    FDCANx->IR |= FDCAN_IR_TFE; // Clear Tx FIFO Empty flag

    if ((FDCANx->TXFQS & FDCAN_TXFQS_TFQF) == 0U) {
      CANPacket_t to_send;
      if (can_pop(can_queues[bus_number], &to_send)) {
        if (can_check_checksum(&to_send)) {
          can_health[can_number].total_tx_cnt += 1U;

          uint32_t TxFIFOSA = FDCAN_START_ADDRESS + (can_number * FDCAN_OFFSET) + (FDCAN_RX_FIFO_0_EL_CNT * FDCAN_RX_FIFO_0_EL_SIZE);
          // get the index of the next TX FIFO element (0 to FDCAN_TX_FIFO_EL_CNT - 1)
          uint32_t tx_index = (FDCANx->TXFQS >> FDCAN_TXFQS_TFQPI_Pos) & 0x1FU;
          // only send if we have received a packet
          canfd_fifo *fifo;
          fifo = (canfd_fifo *)(TxFIFOSA + (tx_index * FDCAN_TX_FIFO_EL_SIZE));

          fifo->header[0] = (to_send.extended << 30) | ((to_send.extended != 0U) ? (to_send.addr) : (to_send.addr << 18));

          // If canfd_auto is set, outgoing packets will be automatically sent as CAN-FD if an incoming CAN-FD packet was seen
          bool fd = bus_config[can_number].canfd_auto ? bus_config[can_number].canfd_enabled : (bool)(to_send.fd > 0U);
          uint32_t canfd_enabled_header = fd ? (1UL << 21) : 0UL;

          uint32_t brs_enabled_header = bus_config[can_number].brs_enabled ? (1UL << 20) : 0UL;
          fifo->header[1] = (to_send.data_len_code << 16) | canfd_enabled_header | brs_enabled_header;

          uint8_t data_len_w = (dlc_to_len[to_send.data_len_code] / 4U);
          data_len_w += ((dlc_to_len[to_send.data_len_code] % 4U) > 0U) ? 1U : 0U;
          for (unsigned int i = 0; i < data_len_w; i++) {
            BYTE_ARRAY_TO_WORD(fifo->data_word[i], &to_send.data[i*4U]);
          }

          FDCANx->TXBAR = (1UL << tx_index);

          // Send back to USB
          CANPacket_t to_push;

          to_push.fd = fd;
          to_push.returned = 1U;
          to_push.rejected = 0U;
          to_push.extended = to_send.extended;
          to_push.addr = to_send.addr;
          to_push.bus = bus_number;
          to_push.data_len_code = to_send.data_len_code;
          (void)memcpy(to_push.data, to_send.data, dlc_to_len[to_push.data_len_code]);
          can_set_checksum(&to_push);

          rx_buffer_overflow += can_push(&can_rx_q, &to_push) ? 0U : 1U;
        } else {
          can_health[can_number].total_tx_checksum_error_cnt += 1U;
        }

        refresh_can_tx_slots_available();
      }
    }
    EXIT_CRITICAL();
  }
}

// FDFDCANx_IT0 IRQ Handler (RX and errors)
// blink blue when we are receiving CAN messages
void can_rx(uint8_t can_number) {
  FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
  uint8_t bus_number = BUS_NUM_FROM_CAN_NUM(can_number);

  uint32_t ir_reg = FDCANx->IR;

  // Clear all new messages from Rx FIFO 0
  FDCANx->IR |= FDCAN_IR_RF0N;
  while ((FDCANx->RXF0S & FDCAN_RXF0S_F0FL) != 0U) {
    can_health[can_number].total_rx_cnt += 1U;
    // get the index of the next RX FIFO element (0 to FDCAN_RX_FIFO_0_EL_CNT - 1)
    uint32_t rx_fifo_idx = (uint8_t)((FDCANx->RXF0S >> FDCAN_RXF0S_F0GI_Pos) & 0x3FU);

    // Recommended to offset get index by at least +1 if RX FIFO is in overwrite mode and full (datasheet)
    if ((FDCANx->RXF0S & FDCAN_RXF0S_F0F) == FDCAN_RXF0S_F0F) {
      rx_fifo_idx = ((rx_fifo_idx + 1U) >= FDCAN_RX_FIFO_0_EL_CNT) ? 0U : (rx_fifo_idx + 1U);
      can_health[can_number].total_rx_lost_cnt += 1U; // At least one message was lost
    }

    uint32_t RxFIFO0SA = FDCAN_START_ADDRESS + (can_number * FDCAN_OFFSET);
    CANPacket_t to_push;
    canfd_fifo *fifo;

    // getting address
    fifo = (canfd_fifo *)(RxFIFO0SA + (rx_fifo_idx * FDCAN_RX_FIFO_0_EL_SIZE));

    bool canfd_frame = ((fifo->header[1] >> 21) & 0x1U);
    bool brs_frame = ((fifo->header[1] >> 20) & 0x1U);

    to_push.fd = canfd_frame;
    to_push.returned = 0U;
    to_push.rejected = 0U;
    to_push.extended = (fifo->header[0] >> 30) & 0x1U;
    to_push.addr = ((to_push.extended != 0U) ? (fifo->header[0] & 0x1FFFFFFFU) : ((fifo->header[0] >> 18) & 0x7FFU));
    to_push.bus = bus_number;
    to_push.data_len_code = ((fifo->header[1] >> 16) & 0xFU);

    uint8_t data_len_w = (dlc_to_len[to_push.data_len_code] / 4U);
    data_len_w += ((dlc_to_len[to_push.data_len_code] % 4U) > 0U) ? 1U : 0U;
    for (unsigned int i = 0; i < data_len_w; i++) {
      WORD_TO_BYTE_ARRAY(&to_push.data[i*4U], fifo->data_word[i]);
    }
        can_set_checksum(&to_push);

    xnor_v137_scrub_startup_rx(&to_push);

    // forwarding (panda only)
    CANPacket_t to_send = to_push;
    to_send.returned = 0U;
    to_send.rejected = 0U;
    int bus_fwd_num = safety_fwd_hook(bus_number, &to_send);
    const bool xnor_v137_block_hook_fwd = xnor_v137_block_startup_forwarding(bus_number, bus_fwd_num, &to_send);
    if (xnor_v137_block_hook_fwd) {
      bus_fwd_num = -1;
    } else if (bus_fwd_num < 0) {
      const int fallback_bus = bus_config[can_number].forwarding_bus;
      if (!xnor_v137_block_startup_forwarding(bus_number, fallback_bus, &to_push)) {
        bus_fwd_num = fallback_bus;
        to_send = to_push;
        to_send.returned = 0U;
        to_send.rejected = 0U;
      }
    }
    if (bus_fwd_num != -1) {
      to_send.bus = (uint8_t)bus_fwd_num;
      can_set_checksum(&to_send);

      can_send(&to_send, bus_fwd_num, true);
      can_health[can_number].total_fwd_cnt += 1U;
    }

    #ifdef PANDA_BODY
    body_can_rx(&to_push);
    #endif

    safety_rx_invalid += safety_rx_hook(&to_push) ? 0U : 1U;
    ignition_can_hook(&to_push);

    led_set(LED_BLUE, true);
    rx_buffer_overflow += can_push(&can_rx_q, &to_push) ? 0U : 1U;

    // Enable CAN FD and BRS if CAN FD message was received
    if (!(bus_config[can_number].canfd_enabled) && (canfd_frame)) {
      bus_config[can_number].canfd_enabled = true;
    }
    if (!(bus_config[can_number].brs_enabled) && (brs_frame)) {
      bus_config[can_number].brs_enabled = true;
    }

    // update read index
    FDCANx->RXF0A = rx_fifo_idx;
  }

  // Error handling
  if ((ir_reg & (FDCAN_IR_PED | FDCAN_IR_PEA | FDCAN_IR_EP | FDCAN_IR_BO | FDCAN_IR_RF0L)) != 0U) {
    update_can_health_pkt(can_number, ir_reg);
  }
}

static void FDCAN1_IT0_IRQ_Handler(void) { can_rx(0); }
static void FDCAN1_IT1_IRQ_Handler(void) { process_can(0); }

static void FDCAN2_IT0_IRQ_Handler(void) { can_rx(1); }
static void FDCAN2_IT1_IRQ_Handler(void) { process_can(1); }

static void FDCAN3_IT0_IRQ_Handler(void) { can_rx(2);  }
static void FDCAN3_IT1_IRQ_Handler(void) { process_can(2); }

bool can_init(uint8_t can_number) {
  bool ret = false;

  REGISTER_INTERRUPT(FDCAN1_IT0_IRQn, FDCAN1_IT0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(FDCAN1_IT1_IRQn, FDCAN1_IT1_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(FDCAN2_IT0_IRQn, FDCAN2_IT0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(FDCAN2_IT1_IRQn, FDCAN2_IT1_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(FDCAN3_IT0_IRQn, FDCAN3_IT0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(FDCAN3_IT1_IRQn, FDCAN3_IT1_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

  if (can_number != 0xffU) {
    FDCAN_GlobalTypeDef *FDCANx = CANIF_FROM_CAN_NUM(can_number);
    ret &= can_set_speed(can_number);
    ret &= llcan_init(FDCANx);
    // in case there are queued up messages
    process_can(can_number);
  }
  return ret;
}
