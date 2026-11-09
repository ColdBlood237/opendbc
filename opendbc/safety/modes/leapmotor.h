#pragma once

#include "opendbc/safety/declarations.h"

// DRAFT for the 2025 global-market Leapmotor B10. Not yet validated on a car:
// it assumes the front camera sends ADAS_LKA (0x83), which is still to be confirmed.

#define LEAPMOTOR_ADAS_LKA          0x83U   // TX from openpilot (stock: front camera), steering angle request
#define LEAPMOTOR_VEHICLE_STATE     0xB6U   // RX from VCU, accelerator pedal, brake pressed, gear
#define LEAPMOTOR_POWERTRAIN_STATE  0xB7U   // RX from VCU, stock ACC/LCC engaged
#define LEAPMOTOR_WHEEL_SPEEDS      0x184U  // RX from ESC, wheel speeds
#define LEAPMOTOR_EPS_STEERING      0x18DU  // RX from EPS, steering angle

#define LEAPMOTOR_MAIN_BUS 0U
#define LEAPMOTOR_CAM_BUS  2U

// Steering angle is 0.02 deg per bit with the centre at raw 32490 (offset -649.8 deg)
#define LEAPMOTOR_ANGLE_CENTER 32490

static uint8_t leapmotor_crc8_lut_8h2f[256];

typedef struct {
  uint32_t addr;
  uint8_t constants[16];
} LeapmotorE2EConstants;

static uint32_t leapmotor_get_checksum(const CANPacket_t *msg) {
  return (uint8_t)msg->data[0];
}

static uint8_t leapmotor_get_counter(const CANPacket_t *msg) {
  return (uint8_t)msg->data[1] & 0xFU;
}

// AUTOSAR E2E Profile 2, as volkswagen_mqb_meb_compute_crc: CRC-8H2F over bytes 1..end, then a
// per-message constant chosen by the counter. Constants learned from logs; exact on held-out routes.
static uint32_t leapmotor_compute_checksum(const CANPacket_t *msg) {
  static const LeapmotorE2EConstants E2E[] = {
    {LEAPMOTOR_WHEEL_SPEEDS, {0xD9U, 0x90U, 0x33U, 0xE1U, 0x3AU, 0x1AU, 0xDFU, 0xC6U, 0xB5U, 0xDCU, 0xEEU, 0x04U, 0xFAU, 0xD7U, 0xD4U, 0x99U}},
    {LEAPMOTOR_EPS_STEERING, {0x61U, 0x4BU, 0x17U, 0xE4U, 0x78U, 0x0BU, 0xA1U, 0xB4U, 0xBCU, 0xD8U, 0xEAU, 0xF5U, 0xD9U, 0x21U, 0xCAU, 0x86U}},
    {LEAPMOTOR_VEHICLE_STATE, {0x71U, 0x0DU, 0x0BU, 0xC2U, 0x08U, 0xC6U, 0x63U, 0x4BU, 0x3FU, 0x2BU, 0x06U, 0x10U, 0xF0U, 0xD2U, 0x87U, 0x0DU}},
    {LEAPMOTOR_POWERTRAIN_STATE, {0xEBU, 0x4EU, 0xAEU, 0xCEU, 0x79U, 0x7CU, 0x52U, 0x03U, 0x6AU, 0x84U, 0xC2U, 0x3AU, 0x0AU, 0x64U, 0x2FU, 0x17U}},
  };

  uint8_t constant = 0U;
  for (size_t i = 0U; i < (sizeof(E2E) / sizeof(E2E[0])); i++) {
    constant = (E2E[i].addr == msg->addr) ? E2E[i].constants[leapmotor_get_counter(msg)] : constant;
  }

  int len = GET_LEN(msg);
  uint8_t crc = 0xFFU;
  for (int i = 1; i < len; i++) {
    crc = leapmotor_crc8_lut_8h2f[crc ^ (uint8_t)msg->data[i]];
  }
  crc = leapmotor_crc8_lut_8h2f[crc ^ constant];
  return (uint8_t)(crc ^ 0xFFU);
}

static void leapmotor_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == LEAPMOTOR_MAIN_BUS) {
    if (msg->addr == LEAPMOTOR_WHEEL_SPEEDS) {
      // four 16-bit little-endian wheel speeds at 0.01 km/h: FL, FR, RL, RR
      uint32_t sum = GET_BYTES(msg, 2, 2) + GET_BYTES(msg, 4, 2) + GET_BYTES(msg, 6, 2) + GET_BYTES(msg, 8, 2);
      vehicle_moving = sum > 0U;
      UPDATE_VEHICLE_SPEED((sum / 4.0) * 0.01 * KPH_TO_MS);
    }

    if (msg->addr == LEAPMOTOR_EPS_STEERING) {
      int angle_meas_new = (int)GET_BYTES(msg, 3, 2) - LEAPMOTOR_ANGLE_CENTER;
      update_sample(&angle_meas, angle_meas_new);
    }

    if (msg->addr == LEAPMOTOR_VEHICLE_STATE) {
      gas_pressed = msg->data[11] > 0U;           // ACCEL_PEDAL
      brake_pressed = GET_BIT(msg, 99U);          // BRAKE_PRESSED
    }

    if (msg->addr == LEAPMOTOR_POWERTRAIN_STATE) {
      pcm_cruise_check(GET_BIT(msg, 30U));        // ASSIST_ENGAGED
    }
  }
}

static bool leapmotor_tx_hook(const CANPacket_t *msg) {
  static const AngleSteeringLimits LEAPMOTOR_STEERING_LIMITS = {
    .max_angle = 20000,       // 400 deg at 50 per deg; lock is about 520 deg
    .angle_deg_to_can = 50,
    .angle_rate_up_lookup = {
      {0., 5., 25.},
      {1.0, .6, .08},
    },
    .angle_rate_down_lookup = {
      {0., 5., 25.},
      {2.0, .8, .12},
    },
  };

  bool tx = true;
  if (msg->addr == LEAPMOTOR_ADAS_LKA) {
    int desired_angle = (int)GET_BYTES(msg, 13, 2) - LEAPMOTOR_ANGLE_CENTER;  // STEER_ANGLE_REQ
    bool lka_active = GET_BIT(msg, 99U);                                         // LCC_STEERING_ACTIVE_COPY
    if (steer_angle_cmd_checks(desired_angle, lka_active, LEAPMOTOR_STEERING_LIMITS)) {
      tx = false;
    }
  }
  return tx;
}

static safety_config leapmotor_init(uint16_t param) {
  SAFETY_UNUSED(param);
  gen_crc_lookup_table_8(0x2F, leapmotor_crc8_lut_8h2f);

  static const CanMsg LEAPMOTOR_TX_MSGS[] = {
    {LEAPMOTOR_ADAS_LKA, LEAPMOTOR_MAIN_BUS, 32, .check_relay = true},
  };

  static RxCheck leapmotor_rx_checks[] = {
    {.msg = {{LEAPMOTOR_WHEEL_SPEEDS, LEAPMOTOR_MAIN_BUS, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_EPS_STEERING, LEAPMOTOR_MAIN_BUS, 32, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_VEHICLE_STATE, LEAPMOTOR_MAIN_BUS, 16, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{LEAPMOTOR_POWERTRAIN_STATE, LEAPMOTOR_MAIN_BUS, 32, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  return BUILD_SAFETY_CFG(leapmotor_rx_checks, LEAPMOTOR_TX_MSGS);
}

const safety_hooks leapmotor_hooks = {
  .init = leapmotor_init,
  .rx = leapmotor_rx_hook,
  .tx = leapmotor_tx_hook,
  .get_counter = leapmotor_get_counter,
  .get_checksum = leapmotor_get_checksum,
  .compute_checksum = leapmotor_compute_checksum,
};
