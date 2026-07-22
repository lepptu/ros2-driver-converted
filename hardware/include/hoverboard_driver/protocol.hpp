// Serial protocol shared with the hoverboard firmware fork
// (lepptu/hoverboard-firmware-hack-FOC, "Robot standby feature" protocol).
// Field order and checksums MUST match Src/main.c / Inc/util.h in the firmware.

#ifndef _FOC_PROTOCOL_H
#define _FOC_PROTOCOL_H

#include <cstdint>

#define START_FRAME 0xABCD

typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  int16_t  flags;      // bit0: motors allowed. 0 = force disarm (standby), 1 = normal arming
  uint16_t checksum;   // XOR of all preceding fields
} SerialCommand;
static_assert(sizeof(SerialCommand) == 10, "SerialCommand must match the firmware layout (10 bytes)");

typedef struct {
  uint16_t start;
  int16_t  cmd1;           // input1 the firmware accepted (steer echo)
  int16_t  cmd2;           // input2 the firmware accepted (speed echo)
  int16_t  speedR_meas;    // RPM
  int16_t  speedL_meas;    // RPM
  int16_t  wheelR_cnt;     // hall ticks, modulo 9000
  int16_t  wheelL_cnt;     // hall ticks, modulo 9000
  int16_t  left_dc_curr;   // DC link current * 100 [A]
  int16_t  right_dc_curr;  // DC link current * 100 [A]
  int16_t  iq_l;           // FOC q-axis (torque) current, raw fixdt; /50 (A2BIT_CONV) = A
  int16_t  iq_r;
  int16_t  batVoltage;     // V * 100
  int16_t  boardTemp;      // degC * 10
  uint16_t cmdLed;         // status word: bits 0-3 errCode L, 4-7 errCode R, bit 8 motors enabled, bit 9 serial timeout
  uint16_t checksum;       // XOR of all preceding fields
} SerialFeedback;
static_assert(sizeof(SerialFeedback) == 30, "SerialFeedback must match the firmware layout (30 bytes)");

#endif
