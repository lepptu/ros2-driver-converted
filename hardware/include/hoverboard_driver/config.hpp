#pragma once

// Wheel tick counters from the firmware are modulo 9000 (100 wheel revolutions):
// 15 pole pairs x 6 hall states = 90 ticks per mechanical revolution.
#define ENCODER_MIN 0
#define ENCODER_MAX 9000
#define ENCODER_LOW_WRAP_FACTOR 0.3
#define ENCODER_HIGH_WRAP_FACTOR 0.7

#define TICKS_PER_ROTATION 90
