#ifndef RALGO_CNC_DEFS_H
#define RALGO_CNC_DEFS_H

#include <cstdint>

// Maximum number of axes supported
constexpr size_t NMAX_AXES = 10;

// Fixed-point arithmetic for DDA
// Higher precision needed for low accelerations to minimize integration error
// At freq=100kHz, steps_per_unit=1000, M=1 unit/s²:
//   acc = 1e-7 steps/tick²
// With 2^24: acc_fixed = 1.68 -> 40% error, unusable
// With 2^32: acc_fixed = 429 -> 0.12% error, ~1 step error over 1000
// With 2^40: acc_fixed = 109951 -> 0.0004% error, negligible
//
// Overflow check at 40 bits:
//   Max velocity ~10 steps/tick, vel_fixed = 10 * 2^40 = 1.1e13
//   Over 100k ticks: pos_fixed accumulates ~1.1e18, int64_t max is 9.2e18 -> OK
constexpr int FIXED_POINT_BITS = 40;
constexpr int64_t FIXED_POINT_MUL = 1LL << FIXED_POINT_BITS;  // 1099511627776

// Type aliases for clarity
using cnc_float_type = double;      // For configuration and planning (mm, mm/sec)
using steps_t = int64_t;            // Position in steps
using fixed_t = int64_t;            // Fixed-point values (steps * FIXED_POINT_MUL)

// Default tick frequency (can be configured)
// At 480MHz with suitable prescalers, 100kHz-200kHz is typical
constexpr uint32_t DEFAULT_TICK_FREQUENCY_HZ = 100000;

#endif
