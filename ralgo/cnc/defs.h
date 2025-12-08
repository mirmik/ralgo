#ifndef RALGO_CNC_DEFS_H
#define RALGO_CNC_DEFS_H

#include <cstdint>

// Maximum number of axes supported
constexpr size_t NMAX_AXES = 10;

// Fixed-point arithmetic for DDA
// 2^32 gives better precision for low accelerations
// At freq=100kHz, steps_per_unit=1000, M=1 unit/s²:
//   acc = 1e-7 steps/tick², acc_fixed = 1e-7 * 2^32 = 430
// With 2^24: acc_fixed = 1.68 -> 40% error!
// With 2^32: acc_fixed = 430 -> 0.2% error
constexpr int FIXED_POINT_BITS = 32;
constexpr int64_t FIXED_POINT_MUL = 1LL << FIXED_POINT_BITS;  // 4294967296

// Type aliases for clarity
using cnc_float_type = double;      // For configuration and planning (mm, mm/sec)
using steps_t = int64_t;            // Position in steps
using fixed_t = int64_t;            // Fixed-point values (steps * FIXED_POINT_MUL)

// Default tick frequency (can be configured)
// At 480MHz with suitable prescalers, 100kHz-200kHz is typical
constexpr uint32_t DEFAULT_TICK_FREQUENCY_HZ = 100000;

#endif
