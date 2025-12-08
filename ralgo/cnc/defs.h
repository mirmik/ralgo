#ifndef RALGO_CNC_DEFS_H
#define RALGO_CNC_DEFS_H

#include <cstdint>

// Maximum number of axes supported
constexpr size_t NMAX_AXES = 10;

// Fixed-point arithmetic for DDA
// 2^24 gives ~7 bits of sub-step precision (1/128 step resolution)
constexpr int FIXED_POINT_BITS = 24;
constexpr int64_t FIXED_POINT_MUL = 1LL << FIXED_POINT_BITS;  // 16777216

// Type aliases for clarity
using cnc_float_type = double;      // For configuration and planning (mm, mm/sec)
using steps_t = int64_t;            // Position in steps
using fixed_t = int64_t;            // Fixed-point values (steps * FIXED_POINT_MUL)

// Default tick frequency (can be configured)
// At 480MHz with suitable prescalers, 100kHz-200kHz is typical
constexpr uint32_t DEFAULT_TICK_FREQUENCY_HZ = 100000;

#endif
