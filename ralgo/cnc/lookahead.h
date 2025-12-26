#ifndef RALGO_CNC_LOOKAHEAD_H
#define RALGO_CNC_LOOKAHEAD_H

/**
 * @file lookahead.h
 * @brief Look-ahead planning parameters and conversion utilities.
 *
 * Look-ahead (junction deviation algorithm) allows smooth transitions
 * between motion segments by calculating appropriate junction velocities
 * based on the angle between segments.
 *
 * The junction_deviation parameter (in mm) defines the maximum allowed
 * deviation from the ideal path at corners. Typical values: 0.01-0.05 mm.
 *
 * Usage:
 *   cnc::lookahead_params params;
 *   params.set_junction_deviation(0.02);  // 0.02 mm
 *   params.update_control_scale(control_scale_array, num_axes);
 *
 *   if (params.is_enabled()) {
 *       planner.recalculate_block_velocities(params.junction_deviation_steps());
 *   }
 */

#include <ralgo/cnc/defs.h>
#include <array>

namespace cnc
{
    /**
     * Look-ahead planning parameters.
     *
     * Manages junction deviation settings and unit conversion between
     * user units (mm) and internal units (steps).
     */
    class lookahead_params
    {
    private:
        // Junction deviation in user units (mm)
        cnc_float_type _junction_deviation = 0;

        // Junction deviation in steps (converted value)
        cnc_float_type _junction_deviation_steps = 0;

        // Whether look-ahead is enabled
        bool _enabled = false;

        // Cached average control_scale for conversion (pulses_per_unit)
        cnc_float_type _avg_control_scale = 1.0;

    public:
        lookahead_params() = default;

        /**
         * Set junction deviation in user units (mm).
         * Enables look-ahead if value > 0, disables if value == 0.
         *
         * @param value Junction deviation in mm (typical: 0.01-0.05)
         */
        void set_junction_deviation(cnc_float_type value)
        {
            _junction_deviation = value;
            _enabled = (value > 0);
            recalculate_steps();
        }

        /**
         * Update control_scale conversion factor.
         * Call this when control_scale (pulses_per_unit) changes.
         *
         * @param control_scale Array of control_scale for each axis
         * @param num_axes Number of axes
         */
        void update_control_scale(const cnc_float_type *control_scale,
                                  int num_axes)
        {
            if (num_axes <= 0)
            {
                _avg_control_scale = 1.0;
                recalculate_steps();
                return;
            }

            // Calculate average control_scale from active axes
            cnc_float_type sum = 0;
            int count = 0;
            for (int i = 0; i < num_axes; ++i)
            {
                if (control_scale[i] > 0)
                {
                    sum += control_scale[i];
                    count++;
                }
            }

            if (count > 0)
                _avg_control_scale = sum / count;
            else
                _avg_control_scale = 1.0;

            recalculate_steps();
        }

        /**
         * Update from std::array of control_scale.
         */
        template <size_t N>
        void update_control_scale(const std::array<cnc_float_type, N> &control_scale,
                                  int num_axes)
        {
            update_control_scale(control_scale.data(), num_axes);
        }

        /**
         * Get junction deviation in user units (mm).
         */
        cnc_float_type junction_deviation() const
        {
            return _junction_deviation;
        }

        /**
         * Get junction deviation in steps.
         */
        cnc_float_type junction_deviation_steps() const
        {
            return _junction_deviation_steps;
        }

        /**
         * Check if look-ahead is enabled.
         */
        bool is_enabled() const
        {
            return _enabled;
        }

        /**
         * Disable look-ahead.
         */
        void disable()
        {
            _junction_deviation = 0;
            _junction_deviation_steps = 0;
            _enabled = false;
        }

    private:
        void recalculate_steps()
        {
            if (_junction_deviation <= 0)
            {
                _junction_deviation_steps = 0;
                return;
            }

            _junction_deviation_steps = _junction_deviation * _avg_control_scale;
        }
    };
}

#endif
