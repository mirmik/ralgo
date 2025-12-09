#ifndef RALGO_CNC_MATH_H
#define RALGO_CNC_MATH_H

/**
 * @file cnc_math.h
 * @brief Mathematical utilities for CNC motion calculations.
 *
 * Contains functions for calculating velocity and acceleration limits
 * based on axis constraints and motion direction.
 */

#include <cmath>
#include <limits>
#include <igris/container/static_vector.h>
#include <ralgo/cnc/defs.h>
#include <ralgo/linalg/vecops.h>

namespace cnc
{
    /**
     * Calculate velocity or acceleration limit based on per-axis constraints.
     *
     * Given a motion direction and per-axis maximum values, calculates
     * the maximum velocity/acceleration that respects all axis limits.
     *
     * @param direction Normalized direction vector of motion
     * @param absolute_maximum Overall limit (0 = use only per-axis limits)
     * @param element_maximums Per-axis maximum values
     * @return Maximum allowed velocity/acceleration
     */
    inline cnc_float_type evaluate_external_accfeed(
        const ralgo::vector<cnc_float_type> &direction,
        cnc_float_type absolute_maximum,
        const igris::static_vector<cnc_float_type, NMAX_AXES> &element_maximums)
    {
        int total_axes = (int)direction.size();
        cnc_float_type minmul = std::numeric_limits<cnc_float_type>::max();

        if (absolute_maximum == 0 &&
            ralgo::vecops::norm(element_maximums) == 0)
            return 0;

        if (absolute_maximum != 0)
            minmul = absolute_maximum;

        for (int i = 0; i < total_axes; i++)
            if (element_maximums[i] != 0)
            {
                cnc_float_type lmul =
                    element_maximums[i] / fabs(direction[i]);
                if (minmul > lmul)
                    minmul = lmul;
            }

        if (minmul == std::numeric_limits<cnc_float_type>::max())
            return 1000;
        return minmul;
    }

    /**
     * Calculate velocity or acceleration limit with box bounding.
     *
     * Alternative algorithm that bounds the velocity/acceleration vector
     * to a box defined by per-axis limits.
     *
     * @param direction Normalized direction vector of motion
     * @param absolute_maximum Overall limit (0 = compute from per-axis limits)
     * @param element_maximums Per-axis maximum values
     * @return Maximum allowed velocity/acceleration
     */
    inline cnc_float_type evaluate_external_accfeed_2(
        const ralgo::vector<cnc_float_type> &direction,
        cnc_float_type absolute_maximum,
        const igris::static_vector<cnc_float_type, NMAX_AXES> &element_maximums)
    {
        if (ralgo::vecops::norm(element_maximums) == 0)
        {
            return absolute_maximum;
        }

        if (absolute_maximum == 0)
        {
            auto bounded =
                ralgo::vecops::ray_to_box<ralgo::vector<cnc_float_type>>(
                    direction, element_maximums);
            return ralgo::vecops::norm(bounded);
        }
        else
        {
            auto vec = ralgo::vecops::mul_vs(direction, absolute_maximum);
            auto bounded =
                ralgo::vecops::bound_to_box<ralgo::vector<cnc_float_type>>(
                    vec, element_maximums);
            return ralgo::vecops::norm(bounded);
        }
    }
}

#endif
