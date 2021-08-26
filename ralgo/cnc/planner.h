/// @file
/// Основано на https://github.com/grbl/grbl/blob/master/grbl/planner.h

#ifndef RALGO_CNC_PLANNER_H
#define RALGO_CNC_PLANNER_H

#include <igris/datastruct/ring.h>

#define CNC_PLANNER_BLOCK_BUFFER_SIZE 10

namespace cnc
{
	class plan_block
	{
		uint32_t steps[N_AXIS];    // Step count along each axis
		uint32_t step_event_count; // The maximum step axis count and number of steps required to complete this block.

		// Fields used by the motion planner to manage acceleration. Some of these values may be updated
		// by the stepper module during execution of special motion cases for replanning purposes.
		float entry_speed_sqr;     // The current planned entry speed at block junction in (mm/min)^2
		float max_entry_speed_sqr; // Maximum allowable entry speed based on the minimum of junction limit and

		//   neighboring nominal speeds with overrides in (mm/min)^2
		float acceleration;        // Axis-limit adjusted line acceleration in (mm/min^2). Does not change.
		float millimeters;         // The remaining distance for this block to be executed in (mm).

		// Stored rate limiting data used by planner when changes occur.
		float max_junction_speed_sqr; // Junction entry speed limit based on direction vectors in (mm/min)^2
		float rapid_rate;             // Axis-limit adjusted maximum rate for this block direction in (mm/min)
		float programmed_rate;        // Programmed rate of this block (mm/min).

#ifdef VARIABLE_SPINDLE
		// Stored spindle speed data used by spindle overrides and resuming methods.
		float spindle_speed;    // Block spindle speed. Copied from pl_line_data.
#endif
	};

	class plan_line_data
	{
		float feed_rate;          // Desired feed rate for line motion. Value is ignored, if rapid motion.
	};

	class profile_plan
	{
		plan_block block_buffer[CNC_PLANNER_BLOCK_BUFFER_SIZE];

		void reset();

		void discard_current_block();

		plan_block& current_block();

		uint8_t plan_buffer_line(float *target, plan_line_data *pl_data);
	};
}

#endif