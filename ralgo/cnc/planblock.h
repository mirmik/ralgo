#ifndef RALGO_CNC_PLANBLOCK_H
#define RALGO_CNC_PLANBLOCK_H

#include <stdint.h>
#include <ralgo/cnc/defs.h>
#include <nos/print.h>

namespace cnc
{
	class planner_block
	{
	public:
		int32_t steps[NMAX_AXES];
		int32_t major_steps;
		float nominal_velocity;

		float acceleration;
		float multipliers[NMAX_AXES];

		// отметки времени хранят инкрементное время до планирования и абсолютное
		// время после активации блока.
		int64_t acceleration_before_ic;
		int64_t deceleration_after_ic;
		int64_t block_finish_ic;

		uint8_t exact_stop;

	public:
		void shift_timestampes(int64_t iteration_counter)
		{
			acceleration_before_ic += iteration_counter;
			deceleration_after_ic += iteration_counter;
			block_finish_ic += iteration_counter;
		}

		bool is_active(int64_t interrupt_counter)
		{
			if (exact_stop)
				return interrupt_counter < block_finish_ic;
			else
				return interrupt_counter < deceleration_after_ic;
		}

		bool is_active_or_postactive(int64_t interrupt_counter)
		{
			return interrupt_counter < block_finish_ic;
		}

		bool is_accel(int64_t interrupt_counter)
		{
			return interrupt_counter < acceleration_before_ic;
		}

		float current_acceleration(int64_t interrupt_counter)
		{
			// Вычисление ускорения для трапециидального паттерна.
			if (interrupt_counter < acceleration_before_ic) return acceleration;
			if (interrupt_counter < deceleration_after_ic) return 0;
			if (interrupt_counter < block_finish_ic) return -acceleration;
			return 0;
		}

		void assign_accelerations(float * accs, int len, int64_t itercounter)
		{
			float acceleration = current_acceleration(itercounter);

			if (acceleration == 0)
			{
				for (int i = 0; i < len; ++i)
					accs[i] = 0;
				return;
			}

			for (int i = 0; i < len; ++i)
				accs[i] = acceleration * multipliers[i];
		}

		void append_accelerations(float * accs, int len, int64_t itercounter)
		{
			float acceleration = current_acceleration(itercounter);

			if (acceleration == 0)
				return;

			for (int i = 0; i < len; ++i)
			{
				accs[i] += acceleration * multipliers[i];
			}
		}
	};
}

#endif