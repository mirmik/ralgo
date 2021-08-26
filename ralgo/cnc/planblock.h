#ifndef RALGO_CNC_PLANBLOCK_H
#define RALGO_CNC_PLANBLOCK_H

#include <stdint.h>
#include <ralgo/cnc/defs.h>
#include <nos/print.h>

#include <assert.h>

namespace cnc
{
	class planner_block
	{
	public:
		int32_t steps[NMAX_AXES];
		
		float nominal_velocity;
		float acceleration;
		float fullpath;

		float multipliers[NMAX_AXES];

		// отметки времени хранят инкрементное время до планирования и абсолютное
		// время после активации блока.
		int64_t start_ic;
		int64_t acceleration_before_ic;
		int64_t deceleration_after_ic;
		int64_t block_finish_ic;

		uint8_t exact_stop;

	public:
		bool validation() 
		{
			// TODO: remove assertation
			assert(fabs(acceleration_before_ic * acceleration - nominal_velocity) < 1e-5);
			assert(fabs(nominal_velocity * deceleration_after_ic - fullpath) < 1e-5);
			assert(block_finish_ic - deceleration_after_ic == acceleration_before_ic - start_ic);

			if (fabs(acceleration_before_ic * acceleration - nominal_velocity) > 1e-5)
				return false;

			if (fabs(nominal_velocity * deceleration_after_ic - fullpath) > 1e-5)
				return false;

			if (block_finish_ic - deceleration_after_ic != acceleration_before_ic - start_ic)
				return false;

			return true;
		}

		void shift_timestampes(int64_t iteration_counter)
		{
			start_ic += iteration_counter;
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

		void set_state(int32_t * steps, int axes, float velocity, float acceleration) 
		{	
			(void) steps;
			(void) axes;

			assert(velocity < 1);
			assert(acceleration < 1);

			float pathsqr = 0; 
			for (int i = 0; i < axes; ++i)
				pathsqr += steps[i] * steps[i];
			float path = sqrtf(pathsqr);
			float time = path / velocity;

			int itime = ceil(time);
			int preftime = ceil(velocity / acceleration);

			start_ic = 0;
			acceleration_before_ic = preftime;
			deceleration_after_ic = itime;
			block_finish_ic = itime + preftime;

			this->nominal_velocity = path / itime;
			this->acceleration = this->nominal_velocity / preftime;
			this->fullpath = path;

			memcpy(this->steps, steps, sizeof(int32_t) * axes);

			assert(validation());	
		}
	};
}

#endif