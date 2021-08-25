#ifndef RALGO_PLANNER_RING_H
#define RALGO_PLANNER_RING_H

#include <stdint.h>
#include <igris/datastruct/dlist.h>
#include <igris/container/ring.h>

#include <ralgo/cnc/shift.h>

#define NMAX_AXES 10

namespace cnc
{

	class control_block
	{
		int32_t steps[NMAX_AXES];

		int32_t major_step;
		int32_t acceleration_before;
		int32_t deceleration_after;

		float nominal_increment;
		float acceleration;
		float deceleration;

		float multipliers[NMAX_AXES];
		float accelerations[NMAX_AXES];
		float decelerations[NMAX_AXES];

		int64_t acceleration_before_ic;
		int64_t deaceleration_after_ic;
		int64_t block_finish_ic;

		uint8_t exact_stop;

		//runtime
		uint8_t state;

	public:
		bool is_active(int64_t interrupt_counter)
		{
			if (exact_stop)
				return interrupt_counter < block_finish_ic;
			else
				return interrupt_counter < deaceleration_after_ic;
		}

		bool is_postactive(int64_t interrupt_counter)
		{
			return interrupt_counter < block_finish_ic;
		}

		float current_acceleration(int64_t interrupt_counter)
		{
			if (interrupt_counter < acceleration_before_ic) return acceleration;
			if (interrupt_counter < deaceleration_after_ic) return 0;
			return deceleration;
		}
	};

	class planner_ring
	{
		int64_t interrupt_counter;
		int64_t revolver_iteration_counter;

		int64_t * reference_position; /// < для внутреннего контроля

		float delta;
		float delta_sqr_div_2;

		control_block * active_block;

		igris::ring<cnc::control_block> * blocks;
		igris::ring<cnc::control_shift> * shifts;
		int ring_postactive_head;

		int total_axes;

	public:
		planner_ring(igris::ring<cnc::control_block> * blocks,
		             igris::ring<cnc::control_shift> * shifts)
			: blocks(blocks), shifts(shifts)
		{}

		int change_active_block()
		{
			blocks->pop();

			
		}

		int block_index(control_block * it)
		{
			return it - planned;
		}

		void fixup_postactive_blocks()
		{
			while (ring_postactive_counter_head != ring->head)
			{
				if (!planned[ring_postactive_counter_head].is_postactive())
				{
					ring_postactive_head = (ring_postactive_head + 1) % ring->size;
					planned[ring_postactive_counter_head].valid = false;
				}

				else
				{
					break;
				}
			}
		}

		void discard_postactive(control_block * it)
		{
			if (block_index(it) == ring_postactive_counter_head)
				fixup_postactive_blocks();
		}

		void algorithm_step_for_trapecidal_profile()
		{
			int final;
			int room = revolver_cycle->room();

			while (room--)
			{
				// Планируем поведение револьвера на несколько циклов вперёд
				// попутно инкрементируя модельное время.
				final = iteration(revolver_iteration_counter);

				if (final)
					return final;

				++revolver_iteration_counter;
			}
		}

		int iteration()
		{
			control_block * itblock;

			if (!active_block->is_active(current_iteration))
			{
				int final = change_active_block(active_block);

				if (final)
					return final;
			}

			auto acceleration = active_block->current_acceleration();

			dlist_for_each_entry_safe(itblock, safeit, postactive_blocks, lnk)
			{
				if (!itblock->is_postactive())
					discard_postactive(itblock);

				acceleration += itblock->current_acceleration();
			}

			for (int i = 0; i < total_axes; ++i)
			{
				axes_records[i]->dda_counter +=
				    axes_records[i]->velocity * delta +
				    axes_records[i]->acceleration * delta_sqr_div_2;

				if (axes_records[i]->dda_counter > 1)
				{
					axes_records[i]->dda_counter -= 1;
					axes_records[i]->steps += 1;
				}
				else if (axes_records[i]->dda_counter < -1)
				{
					axes_records[i]->dda_counter += 1;
					axes_records[i]->steps -= 1;
				}

				axes_records[i]->velocity += axes_records[i]->acceleration * delta;
			}
		}
	};
}

#endif