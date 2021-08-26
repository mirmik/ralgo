#ifndef RALGO_PLANNER_RING_H
#define RALGO_PLANNER_RING_H

#include <stdint.h>

#include <ralgo/cnc/planblock.h>
#include <ralgo/cnc/shift.h>

#include <igris/datastruct/dlist.h>
#include <igris/container/ring.h>
#include <igris/sync/syslock.h>

#include <nos/print.h>

#define NMAX_AXES 10

namespace cnc
{
	/**
	 * Structure of blocks ring:
	 *
	 * accessors:   blocks.tail()
	 *                  |
	 *          :       |               acceleration
	 * signation:       |                   or        (place for new block)
	 *          :       |  deceleration   cruis               |
	 *                  |  |    |     |     |                 |
	 *                  v  v    v     v     v                 v
	 *      ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
	 *     |     |     |     |     |     |     |     |     |     |     |     |
	 *     |     |     |  .  |  .  |  .  |  .  |  .  |  .  |     |     |     |
	 *     |     |     |     |     |     |     |     |     |     |     |     |
	 *      ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
	 *                    ^                 ^                 ^
	 *                    |                 |                 |
	 * counters:       (tail)--->       (active)--->       (head)--->
	 * */
	class planner
	{
	public:
		int64_t iteration_counter = 0;
		int64_t * reference_position; /// < для внутреннего контроля

		float delta = 1;
		float delta_sqr_div_2 = 0.5;

		float accelerations[NMAX_AXES];
		float velocities[NMAX_AXES];
		int64_t steps[NMAX_AXES];
		float dda_counters[NMAX_AXES];

		int active = 0; // index of active block
		planner_block * active_block = nullptr;

		igris::ring<cnc::planner_block> * blocks;
		igris::ring<cnc::control_shift> * shifts;

		int total_axes = 0;
		bool need_to_reevaluate = false;
		uint8_t state = 0;
		int count_of_reevaluation = 0;

	public:

		void set_dim(int axes)
		{
			total_axes = axes;
		}

		/*void set_revolver_delta(float delta)
		{
			this->delta = delta;
			this->delta_sqr_div_2 = delta * delta / 2;
		}*/

		void reset_iteration_counter()
		{
			iteration_counter = 0;
		}

		planner(igris::ring<cnc::planner_block> * blocks,
		        igris::ring<cnc::control_shift> * shifts)
			: blocks(blocks), shifts(shifts)
		{
			memset(accelerations, 0, sizeof(accelerations));
			memset(velocities, 0, sizeof(velocities));
			memset(steps, 0, sizeof(steps));
			memset(dda_counters, 0, sizeof(dda_counters));
		}

		int block_index(planner_block * it)
		{
			return blocks->index_of(it);
		}

		void fixup_postactive_blocks()
		{
			while (blocks->tail_index() != active)
			{
				if (!blocks->tail().is_active_or_postactive(iteration_counter))
					blocks->pop();

				else
					break;
			}
		}

		bool has_postactive_blocks()
		{
			return active != blocks->tail_index();
		}

		int count_of_postactive()
		{
			return blocks->distance(active, blocks->tail_index());
		}

		void start_with_first_block()
		{
			active_block = &blocks->tail();
			iteration_counter = 0;
			need_to_reevaluate = true;
		}

		int serve()
		{
			int final;

			system_lock();
			int room = shifts->room();
			system_unlock();

			if (active_block == nullptr && !has_postactive_blocks())
			{
				if (blocks->empty())
					return 1;

				else
					start_with_first_block();
			}

			while (room--)
			{
				// Планируем поведение револьвера на несколько циклов вперёд
				// попутно инкрементируя модельное время.
				final = iteration();

				if (final)
					return final;
			}

			return 0;
		}

		void evaluate_accelerations()
		{
			if (active_block)
				active_block->assign_accelerations(
				    accelerations, total_axes, iteration_counter);
			else
			{
				for (int i = 0; i < total_axes; ++i)
					accelerations[i] = 0;
			}

			for (int i = blocks->tail_index(); i != active; i = blocks->fixup_index(i + 1))
				blocks->get(i).append_accelerations(
				    accelerations, total_axes, iteration_counter);
		}

		void change_active_block()
		{
			active = blocks->fixup_index(active + 1);

			if (active == blocks->head_index())
			{
				active_block = nullptr;
				return;
			}

			active_block = &blocks->get(active);
			active_block -> shift_timestampes(iteration_counter);
		}

		int iteration()
		{
			if (active_block)
			{
				if (state == 0)
				{
					if (!active_block->is_accel(iteration_counter))
					{
						need_to_reevaluate = true;
						state = 1;
					}
				}

				else
				{
					if (!active_block->is_active(iteration_counter))
					{
						change_active_block();
						need_to_reevaluate = true;
						state = 0;
					}
				}
			}

			for (int i = blocks->tail_index(); i != active;
			        i = blocks->fixup_index(i + 1))
			{
				if (!blocks->get(i).is_active_or_postactive(iteration_counter))
				{
					need_to_reevaluate = true;
				}
			}

			if (need_to_reevaluate)
			{
				fixup_postactive_blocks();
				evaluate_accelerations();
				need_to_reevaluate = false;
				count_of_reevaluation++;
			}

			if (active_block == nullptr && !has_postactive_blocks())
			{
				return 1;
			}

			revolver_t mask, step = 0, dir = 0;

			for (int i = 0; i < total_axes; ++i)
			{
				mask = (1 << i);

				dda_counters[i] +=
				    velocities[i] + //* delta +
				    accelerations[i] * 0.5; //* delta_sqr_div_2;

				if (dda_counters[i] > 0.9)
				{
					dda_counters[i] -= 1;
					steps[i] += 1;

					dir |= mask;
					step |= mask;
				}
				else if (dda_counters[i] < -0.9)
				{
					dda_counters[i] += 1;
					steps[i] -= 1;

					dir |= mask;
				}
				velocities[i] += accelerations[i] * delta;
			}
			shifts->emplace(dir, step);
			iteration_counter++;

			return 0;
		}
	};
}

#endif