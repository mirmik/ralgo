#ifndef RALGO_REVOLVER_H
#define RALGO_REVOLVER_H

#include <stdint.h>

#include <igris/container/ring.h>
#include <igris/sync/syslock.h>

#include <ralgo/robo/stepper.h>
#include <ralgo/cnc/shift.h>

namespace cnc
{

	/**
	 * Structure of shifts ring:
	 *
	 *              blocks.tail()                     (place for new shift)
	 *                    |                                   |
	 *                    |                                   |
	 *                    v                                   v
	 *      ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
	 *     |     |     |     |     |     |     |     |     |     |     |     |
	 *     |     |     |  .  |  .  |  .  |  .  |  .  |  .  |     |     |     |
	 *     |     |     |     |     |     |     |     |     |     |     |     |
	 *      ----- ----- ----- ----- ----- ----- ----- ----- ----- ----- -----
	 *                    ^                                   ^
	 *                    |                                   |
	 * counters:       (tail)-->                           (head)-->
	 * */
	class revolver
	{
	public:
		bool info_mode = false;

	private:
		bool first_iteration_label = false; 
		bool all_blocks_resolved = true; 

		int64_t iteration_counter;
		igris::ring<cnc::control_shift> * shifts_ring;

		robo::stepper ** steppers = nullptr;
		int steppers_total = 0;

	public:
		revolver(igris::ring<cnc::control_shift> * ring) :
			shifts_ring(ring)
		{}

		void set_steppers(robo::stepper ** steppers_table, int size)
		{
			steppers = steppers_table;
			steppers_total = size;
		}

		int queue_size()
		{
			int size;

			system_lock();
			size = shifts_ring->avail();
			system_unlock();

			return size;
		}

		int room()
		{
			// Операция взятия оставшегося места над кольцевым буфером
			// требует сравнения head и tail, изменяемых в разных потоках,
			// поэтому не является атомарной.
			system_lock();
			int ret = shifts_ring->room();
			system_unlock();

			return ret;
		}

		void push(uint16_t step, uint16_t dir)
		{
			// Добавление данных в очередь не требует блокировки,
			// потому что ring_head lockfree на добавление и чтение,
			// если unsigned int атомарен.

			shifts_ring->emplace(step, dir);
		}

		void serve()
		{

			if (first_iteration_label == false)
			{
				assert(steppers);

				first_iteration_label = true;
				if (info_mode)
				{
					ralgo::info("revolver: first start. success");
				}
			}

			// В общем случае ring_empty не атомарен. Однако, здесь должен
			// быть контекст приоритетного прерывания.
			if (shifts_ring->empty()) 
			{
				if (all_blocks_resolved == false) 
				{
					all_blocks_resolved = true;
					if (info_mode) 
					{
						ralgo::info("revolver: all blocks resolved");
					}
				} 
				return;
			}
			else 
			{
				all_blocks_resolved = false;
			}

			//int idx = shifts_ring->tail_index();
			auto & shift = shifts_ring->tail();

			for (int i = 0; i < steppers_total; ++i)
			{
				revolver_t mask = 1 << i;
				bool step = shift.step & mask;

				if (!step)
					continue;

				bool dir = shift.direction & mask;

				if (dir)
					steppers[i]->inc();
				else
					steppers[i]->dec();
			}

			shifts_ring->move_tail_one();
			return;
		}
	};
}

#endif