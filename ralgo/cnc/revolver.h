#ifndef RALGO_REVOLVER_H
#define RALGO_REVOLVER_H

#include <stdint.h>

#include <igris/datastruct/ring.h>
#include <igris/sync/syslock.h>

#include <ralgo/robo/stepper.h>

namespace cnc
{
	class revolver_ring
	{
		typedef uint16_t revolver_t;

	public:
		struct shift
		{
			revolver_t step;
			revolver_t direction;
		};

	private:
		int64_t iteration_counter;
		ring_head ring;
		shift * shifts;

		robo::stepper ** steppers;
		int steppers_total;

	public:
		revolver_ring(shift * shifts, int ring_size) :
			shifts(shifts)
		{
			ring_init(&ring, ring_size);
		}

		void set_steppers(robo::stepper ** steppers_table, int size) 
		{
			steppers = steppers_table;
			steppers_total = size;
		}

		int queue_size() 
		{
			int size;

			system_lock();
			size = ring_avail(&ring);
			system_unlock();

			return size;
		}

		int room()
		{
			// Операция взятия оставшегося места над кольцевым буфером
			// требует сравнения head и tail, изменяемых в разных потоках,
			// поэтому не является атомарной.
			system_lock();
			int ret = ring_room(&ring);
			system_unlock();

			return ret;
		}

		void push(uint16_t step, uint16_t dir)
		{
			// Добавление данных в очередь не требует блокировки,
			// потому что ring_head lockfree на добавление и чтение,
			// если unsigned int атомарен.

			int idx = ring.head;
			shifts[idx].step = step;
			shifts[idx].direction = dir;
			ring_move_head_one(&ring);
		}

		void serve()
		{
			// В общем случае ring_empty не атомарен. Однако, здесь должен
			// быть контекст приоритетного прерывания.
			if (ring_empty(&ring))
				return;

			int idx = ring.tail;
			auto & shift = shifts[idx];

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

			ring_move_tail_one(&ring);
			return;
		}
	};
}

#endif