

class control_block
{
	struct dlist_head lnk;

	int32_t steps[N_AXES];

	int32_t major_step;
	int32_t acceleration_before;
	int32_t deceleration_after;

	float nominal_increment;
	float acceleration;
	float deceleration;

	float multipliers[N_AXES];
	float accelerations[N_AXES];
	float decelerations[N_AXES];

	int64_t acceleration_before_ic;
	int64_t deaceleration_after_ic;
	int64_t block_finish_ic;

	bool is_active(int64_t interrupt_counter)
	{
		return interrupt_counter < block_finish_ic;
	}

	float current_acceleration(int64_t interrupt_counter)
	{
		if (interrupt_counter < acceleration_before_ic) return acceleration;
		if (interrupt_counter < deaceleration_after_ic) return 0;
		return deceleration;
	}

	//runtime
	uint8_t state;
}

class control_axis_phase
{
	int64_t steps;
	float dda_counter;
	float velocity;
	float acceleration;
}

class control_applier
{
	int64_t interrupt_counter;
	int64_t revolver_iteration_counter;

	dlist_head active_blocks;
	int64_t reference_position [N_AXES];

	float delta;
	float delta_sqr_div_2;

	void algorithm_step_for_trapecidal_profile()
	{
		int room = revolver_cycle->room();

		while (room--) 
		{
			// Планируем поведение револьвера на несколько циклов вперёд
			// попутно инкрементируя модельное время.
			iteration(revolver_iteration_counter);
			++revolver_iteration_counter;
		}
	}

	void iteration()
	{
		control_block * itblock;

		// plan moment
		if (kept_another_block())
		{
			for (int i = 0; i < total_axes; ++i)
			{
				axes[i]->acceleration += itblock->acceleration * itblock->multipliers[i];
			}

			move_block_to_accel(itblock);
			continue;
		}

		// accel finish
		while ()
		{
			for (int i = 0; i < total_axes; ++i)
			{
				axes[i]->acceleration += itblock->acceleration * itblock->multipliers[i];
			}

			move_block_to_cruis(itblock);
			continue;
		}

		// cruis finish
		while ()
		{
			for (int i = 0; i < total_axes; ++i)
			{
				axes[i]->acceleration += itblock->deceleration * itblock->multipliers[i];
			}

			move_block_to_decel(itblock);
			continue;
		}

		// decel finish
		{
			for (int i = 0; i < total_axes; ++i)
			{
				axes[i]->acceleration -= itblock->deceleration * itblock->multipliers[i];
			}
			discard_block(itblock);
		}

		// Normalization;
		if (cruise_situation())
		{
			auto * block = cruis_block();
			for (int i = 0; i < total_axes; ++i)
			{
				axes_records[i] -> acceleration = 0;
				axes_records[i] -> velocity = block->nominal_velocity * block->multiplier;
			}
		}

		// check position
		if (idle_situation())
		{
			for (int i = 0; i < total_axes; ++i)
			{
				axes_records[i] -> acceleration = 0;
				axes_records[i] -> velocity = 0;

				assert(axes_records[i]->steps[i] == reference_position[i]);
			}
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

	class step_revolver
	{
		typedef uint16_t revolver_t;

	private:
		struct revolver_shift
		{
			revolver_t step;
			revolver_t direction;
		};

	private:
		int64_t iteration_counter;
		ring_head ring;
		impulse_shift shifts[SHIFTER_RING_SIZE];

	public:
		void room() 
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
			
			int idx = ring_head_index(&ring);
			shifts[idx] = step;
			shifts[idx] = direction;
			ring_move_head_one(&ring);
		}

		void serve()
		{
			// В общем случае ring_empty не атомарен. Однако, здесь должен
			// быть контекст приоритетного прерывания.
			if (ring_empty(&ring))
				return;

			int idx = ring_index(&ring);
			auto & shift = shifts[idx];

			for (int i = 0; i < axes_total; ++i)
			{
				revolver_t mask = 1 << i;
				bool step = shift.step & mask;

				if (!step)
					continue;

				bool dir = shift.direction & mask;
				switch (dir)
				{
					case -1: axes[i].dec(); break;
					case 1: axes[i].inc(); break;
					default: break;
				}
			}

			ring_move_tail_one(&ring);
			return;
		}
	}
}
