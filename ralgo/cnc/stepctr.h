#ifndef RALGO_CNC_STEPCTR_H
#define RALGO_CNC_STEPCTR_H

#define MOVETASK_BUFFER_SIZE 5

namespace cnc
{
	struct movetask 
	{
		int64_t time_0;
		int64_t time_1;
		int64_t pos_0;
		int64_t pos_1;
		float   spd_0;
		float   spd_1;
		float   acc;
	};

	class stepctr
	{
	public:
		struct movetask  mt;

		struct movetask  mt_queue_buffer[MOVETASK_BUFFER_SIZE];
		struct ring_head mt_queue_ring;
		int              mt_queue_counter;

		stepctr() 
		{
			ring_init(&ring, MOVETASK_BUFFER_SIZE);
			mtcounter = 0;
		}

		int next_movetask() 
		{
			if (mt_queue_counter == 0)
				return -1;

			mt = mt_queue_buffer[ring.tail];
			ring_move_tail_one(&ring);

			return 0;
		}

		void serve_impl(int64_t curtime) override
		{
			int64_t virtpos;
			
			if (curtime >= mt.time_1) 
			{
				int sts = next_movetask();
				(void) sts; 
			} 
			
			if (curtime < mt.time_1) 
			{
				int32_t time = curtime - mt.time_0;
				virtpos = mt.pos_0 + time * mt.spd_0 + mt.acc * time * time / 2.0;
			}
			else 
			{
				virtpos = pos_1;
			}
			
			int32_t diffpos = virtpos - ctrpos;
			bool positive = diffpos > 0;

			if (positive)
			{
				if (diffpos > pulsewidth_triggered)
				{
					inc();
					steps_total++;
					ctrpos += pulsewidth;
				}
			}

			else
			{
				if (diffpos < -pulsewidth_triggered)
				{
					dec();
					steps_total--;
					ctrpos -= pulsewidth;
				}
			}
		}
	};
}

#endif