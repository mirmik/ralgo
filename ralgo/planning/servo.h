#ifndef RALGO_SERVO_H
#define RALGO_SERVO_H

#include <ralgo/planning/multiax.h>
#include <ralgo/planning/limit_switch.h>

#define RALGO_STOP_IMMEDIATE 1
#define RALGO_STOP_SMOOTH 2

namespace ralgo 
{
	enum class ServoOperationStatus : uint8_t 
	{
		stoped,
		moved,
		external
	};

	class external_servo_controller 
	{
		virtual void stop(uint8_t stopcode) = 0; 
	};

	struct servo_options 
	{
		bool forward_limit_switch = true;
		bool backward_limit_switch = true;
	};

	class servo 
	{
		int64_t acctime = 0;
		int64_t dcctime = 0;

		ralgo::traj1d_line line_traj;

		ralgo::external_servo_controller * extcontroller; 		
		
		ralgo::traj1d * current_trajectory;
		ralgo::speed_deformator spddeform;

		bool external_control = false;

		ralgo::phase_driver * drv;
		ralgo::limit_switch * flimit;
		ralgo::limit_switch * blimit;

		ServoOperationStatus opstat;

		servo_options options;

		bool is_powered = false;

	public:
		void incremental_move(int64_t imps, float spd) 
		{
			time_t settime = imps / spd;
			line_traj.reset(imps, settime);
			spddeform.set_accdcc(options.acc, options.dcc, settime);

			current_trajectory = &line_traj;
			current_trajectory.set_deform(&spddeform);
		}

		void serve(int64_t time) 
		{
			if (external_control)
				return;

			else 
			{
				ralgo::phase<> phase;
				int ret = current_trajectory->inloctime_deformed(time, &phs);

				if (ret == RALGO_TRAJECTORY_FINISHED) 
				{
					dprln("finished TODO.");
				}

				drv->set_phase(phs);
			}
		}

		void power(bool en) 
		{
			is_powered = en;
			drv->power(en);
		}

		void stop(uint8_t stopcode) 
		{
			if (external_control) 
			{
				extcontroller->stop(stopcode);
				return;
			}

			switch (stopcode) 
			{
				case RALGO_STOP_IMMEDIATE:
					BUG();

				case RALGO_STOP_SMOOTH:
					BUG();

				default:
					BUG();
			}
		}

		void flimit_event_handler() 
		{
			stop(RALGO_STOP_IMMEDIATE);
		}

		void blimit_event_handler() 
		{
			stop(RALGO_STOP_IMMEDIATE);
		}
	};
}

#endif