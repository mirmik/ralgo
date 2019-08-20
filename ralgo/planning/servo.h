#ifndef RALGO_SERVO_H
#define RALGO_SERVO_H

#include <ralgo/planning/speed_deformer.h>
#include <ralgo/planning/phase_driver.h>

#include <ralgo/planning/multiax.h>
#include <ralgo/planning/limit_switch.h>

namespace ralgo 
{
	enum class ServoOperationStatus : uint8_t 
	{
		stoped = 0,
		moved = 1,
		external = 2,
		alarm = 3
	};

	enum class ServoStopCommand 
	{
		immediate = 0,
		smooth = 1
	};

	class external_servo_controller 
	{
	public:
		virtual void stop(ServoStopCommand stopcode) = 0; 
	};

	struct servo_options 
	{
		int32_t acctime = 0;
		int32_t dcctime = 0;

		bool forward_limit_switch = true;
		bool backward_limit_switch = true;
	};

	class servo 
	{
		int alarm_status = 0;
		//ServoOperationStatus opstat;

		//ralgo::speed_deformer spddeform;
		ralgo::traj1d_line line_traj;
		ralgo::traj1d * current_trajectory;
		
		ralgo::external_servo_controller * extcontroller; 		

		bool external_control = false;
		ServoOperationStatus current_status;

		ralgo::phase_driver * drv;
		ralgo::limit_switch * flimit;
		ralgo::limit_switch * blimit;
		
		servo_options options;

		bool is_powered = false;

		int64_t opstart_tstamp;

	private:
		void set_status(ServoOperationStatus status) 
		{
			if (ServoOperationStatus::alarm == current_status)
				return;

			current_status = status; 
		}

	public:
		servo(ralgo::phase_driver* drv) : drv(drv) {}

		int can_operation_start() 
		{
			return 
				current_status != ServoOperationStatus::alarm;
		}

		void incremental_move_time(int64_t imps, int64_t settime) 
		{
			if (!can_operation_start())
				return;

			dprln("incremental_move");
			line_traj.reset(imps, settime);
		
			line_traj.spddeform
				.reset(0.3,0.3);

			current_trajectory = &line_traj;

			current_status = ServoOperationStatus::moved;
			opstart_tstamp = millis();
		}

		void serve(int64_t time) 
		{
			//dprchar('s');

			if (external_control)
				return;

			if (status() == ServoOperationStatus::moved)
			{
				ralgo::phase<int64_t, float> phs;
				int ret = current_trajectory->inloctime(time - opstart_tstamp, &phs);

				//dprln(ret);
				dprln(phs.d0, phs.d1);

				if (ret == RALGO_TRAJECTORY_FINISHED) 
				{
					//dprln("FINISHED");
					//current_status = ServoOperationStatus::stoped;
				}

				drv->set_phase(phs.d0, phs.d1);
			}
		}

		ServoOperationStatus status() 
		{
			return current_status;
		}

		void power(bool en) 
		{
			is_powered = en;
			drv->power(en);
		}

		void stop(ServoStopCommand stopcode) 
		{
			if (external_control) 
			{
				extcontroller->stop(stopcode);
				return;
			}

			switch (stopcode) 
			{
				case ServoStopCommand::immediate:
					BUG();

				case ServoStopCommand::smooth:
					BUG();

				default:
					BUG();
			}
		}

		void flimit_event_handler() 
		{
			stop(ServoStopCommand::immediate);
		}

		void blimit_event_handler() 
		{
			stop(ServoStopCommand::immediate);
		}

		void alarm_event_handler(uint8_t errcode) 
		{

		}
	};
}

#endif