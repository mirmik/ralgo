#ifndef RALGO_SERVO_H
#define RALGO_SERVO_H

#include <ralgo/planning/speed_deformer.h>
#include <ralgo/planning/phase_driver.h>
#include <ralgo/planning/position_reader.h>

#include <ralgo/planning/traj1d.h>
#include <ralgo/planning/limit_switch.h>

#include <ralgo/planning/disctime.h>

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
		float nominal_speed = 0;

		bool forward_limit_switch = true;
		bool backward_limit_switch = true;
	};

	class servo : public external_servo_controller
	{
		int alarm_status = 0;
		//ServoOperationStatus opstat;

		//ralgo::speed_deformer spddeform;
		ralgo::traj1d_line line_traj;
		ralgo::traj1d * current_trajectory;

		ralgo::external_servo_controller * external_controller = nullptr;

		ServoOperationStatus current_status;

		ralgo::phase_driver * drv;
		ralgo::limit_switch * flimit;
		ralgo::limit_switch * blimit;


		bool is_powered = false;

		int64_t opstart_tstamp;
	//	int64_t opstart_position;

		servo * mirror; // Ведомый сервоусилитель.

		ralgo::position_reader * absolute_reader;

		float control_to_drive_gear = 1;

	public:
		servo_options options;

	private:
		void set_status(ServoOperationStatus status)
		{
			if (ServoOperationStatus::alarm == current_status)
				return;

			current_status = status;
		}

	public:
		servo(ralgo::phase_driver* drv) : drv(drv) {}

		int64_t absolute_position_read() 
		{
			return 
				absolute_reader->read() / control_to_drive_gear;			
		}

		int can_operation_start()
		{
			return
			    current_status != ServoOperationStatus::alarm;
		}

		void incremental_move_time(int64_t incpos, int64_t bytime)
		{
			if (!can_operation_start())
				return;

			auto curpos = drv->target_position();
			auto curtim = ralgo::discrete_time();

			line_traj.reset(
				curpos, curtim,
				curpos + incpos, curtim + bytime);

			line_traj.set_standart_accdcc_patern(
				options.acctime, 
				options.dcctime, 
				options.nominal_speed				
			);

			current_trajectory = &line_traj;

			current_status = ServoOperationStatus::moved;
			opstart_tstamp = curtim;
		}

		void serve(int64_t time)
		{
			//dprchar('s');

			if (external_controller)
				return;

			if (status() == ServoOperationStatus::moved)
			{
				ralgo::phase<int64_t, float> phs;
				int ret = current_trajectory->inloctime(time - opstart_tstamp, &phs);

				if (ret == RALGO_TRAJECTORY_FINISHED)
				{
					//dprln("FINISHED");
					//current_status = ServoOperationStatus::stoped;
				}

				set_phase(phs.d0, phs.d1);
			}
		}

		void set_phase(int64_t pos, float spd)
		{
			drv->set_phase(pos, spd);

			// Если у сервы есть ведомый, управление передаётся и на него.
			if (mirror)
				mirror->set_phase(pos, spd);
		}

		void set_mirror(servo* srv)
		{
			mirror = srv;
			mirror->external_controller = this;
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

		void stop_impl(ServoStopCommand stopcode)
		{
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

		void stop(ServoStopCommand stopcode)
		{
			if (external_controller)
			{
				external_controller->stop(stopcode);
				return;
			}

			stop_impl(stopcode);
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

		void swift_home_position(int64_t imps) 
		{
			drv->swift_zero(imps);
		}
	};
}

#endif