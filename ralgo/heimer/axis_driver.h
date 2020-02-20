#ifndef RALGO_HEIMER_AXIS_DRIVER_H
#define RALGO_HEIMER_AXIS_DRIVER_H

#include <ralgo/heimer/axis_device.h>
#include <ralgo/planning/traj1d.h>

#include <igris/dtrace.h>
#include <igris/event/event.h>

#include <iostream>

namespace ralgo
{
	namespace heimer
	{
		// Возможно следует объединить с классом axis_device.
		template <class Position, class Speed>
		class axis_driver : public axis_device<Position, Speed>
		{
			using parent = axis_device<Position, Speed>;
			Position ctrpos = 0;
			Speed compspd = 0;
			Speed ctrspd = 0;
			float poskoeff = 0;

			bool operation_finished_flag = true;

		public:
			using parent::current_position;
			using parent::set_speed;
			ralgo::traj1d_line<Position, Speed> line_traj;
			
			igris::event operation_finish_event;

			void set_operation_finish_event(igris::event ev)
			{
				DTRACE();
				operation_finish_event = ev;
			}

		private:
			ralgo::traj1d<Position, Speed> * _current_trajectory = nullptr;

		public:
			ralgo::traj1d_line<Position, Speed> * linear_trajectory()
			{
				return &line_traj;
			}

			ralgo::traj1d<Position, Speed> * current_trajectory()
			{
				return _current_trajectory;
			}

			void set_position_compensate(float koeff)
			{
				DTRACE();
				poskoeff = koeff;
			}

			void set_trajectory(ralgo::traj1d<Position, Speed> * traj)
			{
				DTRACE();
				_current_trajectory = traj;
			}

			int _absmove_unsafe(Position curpos, Position tgtpos)
			{
				DTRACE();

				auto dist = tgtpos - curpos;

				int64_t curtim = ralgo::discrete_time();

				Speed dist_mul_freq = (Speed)fabs(dist) * ralgo::discrete_time_frequency();
				int64_t tgttim = curtim + (int64_t)(dist_mul_freq / parent::_speed);


				line_traj.reset(curpos, curtim, tgtpos, tgttim);

				line_traj.set_speed_pattern(parent::_acc_val, parent::_dcc_val,
				                            parent::_speed);

				operation_finished_flag = false;
				set_trajectory(&line_traj);
				return 0;
			}

			int incmove_unsafe(Position dist) override
			{
				DTRACE();
				auto curpos = current_position();
				return _absmove_unsafe(curpos, curpos + dist);
			}

			int absmove_unsafe(Position pos) override
			{
				DTRACE();
				auto curpos = current_position();
				return _absmove_unsafe(curpos, pos);
			}

			int attime(int64_t time, Position& pos, Speed& spd)
			{
				return _current_trajectory->attime(time, pos, spd, ralgo::discrete_time_frequency());
			}

			std::pair<Position, Speed> phase()
			{
				Position pos = 0;
				Speed spd = 0;
				
				if (_current_trajectory)
					_current_trajectory->attime(
					    ralgo::discrete_time(), pos, spd,
					    ralgo::discrete_time_frequency());
				return std::make_pair(pos, spd);
			}

			// Вычислить текущую фазу.
			void update_control_by_trajectory()
			{
				// Установить текущие целевые параметры.
				int sts = attime(ralgo::discrete_time(), ctrpos, ctrspd);
				if (sts && !operation_finished_flag)
				{
					dprln("operation finish!!!!!!!!!!!!!!!!");
					operation_finished_flag = true;
					operation_finish_event((void*)parent::name());
					_current_trajectory = nullptr;
				}
			}

			// Вычислить желаемую скорость по текущей фазе
			Speed eval_compensated_speed()
			{
				// Счетчик меняется в прерывании, так что
				// снимаем локальную копию.
				auto current = current_position();

				// Ошибка по установленному значению.
				auto diff = ctrpos - current;

				// Скорость вычисляется как
				// сумма уставной скорости на
				auto evalspeed = ctrspd + poskoeff * diff;
				//DPRINT(evalspeed);
				return evalspeed;
			}

			Speed compensated_speed()
			{
				return compspd;
			}

			void serve()
			{

				//DPRINTPTR(parent::controller());
				if (_current_trajectory && parent::controller() == this)
				{
					update_control_by_trajectory();
				}

				apply_control();
			}

			virtual void apply_speed(Speed spd) = 0;

			void apply_control() 
			{
				compspd = eval_compensated_speed();
				apply_speed(compspd);
			} 

			void direct_control(Position pos, Speed spd)
			{
				ctrpos = pos;
				ctrspd = spd;
			}

			int hardstop() override
			{
				parent::release_control_force();
				ctrpos = 0;
				ctrspd = 0;
				_current_trajectory = nullptr;
				return 0;
			}

			void stop_impl() override
			{
				DPRINT(compspd);
				line_traj.set_stop_trajectory(
					current_position(), 
					compspd,
					parent::_dcc_val);
			
				_current_trajectory = & line_traj;
			}
		};
	}
}

#endif