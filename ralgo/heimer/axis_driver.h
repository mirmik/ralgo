#ifndef RALGO_HEIMER_AXIS_DRIVER_H
#define RALGO_HEIMER_AXIS_DRIVER_H

#include <ralgo/heimer/axis_device.h>
#include <ralgo/disctime.h>
#include <ralgo/planning/traj1d.h>

#include <igris/event/event.h>

#include <nos/log/logger.h>
#include <iostream>
#include <ralgo/heimer/control.h>

extern nos::log::logger * syslog;

namespace ralgo
{
	namespace heimer
	{
		// Возможно следует объединить с классом axis_device.
		template <class Position, class Speed>
		class axis_driver :
			public axis_device<Position, Speed>,
			public external_control_slot
		{
			using parent = axis_device<Position, Speed>;
			float poskoeff = 0;

			bool operation_finished_flag = true;

		public:
			Position ctrpos = 0;
			Speed ctrspd = 0;

			Position feedpos = 0;
			Speed feedspd = 0;

			using parent::operation_finish;
			using parent::current_position;
			using parent::current_speed;
			using parent::set_speed;
			ralgo::traj1d_line<Position, Speed> line_traj;

			igris::event operation_finish_event;

			void set_operation_finish_event(igris::event ev)
			{
				operation_finish_event = ev;
			}

			bool is_in_operation_state()
			{
				return !operation_finished_flag;
			}

		protected:
			ralgo::traj1d<Position, Speed> * _current_trajectory = nullptr;

		public:
			ralgo::traj1d_line<Position, Speed> & linear_trajectory()
			{
				return line_traj;
			}

			ralgo::traj1d<Position, Speed> * current_trajectory()
			{
				return _current_trajectory;
			}

			void set_position_compensate(float koeff)
			{
				poskoeff = koeff;
			}

			void set_trajectory(ralgo::traj1d<Position, Speed> * traj)
			{
				_current_trajectory = traj;
			}

			int _absmove_unsafe(Position curpos, Position tgtpos)
			{
				auto dist = tgtpos - curpos;
				int64_t curtim = ralgo::discrete_time();

				Speed dist_mul_freq = (Speed)fabs(dist) * ralgo::discrete_time_frequency();
				int64_t tgttim = curtim + (int64_t)(dist_mul_freq / parent::_speed);

				if (curtim == tgttim)
				{
					operation_finished_flag = true;
					operation_finish(0);
					operation_finish_event(this);
					_current_trajectory = nullptr;

					return 0;
				}

				line_traj.reset(curpos, curtim, tgtpos, tgttim);

				line_traj.set_speed_pattern(parent::_acc_val, parent::_dcc_val,
				                            parent::_speed);

				operation_finished_flag = false;
				set_trajectory(&line_traj);
				return 0;
			}

			int incmove_unsafe(Position dist) override
			{
				auto curpos = current_position();
				return _absmove_unsafe(curpos, curpos + dist);
			}

			int absmove_unsafe(Position pos) override
			{
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
					operation_finished_flag = true;
					operation_finish(0);
					operation_finish_event(this);
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

			void direct_control(Position pos, Speed spd)
			{
				assert(is_extern_controlled());

				ctrpos = pos;
				ctrspd = spd;
			}

			void set_ctrphase(Position pos, Speed spd)
			{
				ctrpos = pos;
				ctrspd = spd;
			}

			int hardstop() override
			{
				_current_trajectory = nullptr;
				operation_finish(1);

				return 0;
			}

			// Обновляет текущие переменные контроля.
			// Или на основе траекторрии, или ничего не делает, если контролль внешний.
			void evaluate_ctrvars()
			{
				if (_current_trajectory && !is_extern_controlled())
				{
					update_control_by_trajectory();
				}
			}

			void stop_impl() override
			{
				if (_current_trajectory == nullptr)
					return;

				line_traj.set_stop_trajectory(
				    current_position(),
				    current_speed(),
				    parent::_dcc_val);

				_current_trajectory = & line_traj;
			}


			void set_compensate(double k)
			{
				poskoeff = k;
			}

			virtual external_control_slot * as_controlled() = 0;

			virtual const char * name() = 0;
			virtual Position current_position() final { return feedpos; }
			virtual Speed current_speed() final { return feedspd; }

			bool in_operation()
			{
				return _current_trajectory != nullptr;
			}

			virtual bool can_operate() = 0;

			void print_feed()  override
			{
				nos::fprintln("feed:(pos:{},spd:{})", feedpos, feedspd);
			};
		};
	}
}

#endif