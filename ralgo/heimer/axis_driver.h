#ifndef RALGO_HEIMER_AXIS_DRIVER_H
#define RALGO_HEIMER_AXIS_DRIVER_H

#include <ralgo/heimer/axis_device.h>
#include <ralgo/planning/traj1d.h>

#include <igris/dtrace.h>

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
			Position tgtpos = 0;
			Speed compspd = 0;
			Speed tgtspd = 0;
			float poskoeff = 0;

		public:
			using parent::current_position;
			using parent::set_speed;
			ralgo::traj1d_line<Position, Speed> line_traj;
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
				poskoeff = koeff;
			}

			void set_trajectory(ralgo::traj1d<Position, Speed> * traj)
			{
				_current_trajectory = traj;
			}

			int _absmove_unsafe(Position curpos, Position tgtpos)
			{
				DTRACE();

				dprln("1");
				auto dist = tgtpos - curpos;

				dprln("2");
				int64_t curtim = ralgo::discrete_time();
				
				Speed dist_mul_freq = (Speed)fabs(dist) * ralgo::discrete_time_frequency();
				int64_t tgttim = curtim + (int64_t)(dist_mul_freq / parent::_speed);


				DPRINT(fabs(dist));
				DPRINT(ralgo::discrete_time_frequency());
				DPRINT(dist_mul_freq);
				DPRINT(parent::_speed);
				//DPRINT(((Speed)abs(dist)) * ((Speed)ralgo::discrete_time_frequency()) / parent::_speed);

				//auto acc = parent::_acc / (tgttim - curtim);
				//auto dcc = parent::_accdcc / (tgttim - curtim);

				dprln("3");
				DPRINT(parent::_acc_val);
				DPRINT(parent::_dcc_val);

				dprln("a");
				DPRINT(dist);
				DPRINT(parent::_speed);
				DPRINT(curtim);
				DPRINT(tgttim);
				line_traj.reset(curpos, curtim, tgtpos, tgttim);
				
				dprln("c");
				line_traj.set_speed_pattern(parent::_acc_val, parent::_dcc_val,
					parent::_speed);

				dprln("b");
				set_trajectory(&line_traj);
				dprln("out");
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
			void update_phase()
			{
				// Установить текущие целевые параметры.
				attime(ralgo::discrete_time(), tgtpos, tgtspd);
				//tgtspd = tgtspd * ralgo::discrete_time_frequency();
				//DPRINT(tgtspd);
				compspd = eval_compensated_speed(tgtpos, tgtspd);
			}

			// Вычислить желаемую скорость по текущей фазе
			Speed eval_compensated_speed(float tgtpos, float tgtspd)
			{
				// Счетчик меняется в прерывании, так что
				// снимаем локальную копию.
				auto current = current_position();

				// Ошибка по установленному значению.
				auto diff = tgtpos - current;

				// Скорость вычисляется как
				// сумма уставной скорости на
				auto evalspeed = tgtspd + poskoeff * diff;
				return evalspeed;
			}

			Speed compensated_speed()
			{
				return compspd;
			}

			void serve()
			{
				if (_current_trajectory && parent::controller() == this)
				{
					update_phase();
					apply_speed(compensated_speed());
				}
			}

			virtual void apply_speed(Speed spd) = 0;

			void direct_control(Speed spd)
			{
				apply_speed(spd);
			}
		};
	}
}

#endif