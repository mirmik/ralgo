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
			void set_position_compensate(float koeff) 
			{
				poskoeff = koeff;
			}

			void set_trajectory(ralgo::traj1d<Position, Speed> * traj)
			{
				_current_trajectory = traj;
			}

			int incmove_unsafe(Position dist) override
			{
				auto curpos = current_position();
				int64_t curtim = ralgo::discrete_time();
				auto tgtpos = curpos + dist;
				int64_t tgttim = curtim 
					+ (int64_t)((Speed)abs(dist) / parent::_speed
					* ralgo::discrete_time_frequency());

				auto acc = parent::_accdcc / (tgttim - curtim);
				auto dcc = parent::_accdcc / (tgttim - curtim);
				
				//auto acc = 0.1;
				//auto dcc = 0.1;

				line_traj.reset(curpos, curtim, tgtpos, tgttim);
				line_traj.spddeform.reset(acc, dcc);

				set_trajectory(&line_traj);
			}

			int absmove_unsafe(Position pos) override
			{
				BUG();
			}

			int attime(int64_t time, Position& pos, Speed& spd)
			{
				return _current_trajectory->attime(time, pos, spd);
			}

			std::pair<Position,Speed> phase() 
			{
				Position pos;
				Speed spd;
				_current_trajectory->attime(ralgo::discrete_time(), pos, spd);
				return std::make_pair(pos, spd);
			}

			// Вычислить текущую фазу.
			void update_phase()
			{
				// Установить текущие целевые параметры.
				attime(ralgo::discrete_time(), tgtpos, tgtspd);
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
				update_phase();
				apply_speed(compensated_speed());
			}

			virtual void apply_speed(Speed spd) = 0;
		};
	}
}

#endif