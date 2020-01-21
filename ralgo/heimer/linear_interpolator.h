#ifndef RALGO_HEIMER_LINEAR_INTERPOLATOR_H
#define RALGO_HEIMER_LINEAR_INTERPOLATOR_H

#include <ralgo/planning/trajNd.h>
#include <ralgo/vecops.h>

namespace ralgo
{
	namespace heimer
	{
		template<size_t Dim, class Position, class Speed>
		class linear_interpolator : public heimer::device
		{
			using parent = heimer::device;

			Speed _speed = 0;
			float _accdcc = 0;

			trajNd<Dim, Position, Speed> * trajectory;
			trajNd_line<Dim, Position, Speed> lintraj;

			Position tgtpos[Dim];
			Speed compspd[Dim];
			Speed tgtspd[Dim];
			float poskoeff = 0.01;

		public:
			linear_interpolator(
			    igris::array_view<heimer::device*> axes)
			{
				set_controlled(axes);
			}

			axis_device<float, float> * get_axis(int index)
			{
				return static_cast<axis_device<float, float>*>(controlled()[index]);
			}

			void incmove(
			    igris::array_view<Position> mov
			)
			{
				take_control();
				auto dist = ralgo::vecops::norm(mov);
				int64_t time = (int64_t)((Speed)abs(dist) / _speed * ralgo::discrete_time_frequency());
				int64_t curtime = ralgo::discrete_time();
				int64_t tgttim = curtime + time;

				for (int i = 0; i < mov.size(); ++i)
				{
					auto curpos = static_cast<ralgo::heimer::axis_device<Position, Speed>*>(
					                               _controlled[i])->current_position();
					lintraj.set_start_position(i, curpos);
				}

				for (int i = 0; i < mov.size(); ++i)
				{
					lintraj.set_finish_position_inc(i, mov[i]);
				}

				lintraj.reset(curtime, tgttim);
				lintraj.spddeform.reset(_accdcc, _accdcc);

				trajectory = &lintraj;
			}

			void set_speed(Speed speed)
			{
				_speed = speed;
			}

			void set_accdcc(float accdcc)
			{
				_accdcc = accdcc;
			}

			void update_phase()
			{
				trajectory->attime(ralgo::discrete_time(), tgtpos, tgtspd);

				// Счетчик меняется в прерывании, так что
				// снимаем локальную копию.
				//		auto current[Dim];
				//		auto diff[Dim];

				for (int i = 0; i < Dim; ++i)
				{

					Position current = static_cast<ralgo::heimer::axis_device<Position, Speed>*>(
					                 _controlled[i])->current_position();

					// Ошибка по установленному значению.
					Position diff = tgtpos[i] - current;

					// Скорость вычисляется как
					// сумма уставной скорости на
					compspd[i] = tgtspd[i]  + poskoeff * diff;
				}
			}

			void apply_phase()
			{
				for (int i = 0; i < Dim; ++i)
				{
					static_cast<ralgo::heimer::axis_device<Position, Speed>*>(
					                 _controlled[i])->direct_control(compspd[i]);
				}
			}

			void serve()
			{
				if (trajectory && parent::controller() == this)
				{
					update_phase();
					apply_phase();
				}
			}
		};
	}
}

#endif