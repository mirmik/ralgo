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
			float _accdcc;

			trajNd<Dim, Position, Speed> * trajectory;
			trajNd_line<Dim, Position, Speed> lintraj;

			Position tgtpos[Dim];
			Speed compspd[Dim];
			Speed tgtspd[Dim];
			float poskoeff[Dim];

		public:
			linear_interpolator(
			    igris::array_view<heimer::device*> axes)
			{
				set_controlled(axes);				
			}

			axis_device<float,float> * get_axis(int index) 
			{
				return static_cast<axis_device<float,float>*>(controlled()[index]);
			}

			void incmove(
				igris::array_view<Position> mov
			) 
			{
				take_control();
				auto dist = ralgo::vecops::norm(mov);
				auto time = dist / _speed;
				auto curtime = ralgo::discrete_time();

				for (int i = 0; i < mov.size(); ++i) 
				{
					lintraj.set_start_position(i, 
						static_cast<ralgo::heimer::axis_device<Position,Speed>*>(
							_controlled[i])->current_position());
				}

				for (int i = 0; i < mov.size(); ++i) 
				{
					lintraj.set_finish_position_inc(i, mov[i]);
				}

				lintraj.reset(curtime, curtime + time);

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

			void serve() 
			{
				if (trajectory && parent::controller() == this)
				{
				}
			}
		};
	}
}

#endif