#ifndef RALGO_HEIMER_LINEAR_INTERPOLATOR_H
#define RALGO_HEIMER_LINEAR_INTERPOLATOR_H

#include <ralgo/planning/trajNd.h>

namespace ralgo
{
	namespace heimer
	{
		template<size_t Dim, class Position, class Speed>
		class linear_interpolator : public heimer::device
		{
			Speed _speed = 0;
			float _accdcc;

			trajNd<Dim, Position, Speed> * trajectory;
			trajNd_line<Dim, Position, Speed> lintraj;

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
				igris::array_view<Position> dist
			) 
			{
				take_control();


			}

			void set_speed(Speed speed) 
			{
				_speed = speed;
			}

			void set_accdcc(float accdcc) 
			{
				_accdcc = accdcc;
			}
		};
	}
}

#endif