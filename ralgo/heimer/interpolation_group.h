#ifndef RALGO_HEIMER_INTERPOLATION_GROUP_H
#define RALGO_HEIMER_INTERPOLATION_GROUP_H

#include <ralgo/heimer/control.h>

namespace ralgo 
{
	namespace heimer 
	{
		template<class Position, class Speed>
		class interpolation_group
		{
		public:
			interpolation_group() {}

			virtual int incmove(Position * mov) = 0;
			virtual int absmove(Position * pos) = 0;

			virtual int set_speed(Speed spd) = 0;
			virtual int set_accdcc_value(float acc, float dcc) = 0;
			virtual int dim() = 0;

			virtual Speed speed() = 0;
			virtual Speed acceleration() = 0;
			virtual Speed deceleration() = 0;

			virtual void debug_print_traj() = 0;
			virtual void debug_print_state() = 0;
		};
	}
}

#endif