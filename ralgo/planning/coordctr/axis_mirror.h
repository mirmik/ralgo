#ifndef RALGO_PLANNING_AXIS_MIRROR_H
#define RALGO_PLANNING_AXIS_MIRROR_H

#include <ralgo/planning/axis.h>

namespace ralgo 
{
	class axis_ofsetter 
	{
		ralgo::axis_controller<float,float> * followed;
		ralgo::phase_driver * drv;

	public:
		ralgo::axis_controller<float,float> ctr;

		void serve() 
		{
			auto time = ralgo::discrete_time();

			float followed_pos;
			float followed_spd;

			float pos;
			float spd;

			followed->attime(time, followed_pos, followed_spd);
			ctr.attime(time, pos, spd);

			drv->set_phase(
				pos + followed_pos,
				spd + followed_spd);
		}

		void set_driver(ralgo::phase_driver * drv) 
		{
			this->drv = drv;
		}

		void set_target(ralgo::axis_controller<float,float> * tgt) 
		{
			this->followed = tgt;
		}
	};
}

#endif