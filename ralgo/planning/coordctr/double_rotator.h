#ifndef RALGO_PLANNING_DOUBLE_ROTATOR_H
#define RALGO_PLANNING_DOUBLE_ROTATOR_H

#include <ralgo/virtdevs/device.h>

#warning "TODO Referenced axis."

namespace ralgo
{
	class double_rotator : public virtdevs::device
	{
	public:
		ralgo::axis_controller<float, float> bot;
		ralgo::axis_controller<float, float> top;

		ralgo::phase_driver * bot_drv;
		ralgo::phase_driver * top_drv;

		union 
		{
			virtdevs::device * _deps[2];
			struct {
				ralgo::axis_controller<float, float> * real_bot;
				ralgo::axis_controller<float, float> * real_top;
			};
		};

		igris::array_view<virtdevs::device *> dependence() 
		{
			return { _deps, 2 };
		}

		double_rotator(const char* name, 
			ralgo::phase_driver * a, ralgo::phase_driver * b) 
				: virtdevs::device(name)
		{
			bot_drv = a;
			top_drv = b;
		}

		void setup(ralgo::phase_driver * a, ralgo::phase_driver * b) 
		{
			bot_drv = a;
			top_drv = b;
		}

		double_rotator(const char* name) : virtdevs::device(name) {}

		void serve() 
		{
			float bpos, tpos;
			float bspd, tspd;

			auto time = ralgo::discrete_time();			

			top.attime(time, tpos, tspd);
			bot.attime(time, bpos, bspd);

			//nos::print(bpos);

			tpos = tpos - bpos;
			tspd = tspd - bspd;

			/*PRINT(tpos);
			PRINT(bpos);
			PRINT(tspd);
			PRINT(bspd);*/


			bot_drv->set_phase(bpos, bspd);
			top_drv->set_phase(tpos, tspd);
		}

		void restore_control_model()
		{
			bot.set_current_position(
				bot_drv->current_position());

			top.set_current_position(
				bot_drv->current_position() 
				+ top_drv->current_position());
		}
	};
}

#endif