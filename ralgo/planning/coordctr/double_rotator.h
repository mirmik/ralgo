#ifndef RALGO_PLANNING_DOUBLE_ROTATOR_H
#define RALGO_PLANNING_DOUBLE_ROTATOR_H

namespace ralgo
{
	class double_rotator 
	{
	public:
		ralgo::axis_controller<float, float> bot;
		ralgo::axis_controller<float, float> top;

		ralgo::phase_driver * bot_drv;
		ralgo::phase_driver * top_drv;

		double_rotator(ralgo::phase_driver * a, ralgo::phase_driver * b) 
		{
			bot_drv = a;
			top_drv = b;
		}

		void setup(ralgo::phase_driver * a, ralgo::phase_driver * b) 
		{
			bot_drv = a;
			top_drv = b;
		}

		double_rotator(){}

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