#ifndef RALGO_PLANNING_DOUBLE_ROTATOR_H
#define RALGO_PLANNING_DOUBLE_ROTATOR_H

namespace 
{
	class double_rotator 
	{
	public:
		ralgo::axis_controller<float, float> bot;
		ralgo::axis_controller<float, float> top;

		ralgo::phase_driver * bot_drv;
		ralgo::phase_driver * top_drv;

		void serve() 
		{
			float bpos, tpos;
			float bspd, tspd;

			auto time = ralgo::discrete_time();			

			top.attime(time, tpos, tspd);
			bot.attime(time, bpos, bspd);

			bot_drv.set_phase(bpos, bspd);
			top_drv.set_phase(tpos, tspd);
		}

		void restore_control_model() override 
		{
			bot.set_current_position(
				bot_drv.control_position_unit());

			top.set_current_position(
				bot_drv.control_position_unit() 
				+ top_drv.control_position_unit());
		}
	};
}

#endif