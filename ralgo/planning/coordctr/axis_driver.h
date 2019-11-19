#ifndef RALGO_PLANNING_AXIS_DRIVER_H
#define RALGO_PLANNING_AXIS_DRIVER_H

#include <ralgo/planning/axis.h>

namespace ralgo 
{
	template <
		class ExtPos=float, class IntPos=float, 
		class Speed=float, class Time=int64_t>
	class axis_driver : 
		public axis_controller<ExtPos, IntPos, Speed, Time>
	{
		ralgo::phase_driver * drv;
		float output_multiplier = 1;

	public:
		void serve() 
		{
			float pos;
			float spd;

			this->attime(ralgo::discrete_time(), pos, spd);
		
			drv->set_phase(
				pos * output_multiplier, 
				spd * output_multiplier);
		}				
	
		void set_driver(ralgo::phase_driver * drv) 
		{
			this->drv = drv;
		}
	};
}

#endif