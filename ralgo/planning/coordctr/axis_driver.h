#ifndef RALGO_PLANNING_AXIS_DRIVER_H
#define RALGO_PLANNING_AXIS_DRIVER_H

#include <ralgo/planning/axis.h>

namespace ralgo 
{
	template <
		class ExtPos=float, class IntPos=float, 
		class Speed=float, class Time=int64_t>
	class axis_driver : 
		public axis_controller<ExtPos, IntPos, Speed, Time>,
		public ralgo::served
	{
	public:
		float output_multiplier = 1;
		ralgo::phase_driver * drv;
		
	public:
		axis_driver(const char * name, ralgo::phase_driver * drv) 
			: drv(drv)
		{
			this->set_name(name);
		}

		void serve() override
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

		void activate() {}
		void deactivate() {}
	};
}

#endif