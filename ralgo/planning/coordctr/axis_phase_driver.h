#ifndef RALGO_PLANNING_AXIS_PHASE_DRIVER_H
#define RALGO_PLANNING_AXIS_PHASE_DRIVER_H

#include <ralgo/planning/axis_controller.h> 
#include <ralgo/planning/phase_driver_controller.h>

namespace ralgo 
{
	class axis_phase_driver_controller 
		: public axis_controller, public phase_driver_controller
	{
		
		
		void enable_mirror_mode(axis_controller & ctr, float reference) 
		{

		}
	};
}

#endif