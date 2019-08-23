#ifndef RALGO_PLANNING_AXIS_H
#define RALGO_PLANNING_AXIS_H

namespace ralgo 
{
	enum stop_pattern 
	{
		immediate,
		smooth 
	};

	class axis 
	{
		virtual void incmove_tstamp(int64_t pulses, int64_t tstamp);	
		virtual void incmove(int64_t pulses, float speed);	

		virtual void absmove_tstamp(int64_t pulses, int64_t tstamp);	
		virtual void absmove(int64_t pulses, float speed);

		virtual void stop(ralgo::stop_pattern stpcode);
	};
}

#endif