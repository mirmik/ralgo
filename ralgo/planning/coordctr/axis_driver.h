#ifndef RALGO_PLANNING_AXIS_DRIVER_H
#define RALGO_PLANNING_AXIS_DRIVER_H

namespace ralgo 
{
	class axis_driver : public axis_controller<float,float>
	{
		ralgo::phase_driver * drv;

	public:
		void serve() 
		{
			float pos;
			float spd;

			attime(ralgo::discrete_time(), pos, spd);
		
			drv->set_phase(pos, spd);
		}				
	
		void set_driver(ralgo::phase_driver * drv) 
		{
			this->drv = drv;
		}
	};
}

#endif