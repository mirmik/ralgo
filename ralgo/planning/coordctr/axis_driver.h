#ifndef RALGO_PLANNING_AXIS_DRIVER_H
#define RALGO_PLANNING_AXIS_DRIVER_H

#include <ralgo/planning/axis.h>
#include <ralgo/planning/speed_phaser.h>

namespace ralgo
{
	template <
	    class ExtPos = float, class IntPos = int64_t,
	    class Speed = float, class Time = int64_t >
	class axis_driver :
		public virtual axis_controller<ExtPos, IntPos, Speed, Time>,
		public ralgo::served
	{
		using parent = axis_controller<ExtPos, IntPos, Speed, Time>;

	public:
		float output_multiplier = 1;
		ralgo::speed_phaser<IntPos, Speed> * drv;

	public:
		axis_driver(
		    const char * name,
		    ralgo::speed_phaser<IntPos, Speed> * drv)
			:
			drv(drv)
		{
			this->set_name(name);
		}

		IntPos current_position() 
		{
			return drv->current_position();
		}

		void serve() override
		{
			parent::update_phase();
			drv->set_speed(parent::compensated_speed());
		}

		void set_driver(ralgo::speed_phaser<IntPos, Speed> * drv)
		{
			this->drv = drv;
		}

		void activate() {}
		void deactivate() {}

		void update_control() {  }
	};
}

#endif