#ifndef RALGO_HEIMER_SPEED_PHASER_AXIS_H
#define RALGO_HEIMER_SPEED_PHASER_AXIS_H

#include <ralgo/heimer/speed_phaser.h>

namespace ralgo 
{
	namespace heimer 
	{
		template < class Position, class Speed >
		class speed_phaser_axis : public axis_driver<Position, Speed> 
		{
			speed_phaser * phaser;

		public:
			int set_phase() 
			{

			};
		};
	}
}

#endif