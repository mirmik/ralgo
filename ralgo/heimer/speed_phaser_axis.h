#ifndef RALGO_HEIMER_SPEED_PHASER_AXIS_H
#define RALGO_HEIMER_SPEED_PHASER_AXIS_H

#include <ralgo/heimer/speed_phaser.h>
#include <ralgo/heimer/axis_driver.h>

namespace ralgo 
{
	namespace heimer 
	{
		template < class Position, class IntPos, class Speed >
		class speed_phaser_axis : public axis_driver<Position, Speed> 
		{
			speed_phaser<Position,IntPos,Speed> * _phaser;

		public:
			speed_phaser_axis(speed_phaser<Position,IntPos,Speed>* phaser) 
				: _phaser(phaser)
			{}

			Position current_position() override 
			{
				return _phaser->target_position();
			}

			void apply_speed(Speed spd) override 
			{
				_phaser->set_speed(spd);
			}
		};
	}
}

#endif