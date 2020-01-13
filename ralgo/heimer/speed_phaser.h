#ifndef RALGO_HEIMER_SPEED_PHASER
#define RALGO_HEIMER_SPEED_PHASER

#include <ralgo/heimer/device.h>

namespace ralgo 
{
	namespace heimer 
	{
		template <class Position, class Speed>
		class speed_phaser : public ralgo::heimer::device
		{
		public:
			void set_speed();
			Position target_position();
			Position feedback_position();
		};
	}
}

#endif