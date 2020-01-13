#ifndef RALGO_HEIMER_AXIS_CONSOLE_H
#define RALGO_HEIMER_AXIS_CONSOLE_H

#include <nos/format.h>
#include <ralgo/heimer/axis_device.h>

namespace ralgo 
{
	namespace heimer 
	{
		template <class Position, class Speed>
		class axis_console 
		{
		public:
			igris::array_view<ralgo::heimer::axis_device<Position, Speed> *>


			int current_position(int argc, char** argv, char* retbuf, size_t retmax) 
			{
			}	
		}
	}
}

#endif