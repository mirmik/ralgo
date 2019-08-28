#ifndef RALGO_PLANNING_HTRANS2_MOVER_H
#define RALGO_PLANNING_HTRANS2_MOVER_H

#include <rabbit/space/htrans2.h>

namespace ralgo 
{
	class htrans2_mover
	{
		virtual rabbit::htrans2<float> control_position() = 0;
	};
}

#endif