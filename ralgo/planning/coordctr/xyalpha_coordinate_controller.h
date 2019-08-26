#ifndef RALGO_XYALPHA_COORDINATE_CONTROLLER_H
#define RALGO_XYALPHA_COORDINATE_CONTROLLER_H

#include <ralgo/planning/htrans2_mover.h>
#include <ralgo/planning/cynematic_chain_object2d.h>

namespace ralgo
{
	class xyalpha_coordinate_controller_axis : public ralgo::axis
	{
		int axno;

		
	};

	class xyalpha_coordinate_controller : public htrans2_mover
	{
	public:
		xyalpha_coordinate_controller_axis x_axis;
		xyalpha_coordinate_controller_axis y_axis;
		xyalpha_coordinate_controller_axis alpha_axis;

	public:
		ralgo::cynematic_chain2d_output_controller* cchain;

		xyalpha_coordinate_controller(ralgo::cynematic_chain2d_output_controller* cchain)
			: cchain(cchain)
		{}
	};
}

#endif