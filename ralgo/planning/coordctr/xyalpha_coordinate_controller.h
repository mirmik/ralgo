#ifndef RALGO_XYALPHA_COORDINATE_CONTROLLER_H
#define RALGO_XYALPHA_COORDINATE_CONTROLLER_H

#include <ralgo/planning/axis.h>
#include <ralgo/planning/htrans2_mover.h>
#include <ralgo/planning/cynchain2_output_mover.h>

namespace ralgo
{
	class xyalpha_coordinate_controller_axis 
		: public ralgo::axis_controller<float, float>
	{};

	class xyalpha_coordinate_controller 
		: public cynchain2_output_mover, public virtdevs::device
	{
	public:
		xyalpha_coordinate_controller_axis x_axis;
		xyalpha_coordinate_controller_axis y_axis;
		xyalpha_coordinate_controller_axis a_axis;

		virtdevs::device* _deps[3];

	public:
		xyalpha_coordinate_controller(const char* name) 
			: virtdevs::device(name)
		{

		}

		igris::array_view<virtdevs::device*> dependence() override 
		{
			return _deps;
		}

		void get_control_phase(
			int64_t time,
			rabbit::htrans2<float>& pos, 
			rabbit::screw2<float>& spd) 
		{
			float xpos, ypos, apos, xspd, yspd, aspd;

			x_axis.attime(time, xpos, xspd);		
			y_axis.attime(time, ypos, yspd);		
			a_axis.attime(time, apos, aspd);

			//nos::print(xpos, ypos, apos, xspd, yspd, aspd);
			//nos::print("fadsfa");
			//nos::print(xpos);

			pos = rabbit::htrans2<float>{ apos, { xpos, ypos } };
			spd = rabbit::screw2<float>{ aspd, {xspd, yspd} };
		}

		void restore_control_model() override 
		{
			rabbit::htrans2<float> curpos = chain.out()->global_location;
			
			x_axis.set_current_position(curpos.translation().x);
			y_axis.set_current_position(curpos.translation().y);
			a_axis.set_current_position(curpos.rotation());
		}
	};
}

#endif