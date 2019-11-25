#ifndef RALGO_XYALPHA_COORDINATE_CONTROLLER_H
#define RALGO_XYALPHA_COORDINATE_CONTROLLER_H

#include <ralgo/planning/axis.h>
#include <ralgo/planning/htrans2_mover.h>
#include <ralgo/planning/cynchain2_output_mover.h>

#include <ralgo/objects/served.h>
#include <ralgo/objects/named.h>

namespace ralgo
{
	class xyalpha_coordinate_controller;
	
	class xyalpha_coordinate_controller_axis 
		: public ralgo::axis_controller<float, float>
	{
		xyalpha_coordinate_controller * parent;

	public:
		xyalpha_coordinate_controller_axis(
			xyalpha_coordinate_controller* parent,
			const char * selfname);
	};

	class xyalpha_coordinate_controller 
		: public cynchain2_output_mover, 
		public virtual named_buffer<16>
	{
	public:
		const char* _name;
		xyalpha_coordinate_controller_axis x_axis;
		xyalpha_coordinate_controller_axis y_axis;
		xyalpha_coordinate_controller_axis a_axis;

		//virtdevs::device* _deps[3];

	public:
		xyalpha_coordinate_controller(const char* name) :
			_name(name), 
			x_axis(this, "x"), 
			y_axis(this, "y"), 
			a_axis(this, "a") 
		{}

		/*igris::array_view<virtdevs::device*> dependence() override 
		{
			return _deps;
		}*/

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


	xyalpha_coordinate_controller_axis::xyalpha_coordinate_controller_axis(
			xyalpha_coordinate_controller* parent,
			const char * selfname) : 
				parent(parent)
		{
			set_name(parent->_name, selfname);
		}

}

#endif