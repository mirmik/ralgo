#ifndef RALGO_HEIMER_XYALPHA_COORDINATE_CONTROLLER_H
#define RALGO_HEIMER_XYALPHA_COORDINATE_CONTROLLER_H

//#include <ralgo/planning/axis.h>
//#include <ralgo/planning/htrans2_mover.h>
//#include <ralgo/planning/cynchain2_output_mover.h>

//#include <ralgo/objects/served.h>
//#include <ralgo/objects/named.h>

#include <ralgo/heimer/kin2d_controller.h>

namespace ralgo
{
	namespace heimer
	{
		template <class P, class V>
		class xyalpha_controller: public kin2d_controller<P, V>
		{
			using kin2d = kin2d_controller<P, V>;
			using kin2d::chain;

			ralgo::actuator2 x_link;
			ralgo::actuator2 y_link;
			ralgo::rotator2 a_link;
			ralgo::unit2d output_link;
			
			ralgo::unit2d* axylinks[7];
			ralgo::kinematic_unit2d* axypairs[3];

		public:
			axis_driver<P, V>* controlled_axes[3];
			heimer::controlled* _controlled_devices[3];

			union
			{
				virtual_multiax_axis<P, V> ctraxes[3];

				struct
				{
					virtual_multiax_axis<P, V> x_axis;
					virtual_multiax_axis<P, V> y_axis;
					virtual_multiax_axis<P, V> a_axis;
				};
			};
			rabbit::htrans2<float> curpos;
			P axposes[3];

			//virtdevs::device* _deps[3];

		public:
			xyalpha_controller(const char* name, axis_driver<P,V>* controlleds[3]) :
				kin2d(name, ctraxes, axposes, 3),
				
				x_link({1,0},1),
				y_link({0,1},1),
				a_link(1),
				
				x_axis(this, 0),
				y_axis(this, 1),
				a_axis(this, 2)
			{
				x_link.link(&y_link);
				y_link.link(&a_link);
				a_link.link(&output_link);

				kin2d::setup(axylinks, axypairs, &output_link);

				for (int i = 0; i < 3; ++i) 
				{
					controlled_axes[i]  = controlleds[i];
					_controlled_devices[i] = controlleds[i]->as_controlled();
				}
			}

			void relocate(
				rabbit::htrans2<float> x, 
				rabbit::htrans2<float> y, 
				rabbit::htrans2<float> a, 
				rabbit::htrans2<float> out) 
			{
				x_link.relocate(x);
				y_link.relocate(y);
				a_link.relocate(a);
				output_link.relocate(out);
			}

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

				dprln("get control phase");

				x_axis.attime(time, xpos, xspd);
				y_axis.attime(time, ypos, yspd);
				a_axis.attime(time, apos, aspd);

				//nos::print(xpos, ypos, apos, xspd, yspd, aspd);
				//nos::print("fadsfa");
				//nos::print(xpos);

				pos = rabbit::htrans2<float> { apos, { xpos, ypos } };
				spd = rabbit::screw2<float> { aspd, {xspd, yspd} };
			}

			void restore_control_model() override
			{


				curpos = chain.out()->global_location;

				axposes[0] = curpos.translation().x;
				axposes[1] = curpos.translation().y;
				axposes[2] = curpos.rotation();

				//x_axis._set_point_trajectory(axposes);
			}

			igris::array_view<controlled*> controlled_devices() override
			{
				return _controlled_devices;
			}
		};
	}
}

#endif