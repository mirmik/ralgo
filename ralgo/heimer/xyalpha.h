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
		template <class P, class V> class xyalpha_controller;

		template <class P, class V>
		class xyalpha_axis
			: public heimer::axis_driver<P, V>, public heimer::controlled
		{
			xyalpha_controller<P, V> * parent;
			int index;

		public:
			xyalpha_axis(xyalpha_controller<P, V>* parent, int index) :
				parent(parent), index(index)
			{
				//igris::array_view<heimer::device*> arr {parent, 1};
				//set_controlled(arr);
			}

			P current_position() override
			{
				return parent->axposes[index];
			}

			V current_speed() override
			{
				BUG();
			}

			bool try_operation_begin(int priority) override { BUG(); }
			void operation_finish(int priority) override { BUG(); }

			heimer::controlled* as_controlled() { return this; }

			bool take_control(device * dev) override { return parent->take_control(dev); }
			bool take_control_force(device * dev) override { return parent->take_control_force(dev); }
			void release_control(device * dev) override { parent->release_control(dev); }
			void release_control_force(device * dev) override { parent->release_control_force(dev); }

			const char* name() { BUG(); }
		};

		template <class P, class V>
		class xyalpha_controller: public kin2d_controller<P, V>, public heimer::device
		{
			using kin2d = kin2d_controller<P,V>;
			using kin2d::chain;

		public:
			xyalpha_axis<P, V> x_axis;
			xyalpha_axis<P, V> y_axis;
			xyalpha_axis<P, V> a_axis;

			rabbit::htrans2<float> curpos;
			P axposes[3];

			//virtdevs::device* _deps[3];

		public:
			xyalpha_controller() :
				x_axis(this, 0),
				y_axis(this, 1),
				a_axis(this, 2)
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
			}

			void serve()
			{
				if (heimer::device::controller())
				{
					rabbit::htrans2<float> pos{};
					rabbit::screw2<float> spd{};

					get_control_phase(ralgo::discrete_time(), pos, spd);
					kin2d::set_phase(pos, spd);
				}
			}

		};
	}
}

#endif