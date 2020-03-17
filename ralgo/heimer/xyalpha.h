#ifndef RALGO_HEIMER_XYALPHA_COORDINATE_CONTROLLER_H
#define RALGO_HEIMER_XYALPHA_COORDINATE_CONTROLLER_H

//#include <ralgo/planning/axis.h>
//#include <ralgo/planning/htrans2_mover.h>
//#include <ralgo/planning/cynchain2_output_mover.h>

//#include <ralgo/objects/served.h>
//#include <ralgo/objects/named.h>

#include <nos/print.h>
#include <nos/fprint.h>
#include <rabbit/space/screw.h>
#include <ralgo/heimer/kin2d_controller.h>

namespace ralgo
{
	namespace heimer
	{
		template <class P, class V>
		class xyalpha_controller:
			public kin2d_controller<P, V>,
			public control_info_node
		{
			using kin2d = kin2d_controller<P, V>;
			using kin2d::chain;

			ralgo::unit2d* axylinks[7];
			ralgo::kinematic_unit2d* axypairs[3];

			double ctrpos[3];
			double ctrspd[3];

			int64_t lasttime = 0;

			rabbit::htrans2<float> outpos;
			bool finish_inited = false;
			int finish_timeout = 100;
			int64_t finish_inited_time;

		public:
			ralgo::actuator2 x_link;
			ralgo::actuator2 y_link;
			ralgo::rotator2 a_link;
			ralgo::unit2d output_link;

			axis_driver<P, V>* _controlled_axes[3];

			rabbit::htrans2<float> nullpos;
			rabbit::htrans2<float> invnullpos;

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
			xyalpha_controller(const char* name, axis_driver<P, V>* controlleds[3]) :
				kin2d(ctraxes, 3),
				control_info_node(name, this, this, nullptr),

					x_link({1, 0}, 1),
			        y_link({0, 1}, 1),
			        a_link(1),

			        x_axis("x", this, 0),
			        y_axis("y", this, 1),
			        a_axis("a", this, 2)
			{
				x_link.link(&y_link);
				y_link.link(&a_link);
				a_link.link(&output_link);

				kin2d::setup(axylinks, axypairs, &output_link);

				for (int i = 0; i < 3; ++i)
				{
					_controlled_axes[i]  = controlleds[i];
					//_controlled_devices[i] = controlleds[i]->as_controlled();
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

				/*if (kin2d::is_extern_controlled() == false)
				{
					int sts = 1;
					sts = sts & x_axis.attime(time, xpos, xspd);
					sts = sts & y_axis.attime(time, ypos, yspd);
					sts = sts & a_axis.attime(time, apos, aspd);

					ctrpos[0] = xpos;
					ctrpos[1] = ypos;
					ctrpos[2] = apos;

					if (sts)
					{
						finish_inited = true;
						finish_inited_time = ralgo::discrete_time();
					}

					//syslog->info("control: {} {} {}", xpos, ypos, apos);
				}
				else
				{*/
					//	BUG();
				BUG();

					xpos = x_axis.ctrpos;
					xspd = x_axis.ctrspd;
					ypos = y_axis.ctrpos;
					yspd = y_axis.ctrspd;
					apos = a_axis.ctrpos;
					aspd = a_axis.ctrspd;
				
				//}

				pos = rabbit::htrans2<float> { apos, { xpos, ypos } };
				spd = rabbit::screw2<float> { aspd, {xspd, yspd} };

				pos = nullpos * pos;
				spd = nullpos.rotate_screw(spd);
			}

			void restore_control_model() override
			{
				finish_inited = false;
				double xpos, ypos, apos;

				xpos = _controlled_axes[0]->current_position();
				ypos = _controlled_axes[1]->current_position();
				apos = _controlled_axes[2]->current_position();

				DPRINT(xpos);
				DPRINT(ypos);
				DPRINT(apos);

				x_link.set_coord(xpos);
				y_link.set_coord(ypos);
				a_link.set_coord(apos);

				chain.update_location();
				outpos = chain.out()->global_location;

				auto outpos_corrected = invnullpos * outpos;

				x_axis._set_point_trajectory(outpos_corrected.translation().x);
				y_axis._set_point_trajectory(outpos_corrected.translation().y);
				a_axis._set_point_trajectory(outpos_corrected.rotation());
			}

			//igris::array_view<controlled*> controlled_devices() override
			//{
			//	return _controlled_devices;
			//}

			igris::array_view<axis_driver<float, float>*> controlled_axes() override
			{
				return _controlled_axes;
			}

			void update_state()
			{
				double xpos = _controlled_axes[0]->current_position();
				double ypos = _controlled_axes[1]->current_position();
				double apos = _controlled_axes[2]->current_position();

				x_link.set_coord(xpos);
				y_link.set_coord(ypos);
				a_link.set_coord(apos);

				chain.update_location();
				outpos = chain.out()->global_location;

				auto outpos_corrected = invnullpos * outpos;

				x_axis.feedpos = outpos_corrected.translation().x;
				y_axis.feedpos = outpos_corrected.translation().y;
				a_axis.feedpos = outpos_corrected.rotation();

				if (finish_inited & (ralgo::discrete_time() - finish_inited_time > finish_timeout))
				{
					BUG();
					//device::release_control_self();
					//x_axis.set_controller_force(nullptr);
					//y_axis.set_controller_force(nullptr);
					//a_axis.set_controller_force(nullptr);
					finish_inited = false;
				}

				//syslog->info("current: {} {} {}", _controlled_axes[0]->current_position(), _controlled_axes[1]->current_position(), _controlled_axes[2]->current_position());

				chain.update_location();
			}

			double * ctrspd_array() override { return ctrspd; }

			void apply_control()
			{
				int64_t time = millis();
				//syslog->info("{} {} {}", ctrspd[0], ctrspd[1], ctrspd[2]);

				//double delta = (double)(time - lasttime) / 1000;
				lasttime = time;

				//_controlled_axes[0]->direct_control(_controlled_axes[0]->current_position() + ctrspd[0]*delta, ctrspd[0]);
				//_controlled_axes[1]->direct_control(_controlled_axes[1]->current_position() + ctrspd[1]*delta, ctrspd[1]);
				//_controlled_axes[2]->direct_control(_controlled_axes[2]->current_position() + ctrspd[2]*delta, ctrspd[2]);

				_controlled_axes[0]->direct_control(_controlled_axes[0]->current_position(), ctrspd[0]);
				_controlled_axes[1]->direct_control(_controlled_axes[1]->current_position(), ctrspd[1]);
				_controlled_axes[2]->direct_control(_controlled_axes[2]->current_position(), ctrspd[2]);
			}

			void print_info() override
			{
				nos::fprintln("current: {} {} {}", _controlled_axes[0]->current_position(), _controlled_axes[1]->current_position(), _controlled_axes[2]->current_position());
				nos::fprintln("outpos: {}", outpos);
				nos::fprintln("control: {} {} {}", ctrpos[0], ctrpos[1], ctrpos[2]);
				nos::fprintln("ctrspd: {} {} {}", ctrspd[0], ctrspd[1], ctrspd[2]);
				nos::fprintln("links: {} {} {}", x_link.coord, y_link.coord, a_link.coord);

				nos::println("link poses");
				nos::fprintln("x_link: {} {}", x_link.global_location, x_link.output.global_location);
				nos::fprintln("y_link: {} {}", y_link.global_location, y_link.output.global_location);
				nos::fprintln("a_link: {} {}", a_link.global_location, a_link.output.global_location);
				nos::fprintln("o_link: {}", output_link.global_location);

				nos::println("link senses");
				nos::fprintln("x_link: {}", x_link.sensivity());
				nos::fprintln("y_link: {}", y_link.sensivity());
				nos::fprintln("a_link: {}", a_link.sensivity());
			}

			void control_interrupt_from(external_control_slot * slot)
			{
				BUG();
				//try_release_control();
				//deactivate();
			}

			external_control_slot* iterate(external_control_slot* slt) override
			{
				if (slt == nullptr)
					return _controlled_axes[0]->as_controlled();

				if (slt == _controlled_axes[0]->as_controlled())
					return _controlled_axes[1]->as_controlled();

				if (slt == _controlled_axes[1]->as_controlled())
					return _controlled_axes[2]->as_controlled();

				if (slt == _controlled_axes[2]->as_controlled())
					return nullptr;
				else
					return nullptr;
			}

			const char * name() override { return mnemo(); }
		};
	}
}

#endif