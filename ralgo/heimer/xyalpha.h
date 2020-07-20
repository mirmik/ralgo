#ifndef HEIMER_XYALPHA_COORDINATE_CONTROLLER_H
#define HEIMER_XYALPHA_COORDINATE_CONTROLLER_H

#include <nos/print.h>
#include <nos/fprint.h>

#include <ralgo/disctime.h>

#include <ralgo/space/screw.h>
#include <ralgo/space/htrans2.h>
#include <ralgo/heimer/kin2d_controller.h>

namespace heimer
{
	template <class P, class V>
	class xyalpha_chain2d_controller :
		public kinematic_chain2d_controller<P, V>
	{
		using kin2d = kinematic_chain2d_controller<P, V>;
		using kin2d::chain;

		ralgo::unit2d*           axylinks[7];
		ralgo::kinematic_unit2d* axypairs[3];

		double ctrpos[3];
		double ctrspd[3];

		int64_t lasttime = 0;

		ralgo::htrans2<float> outpos;

	public:
		ralgo::actuator2 x_link;
		ralgo::actuator2 y_link;
		ralgo::rotator2 a_link;
		ralgo::unit2d output_link;

		heimer::axis_node <P, V> * x_controlled;
		heimer::axis_node <P, V> * y_controlled;
		heimer::axis_node <P, V> * a_controlled;

		ralgo::htrans2<float> nullpos;
		ralgo::htrans2<float> invnullpos;

		union
		{
			virtual_axis_node<P, V> ctraxes[3];

			struct
			{
				virtual_axis_node<P, V> x_axis;
				virtual_axis_node<P, V> y_axis;
				virtual_axis_node<P, V> a_axis;
			};
		};
		ralgo::htrans2<float> curpos;
		P axposes[3];

		//virtdevs::device* _deps[3];

	public:
		xyalpha_chain2d_controller(
			const char* name,
			axis_node<P,V> * x_controlled,
			axis_node<P,V> * y_controlled,
			axis_node<P,V> * a_controlled
		) :
			kin2d(ctraxes, 3),
			control_node(name),

			x_link( {1, 0}, 1),
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

			x_axis = x_controlled;
		}

		void relocate(
			ralgo::htrans2<float> x,
			ralgo::htrans2<float> y,
			ralgo::htrans2<float> a,
			ralgo::htrans2<float> out)
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
			ralgo::htrans2<float>& pos,
			ralgo::screw2<float>& spd)
		{
			float xpos, ypos, apos, xspd, yspd, aspd;

			x_axis.evaluate_ctrvars();
			y_axis.evaluate_ctrvars();
			a_axis.evaluate_ctrvars();

			xpos = x_axis.ctrpos;
			xspd = x_axis.ctrspd;
			ypos = y_axis.ctrpos;
			yspd = y_axis.ctrspd;
			apos = a_axis.ctrpos;
			aspd = a_axis.ctrspd;

			pos = ralgo::htrans2<float> { apos, { xpos, ypos } };
			spd = ralgo::screw2<float> { aspd, {xspd, yspd} };

			pos = nullpos * pos;
			spd = nullpos.rotate_screw(spd);
		}

		void restore_control_model() override
		{
			double xpos, ypos, apos;

			xpos = x_controlled->current_position();
			ypos = y_controlled->current_position();
			apos = a_controlled->current_position();

			x_link.set_coord(xpos);
			y_link.set_coord(ypos);
			a_link.set_coord(apos);

			chain.update_location();
			outpos = chain.out()->global_location;

			auto outpos_corrected = invnullpos * outpos;

			x_axis.set_ctrphase(outpos_corrected.translation().x, 0);
			y_axis.set_ctrphase(outpos_corrected.translation().y, 0);
			a_axis.set_ctrphase(outpos_corrected.rotation(), 0);
		}

		/*igris::array_view<axis_driver<float, float>*> controlled_axes() override
		{
			return _controlled_axes;
		}*/

		void feedback()
		{
			double xpos = x_controlled->current_position();
			double ypos = y_controlled->current_position();
			double apos = a_controlled->current_position();

			x_link.set_coord(xpos);
			y_link.set_coord(ypos);
			a_link.set_coord(apos);

			chain.update_location();
			outpos = chain.out()->global_location;

			auto outpos_corrected = invnullpos * outpos;

			x_axis.feedpos = outpos_corrected.translation().x;
			y_axis.feedpos = outpos_corrected.translation().y;
			a_axis.feedpos = outpos_corrected.rotation();

			// TODO: Реимплементировать через подчиненные оси
			x_axis.feedspd = x_axis.ctrspd;
			y_axis.feedspd = y_axis.ctrspd;
			a_axis.feedspd = a_axis.ctrspd;

			chain.update_location();
		}

		double * ctrspd_array() override { return ctrspd; }

		void apply_control()
		{
			int64_t time = ralgo::discrete_time();

			//double delta = (double)(time - lasttime) / 1000;
			lasttime = time;

			//_controlled_axes[0]->direct_control(_controlled_axes[0]->current_position() + ctrspd[0]*delta, ctrspd[0]);
			//_controlled_axes[1]->direct_control(_controlled_axes[1]->current_position() + ctrspd[1]*delta, ctrspd[1]);
			//_controlled_axes[2]->direct_control(_controlled_axes[2]->current_position() + ctrspd[2]*delta, ctrspd[2]);

			x_controlled->direct_control(x_controlled->current_position(), ctrspd[0]);
			y_controlled->direct_control(y_controlled->current_position(), ctrspd[1]);
			a_controlled->direct_control(a_controlled->current_position(), ctrspd[2]);
		}

		void print_info() override
		{
			nos::fprintln("current: {} {} {}", x_controlled->current_position(), y_controlled->current_position(), a_controlled->current_position());
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

		control_node* iterate(control_node* slt) override
		{
			if (slt == nullptr)
				return x_controlled;

			if (slt == x_controlled)
				return y_controlled;

			if (slt == y_controlled)
				return a_controlled;

			if (slt == a_controlled)
				return nullptr;
			else
				return nullptr;
		}
	};
}

#endif