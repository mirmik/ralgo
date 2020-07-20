#ifndef HEIMER_AXISCTR_H
#define HEIMER_AXISCTR_H

#include <ralgo/log.h>
#include <ralgo/heimer/axis.h>
#include <ralgo/trajectory/traj1d.h>

#include <igris/math.h>

namespace heimer
{
	template <class P, class V>
	class axisctr : public control_node
	{
		struct dlist_head axes_list = DLIST_HEAD_INIT(axes_list);

		P offset = 0;

		V spd = 1;
		V acc = 1;
		V dcc = 1;

		V maxspd = 1;
		V maxacc = 1;
		V maxdcc = 1;

		float gain = 1;

		heimer::axis_node<P, V> * controlled;

		ralgo::traj1d<P, V> *    curtraj = nullptr;
		ralgo::traj1d_line<P, V> lintraj;

		bool _limited = false;
		P _forw;
		P _back;

	public:
		constexpr
		axisctr(
		    const char * mnemo,
		    heimer::axis_node<P, V> * controlled
		) :
			control_node(mnemo),
			controlled(controlled)
		{}

		int jog(int direction);
		int incmove(P dist);
		int absmove(P pos);

		int incmove_unsafe(P dist);
		int absmove_unsafe(P pos);

		P feedback_position() { return controlled->feedpos; }
		V feedback_speed()    { return controlled->feedspd; }

		P target_position() { return controlled->ctrpos; }
		V target_speed()    { return controlled->ctrspd; }

		V setted_speed()        { return spd; }
		V setted_acceleration() { return acc; }
		V setted_deceleration() { return dcc; }

		void set_speed(V spd)        { this->spd = spd; }
		void set_acceleration(V acc) { this->acc = acc; }
		void set_deceleration(V dcc) { this->dcc = dcc; }
		void set_offset(P offset)    { this->offset = offset; }

		void serve();
	};

	template <class P, class V>
	int axisctr<P, V>::incmove(P dist)
	{
		dist = dist * gain;

		if (flags && HEIM_IS_ACTIVE)
		{
			ralgo::warn("axisctr: not active");
			return -1;
		}

		P curpos = target_position();
		P tgtpos = curpos + dist;

		if (_limited)
			tgtpos = igris::clamp(tgtpos, _back, _forw);

		P ndist = tgtpos - curpos;

		return incmove_unsafe(ndist);
	}

}

#endif