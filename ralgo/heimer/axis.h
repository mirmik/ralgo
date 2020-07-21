#ifndef HEIMER_AXIS_H
#define HEIMER_AXIS_H

#include <ralgo/heimer/control.h>

namespace heimer
{
	template <class P, class V>
	class axis_node : public heimer::control_node
	{
	public:
		P feedpos;
		V feedspd;

		P ctrpos;
		V ctrspd;

		constexpr 
		axis_node(const char* mnemo) :
			control_node(mnemo)
		{}

		P feedback_position() { return feedpos; }
		P feedback_speed() { return feedspd; }

		P target_position() { return ctrpos; }
		P target_speed() { return ctrspd; }

		void restore_control(P pos, V spd) 
		{
			ctrpos = pos;
			ctrspd = spd;

			feedpos = pos;
			feedspd = spd;
		}

		void control(P pos, V spd) 
		{
			ctrpos = pos;
			ctrspd = spd;
		}
	};
}

#endif