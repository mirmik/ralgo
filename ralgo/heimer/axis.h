#ifndef HEIMER_AXIS_H
#define HEIMER_AXIS_H

#include <ralgo/heimer/control.h>

namespace heimer
{
	template <class P, class V>
	class axis_node : public heimer::control_node
	{
	public:
		P feedpos = 0;
		V feedspd = 0;

		P ctrpos = 0;
		V ctrspd = 0;

		constexpr 
		axis_node(const char* mnemo) :
			control_node(mnemo)
		{}

		void print_info() override 
		{
			nos::println("ctrpos:", ctrpos);
			nos::println("ctrspd:", ctrspd);
			nos::println("feedpos:", feedpos);
			nos::println("feedspd:", feedspd);
		}

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

		virtual void hardstop() 
		{
			ctrpos = feedpos;
			ctrspd = 0;
		}
	};
}

#endif