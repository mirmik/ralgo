#ifndef RALGO_HEIMER_PHASE_SIGNALS_H
#define RALGO_HEIMER_PHASE_SIGNALS_H

#include <nos/fprint.h>
#include <ralgo/heimer/sigtypes.h>
#include <ralgo/heimer/heimer_types.h>
#include <ralgo/heimer/signal.h>

namespace heimer 
{
	template <class P, class V>
	class phase_signal_base : public signal_head
	{
	public:
		P ctrpos = {};
		V ctrvel = {};

		P feedpos = {};
		V feedvel = {};

	public:
		phase_signal_base(const char * name, uint8_t type)
			: signal_head(name, type)
		{}

		int info(char * data, int) override
		{
			nos::format_buffer(data, "(cpos:{}, cvel:{}, fpos:{}, fvel:{})\r\n", ctrpos, ctrvel, feedpos, feedvel);
			return 0;
		}
	};

	class axis_state : public phase_signal_base<position_t, velocity_t>
	{
	public:
		axis_state(const char * name) : phase_signal_base<position_t, velocity_t>(name, SIGNAL_TYPE_AXIS_STATE) {};
	};
}

#endif