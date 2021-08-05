#ifndef RALGO_HEIMER_PHASE_SIGNALS_H
#define RALGO_HEIMER_PHASE_SIGNALS_H

#include <nos/fprint.h>
#include <ralgo/heimer/sigtypes.h>
#include <ralgo/heimer/heimer_types.h>
#include <ralgo/heimer/signal.h>

#include <ralgo/linalg/linalg.h>
#include <ralgo/space/pose3.h>
#include <ralgo/space/screw.h>

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
}

#define PHASE_SIGNAL_CLASS(clsname, postype, veltype, typeno)                                 \
	class clsname : public phase_signal_base<postype, veltype>                             \
	{                                                                                         \
	public:                                                                                   \
		clsname(const char * name) : phase_signal_base<postype, veltype>(name, typeno) {}; \
	};

namespace heimer
{
	using posvec2 = linalg::vec<position_t, 2>;
	using velvec2 = linalg::vec<velocity_t, 2>;
	using posvec3 = linalg::vec<position_t, 3>;
	using velvec3 = linalg::vec<velocity_t, 3>;
	using pospose3 = ralgo::pose3<position_t>;
	using velscr3 = ralgo::screw3<velocity_t>;

	PHASE_SIGNAL_CLASS(axis_state, position_t, velocity_t, SIGNAL_TYPE_AXIS_STATE)
	PHASE_SIGNAL_CLASS(phase3_state, posvec3, velvec3, SIGNAL_TYPE_PHASE3)
	PHASE_SIGNAL_CLASS(phase2_state, posvec2, velvec2, SIGNAL_TYPE_PHASE2)
	//PHASE_SIGNAL_CLASS(dof6_state, pospose3, velscr3, SIGNAL_TYPE_DOF6)
}

#endif