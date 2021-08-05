#ifndef HEIMER_PHASE3_SIGNAL_H
#define HEIMER_PHASE3_SIGNAL_H

#include <igris/compiler.h>
#include <ralgo/heimer/heimer_types.h>
#include <ralgo/heimer/signal.h>

namespace heimer
{
	class phase3_signal : public signal_head
	{
		linalg::vec<position_t, 3> ctrpos = {};
		linalg::vec<velocity_t, 3> ctrvel = {};
		linalg::vec<position_t, 3> feedpos = {};
		linalg::vec<velocity_t, 3> feedvel = {};

	public:
		phase3_signal(const char * name);
		int info(char * data, int maxsize) override;
	}
}

#endif