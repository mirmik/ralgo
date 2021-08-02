#ifndef RALGO_HEIMER_DOF6_SIGNAL_H
#define RALGO_HEIMER_DOF6_SIGNAL_H

#include <rabbit/space/pose3.h>
#include <rabbit/space/screw.h>

#include <ralgo/heimer/signal.h>
#include <ralgo/heimer/heimer_types.h>

namespace heimer 
{
	class dof6_signal : public signal_head
	{
		rabbit::pose3<position_t> _pos;
		rabbit::screw3<velocity_t> _vel;

	public:
		dof6_signal() = default;
		dof6_signal(const char * name);

		void init(const char * name);
		int info(char * data, int maxsize) override;
	};
}

#endif