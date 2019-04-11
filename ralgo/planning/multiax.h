#ifndef RALGO_MULTIAX_H
#define RALGO_MULTIAX_H

#include <ralgo/planning/traj.h>

namespace ralgo
{
	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct multiax_traj
	{
	};

	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct multiax_traj_adaptor
	{
		struct multiax_traj *traj;
		uint8_t idx;
	};
} // namespace ralgo

#endif