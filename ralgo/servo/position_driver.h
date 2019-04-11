#ifndef RALGO_SPDPOS_DRIVER
#define RALGO_SPDPOS_DRIVER

#include <ralgo/planning/phase.h>

namespace ralgo
{
	template <typename P = int64_t, typename V = float, typename A = float>
	struct position_driver
	{
		virtual void serve(const ralgo::phase<P, V, A> &phs) = 0;
	};
} // namespace ralgo

#endif