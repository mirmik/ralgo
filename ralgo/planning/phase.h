#ifndef RALGO_PHASE_H
#define RALGO_PHASE_H

#include <nos/fprint.h>
#include <ralgo/defs.h>

namespace ralgo
{
	template <typename P = pos_t, typename V = spd_t, typename A = acc_t>
	struct phase
	{
		using pos_t = P;
		using spd_t = V;
		using acc_t = A;

		P d0;
		V d1;
		A d2;

		phase() = default;
		phase(P p, V v, A a) : d0(p), d1(v), d2(a)
		{
		}

		ssize_t print_to(nos::ostream& os) const
		{
			return nos::fprint_to(os, "phase({},{},{})", d0, d1, d2);
		}
	};
} // namespace ralgo

#endif