#ifndef RALGO_PHASE_H
#define RALGO_PHASE_H

namespace ralgo 
{
	template <typename P = int64_t, typename V = float, typename A = float>
	struct phase
	{
		using pos_t = P;
		using spd_t = V;
		using acc_t = A;

		P d0;
		V d1;
		A d2;
	};

	template<class PHASE> using pos_t = typename PHASE::pos_t;
	template<class PHASE> using spd_t = typename PHASE::spd_t;
	template<class PHASE> using acc_t = typename PHASE::acc_t;
}

#endif