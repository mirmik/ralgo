#ifndef RALGO_MULTIAX_H
#define RALGO_MULTIAX_H

#include <ralgo/planning/traj.h>

namespace ralgo
{
	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct multiax_traj
	{
		///Возвращает -1 при выходе за целевой интервал времени.
		///Иначе ноль. Фаза возваращается через указатель.
		virtual int inloctime_placed(T time, phase<P, V, A> *phs, int idx) = 0;
	};

	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct multiax_traj_adaptor
	{
		struct multiax_traj *traj;
		uint8_t idx;

		int inloctime_placed(T time, phase<P, V, A> *phs) override 
		{
			return traj -> inloctime_placed(time, phs, idx);
		}
	};
} // namespace ralgo

#endif