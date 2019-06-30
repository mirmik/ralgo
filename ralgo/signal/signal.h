#ifndef RALGO_SIGNAL_SIGNAL_H
#define RALGO_SIGNAL_SIGNAL_H

#include <igris/util/bug.h>
#include <nos/print.h>

namespace ralgo
{
	namespace signal
	{
		/**
			lerp_values строит значения функции, заданной на отрезках вектором точек vals/stamp,
			в точках, заданных фектором keys. Значения точек линейно интерполируются. 
		*/

		template<class VI, class SI, class KI, class RI>
		void lerp_values(
		    VI vit, const VI vend,
		    SI sit, const SI send,
		    KI kit, const KI kend,
		    RI rit)
		{
			SI sitnext = std::next(sit);
			SI vitnext = std::next(vit);

			for (; kit != kend; kit++, rit++)
			{
				auto key = *kit;

				while(key > *sitnext) { 
					sit++; sitnext++; 
					vit++; vitnext++;
				
					if (sitnext == send) 
						BUG(); 
				} 

				auto vleft = *vit;
				auto vright = *vitnext;
				auto sleft = *sit;
				auto sright = *sitnext;
				auto lkoeff = ralgo::lerpkoeff(sleft, sright, key);
				auto lerp = ralgo::lerp(vleft, vright, lkoeff);

				/*nos::println();
				PRINT(vleft);
				PRINT(vright);
				PRINT(sleft);
				PRINT(sright);
				PRINT(key);
				PRINT(lkoeff);
				PRINT(lerp);*/

				*rit = lerp;
			}
		}

		template<class V, class S, class K>
		std::vector<value_t<V>> lerp_values(const V& vals, const S& stamp, const K& keys)
		{
			std::vector<value_t<V>> r(keys.size());

			/*PRINT(vals);
			PRINT(stamp);
			PRINT(keys);*/

			lerp_values(
			    vals.begin(), vals.end(),
			    stamp.begin(), stamp.end(),
			    keys.begin(), keys.end(),
			    r.begin());

			return r;
		}
	}
}

#endif