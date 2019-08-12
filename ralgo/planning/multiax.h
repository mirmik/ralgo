#ifndef RALGO_MULTIAX_H
#define RALGO_MULTIAX_H

#include <ralgo/planning/traj.h>
#include <ralgo/planning/speed_deformer.h>
#include <rabbit/geom/curve2.h>

namespace ralgo
{
	template <class P = int64_t, class V = float, class A = float,
	          class T = time_t>
	class multiax_traj
	{
	private:
		int dim;
		T _start;

	public:
		multiax_traj(int dim) : dim(dim) {}

		void set_start_time(T time)
		{
			_start = time;
		}

		///Возвращает -1 при выходе за целевой интервал времени.
		///Иначе ноль. Фаза возваращается через указатель.
		//virtual int inloctime_placed(T time, phase<P, V, A> *phs, int idx) = 0;
		virtual int inloctime_placed(T time, phase<P, V, A> *phs) = 0;

		int inabstime(T time, phase<P, V, A> *phs)
		{
			return inloctime_placed(time - _start, phs);
		}
	};
	
	template <class P = int64_t, class V = float, class A = float,
	          class T = time_t>
	struct geom2d_trajectory
	{
		rabbit::bounded_curve2<V> * crv;
		ralgo::speed_deformer<P,V,A> * spddeform;
		float timemul;

		T fulltime;

		geom2d_trajectory(
		    rabbit::bounded_curve2<V>* crv,
		    ralgo::speed_deformer<P,V,A>* def,
		    float fulltime
		) 
			: crv(crv), spddeform(def), fulltime(fulltime)
		{}

		geom2d_trajectory(
		    rabbit::bounded_curve2<V>* crv,
		    float fulltime
		) 
			: crv(crv), fulltime(fulltime)
		{

		}

		int inloctime_placed(T time, phase<P, V, A> *phs)
		{
			T tkoeff = (fulltime - time) / fulltime;

//			PRINT(tkoeff);

			T time_to_traj_param_koeff = crv->tlength() / fulltime;

//			PRINT(time_to_traj_param_koeff);

			auto time_unit = (float)time / fulltime;
			auto traj_param = spddeform->posmod(time_unit) * crv->tfinish();

//			PRINT(traj_param);

			auto speed_modifier = spddeform->spdmod(time_unit) * time_to_traj_param_koeff;

			PRINT(traj_param);
			auto pos = crv->d0(traj_param);
			PRINT(pos);

			auto spd = crv->d1(traj_param) * speed_modifier;

			phs[0].d0 = pos[0];
			phs[1].d0 = pos[1];

			phs[0].d1 = spd[0];
			phs[1].d1 = spd[1];
		}
	};
}

#endif