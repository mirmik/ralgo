#ifndef RALGO_MULTIAX_H
#define RALGO_MULTIAX_H

#include <ralgo/planning/traj.h>

namespace ralgo
{
	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct multiax_traj
	{
		int dim;

		multiax_traj(int dim) : dim(dim) {}

		///Возвращает -1 при выходе за целевой интервал времени.
		///Иначе ноль. Фаза возваращается через указатель.
		//virtual int inloctime_placed(T time, phase<P, V, A> *phs, int idx) = 0;

		virtual int inloctime_placed(T time, phase<P,V,A> *phs) = 0;

		int inabstime(T time, phase<P, V, A> *phs)
		{
			return inloctime_placed(time - _start, phs);
		}
	};

	/*template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct multiax_traj_adaptor
	{
		struct multiax_traj *traj;
		uint8_t idx;

		int inloctime_placed(T time, phase<P, V, A> *phs) override 
		{
			return traj -> inloctime_placed(time, phs, idx);
		}
	};*/

	// Траектория дуги окружности для двух координат
	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct circle_traj 
	{
		float begangle;
		float finangle;
		float fintime;



		V spd;


		//rabbit::circle2 circ;

		circle_traj() {}

		int inloctime_placed(T time, phase<P, V, A> *phs) 
		{
			T tkoeff = (fintime - time) / fintime;

			T time_to_traj_param_koeff = crv.tlength() / fintime;

			auto traj_param = spddeform.posmod(t) * finangle;
			auto speed_modifier = spddeform.spdmod(t) * time_to_traj_param_koeff;

			auto pos = circ.d0(traj_param);
			auto spd = circ.d1(traj_param) * speed_modifier;
		}
	};


	struct geom2d_trajectory 
	{
		rabbit::trimmed_curve2 * crv;
		ralgo::speed_deformer * spddeform;
		float timemul;		
	};


} // namespace ralgo

#endif