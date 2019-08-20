#ifndef RALGO_MULTIAX_H
#define RALGO_MULTIAX_H

#include <ralgo/planning/traj.h>
#include <ralgo/planning/speed_deformer.h>

#include <rabbit/geom/curve2.h>

#include <ralgo/defs.h>

namespace ralgo
{
	template <class P = pos_t, class V = spd_t, class A = acc_t,
	          class T = tim_t>
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
	
	template <class P = pos_t, class V = spd_t, class A = acc_t,
	          class T = tim_t>
	struct geom2d_trajectory
	{
		rabbit::bounded_curve2<V> * crv;
		ralgo::speed_deformer* spddeform;
		float timemul;

		T fulltime;

		geom2d_trajectory(
		    rabbit::bounded_curve2<V>* crv,
		    ralgo::speed_deformer* def,
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
			//T tkoeff = (fulltime - time) / fulltime;

//			PRINT(tkoeff);

			T time_to_traj_param_koeff = crv->tlength() / fulltime;

//			PRINT(time_to_traj_param_koeff);

			auto time_unit = (float)time / fulltime;
			auto posmod = spddeform->posmod(time_unit);

			auto traj_param = posmod * crv->tfinish();

//			PRINT(traj_param);

			auto speed_modifier = spddeform->spdmod(time_unit) * time_to_traj_param_koeff;

			//PRINT(traj_param);
			auto pos = crv->d0(traj_param);
			//PRINT(pos);

			auto spd = crv->d1(traj_param) * speed_modifier;

			phs[0].d0 = pos[0];
			phs[1].d0 = pos[1];

			phs[0].d1 = spd[0];
			phs[1].d1 = spd[1];

			return posmod >= 1;
		}
	};

	#define RALGO_TRAJECTORY_FINISHED 1

	class traj1d 
	{
	public:
		virtual int inloctime(int64_t time, phase<int64_t, float> *phs) = 0;
	};

	class traj1d_line : public traj1d
	{
		int64_t ftime;
		int64_t length;

		float setted_speed;

	public:
		ralgo::speed_deformer spddeform;

		traj1d_line() {}
		
		traj1d_line(int64_t incpos, int64_t time) 
		{
			reset(incpos, time);
		}

		void reset(int64_t incpos, int64_t time) 
		{
			length = incpos;
			ftime = time;

			DPRINT(length);
			DPRINT(ftime);

			//while(1);

			setted_speed = incpos / time;

		}

		int inloctime(int64_t time, phase<int64_t, float> *phs) 
		{
			float time_unit = (float)time / ftime;

			auto posmod = spddeform.posmod(time_unit);
			auto spdmod = spddeform.spdmod(time_unit);

			float traj_param = posmod * length;
			float speed_modifier = spdmod;

			//DPRINT(time_unit);
			DPRINT(traj_param);
			DPRINT(speed_modifier);

			auto pos = traj_param;
			auto spd = setted_speed * speed_modifier;

			phs[0].d0 = pos;
			phs[0].d1 = spd;

			if (posmod >= 1) return 1;
			else return 0;
		}
	};
}

#endif