#ifndef RALGO_TRAJ_H
#define RALGO_TRAJ_H

#include <ralgo/planning/phase.h>
#include <gxx/datastruct/dlist.h>

namespace ralgo
{

	template <typename P = int64_t, typename V = float, typename A = float, typename T = time_t>
	struct trajectory
	{
		void set_start_time(T time)
		{
			_start = time;
		}

		int inabstime(T time, phase<P, V, A>* phs)
		{
			return inloctime(time - _start, phs);
		}

		///Возвращает -1 при выходе за целевой интервал времени.
		///Иначе ноль. Фаза возваращается через указатель.
		virtual int inloctime(T time, phase<P, V, A>* phs) = 0;

	public:
		dlist_head lnk; // Для подключения в стэк траекторий.

	private:
		T _start;
	};

	/// Траектория удержания текущей позиции. Бесконечное время действия.
	template <typename P = int64_t, typename V = float, typename A = float, typename T = time_t>
	struct keep_trajectory : public trajectory<P, V, A, T>
	{
		P x;

		keep_trajectory(P pos) : x(pos) {};

		int inloctime(T t, phase<P, V, A>* phs) override
		{
			phs->d0 = x;
			phs->d1 = 0;
			phs->d2 = 0;

			return 0;
		}
	};

	///Траектория движения с постоянной скоростью. Бесконечное время действия.
	template <typename P = int64_t, typename V = float, typename A = float, typename T = time_t>
	struct jog_trajectory : public trajectory<P, V, A, T>
	{
		P x0;
		V v;

		jog_trajectory(P startpos, V speed) : x0(startpos), v(speed) {};

		int inloctime(T t, phase<P, V, A>* phs) override
		{
			phs->d0 = x0 + v * t;
			phs->d1 = v;
			phs->d2 = 0;

			return 0;
		}
	};

	template <typename P = int64_t, typename V = float, typename A = float, typename T = time_t>
	struct line_by_time_trajectory : public trajectory<P, V, A, T>
	{
		P x0;
		P x1;
		T t01;

		P x01;
		V v;

		line_by_time_trajectory(P spos, P fpos, T interval) : x0(spos), x1(fpos), t01(interval)
		{
			x01 = x1 - x0;
			v = (V)x01 / (V)t01;
		}

		int inloctime(T t, phase<P, V, A>* phs) override
		{
			if (t > t01) {
				phs->d0 = x1;
				phs->d1 = 0;
				phs->d2 = 0;
				return -1;
			}

			phs->d0 = x0 + ((float)t / (float)t01) * x01;
			phs->d1 = v;
			phs->d2 = 0;

			return 0;
		}
	};

}

#endif