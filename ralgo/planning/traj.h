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

	template <typename P = int64_t, typename V = float, typename A = float, typename T = time_t>
	struct accdcc_by_time_trajectory : public trajectory<P, V, A, T>
	{
		P x0;
		P x1;
		T t_acc;
		T t_lin;
		T t_dcc;
		T t01;

		P xacc;
		P xlin;

		A acc;
		A dcc;

		P x01;
		V v;

		accdcc_by_time_trajectory(P spos, P fpos, T t_acc, T t_lin, T t_dcc) 
			: x0(spos), x1(fpos), t_acc(t_acc), t_dcc(t_dcc), t_lin(t_lin)
		{
			t01 = t_acc + t_lin + t_dcc;
			x01 = x1 - x0;
			v = 2 * (V)x01 / (V)(t_acc + 2*t_lin + t_dcc);
			xacc = t_acc * v / 2;
			xlin = xacc + t_lin * v;

			A acc = v / t_acc;
			A dcc = - v / t_dcc;
		}

		int inloctime(T t, phase<P, V, A>* phs) override
		{
			if (t > t01) {
				phs->d0 = x1;
				phs->d1 = 0;
				phs->d2 = 0;
				return -1;
			}

			if (t < t_acc) 
			{
				phs->d2 = acc;
				phs->d1 = acc * t;
				phs->d0 = x0 + phs->d1 * t / 2;
			}
			else if (t < t_acc + t_lin) 
			{
				phs->d2 = 0;	
				phs->d1 = v;
				phs->d0 = xacc + v * t;
			}
			else 
			{
				phs->d2 = dcc;	
				phs->d1 = v + dcc * (t - t_acc - t_lin);
				phs->d0 = xlin + phs->d1 * (t - t_acc - t_lin) / 2;
			} 

			return 0;
		}
	};

}

#endif