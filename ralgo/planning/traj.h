#ifndef RALGO_TRAJ_H
#define RALGO_TRAJ_H

#include <igris/datastruct/dlist.h>
#include <ralgo/planning/phase.h>
#include <utility>

namespace ralgo
{
	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct trajectory
	{
		void set_start_time(T time)
		{
			_start = time;
		}

		int inabstime(T time, phase<P, V, A> *phs)
		{
			return inloctime_placed(time - _start, phs);
		}

		///Возвращает -1 при выходе за целевой интервал времени.
		///Иначе ноль. Фаза возваращается через указатель.
		virtual int inloctime_placed(T time, phase<P, V, A> *phs) = 0;

		std::pair<phase<P, V, A>, int> inloctime(T time)
		{
			phase<P, V, A> phs;
			auto ret = inloctime_placed(time, &phs);
			return std::make_pair(phs, ret);
		}

	  public:
		dlist_head lnk; // Для подключения в стэк траекторий.

	  private:
		T _start;
	};

	/// Траектория удержания текущей позиции. Бесконечное время действия.
	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct keep_trajectory : public trajectory<P, V, A, T>
	{
		P x;

		keep_trajectory() : x(0)
		{
		}

		keep_trajectory(P pos) : x(pos){};

		int inloctime_placed(T t, phase<P, V, A> *phs) override
		{
			phs->d0 = x;
			phs->d1 = 0;
			phs->d2 = 0;

			return 0;
		}
	};

	///Траектория движения с постоянной скоростью. Бесконечное время действия.
	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct jog_trajectory : public trajectory<P, V, A, T>
	{
		P x0;
		V v;

		jog_trajectory(P startpos, V speed) : x0(startpos), v(speed){};

		int inloctime_placed(T t, phase<P, V, A> *phs) override
		{
			phs->d0 = x0 + v * t;
			phs->d1 = v;
			phs->d2 = 0;

			return 0;
		}
	};

	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct line_by_time_trajectory : public trajectory<P, V, A, T>
	{
		P x0;
		P x1;
		T t01;

		P x01;
		V v;

		line_by_time_trajectory(P spos, P fpos, T interval)
			: x0(spos), x1(fpos), t01(interval)
		{
			x01 = x1 - x0;
			v = (V)x01 / (V)t01;
		}

		int inloctime_placed(T t, phase<P, V, A> *phs) override
		{
			if (t > t01)
			{
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

	template <class P = int64_t, class V = float, class A = float,
			  class T = time_t>
	struct accdcc_by_time_trajectory : public trajectory<P, V, A, T>
	{
		P x0;
		P x1;
		T t_acc; ///< Acceleration time
		T t_lin; ///< Carrier time
		T t_dcc; ///< Decceleration time
		T t01;   ///< Full time

		P xacc;

		A acc;
		A dcc;

		P x01;
		V v;

		accdcc_by_time_trajectory() = default;

		accdcc_by_time_trajectory(P spos, P fpos, T t_acc, T t_lin, T t_dcc)
		{
			init(spos, fpos, t_acc, t_lin, t_dcc);
		}

		void init(P spos, P fpos, T t_acc, T t_lin, T t_dcc)
		{
			x0 = spos;
			x1 = fpos;
			this->t_acc = t_acc;
			this->t_dcc = t_dcc;
			this->t_lin = t_lin;

			t01 = t_acc + t_lin + t_dcc;
			x01 = x1 - x0;

			v = 2 * (V)x01 / (V)(t_acc + 2 * t_lin + t_dcc);
			xacc = t_acc * v / 2;

			acc = v / t_acc;
			dcc = -v / t_dcc;
		}

		int inloctime_placed(T t, phase<P, V, A> *phs) override
		{
			if (t > t01)
			{
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
				phs->d0 = xacc + v * (t - t_acc);
			}
			else
			{
				phs->d2 = dcc;
				phs->d1 = -dcc * (t01 - t);
				phs->d0 = x1 - phs->d1 * (t01 - t) / 2;
			}

			return 0;
		}
	};

} // namespace ralgo

#endif