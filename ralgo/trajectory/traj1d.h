#ifndef HEIMER_TRAJ1D_H
#define HEIMER_TRAJ1D_H

#include <ralgo/trajectory/speed_deformer.h>
#include <ralgo/disctime.h>
#include <nos/fprint.h>

#define HEIMER_TRAJ1D_CONTINUE 0
#define HEIMER_TRAJ1D_FINISHED 1

namespace ralgo
{
	template <class P, class V>
	struct traj1d_nominal_speed_params
	{
		int64_t stim;
		P spos;
		P fpos;

		V speed;
		V acc;
		V dcc;
	};

	template <class P, class V>
	struct traj1d_timestamp_params
	{
		int64_t stim;
		int64_t ftim;
		int64_t acctime;
		int64_t dcctime;
		P spos;
		P fpos;
	};

	template <class P, class V>
	class traj1d
	{
	public:
		virtual int attime(int64_t time, P& pos, V& spd) = 0;
		virtual bool is_finished(int64_t time) = 0;
	};


	template <class P, class V>
	class traj1d_line : public traj1d<P, V>
	{
	public:
		int64_t stim = -1;
		int64_t ftim = 0;

		P spos = 0;
		P fpos = 0;

		V setted_speed = 0;

	public:
		ralgo::speed_deformer spddeform;

		void init_timestamp_mode(
			struct traj1d_timestamp_params<P,V> * args)
		{
			spos = args->spos;
			fpos = args->fpos;

			stim = args->stim;
			ftim = args->ftim;

			if (stim == ftim)
				setted_speed = 0;
			else
			{
				setted_speed = 
					(float)(fpos - spos) / (ftim - stim);
			}

			float acc_part = (float)args->acctime / (float)(ftim - stim);
			float dcc_part = (float)args->dcctime / (float)(ftim - stim);

			if (acc_part + dcc_part > 1)
			{
				acc_part = 0.5;
				dcc_part = 0.5;
			}

			spddeform.set_time_pattern(
			    acc_part, dcc_part
			);
		}

		void init_nominal_speed_mode(
		    struct traj1d_nominal_speed_params<P,V> * args
		)
		{
			setted_speed = args->speed / 
				ralgo::discrete_time_frequency();

			P dist = args->fpos - args->spos;
			float time = std::fabs(dist) / setted_speed;

			spos = args->spos;
			fpos = args->fpos;
			
			stim = args->stim;
			ftim = stim + (int64_t)time;

			float acc_time = args->speed / args->acc
				* ralgo::discrete_time_frequency();
			float dcc_time = args->speed / args->dcc
				* ralgo::discrete_time_frequency();

			float acc_part = acc_time / time;
			float dcc_part = dcc_time / time;

			// Учёт возможного треугольного паттерна осуществляется
			// здесь:
			spddeform.set_speed_pattern(acc_part, dcc_part);	
		}


		bool is_finished(int64_t time) override
		{
			return time > ftim;
		}

		int attime(int64_t time, P& pos, V& spd) override
		{
			// Умножение на коэффициент времени перерасщитывает скорость
			// взятую на дискретную единицу времени в скорость взятую
			// на единицу времени рабочего пространства.
			float time_unit = (float)(time - stim) / (ftim - stim);

			auto posmod = spddeform.posmod(time_unit);
			auto spdmod = spddeform.spdmod(time_unit);

			pos = fpos * posmod + spos * (1 - posmod);
			spd = setted_speed * spdmod * ralgo::discrete_time_frequency();

			return spddeform.is_finished(time_unit) ? 1 : 0;
		}

		/*void set_time_pattern(int64_t acctime, int64_t dcctime, float nominal_speed)
		{
			float koeff = setted_speed / nominal_speed;

			float acc = acctime * koeff / (ftim - stim);
			float dcc = dcctime * koeff / (ftim - stim);

			if (acc + dcc > 1)
			{
				acc = 0.5;
				dcc = 0.5;
			}

			spddeform.set_time_pattern(
			    acc, dcc
			);
		}*/

		/*void set_accdcc_pattern(float acc, float dcc)
		{
			// Чтобы расчитать интервал времени разгона, необходимо
			// соотнести значение ускорения и скорости.
			// Здесь acc - тангенс угла,
			// setted_speed - установленная скорость в дискреных единицах
			// тогда setted_speed / acc = acc_time в дискретных единицах
			float time = ftim - stim;

			float acc_time = setted_speed / acc * ralgo::discrete_time_frequency();
			float dcc_time = setted_speed / dcc * ralgo::discrete_time_frequency();

			float acc_part = acc_time / time;
			float dcc_part = dcc_time / time;

			spddeform.set_speed_pattern(acc_part, dcc_part);
		}*/

		void set_stop_trajectory(P curpos, V curspd, V dccval)
		{
			// скоростной деформатор работает с точным выведением в позицию, и изменяет время,
			// поэтому подменяем время в два раза, чтобы соответствовать равнозамедленному паттерну.

			stim = ralgo::discrete_time();
			float realdiff = (fabs(curspd) / dccval);
			ftim = stim + (int)(realdiff * ralgo::discrete_time_frequency() / 2);

			assert(ftim >= stim);

			spos = curpos;
			fpos = curpos + curspd * realdiff / 2;

			setted_speed = curspd / ralgo::discrete_time_frequency();
			spddeform.set_stop_pattern();
		}

		void set_point_hold(P pos)
		{
			ftim = ralgo::discrete_time();
			stim = ftim - 1;

			spos = pos;
			fpos = pos;

			spddeform.set_stop_pattern();
		}

		ssize_t print_to(nos::ostream& out) const
		{
			return nos::fprint_to(out, "stim:{} ftim:{} spos:{} fpos:{} spd:{}",
			                      stim, ftim, spos, fpos, setted_speed);
		}
	};
}

#endif