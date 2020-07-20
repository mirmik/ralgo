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
	class traj1d
	{
	public:
		virtual int attime(int64_t time, P& pos, V& spd) = 0;
		virtual bool is_finished(int64_t time) = 0;
	};


	template <class P, class V>
	class traj1d_line : public traj1d<P,V>
	{
	public:
		int64_t stim = 0;
		int64_t ftim = 0;

		P spos = 0;
		P fpos = 0;

		V setted_speed = 0;

	public:
		ralgo::speed_deformer spddeform;

		void reset(P spos, int64_t stim, P fpos, int64_t ftim)
		{
			this->spos = spos;
			this->fpos = fpos;

			this->stim = stim;
			this->ftim = ftim;

			DPRINT(spos);
			DPRINT(fpos);

			DPRINT(stim);
			DPRINT(ftim);

			if (stim == ftim)
				setted_speed = 0;
			else
			{
				setted_speed = (float)(fpos - spos) / (ftim - stim);
			}
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

			if (posmod >= 1) return 1;
			else return 0;
		}

		void set_standart_accdcc_patern(int64_t acctime, int64_t dcctime, float nominal_speed)
		{
			float koeff = setted_speed / nominal_speed;

			float acc = acctime * koeff / (ftim - stim);
			float dcc = dcctime * koeff / (ftim - stim);

			if (acc + dcc > 1)
			{
				//dprln("warn: acc + dcc > 1");

				acc = 0.5;
				dcc = 0.5;
			}
			//dprln(acc, dcc);

			spddeform.reset(
			    acc, dcc

			    //	acc,
			    //	dcc);
			);
		}

		void set_speed_pattern(float acc, float dcc, float speed) 
		{
			// Чтобы расчитать интервал времени разгона, необходимо
			// соотнести значение ускорения и скорости.
			// Здесь acc - тангенс угла, 
			// setted_speed - установленная скорость в дискреных единицах
			// тогда setted_speed / acc = acc_time в дискретных единицах
			float time = ftim - stim;

			float acc_time = speed / acc * ralgo::discrete_time_frequency();
			float dcc_time = speed / dcc * ralgo::discrete_time_frequency();

			float acc_part = acc_time / time;
			float dcc_part = dcc_time / time;

			//std::cout << dcc_part << std::endl;

			spddeform.reset2(acc_part, dcc_part);
		}

		void set_stop_trajectory(P curpos, V curspd, V dccval) 
		{
			// скоростной деформатор работает с точным выведением в позицию, и изменяет время,
			// поэтому подменяем время в два раза, чтобы соответствовать равнозамедленному паттерну.

			stim = ralgo::discrete_time();
			float realdiff = (fabs(curspd) / dccval);
			ftim = stim + realdiff / 2 * ralgo::discrete_time_frequency();

			assert(ftim >= stim);

			spos = curpos;
			fpos = curpos + curspd * realdiff / 2;

			setted_speed = curspd / ralgo::discrete_time_frequency();
			//setted_speed = (float)(fpos - spos) / (ftim - stim);

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