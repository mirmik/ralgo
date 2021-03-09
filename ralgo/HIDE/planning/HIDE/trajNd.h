#ifndef RALGO_PLANNING_TRAJ_ND_H
#define RALGO_PLANNING_TRAJ_ND_H

#include <nos/fprint.h>
#include <ralgo/planning/phase.h>
#include <ralgo/planning/speed_deformer.h>

namespace ralgo
{
	template <size_t Dim, class P, class V>
	class trajNd
	{
	protected:
		// Время в дискретных единицах времени. 
		// (см. ralgo::discrete_time_frequency) 
		int64_t stim;
		int64_t ftim;

	public:
		virtual int attime(
		    int64_t time,
		    igris::array_view<P> pos,
		    igris::array_view<V> spd,
		    int64_t time_multiplier
		) = 0;
		
		virtual bool is_finished(int64_t time) 
		{
			return time > ftim;
		}
	};

	template <size_t Dim, class P, class V>
	class trajNd_line : public trajNd<Dim, P, V>
	{
	public:
		using trajNd<Dim, P, V>::stim;
		using trajNd<Dim, P, V>::ftim;

		P spos[Dim];
		P fpos[Dim];

		V setted_speed[Dim];

	public:
		ralgo::speed_deformer spddeform;

		void set_start_position(int i, P pos) 
		{
			spos[i] = pos;
		}

		void set_finish_position_inc(int i, P inc) 
		{
			fpos[i] = spos[i] + inc;
		}

		// Инициализировать траекторию, когда spos и fpos уже установлены.
		int reset(
		    int64_t stim,
		    int64_t ftim
		)
		{
			this->stim = stim;
			this->ftim = ftim;

			assert(stim != ftim);

			for (unsigned int i = 0; i < Dim; ++i)
			{
				if (stim == ftim)
					setted_speed[i] = 0;
				else
				{
					setted_speed[i] = ((float)(fpos[i] - spos[i])) / ((float)(ftim - stim));
				}
			}

			return 0;
		}

		int reset(
		    igris::array_view<P>& spos,
		    int64_t stim,
		    igris::array_view<P>& fpos,
		    int64_t ftim
		)
		{
			std::copy(std::begin(spos), std::end(spos), std::begin(this->spos));
			std::copy(std::begin(fpos), std::end(fpos), std::begin(this->fpos));
			reset(stim, ftim);

			return 0;
		}

		int attime(int64_t time, 
			igris::array_view<P> pos, 
			igris::array_view<V> spd,
			int64_t time_multiplier) override
		{
			// Умножение на коэффициент времени перерасщитывает скорость
			// взятую на дискретную единицу времени в скорость взятую
			// на единицу времени рабочего пространства.  

			float time_unit = ((float)(time - stim)) / ((float)(ftim - stim));
			auto posmod = spddeform.posmod(time_unit);
			auto spdmod = spddeform.spdmod(time_unit);

			for (unsigned int i = 0; i < Dim; ++i) {
				pos[i] = fpos[i] * posmod + spos[i] * (1 - posmod);
				spd[i] = setted_speed[i] * spdmod * time_multiplier;
			}

			if (posmod >= 1) return 1;
			else return 0;
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

			//ralgo::speed_deformer::acc_dcc_balance(acc_part, dcc_part);

			spddeform.reset2(acc_part, dcc_part);
		}

		void set_stop_trajectory(igris::array_view<P> curpos, igris::array_view<V> curspd, V dccval) 
		{
			// скоростной деформатор работает с точным выведением в позицию, и изменяет время,
			// поэтому подменяем время в два раза, чтобы соответствовать равнозамедленному паттерну.

			stim = ralgo::discrete_time();
			float realdiff = ralgo::vecops::length(curspd) / dccval;
			ftim = stim + realdiff / 2 * ralgo::discrete_time_frequency();

			//assert(ftim >= stim);
			if (ftim < stim) { ftim = stim + 1; }

			std::copy(std::begin(curpos), std::end(curpos), std::begin(this->spos));
			for (unsigned int i = 0; i < Dim ; ++i) 
				fpos[i] = spos[i] + curspd[i] * realdiff / 2;

			for (unsigned int i = 0; i < Dim ; ++i) 
				setted_speed[i] = curspd[i] / ralgo::discrete_time_frequency();
			
			spddeform.set_stop_pattern();
		}

		ssize_t print_to(nos::ostream& os) const 
		{
			return nos::fprint_to(os, "({},{},{})", spos, fpos, setted_speed);
		}
	};
}

#endif