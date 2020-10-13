#ifndef RALGO_SPEED_DEFORMATION_H
#define RALGO_SPEED_DEFORMATION_H

#include <hal/irq.h>

#include <math.h>
#include <assert.h>

/**
	Деформаторы скорости для траекторных проходов.

	Деформаторы скорости нужны для того, чтобы наложить скоростной паттерн на
	траекторию описываемую уравнением s(t).

	Деформатор скорости D следует применять как:
	s = s(D_pos(t / tfull) * t)
	x = D_spd(t / tfull) \dot{s}(D_pos(t / tfull) * t)
*/

namespace ralgo
{
	class speed_deformer
	{
	public:
		float acc = 0;
		float dcc = 0;
		float f_time = 1;

		float strt_spd = 0;
		float fini_spd = 0;
		float real_spd = 0;

		float fini_acc_pos = 0;
		float strt_dcc_pos = 0;

	public:
		bool is_finished(float t) 
		{
			return t >= f_time;
		}

		// Вариант движения с увеличением максимальной скорости.
		void set_time_pattern(float acc, float dcc, float sspd = 0, float fspd = 0)
		{
			this->acc = acc;
			this->dcc = dcc;
			this->f_time = 1;

			strt_spd = sspd;
			fini_spd = fspd;

			// Формула выводится из равенства площадей под идеальной и реальной кривыми.
			real_spd =
			    (f_time - strt_spd * acc / 2 - fini_spd * dcc / 2) /
			    (f_time - acc / 2 - dcc / 2);

			//Вычисляем коэффициенты позиции в точках окончания участков.
			fini_acc_pos = (strt_spd + real_spd) * acc / 2;
			strt_dcc_pos = fini_acc_pos + real_spd * (f_time - acc - dcc);
		}

		[[deprecated]]
		void reset(float acc, float dcc, float sspd = 0, float fspd = 0) 
		{
			set_time_pattern(acc, dcc, sspd, fspd);
		}

		// Второй вариант инициализации, когда вместо увеличения максимальной скорости
		// расширяется время довода изделия.
		void set_speed_pattern(float acc, float dcc, float sspd = 0, float fspd = 0)
		{
			this->acc = acc;
			this->dcc = dcc;

			strt_spd = sspd;
			fini_spd = fspd;
			real_spd = 1;

			if (acc + dcc < 2) 
			{
				f_time = 1 
					+ (1 - sspd) * acc / 2
					+ (1 - fspd) * dcc / 2;	
			}
			else 
			{
				real_spd = sqrt(2 / (acc + dcc));
				this->acc = acc * real_spd;
				this->dcc = dcc * real_spd;	
				f_time = this->acc + this->dcc;
			}

			//Вычисляем коэффициенты позиции в точках окончания участков.
			fini_acc_pos = (strt_spd + real_spd) * this->acc / 2;
			strt_dcc_pos = fini_acc_pos + real_spd 
				* (f_time - this->acc - this->dcc);
		}

		[[deprecated]]
		void reset2(float acc, float dcc, float sspd = 0, float fspd = 0) 
		{
			set_speed_pattern(acc, dcc, sspd, fspd);
		}

		// Интеграл коэффициента деформации
		float posmod(float t)
		{
			if (t >= f_time)
			{
				return 1;
			}

			if (t < acc)
			{
				return
				    t * strt_spd
				    + t * (t / acc * (real_spd - strt_spd) / 2);
			}

			if (t < f_time - dcc)
			{
				return
				    fini_acc_pos
				    + real_spd * (t - acc);
			}

			else
			{
				auto loct = t - f_time + dcc;
				return strt_dcc_pos
				       + (loct) * real_spd
				       - (loct) * ((loct) / dcc * (real_spd - fini_spd)) / 2;
			}
		}

		// Коэффициент деформации
		float spdmod(float t)
		{
			if (t >= f_time)
			{
				assert(!isnan(fini_spd));
				return fini_spd;
			}

			if (t < acc)
			{
				float k = t / acc;
				assert(!isnan(k));
				assert(!isnan(strt_spd));
				assert(!isnan(real_spd));
				return strt_spd * (1 - k) + real_spd * k;
			}

			else if (t < f_time - dcc)
			{
				assert(!isnan(real_spd));
				return real_spd;
			}

			else
			{
				float k = (f_time - t) / dcc;
				
				if (isnan(k)) 
				{
					irqs_disable();
					DPRINT(f_time);
					DPRINT(dcc);
					DPRINT(t); 
				}

				assert(dcc != 0);
				assert(!isnan(f_time));
				assert(!isnan(t));
				assert(!isnan(k));
				assert(!isnan(strt_spd));
				assert(!isnan(real_spd));
				return fini_spd * (1 - k) + real_spd * k;
			}
		}

		// Сбросить состояние.
		void nullify()
		{
			strt_spd = fini_spd = real_spd = 0;
			f_time = 1;
		}

		// Паттерн остановки с линейным убыванием скорости.
		void set_stop_pattern() 
		{
			acc = 0;
			dcc = 2;
			f_time = 2;

			strt_spd = 1;
			fini_spd = 0;
			real_spd = 1;

			fini_acc_pos = 0;
			strt_dcc_pos = 0;
		}

	private:
		static void acc_dcc_balance(float& acc, float& dcc)
		{
			float sum = acc + dcc;

			if (sum > 1.0)
			{
				acc = acc / sum;
				dcc = dcc / sum;
			}
		}
	};
}

#endif