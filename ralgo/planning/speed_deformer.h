#ifndef RALGO_SPEED_DEFORMATION_H
#define RALGO_SPEED_DEFORMATION_H

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
		float acc;
		float dcc;
		//int32_t s_time;
		float f_time;

		float strt_spd;
		float fini_spd;
		//float nati_spd;
		float real_spd;

		float fini_acc_pos;
		float strt_dcc_pos;

	public:
		speed_deformer() {}

		void reset(float acc, float dcc, float sspd = 0, float fspd = 0)
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

		speed_deformer& operator = (const speed_deformer& oth) = default;

		speed_deformer(float acc, float dcc, float sspd = 0, float fspd = 0)
			: acc(acc), dcc(dcc), strt_spd(sspd), fini_spd(fspd)
		{
			reset(acc, dcc, sspd, fspd);
		}

		float posmod(float t)
		{
//			DPRINT(acc);
//			DPRINT(dcc);
//			DPRINT(f_time);
//			DPRINT(t);

			//while(1);
			if (t >= f_time)
			{
				return 1;
			}

			if (t < acc)
			{
				//dprln("variant 1");
				//return
				//    t * (real_spd + strt_spd) / 2;
				return
				    t * strt_spd
				    + t * (t / acc * (real_spd - strt_spd) / 2);
			}

			if (t < f_time - dcc)
			{
				//dprln("variant 2");
				return
				    fini_acc_pos
				    + real_spd * (t - acc);
			}

			else
			{
				//dprln("variant 3");
				auto loct = t - f_time + dcc;
				return strt_dcc_pos
				       + (loct) * real_spd
				       - (loct) * ((loct) / dcc * (real_spd - fini_spd)) / 2;
			}
		}

		float spdmod(float t)
		{
			if (t >= f_time)
			{
				return fini_spd;
			}

			if (t < acc)
			{
				float k = t / acc;
				return strt_spd * (1 - k) + real_spd * k;
			}

			else if (t < f_time - dcc)
			{
				return real_spd;
			}

			else
			{
				float k = (1 - t) / dcc;
				return fini_spd * (1 - k) + real_spd * k;
			}
		}

	};

	class speed_deformed
	{
		int64_t stim, ftim;

	public:
		ralgo::speed_deformer spddeform;		

		speed_deformed(){}

		speed_deformed(int64_t stim, int64_t ftim) : 
			stim(stim), ftim(ftim)
		{}
	};
}

#endif