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
	struct speed_deformer
	{
		virtual float spdmod(float param) = 0;
		virtual float posmod(float param) = 0;
	};

	struct acc_speed_deformer : public speed_deformer
	{
		float start_speed;
		float finacc_position;
		float evaluated_linear_speed;

		//float lin;
		float acc;

		acc_speed_deformer(float acc, float sspd = 0)
			: acc(acc), start_speed(sspd)
		{
			evaluated_linear_speed =
			    (1 - start_speed * acc / 2) /
			    (1 - acc / 2);

			finacc_position = (start_speed + evaluated_linear_speed) * acc / 2;
		}

		float posmod(float param) override
		{
			if (param < acc)
			{
				//PRINT(param * start_speed +
				//	param * (evaluated_linear_speed - start_speed) / 2);
				return
				    param * (evaluated_linear_speed + start_speed) / 2;
			}

			else
			{
				//PRINT(finacc_position + evaluated_linear_speed * (param - acc));
				return finacc_position + evaluated_linear_speed * (param - acc);
			}
		}

		float spdmod(float param) override
		{
			if (param < acc)
			{
				float k = param / acc;
				return start_speed * (1 - k) + evaluated_linear_speed * k;
			}

			else
			{
				return evaluated_linear_speed;
			}
		}
	};

	class accdcc_speed_deformer : public speed_deformer
	{
		float strt_spd;
		float fini_spd;

		float fini_acc_pos;
		float strt_dcc_pos;
		float real_spd;

		float acc;
		float dcc;

	public:
		accdcc_speed_deformer(float acc, float dcc, float sspd = 0, float fspd = 0)
			: acc(acc), dcc(dcc), strt_spd(sspd), fini_spd(fspd)
		{
			real_spd =
			    (1 - strt_spd * acc / 2 - fini_spd * dcc / 2) /
			    (1 - acc / 2 - dcc / 2);

			fini_acc_pos = (strt_spd + real_spd) * acc / 2;
			strt_dcc_pos = fini_acc_pos + real_spd * (1-acc-dcc);
		}

		float posmod(float param) override
		{
			if (param < acc)
			{
				//return
				//    param * (real_spd + strt_spd) / 2;
				return 
					param * strt_spd 
					+ param * (param / acc * (real_spd - strt_spd) / 2);
			}

			if (param < 1 - dcc)
			{
				return
				    fini_acc_pos 
				    + real_spd * (param - acc);
			}

			else
			{
				auto locparam = param - 1 + dcc;
				return strt_dcc_pos 
				+ (locparam) * real_spd
				- (locparam) * ((locparam) / dcc * (real_spd - fini_spd)) / 2;
			}
		}

		float spdmod(float param) override
		{
			if (param < acc)
			{
				float k = param / acc;
				return strt_spd * (1 - k) + real_spd * k;
			}

			if (param < 1 - dcc)
			{
				return real_spd;
			}

			else
			{
				float k = (1-param) / dcc;
				return fini_spd * (1 - k) + real_spd * k;
			}
		}

	};
}

#endif