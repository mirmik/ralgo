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
	template < class P=int64_t, class V=float, class A=float >
	struct speed_deformer
	{
		virtual float spdmod(float param) = 0;
		virtual float posmod(float param) = 0;
	};

	template < class P=int64_t, class V=float, class A=float >
	struct acc_speed_deformer : public speed_deformer<P,V,A>
	{
		float start_speed;
		float finacc_position;
		float evaluated_linear_speed;

		//float lin;
		float acc;

		acc_speed_deformer(float acc, float sspd = 0) 
			: acc(acc), start_speed(sspd)
		{}

		V posmod(float param) 
		{
			if (param < acc) 
			{
				return 
					param * start_speed + 
					param * (evaluated_linear_speed - start_speed) / 2;
			}

			else 
			{
				return finacc_position + evaluated_linear_speed * (param - acc);
			}
		}

		V spdmod(float param) 
		{
			if (param < acc) 
			{
				float k = param / acc;
				return start_speed * (1-k) + evaluated_linear_speed * k;
			}

			else 
			{
				return evaluated_linear_speed;
			}
		}
	};
}

#endif