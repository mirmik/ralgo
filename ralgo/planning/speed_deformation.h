#ifndef RALGO_SPEED_DEFORMATION_H
#define RALGO_SPEED_DEFORMATION_H

namespace ralgo 
{
	struct speed_deformation 
	{
		virtual float spdmod(float param) = 0;
		virtual float posmod(float param) = 0;
	};

	template < class P, class V, class A >
	struct acc_speed_deformation
	{
		float start_speed;
		//float tasked_linear_speed;
		float evaluated_linear_speed;

		float lin;
		float acc;

		P posmod(float param) 
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
				return std::lerp(start_speed, evaluated_linear_speed, param / acc);
			}

			else 
			{
				return evaluated_linear_speed;
			}
		}
	};
}

#endif