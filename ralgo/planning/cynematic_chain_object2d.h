#ifndef RALGO_CYNEMATIC_CHAIN_OBJECT2D_H
#define RALGO_CYNEMATIC_CHAIN_OBJECT2D_H

#include <ralgo/cynematic/chain2d.h>

namespace ralgo 
{
	class cynematic_chain2d_output_controller 
	{
		ralgo::cynematic_chain2d chain;
		float compensation_koefficient;

		cynematic_chain2d_output_controller() 
		{
			compensation_koefficient = 10 / ralgo::discrete_frequency();
		}

		htrans2 location() 
		{
			return chain->out()->global_location;
		}

		void update_model_location() 
		{
			chain.update_model_location();
		}

		void set_phase(rabbit::htrans2<float> pos, rabbit::screw2<float> spd) 
		{
			rabbit::screw2<float> senses[chain.pairs.size()];
			float spdarr[chain.pairs.size()];
			memset(spdarr, 0, sizeof(float) * chain.pairs.size());

			chain.sensivity(&senses);

			rabbit::screw2<float> target = 
				spd + (pos - location) * compensation_koefficient; 	

			// Поиск скоростей звеньев удовлетворяющих заданному
			// управлению.
			ralgo::gradient_backpack(spdarr, target, 
				senses, chain.pairs.size());

			// Выставляем найденные скорости прилинкованным 
			// сервам.
			for (int i = 0; i < chain.pairs.size(); ++i) 
			{
				cynematic_unit2d * _unit = chain.pairs[i];
				cynematic_unit2d_1dof * unit = (cynematic_unit2d_1dof *) _unit;

				unit -> set_speed_for_linked(spdarr[i]);
			}
		}
	};
}

#endif