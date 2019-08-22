#ifndef RALGO_CYNEMATIC_CHAIN2D_H
#define RALGO_CYNEMATIC_CHAIN2D_H

#include <ralgo/cynematic/link2d.h>

#include <igris/container/array_view.h>

namespace ralgo 
{
	class cynematic_chain2d 
	{
		igris::array_view<unit2d> chain;
		igris::array_view<cynematic_unit2d> pairs;

	public:
		cynematic_chain2d(
			igris::array_view<unit2d> chain
			igris::array_view<cynematic_unit2d> pairs) 
		: 
			chain(chain),
			pairs(pairs) 
		{}

		void update_location() 
		{
			for (auto& c : chain)
			{
				c.update_location();
			}
		}

		void sensivity(rabbit::screw2<float>* senses, unit2d basis) 
		{
			htrans2 trsf();		
			auto outtrans = chain[chain.size()-1]->global_location;	

			for (int i = 0; i < pairs.size(); ++i) 
			{
				rabbit::screw2<float> lsens = pairs[i]->sensivity();
				auto linktrans = pairs[i]->global_location;

				trsf = linktrans.inverse() * outtrans;

				auto radius = trsf.translation();

				auto wsens = lsens.rotation();
				auto vsens = linalg::cross(wsens, radius) + lsens.translation();

				senses[i] = 
				{
					wsens - trsf.rotation(),
					trsf.inverse_rotate(vsens)
				}
			}

			if (basis != nullptr) 
			{
				BUG();
			}
		}


/*					"""Вернуть массив тензоров производных положения выходного
		звена по вектору координат в виде [(w_i, v_i) ...]"""

		trsf = pyservoce.nulltrans()
		senses = []

		outtrans = self.chain[0].global_location

		"""Два разных алгоритма получения масива тензоров чувствительности.
		Первый - проход по цепи с аккумулированием тензора трансформации.
		Второй - по глобальным объектам трансформации

		Возможно следует использовать второй и сразу же перегонять в btrsf вместо outtrans"""

			for link in self.cynematic_pairs:
				lsenses = link.senses()
				
				linktrans = link.output.global_location
				trsf = linktrans.inverse() * outtrans
			
				radius = trsf.translation()
			
				for sens in reversed(lsenses):
					
					wsens = sens[0]
					vsens = wsens.cross(radius) + sens[1]
				
					itrsf = trsf.inverse()
				
					senses.append((
						itrsf(wsens), 
						itrsf(vsens)
					))

		"""Для удобства интерпретации удобно перегнать выход в интуитивный базис."""
		if basis is not None:
			btrsf = basis.global_location
			#trsf =  btrsf * outtrans.inverse()
			#trsf =  outtrans * btrsf.inverse() #ok
			trsf =  btrsf.inverse() * outtrans #ok
			#trsf =  outtrans.inverse() * btrsf
			#trsf =  trsf.inverse()

			senses = [ (trsf(w), trsf(v)) for w, v in senses ]

		return list(reversed(senses))
*/	
	};
}

#endif