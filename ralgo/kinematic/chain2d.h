#ifndef RALGO_kinematic_CHAIN2D_H
#define RALGO_kinematic_CHAIN2D_H

#include <ralgo/kinematic/link2d.h>
#include <igris/container/array_view.h>

namespace ralgo 
{
	class kinematic_chain2d 
	{
	public:
		igris::array_view<unit2d*> chain;
		igris::array_view<kinematic_unit2d*> pairs;

	public:
		kinematic_chain2d(){};

		unit2d* out() { return chain[chain.size() - 1]; }

		kinematic_chain2d(
			igris::array_view<unit2d*> chain,
			igris::array_view<kinematic_unit2d*> pairs) 
		: 
			chain(chain),
			pairs(pairs) 
		{}

		void setup(
			igris::array_view<unit2d*> chain,
			igris::array_view<kinematic_unit2d*> pairs) 
		{
			this->chain = chain;
			this->pairs = pairs; 
		}

		/*void read_coords() 
		{
			for (auto& c : pairs)
			{
				c->read_coords();
			}
		}*/

		void collect_chain(unit2d* finallink, unit2d* startlink = nullptr) 
		{
			unit2d* link = finallink;
			auto cit = pairs.rbegin();
			auto it = chain.rbegin();

			do 
			{
				*it-- = link;
				if (link->iscynem())
				{
					*cit-- = (kinematic_unit2d*)link;
				}
				link = link->parent;
			} 
			while(link != startlink);
		}

		void update_location() 
		{
			for (auto& c : chain)
			{
				c->update_location();
			}
		}

		void update_model_location() 
		{
		//	read_coords();
			update_location();
		}

		void sensivity(rabbit::screw2<float>* senses, unit2d* basis) 
		{
			rabbit::htrans2<float> trsf{};		
			auto outtrans = chain[chain.size()-1]->global_location;	

			//PRINT(outtrans);

			for (unsigned int i = 0; i < pairs.size(); ++i) 
			{
				rabbit::screw2<float> lsens = pairs[i]->sensivity();

				//PRINT(lsens);

				auto linktrans = pairs[i]->global_location;
				trsf = linktrans.inverse() * outtrans;

				//PRINT(linktrans);
				//PRINT(trsf);

				auto radius = trsf.translation();

				float wsens = lsens.rotation();
				auto vsens = linalg::cross(wsens, radius) + lsens.translation();

				auto itrsf = trsf.inverse();

				senses[i] = 
				{
					wsens,
					itrsf.rotate(vsens)
				};
			}

			if (basis != nullptr) 
			{
				rabbit::htrans2<float> btrsf = basis->global_location;
				rabbit::htrans2<float> ctrsf = btrsf.inverse() * outtrans;

				for (unsigned int i = 0; i < pairs.size(); ++i) 
				{
					senses[i].lin = ctrsf.rotate(senses[i].lin);
//					PRINT(senses[i]);
				}
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

			for link in self.kinematic_pairs:
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