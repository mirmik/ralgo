#ifndef RALGO_HEIMER_COORDINATE_CHECKER_H
#define RALGO_HEIMER_COORDINATE_CHECKER_H

#include <linalg/linalg.h>
#include <igris/container/array_view.h>

namespace ralgo
{
	namespace heimer
	{
		template <class P>
		class interpolation_coordinate_checker
		{
			virtual bool is_valid(igris::array_view<P> arr) = 0;
		};

		template <class P>
		class plane_zone_checker : public interpolation_coordinate_checker<P>
		{
		public:
			igris::array_view<P> polysegment;			

			bool is_valid(igris::array_view<P> arr) 
			{
				assert(arr.size() == 2);

				int lastsign = 0;
				for (int i = 0; i < polysegment.size(); i += 2) 
				{
					linalg::vec<P,2> a = { polysegment[i], polysegment[i+1] }; 
					linalg::vec<P,2> b = { polysegment[i+1], polysegment[i+2] };

					auto ba = b - a;

					linalg::vec<P,2> t = { arr[0], arr[1] };
					auto ta = t - a;

					int sign = linalg::cross(ba, ta) >= 0 ? +1 : -1;
					if (lastsign != 0 && lastsign!=sign)
						return false;

					lastsign = sign;
				}
				return true;
			}	
		};
	}
}

#endif