#ifndef RALGO_HEIMER_COORDINATE_CHECKER_H
#define RALGO_HEIMER_COORDINATE_CHECKER_H

#include <linalg/linalg.h>
#include <ralgo/geom/zone_check.h>
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
			igris::array_view<linalg::vec<P,2>> polygon;

			plane_zone_checker(igris::array_view<linalg::vec<P,2>> arr) :
				polygon(arr.data(), arr.size()/2) 
			{}
				
			bool is_valid(igris::array_view<P> pnt) 
			{
				linalg::vec<P,2> t(pnt[0], pnt[1]);
				return point2_in_polygon(polygon, t);
			}	
		};
	}
}

#endif