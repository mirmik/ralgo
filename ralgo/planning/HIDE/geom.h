#ifndef RALGO_GEOM_H
#define RALGO_GEOM_H

#include <malgo/vector.h>

namespace ralgo 
{
	struct line_traj 
	{
		malgo::vector<float> x0;
		malgo::vector<float> x1;
		malgo::vector<float> diff;
		float tdist;

		line_traj(malgo::vector<float>&& x0, malgo::vector<float>&& x1) 
			: x0(x0), x1(x1) 
		{
			diff = x1 - x0;
			tdist = length(diff);
			for (auto& r: diff) r /= tdist;
		}

		malgo::vector<float> d0(float t) 
		{
			return x0 + diff * t;
		}

		malgo::vector<float> tang(float t) 
		{
			return diff;
		}
	};
}

#endif