/** @file */

#ifndef RALGO_HEIMER_COORDINATE_CHECKER_H
#define RALGO_HEIMER_COORDINATE_CHECKER_H

#include <cmath>
#include <cstdlib>

#include <ralgo/linalg/linalg.h>
#include <ralgo/geom/zone_check.h>
#include <ralgo/heimer/interpolation_group.h>
#include <igris/container/array_view.h>

namespace heimer
{
	template <class P>
	class coordinate_checker 
	{
		coordinate_checker * next = nullptr;

	public:
		bool check(control_node * dev, P * val, int dim, char * msgbuf) 
		{
			if (check_impl(dev, val, dim, msgbuf)) 
			{
				return true;
			}

			if (next) 
			{
				return next->check(dev, val, dim, msgbuf);
			}

			return false;
		}

		virtual bool check_impl(control_node * dev, P * val, int dim, char * msgbuf) = 0;
		virtual int command(int argc, char ** argv) { return -1; }

		void link_next(coordinate_checker * next) { this->next = next; }
	};
}

#endif
