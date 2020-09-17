#ifndef RALGO_HEIMER_COORDINATE_CHECKER_H
#define RALGO_HEIMER_COORDINATE_CHECKER_H

#include <linalg/linalg.h>
#include <ralgo/geom/zone_check.h>
#include <igris/container/array_view.h>

namespace heimer
{
	template <class P>
	class coordinate_checker 
	{
	public:
		virtual bool check(control_node * dev, P * val, int dim, char * msgbuf) = 0;
		virtual int command(int argc, char ** argv) = 0;
	};

	template <class P>
	class plane_zone_checker : public coordinate_checker<P>
	{
	public:
		linalg::vec<P, 2> * arr;		
		int arrcap;
		int arrsize;

		plane_zone_checker(igris::array_view<linalg::vec<P, 2>> arr)
		{
			this->arr = arr.data();
			this->arrcap = arr.size();
			this->arrsize = 0;
		}

		bool check(control_node * dev, P * val, int dim, char* msgbuf) override
		{
			assert(dim == 2);

			linalg::vec<P, 2> t(val[0], val[1]);
			bool in = ralgo::point2_in_polygon(
				igris::array_view<linalg::vec<P, 2>>
					{arr, (size_t)arrsize}, t);

			if (in) 
			{
				return 0;
			}

			else 
			{
				strcpy(msgbuf, "target is not in permitted polygon");
				return -1;
			}
		}

		int command(int argc, char ** argv) override 
		{
			if (argc == 0) 
			{
				nos::println("table cappasity:", arrcap);
				nos::println("table size:", arrsize);

				for (int i = 0; i < arrsize; ++i) 
				{
					nos::println(i, ":", arr[i]);
				}

				return 0;
			}

			if (strcmp(argv[0], "clean") == 0) 
			{
				arrsize = 0;
				return 0;
			}

			if (strcmp(argv[0], "add") == 0)
			{
				if (argc != 3) 
				{
					nos::println("wrong pnt length");
				}

				if (arrsize == arrcap) {
					nos::println("table is full");
					return -1;
				}

				float a = atof32(argv[1], nullptr);
				float b = atof32(argv[2], nullptr);

				arr[arrsize++] = {a,b};
				return 0;
			}

			return -1;
		}
	};
}

#endif