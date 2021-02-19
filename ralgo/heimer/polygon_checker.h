#ifndef RALGO_HEIMER_POLYGON_CHECKER_H
#define RALGO_HEIMER_POLYGON_CHECKER_H

namespace heimer
{
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

		bool check_impl(
		    control_node * dev,
		    P * val,
		    int dim,
		    char* msgbuf) override
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

		int add(linalg::vec<P, 2> pnt)
		{
			if (arrsize == arrcap)
				return -1;

			arr[arrsize++] = pnt;
			return 0;
		}

		int clean()
		{
			arrsize = 0;
			return 0;
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
				return clean();
			}

			if (strcmp(argv[0], "add") == 0)
			{
				if (argc != 3)
				{
					nos::println("wrong pnt length");
				}

				if (arrsize == arrcap)
				{
					nos::println("table is full");
					return -1;
				}

				float a = atof32(argv[1], nullptr);
				float b = atof32(argv[2], nullptr);

				return add({a, b});
			}

			return -1;
		}
	};

	template <class P, class V>
	class polygon_difference_checker : public plane_zone_checker<P>
	{
	private:
		using parent = plane_zone_checker<P>;

		heimer::linintctr_basic<P, V> * positive;
		heimer::linintctr_basic<P, V> * negative;

	public:
		polygon_difference_checker(
		    heimer::linintctr_basic<P, V> * positive,
		    heimer::linintctr_basic<P, V> * negative,
		    igris::array_view<linalg::vec<P, 2>> arr)
			:
			plane_zone_checker<P>(arr),
			positive(positive),
			negative(negative)
		{
			assert(positive != negative);
		}

		bool check_impl(
		    control_node * dev,
		    P * val,
		    int dim,
		    char * msgbuf) override
		{
			assert(positive == dev || negative == dev);
			assert(dim == 2);

			linalg::vec<P, 2> t(val[0], val[1]);
			linalg::vec<P, 2> pos_point;
			linalg::vec<P, 2> neg_point;

			positive->current_point((P*) &pos_point);
			negative->current_point((P*) &neg_point);

			if (positive == dev)
			{
				t -= neg_point;
			}
			else
			if (negative == dev) 
			{
				t -= pos_point;
			}

			//auto pnt = pos_point - neg_point;

			bool in_polygon = ralgo::point2_in_polygon(
			                      igris::array_view<linalg::vec<P, 2>>
			{parent::arr, (size_t)parent::arrsize}, t);

			if (in_polygon)
			{
				return 0;
			}

			else
			{
				strcpy(msgbuf, "target is not in permitted difference polygon");
				return -1;
			}
		}
	};
}

#endif