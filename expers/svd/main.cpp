#include <ralgo/util/backpack.h>
#include <rabbit/space/screw.h>

int main()
{	
	rabbit::screw2<double> arr[3] = 
	{
		{1, {4,2}},
		{9, {1,5}},
		{66, {8,5}}
	};

	rabbit::screw2<double> t{66, {8,5}};

	double coords[3];

	ralgo::svd_backpack(coords, t, arr, 3);

	for (auto a : coords)
		nos::println(a);
}
