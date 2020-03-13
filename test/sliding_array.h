#include <ralgo/matstruct/sliding_array.h>
#include <ralgo/vecops.h>

LT_BEGIN_TEST(ralgo_test_suite, sliding_array)
{
	ralgo::sliding_array<double> arr(5);

	igris::array_view<double> check;
	igris::array_view<double> window = arr.window();

	check = {0,0,0,0,0};
	LT_CHECK(ralgo::vecops::equal(window, check));
}
LT_END_TEST(sliding_array)



