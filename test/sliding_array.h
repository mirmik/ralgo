#include <ralgo/signal/sliding_array.h>
#include <ralgo/linalg/vecops.h>

using namespace ralgo;

LT_BEGIN_TEST(ralgo_test_suite, sliding_array)
{
	ralgo::sliding_array<double> arr(4);
	igris::array_view<double> check;

	LT_CHECK(vecops::equal_all(arr.window(), dvec{0,0,0,0}));
	LT_CHECK_EQ(arr.cursor,0);

	arr.push(1);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{0,0,0,1}));
	LT_CHECK_EQ(arr.cursor,1);

	arr.push(2);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{0,0,1,2}));
	LT_CHECK_EQ(arr.cursor,2);

	arr.push(3);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{0,1,2,3}));
	LT_CHECK_EQ(arr.cursor,3);

	arr.push(4);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{1,2,3,4}));
	LT_CHECK_EQ(arr.cursor,0);

	arr.push(5);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{2,3,4,5}));
	LT_CHECK_EQ(arr.cursor,1);

	arr.push(6);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{3,4,5,6}));
	LT_CHECK_EQ(arr.cursor,2);

	arr.push(7);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{4,5,6,7}));
	LT_CHECK_EQ(arr.cursor,3);

	arr.push(8);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{5,6,7,8}));
	LT_CHECK_EQ(arr.cursor,0);

	arr.push(9);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{6,7,8,9}));
	LT_CHECK_EQ(arr.cursor,1);

	arr.push(10);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{7,8,9,10}));
	LT_CHECK_EQ(arr.cursor,2);

	double inarr0[] = {2,3,4};
	arr.push(inarr0, 3);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{10,2,3,4}));
	LT_CHECK_EQ(arr.cursor,1);

	double inarr1[] = {5,6};
	arr.push(inarr1, 2);
	LT_CHECK(vecops::equal_all(arr.window(), dvec{3,4,5,6}));
	LT_CHECK_EQ(arr.cursor,3);

}
LT_END_TEST(sliding_array)



