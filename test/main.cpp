#include "littletest.hpp"

#include <vector>

using dvec = std::vector<double>;
using dvec2 = std::vector<std::vector<double>>;
using bvec = std::vector<bool>;

LT_BEGIN_SUITE(ralgo_test_suite)
	void set_up()
	{}

	void tear_down()
	{}
LT_END_SUITE(ralgo_test_suite)

#include "sliding_array.h"
#include "vecops.h"
#include "matops.hpp"
#include "backpack.h"

LT_BEGIN_AUTO_TEST_ENV()
    AUTORUN_TESTS()
LT_END_AUTO_TEST_ENV()