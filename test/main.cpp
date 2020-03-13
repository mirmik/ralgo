#include "littletest.hpp"

LT_BEGIN_SUITE(ralgo_test_suite)
	void set_up()
	{}

	void tear_down()
	{}
LT_END_SUITE(ralgo_test_suite)

#include "sliding_array.h"

LT_BEGIN_AUTO_TEST_ENV()
    AUTORUN_TESTS()
LT_END_AUTO_TEST_ENV()