#include <doctest/doctest.h>
#include <ralgo/linalg/matrix.h>
#include <ralgo/util/helpers.h>

TEST_CASE("test") 
{
	CHECK_EQ(ralgo::is_matrix_compatible<ralgo::matrix<double>>(), 1);
	CHECK_EQ(ralgo::is_matrix_compatible<std::vector<double>>(), 0);
	CHECK_EQ(ralgo::is_matrix_compatible<int>(), 0);

	CHECK_EQ(ralgo::is_vector_compatible<ralgo::matrix<double>>(), 0);
	CHECK_EQ(ralgo::is_vector_compatible<std::vector<double>>(), 1);
	CHECK_EQ(ralgo::is_vector_compatible<int>(), 0);
}